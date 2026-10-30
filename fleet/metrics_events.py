"""Structured metrics-event emitter for fleetSupervisor.py.

Streams the Page-4 metrics event contract (schema_version 1) to the Flask
backend on the Windows laptop over plain HTTP -- POST
{base}/api/runs/{run_id}/events/batch -- from a background daemon thread
with a bounded queue, so NOTHING here can ever block or crash a ROS
callback or the dispatch loop:

  * emit() only enqueues (put_nowait; overflow drops the event and counts
    it). All HTTP happens on the worker thread.
  * Every public method swallows every exception.
  * Disabled entirely (all no-ops) until configure_from_mission() sees a
    mission carrying metrics_run_id -- missions dispatched without the
    metrics backend registered behave exactly as before.

Configuration precedence for the backend URL:
  1. METRICS_URL environment variable (set on the Linux laptop),
  2. the mission payload's metrics_url (filled in by the HTML page),
Neither present -> emitter stays disabled even if a run_id arrived.

Timekeeping (spec): elapsed_ms is MONOTONIC time since RUN_STARTED (or
since configuration until a RUN_STARTED is emitted, which re-anchors t0);
time_utc is a wall timestamp for display only, never used for duration
math.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
import urllib.request
from datetime import datetime, timezone

SCHEMA_VERSION = 1
BATCH_MAX = 200            # events per POST
FLUSH_INTERVAL_SEC = 1.0   # worker wakes at least this often
QUEUE_MAX = 10000          # bounded; overflow drops (counted) rather than blocks
HTTP_TIMEOUT_SEC = 3.0
WARN_EVERY_FAILURES = 20   # print one warning per this many consecutive fails

EVENT_FIELDS = (
    "robot_id", "job_id", "workstation_id", "conflict_id", "command_id",
    "state_from", "state_to", "reason", "x_m", "y_m", "yaw_deg",
    "battery_pct", "pose_age_ms",
)

IN_TO_M = 0.0254


class MetricsEmitter:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._queue: queue.Queue = queue.Queue(maxsize=QUEUE_MAX)
        self._thread: threading.Thread | None = None
        self._base_url: str | None = None
        self._run_id: str | None = None
        #: The navigator Namespace, handed over by the supervisor when it
        #: builds its workers. Held so push_tuning() can record the 47 motion
        #: parameters a run actually executed under -- including any env or
        #: CLI override, which a snapshot of module defaults would miss.
        self._nav_args = None
        self._seq = 0
        self._t0 = time.monotonic()
        self._dropped = 0
        self._consecutive_failures = 0
        # mission-derived context used by travel_state()
        self._grid_nodes: int | None = None
        self._has_picks = False

    # -- configuration ------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._base_url is not None and self._run_id is not None

    def configure_from_mission(self, mission: dict) -> None:
        try:
            run_id = mission.get("metrics_run_id")
            url = os.environ.get("METRICS_URL") or mission.get("metrics_url")
            if not run_id or not url:
                self._run_id = None  # explicit: this mission is untracked
                return
            with self._lock:
                self._run_id = str(run_id)
                self._base_url = str(url).rstrip("/")
                self._seq = 0
                self._t0 = time.monotonic()
            schedule = mission.get("schedule") or {}
            grid = schedule.get("grid") or {}
            try:
                self._grid_nodes = int(grid.get("rows", 0)) * int(grid.get("cols", 0)) or None
            except (TypeError, ValueError):
                self._grid_nodes = None
            self._has_picks = any(
                (v or {}).get("mode") in ("pick", "service")
                for route in (schedule.get("routes") or [])
                for v in (route.get("visits") or []))
            self._ensure_thread()
            print(f"[metrics] streaming events for run {self._run_id} "
                  f"to {self._base_url}")
            self.push_tuning()
        except Exception:
            pass

    def remember_nav_args(self, nav_args) -> None:
        """Called by the supervisor as it constructs each navigator.

        TWO BUGS FIXED 2026-10-21. The configure-time push_tuning() runs
        before any navigator exists, so its snapshot had no navigator args
        and recorded effective_turn_method "unknown" -- every run since
        2026-10-15. And the args were kept only the FIRST time, so in listen
        mode every later mission reported the first mission's turn method
        even after the Drive mode buttons changed it. Now the newest args
        always win, and the snapshot is re-sent once per run as soon as the
        first navigator of that run is built (the PATCH replaces the earlier
        one)."""
        self._nav_args = nav_args
        if (self.enabled and self._run_id
                and getattr(self, "_tuned_run", None) != self._run_id):
            self._tuned_run = self._run_id
            self.push_tuning(nav_args)

    def push_tuning(self, nav_args=None) -> None:
        """Record the control tuning this run executes under.

        Sent once, at mission configure, as a PATCH on the run row. The
        browser creates the run and cannot know these values -- they live in
        the supervisor and navigator modules on this machine.

        Fire-and-forget on a daemon thread: a metrics backend that is down
        must never delay a dispatch, and a run with no tuning recorded is a
        gap in the analysis, not a reason to refuse to drive.
        """
        if not self.enabled:
            return

        args = nav_args if nav_args is not None else self._nav_args

        def _send():
            try:
                import tuning_snapshot
                snap = tuning_snapshot.collect(args)
                url = f"{self._base_url}/api/runs/{self._run_id}"
                body = json.dumps({"tuning_json": snap}).encode("utf-8")
                req = urllib.request.Request(
                    url, data=body, method="PATCH",
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_SEC):
                    pass
                print(f"[metrics] tuning captured: {tuning_snapshot.summary(snap)}")
            except Exception as exc:
                print(f"[metrics] tuning capture failed ({exc!r}) -- the run "
                      f"will record no configuration")

        threading.Thread(target=_send, name="metrics-tuning",
                         daemon=True).start()

    # -- helpers for callers -------------------------------------------------

    def travel_state(self, ahead_labels) -> str | None:
        """Classify a move dispatch into the mutually-exclusive state model,
        only when it can be done TRUTHFULLY:
          * no workstation/entry node left ahead -> RETURN_REPOSITION
          * pure-delivery mission (no picks)     -> PRODUCTIVE_TRAVEL_LOADED
          * RPD mission (loaded/empty unknown)   -> None (engine counts the
            time as UNCLASSIFIED travel; loaded/empty-split metrics report
            missing data instead of a fabricated split)."""
        try:
            if self._grid_nodes:
                has_ws_ahead = any(
                    str(lbl).isdigit() and int(lbl) > self._grid_nodes
                    for lbl in ahead_labels)
                if not has_ws_ahead:
                    return "RETURN_REPOSITION"
            if not self._has_picks:
                return "PRODUCTIVE_TRAVEL_LOADED"
            return None
        except Exception:
            return None

    # -- emission ------------------------------------------------------------

    def emit(self, event_type: str, details: dict | None = None,
             **fields) -> None:
        if not self.enabled:
            return
        try:
            with self._lock:
                if event_type == "RUN_STARTED":
                    self._t0 = time.monotonic()
                self._seq += 1
                seq = self._seq
            ev = {
                "schema_version": SCHEMA_VERSION,
                "seq": seq,
                "time_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_ms": int((time.monotonic() - self._t0) * 1000),
                "event_type": str(event_type),
            }
            for key in EVENT_FIELDS:
                if key in fields and fields[key] is not None:
                    ev[key] = fields[key]
            if details:
                ev["details"] = details
            try:
                self._queue.put_nowait(ev)
            except queue.Full:
                self._dropped += 1
        except Exception:
            pass

    def emit_depot_arrived_in(self, robot_id: str, x_in=None, y_in=None,
                              yaw_deg=None) -> None:
        """DEPOT_ARRIVED carrying the terminal pose.

        The ingest side has always written run_robots.terminal_x_m/y_m/
        yaw_deg from this event's x_m/y_m/yaw_deg fields, but the event was
        emitted with only robot_id and elapsed_ms, so all three columns were
        NULL on every row ever recorded (145 of 145 as of 2026-09-22) and the
        Robots tab's "final yaw" had nothing to display. Same inch-to-meter
        convention as emit_pose_in.
        """
        try:
            self.emit("DEPOT_ARRIVED", robot_id=robot_id,
                      x_m=None if x_in is None else float(x_in) * IN_TO_M,
                      y_m=None if y_in is None else float(y_in) * IN_TO_M,
                      yaw_deg=None if yaw_deg is None else float(yaw_deg))
        except Exception:
            pass

    def emit_pose_in(self, robot_id: str, x_in, y_in, yaw_deg=None,
                     battery_pct=None, pose_age_ms=None) -> None:
        """POSE_SAMPLE from testbed-inch coordinates (stored in meters)."""
        try:
            self.emit("POSE_SAMPLE", robot_id=robot_id,
                      x_m=None if x_in is None else float(x_in) * IN_TO_M,
                      y_m=None if y_in is None else float(y_in) * IN_TO_M,
                      yaw_deg=yaw_deg, battery_pct=battery_pct,
                      pose_age_ms=pose_age_ms)
        except Exception:
            pass

    # -- worker --------------------------------------------------------------

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._worker, daemon=True,
                                        name="metrics-emitter")
        self._thread.start()

    def _worker(self) -> None:
        while True:
            batch = []
            try:
                batch.append(self._queue.get(timeout=FLUSH_INTERVAL_SEC))
                while len(batch) < BATCH_MAX:
                    batch.append(self._queue.get_nowait())
            except queue.Empty:
                pass
            if not batch:
                continue
            self._post(batch)

    def _post(self, batch: list) -> None:
        url = f"{self._base_url}/api/runs/{self._run_id}/events/batch"
        body = json.dumps({"events": batch}).encode("utf-8")
        req = urllib.request.Request(
            url, data=body, method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_SEC):
                pass
            self._consecutive_failures = 0
        except Exception as exc:
            self._consecutive_failures += 1
            if self._consecutive_failures % WARN_EVERY_FAILURES == 1:
                print(f"[metrics] delivery failing ({exc!r}) -- events are "
                      f"dropped, robot control is unaffected "
                      f"(failure #{self._consecutive_failures})")


# module-level singleton importers use
metrics_emitter = MetricsEmitter()
