#!/usr/bin/env python3
"""
fleet_yaw_stress_test.py -- runs camera_grid_navigate.py's yaw-source
stress test (see CameraGridNavigator.yaw_stress_test_route()) on MULTIPLE
robots SIMULTANEOUSLY, counterbalanced across a 6x3 Latin square so mode
order isn't confounded with battery/heat/experiment-order effects, then a
4th "encoder_drift" stage for everyone.

Built 2026-08-20, rewritten same day after a real 6-robot hardware run
(see fleet_yaw_stress_run_2026-08-18.log) hit "no vision at stress-test
start" on 5/6 robots simultaneously going into stage 4. ROOT CAUSE
CONFIRMED from the actual timestamps: the original design used
threading.Barrier, a bare blocking call that does NOT spin a robot's ROS
executor while parked in it. Any robot that finished a stage before the
slowest sibling sat idle with zero _on_vision_pose callbacks processed --
past VISION_STALE_SEC=0.5s (routine; stages commonly spread 1-1.5s across
6 robots) fresh_pose() was stale the INSTANT the barrier released, and the
next stage checks it immediately. Alvik1 (the slowest finisher, zero idle
time) was the only one unaffected -- consistent with this mechanism, not
with a shared-camera-pipeline stall (which would have hit Alvik1 too) or
a hang (no thread was blocked past its timeout, no exception was raised).

FIX: replaced threading.Barrier with a manual stage-gate (counter +
condition variable) where each waiting robot's thread periodically spins
its OWN executor (processing incoming pose/status messages) between
checks instead of blocking blind -- vision never goes stale while
waiting, by construction, rather than being allowed to go stale and then
re-freshened after the fact.

COUNTERBALANCING (added 2026-08-20, per explicit user direction after
reviewing this failure with a second AI's code review): each robot runs
the three comparable modes (encoder / camera_assist / camera_only) in a
DIFFERENT order, per a 6x3 Latin square (LATIN_SQUARE below) -- so
"stage 1" is not the same mode for every robot, controlling for
battery/motor-heat/fatigue effects being confounded with which mode ran
first across the whole fleet. The barrier/stage-gate still syncs on STAGE
INDEX (1/2/3/4), not mode name: all 6 robots finish their own stage-1
mode before any starts stage 2, etc. Stage 4 is "encoder_drift" for
everyone (same mode, no counterbalancing needed there since it's not an
accuracy comparison) -- inherits whatever onboard yaw state each robot's
own stage-3 mode left, which differs by robot since stage 3 isn't the
same mode for everyone; that's expected and documented in the summary.

HARDENING added in the same rewrite, per explicit user direction:
  - try/except around each stage: a real exception is caught, logged with
    full traceback, marks that robot's remaining stages as failed, and
    does NOT kill other robots' threads or the whole run.
  - Incremental CSV writes: every rotation's result is flushed to its
    robot's file the instant it completes (via yaw_stress_test_route()'s
    new on_result callback), not just at stage-end -- a crash/hang
    partway through a 128-rotation stage no longer loses the rotations
    that already succeeded.
  - A watchdog thread tracks each robot's last-progress timestamp
    (updated on every rotation result AND every stage-gate spin tick) and
    force-STOPs + excludes a robot that genuinely stalls (not just one
    waiting normally at the gate) -- see WATCHDOG_TIMEOUT_SEC.
  - A 5-second fixed pause after ALL 6 robots reach a stage gate, before
    release into the next stage -- explicit warm-up window, not an
    instant release, so nothing starts moving again mid-settle.

OUTPUT (per explicit user direction -- no terminal-scraping required):
  <outdir>/<Robot>_stage<N>_<mode>.csv   one row per rotation, written
                                         incrementally as each rotation
                                         completes (see RESULT_FIELDS)
  <outdir>/SUMMARY.md                    scope/goal, per-robot per-mode
                                         stats, PER-ROBOT mode ranking +
                                         a sign test across robots (the
                                         fleet-level test statistic -- see
                                         "Statistics" note below for why
                                         this replaced a pooled t-test),
                                         and the encoder_drift trend

STATISTICS NOTE: a naive pooled paired t-test across all 768 rotations
per mode (128 x 6 robots) treats rotations as independent, which they are
not -- they're nested within 6 robots (pseudoreplication). The real
hardware-level sample size for "does mode X generalize across robots" is
6, not 768. This script reports each robot's own mode ranking (by mean
|final_error|), then a sign test across the 6 robots for each mode pair
(how many robots had X more accurate than Y) as the fleet-level
statistic, alongside the full per-robot/pooled descriptive stats for
reference.

Usage:
    python3 fleet_yaw_stress_test.py --robots Alvik1,Alvik2,Alvik3,Alvik4,Alvik5,Alvik6
    python3 fleet_yaw_stress_test.py --robots Alvik1,Alvik2 --outdir ./my_run
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import threading
import time
import traceback
from datetime import datetime

import rclpy

from camera_grid_navigate import CameraGridNavigator

ACCURACY_MODES = ["encoder", "camera_assist", "camera_only"]
DRIFT_MODE = "encoder_drift"

# 6x3 Latin square: LATIN_SQUARE[robot_index] = [stage1_mode, stage2_mode,
# stage3_mode], each mode appears exactly once per stage column across the
# 6 rows. Exact assignment per explicit user direction 2026-08-20.
LATIN_SQUARE: list[list[str]] = [
    ["encoder", "camera_assist", "camera_only"],   # robot 0 (1st in --robots)
    ["camera_assist", "camera_only", "encoder"],   # robot 1
    ["camera_only", "encoder", "camera_assist"],   # robot 2
    ["encoder", "camera_only", "camera_assist"],   # robot 3
    ["camera_only", "camera_assist", "encoder"],   # robot 4
    ["camera_assist", "encoder", "camera_only"],   # robot 5
]

# Same nav-tuning Namespace as fleetSupervisor.py's VisionLegWorker.__init__
# -- kept identical so this test exercises the SAME turn_to_heading_rotate_
# rel()/taper-law constants a real mission would use, not a different
# untested tuning. See that constructor for the full history/reasoning
# behind each value; not re-derived here.
NAV_ARGS = argparse.Namespace(
    verbose=False, rows=8, cols=8,
    turn_tol_deg=3.0, turn_settle_count=3, turn_min_speed=20.0,
    turn_decel_zone_deg=35.0, turn_creep_speed=4.0,
    turn_scale_deg=60.0, turn_max_speed=35.0,
    turn_rpm=35.0, turn_brake_lead_deg=30.0, turn_creep_rpm=10.0,
    cruise_rpm=60.0, brake_lead_in=1.9,
    kp_yaw=0.6, max_turn_adjust=10.0,
)

RESULT_FIELDS = [
    "leg_label", "mode", "target_heading", "start_yaw", "commanded_rel_deg",
    "final_yaw", "final_error", "corrected", "elapsed_sec",
]

# How long a stage-gate poll sleeps between spin/check cycles while
# waiting for siblings -- short enough that vision (published every
# ~POSE_PERIOD_MS=300ms by the firmware, faster by apriltag_localize.py)
# never has a chance to go stale (VISION_STALE_SEC=0.5s) between spins.
GATE_POLL_SEC = 0.1
# Fixed warm-up pause AFTER all 6 robots reach a gate, before release.
GATE_WARMUP_SEC = 5.0
# If a robot's last_progress_at goes older than this while its thread is
# still alive and NOT waiting at a gate, the watchdog treats it as
# genuinely stalled (not just "still mid-rotation" -- 128 rotations at
# the slowest observed ~3.5s/rotation is well under this) and force-stops
# it. Deliberately larger than a single rotation's own internal timeouts
# (max ~30s for the largest angle) so this only fires on a REAL stall the
# rotation's own timeout logic somehow didn't catch.
WATCHDOG_TIMEOUT_SEC = 60.0
WATCHDOG_POLL_SEC = 2.0


class StageGate:
    """Replaces threading.Barrier: N robots must all call arrive() for a
    given stage index before any of them proceeds past it. Unlike
    Barrier.wait(), waiting robots call back into a spin function every
    GATE_POLL_SEC instead of blocking blind -- this is THE fix for the
    2026-08-18 failure (see module docstring): a robot's ROS executor
    keeps processing incoming vision/status messages the entire time it
    waits, so fresh_pose() never goes stale from sitting idle.

    Fixed GATE_WARMUP_SEC pause is applied by the LAST robot to arrive,
    while every other robot polls until that robot signals release --
    ensures everyone actually starts the next stage together, not staggered
    by however long the warm-up implementation takes per-thread.

    exclude(): lowers the required participant count for a robot that has
    crashed/been watchdog-excluded and will NEVER call arrive_and_wait()
    again -- WITHOUT this, the other still-working robots would wait
    forever for a count that can no longer be reached. Deliberately does
    NOT force an early release on its own (an earlier version of this
    class did exactly that via a naive force_advance() and it was a real
    bug: it cut short every OTHER robot's still-in-progress stage the
    instant one robot crashed, defeating the whole point of the gate --
    caught in review 2026-08-20 before ever running on hardware). A
    crashed/excluded robot's absence only lowers the bar; the remaining
    robots still all have to actually arrive."""

    def __init__(self, n_participants: int):
        self._n = n_participants
        self._lock = threading.Lock()
        self._count = 0
        self._departed = 0  # how many have SEEN the release and left the wait loop
        self._release_at: float | None = None  # monotonic time, set once
        self._arrived_total: int | None = None  # snapshot of _count AT release time
        self._stage_index = 0

    def arrive_and_wait(self, stage_index: int, spin_fn, progress_fn) -> None:
        """spin_fn(): spin this robot's own executor once (processes
        incoming messages, does not block). progress_fn(): called every
        poll tick so the watchdog sees this robot as making progress
        (waiting at a gate is legitimate progress, not a stall).

        REAL BUG found 2026-08-21 on hardware (5/6 robots stuck forever
        after stage 1, only the robot that happened to arrive last ever
        reached stage 2): the ORIGINAL version reset _release_at back to
        None the instant ANY ONE robot's poll loop first observed the
        release, on the theory that "resetting twice is a harmless no-op".
        That reasoning was wrong -- resetting EARLY, while other robots
        are still mid-poll-cycle and haven't yet observed release_at
        themselves, wipes the exact signal they're waiting to see. Their
        next poll tick then reads release_at=None and concludes they were
        NOT released, so they loop forever -- nothing will ever set
        release_at again for a stage_index the gate has already moved past.

        FIRST FIX ATTEMPT (also wrong, caught before a second hardware run):
        comparing _departed against a PER-ROBOT local snapshot of _count
        taken at THIS robot's own arrival time. Every robot except the
        true last-arriver has a smaller snapshot than the eventual total,
        so that comparison goes true after only a few departures, not all
        of them -- same premature-reset bug, just needing more departures
        to trigger instead of one. Fixed for real by snapshotting the
        total ONCE, in the same locked block that sets _release_at (i.e.
        the true final _count, not any one robot's view of it at their
        own arrival), and comparing _departed against THAT shared value."""
        with self._lock:
            if stage_index != self._stage_index:
                raise RuntimeError(
                    f"StageGate.arrive_and_wait called with stage_index="
                    f"{stage_index} but gate is on stage {self._stage_index} "
                    "-- caller bug, stages must be entered in order")
            self._count += 1
            is_last = self._count >= self._n
            if is_last and self._release_at is None:
                self._release_at = time.monotonic() + GATE_WARMUP_SEC
                self._arrived_total = self._count

        while True:
            spin_fn()
            progress_fn()
            with self._lock:
                release_at = self._release_at
            if release_at is not None and time.monotonic() >= release_at:
                break
            time.sleep(GATE_POLL_SEC)

        # Only reset once EVERY robot that arrived for this stage has also
        # departed -- see the docstring above for why resetting on the
        # first (or Nth-but-not-last) departure stranded the rest.
        with self._lock:
            self._departed += 1
            if (self._stage_index == stage_index
                    and self._arrived_total is not None
                    and self._departed >= self._arrived_total):
                self._count = 0
                self._departed = 0
                self._release_at = None
                self._arrived_total = None
                self._stage_index = stage_index + 1

    def exclude(self) -> None:
        """Permanently lowers the required participant count by 1 -- call
        exactly once per crashed/watchdog-excluded robot, from that
        robot's own thread, BEFORE it stops calling arrive_and_wait(). Does
        NOT release the gate early by itself; if the remaining robots are
        already all waiting when this drops the count to match, this
        triggers the release itself.

        Must also set _arrived_total when IT is the thing that triggers
        release (not just arrive_and_wait()) -- otherwise the departing
        robots' reset check (self._arrived_total is not None) never fires,
        and the gate gets stuck on the old stage_index forever, tripping
        arrive_and_wait()'s stage-mismatch RuntimeError on every robot's
        next call. Same class of bug as the arrive_and_wait() fix above,
        just a second call site that sets _release_at and needs the same
        bookkeeping."""
        with self._lock:
            self._n = max(1, self._n - 1)
            if self._count >= self._n and self._release_at is None:
                self._release_at = time.monotonic() + GATE_WARMUP_SEC
                self._arrived_total = self._count


class RobotWorker:
    """One robot's node + thread. Mirrors fleetSupervisor.py's
    VisionLegWorker in spirit (dedicated node/executor per robot, driven
    off its own thread)."""

    def __init__(self, robot: str, modes: list[str], outdir: str, gate: StageGate):
        self.robot = robot
        self.modes = modes  # [stage1_mode, stage2_mode, stage3_mode] for THIS robot
        self.outdir = outdir
        self.gate = gate
        self.nav = CameraGridNavigator(robot, NAV_ARGS)
        # results[mode] -- keyed by MODE NAME, not stage index, since
        # summary code compares by mode across robots regardless of which
        # stage each robot ran it in.
        self.results: dict[str, list[dict]] = {m: [] for m in ACCURACY_MODES + [DRIFT_MODE]}
        self.setup_failed: str | None = None
        self.crashed_stage: str | None = None  # mode name a real exception happened in
        self.crash_traceback: str | None = None
        self.excluded_by_watchdog = False
        self.last_progress_at = time.monotonic()
        self._progress_lock = threading.Lock()
        self._open_files: list = []
        self.thread = threading.Thread(target=self._run, name=f"yaw-stress-{robot}")

    def _touch_progress(self) -> None:
        with self._progress_lock:
            self.last_progress_at = time.monotonic()

    def seconds_since_progress(self) -> float:
        with self._progress_lock:
            return time.monotonic() - self.last_progress_at

    def start(self) -> None:
        self.thread.start()

    def join(self, timeout: float | None = None) -> None:
        self.thread.join(timeout=timeout)

    def _run(self) -> None:
        nav = self.nav
        # Same discovery-wait as fleetSupervisor.py's VisionLegWorker._run()
        # -- a freshly-created node's subscription is not instantly matched
        # to apriltag_localize.py's publisher.
        t0 = time.monotonic()
        matched = False
        while time.monotonic() - t0 < 8.0:
            nav._spin_once(0.1)
            self._touch_progress()
            if nav.vision_pose_sub.get_publisher_count() > 0:
                matched = True
                break
        if not matched:
            self.setup_failed = "vision_pose subscription never matched apriltag_localize.py's publisher"
        elif not nav.wait_for_fresh_vision(timeout_sec=8.0):
            self.setup_failed = "no vision at worker start"

        if self.setup_failed is not None:
            nav.get_logger().error(
                f"{self.robot}: setup failed ({self.setup_failed}) -- still "
                "participating in every stage gate as a no-op (see "
                "StageGate's docstring for why this differs from a "
                "watchdog exclude()) so other robots' required-count isn't "
                "left permanently short")
            for stage_index in range(4):
                self.gate.arrive_and_wait(
                    stage_index, lambda: nav._spin_once(0.0), self._touch_progress)
            return

        all_modes = self.modes + [DRIFT_MODE]
        for stage_index, mode in enumerate(all_modes):
            try:
                nav.get_logger().info(
                    f"{self.robot}: stage {stage_index + 1}/4 -> [{mode}]")
                if mode == DRIFT_MODE:
                    # Deliberately NOT reset/resynced -- inherits whatever
                    # onboard yaw state THIS ROBOT'S OWN stage-3 mode left
                    # (which differs by robot under counterbalancing --
                    # documented in the summary, not a bug).
                    results = nav.yaw_stress_test_route(
                        mode, reset_first=False, resync_after_each=False,
                        on_result=self._make_on_result(mode))
                else:
                    results = nav.yaw_stress_test_route(
                        mode, on_result=self._make_on_result(mode))
                self.results[mode] = results
            except Exception:
                self.crashed_stage = mode
                self.crash_traceback = traceback.format_exc()
                nav.get_logger().error(
                    f"{self.robot}: EXCEPTION during stage {stage_index + 1} "
                    f"[{mode}] -- marking remaining stages failed for this "
                    f"robot, other robots continue:\n{self.crash_traceback}")
                # Still take part in every remaining gate (as a no-op) so
                # siblings' required-count isn't left permanently short --
                # this robot's thread is still alive and running, it just
                # has nothing useful left to contribute.
                for remaining in range(stage_index, 4):
                    self.gate.arrive_and_wait(
                        remaining, lambda: nav._spin_once(0.0), self._touch_progress)
                return

            nav.get_logger().info(
                f"{self.robot}: stage {stage_index + 1}/4 [{mode}] done, "
                f"{len(results)}/128 succeeded, waiting at gate")
            self.gate.arrive_and_wait(
                stage_index, lambda: nav._spin_once(GATE_POLL_SEC), self._touch_progress)

    def _make_on_result(self, mode: str):
        stage_num = (self.modes + [DRIFT_MODE]).index(mode) + 1
        path = os.path.join(self.outdir, f"{self.robot}_stage{stage_num}_{mode}.csv")
        is_new = not os.path.exists(path)
        f = open(path, "a", newline="")
        self._open_files.append(f)  # closed in shutdown(), see that method
        writer = csv.DictWriter(f, fieldnames=RESULT_FIELDS)
        if is_new:
            writer.writeheader()
            f.flush()

        def on_result(result: dict) -> None:
            try:
                writer.writerow({k: result.get(k) for k in RESULT_FIELDS})
                f.flush()
            except ValueError:
                # File already closed -- only possible if shutdown() ran
                # while this thread was still mid-stage (e.g. Ctrl+C
                # racing join()). Nothing useful to do; the row is lost
                # but every prior row for this stage is already on disk.
                pass
            self._touch_progress()

        return on_result

    def force_stop(self, reason: str) -> None:
        """Called by the watchdog thread (a DIFFERENT thread than this
        worker's own) when this robot has made no progress for
        WATCHDOG_TIMEOUT_SEC -- meaning its OWN thread is presumed stuck
        and will never call arrive_and_wait() again (unlike a clean
        setup-failure or caught exception, both of which keep participating
        as a no-op -- see those call sites). send_cmd() is just a
        rosbridge publish, safe to call cross-thread. Does not (cannot,
        safely) kill the worker's own thread if it's truly stuck in a
        blocking call -- marks state so the summary reflects the exclusion
        and PERMANENTLY lowers the gate's required count via exclude() so
        the remaining, still-working robots aren't left waiting for a
        headcount that can never be reached again."""
        self.excluded_by_watchdog = True
        try:
            self.nav.send_cmd("STOP")
        except Exception:
            pass
        self.nav.get_logger().error(
            f"{self.robot}: WATCHDOG force-stop ({reason}) -- excluded from "
            "the rest of this run")
        self.gate.exclude()

    def shutdown(self) -> None:
        try:
            self.nav.send_cmd("STOP")
            for _ in range(5):
                self.nav._spin_once(0.05)
        except Exception:
            pass
        try:
            self.nav.destroy_node()
        except Exception:
            pass
        # Called after this worker's thread has already joined (see main()),
        # so no other thread is still writing to these -- safe to close here
        # without a lock.
        for f in self._open_files:
            try:
                f.close()
            except Exception:
                pass


def watchdog_loop(workers: list[RobotWorker], stop_event: threading.Event) -> None:
    while not stop_event.wait(WATCHDOG_POLL_SEC):
        for w in workers:
            if w.excluded_by_watchdog or not w.thread.is_alive():
                continue
            stale = w.seconds_since_progress()
            if stale > WATCHDOG_TIMEOUT_SEC:
                w.force_stop(f"no progress for {stale:.0f}s")


# -- statistics (pure stdlib, no scipy dependency) --------------------------

def linear_trend_slope(values: list[float]) -> float | None:
    """Ordinary least-squares slope of values against their own index
    (0, 1, 2, ...) -- degrees of |final_error| drift per rotation, for the
    encoder_drift stage. Hand-rolled (no numpy/scipy dependency assumed on
    the Linux laptop this runs on): slope = sum((x-xbar)(y-ybar)) /
    sum((x-xbar)^2)."""
    n = len(values)
    if n < 2:
        return None
    xs = list(range(n))
    xbar = statistics.mean(xs)
    ybar = statistics.mean(values)
    num = sum((x - xbar) * (y - ybar) for x, y in zip(xs, values))
    den = sum((x - xbar) ** 2 for x in xs)
    if den == 0:
        return None
    return num / den


def summarize_mode(results: list[dict]) -> dict:
    if not results:
        return {"n": 0}
    errs = [abs(r["final_error"]) for r in results]
    elapsed = [r["elapsed_sec"] for r in results]
    corrected = sum(1 for r in results if r.get("corrected"))
    return {
        "n": len(errs),
        "err_mean": statistics.mean(errs),
        "err_min": min(errs),
        "err_max": max(errs),
        "err_stdev": statistics.stdev(errs) if len(errs) > 1 else 0.0,
        "elapsed_mean": statistics.mean(elapsed),
        "elapsed_min": min(elapsed),
        "elapsed_max": max(elapsed),
        "elapsed_total": sum(elapsed),
        "corrected": corrected,
    }


def write_summary(outdir: str, robots: list[str], robot_modes: dict[str, list[str]],
                   all_results: dict[str, dict[str, list[dict]]],
                   setup_failures: dict[str, str | None],
                   crashes: dict[str, tuple[str, str] | None],
                   watchdog_excluded: dict[str, bool]) -> None:
    path = os.path.join(outdir, "SUMMARY.md")
    lines: list[str] = []
    lines.append("# Fleet Yaw-Source Stress Test Summary")
    lines.append("")
    lines.append(f"Generated: {datetime.now().isoformat(timespec='seconds')}")
    lines.append("")
    lines.append("## Scope and goal")
    lines.append("")
    lines.append(
        "This experiment ran camera_grid_navigate.py's yaw-source stress "
        "test (128 there-and-back ROTATE_REL rotations per stage) on "
        "MULTIPLE robots SIMULTANEOUSLY, testing three comparable modes "
        "(encoder / camera_assist / camera_only -- see "
        "YAW_STRESS_TEST_FINDINGS.md for what each means) plus a 4th "
        "encoder_drift stage that observes real accumulated onboard-"
        "encoder drift with no correction. turn_to_heading_rotate_rel() -- "
        "the method actually used by run() and fleetSupervisor.py's live "
        "dispatch path -- defaults to camera_assist behavior based on "
        "earlier single-robot results; this run's goal is to confirm that "
        "holds up under real multi-robot camera/rosbridge contention and "
        "to check for robot-to-robot variation single-robot tests can't "
        "reveal.")
    lines.append("")
    lines.append(
        "**Counterbalanced order**: each robot ran the 3 comparable modes "
        "in a DIFFERENT sequence (6x3 Latin square, so battery/motor-heat/"
        "fatigue effects aren't confounded with which mode happened to run "
        "first fleet-wide). All 6 robots still start each STAGE together "
        "(a shared gate holds every robot until all 6 finish the current "
        "stage, plus a 5s warm-up, before any robot starts the next) -- "
        "but 'stage 1' is a different MODE for different robots. Per-robot "
        "assignment:")
    lines.append("")
    lines.append("| Robot | Stage 1 | Stage 2 | Stage 3 | Stage 4 |")
    lines.append("|---|---|---|---|---|")
    for robot in robots:
        modes = robot_modes.get(robot, [])
        row = " | ".join(modes) if modes else "?"
        lines.append(f"| {robot} | {row} | encoder_drift |")
    lines.append("")

    any_issues = False
    for robot in robots:
        if setup_failures.get(robot):
            lines.append(f"**{robot}: SETUP FAILED** -- {setup_failures[robot]}. "
                          "No data collected for this robot.")
            any_issues = True
        crash = crashes.get(robot)
        if crash:
            mode, tb = crash
            lines.append(f"**{robot}: CRASHED** during [{mode}] -- see this "
                          f"robot's terminal log for the full traceback. "
                          "Data from stages before the crash is still valid "
                          "(written incrementally).")
            any_issues = True
        if watchdog_excluded.get(robot):
            lines.append(f"**{robot}: WATCHDOG-EXCLUDED** -- stalled with no "
                          "progress and was force-stopped. Data up to the "
                          "stall point is still valid.")
            any_issues = True
    if any_issues:
        lines.append("")

    lines.append("## Per-robot, per-mode statistics")
    lines.append("")
    lines.append("| Robot | Mode | n | mean err (deg) | min | max | stdev | "
                  "mean elapsed (s) | min | max | total (s) | corrected |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for robot in robots:
        for mode in ACCURACY_MODES + [DRIFT_MODE]:
            results = all_results.get(robot, {}).get(mode, [])
            s = summarize_mode(results)
            if s["n"] == 0:
                lines.append(f"| {robot} | {mode} | 0 | - | - | - | - | - | - | - | - | - |")
                continue
            corrected_str = f"{s['corrected']}/{s['n']}" if mode == "camera_assist" else "-"
            lines.append(
                f"| {robot} | {mode} | {s['n']} | {s['err_mean']:.2f} | "
                f"{s['err_min']:.2f} | {s['err_max']:.2f} | {s['err_stdev']:.2f} | "
                f"{s['elapsed_mean']:.2f} | {s['elapsed_min']:.2f} | "
                f"{s['elapsed_max']:.2f} | {s['elapsed_total']:.1f} | {corrected_str} |")
    lines.append("")

    lines.append("## Pooled across all robots, per mode (descriptive only)")
    lines.append("")
    lines.append(
        "Descriptive reference numbers only -- NOT the test statistic (see "
        "'Statistics: per-robot ranking + sign test' below for why pooling "
        "768 rotations as if independent overstates confidence; the real "
        "sample size for cross-robot claims is 6 robots, not 768 rotations).")
    lines.append("")
    lines.append("| Mode | n | mean err (deg) | min | max | stdev | "
                  "mean elapsed (s) | min | max | total (s) |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    pooled: dict[str, list[dict]] = {m: [] for m in ACCURACY_MODES + [DRIFT_MODE]}
    for robot in robots:
        for mode in ACCURACY_MODES + [DRIFT_MODE]:
            pooled[mode].extend(all_results.get(robot, {}).get(mode, []))
    for mode in ACCURACY_MODES + [DRIFT_MODE]:
        s = summarize_mode(pooled[mode])
        if s["n"] == 0:
            lines.append(f"| {mode} | 0 | - | - | - | - | - | - | - | - |")
            continue
        lines.append(
            f"| {mode} | {s['n']} | {s['err_mean']:.2f} | {s['err_min']:.2f} | "
            f"{s['err_max']:.2f} | {s['err_stdev']:.2f} | {s['elapsed_mean']:.2f} | "
            f"{s['elapsed_min']:.2f} | {s['elapsed_max']:.2f} | {s['elapsed_total']:.1f} |")
    lines.append("")

    lines.append("## Statistics: per-robot ranking + sign test")
    lines.append("")
    lines.append(
        "The FLEET-LEVEL test statistic. Each robot is treated as ONE "
        "unit of replication (n=6), not 768 independent rotations "
        "(pseudoreplication) -- for each robot, the 3 modes are ranked by "
        "mean |final_error|; then for each mode pair, a sign test counts "
        "how many of the 6 robots had mode X more accurate than mode Y. "
        "6/6 or 0/6 is a clean, robot-independent result; anything closer "
        "to 3/6 means the modes don't reliably differ once you account for "
        "robot-to-robot variation, even if a naive pooled comparison would "
        "look significant.")
    lines.append("")
    lines.append("### Per-robot mode ranking (best/lowest mean error first)")
    lines.append("")
    lines.append("| Robot | 1st | 2nd | 3rd |")
    lines.append("|---|---|---|---|")
    robot_rankings: dict[str, list[str]] = {}
    for robot in robots:
        means = {}
        for mode in ACCURACY_MODES:
            results = all_results.get(robot, {}).get(mode, [])
            if results:
                means[mode] = statistics.mean(abs(r["final_error"]) for r in results)
        ranked = sorted(means, key=lambda m: means[m])
        robot_rankings[robot] = ranked
        if len(ranked) == 3:
            lines.append(
                f"| {robot} | {ranked[0]} ({means[ranked[0]]:.2f}deg) | "
                f"{ranked[1]} ({means[ranked[1]]:.2f}deg) | "
                f"{ranked[2]} ({means[ranked[2]]:.2f}deg) |")
        else:
            lines.append(f"| {robot} | incomplete data ({len(ranked)}/3 modes) | | |")
    lines.append("")

    lines.append("### Sign test across robots, per mode pair")
    lines.append("")
    lines.append("| Comparison | X more accurate | Y more accurate | tied | result |")
    lines.append("|---|---|---|---|---|")
    mode_pairs = [("encoder", "camera_assist"), ("encoder", "camera_only"),
                  ("camera_assist", "camera_only")]
    for mode_x, mode_y in mode_pairs:
        x_wins = y_wins = ties = 0
        for robot in robots:
            results_x = all_results.get(robot, {}).get(mode_x, [])
            results_y = all_results.get(robot, {}).get(mode_y, [])
            if not results_x or not results_y:
                continue
            mean_x = statistics.mean(abs(r["final_error"]) for r in results_x)
            mean_y = statistics.mean(abs(r["final_error"]) for r in results_y)
            if mean_x < mean_y:
                x_wins += 1
            elif mean_y < mean_x:
                y_wins += 1
            else:
                ties += 1
        n_decisive = x_wins + y_wins
        if n_decisive == 0:
            verdict = "insufficient data"
        elif x_wins == n_decisive:
            verdict = f"{mode_x} more accurate on EVERY robot ({x_wins}/{n_decisive})"
        elif y_wins == n_decisive:
            verdict = f"{mode_y} more accurate on EVERY robot ({y_wins}/{n_decisive})"
        else:
            verdict = (f"mixed ({x_wins}/{n_decisive} favor {mode_x}, "
                       f"{y_wins}/{n_decisive} favor {mode_y}) -- not a "
                       "robot-independent result")
        lines.append(f"| {mode_x} vs {mode_y} | {x_wins} | {y_wins} | {ties} | {verdict} |")
    lines.append("")

    lines.append("## encoder_drift: real accumulated drift over the run")
    lines.append("")
    lines.append(
        "Stage 4 for every robot, run with NO reset and NO per-rotation "
        "resync -- every other stage resyncs onboard yaw to vision after "
        "every single rotation, so this is the only stage where real "
        "accumulated onboard-encoder drift is actually visible. Inherits "
        "whatever onboard yaw state each robot's OWN stage-3 mode left "
        "(which differs by robot under counterbalancing -- see the "
        "assignment table above). Reported as the linear trend slope of "
        "|final_error| against rotation index: positive = error grew over "
        "the run (real drift); near zero = the onboard encoder held up "
        "over 128 rotations with no correction.")
    lines.append("")
    lines.append("| Robot | n | slope (deg/rotation) | first-10 mean err | last-10 mean err |")
    lines.append("|---|---|---|---|---|")
    for robot in robots:
        results = all_results.get(robot, {}).get(DRIFT_MODE, [])
        if len(results) < 2:
            lines.append(f"| {robot} | {len(results)} | - | - | - |")
            continue
        errs = [abs(r["final_error"]) for r in results]
        slope = linear_trend_slope(errs)
        first10 = statistics.mean(errs[:10]) if len(errs) >= 10 else statistics.mean(errs)
        last10 = statistics.mean(errs[-10:]) if len(errs) >= 10 else statistics.mean(errs)
        slope_str = f"{slope:+.4f}" if slope is not None else "-"
        lines.append(f"| {robot} | {len(errs)} | {slope_str} | {first10:.2f} | {last10:.2f} |")
    lines.append("")
    lines.append(
        "Per-robot slopes reported individually rather than pooled -- "
        "pooling all robots into one continuously-increasing index would "
        "let between-robot differences masquerade as a time trend.")
    lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Summary written to {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", required=True,
                     help="comma-separated robot names, e.g. "
                          "Alvik1,Alvik2,Alvik3,Alvik4,Alvik5,Alvik6 -- "
                          "assigned to LATIN_SQUARE rows in order given, "
                          "so the ORDER of this list determines each "
                          "robot's mode sequence (see SUMMARY.md's "
                          "assignment table after the run)")
    ap.add_argument("--outdir", default=None,
                     help="output directory for per-robot CSVs + SUMMARY.md "
                          "(default: fleet_yaw_stress_<timestamp>/)")
    args = ap.parse_args()

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    if len(robots) < 1:
        ap.error("--robots requires at least one robot name")
    if len(robots) > len(LATIN_SQUARE):
        ap.error(f"--robots supports at most {len(LATIN_SQUARE)} robots "
                  f"(LATIN_SQUARE only has {len(LATIN_SQUARE)} rows), got {len(robots)}")

    robot_modes = {robot: LATIN_SQUARE[i] for i, robot in enumerate(robots)}

    outdir = args.outdir or f"fleet_yaw_stress_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(outdir, exist_ok=True)
    print(f"Output directory: {outdir}")
    print(f"Robots: {', '.join(robots)}")
    for robot in robots:
        print(f"  {robot}: {' -> '.join(robot_modes[robot])} -> {DRIFT_MODE}")

    rclpy.init()
    gate = StageGate(len(robots))
    workers = [RobotWorker(r, robot_modes[r], outdir, gate) for r in robots]
    watchdog_stop = threading.Event()
    watchdog_thread = threading.Thread(
        target=watchdog_loop, args=(workers, watchdog_stop), daemon=True)

    try:
        watchdog_thread.start()
        for w in workers:
            w.start()
        for w in workers:
            w.join()
    except KeyboardInterrupt:
        print("KeyboardInterrupt -- stopping all robots")
    finally:
        watchdog_stop.set()
        for w in workers:
            w.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    all_results = {w.robot: w.results for w in workers}
    setup_failures = {w.robot: w.setup_failed for w in workers}
    crashes = {w.robot: ((w.crashed_stage, w.crash_traceback)
                          if w.crashed_stage else None) for w in workers}
    watchdog_excluded = {w.robot: w.excluded_by_watchdog for w in workers}
    write_summary(outdir, robots, robot_modes, all_results,
                  setup_failures, crashes, watchdog_excluded)


if __name__ == "__main__":
    main()
