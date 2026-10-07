#!/usr/bin/env python3
"""Run the same mission back to back, unattended, and report what happened.

WHY
---
A completed six-robot run currently costs about 1.9 hours of lab time, and
almost none of that is the mission -- a run is ~4 minutes. The cost is the
turnaround: re-sending the mission from the browser, watching for the finish,
deciding whether the fleet is in a fit state to go again. This drives that
loop from one process so a session can bank runs instead of babysitting them.

REPEATABILITY IS THE POINT
--------------------------
The mission payload is CAPTURED ONCE from the browser and replayed byte for
byte. Nothing is reconstructed, so every run in a batch carries the identical
schedule, the identical commands and the identical drive mode. Only two
fields change between runs, and they have to: a fresh metrics run_id, and the
timestamp. That is what makes a batch a set of replicates rather than a set
of similar-looking runs.

    # once, with the browser open: press Send mission and let this record it
    python3 tools/run_batch.py --capture --mission runs_linux/mission.json

    # then, as many times as the session allows
    python3 tools/run_batch.py --mission runs_linux/mission.json --repeat 8

THE PROTOCOL, WHICH THIS DOES NOT INVENT
----------------------------------------
    publish /fleet_mission  (std_msgs/String, JSON)   -> status "loaded"
    publish /fleet_control  "START"                   -> status "running"
    wait                                              -> "finished" | "aborted" | "error"

fleetSupervisor.py must already be up in --listen mode; this is the same
sequence the HTML page performs, so nothing on the robot side changes.

WHAT IT WILL NOT DO
-------------------
It will not pick a robot up. After a failed run the fleet is somewhere in the
middle of the grid, and the next run must start from the depot stickers or it
is not a replicate. So a failure stops the batch and says so, unless you pass
--continue-on-fail, which pauses for you to re-park and press Enter.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from datetime import datetime

try:
    import roslibpy
except ImportError:                                            # pragma: no cover
    sys.exit("roslibpy is required:  python3 -m pip install roslibpy")
try:
    import requests
except ImportError:                                            # pragma: no cover
    requests = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TERMINAL = ("finished", "aborted", "error")


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


class Fleet:
    """Thin wrapper over the three topics the supervisor already speaks."""

    def __init__(self, host: str, port: int):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"could not reach rosbridge at {host}:{port}")
        self.mission = roslibpy.Topic(self.ros, "/fleet_mission",
                                      "std_msgs/String")
        self.control = roslibpy.Topic(self.ros, "/fleet_control",
                                      "std_msgs/String")
        self.status = roslibpy.Topic(self.ros, "/fleet_status",
                                     "std_msgs/String")
        self.last = None
        self.seen: list[dict] = []
        self.status.subscribe(self._on_status)

    def _on_status(self, msg):
        try:
            d = json.loads(msg["data"])
        except (ValueError, KeyError, TypeError):
            return
        self.last = d
        self.seen.append(d)

    def send_mission(self, payload: dict) -> None:
        self.mission.publish(roslibpy.Message(
            {"data": json.dumps(payload, separators=(",", ":"))}))

    def send_control(self, cmd: str) -> None:
        self.control.publish(roslibpy.Message({"data": cmd}))

    def wait_for(self, states, timeout: float) -> dict | None:
        """Block until fleet_status reports one of `states`."""
        deadline = time.monotonic() + timeout
        self.seen.clear()
        while time.monotonic() < deadline:
            for d in list(self.seen):
                if str(d.get("state", "")).lower() in states:
                    return d
            time.sleep(0.2)
        return None

    def close(self):
        for t in (self.mission, self.control, self.status):
            try:
                t.unadvertise()
            except Exception:
                pass
        self.ros.terminate()


def capture(fleet: Fleet, path: str, timeout: float) -> None:
    log("waiting for a mission from the browser -- press Send mission now")
    grabbed = {}

    def on_mission(msg):
        if not grabbed:
            grabbed["payload"] = msg.get("data", "")

    topic = roslibpy.Topic(fleet.ros, "/fleet_mission", "std_msgs/String")
    topic.subscribe(on_mission)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not grabbed:
        time.sleep(0.2)
    topic.unsubscribe()
    if not grabbed:
        raise SystemExit("no mission seen; is the browser connected to the "
                         "same rosbridge?")
    payload = json.loads(grabbed["payload"])
    robots = sorted((payload.get("commands") or {}).keys())
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=1)
    log(f"captured -> {path}")
    log(f"  robots={len(robots)} {' '.join(robots)}  "
        f"drive_mode={payload.get('drive_mode')}  "
        f"algorithm={payload.get('algorithm')}")


def register_run(metrics_url: str, payload: dict, prev_run_id: str | None):
    """Create the metrics run this mission will report against, mirroring
    metricsCreateRunForMission() in the browser. Without it the supervisor
    still runs, it just has nowhere to stream events."""
    if not metrics_url or requests is None:
        return None
    sched = payload.get("schedule") or {}
    jobs = sum(len(r.get("visits") or []) for r in (sched.get("routes") or []))
    body = {
        "experimental_condition": "%dr-%dws-%s" % (
            sched.get("agents") or 0,
            len(sched.get("workstations") or []),
            payload.get("drive_mode") or "vision"),
        "algorithm": payload.get("algorithm") or "built-in",
        "drive_mode": payload.get("drive_mode"),
        "robot_count": sched.get("agents"),
        "job_count": jobs,
        "workstation_count": len(sched.get("workstations") or []),
        "process_sec": (sched.get("rpd") or {}).get("process_sec"),
        "capacity": sched.get("capacity"),
        "planned_makespan_sec": sched.get("makespan_sec"),
        "plan_json": sched,
    }
    # Hands back the row an unstarted previous attempt already created, so a
    # batch does not litter the database the way re-arming used to.
    if prev_run_id:
        body["reuse_run_id"] = prev_run_id
    try:
        r = requests.post(f"{metrics_url.rstrip('/')}/api/runs",
                          json=body, timeout=6)
        r.raise_for_status()
        return r.json().get("run_id")
    except Exception as exc:                                   # noqa: BLE001
        log(f"  metrics registration skipped: {exc}")
        return None


def one_run(fleet: Fleet, payload: dict, args, run_id) -> dict:
    payload = dict(payload)
    payload["ms"] = int(time.time() * 1000)
    if run_id:
        payload["metrics_run_id"] = run_id
        payload["metrics_url"] = args.metrics
    else:
        payload.pop("metrics_run_id", None)

    t0 = time.monotonic()
    fleet.send_mission(payload)
    loaded = fleet.wait_for({"loaded"}, args.arm_timeout)
    if loaded is None:
        return {"outcome": "no-load", "sec": time.monotonic() - t0,
                "note": "supervisor never reported 'loaded'"}

    fleet.send_control("START")
    done = fleet.wait_for(set(TERMINAL), args.timeout)
    elapsed = time.monotonic() - t0
    if done is None:
        fleet.send_control("ABORT")
        return {"outcome": "timeout", "sec": elapsed,
                "note": f"no terminal state within {args.timeout:.0f}s"}

    state = str(done.get("state", "")).lower()
    note = str(done.get("note", ""))
    errored = re.search(r"errored=\[(.*?)\]", note)
    bad = bool(errored and errored.group(1).strip())
    outcome = "completed" if (state == "finished" and not bad) else state
    return {"outcome": outcome, "sec": elapsed, "note": note[:120]}


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--metrics", default="http://192.168.0.162:8000",
                    help="metrics backend; empty string to skip registration")
    ap.add_argument("--mission", default=os.path.join(ROOT, "runs_linux",
                                                      "mission.json"))
    ap.add_argument("--capture", action="store_true",
                    help="record the next mission from the browser and exit")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=420.0,
                    help="per-run ceiling in seconds")
    ap.add_argument("--arm-timeout", type=float, default=20.0)
    ap.add_argument("--settle", type=float, default=20.0,
                    help="pause between runs, for the fleet to sit still")
    ap.add_argument("--continue-on-fail", action="store_true",
                    help="pause for re-parking and carry on instead of stopping")
    args = ap.parse_args()

    fleet = Fleet(args.host, args.port)
    log(f"connected to rosbridge {args.host}:{args.port}")
    try:
        if args.capture:
            capture(fleet, args.mission, timeout=180.0)
            return

        with open(args.mission, encoding="utf-8") as fh:
            payload = json.load(fh)
        robots = sorted((payload.get("commands") or {}).keys())
        log(f"mission: {len(robots)} robots, drive_mode="
            f"{payload.get('drive_mode')}, algorithm={payload.get('algorithm')}")
        log(f"batch of {args.repeat}, {args.timeout:.0f}s ceiling, "
            f"{args.settle:.0f}s between")

        results, batch_t0, prev_id = [], time.monotonic(), None
        for i in range(1, args.repeat + 1):
            run_id = register_run(args.metrics, payload, prev_id)
            log(f"--- run {i}/{args.repeat}"
                + (f"  {run_id[:8]}" if run_id else "  (unregistered)"))
            res = one_run(fleet, payload, args, run_id)
            res["n"] = i
            res["run_id"] = run_id
            results.append(res)
            # A run that never started leaves its row reusable; a started one
            # must not be handed to the next attempt.
            prev_id = run_id if res["outcome"] == "no-load" else None
            log(f"    {res['outcome']}  {res['sec']:.0f}s"
                + (f"  {res['note']}" if res["note"] else ""))

            if res["outcome"] != "completed":
                if not args.continue_on_fail:
                    log("    stopping: the fleet is not on its depot stickers, "
                        "so the next run would not be a replicate")
                    break
                input("    re-park the robots, then press Enter to continue... ")
            if i < args.repeat:
                time.sleep(args.settle)

        mins = (time.monotonic() - batch_t0) / 60.0
        good = sum(1 for r in results if r["outcome"] == "completed")
        print()
        print("  %-4s %-11s %8s  %s" % ("run", "outcome", "sec", "run_id"))
        for r in results:
            print("  %-4d %-11s %8.0f  %s"
                  % (r["n"], r["outcome"], r["sec"], (r["run_id"] or "-")[:8]))
        print()
        log(f"{good}/{len(results)} completed in {mins:.0f} min"
            + (f"  ({mins / good:.1f} min per completed run)" if good else ""))
        if good:
            log(f"session rate {good / (mins / 60.0):.1f} completed/hour "
                f"against a historical 0.52")
    finally:
        fleet.close()


if __name__ == "__main__":
    main()
