#!/usr/bin/env python3
"""Measure commanded-vs-achieved rotation across the angle range, one robot.

WHY
---
The 2026-09-23 yaw stress run showed the carrier undershoots every rotation by
a consistent per-robot factor (Alvik1 0.9579 ... Alvik5 0.9738, IQR 0.003-0.006,
no drift over 2000 rotations). Applying 1/k to the angle handed to
alvik.rotate() should remove it. Two gaps block that:

  1. The stress angles were 0.9, 1.5, 2.6, 3.0, 90, 90.3, 177.5, 179 -- so
     10-45deg was never sampled and the gain there is unmeasured.
  2. Alvik3 aborted its camera_only stage, so it has no gain at all.

Alvik3 closes both at once, and the sweep repeats angles the other five
already have (3 / 90 / 179), so its curve can be checked against theirs
before anyone assumes one robot's shape generalises.

WHAT THIS DELIBERATELY DOES NOT DO
----------------------------------
It never corrects the yaw. The quantity being measured is
(achieved - commanded), and a corrective rotation afterwards measures nothing
while costing several seconds per sample. The robot is allowed to end up
wherever it ends up; angles are issued in +/- pairs so its absolute heading
stays bounded without any correction.

Settling is the one thing it IS strict about, because the whole measurement is
a difference of two yaw readings: a sample is only taken once vision has been
quiet (SETTLE_TOL_DEG over SETTLE_WINDOW_SEC) both before the command and
after the firmware reports terminal.

TRANSPORT
---------
roslibpy over rosbridge, so this runs from the Windows laptop with nothing to
deploy. It publishes /<robot>_cmd and reads /<robot>_status and
/<robot>_vision_pose -- the same three topics the navigator uses.

    python3 tools/rotation_gain_sweep.py --robot Alvik3

Prerequisites: micro-ROS agent + rosbridge up on the Linux laptop, and
apriltag_localize.py publishing vision poses. The robot must be on its tag and
visible to the overhead camera, with room to spin in place.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics as st
import sys
import time
from collections import deque
from datetime import datetime

try:
    import roslibpy
except ImportError:                                            # pragma: no cover
    sys.exit("roslibpy is required:  python3 -m pip install roslibpy")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Angles to sweep, degrees. 3/90/179 overlap the 2026-09-23 stress run on the
# other five robots, so this robot's curve can be compared against theirs;
# 12-40 is the band that run never touched.
SWEEP_DEG = [3, 5, 8, 12, 16, 20, 25, 30, 35, 40, 45, 60, 75, 90, 120, 150, 179]

# Settling. Deliberately stricter and slower than camera_grid_navigate.py's
# CAM_STABLE_SEC=0.12 / CAM_STABLE_TOL_DEG=1.0: that gate is tuned to release a
# mission leg quickly, this one is tuned so the number is right.
SETTLE_WINDOW_SEC = 0.60
SETTLE_TOL_DEG = 0.40
SETTLE_MIN_SAMPLES = 6
SETTLE_TIMEOUT_SEC = 8.0
# Dead time after the terminal status before settle evaluation starts, so the
# tail of the deceleration is never inside the window.
POST_TERMINAL_DWELL_SEC = 0.45

VISION_STALE_SEC = 0.5
TERMINAL = ("ROTATE_REL COMPLETE", "ROTATE_REL BRAKED_UNCONFIRMED")


def norm(a: float) -> float:
    """Wrap to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


class Robot:
    """The three topics, and a settled-yaw reader over the vision stream."""

    def __init__(self, ros, name: str):
        self.name = name
        self.cmd = roslibpy.Topic(ros, f"/{name}_cmd", "std_msgs/String")
        self.status = roslibpy.Topic(ros, f"/{name}_status", "std_msgs/String")
        self.pose = roslibpy.Topic(ros, f"/{name}_vision_pose",
                                   "std_msgs/String")
        self.poses: deque = deque(maxlen=400)      # (t_monotonic, yaw_deg)
        # (seq, t_monotonic, text). The seq is NOT decoration: the deque is
        # bounded, so once it fills, len() pins at maxlen and any cursor based
        # on it stops advancing -- wait_terminal() then never sees another
        # message and every rotation reports TIMEOUT. That happened at sample
        # 80 of 102 on the 2026-09-24 Alvik3 run (~5 statuses per rotation x
        # 79 rotations = 395, just under 400). The measurements stayed valid,
        # since settled_yaw() does not depend on this, but the terminal state
        # and DIAG fields were lost and every sample cost the full timeout.
        self.statuses: deque = deque(maxlen=400)
        self.status_seq = 0
        self.status.subscribe(self._on_status)
        self.pose.subscribe(self._on_pose)

    def _on_status(self, msg):
        self.status_seq += 1
        self.statuses.append((self.status_seq, time.monotonic(),
                              str(msg.get("data", ""))))

    def _on_pose(self, msg):
        try:
            d = json.loads(msg["data"])
            self.poses.append((time.monotonic(), float(d["yaw_deg"])))
        except (ValueError, KeyError, TypeError):
            pass

    def send(self, text: str) -> None:
        self.cmd.publish(roslibpy.Message({"data": text}))

    def settled_yaw(self, timeout=SETTLE_TIMEOUT_SEC):
        """Median yaw once vision has been quiet, or None on timeout.

        Spread is computed against the first sample through norm() so a robot
        parked near +/-180 does not read as a 360deg swing.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            win = [(t, y) for t, y in self.poses
                   if now - t <= SETTLE_WINDOW_SEC]
            if len(win) >= SETTLE_MIN_SAMPLES and now - win[-1][0] < VISION_STALE_SEC:
                base = win[0][1]
                rel = [norm(y - base) for _t, y in win]
                if (max(rel) - min(rel) <= SETTLE_TOL_DEG
                        and win[-1][0] - win[0][0] >= 0.8 * SETTLE_WINDOW_SEC):
                    return norm(base + st.median(rel))
            time.sleep(0.03)
        return None

    def wait_terminal(self, timeout: float):
        """Block until the firmware reports a terminal ROTATE_REL state.

        Returns (terminal_text, diag_text) -- diag is the last
        'ROTATE_REL DIAG ...' seen in the same command, which carries peakgz
        and the firmware's own moved= estimate.
        """
        start = time.monotonic()
        diag = ""
        after = self.status_seq
        while time.monotonic() - start < timeout:
            for seq, _t, text in list(self.statuses):
                if seq <= after:
                    continue
                after = seq
                if "ROTATE_REL DIAG" in text:
                    diag = text
                if text in TERMINAL:
                    return text, diag
            time.sleep(0.02)
        return "TIMEOUT", diag

    def wait_idle(self, timeout=10.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self.statuses and self.statuses[-1][2] == "IDLE":
                return True
            time.sleep(0.05)
        return False

    def close(self):
        for t in (self.cmd, self.status, self.pose):
            try:
                t.unsubscribe()
            except Exception:
                pass
            try:
                t.unadvertise()
            except Exception:
                pass


def one_rotation(rb: Robot, angle: float, args) -> dict | None:
    """Issue one ROTATE_REL and measure what the chassis actually did."""
    if not rb.wait_idle():
        log("  robot never reported IDLE -- skipping")
        return None
    before = rb.settled_yaw()
    if before is None:
        log("  no settled vision before the command -- skipping")
        return None

    t0 = time.monotonic()
    rb.send(f"ROTATE_REL {angle:.2f}")
    # Firmware deadline is |angle|/ROTATE_DEG_PER_SEC + its own timeout; this
    # ceiling sits well outside it so a slow turn is measured, not abandoned.
    terminal, diag = rb.wait_terminal(timeout=abs(angle) / 100.0 + 9.0)
    elapsed = time.monotonic() - t0

    time.sleep(POST_TERMINAL_DWELL_SEC)
    after = rb.settled_yaw()
    if after is None:
        log(f"  {angle:+.1f}deg: no settled vision after the command")
        return None

    achieved = norm(after - before)
    pk = re.search(r"peakgz=(-?[\d.]+)", diag)
    mv = re.search(r"moved=(-?[\d.]+)", diag)
    return {
        "ts": f"{datetime.now():%H:%M:%S}",
        "commanded_deg": round(angle, 3),
        "yaw_before": round(before, 3),
        "yaw_after": round(after, 3),
        "achieved_deg": round(achieved, 3),
        "ratio": round(achieved / angle, 5) if angle else "",
        "terminal": terminal,
        "peakgz": pk.group(1) if pk else "",
        "fw_moved": mv.group(1) if mv else "",
        "elapsed_sec": round(elapsed, 2),
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="Alvik3")
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--repeats", type=int, default=6,
                    help="samples per angle; issued as +/- pairs so the "
                         "robot's absolute heading stays bounded without any "
                         "corrective rotation")
    ap.add_argument("--angles", default=None,
                    help="comma-separated override for the sweep")
    ap.add_argument("--settle-pause", type=float, default=0.3,
                    help="extra quiet time between rotations")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    angles = ([float(a) for a in args.angles.split(",")]
              if args.angles else [float(a) for a in SWEEP_DEG])
    out = args.out or os.path.join(
        ROOT, "runs_linux",
        f"rotation_gain_{args.robot}_{datetime.now():%m%d_%H%M}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    if not ros.is_connected:
        sys.exit(f"could not reach rosbridge at {args.host}:{args.port}")
    rb = Robot(ros, args.robot)
    log(f"connected; listening for {args.robot} status + vision pose")

    # Do not start measuring until both streams are actually flowing --
    # otherwise the first few angles get thrown away on 'no settled vision'.
    warm = time.monotonic()
    while time.monotonic() - warm < 10.0:
        if len(rb.poses) > 10 and rb.statuses:
            break
        time.sleep(0.2)
    if len(rb.poses) <= 10:
        rb.close(); ros.terminate()
        sys.exit(f"no vision poses on /{args.robot}_vision_pose -- is "
                 "apriltag_localize.py running and the robot on camera?")

    total = len(angles) * args.repeats
    log(f"{len(angles)} angles x {args.repeats} repeats = {total} rotations; "
        f"writing {out}")

    rows: list[dict] = []
    fields = ["ts", "commanded_deg", "yaw_before", "yaw_after", "achieved_deg",
              "ratio", "terminal", "peakgz", "fw_moved", "elapsed_sec"]
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        try:
            n = 0
            for ang in angles:
                got = []
                for i in range(args.repeats):
                    # Alternate sign: each +/- pair returns the robot to
                    # roughly where it started, which is why no corrective
                    # rotation is needed anywhere in this test.
                    signed = ang if i % 2 == 0 else -ang
                    n += 1
                    r = one_rotation(rb, signed, args)
                    if r:
                        rows.append(r)
                        w.writerow(r)
                        fh.flush()          # survive a Ctrl-C mid-sweep
                        got.append(abs(r["achieved_deg"]) / ang)
                        log(f"  [{n}/{total}] {signed:+7.1f}deg -> "
                            f"{r['achieved_deg']:+7.2f}  ratio={r['ratio']}  "
                            f"{r['terminal']}")
                    time.sleep(args.settle_pause)
                if got:
                    log(f"  angle {ang:>5.1f}deg  n={len(got)}  "
                        f"median ratio={st.median(got):.4f}")
        except KeyboardInterrupt:
            log("interrupted -- rows already written are kept")
        finally:
            try:
                rb.send("STOP")
                time.sleep(0.3)
            except Exception:
                pass
            rb.close()
            ros.terminate()

    if not rows:
        log("no usable samples")
        return

    print()
    print(f"  {'angle':>7} {'n':>4} {'median k':>10} {'IQR':>8} "
          f"{'-> command x':>13}")
    byang: dict[float, list[float]] = {}
    for r in rows:
        byang.setdefault(abs(r["commanded_deg"]), []).append(
            abs(r["achieved_deg"]) / abs(r["commanded_deg"]))
    for a in sorted(byang):
        v = sorted(byang[a])
        med = st.median(v)
        iqr = (v[min(len(v) - 1, int(0.75 * len(v)))]
               - v[int(0.25 * len(v))]) if len(v) >= 4 else float("nan")
        print(f"  {a:>7.1f} {len(v):>4} {med:>10.4f} {iqr:>8.4f} "
              f"{1 / med if med else float('nan'):>13.4f}")
    big = [k for a in byang if a >= 45 for k in byang[a]]
    if big:
        print(f"\n  {args.robot} gain over |angle| >= 45deg: "
              f"k={st.median(big):.4f}  -> multiply command by "
              f"{1 / st.median(big):.4f}   (n={len(big)})")
    print(f"\n  wrote {out}")


if __name__ == "__main__":
    main()
