#!/usr/bin/env python3
"""Measure how far a robot bows off the straight line between two nodes.

WHY
---
`drive_leg()` steers on bearing to the target node:

    target_heading = heading_to_target_deg(dx, dy)
    heading_err    = yaw_error_deg(target_heading, pose.yaw)
    turn_adjust    = clamp(kp_yaw * heading_err, +/-max_turn_adjust)

There is **no cross-track term**. Nothing in that law references the tape line
between the two nodes, so a robot displaced sideways does not rejoin the line
-- it cuts a diagonal to the target and reports success. Staying on tape has
been relying on starting well-aligned and never being disturbed, and nothing
has ever measured the deviation.

2026-09-24: raising the pose publish rate from 20Hz to 50Hz made a robot drive
visible arcs between depot-entry nodes. The faster feed did not create the
weakness, it exposed it -- bearing-to-target now updates 2.5x more often, so
the chase is tighter and any oscillation shows as a bow off the line. Fixing it
properly means both retuning kp_yaw for the new rate AND adding the cross-track
term, and neither can be done without a number for "inches off the line".

WHAT IT MEASURES
----------------
Nothing about the plan or the grid -- purely what the camera saw. Each period
of continuous motion is one leg; the chord is that leg's own first and last
measured point, and the deviation is the maximum perpendicular distance from
the path to that chord. So it answers exactly one question, in inches: between
leaving one node and reaching the next, how far off the straight line did it
get?

Deriving the chord from measured endpoints rather than from node coordinates is
deliberate: it needs no grid constants, survives the robot stopping short, and
cannot be wrong in the same way the navigator might be.

USE
---
Start this first, then run the route on the Linux laptop:

    python3 tools/drive_straightness.py --robot Alvik4

    # on the Linux laptop, the real production code path
    python3 camera_grid_navigate.py --robot Alvik4 --route 1,9,17,25 --kp-yaw 0.6

Ctrl-C here when the route finishes; the report prints on exit. Sweep --kp-yaw
across runs and compare the worst-case column.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics as st
import sys
import time
from datetime import datetime

try:
    import roslibpy
except ImportError:                                            # pragma: no cover
    sys.exit("roslibpy is required:  python3 -m pip install roslibpy")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A leg is a run of continuous motion. These split it from the stationary
# periods around turns and stops.
MOVING_IN_PER_SEC = 1.0      # below this the robot counts as stopped
STOP_GAP_SEC = 0.45          # stationary for this long ends a leg
MIN_LEG_IN = 4.0             # ignore twitches; real legs are ~6-14in
# A driven leg is seen at the full vision rate (~50Hz) and moves at roughly
# cruise. A segment well outside either is reported as such and excluded from
# the summary, WITHOUT asserting a cause -- degraded AprilTag detection at that
# table position produces exactly the same signature as anything else, and on
# 2026-09-24 this tool wrongly told the operator a stationary robot had been
# picked up. It reports what the numbers are; it does not explain them.
LOW_RATE_HZ = 25.0
HIGH_SPEED_IN_PER_SEC = 4.0


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


def cross_track(px, py, ax, ay, bx, by) -> float:
    """Perpendicular distance from P to the infinite line AB, in inches."""
    dx, dy = bx - ax, by - ay
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dx * (ay - py) - (ax - px) * dy) / n


def segment(samples: list[tuple]) -> list[list[tuple]]:
    """Split (t, x, y, yaw) into continuous-motion legs."""
    legs, cur, still_since = [], [], None
    for i in range(1, len(samples)):
        t0, x0, y0, _ = samples[i - 1]
        t1, x1, y1, _ = samples[i]
        dt = t1 - t0
        if dt <= 0:
            continue
        speed = math.hypot(x1 - x0, y1 - y0) / dt
        if speed >= MOVING_IN_PER_SEC:
            still_since = None
            if not cur:
                cur = [samples[i - 1]]
            cur.append(samples[i])
        else:
            if cur:
                if still_since is None:
                    still_since = t1
                elif t1 - still_since >= STOP_GAP_SEC:
                    legs.append(cur)
                    cur, still_since = [], None
    if cur:
        legs.append(cur)
    out = []
    for leg in legs:
        ax, ay = leg[0][1], leg[0][2]
        bx, by = leg[-1][1], leg[-1][2]
        if math.hypot(bx - ax, by - ay) >= MIN_LEG_IN:
            out.append(leg)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="Alvik4")
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--label", default="",
                    help="free text recorded in the CSV, e.g. the kp_yaw used")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out_path = args.out or os.path.join(
        ROOT, "runs_linux",
        f"straightness_{args.robot}_{datetime.now():%m%d_%H%M}.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    samples: list[tuple] = []
    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    if not ros.is_connected:
        sys.exit(f"could not reach rosbridge at {args.host}:{args.port}")
    topic = roslibpy.Topic(ros, f"/{args.robot}_vision_pose", "std_msgs/String")

    def on_pose(msg):
        try:
            d = json.loads(msg["data"])
            samples.append((time.monotonic(), float(d["x_in"]),
                            float(d["y_in"]), float(d["yaw_deg"])))
        except (ValueError, KeyError, TypeError):
            pass

    topic.subscribe(on_pose)
    log(f"recording /{args.robot}_vision_pose — run the route now, "
        "Ctrl-C when it finishes")
    last_report = time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            if time.monotonic() - last_report >= 10.0:
                last_report = time.monotonic()
                log(f"  {len(samples)} poses, {len(segment(samples))} legs so far")
    except KeyboardInterrupt:
        log("stopping")
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
        ros.terminate()

    legs = segment(samples)
    if not legs:
        log(f"no legs detected in {len(samples)} poses — did the robot drive?")
        return

    rows = []
    for i, leg in enumerate(legs, 1):
        ax, ay = leg[0][1], leg[0][2]
        bx, by = leg[-1][1], leg[-1][2]
        devs = [cross_track(x, y, ax, ay, bx, by) for _t, x, y, _yaw in leg]
        rows.append({
            "leg": i,
            "label": args.label,
            "length_in": round(math.hypot(bx - ax, by - ay), 2),
            "sec": round(leg[-1][0] - leg[0][0], 2),
            "max_dev_in": round(max(devs), 3),
            "rms_dev_in": round(math.sqrt(st.fmean(d * d for d in devs)), 3),
            "n_poses": len(leg),
            "hz": round(len(leg) / max(1e-6, leg[-1][0] - leg[0][0]), 1),
            "in_per_sec": round(math.hypot(bx - ax, by - ay)
                                / max(1e-6, leg[-1][0] - leg[0][0]), 2),
        })
        rows[-1]["excluded"] = int(rows[-1]["hz"] < LOW_RATE_HZ
                                   or rows[-1]["in_per_sec"] > HIGH_SPEED_IN_PER_SEC)

    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print()
    print(f"  {'leg':>4} {'length in':>10} {'sec':>6} {'MAX off in':>11} "
          f"{'rms in':>8} {'Hz':>6} {'in/s':>6}  note")
    for r in rows:
        print(f"  {r['leg']:>4} {r['length_in']:>10.2f} {r['sec']:>6.2f} "
              f"{r['max_dev_in']:>11.3f} {r['rms_dev_in']:>8.3f} "
              f"{r['hz']:>6.1f} {r['in_per_sec']:>6.2f}"
              + (("  EXCLUDED: "
                  + ("pose rate %.1fHz, well under the ~50Hz a driven leg "
                     "gives" % r["hz"] if r["hz"] < LOW_RATE_HZ
                     else "%.2f in/s, faster than cruise" % r["in_per_sec"]))
                 if r["excluded"] else ""))
    good = [r for r in rows if not r["excluded"]]
    print()
    if not good:
        print("  NO USABLE LEGS. Every segment was excluded on pose rate or "
              "speed -- check the route actually ran and that the tag is "
              "being detected cleanly at this part of the table.")
    else:
        mx = [r["max_dev_in"] for r in good]
        print(f"  {len(good)} driven legs   worst {max(mx):.3f} in   "
              f"mean of per-leg maxima {st.fmean(mx):.3f} in"
              + (f"   [{args.label}]" if args.label else ""))
        if len(good) != len(rows):
            print(f"  ({len(rows) - len(good)} segment(s) excluded, see above)")
    print(f"  wrote {out_path}")


if __name__ == "__main__":
    main()
