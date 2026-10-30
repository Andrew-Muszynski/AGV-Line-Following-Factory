#!/usr/bin/env python3
"""Compare today's turn-method arms: staged vs rotate_rel.

    python3 tools/compare_turn_arms.py                 # runs from today
    python3 tools/compare_turn_arms.py --since 2026-10-15

Groups runs by the EFFECTIVE turn method captured in runs.tuning_json, so an
arm is identified by what actually ran rather than by when you think you
switched. Runs with no capture are listed separately and never pooled.

Reports the two things the choice turns on:

    accuracy   terminal yaw vs the 2 deg requirement
    cost       makespan, and time lost to the drive->turn seam fault

THE SEAM FAULT IS THE RISK. rotate_rel turns on the motor MCU via
alvik.rotate(); drive legs stream wheel speeds, and any wheel-speed packet
cancels an in-flight rotate at the carrier. Every no-motion failure recorded
on 2026-09-22 and 09-24 was the FIRST rotation after a drive leg. It never
appeared in 1,354 isolated rotations. So watch per-robot exclusions, not just
makespan: this method's failure mode is losing a robot mid-run, which shows
up as an abort or a short completion, not as a bad angle.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import math
import os
import sqlite3
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from metrics_engine import REQUIRED_DEPOT_HEADING_DEG, wrap_deg  # noqa: E402

LATEST = ("m.revision = (SELECT MAX(revision) FROM metric_values x "
          "WHERE x.run_id = m.run_id)")


def arm_of(tuning_json):
    if not tuning_json:
        return None
    try:
        t = json.loads(tuning_json)
    except (TypeError, ValueError):
        return None
    return t.get("effective_turn_method") or (t.get("motion") or {}).get("turn_method")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default=dt.date.today().isoformat(),
                    help="ISO date; default today")
    ap.add_argument("--db", default=os.path.join(ROOT, "data",
                                                 "agv_metrics.sqlite3"))
    a = ap.parse_args()

    conn = sqlite3.connect(a.db)
    conn.row_factory = sqlite3.Row
    runs = [dict(r) for r in conn.execute(
        """SELECT run_id, created_at_utc, status, robot_count, tuning_json
             FROM runs
            WHERE substr(created_at_utc,1,10) >= ?
              AND status IN ('completed','aborted','failed')
            ORDER BY created_at_utc""", (a.since,))]
    if not runs:
        print(f"no executed runs since {a.since}")
        return 0

    by_arm = collections.defaultdict(list)
    for r in runs:
        by_arm[arm_of(r["tuning_json"]) or "(not captured)"].append(r)

    print(f"{len(runs)} executed run(s) since {a.since}\n")
    print(f"{'arm':16s} {'runs':>5s} {'completed':>10s} {'aborted':>8s} "
          f"{'failed':>7s}")
    for arm, rs in sorted(by_arm.items()):
        c = sum(1 for x in rs if x["status"] == "completed")
        ab = sum(1 for x in rs if x["status"] == "aborted")
        f = sum(1 for x in rs if x["status"] == "failed")
        print(f"{arm:16s} {len(rs):>5d} {c:>10d} {ab:>8d} {f:>7d}")

    print("\nACCURACY — terminal yaw vs the 2 deg requirement")
    print(f"{'arm':16s} {'n':>5s} {'mean':>8s} {'sd':>6s} {'>2deg':>7s} "
          f"{'p95':>7s} {'max':>7s}")
    for arm, rs in sorted(by_arm.items()):
        ids = [x["run_id"] for x in rs]
        q = ",".join("?" * len(ids))
        errs = [wrap_deg(row[0] - REQUIRED_DEPOT_HEADING_DEG)
                for row in conn.execute(
                    f"""SELECT terminal_yaw_deg FROM run_robots
                         WHERE run_id IN ({q}) AND terminal_yaw_deg IS NOT NULL""",
                    ids)]
        if not errs:
            print(f"{arm:16s} {'0':>5s}   (no terminal yaw recorded)")
            continue
        aerr = sorted(abs(e) for e in errs)
        print(f"{arm:16s} {len(errs):>5d} {st.mean(errs):+8.2f} "
              f"{st.pstdev(errs):6.2f} "
              f"{100*sum(1 for e in errs if abs(e) > 2)/len(errs):6.0f}% "
              f"{aerr[int(0.95*len(aerr))]:7.2f} {max(aerr):7.2f}")

    print("\nCOST — makespan and robot completion")
    print(f"{'arm':16s} {'n':>5s} {'makespan':>9s} {'sd':>6s} "
          f"{'robots done':>12s}")
    for arm, rs in sorted(by_arm.items()):
        ids = [x["run_id"] for x in rs]
        q = ",".join("?" * len(ids))
        mk = [row[0] for row in conn.execute(
            f"""SELECT m.value FROM metric_values m
                 WHERE m.run_id IN ({q}) AND m.metric_key='actual_makespan_sec'
                   AND m.value IS NOT NULL AND {LATEST}""", ids)]
        done = conn.execute(
            f"""SELECT SUM(status='completed'), COUNT(*) FROM run_robots
                 WHERE run_id IN ({q})""", ids).fetchone()
        frac = (f"{done[0]}/{done[1]}" if done and done[1] else "-")
        if mk:
            print(f"{arm:16s} {len(mk):>5d} {st.mean(mk):9.1f} "
                  f"{(st.pstdev(mk) if len(mk) > 1 else 0):6.1f} {frac:>12s}")
        else:
            print(f"{arm:16s} {'0':>5s} {'-':>9s} {'-':>6s} {frac:>12s}")

    print("\nSEAM-FAULT WATCH — robots that did not finish, by arm")
    for arm, rs in sorted(by_arm.items()):
        ids = [x["run_id"] for x in rs]
        q = ",".join("?" * len(ids))
        bad = [dict(r) for r in conn.execute(
            f"""SELECT robot_id, status, failure_reason FROM run_robots
                 WHERE run_id IN ({q}) AND status != 'completed'""", ids)]
        if not bad:
            print(f"  {arm:16s} none")
            continue
        tally = collections.Counter(
            (b["robot_id"], (b["failure_reason"] or b["status"])[:52])
            for b in bad)
        print(f"  {arm:16s} {len(bad)} incomplete")
        for (robot, why), n in tally.most_common(8):
            print(f"      {robot:8s} x{n}  {why}")

    if len(by_arm) >= 2 and "(not captured)" not in by_arm:
        print("\nBoth arms captured. Re-run after each block; the arm label "
              "comes from the run row, so it cannot drift from what ran.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
