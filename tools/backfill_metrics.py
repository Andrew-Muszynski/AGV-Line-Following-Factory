#!/usr/bin/env python3
"""Repair two gaps in already-recorded runs. Nothing is ever deleted.

WHY THIS EXISTS
---------------
Both gaps were found on 2026-09-22 and both are fixed going forward (auto
finalize in app.py, terminal pose on DEPOT_ARRIVED in fleet/metrics_events.py).
Neither fix helps a run that is already in the database, and there were 18 of
those, including the r4 reference run. This backfills them from data that was
captured at the time.

1. TERMINAL POSE. run_robots.terminal_x_m / terminal_y_m / terminal_yaw_deg
   were NULL on all 145 rows, because DEPOT_ARRIVED was emitted with only a
   robot id and an elapsed time while the ingest reads x_m/y_m/yaw_deg. The
   pose was still recorded, though -- POSE_SAMPLE wrote it to
   telemetry_samples ~5x a second -- so the terminal pose is recoverable as
   the last valid sample at or before the robot's completion time. That is
   what the Robots tab's "final yaw" column reads, and what
   terminal_heading_error_deg is computed from.

2. FINALIZATION. A run's metric values stay 'provisional' until something
   POSTs /finalize, and the only caller was the browser fallback, for the run
   that browser session created. Runs that ended with page 4 closed, or that
   were aborted, kept every event and never got a final revision -- so the
   page showed a dash for a makespan it had all the data to compute.

ORDER MATTERS: pose is backfilled first, so the finalized values include the
heading metrics rather than locking in their absence.

Dry run by default. --apply writes, after copying the database beside itself.
"""
from __future__ import annotations

import argparse
import os
import shutil
import sqlite3
import sys
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

DEFAULT_DB = os.path.join(ROOT, "data", "agv_metrics.sqlite3")

# A sample further than this from the recorded completion time is not the
# terminal pose -- the robot was somewhere else. Telemetry runs at ~5 Hz, so
# a genuine match is normally within a few hundred ms.
POSE_TOLERANCE_MS = 2000


def backfill_terminal_pose(conn, apply_changes):
    rows = conn.execute(
        """SELECT run_id, robot_id, completion_elapsed_ms
           FROM run_robots
           WHERE terminal_yaw_deg IS NULL
             AND completion_elapsed_ms IS NOT NULL""").fetchall()
    filled, too_far, no_sample = 0, 0, 0
    for r in rows:
        s = conn.execute(
            """SELECT elapsed_ms, x_m, y_m, yaw_deg FROM telemetry_samples
               WHERE run_id = ? AND robot_id = ? AND valid = 1
                 AND yaw_deg IS NOT NULL AND elapsed_ms <= ?
               ORDER BY elapsed_ms DESC LIMIT 1""",
            (r["run_id"], r["robot_id"], r["completion_elapsed_ms"])).fetchone()
        if s is None:
            no_sample += 1
            continue
        if r["completion_elapsed_ms"] - s["elapsed_ms"] > POSE_TOLERANCE_MS:
            too_far += 1
            continue
        if apply_changes:
            conn.execute(
                """UPDATE run_robots
                      SET terminal_x_m = COALESCE(terminal_x_m, ?),
                          terminal_y_m = COALESCE(terminal_y_m, ?),
                          terminal_yaw_deg = COALESCE(terminal_yaw_deg, ?)
                    WHERE run_id = ? AND robot_id = ?""",
                (s["x_m"], s["y_m"], s["yaw_deg"], r["run_id"], r["robot_id"]))
        filled += 1
    return {"candidates": len(rows), "filled": filled,
            "sample_too_old": too_far, "no_sample": no_sample}


def finalize_runs(conn, apply_changes):
    import database
    import metrics_engine
    rows = conn.execute(
        """SELECT r.run_id, r.created_at_utc, r.status, r.robot_count
           FROM runs r
           WHERE r.started_at_utc IS NOT NULL
             AND r.status IN ('completed', 'failed', 'aborted')
             AND EXISTS (SELECT 1 FROM events e WHERE e.run_id = r.run_id)
           ORDER BY r.created_at_utc""").fetchall()
    done, skipped, failed = [], 0, []
    for r in rows:
        if database.max_final_revision(conn, r["run_id"]) is not None:
            skipped += 1
            continue
        try:
            values = metrics_engine.compute_run_metrics(conn, r["run_id"])
            for v in values:
                if v["status"] == "ok":
                    v["status"] = "final"
            n = (database.store_metric_values(conn, r["run_id"], values,
                                              "final", revision=0)
                 if apply_changes else len(values))
            done.append((r["created_at_utc"][:16], r["status"],
                         r["robot_count"], n))
        except Exception as exc:                        # noqa: BLE001
            failed.append((r["run_id"][:8], f"{type(exc).__name__}: {exc}"))
    return {"already_final": skipped, "finalized": done, "errors": failed}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--db", default=DEFAULT_DB)
    ap.add_argument("--apply", action="store_true",
                    help="write changes (default is a dry run)")
    args = ap.parse_args()

    if args.apply:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = f"{args.db}.backup_{stamp}"
        shutil.copyfile(args.db, backup)
        print(f"database copied to {os.path.basename(backup)}")
    else:
        print("DRY RUN -- nothing will be written. Re-run with --apply.")
    print()

    conn = sqlite3.connect(args.db)
    conn.row_factory = sqlite3.Row

    pose = backfill_terminal_pose(conn, args.apply)
    print("TERMINAL POSE")
    print(f"  rows missing a terminal yaw : {pose['candidates']}")
    print(f"  recoverable from telemetry  : {pose['filled']}")
    print(f"  nearest sample too old      : {pose['sample_too_old']}")
    print(f"  no usable sample at all     : {pose['no_sample']}")
    if args.apply:
        conn.commit()

    print()
    fin = finalize_runs(conn, args.apply)
    print("FINALIZATION")
    print(f"  already final, left alone   : {fin['already_final']}")
    print(f"  finalized now               : {len(fin['finalized'])}")
    for created, status, rc, n in fin["finalized"]:
        print(f"      {created}  {status:<9} {rc}r  {n} values")
    if fin["errors"]:
        print(f"  could not compute           : {len(fin['errors'])}")
        for rid, msg in fin["errors"]:
            print(f"      {rid}  {msg[:70]}")
    if args.apply:
        conn.commit()
    conn.close()
    print()
    print("done." if args.apply else "dry run complete -- no changes written.")


if __name__ == "__main__":
    main()
