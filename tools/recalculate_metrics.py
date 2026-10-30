#!/usr/bin/env python3
"""Recompute stored metrics for runs whose data arrived after finalization.

WHY THIS EXISTS. Metrics are finalized when the run's terminal event lands,
but the supervisor keeps flushing pose batches well past that moment -- on the
2026-10-13 reference run the last event arrived 71 minutes after the metrics
were computed. Anything derived from telemetry was therefore frozen against a
partial timeline, and a run finalized before its own terminal event was
visible kept a null makespan forever.

As of 2026-10-14 that was 64 of 98 finalized runs: 67 values that should have
had a number showed a dash, and 755 more (mostly per-robot distance) were
computed from a fraction of the poses.

app.py now refreshes a stale run on read, so this is a one-shot backfill plus
a way to re-run the whole database deliberately.

    python3 tools/recalculate_metrics.py --dry-run     # report, change nothing
    python3 tools/recalculate_metrics.py               # backfill stale runs
    python3 tools/recalculate_metrics.py --all         # every finalized run

Each recomputation is written as a NEW revision. Nothing is overwritten, and
the API serves the highest revision, so the old numbers stay auditable.
"""

from __future__ import annotations

import argparse
import os
import sqlite3
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import database          # noqa: E402
import metrics_engine    # noqa: E402

DB = os.path.join(ROOT, "data", "agv_metrics.sqlite3")


def stale_runs(conn, include_all=False):
    """Finalized runs whose newest event postdates their newest metric."""
    rows = conn.execute(
        """SELECT r.run_id, r.status, r.created_at_utc,
                  (SELECT MAX(time_utc) FROM events e
                    WHERE e.run_id = r.run_id)            AS last_event,
                  (SELECT MAX(computed_at_utc) FROM metric_values m
                    WHERE m.run_id = r.run_id)            AS last_computed
             FROM runs r
            ORDER BY r.created_at_utc""").fetchall()
    out = []
    for row in rows:
        if row["last_computed"] is None:
            continue                       # never finalized; not ours to fix
        if include_all or (row["last_event"] is not None
                           and str(row["last_event"]) > str(row["last_computed"])):
            out.append(row)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default=DB)
    ap.add_argument("--dry-run", action="store_true",
                    help="report what would change and write nothing")
    ap.add_argument("--all", action="store_true",
                    help="every finalized run, not only the stale ones")
    args = ap.parse_args()

    conn = sqlite3.connect(args.db)
    conn.row_factory = sqlite3.Row
    targets = stale_runs(conn, include_all=args.all)
    total = conn.execute("SELECT COUNT(*) FROM runs").fetchone()[0]
    print(f"database : {args.db}")
    print(f"runs     : {total}")
    print(f"targets  : {len(targets)}"
          f"{' (all finalized)' if args.all else ' stale'}")
    if not targets:
        print("nothing to do")
        return 0

    filled = changed = 0
    touched = 0
    for row in targets:
        rid = row["run_id"]
        prev_rev = database.max_final_revision(conn, rid)
        before = {(m["metric_key"], m["scope_type"], m["scope_id"] or ""): m["value"]
                  for m in conn.execute(
                      "SELECT * FROM metric_values WHERE run_id = ? AND revision = ?",
                      (rid, prev_rev))}
        try:
            values = metrics_engine.compute_run_metrics(conn, rid)
        except Exception as exc:
            print(f"  SKIP {rid[:8]}  {type(exc).__name__}: {exc}")
            continue

        n_fill = n_chg = 0
        for v in values:
            key = (v["metric_key"], v["scope_type"], v["scope_id"] or "")
            old, new = before.get(key), v["value"]
            if old is None and new is not None:
                n_fill += 1
            elif old is not None and new is None:
                # A value WITHDRAWN -- a stored number the engine no longer
                # stands behind, e.g. a duration from a stale ended_at_utc.
                # Counted as a change: missing this case left three runs
                # publishing a 29-day "observed duration" after the fix that
                # was meant to retract it.
                n_chg += 1
            elif old is not None and new is not None and abs(new - old) > 1e-6:
                n_chg += 1
        if not (n_fill or n_chg):
            continue

        touched += 1
        filled += n_fill
        changed += n_chg
        print(f"  {row['created_at_utc'][:16]}  {rid[:8]}  {row['status']:9s}"
              f"  +{n_fill:3d} filled  ~{n_chg:3d} changed"
              f"  rev {prev_rev} -> {prev_rev + 1}"
              f"{'  [dry run]' if args.dry_run else ''}")
        if args.dry_run:
            continue
        for v in values:
            if v["status"] == "ok":
                v["status"] = "final"
        database.store_metric_values(conn, rid, values, "final",
                                     revision=prev_rev + 1)

    verb = "would fill" if args.dry_run else "filled"
    verb2 = "would correct" if args.dry_run else "corrected"
    print(f"\n{touched} run(s): {verb} {filled} null value(s), "
          f"{verb2} {changed} stale value(s)")
    if args.dry_run:
        print("nothing written (--dry-run)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
