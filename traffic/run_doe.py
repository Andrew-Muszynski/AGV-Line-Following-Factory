"""Run the DOE's simulation tier: scenarios x traffic-planning methods.

    python -m traffic.run_doe solved_plans/a.json solved_plans/b.json
    python -m traffic.run_doe generated_plans/*.json --methods pp,cbs --time-limit 60
    python -m traffic.run_doe --check-page generated_plans/nearest_2026-10-06_17-00-11.json

Each scenario is a saved plan whose job allocation (which robot visits which
bays, in what order) is frozen; only the traffic planner changes. For every
(scenario, method) this writes:

  * one row of traffic_results.csv -- planner success, independent validity,
    planned makespan, conflicts by kind, waits, path length, turns, planning
    time. The variant/experiment/dataset/instance/makespan columns let
    external/vrp_rpd/statistical_analysis.py read the file as it stands;
  * one plan file in traffic_plans/, in the page's export format, which the
    page's "Import plan" takes as planned -- the road to the hardware tier.

--check-page judges an existing page export (the incumbent's own JS WAIT
plan, as dispatched today) with the same validator.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

from .export import load_page_paths, to_page_plan, write_page_plan
from .instance import load
from .planners import METHODS, solve
from .validate import metrics, validate

FIELDS = [
    "run_at", "scenario", "robots", "rpd", "method", "planner_success",
    "valid", "planning_sec", "planned_makespan_sec", "sum_completion_sec",
    "conflicts", "conflicts_node", "conflicts_lane", "conflicts_rotation",
    "readiness_violations", "job_errors", "traffic_wait_sec",
    "service_wait_sec", "pick_ready_wait_sec", "hops", "drive_sec",
    "turns_90", "turns_180", "note", "stats",
    # statistical_analysis.py's columns
    "variant", "experiment", "dataset", "instance", "makespan",
]


def _row(inst, method, success, planning_sec, note, stats, paths):
    rep = validate(inst, paths) if paths else {
        "valid": False, "errors": ["no plan"], "conflicts": [],
        "readiness": []}
    met = metrics(inst, paths, rep)
    row = {
        "run_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "scenario": inst.name, "robots": len(inst.tasks), "rpd": inst.rpd,
        "method": method, "planner_success": success, "valid": rep["valid"],
        "planning_sec": round(planning_sec, 3), "note": note,
        "stats": json.dumps(stats, default=str),
        **met,
        "variant": "base", "experiment": method,
        "dataset": f"{len(inst.tasks)}r", "instance": inst.name,
        "makespan": met.get("planned_makespan_sec", ""),
    }
    return row, rep


def _append(out_csv: Path, rows: list[dict]) -> None:
    new = not out_csv.exists()
    with open(out_csv, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("scenarios", nargs="*", help="saved plan JSON files")
    ap.add_argument("--methods", default=",".join(METHODS),
                    help=f"comma list from {','.join(METHODS)}")
    ap.add_argument("--time-limit", type=float, default=30.0,
                    help="per (scenario, method) planning budget, seconds")
    ap.add_argument("--seed", type=int, default=0,
                    help="PP restart order seed")
    ap.add_argument("--out", default="traffic_results.csv")
    ap.add_argument("--plans", default="traffic_plans",
                    help="directory for dispatchable plan files ('' = none)")
    ap.add_argument("--check-page", metavar="PAGE_JSON",
                    help="validate a page export as dispatched")
    args = ap.parse_args(argv)

    rows = []
    if args.check_page:
        inst = load(args.check_page)
        data = json.loads(Path(args.check_page).read_text(encoding="utf-8"))
        paths = load_page_paths(inst, data)
        row, rep = _row(inst, "page", True, 0.0, "page export as dispatched",
                        {}, paths)
        rows.append(row)
        print(f"{inst.name}: valid={rep['valid']} makespan="
              f"{row.get('planned_makespan_sec')}s conflicts={row.get('conflicts')}")
        for e in rep["errors"]:
            print("  error:", e)
        for c in rep["conflicts"][:20]:
            print("  conflict:", c.describe())
    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    plans_dir = Path(args.plans) if args.plans else None
    if plans_dir:
        plans_dir.mkdir(parents=True, exist_ok=True)
    for sc in args.scenarios:
        inst = load(sc)
        for m in methods:
            res = solve(inst, m, time_limit=args.time_limit, seed=args.seed)
            row, rep = _row(inst, m, res.success, res.planning_sec, res.note,
                            res.stats, res.paths)
            rows.append(row)
            if plans_dir and res.paths:
                write_page_plan(plans_dir / f"{inst.name}__{m}.json",
                                to_page_plan(inst, res.paths, m, rep, row))
            print(f"{inst.name:40s} {m:6s} valid={rep['valid']!s:5s} "
                  f"makespan={row.get('planned_makespan_sec', '-'):>8} "
                  f"conflicts={row.get('conflicts', '-'):>3} "
                  f"plan={res.planning_sec:6.2f}s  {res.note}")
    if rows:
        _append(Path(args.out), rows)
        print(f"\n{len(rows)} rows -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
