"""Run the joint-vs-sequential experiment's planning tier.

    python3 -m traffic.run_alns --plan generated_plans/DOE_6r_pp.json --seeds 1,2,3
    python3 -m traffic.run_alns --n-bays 24 --bay-seed 7 --robots 6 --time-limit 120
    python3 -m traffic.run_alns --bays 9,10,5,1,14,24,27,30,42,17,22,31 --robots 6
    python3 -m traffic.run_alns --plan generated_plans/DOE_6r_pp.json --add-bays 12 --swaps --split

For every (workload, seed) both arms run the same ALNS for the same
wall-clock budget (traffic/alns.py explains the design). Each arm's best
allocation is then routed with PP exactly as the DOE's plans were, and:

  * one row per (workload, seed, arm) goes to alns_results.csv -- the
    arm's objective, the final PP plan's makespan and validity, and the
    traffic-free (A*) makespan of the same allocation, whose gap to PP is
    the traffic cost that allocation carries;
  * the PP plan goes to generated_plans/ in the page's export format, so it
    shows up in the page's Saved plans list and runs as planned (Import
    plan / Saved plans -> Generate commands -> Send mission).
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import sys
import time
from pathlib import Path

from .alns import ARMS, Problem, Workload, proxy_makespan, search
from .export import to_page_plan, write_page_plan
from .planners import solve_astar, solve_pp
from .validate import metrics, validate

ROOT = Path(__file__).resolve().parents[1]

FIELDS = [
    "run_at", "workload", "bays", "n_bays", "robots", "capacity", "process_sec",
    "swaps", "split", "service", "drop_sec", "seed", "arm", "time_limit_sec", "search_sec",
    "iterations", "evaluations", "start_objective_sec", "best_objective_sec",
    "final_pp_valid", "final_pp_makespan_sec", "final_astar_makespan_sec",
    "traffic_cost_sec", "proxy_makespan_sec", "final_conflicts",
    "traffic_wait_sec", "pick_ready_wait_sec", "service_wait_sec",
    "drive_sec", "turns_90", "turns_180", "allocation", "plan_file", "stats",
    # statistical_analysis.py's columns
    "variant", "experiment", "dataset", "instance", "makespan",
]


def parse_ints(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def workload_from_args(args) -> Workload:
    if args.plan:
        data = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        bays = sorted(int(w["bay"]) for w in data["workstations"])
        robots = args.robots or len(data["routes"])
        process = float((data.get("rpd") or {}).get("process_sec",
                                                    data.get("process_sec", 30)))
        if args.add_bays:
            n_total = (args.rows - 1) * (args.cols - 1)
            pool = [b for b in range(1, n_total + 1) if b not in bays]
            bays = sorted(bays + random.Random(args.bay_seed).sample(pool, args.add_bays))
        name = args.name or (f"{Path(args.plan).stem}_{robots}r"
                             + (f"_plus{args.add_bays}s{args.bay_seed}" if args.add_bays else ""))
    else:
        robots = args.robots or 6
        process = args.process
        if args.bays:
            bays = sorted(parse_ints(args.bays))
        else:
            n_total = (args.rows - 1) * (args.cols - 1)
            bays = sorted(random.Random(args.bay_seed).sample(
                range(1, n_total + 1), args.n_bays))
        name = args.name or f"{len(bays)}bays_s{args.bay_seed}_{robots}r"
    return Workload(bays=bays, robots=robots, capacity=args.capacity,
                    process_sec=process if args.process is None else args.process,
                    rows=args.rows, cols=args.cols, swaps=args.swaps,
                    split=args.split, service=args.service,
                    drop_sec=args.drop_sec, name=name)


def strip_page_dwell(plan: dict, inst, paths: dict, sec: float) -> None:
    """The plan times a `sec` stay at every workstation visit, but the page
    already emits a bare DWELL there (default 2 s) for every arrival, so
    that part of the stay must not be dispatched a second time. Only the
    waits beyond it (traffic, readiness) stay in the exported plan."""
    if sec <= 0:
        return
    by_name = {r["agv"]: r for r in plan["routes"]}
    for task in inst.tasks:
        p, route = paths[task.name], by_name[task.name]
        gi = 0
        for i in range(p.prefix, len(p.steps) - 1):
            st = p.steps[i]
            if gi < len(task.goals) - 1 and st.node == task.goals[gi].node:
                if task.goals[gi].mode in ("drop", "pick"):
                    left = round(max(0.0, st.wait - sec), 3)
                    key = str(i + 1)
                    if left > 1e-9:
                        route["wait_before_segment_sec"][key] = left
                    else:
                        route["wait_before_segment_sec"].pop(key, None)
                    route["segments"][i]["wait_before_sec"] = left
                    route["node_events"][i]["dwell_sec"] = left
                gi += 1
        route["dwell_wait_sec"] = round(sum(
            float(v) for k, v in route["wait_before_segment_sec"].items()
            if int(k) - 1 >= p.prefix), 3)
    plan["page_dwell_sec_not_exported"] = sec


def finalize(pb: Problem, res, seed: int, pp_limit: float):
    """Route the arm's best allocation with PP (the DOE's dispatch planner)
    and, for the record, with A* (traffic ignored)."""
    inst = pb.instance(res.best, name=f"ALNS_{pb.wl.name}_{res.arm}_s{seed}")
    pp = solve_pp(inst, time_limit=pp_limit, restarts=20, seed=0)
    astar = solve_astar(inst, time_limit=pp_limit)
    rep = validate(inst, pp.paths) if pp.paths else None
    met = metrics(inst, pp.paths, rep) if pp.paths else {}
    a_mk = max(p.end for p in astar.paths.values()) if astar.paths else None
    return inst, pp, rep, met, a_mk


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = ap.add_argument_group("workload")
    g.add_argument("--plan", help="take bays, robots and processing time from a saved plan")
    g.add_argument("--add-bays", type=int, default=0,
                   help="with --plan: add this many random bays (with --bay-seed)")
    g.add_argument("--bays", help="bay list, e.g. 9,10,5 or 1-12")
    g.add_argument("--n-bays", type=int, default=12, help="random bays (with --bay-seed)")
    g.add_argument("--bay-seed", type=int, default=0)
    g.add_argument("--robots", type=int, default=None)
    g.add_argument("--capacity", type=int, default=2)
    g.add_argument("--process", type=float, default=None,
                   help="processing time, s (default 30, or the plan's)")
    g.add_argument("--swaps", action="store_true",
                   help="allow drop and pick of a bay by different robots")
    g.add_argument("--split", action="store_true",
                   help="allow a robot's drop and pick of a bay to be apart")
    g.add_argument("--drop-sec", type=float, default=2.0,
                   help="the page's DWELL at a workstation (default 2 s, the "
                        "firmware's WORKSTATION_WAIT_MS); a drop's processing "
                        "starts when it ends")
    g.add_argument("--service", action="store_true",
                   help="allow a robot to wait in the bay for its own part "
                        "(the DOE's service stays); off by default")
    g.add_argument("--rows", type=int, default=8)
    g.add_argument("--cols", type=int, default=8)
    g.add_argument("--name", help="workload name used in file names")
    s = ap.add_argument_group("search")
    s.add_argument("--arms", default=",".join(ARMS))
    s.add_argument("--seeds", default="1", help="e.g. 1,2,3 or 1-5")
    s.add_argument("--time-limit", type=float, default=60.0,
                   help="ALNS wall-clock budget per arm and seed, s")
    s.add_argument("--max-iterations", type=int, default=None,
                   help="also stop after this many iterations (equal-effort runs)")
    s.add_argument("--final-limit", type=float, default=30.0,
                   help="budget for the final PP routing, s")
    o = ap.add_argument_group("output")
    o.add_argument("--out", default=str(ROOT / "alns_results.csv"))
    o.add_argument("--plans", default=str(ROOT / "generated_plans"),
                   help="directory for page-format plans ('' = none)")
    o.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    if args.process is None and not args.plan:
        args.process = 30.0

    wl = workload_from_args(args)
    pb = Problem(wl)
    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    for a in arms:
        if a not in ARMS:
            ap.error(f"--arms must be from {ARMS}")
    print(f"workload {wl.name}: {len(wl.bays)} bays {wl.bays}, {wl.robots} robots, "
          f"capacity {wl.capacity}, process {wl.process_sec:g}s after a "
          f"{wl.drop_sec:g}s drop, swaps={wl.swaps}, split={wl.split}, "
          f"service stays={wl.service}")
    plans_dir = Path(args.plans) if args.plans else None
    if plans_dir:
        plans_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for seed in parse_ints(args.seeds):
        for arm in arms:
            res = search(pb, arm, time_limit=args.time_limit, seed=seed,
                         max_iterations=args.max_iterations,
                         log=None if args.quiet else print)
            inst, pp, rep, met, a_mk = finalize(pb, res, seed, args.final_limit)
            plan_file = ""
            if plans_dir and pp.paths:
                plan = to_page_plan(inst, pp.paths, "pp", rep, met)
                plan["job_planner"] = f"alns:{arm}"
                plan["process_sec"] = wl.process_sec    # inst folds the drop stay in
                strip_page_dwell(plan, inst, pp.paths, wl.drop_sec)
                plan["alns"] = {
                    "arm": arm, "seed": seed, "workload": wl.name,
                    "bays": wl.bays, "time_limit_sec": args.time_limit,
                    "iterations": res.iterations, "evaluations": res.evaluations,
                    "best_objective_sec": res.best_eval.makespan,
                    "history": res.history,
                    "allocation": {n: [list(e) for e in s] for n, s in res.best.items()}}
                f = plans_dir / f"{inst.name}.json"
                write_page_plan(f, plan)
                plan_file = f.name
            pp_mk = met.get("planned_makespan_sec")
            row = {
                "run_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "workload": wl.name, "bays": " ".join(map(str, wl.bays)),
                "n_bays": len(wl.bays), "robots": wl.robots,
                "capacity": wl.capacity, "process_sec": wl.process_sec,
                "swaps": wl.swaps, "split": wl.split, "service": wl.service,
                "drop_sec": wl.drop_sec, "seed": seed, "arm": arm,
                "time_limit_sec": args.time_limit,
                "search_sec": round(res.seconds, 2),
                "iterations": res.iterations, "evaluations": res.evaluations,
                "start_objective_sec": round(res.start_eval.makespan, 2),
                "best_objective_sec": round(res.best_eval.makespan, 2),
                "final_pp_valid": rep["valid"] if rep else False,
                "final_pp_makespan_sec": pp_mk,
                "final_astar_makespan_sec": round(a_mk, 2) if a_mk else "",
                "traffic_cost_sec": round(pp_mk - a_mk, 2) if pp_mk and a_mk else "",
                "proxy_makespan_sec": round(proxy_makespan(pb, res.best)[0], 2),
                "final_conflicts": met.get("conflicts", ""),
                **{k: met.get(k, "") for k in ("traffic_wait_sec", "pick_ready_wait_sec",
                                                 "service_wait_sec", "drive_sec",
                                                 "turns_90", "turns_180")},
                "allocation": json.dumps({n: [f"{b}{e}" for b, e in s]
                                          for n, s in res.best.items()}),
                "plan_file": plan_file,
                "stats": json.dumps(res.op_stats),
                "variant": "base", "experiment": f"alns_{arm}",
                "dataset": f"{wl.robots}r", "instance": wl.name, "makespan": pp_mk,
            }
            rows.append(row)
            print(f"seed {seed} {arm:5s}: objective {res.start_eval.makespan:7.2f} -> "
                  f"{res.best_eval.makespan:7.2f}  |  final PP {pp_mk} "
                  f"(valid={row['final_pp_valid']}), A* {row['final_astar_makespan_sec']}, "
                  f"traffic cost {row['traffic_cost_sec']}  |  {res.iterations} it, "
                  f"{res.evaluations} evals  -> {plan_file}")
    out = Path(args.out)
    new = not out.exists()
    with open(out, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)
    print(f"\n{len(rows)} rows -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
