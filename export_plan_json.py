#!/usr/bin/env python3
"""
export_plan_json.py — run the vrp_rpd solvers (romeshprasad/vrp_rpd,
branch pipeline_completed) on this testbed's instance and emit a visit plan
that agv_grid_workstation_solver.html can load via "Import plan".

The HTML stays the executive: it turns the imported visit order into timed
routes, enforces processing-ready times, deconflicts with dwells, generates
Juan.ino commands and hands the mission to the fleet supervisor — identical
execution stack for every algorithm, which is what makes the comparison fair.

Setup (one time):
    cd vrp_rpd && git checkout pipeline_completed && cd ..
    pip install numpy numba          # solver dependencies

Usage:
    # 1. In the HTML: select bays, set agents/capacity/processing, Solve,
    #    then "Download JSON" -> plan.json  (defines the instance)
    # 2. Run an algorithm on that instance:
    python export_plan_json.py --config plan.json --algorithm nn --out plan_nn.json
    #    algorithms: nn | regret | defer | heuristics(best of 3) | brkga
    # 3. In the HTML: "Import plan" -> plan_nn.json -> review -> Send mission

Travel in the vrp_rpd solvers is measured in grid hops, so processing time
is converted to hop-equivalents with --sec-per-hop (default 3.56 s, the
measured RED->RED hop time) to keep the travel/processing trade-off honest.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SEC_PER_HOP_DEFAULT = 3.56  # measured MOVEMENT_SEC.RED_RED on the table

ALGORITHMS = ("nn", "regret", "defer", "heuristics", "brkga")


def load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    for key in ("grid", "agents", "capacity", "workstations"):
        if key not in cfg:
            raise SystemExit(f"--config {path} has no '{key}' — export it from "
                             "the solver HTML via Download JSON")
    return cfg


def parse_kv_args(text: str) -> dict:
    """'a=1,b=2.5,c=True,d=x' -> {'a': 1, 'b': 2.5, 'c': True, 'd': 'x'}"""
    out = {}
    for part in (text or "").split(","):
        part = part.strip()
        if not part:
            continue
        key, _, value = part.partition("=")
        v = value.strip()
        if v.lower() in ("true", "false"):
            coerced = v.lower() == "true"
        else:
            try:
                coerced = int(v)
            except ValueError:
                try:
                    coerced = float(v)
                except ValueError:
                    coerced = v
        out[key.strip()] = coerced
    return out


def solve(cfg: dict, algorithm: str, process_sec: float, sec_per_hop: float,
          seed: int, repo: str, brkga_kwargs: dict | None = None):
    # pipeline_completed layout: <repo>/vrp_rpd/ is the solver package and
    # <repo>/agv_testbed/ the testbed pipeline; inside the package,
    # vrp_rpd/agv_testbed is a git symlink to ../agv_testbed, which Windows
    # checks out as a stub file — so we extend the package __path__ instead.
    repo_path = Path(repo).resolve()
    pkg = repo_path / "vrp_rpd"
    if not (pkg / "__init__.py").exists() or not (repo_path / "agv_testbed").is_dir():
        raise SystemExit(
            f"--repo {repo_path} doesn't look like a pipeline_completed "
            "checkout (needs vrp_rpd/__init__.py and agv_testbed/). The "
            "plain clone is on master; make a worktree:\n"
            "  git -C vrp_rpd worktree add ../external/vrp_rpd "
            "origin/pipeline_completed")
    sys.path.insert(0, str(repo_path))
    try:
        import vrp_rpd as _pkg
        if not (pkg / "agv_testbed").is_dir():
            _pkg.__path__.append(str(repo_path))  # resolve the symlink stub
        from vrp_rpd import (
            decode_chromosome,
            generate_greedy_defer_solution,
            generate_max_regret_solution,
            generate_nearest_neighbor_solution,
        )
        from vrp_rpd.agv_testbed.grid_env import build_grid_from_ui
        from vrp_rpd.agv_testbed.instance_builder import build_vrp_instance
        from vrp_rpd.agv_testbed.vrp_solver import (
            _build_result, run_brkga, run_heuristics)
    except ImportError as exc:
        raise SystemExit(
            f"cannot import the vrp_rpd pipeline ({exc}).\n"
            f"--repo {repo_path} must be a pipeline_completed checkout.\n"
            "Dependencies: pip install numpy numba") from exc

    rows = cfg["grid"]["rows"]
    cols = cfg["grid"]["cols"]
    agents = int(cfg["agents"])
    capacity = int(cfg["capacity"])

    # Our export's north_entry_nodes are 1-indexed and match the repo's
    # between_nodes convention (verified against workstations.json).
    edges0 = [tuple(n - 1 for n in ws["north_entry_nodes"])
              for ws in cfg["workstations"]]
    proc_hops = [process_sec / sec_per_hop] * len(edges0)
    grid = build_grid_from_ui(rows, cols, edges0, processing_times=proc_hops)

    if algorithm == "brkga":
        # Without workers the solver never evolves — it returns its heuristic
        # warm start. Pass e.g. num_cpu_workers=2,total_generations=300 for a
        # real GA run (parameter values: ask Romesh what he uses).
        result = run_brkga(grid, num_agents=agents,
                           resources_per_agent=capacity,
                           **(brkga_kwargs or {}))
    elif algorithm == "heuristics":
        result = run_heuristics(grid, num_agents=agents,
                                resources_per_agent=capacity)
    else:
        instance = build_vrp_instance(grid, agents, capacity)
        args = (instance.dist, instance.proc, instance.depot,
                instance.m, instance.k, instance.num_customers)
        if algorithm == "nn":
            _, _, tours = generate_nearest_neighbor_solution(
                *args, allow_mixed=True)
            result = _build_result("Nearest Neighbor", tours, instance, grid)
        elif algorithm == "regret":
            _, _, tours = generate_max_regret_solution(*args, allow_mixed=True)
            result = _build_result("Max Regret", tours, instance, grid)
        else:  # defer
            chrom, _ = generate_greedy_defer_solution(
                *args, defer_multiplier=10.0, allow_mixed=True)
            tours = decode_chromosome(chrom, instance, allow_mixed=True)
            result = _build_result("Greedy Defer", tours, instance, grid)

    return result, grid


def events_to_visits(result, grid, cfg: dict) -> dict:
    """SolverResult events [(ws_id, 'D'|'P', idx)] -> HTML import visits."""
    # entry-node pair (1-indexed, sorted) -> our bay number
    pair_to_bay = {tuple(sorted(ws["north_entry_nodes"])): ws["bay"]
                   for ws in cfg["workstations"]}
    ws_id_to_bay = {}
    for i, ws_id in enumerate(grid.workstation_ids):
        a0, b0 = grid.between_nodes[i]
        pair = tuple(sorted((a0 + 1, b0 + 1)))
        if pair not in pair_to_bay:
            raise SystemExit(f"solver workstation between nodes {pair} has no "
                             "matching bay in --config — layouts disagree")
        ws_id_to_bay[ws_id] = pair_to_bay[pair]

    visits: dict[str, list] = {}
    for agent_id, plan in sorted(result.agents.items()):
        name = f"Alvik{agent_id + 1}"
        seq = []
        for ws_id, op, _solver_idx in plan.events:
            if ws_id == grid.depot:
                if seq:  # depot mid-tour = a reload trip
                    raise SystemExit(
                        f"{name}: solver produced a multi-trip tour (depot "
                        "revisit mid-route) — the executor runs one tour per "
                        "robot; raise capacity or reduce bays")
                continue  # leading depot bookend
            seq.append({"bay": int(ws_id_to_bay[ws_id]),
                        "mode": "drop" if op == "D" else "pick"})
        visits[name] = seq
    return visits


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Run a vrp_rpd algorithm on the testbed instance and "
                    "write the HTML-importable plan JSON.")
    ap.add_argument("--config", required=True,
                    help="plan JSON exported from the solver HTML "
                         "(Download JSON) — defines bays/agents/capacity")
    ap.add_argument("--algorithm", choices=ALGORITHMS, default="heuristics")
    ap.add_argument("--process-sec", type=float, default=None,
                    help="workstation processing seconds (default: the "
                         "config's rpd.process_sec, else 15)")
    ap.add_argument("--sec-per-hop", type=float, default=SEC_PER_HOP_DEFAULT,
                    help="seconds per grid hop for unit conversion "
                         f"(default {SEC_PER_HOP_DEFAULT})")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--repo",
                    default=str(Path(__file__).resolve().parent / "external" / "vrp_rpd"),
                    help="path to a vrp_rpd checkout on pipeline_completed "
                         "(default: ./external/vrp_rpd worktree)")
    ap.add_argument("--out", default=None,
                    help="output path (default plan_<algorithm>.json)")
    ap.add_argument("--brkga", default="",
                    help="comma-separated solver kwargs for --algorithm "
                         "brkga, e.g. 'num_cpu_workers=2,total_generations="
                         "300,use_gp=False' (empty = warm-start only)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    process_sec = args.process_sec
    if process_sec is None:
        process_sec = float(cfg.get("rpd", {}).get("process_sec", 15.0))

    result, grid = solve(cfg, args.algorithm, process_sec,
                         args.sec_per_hop, args.seed, args.repo,
                         brkga_kwargs=parse_kv_args(args.brkga))
    visits = events_to_visits(result, grid, cfg)

    out_path = args.out or f"plan_{args.algorithm}.json"
    payload = {
        "algorithm": result.heuristic,
        "source": "romeshprasad/vrp_rpd pipeline_completed",
        "process_sec": process_sec,
        "capacity": int(cfg["capacity"]),
        "solver_makespan_hops": float(result.makespan),
        "solver_makespan_sec_estimate": float(result.makespan) * args.sec_per_hop,
        "visits": visits,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    total = sum(len(v) for v in visits.values())
    print(f"\n{result.heuristic}: solver makespan {result.makespan:.1f} hops "
          f"(~{payload['solver_makespan_sec_estimate']:.0f}s)")
    for name, seq in visits.items():
        drops = sum(1 for v in seq if v["mode"] == "drop")
        print(f"  {name}: {len(seq)} visits ({drops} drops, "
              f"{len(seq) - drops} picks)")
    print(f"{total} visits written to {out_path} — load it in the HTML via "
          "'Import plan'")


if __name__ == "__main__":
    main()
