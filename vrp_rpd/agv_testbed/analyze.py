"""
Analysis & Data Collection Script
----------------------------------
Runs the VRP-RPD + MAPF pipeline across multiple datasets and variants,
collecting metrics at two points:
  1. After VRP-RPD heuristic search (Layer 1)
  2. After MAPF collision resolution (Layer 2)

Usage
-----
  # Run all datasets, base variant only
  python3 analyze.py

  # Run specific datasets
  python3 analyze.py --datasets gr17 bays29 berlin52

  # Run specific variants
  python3 analyze.py --variants base 2x

  # Full run, save results
  python3 analyze.py --datasets gr17 gr21 gr24 gr48 bays29 berlin52 eil51 \
                     --variants base 2x 5x \
                     --output results.json

  # Verbose: print per-agent detail
  python3 analyze.py --datasets gr17 --verbose
"""

from __future__ import annotations
import sys
import json
import argparse
import time
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional

sys.path.insert(0, str(Path(__file__).parent))

from vrp_rpd.agv_testbed.grid_env import load_dataset_grid, WarehouseGrid
from vrp_rpd.agv_testbed.vrp_solver import run_heuristics, SolverResult
from vrp_rpd.agv_testbed.mapf_solver import solve_mapf, MAPFResult
from vrp_rpd.agv_testbed.pipeline import run_pipeline, PipelineResult


# ── Dataset config: (num_agents, resources_per_agent) ───────────────────────
DATASET_CONFIG: Dict[str, tuple] = {
    "gr17":    (3, 5),
    "gr21":    (3, 5),
    "gr24":    (4, 6),
    "gr48":    (4, 6),
    "bays29":  (4, 6),
    "berlin52":(4, 6),
    "eil51":   (4, 6),
}

DEFAULT_DATASETS = list(DATASET_CONFIG.keys())
DEFAULT_VARIANTS = ["base", "2x", "5x"]


# ── Result containers ────────────────────────────────────────────────────────

@dataclass
class AgentMetric:
    agent_id: int
    completion_time: float
    n_dropoffs: int
    n_pickups: int
    mapf_steps: int          # total timesteps in MAPF path (includes waits)
    mapf_done_t: int         # timestep when agent reaches final depot


@dataclass
class RunResult:
    dataset: str
    variant: str
    num_agents: int
    resources_per_agent: int
    n_customers: int
    seed: int

    # VRP-RPD (Layer 1)
    vrp_heuristic: str
    vrp_makespan: float
    vrp_agent_times: List[float]        # completion time per agent
    vrp_priority_order: List[int]       # agent ids, most critical first

    # MAPF (Layer 2)
    mapf_converged: bool
    mapf_iterations: int
    mapf_max_timestep: int              # wall-clock steps in collision-free execution
    mapf_conflicts_remaining: int

    # Per-agent detail
    agents: List[AgentMetric] = field(default_factory=list)

    # Timing
    wall_time_s: float = 0.0

    # Total wait steps inserted by MAPF across all agents (MAPF overhead)
    total_wait_steps: int = 0

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


# ── Core run function ────────────────────────────────────────────────────────

def run_one(
    dataset: str,
    variant: str,
    seed: int = 42,
    max_iterations: int = 5,
    verbose: bool = False,
) -> RunResult:
    num_agents, resources = DATASET_CONFIG[dataset]

    t0 = time.time()

    grid = load_dataset_grid(dataset, variant=variant, seed=seed)

    result: PipelineResult = run_pipeline(
        grid,
        num_agents=num_agents,
        resources_per_agent=resources,
        max_iterations=max_iterations,
        seed=seed,
        dataset_dir=dataset,
        variant=variant,
    )

    vrp = result.vrp_result
    mapf = result.mapf_result
    wall = time.time() - t0

    # Per-agent metrics + compute MAPF wait steps
    agent_metrics = []
    total_waits = 0
    for aid, plan in sorted(vrp.agents.items()):
        tp = mapf.paths.get(aid)
        mapf_steps = len(tp.path) if tp and tp.path else 0
        mapf_done_t = tp.path[-1][1] if tp and tp.path else 0

        # Minimum hops = sum of BFS distances along planned visit sequence
        visit_seq = plan.visit_sequence
        min_hops = sum(
            int(result.vrp_result.grid.dist[visit_seq[i], visit_seq[i+1]])
            for i in range(len(visit_seq) - 1)
        )
        # Wait steps = extra steps added by MAPF beyond the minimum path
        wait_steps = max(0, mapf_steps - 1 - min_hops)
        total_waits += wait_steps

        agent_metrics.append(AgentMetric(
            agent_id=aid,
            completion_time=plan.completion_time,
            n_dropoffs=sum(1 for _, op, _ in plan.events if op == 'D'),
            n_pickups=sum(1 for _, op, _ in plan.events if op == 'P'),
            mapf_steps=mapf_steps,
            mapf_done_t=mapf_done_t,
        ))

    run = RunResult(
        dataset=dataset,
        variant=variant,
        num_agents=num_agents,
        resources_per_agent=resources,
        n_customers=len(result.vrp_result.grid.workstations),
        seed=seed,
        vrp_heuristic=vrp.heuristic,
        vrp_makespan=vrp.makespan,
        vrp_agent_times=[vrp.agents[a].completion_time for a in sorted(vrp.agents)],
        vrp_priority_order=vrp.priority_order(),
        mapf_converged=result.converged,
        mapf_iterations=result.iterations,
        mapf_max_timestep=mapf.max_timestep(),
        mapf_conflicts_remaining=len(mapf.conflicts),
        total_wait_steps=total_waits,
        agents=agent_metrics,
        wall_time_s=round(wall, 2),
    )

    if verbose:
        _print_verbose(run)

    return run


# ── Printing helpers ─────────────────────────────────────────────────────────

def _print_verbose(r: RunResult):
    print(f"\n  Per-agent breakdown:")
    for a in r.agents:
        priority_rank = r.vrp_priority_order.index(a.agent_id) + 1
        print(f"    Agent {a.agent_id} [priority #{priority_rank}]: "
              f"VRP done@{a.completion_time:.0f}  "
              f"drops={a.n_dropoffs}  picks={a.n_pickups}  "
              f"MAPF total steps={a.mapf_steps}  MAPF done@hop={a.mapf_done_t}")


def _print_row(r: RunResult):
    conv = "✓" if r.mapf_converged else "✗"
    print(
        f"  {r.dataset:<10} {r.variant:<6} "
        f"m={r.num_agents} k={r.resources_per_agent} "
        f"n={r.n_customers:>2} | "
        f"VRP makespan={r.vrp_makespan:>8.0f} ({r.vrp_heuristic[:9]:<9}) | "
        f"MAPF hops={r.mapf_max_timestep:>4} "
        f"waits={r.total_wait_steps:>3} "
        f"conf={r.mapf_conflicts_remaining} {conv} "
        f"iters={r.mapf_iterations} | "
        f"{r.wall_time_s:.1f}s"
    )


def print_summary_table(results: List[RunResult]):
    width = 120
    print("\n" + "=" * width)
    print("SUMMARY TABLE")
    print(
        f"  {'Dataset':<10} {'Var':<6} "
        f"{'Cfg':<12} {'n':>3} | "
        f"{'VRP Makespan (time units)':>30} | "
        f"{'MAPF hops':>9} {'Waits':>5} {'Conf':>4} {'Conv':>4} {'Iters':>5} | "
        f"{'Time':>5}"
    )
    print("-" * width)

    prev_ds = None
    for r in results:
        if prev_ds and r.dataset != prev_ds:
            print()
        _print_row(r)
        prev_ds = r.dataset

    print("=" * width)

    # Aggregate stats
    total_waits = sum(r.total_wait_steps for r in results)
    conv_rate = 100.0 * sum(1 for r in results if r.mapf_converged) / len(results)
    print(f"\n  Total MAPF wait steps inserted : {total_waits}")
    print(f"  Avg waits per run              : {total_waits/len(results):.1f}")
    print(f"  Convergence rate               : {conv_rate:.0f}% ({sum(1 for r in results if r.mapf_converged)}/{len(results)} runs)")
    print(f"  Total wall time                : {sum(r.wall_time_s for r in results):.1f}s")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="VRP-RPD + MAPF Analysis")
    parser.add_argument(
        "--datasets", nargs="+", default=DEFAULT_DATASETS,
        choices=list(DATASET_CONFIG.keys()),
        help="Datasets to run (default: all 7)",
    )
    parser.add_argument(
        "--variants", nargs="+", default=["base"],
        choices=["base", "2x", "5x", "1R10", "1R20"],
        help="Processing-time variants (default: base)",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="RNG seed for workstation placement (default: 42)",
    )
    parser.add_argument(
        "--max-iterations", type=int, default=5,
        help="Max MAPF feedback iterations (default: 5)",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Save results to this JSON file",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Print per-agent breakdown for each run",
    )
    args = parser.parse_args()

    all_results: List[RunResult] = []
    total = len(args.datasets) * len(args.variants)
    run_idx = 0

    for dataset in args.datasets:
        for variant in args.variants:
            run_idx += 1
            print(f"\n[{run_idx}/{total}] {dataset.upper()} / {variant}")
            print("-" * 60)
            try:
                r = run_one(
                    dataset=dataset,
                    variant=variant,
                    seed=args.seed,
                    max_iterations=args.max_iterations,
                    verbose=args.verbose,
                )
                all_results.append(r)
                _print_row(r)
            except Exception as e:
                print(f"  ERROR: {e}")

    if not all_results:
        print("No results collected.")
        return

    print_summary_table(all_results)

    if args.output:
        out = {
            "config": {
                "datasets": args.datasets,
                "variants": args.variants,
                "seed": args.seed,
                "max_iterations": args.max_iterations,
            },
            "results": [r.to_dict() for r in all_results],
        }
        with open(args.output, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nResults saved to: {args.output}")


if __name__ == "__main__":
    main()
