"""
Two-Layer Pipeline: VRP-RPD + MAPF with Iterative Feedback
-----------------------------------------------------------
Layer 1: VRP-RPD heuristic search -> agent routes + completion times
Layer 2: Prioritized MAPF -> collision-free timed paths

If MAPF finds conflicts, the conflicted agents' routes are flagged and
Layer 1 is re-run with a perturbed seed so different routes emerge.
This continues until convergence (no conflicts) or max_iterations.

For Plan 1 (no stochasticity, single depot) the feedback is simple:
  - identify which agents had conflicts
  - re-solve only those agents' sub-problems with different random seeds
  - re-run MAPF on full set with updated paths

Returns a PipelineResult with the final collision-free timed paths
ready for visualization.
"""

from __future__ import annotations
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from vrp_rpd.agv_testbed.grid_env import WarehouseGrid, load_bays29_grid, load_dataset_grid, bfs_path
from vrp_rpd.agv_testbed.vrp_solver import SolverResult, run_heuristics
from vrp_rpd.agv_testbed.mapf_solver import MAPFResult, TimedPath, solve_mapf, detect_conflicts
from vrp_rpd.agv_testbed.instance_builder import tours_to_grid_paths


@dataclass
class PipelineResult:
    vrp_result: SolverResult
    mapf_result: MAPFResult
    iterations: int
    converged: bool

    def summary(self):
        print(f"\n{'='*60}")
        print(f"PIPELINE RESULT")
        print(f"  VRP makespan    : {self.vrp_result.makespan:.1f}")
        print(f"  MAPF success    : {self.mapf_result.success}")
        print(f"  Iterations      : {self.iterations}")
        print(f"  Converged       : {self.converged}")
        print(f"  Final max t     : {self.mapf_result.max_timestep()}")
        print(f"  Remaining conf. : {len(self.mapf_result.conflicts)}")
        print(f"{'='*60}\n")


def run_pipeline(
    grid: WarehouseGrid,
    num_agents: int = 3,
    resources_per_agent: int = 5,
    max_iterations: int = 5,
    seed: int = 42,
    dataset_dir: Optional[Path] = None,
    variant: str = "base",
) -> PipelineResult:
    """
    Run VRP-RPD then MAPF iteratively until convergence.

    The grid passed in is the initial placement. On conflict, the seed is
    incremented and a new placement is generated from dataset_dir/variant.
    If dataset_dir is None, the same grid is reused across iterations
    (only the VRP solver will produce different routes via heuristic order).
    """
    vrp_result: Optional[SolverResult] = None
    mapf_result: Optional[MAPFResult] = None
    converged = False
    current_grid = grid

    for iteration in range(1, max_iterations + 1):
        print(f"\n--- Iteration {iteration}/{max_iterations} (seed={seed}) ---")

        # Layer 1: VRP-RPD — use current grid placement
        vrp_result = run_heuristics(
            current_grid,
            num_agents=num_agents,
            resources_per_agent=resources_per_agent,
        )
        vrp_result.summary()

        # Layer 2: MAPF — expand visit sequences to include spur traversals
        visit_seqs = tours_to_grid_paths(
            {aid: plan.events for aid, plan in vrp_result.agents.items()},
            current_grid,
        )
        priority = vrp_result.priority_order()

        print(f"Running MAPF (priority: {priority})...")
        mapf_result = solve_mapf(
            visit_seqs, priority,
            spur_adj=current_grid.spur_adjacency(),
            workstation_ids=set(current_grid.workstation_ids),
        )

        n_conf = len(mapf_result.conflicts)
        print(f"MAPF: success={mapf_result.success}, conflicts={n_conf}, "
              f"max_t={mapf_result.max_timestep()}")

        if mapf_result.success:
            converged = True
            print("Converged — collision-free solution found.")
            break

        # Feedback: log conflicting agents, try new seed
        conflicting = set()
        for c in mapf_result.conflicts:
            conflicting.add(c.agent_i)
            conflicting.add(c.agent_j)
        seed += 1
        print(f"Conflicting agents: {sorted(conflicting)} — retrying with seed={seed}")
        if dataset_dir is not None:
            current_grid = load_dataset_grid(dataset_dir, variant=variant, seed=seed)
        # else: reuse same grid, VRP heuristics will differ by tie-breaking

    return PipelineResult(
        vrp_result=vrp_result,
        mapf_result=mapf_result,
        iterations=iteration,
        converged=converged,
    )


# ---------------------------------------------------------------------------
# Quick test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    base = Path(__file__).parent.parent / "vrp_rpd" / "datasets" / "bays29"
    grid = load_bays29_grid(base, variant="base", seed=42)

    result = run_pipeline(grid, num_agents=3, resources_per_agent=5, max_iterations=5)
    result.summary()

    # Print timed paths per agent
    for aid, tp in sorted(result.mapf_result.paths.items()):
        ct = result.vrp_result.agents[aid].completion_time
        print(f"Agent {aid} (VRP done@{ct:.0f}): "
              f"{len(tp.path)} steps, MAPF done@t={tp.path[-1][1] if tp.path else 0}")
        # Show first few steps
        preview = tp.path[:8]
        print(f"  Path start: {preview}{'...' if len(tp.path) > 8 else ''}")
