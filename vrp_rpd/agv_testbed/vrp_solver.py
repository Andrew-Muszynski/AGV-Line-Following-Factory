"""
VRP-RPD Solver Pipeline for Grid Testbed
-----------------------------------------
Takes a WarehouseGrid, runs heuristics (nearest-neighbor, max-regret,
greedy-defer) via the existing solver, and returns a SolverResult with:
  - per-agent grid-node visit sequences
  - per-agent completion times
  - per-customer dropoff/pickup times
"""

from __future__ import annotations
import sys
import json
import time
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).parent.parent / "vrp_rpd"))
from vrp_rpd import (
    VRPRPDInstance,
    generate_nearest_neighbor_solution,
    generate_max_regret_solution,
    generate_greedy_defer_solution,
    decode_chromosome,
    compute_makespan_from_tours,
)
from vrp_rpd.utils import simulate_solution

from vrp_rpd.agv_testbed.grid_env import WarehouseGrid, load_bays29_grid, bfs_path
from vrp_rpd.agv_testbed.instance_builder import build_vrp_instance, solver_index_to_ws_id


@dataclass
class AgentPlan:
    agent_id: int
    # Ordered list of (grid_node, operation, solver_customer_idx)
    # operation: 'D' = dropoff, 'P' = pickup
    events: List[Tuple[int, str, int]] = field(default_factory=list)
    completion_time: float = 0.0

    @property
    def visit_sequence(self) -> List[int]:
        """Grid nodes visited in order, bookended by depot."""
        from vrp_rpd.agv_testbed.grid_env import DEPOT_NODE
        nodes = [DEPOT_NODE]
        for gnode, _op, _c in self.events:
            nodes.append(gnode)
        nodes.append(DEPOT_NODE)
        return nodes


@dataclass
class CustomerTiming:
    solver_idx: int      # solver index (1-based)
    grid_node: int       # grid node id
    dropoff_time: float
    pickup_time: float
    processing_time: float


@dataclass
class SolverResult:
    heuristic: str
    makespan: float
    agents: Dict[int, AgentPlan]          # agent_id -> AgentPlan
    customer_timing: Dict[int, CustomerTiming]  # grid_node -> CustomerTiming
    grid: WarehouseGrid
    instance: VRPRPDInstance

    def priority_order(self) -> List[int]:
        """Agent ids sorted by completion time descending (highest first = most critical)."""
        return sorted(
            self.agents.keys(),
            key=lambda a: self.agents[a].completion_time,
            reverse=True,
        )

    def summary(self):
        print(f"\n{'='*60}")
        print(f"Solver: {self.heuristic}  |  Makespan: {self.makespan:.1f}")
        print(f"Priority order (most critical first): {self.priority_order()}")
        for aid, plan in sorted(self.agents.items()):
            n_drop = sum(1 for _, op, _ in plan.events if op == 'D')
            n_pick = sum(1 for _, op, _ in plan.events if op == 'P')
            print(f"  Agent {aid}: {n_drop} dropoffs, {n_pick} pickups  "
                  f"| done @ {plan.completion_time:.1f}  "
                  f"| visits: {plan.visit_sequence}")
        print(f"{'='*60}\n")


def _build_result(
    heuristic_name: str,
    tours: dict,
    instance: VRPRPDInstance,
    grid: WarehouseGrid,
) -> SolverResult:
    """Convert raw solver tours into a SolverResult."""

    # Simulate to get timing info
    job_times, agent_tours, agent_completion, customer_assignment = simulate_solution(
        tours, instance
    )

    # Makespan = max completion time
    makespan = max(agent_completion.values()) if agent_completion else 0.0

    # Build per-agent plans
    agents: Dict[int, AgentPlan] = {}
    for a in range(instance.m):
        plan = AgentPlan(agent_id=a, completion_time=float(agent_completion.get(a, 0.0)))
        raw_events = agent_tours.get(a, [])
        for solver_node, op in raw_events:
            ws_id = solver_index_to_ws_id(solver_node, grid)
            plan.events.append((ws_id, op, solver_node))
        agents[a] = plan

    # Build per-customer timing (keyed by grid node)
    customer_timing: Dict[int, CustomerTiming] = {}
    for solver_node, jt in job_times.items():
        ws_id  = solver_index_to_ws_id(solver_node, grid)
        proc_t = instance.proc[solver_node]
        customer_timing[ws_id] = CustomerTiming(
            solver_idx=int(solver_node),
            grid_node=int(ws_id),
            dropoff_time=float(jt.get('dropoff', 0.0)),
            pickup_time=float(jt.get('pickup', jt.get('end', 0.0))),
            processing_time=float(proc_t),
        )

    return SolverResult(
        heuristic=heuristic_name,
        makespan=makespan,
        agents=agents,
        customer_timing=customer_timing,
        grid=grid,
        instance=instance,
    )


def run_heuristics(
    grid: WarehouseGrid,
    num_agents: int = 3,
    resources_per_agent: int = 5,
    allow_mixed: bool = True,
) -> SolverResult:
    """
    Run all three construction heuristics and return the best result.
    """
    instance = build_vrp_instance(grid, num_agents, resources_per_agent)
    dist = instance.dist
    proc = instance.proc
    depot = instance.depot
    m = instance.m
    k = instance.k
    n_cust = instance.num_customers

    candidates = []

    print("Running Nearest Neighbor heuristic...")
    try:
        chrom, _, tours = generate_nearest_neighbor_solution(
            dist, proc, depot, m, k, n_cust, allow_mixed=allow_mixed
        )
        r = _build_result("Nearest Neighbor", tours, instance, grid)
        candidates.append(r)
        print(f"  Makespan: {r.makespan:.1f}")
    except Exception as e:
        print(f"  Failed: {e}")

    print("Running Max Regret heuristic...")
    try:
        chrom, _, tours = generate_max_regret_solution(
            dist, proc, depot, m, k, n_cust, allow_mixed=allow_mixed
        )
        r = _build_result("Max Regret", tours, instance, grid)
        candidates.append(r)
        print(f"  Makespan: {r.makespan:.1f}")
    except Exception as e:
        print(f"  Failed: {e}")

    print("Running Greedy Defer heuristic...")
    try:
        chrom, _ = generate_greedy_defer_solution(
            dist, proc, depot, m, k, n_cust,
            defer_multiplier=10.0, allow_mixed=allow_mixed
        )
        tours = decode_chromosome(chrom, instance, allow_mixed=allow_mixed)
        r = _build_result("Greedy Defer", tours, instance, grid)
        candidates.append(r)
        print(f"  Makespan: {r.makespan:.1f}")
    except Exception as e:
        print(f"  Failed: {e}")

    if not candidates:
        raise RuntimeError("All heuristics failed — cannot produce a solution.")

    best = min(candidates, key=lambda r: r.makespan)
    print(f"\nBest heuristic: {best.heuristic}  (makespan={best.makespan:.1f})")
    return best


# ---------------------------------------------------------------------------
# Quick test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    base = Path(__file__).parent.parent / "vrp_rpd" / "datasets" / "bays29"
    grid = load_bays29_grid(base, variant="base", seed=42)

    result = run_heuristics(grid, num_agents=3, resources_per_agent=5)
    result.summary()

    print("Priority order (for MAPF):", result.priority_order())
    print("Customer timing (first 3):")
    for gnode, ct in list(result.customer_timing.items())[:3]:
        print(f"  grid node {gnode:3d}: drop@{ct.dropoff_time:.1f}  "
              f"pick@{ct.pickup_time:.1f}  proc={ct.processing_time:.1f}")
