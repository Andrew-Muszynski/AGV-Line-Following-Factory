"""
Instance Builder
----------------
Converts a WarehouseGrid into a VRPRPDInstance that the existing solver
understands, and converts solver output back into waypoint sequences for MAPF.

Solver index mapping:
  solver index 0   -> depot (transit node 0)
  solver index i   -> workstation i-1 (virtual workstation ID)

MAPF waypoint expansion:
  Each workstation visit becomes: spur_entry -> workstation -> spur_entry
  so MAPF sees the full physical spur traversal.
"""

from __future__ import annotations
import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "vrp_rpd"))
from vrp_rpd import VRPRPDInstance

from vrp_rpd.agv_testbed.grid_env import WarehouseGrid, load_bays29_grid, node_rc


def build_vrp_instance(
    grid: WarehouseGrid,
    num_agents: int = 3,
    resources_per_agent: int = 5,
) -> VRPRPDInstance:
    """Build a VRPRPDInstance from a WarehouseGrid."""
    D = grid.solver_distance_matrix()   # (n+1)×(n+1)

    n = len(grid.cell_nodes)
    proc = np.zeros(n + 1, dtype=np.float64)
    proc[1:] = grid.processing_times

    return VRPRPDInstance(
        distance_matrix=D,
        processing_times=proc,
        num_agents=num_agents,
        resources_per_agent=resources_per_agent,
        depot=0,
        coordinates=None,
    )


def solver_index_to_ws_id(solver_idx: int, grid: WarehouseGrid) -> int:
    """
    Convert a solver node index to:
      0        -> depot transit node
      i (>=1)  -> virtual workstation ID for workstation i-1
    """
    if solver_idx == 0:
        return grid.depot
    return grid.workstation_ids[solver_idx - 1]


def tours_to_grid_paths(tours: dict, grid: WarehouseGrid) -> dict:
    """
    Expand agent events into MAPF waypoint sequences.

    tours format : {agent_id: [(grid_node_or_ws_id, op, solver_idx), ...]}
                   (plan.events from AgentPlan — first element is already
                    the virtual workstation ID or depot)

    Each workstation visit is expanded to:
      spur_entry_id -> workstation_id -> spur_entry_id
    so MAPF routes through the spur physically.

    Returns {agent_id: [node_id, ...]} including depot at start and end.
    """
    # Build ws_id -> spur_entry_id lookup
    ws_to_entry = dict(zip(grid.workstation_ids, grid.spur_entry_ids))

    grid_tours = {}
    for agent, events in tours.items():
        path = [grid.depot]
        for (node, op, _solver_idx) in events:
            if node == grid.depot:
                path.append(node)
            else:
                entry = ws_to_entry[node]
                path.append(entry)
                path.append(node)
                path.append(entry)
        path.append(grid.depot)
        grid_tours[agent] = path
    return grid_tours


# ---------------------------------------------------------------------------
# Quick test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    from vrp_rpd.agv_testbed.grid_env import load_dataset_grid
    grid = load_dataset_grid("bays29", variant="base", seed=42)
    inst = build_vrp_instance(grid, num_agents=4, resources_per_agent=6)

    print(f"VRPRPDInstance built:")
    print(f"  n (total nodes incl depot) : {inst.n}")
    print(f"  num_customers              : {inst.num_customers}")
    print(f"  num_agents                 : {inst.m}")
    print(f"  resources_per_agent        : {inst.k}")
    print(f"  distance matrix shape      : {inst.dist.shape}")
    print(f"  processing times (first 5) : {inst.proc[1:6]}")

    # Show virtual ID scheme
    n = len(grid.cell_nodes)
    print(f"\nVirtual ID scheme (n={n} workstations):")
    print(f"  Transit nodes  : 0–99")
    print(f"  Spur entries   : {grid.spur_entry_ids[0]}–{grid.spur_entry_ids[-1]}")
    print(f"  Workstations   : {grid.workstation_ids[0]}–{grid.workstation_ids[-1]}")
    print(f"  Cell nodes (first 5): {grid.cell_nodes[:5]}")
    print(f"  Spur transit   (first 5): {grid.spur_transit_nodes[:5]}")
