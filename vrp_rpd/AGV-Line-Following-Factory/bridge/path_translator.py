"""
Path Translator
---------------
Converts agv_testbed pipeline output (saved JSON from web_viewer --save)
into a factory node dispatch payload for physical Alvik AGVs.

Workstations are mapped to their spur_transit_nodes (grid intersections)
so the factory node can route between them using Manhattan paths on the
physical 8x8 tape grid.

Row convention:
  agv_testbed  : row 0 = bottom-left (geometric)
  factory node : row 0 = top-left   (screen/matrix)
  Conversion   : factory_row = ROWS - 1 - agv_row
"""

from __future__ import annotations
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Import grid_env directly to avoid triggering vrp_rpd/__init__.py (solver deps)
sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'vrp_rpd' / 'agv_testbed'))

from grid_env import (
    WarehouseGrid, build_distance_matrix,
    node_rc, N_NODES, ROWS, COLS, DEPOT_NODE,
)


def _to_factory_rc(agv_r: int, c: int) -> List[int]:
    """agv_testbed row → factory node row (flipped)."""
    return [ROWS - 1 - agv_r, c]


def build_factory_payload(pipeline_json_path: str) -> dict:
    """
    Read a saved pipeline JSON and return a factory node dispatch payload.

    Keys in the returned dict:
      routes           : list of per-AGV stop lists  [{node, op}, ...]
      depot_rc         : [factory_row, col]
      node_rcs         : [[factory_row, col], ...] indexed by node ID
      dwell_time       : 0.0  (D/P gating is done per-node, not by fixed dwell)
      processing_times : {str(node_id): seconds}  keyed by workstation node ID
    """
    with open(pipeline_json_path) as f:
        data = json.load(f)

    # Reconstruct WarehouseGrid so we have spur_transit_nodes + workstation_ids
    grid = WarehouseGrid(
        cell_nodes=data['cell_nodes'],
        processing_times=data['processing_times'],
        dist=build_distance_matrix(),
    )
    n_ws = len(grid.workstation_ids)

    # ── node_rcs ────────────────────────────────────────────────────────────
    # Must be indexable by every node_id that appears in any route.
    # Workstation IDs run up to N_NODES + 2*n_ws - 1.
    table_size = N_NODES + 2 * n_ws
    node_rcs: List[List[int]] = [[0, 0] for _ in range(table_size)]

    # Transit nodes: straightforward conversion
    for n in range(N_NODES):
        r, c = node_rc(n)
        node_rcs[n] = _to_factory_rc(r, c)

    # Workstation nodes: placed at their spur_transit_node position so the
    # factory node's Manhattan router reaches the correct grid intersection.
    for idx, ws_id in enumerate(grid.workstation_ids):
        r, c = node_rc(grid.spur_transit_nodes[idx])
        node_rcs[ws_id] = _to_factory_rc(r, c)

    # ── depot ───────────────────────────────────────────────────────────────
    depot_r, depot_c = node_rc(DEPOT_NODE)
    depot_frc = _to_factory_rc(depot_r, depot_c)

    # ── routes ──────────────────────────────────────────────────────────────
    # Each event is [ws_node_id, 'D'|'P', resource_id]
    routes: List[List[dict]] = []
    for aid_str in sorted(data['agents'].keys(), key=int):
        events = data['agents'][aid_str]['events']
        routes.append([{'node': e[0], 'op': e[1]} for e in events])

    # ── processing times ────────────────────────────────────────────────────
    # Keyed by workstation node ID (as string for JSON compatibility)
    proc_times: Dict[str, float] = {
        node_str: float(ct['processing_time'])
        for node_str, ct in data['customer_timing'].items()
    }

    return {
        'routes':           routes,
        'depot_rc':         depot_frc,
        'node_rcs':         node_rcs,
        'dwell_time':       0.0,
        'processing_times': proc_times,
    }
