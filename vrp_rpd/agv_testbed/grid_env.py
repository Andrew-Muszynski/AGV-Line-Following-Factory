"""
Warehouse Grid Environment
--------------------------
8x8 transit grid = 64 nodes (IDs 0–63), all freely navigable.
Node (r, c) has integer id  r * COLS + c.
Depot is always node 0 = (row=0, col=0) = bottom-left corner.

Spur geometry
-------------
Workstations are physical installations inside cells — they do NOT occupy
transit grid nodes.  Each workstation gets two virtual node IDs outside 0–99:

  Spur entry  : ID = N_NODES + i          (i = workstation index)
  Workstation : ID = N_NODES + n_ws + i   (i = workstation index)

The spur entry sits at the top edge of the cell (connected to the transit
grid node directly above the cell's grid position with cost 1).
The workstation sits at the cell center (connected only to its spur entry
with cost SPUR_LEN).

This separation guarantees:
  - Transit graph (0–99) is always clean — robots route freely.
  - No ID collision even when workstations are in adjacent rows/columns.
  - Any grid node can host a workstation regardless of neighbours.
"""

from __future__ import annotations
import json
import numpy as np
from collections import deque
from pathlib import Path

ROWS    = 8
COLS    = 8
N_NODES = ROWS * COLS   # 64 transit nodes
DEPOT_NODE = 0          # (row=0, col=0) — bottom-left
SPUR_LEN   = 5          # half a cell — fixed spur length (transit→entry = 1, entry→ws = SPUR_LEN)


# ---------------------------------------------------------------------------
# Transit grid helpers  (operate on IDs 0–99 only)
# ---------------------------------------------------------------------------

def node_id(r: int, c: int) -> int:
    return r * COLS + c

def node_rc(nid: int):
    """Row, col for a transit node (0–99)."""
    return nid // COLS, nid % COLS

def neighbors(nid: int) -> list[int]:
    """4-connected transit neighbours of a transit node."""
    r, c = node_rc(nid)
    result = []
    if r > 0:        result.append(node_id(r - 1, c))
    if r < ROWS - 1: result.append(node_id(r + 1, c))
    if c > 0:        result.append(node_id(r, c - 1))
    if c < COLS - 1: result.append(node_id(r, c + 1))
    return result


# ---------------------------------------------------------------------------
# BFS on transit grid
# ---------------------------------------------------------------------------

def bfs_distances(source: int) -> np.ndarray:
    """BFS shortest-hop distances from source to all 100 transit nodes."""
    dist = np.full(N_NODES, -1, dtype=np.int32)
    dist[source] = 0
    q = deque([source])
    while q:
        u = q.popleft()
        for v in neighbors(u):
            if dist[v] == -1:
                dist[v] = dist[u] + 1
                q.append(v)
    return dist

def bfs_path(source: int, target: int) -> list[int]:
    """Shortest transit-grid path from source to target (node IDs)."""
    if source == target:
        return [source]
    parent = {source: None}
    q = deque([source])
    while q:
        u = q.popleft()
        for v in neighbors(u):
            if v not in parent:
                parent[v] = u
                if v == target:
                    path, cur = [], v
                    while cur is not None:
                        path.append(cur)
                        cur = parent[cur]
                    return path[::-1]
                q.append(v)
    return []

def build_distance_matrix() -> np.ndarray:
    """100×100 BFS shortest-path distance matrix for transit nodes."""
    D = np.zeros((N_NODES, N_NODES), dtype=np.int32)
    for src in range(N_NODES):
        D[src] = bfs_distances(src)
    return D


# ---------------------------------------------------------------------------
# Grid environment
# ---------------------------------------------------------------------------

class WarehouseGrid:
    """
    Warehouse with spur geometry.

    Virtual node ID scheme (given n workstations):
      Transit nodes  :   0 .. N_NODES-1          (the 8×8 grid)
      Spur entries   :   N_NODES .. N_NODES+n-1
      Workstations   :   N_NODES+n .. N_NODES+2n-1

    Attributes
    ----------
    dist            : (100,100) BFS transit distances
    depot           : int — depot transit node (always 0)
    cell_nodes      : list[int] — transit grid node each workstation is mapped to
                      (the grid node whose cell contains the workstation)
    spur_entry_ids  : list[int] — virtual spur entry node IDs
    workstation_ids : list[int] — virtual workstation node IDs
    processing_times: list[float]
    spur_transit_nodes : list[int] — transit node each spur entry connects to
                         (grid node directly above the cell_node)
    """

    def __init__(
        self,
        cell_nodes: list[int],
        processing_times: list[float],
        dist: np.ndarray | None = None,
    ):
        assert len(cell_nodes) == len(processing_times)
        assert DEPOT_NODE not in cell_nodes, "depot cannot host a workstation"

        n = len(cell_nodes)
        self.depot            = DEPOT_NODE
        self.cell_nodes       = list(cell_nodes)          # transit grid positions
        self.processing_times = list(processing_times)
        self.dist             = dist if dist is not None else build_distance_matrix()

        # Virtual IDs
        self.spur_entry_ids  = [N_NODES + i       for i in range(n)]
        self.workstation_ids = [N_NODES + n + i   for i in range(n)]

        # Each spur entry connects to the transit node directly above the cell node.
        # If cell_node is in the top row, use the cell_node itself (spur stays within row).
        self.spur_transit_nodes = []
        for g in cell_nodes:
            r, c = node_rc(g)
            above = node_id(min(r + 1, ROWS - 1), c)
            self.spur_transit_nodes.append(above)

        # Convenience: sets for fast membership tests
        self._spur_entry_set  = set(self.spur_entry_ids)
        self._workstation_set = set(self.workstation_ids)

        # Backward-compat alias so existing code using .workstations still works
        self.workstations  = self.workstation_ids
        self.spur_entries  = self.spur_entry_ids

    # ------------------------------------------------------------------
    # Spur adjacency dict for MAPF A*
    # ------------------------------------------------------------------

    def spur_adjacency(self) -> dict[int, list[int]]:
        """
        Adjacency dict for all spur edges (bidirectional):
          transit_node  <-> spur_entry   (cost 1 hop)
          spur_entry    <-> workstation  (cost SPUR_LEN hops, but modelled as 1 step here;
                                          the distance matrix handles the cost for VRP)
        Returns {node_id: [neighbour_id, ...]} for virtual nodes only.
        Transit↔transit edges are handled by neighbors() separately.
        """
        adj: dict[int, list[int]] = {}
        for entry_id, ws_id, transit_id in zip(
            self.spur_entry_ids, self.workstation_ids, self.spur_transit_nodes
        ):
            # transit <-> spur entry
            adj.setdefault(transit_id, []).append(entry_id)
            adj.setdefault(entry_id,   []).append(transit_id)
            # spur entry <-> workstation
            adj.setdefault(entry_id, []).append(ws_id)
            adj.setdefault(ws_id,    []).append(entry_id)
        return adj

    def is_workstation(self, node_id: int) -> bool:
        return node_id in self._workstation_set

    def is_spur_entry(self, node_id: int) -> bool:
        return node_id in self._spur_entry_set

    # ------------------------------------------------------------------
    # Distance matrix for the VRP-RPD solver
    # rows/cols: index 0 = depot, indices 1..n = workstations
    # ------------------------------------------------------------------

    def solver_distance_matrix(self) -> np.ndarray:
        """
        (n+1)×(n+1) distance matrix.  Index 0 = depot, 1..n = workstations.

        All transit distances are BFS hops.  Visiting a workstation adds:
          +1       : transit → spur_entry
          +SPUR_LEN: spur_entry → workstation center
        So each workstation visit adds (1 + SPUR_LEN) each way.
        """
        n = len(self.cell_nodes)
        size = n + 1
        D = np.zeros((size, size), dtype=np.float64)
        spur_cost = 1 + SPUR_LEN   # transit→entry + entry→ws

        for i in range(size):
            for j in range(size):
                if i == j:
                    continue
                # Map solver index to transit anchor node
                u = self.depot          if i == 0 else self.spur_transit_nodes[i - 1]
                v = self.depot          if j == 0 else self.spur_transit_nodes[j - 1]
                grid_dist = float(self.dist[u, v])
                cost_i = 0 if i == 0 else spur_cost
                cost_j = 0 if j == 0 else spur_cost
                D[i, j] = grid_dist + cost_i + cost_j
        return D

    def __repr__(self):
        return (
            f"WarehouseGrid(depot={self.depot}, "
            f"n_workstations={len(self.cell_nodes)}, "
            f"grid={ROWS}x{COLS})"
        )


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------

def _load_proc_times(jobs_file: Path) -> list[float]:
    with open(jobs_file) as f:
        data = json.load(f)
    return [float(t) for t in (data.get("processing_times") or data["job_times"])]


def _place_cells(n: int, rng: np.random.Generator) -> list[int]:
    """
    Choose n distinct transit nodes to host workstations.
    Exclude:
      - depot (node 0)
      - top row (row ROWS-1): no row above for the spur transit node
      - right column (col COLS-1): no column to the right for the cell center offset
    """
    candidates = [
        node_id(r, c)
        for r in range(ROWS - 1)      # exclude top row
        for c in range(COLS - 1)      # exclude right column
        if node_id(r, c) != DEPOT_NODE
    ]
    chosen = rng.choice(candidates, size=n, replace=False).tolist()
    return sorted(chosen)


# ---------------------------------------------------------------------------
# Factory: build grid from a bays29-style dataset
# ---------------------------------------------------------------------------

def load_bays29_grid(
    dataset_dir: str | Path,
    variant: str = "base",
    seed: int = 42,
    time_scale: float = 1.0,
) -> WarehouseGrid:
    dataset_dir = Path(dataset_dir)
    jobs_file   = dataset_dir / variant / "job_times.json"
    with open(jobs_file) as f:
        data = json.load(f)
    proc_times  = [t * time_scale for t in data["processing_times"]]
    rng         = np.random.default_rng(seed)
    cell_nodes  = _place_cells(len(proc_times), rng)
    return WarehouseGrid(cell_nodes=cell_nodes, processing_times=proc_times,
                         dist=build_distance_matrix())


# ---------------------------------------------------------------------------
# Generic factory: works for any dataset folder
# ---------------------------------------------------------------------------

AGV_DATASETS_ROOT = Path(__file__).parent / "datasets"


def load_dataset_grid(
    dataset_name: str | Path,
    variant: str = "base",
    seed: int = 42,
    instance: int = 1,
) -> WarehouseGrid:
    p = Path(dataset_name)
    dataset_dir = p if p.is_dir() else AGV_DATASETS_ROOT / str(dataset_name)
    variant_dir = dataset_dir / variant

    if variant in ("1R10", "1R20"):
        jobs_file = variant_dir / f"job_times_{instance}.json"
    else:
        jobs_file = variant_dir / "job_times.json"

    if not jobs_file.exists():
        raise FileNotFoundError(
            f"Dataset file not found: {jobs_file}\n"
            f"Run: python3 agv_testbed/generate_datasets.py"
        )

    proc_times = _load_proc_times(jobs_file)
    rng        = np.random.default_rng(seed)
    cell_nodes = _place_cells(len(proc_times), rng)
    return WarehouseGrid(cell_nodes=cell_nodes, processing_times=proc_times,
                         dist=build_distance_matrix())


# ---------------------------------------------------------------------------
# Quick sanity check
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    grid = load_dataset_grid("bays29", variant="base", seed=42)
    print(grid)
    n = len(grid.cell_nodes)
    print(f"Cell nodes (first 5)      : {grid.cell_nodes[:5]}")
    print(f"Spur entry IDs (first 5)  : {grid.spur_entry_ids[:5]}")
    print(f"Workstation IDs (first 5) : {grid.workstation_ids[:5]}")
    print(f"Spur transit nodes (first 5): {grid.spur_transit_nodes[:5]}")
    D = grid.solver_distance_matrix()
    print(f"Solver dist matrix shape  : {D.shape}")
    print(f"depot->ws[0]: {D[0,1]:.0f}  "
          f"(bfs={grid.dist[grid.depot, grid.spur_transit_nodes[0]]} + {1+SPUR_LEN})")
    print(f"ws[0]->ws[1]: {D[1,2]:.0f}  "
          f"(bfs={grid.dist[grid.spur_transit_nodes[0], grid.spur_transit_nodes[1]]} + {2*(1+SPUR_LEN)})")
    adj = grid.spur_adjacency()
    print(f"Spur adj sample: entry {grid.spur_entry_ids[0]} -> {adj[grid.spur_entry_ids[0]]}")
