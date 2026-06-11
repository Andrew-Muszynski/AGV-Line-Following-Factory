"""
Prioritized MAPF Solver
------------------------
Given agent visit sequences (grid node lists) from VRP-RPD, computes
collision-free time-expanded paths using prioritized A*.

Node ID scheme:
  0–99         : transit grid nodes (freely navigable)
  100–(100+n-1): spur entry virtual nodes
  (100+n)–...  : workstation virtual nodes

A* moves along transit edges (neighbors()) for transit nodes and along
spur edges (spur_adj) for virtual nodes.  No filtering needed — virtual
nodes simply aren't connected to the transit grid except via spur edges.

Priority ordering: highest completion time = most critical = planned first.
"""

from __future__ import annotations
import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from vrp_rpd.agv_testbed.grid_env import neighbors, N_NODES, DEPOT_NODE


# ---------------------------------------------------------------------------
# Time-expanded path representation
# ---------------------------------------------------------------------------

@dataclass
class TimedPath:
    agent_id: int
    path: List[Tuple[int, int]]   # [(node, timestep), ...]

    def node_at(self, t: int) -> Optional[int]:
        if not self.path:
            return None
        if t >= len(self.path):
            return self.path[-1][0]
        return self.path[t][0]

    def edge_at(self, t: int) -> Optional[Tuple[int, int]]:
        if t + 1 >= len(self.path):
            return None
        u = self.path[t][0]
        v = self.path[t + 1][0]
        return None if u == v else (u, v)


@dataclass
class Conflict:
    agent_i: int
    agent_j: int
    kind: str              # 'vertex' or 'edge'
    location: object
    timestep: int

    def __repr__(self):
        return (f"Conflict({self.kind} agents {self.agent_i}&{self.agent_j} "
                f"@ {self.location} t={self.timestep})")


@dataclass
class MAPFResult:
    paths: Dict[int, TimedPath]
    conflicts: List[Conflict]
    success: bool

    def max_timestep(self) -> int:
        return max(
            (p.path[-1][1] if p.path else 0) for p in self.paths.values()
        )


# ---------------------------------------------------------------------------
# Reservation table
# ---------------------------------------------------------------------------

class ReservationTable:
    def __init__(self):
        self._nodes: Set[Tuple[int, int]] = set()
        self._edges: Set[Tuple[int, int, int]] = set()

    def reserve_path(self, path: TimedPath):
        for node, t in path.path:
            if node == DEPOT_NODE:
                continue   # depot is shared staging area — never block it
            self._nodes.add((node, t))
        for i in range(len(path.path) - 1):
            u, t = path.path[i]
            v, _ = path.path[i + 1]
            if u != v:
                self._edges.add((u, v, t))
                self._edges.add((v, u, t))

    def is_node_free(self, node: int, t: int) -> bool:
        return (node, t) not in self._nodes

    def is_edge_free(self, u: int, v: int, t: int) -> bool:
        return (u, v, t) not in self._edges


# ---------------------------------------------------------------------------
# A* on time-expanded graph
# ---------------------------------------------------------------------------

def _node_heuristic(node: int, dst: int, spur_adj: Dict[int, List[int]]) -> int:
    """
    Manhattan distance heuristic.  Virtual nodes (spur entry / workstation)
    inherit the grid position of their transit anchor for heuristic purposes.
    """
    def grid_pos(n):
        if n < N_NODES:
            return n // 10, n % 10
        # For virtual nodes find the transit node in spur_adj and use its position
        for nb in spur_adj.get(n, []):
            if nb < N_NODES:
                return nb // 10, nb % 10
        return 0, 0

    r1, c1 = grid_pos(node)
    r2, c2 = grid_pos(dst)
    return abs(r1 - r2) + abs(c1 - c2)


def _astar_segment(
    src: int,
    dst: int,
    reservations: ReservationTable,
    spur_adj: Dict[int, List[int]],
    workstation_ids: Set[int],
    start_t: int,
    max_t: int,
) -> List[Tuple[int, int]]:
    """
    A* from src to dst starting at start_t.
    Transit nodes (< N_NODES) use neighbors() for movement.
    Virtual nodes (>= N_NODES) use spur_adj only.
    """
    if src == dst:
        return [(src, start_t)]

    def h(n):
        return _node_heuristic(n, dst, spur_adj)

    came_from: List[Optional[int]] = [None]
    node_t_record: List[Tuple[int, int]] = [(src, start_t)]
    # src is where this agent already is — don't treat it as blocked even if reserved
    open_heap = [(h(src), 0, src, start_t, 0)]
    visited: Set[Tuple[int, int]] = set()
    own_start = (src, start_t)   # agent owns this slot unconditionally

    while open_heap:
        f, g, node, t, idx = heapq.heappop(open_heap)

        if (node, t) in visited:
            continue
        visited.add((node, t))

        if node == dst:
            path, cur = [], idx
            while cur is not None:
                path.append(node_t_record[cur])
                cur = came_from[cur]
            return path[::-1]

        if t >= max_t:
            continue

        # Neighbours:
        #   transit nodes (< N_NODES) : grid neighbors + spur entry connections
        #   spur entry nodes           : their transit anchor + their workstation
        #   workstation nodes          : only their spur entry (and only if dst==workstation)
        if node in workstation_ids and node != dst and node != src:
            # Arrived at a workstation that is neither our destination nor start — dead end
            continue

        if node < N_NODES:
            all_nb = neighbors(node) + spur_adj.get(node, [])
        else:
            all_nb = spur_adj.get(node, [])

        for nb in all_nb:
            # Never pass through a workstation unless it is the destination
            if nb in workstation_ids and nb != dst:
                continue
            nt = t + 1
            if (nb, nt) in visited:
                continue
            if not reservations.is_node_free(nb, nt):
                continue
            if not reservations.is_edge_free(node, nb, t):
                continue
            new_idx = len(came_from)
            came_from.append(idx)
            node_t_record.append((nb, nt))
            heapq.heappush(open_heap, (nt + h(nb), nt, nb, nt, new_idx))

        # Wait action — only at transit nodes and spur entries, not inside workstations.
        # Always allow waiting at the segment's starting position (agent already owns it).
        if node not in workstation_ids:
            wt = t + 1
            node_is_free = reservations.is_node_free(node, wt) or (node, t) == own_start
            if (node, wt) not in visited and node_is_free:
                new_idx = len(came_from)
                came_from.append(idx)
                node_t_record.append((node, wt))
                heapq.heappush(open_heap, (wt + h(node), wt, node, wt, new_idx))

    return []


def _astar_timed(
    waypoints: List[int],
    reservations: ReservationTable,
    spur_adj: Dict[int, List[int]],
    workstation_ids: Set[int],
    start_t: int = 0,
    max_t: int = 2000,
) -> List[Tuple[int, int]]:
    """A* through a sequence of waypoints, respecting reservations."""
    full_path: List[Tuple[int, int]] = []
    t_offset = start_t

    for seg in range(len(waypoints) - 1):
        src = waypoints[seg]
        dst = waypoints[seg + 1]
        # src must match where the path actually is
        actual_src = full_path[-1][0] if full_path else src
        seg_path = _astar_segment(actual_src, dst, reservations, spur_adj, workstation_ids, t_offset, max_t)
        if not seg_path:
            # A* failed — stay at actual_src, signal failure by returning what we have
            break
        if full_path:
            full_path.extend(seg_path[1:])
        else:
            full_path.extend(seg_path)
        t_offset = full_path[-1][1]

    return full_path


# ---------------------------------------------------------------------------
# Conflict detection
# ---------------------------------------------------------------------------

def detect_conflicts(paths: Dict[int, TimedPath], depot: int = DEPOT_NODE) -> List[Conflict]:
    conflicts = []
    agent_ids = sorted(paths.keys())
    max_t = max(p.path[-1][1] if p.path else 0 for p in paths.values())

    for t in range(max_t + 1):
        node_occ: Dict[int, int] = {}
        for aid in agent_ids:
            n = paths[aid].node_at(t)
            if n is None:
                continue
            if n == depot:
                continue   # depot is shared staging — overlaps always allowed
            if n in node_occ:
                conflicts.append(Conflict(
                    agent_i=node_occ[n], agent_j=aid,
                    kind='vertex', location=n, timestep=t,
                ))
            else:
                node_occ[n] = aid

        for i, ai in enumerate(agent_ids):
            for aj in agent_ids[i + 1:]:
                e_i = paths[ai].edge_at(t)
                e_j = paths[aj].edge_at(t)
                if e_i and e_j and e_i == (e_j[1], e_j[0]):
                    conflicts.append(Conflict(
                        agent_i=ai, agent_j=aj,
                        kind='edge', location=e_i, timestep=t,
                    ))

    return conflicts


# ---------------------------------------------------------------------------
# Main MAPF solver
# ---------------------------------------------------------------------------

def solve_mapf(
    visit_sequences: Dict[int, List[int]],
    priority_order: List[int],
    spur_adj: Dict[int, List[int]] = None,
    workstation_ids: Set[int] = None,
    max_t: int = 2000,
) -> MAPFResult:
    """
    Prioritized A* MAPF.

    Parameters
    ----------
    visit_sequences : {agent_id: [node0, node1, ..., nodeN]}
                      Waypoints include virtual spur entry and workstation IDs
                      (expanded by tours_to_grid_paths in instance_builder).
    priority_order  : agent ids sorted highest priority first.
    spur_adj        : adjacency dict from WarehouseGrid.spur_adjacency().
    max_t           : maximum timestep to search.
    """
    if spur_adj is None:
        spur_adj = {}
    ws_set: Set[int] = workstation_ids if workstation_ids is not None else set()

    reservations = ReservationTable()
    paths: Dict[int, TimedPath] = {}

    for agent_id in priority_order:
        waypoints = visit_sequences.get(agent_id, [DEPOT_NODE, DEPOT_NODE])
        timed = _astar_timed(waypoints, reservations, spur_adj, ws_set, start_t=0, max_t=max_t)
        tp = TimedPath(agent_id=agent_id, path=timed)
        reservations.reserve_path(tp)
        paths[agent_id] = tp

    for aid in visit_sequences:
        if aid not in paths:
            paths[aid] = TimedPath(agent_id=aid, path=[])

    conflicts = detect_conflicts(paths)
    return MAPFResult(paths=paths, conflicts=conflicts, success=len(conflicts) == 0)
