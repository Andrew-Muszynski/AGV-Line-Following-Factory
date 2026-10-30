"""Single-robot planning: Safe Interval Path Planning, with heading.

Phillips & Likhachev (2011), ICRA: "SIPP: Safe Interval Path Planning for
Dynamic Environments". Time is continuous; instead of searching every
(node, time) pair, the search runs over (node, heading, safe interval) and
always takes the EARLIEST arrival in each interval -- which is optimal for
earliest-arrival because waiting is allowed inside a safe interval.

Heading is part of the state because turns cost time (2.65s / 5.00s) and an
about-face is banned except in a dead end; without it the planner would cut
corners the robots cannot.

Constraints come from conflicts.Constraints: intervals this robot's presence,
drives and turns must keep out of. A* uses none (its own path only),
prioritized planning uses every earlier robot's holds, CBS one branch's.
"""
from __future__ import annotations

import heapq
import math
import time

from .conflicts import EPS, Constraints, free_intervals, overlaps
from .instance import Instance, Task
from .model import Testbed
from .plan import TimedPath, time_path


class Budget:
    """Shared wall-clock / expansion budget so a whole method can be capped."""

    def __init__(self, seconds: float | None = None,
                 expansions: int | None = None):
        self.deadline = (time.monotonic() + seconds) if seconds else None
        self.left = expansions

    def spend(self, n: int = 1) -> bool:
        if self.left is not None:
            self.left -= n
            if self.left < 0:
                return False
        if self.deadline is not None and time.monotonic() > self.deadline:
            return False
        return True


class _Intervals:
    def __init__(self, cons: Constraints):
        self.cons = cons
        self._cache: dict = {}

    def of(self, node) -> list:
        hit = self._cache.get(node)
        if hit is None:
            hit = free_intervals(self.cons.node.get(node, []))
            self._cache[node] = hit
        return hit

    def index(self, node, t: float) -> int | None:
        for i, (lo, hi) in enumerate(self.of(node)):
            if lo - EPS <= t < hi - EPS:
                return i
        return None


def _first_block(blocked: list, a: float, b: float):
    for s, e in blocked:
        if s >= b - EPS:
            return None
        if overlaps(a, b, s, e):
            return s, e
    return None


def _earliest_departure(tb, cons, ivs, v, t, depart_min, hi_i, edge, turn,
                        lo_j, hi_j):
    """Earliest time to START turning at v toward `edge` so that the turn,
    the drive and the arrival all clear the constraints. None if the robot
    would have to stay at v past its safe interval, or arrive past hi_j."""
    m = edge.seconds
    tau = max(t, depart_min, lo_j - turn - m)
    turns = cons.turn.get(v, ())
    lanes = cons.lane.get(edge.corridor, ())
    for _ in range(256):
        if tau + turn > hi_i + EPS:
            return None
        if turn > 0:
            blk = _first_block(turns, tau, tau + turn)
            if blk is not None:
                tau = blk[1]
                continue
        blk = _first_block(lanes, tau + turn, tau + turn + m)
        if blk is not None:
            tau = blk[1] - turn
            continue
        ta = tau + turn + m
        if ta < lo_j - EPS:
            tau = lo_j - turn - m
            continue
        if ta >= hi_j - EPS:
            return None
        return tau
    return None


def plan_leg(tb: Testbed, cons: Constraints, ivs: _Intervals, start, goal,
             min_dwell: float, earliest_dep: float, final: bool,
             budget: Budget | None = None):
    """From start = (node, heading, arrive, depart_min) to `goal`.
    Returns (nodes, waits, arrive_at_goal, heading_at_goal) or None.
    waits[i] is held at nodes[i]; the goal's own wait is left to the caller."""
    v0, h0, t0, dmin0 = start
    i0 = ivs.index(v0, t0)
    if i0 is None:
        return None
    hmap = tb.time_to(goal)
    if v0 not in hmap:
        return None

    def goal_ok(node, t, iv):
        if node != goal:
            return False
        lo, hi = ivs.of(node)[iv]
        if final:
            return hi == math.inf
        return max(t + min_dwell, earliest_dep) < hi - EPS

    seq = 0
    start_key = (v0, h0, i0)
    best = {start_key: t0}
    parent: dict = {start_key: None}
    pq = [(t0 + hmap[v0], t0, seq, start_key)]
    while pq:
        f, t, _s, key = heapq.heappop(pq)
        if t > best.get(key, math.inf) + EPS:
            continue
        if budget is not None and not budget.spend():
            return None
        v, h, iv = key
        if goal_ok(v, t, iv):
            return _reconstruct(parent, key, best)
        hi_i = ivs.of(v)[iv][1]
        dmin = dmin0 if key == start_key else t
        entry_here = tb.kind(v) == "entry"
        for edge in tb.neighbours(v):
            if tb.reverses(h, edge.heading) and not tb.dead_end(v):
                continue
            if entry_here and h is not None:
                # An entry is a doorway, not a waypoint: arriving along the
                # lane you go INTO the bay; coming out you rejoin the lane.
                # Cutting through it (flank -> entry -> flank) would park a
                # robot on the entry mid-lane, which nothing else does.
                into_bay = tb.kind(edge.to) == "workstation"
                if (h in ("E", "W")) != into_bay:
                    continue
            u = edge.to
            if u not in hmap:
                continue
            turn = 0.0 if edge.depot else tb.turn_seconds(h, edge.heading)
            for j, (lo_j, hi_j) in enumerate(ivs.of(u)):
                if hi_j <= t + turn + edge.seconds + EPS:
                    continue
                tau = _earliest_departure(tb, cons, ivs, v, t, dmin, hi_i,
                                          edge, turn, lo_j, hi_j)
                if tau is None:
                    continue
                ta = tau + turn + edge.seconds
                nkey = (u, edge.heading, j)
                if ta < best.get(nkey, math.inf) - EPS:
                    best[nkey] = ta
                    parent[nkey] = (key, tau - t)
                    seq += 1
                    heapq.heappush(pq, (ta + hmap[u], ta, seq, nkey))
    return None


def _reconstruct(parent, key, best):
    chain = []
    k = key
    while k is not None:
        chain.append(k)
        p = parent[k]
        k = p[0] if p else None
    chain.reverse()
    nodes = [k[0] for k in chain]
    waits = []
    for k in chain[1:]:
        waits.append(parent[k][1])
    return nodes, waits, best[key], key[1]


def plan_agent(inst: Instance, task: Task, cons: Constraints,
               ready: dict | None = None, budget: Budget | None = None
               ) -> TimedPath | None:
    """The robot's whole route after its fixed launch prefix: every bay in
    order, then home to its own slot. Legs are planned in turn, each from the
    earliest feasible arrival of the last (the usual multi-goal SIPP)."""
    tb = inst.tb
    cons.finalize()
    ivs = _Intervals(cons)
    pre = task.prefix
    last = pre.steps[-1]
    cur = (last.node, last.heading_in, last.arrive, last.arrive)
    nodes = [last.node]
    waits: list[float] = []
    for g in task.goals:
        earliest = 0.0
        if g.mode == "pick" and ready:
            earliest = ready.get(g.bay, 0.0)
        leg = plan_leg(tb, cons, ivs, cur, g.node, g.min_dwell, earliest,
                       final=(g.mode == "home"), budget=budget)
        if leg is None:
            return None
        lnodes, lwaits, ta, heading = leg
        waits.extend(lwaits)
        nodes.extend(lnodes[1:])
        cur = (g.node, heading, ta, max(ta + g.min_dwell, earliest))
    waits.append(0.0)
    full_nodes = pre.nodes[:-1] + nodes
    full_waits = pre.waits[:-1] + waits
    path = time_path(tb, task.name, full_nodes, full_waits,
                     t0=pre.steps[0].arrive, heading0=pre.steps[0].heading_in,
                     prefix=len(pre.steps) - 1)
    return path
