"""The one conflict model every method is planned against and judged by.

A robot HOLDS resources over time:

  node presence   the node it stands on, from arrival until it starts to
                  drive away, widened by PRE_SEC before and POST_SEC after
                  (its body is in the node's space while it approaches and
                  until it has cleared it -- ~5in at ~2.8in/s);
  lane            the corridor it drives along, for the drive; and a robot
                  STANDING on a bay entry also holds that entry's lane, since
                  the entry lies mid-lane (solver.js corridorKey);
  turn            a turn at a node, for the turn.

Two robots conflict when they hold the same node or the same lane at
overlapping times, or when one TURNS at a node while the other is present at
a squeeze partner of it (an entry and its two lattice neighbours or its
workstation: 5.00/4.99in apart against a 5.625in turning circle). That last
rule is the runtime's rotation gate, so a plan that passes here does not
rely on the gate to hold anyone.

This is deliberately an occupancy model, not solver.js detectConflicts()'s
arrival-time windows: that one compares ARRIVAL times only, so a robot
dwelling 30s on a node is invisible to a second robot arriving 10s later.
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass

from .model import Label, Testbed
from .plan import TimedPath

PRE_SEC = 1.0
POST_SEC = 1.0
EPS = 1e-6


@dataclass(frozen=True)
class Hold:
    agent: str
    kind: str            # 'node' | 'lane' | 'turn'
    resource: object     # node label (node/turn) or corridor key (lane)
    start: float
    end: float
    step: int            # index into the agent's steps
    via: str             # 'presence' | 'standing' | 'drive' | 'turn'


@dataclass(frozen=True)
class Conflict:
    kind: str            # 'node' | 'lane' | 'rotation'
    a: Hold
    b: Hold

    @property
    def time(self) -> float:
        return max(self.a.start, self.b.start)

    def describe(self) -> str:
        if self.kind == "rotation":
            return (f"{self.a.agent} turns at {self.a.resource} "
                    f"{self.a.start:.1f}-{self.a.end:.1f}s while {self.b.agent} "
                    f"is at {self.b.resource}")
        return (f"{self.kind} {self.a.resource}: {self.a.agent} "
                f"{self.a.start:.1f}-{self.a.end:.1f}s vs {self.b.agent} "
                f"{self.b.start:.1f}-{self.b.end:.1f}s")


def overlaps(a0: float, a1: float, b0: float, b1: float) -> bool:
    return a0 < b1 - EPS and b0 < a1 - EPS


def holds(tb: Testbed, path: TimedPath) -> list[Hold]:
    out: list[Hold] = []
    last = len(path.steps) - 1
    for i, s in enumerate(path.steps):
        leave = math.inf if i == last else s.drive_start + POST_SEC
        out.append(Hold(path.agent, "node", s.node, s.arrive - PRE_SEC, leave,
                        i, "presence"))
        if tb.kind(s.node) == "entry":
            out.append(Hold(path.agent, "lane", tb.corridor(*tb.flanks(s.node)),
                            s.arrive - PRE_SEC, leave, i, "standing"))
        if i < last:
            if s.turn > 0:
                out.append(Hold(path.agent, "turn", s.node, s.turn_start,
                                s.drive_start, i, "turn"))
            nxt = path.steps[i + 1].node
            out.append(Hold(path.agent, "lane", tb.corridor(s.node, nxt),
                            s.drive_start, s.drive_start + s.move, i, "drive"))
    return out


def detect_conflicts(tb: Testbed, paths: dict[str, TimedPath],
                     first_only: bool = False) -> list[Conflict]:
    """Every pairwise conflict, earliest first."""
    by_res: dict[tuple, list[Hold]] = defaultdict(list)
    turns: list[Hold] = []
    for p in paths.values():
        for h in holds(tb, p):
            if h.kind == "turn":
                turns.append(h)
            else:
                by_res[(h.kind, h.resource)].append(h)
    found: list[Conflict] = []
    for (kind, _res), hs in by_res.items():
        hs.sort(key=lambda h: h.start)
        for i, a in enumerate(hs):
            for b in hs[i + 1:]:
                if b.start >= a.end - EPS:
                    break
                if a.agent != b.agent and overlaps(a.start, a.end, b.start, b.end):
                    found.append(Conflict(kind, a, b))
    presence = by_res  # node holds keyed ('node', label)
    for t in turns:
        for u in tb.squeeze_partners(t.resource):
            for h in presence.get(("node", u), ()):
                if h.agent != t.agent and overlaps(t.start, t.end, h.start, h.end):
                    found.append(Conflict("rotation", t, h))
    found.sort(key=lambda c: (c.time, c.kind, c.a.agent, c.b.agent))
    return found[:1] if first_only else found


class Constraints:
    """Intervals an agent's own plan must keep out of, per resource.

        node[v]  my PRESENCE (raw [arrive, drive_start]) may not overlap
        lane[k]  my DRIVE on corridor k may not overlap
        turn[v]  my TURN at v may not overlap

    add_against(hold) adds exactly what is needed for none of my holds to
    conflict with `hold` under detect_conflicts(). Prioritized planning feeds
    it every hold of the robots planned before; CBS feeds it one hold per
    branch.
    """

    def __init__(self):
        self.node: dict[Label, list] = defaultdict(list)
        self.lane: dict[str, list] = defaultdict(list)
        self.turn: dict[Label, list] = defaultdict(list)

    def copy(self) -> "Constraints":
        c = Constraints()
        for src, dst in ((self.node, c.node), (self.lane, c.lane),
                         (self.turn, c.turn)):
            for k, v in src.items():
                dst[k] = list(v)
        return c

    def add_against(self, tb: Testbed, h: Hold) -> None:
        if h.kind == "node":
            # my padded presence vs its padded presence
            self.node[h.resource].append((h.start - POST_SEC, h.end + PRE_SEC))
            for v in tb.squeeze_partners(h.resource):
                self.turn[v].append((h.start, h.end))     # my turns beside it
        elif h.kind == "lane":
            self.lane[h.resource].append((h.start, h.end))  # my drives
            e = tb.entry_on_corridor(h.resource)
            if e is not None:                               # my standing on it
                self.node[e].append((h.start - POST_SEC, h.end + PRE_SEC))
        elif h.kind == "turn":
            for u in tb.squeeze_partners(h.resource):       # my presence beside
                self.node[u].append((h.start - POST_SEC, h.end + PRE_SEC))

    def add_path(self, tb: Testbed, path: TimedPath) -> None:
        for h in holds(tb, path):
            self.add_against(tb, h)

    def finalize(self) -> "Constraints":
        for table in (self.node, self.lane, self.turn):
            for k in list(table):
                table[k] = merge(table[k])
        return self


def merge(intervals: list) -> list:
    out: list = []
    for a, b in sorted(intervals):
        if out and a <= out[-1][1] + EPS:
            if b > out[-1][1]:
                out[-1] = (out[-1][0], b)
        else:
            out.append((a, b))
    return out


def free_intervals(blocked: list, horizon: float = math.inf) -> list:
    """Complement of merged `blocked` over [-inf, horizon)."""
    out = []
    t = -math.inf
    for a, b in blocked:
        if a > t + EPS:
            out.append((t, a))
        t = max(t, b)
    if t < horizon:
        out.append((t, horizon))
    return out
