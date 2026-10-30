"""The physical testbed as a graph with measured timings.

Mirrors agv_grid_workstation_solver.html / static/js/solver.js exactly, so a
plan made here means the same thing as one made in the page:

  * labels: lattice 1..rows*cols (row-major from node 1 at the depot corner),
    bay entry 113+bay mid-edge on the lattice, workstation 64+bay in the bay
    spur, depot node 0, lane DE1..DEn, slots D1..Dn -- the same labels the
    supervisor dispatches;
  * move times: MOVEMENT_SEC and DEPOT_MOVE_SEC (measured on hardware);
  * turn times: TURN_SEC, charged at the node before the move, and NOT
    charged on depot moves (solver.js applyRouteTiming: "measured complete
    move");
  * lanes: corridor keys as solver.js corridorKey() -- an entry shares the
    lane it sits on, so 9->10, 9->114 and 114->10 are one resource.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Any

# solver.js MOVEMENT_SEC / DEPOT_MOVE_SEC / TURN_SEC
GRID_GRID_SEC = 3.56       # RED_RED
GRID_ENTRY_SEC = 2.00      # RED_YELLOW
ENTRY_GRID_SEC = 1.70      # YELLOW_RED
ENTRY_WS_SEC = 1.69        # YELLOW_YELLOW, both ways
NODE0_NODE1_SEC = 4.19     # BLUE_RED, both ways
DE1_NODE0_SEC = 1.82       # DEPOT_ENTRY_TO_NODE_0, both ways
DE_DE_SEC = 2.12           # DEPOT_ENTRY_TO_ENTRY
DE_SLOT_SEC = 2.22         # DEPOT_SLOT_TO_ENTRY, both ways
TURN_90_SEC = 2.65
TURN_180_SEC = 5.00

HEADINGS = ("N", "E", "S", "W")

Label = Any   # int for lattice / entry / workstation / node 0, str for DE#, D#


@dataclass(frozen=True)
class Edge:
    to: Label
    seconds: float
    heading: str
    corridor: str
    depot: bool          # measured complete move: no turn charged


def is_depot_label(label: Label) -> bool:
    return isinstance(label, str) and label.startswith("D")


def is_slot(label: Label) -> bool:
    return isinstance(label, str) and label.startswith("D") and not label.startswith("DE")


class Testbed:
    """Graph, geometry and timing for one rows x cols table and n slots."""

    __test__ = False     # not a pytest test class, despite the name

    def __init__(self, rows: int = 8, cols: int = 8, slots: int = 6):
        self.rows, self.cols, self.slots = rows, cols, slots
        self.n_lattice = rows * cols
        self.n_bays = (rows - 1) * (cols - 1)
        self.pos: dict[Label, tuple[float, float]] = {}
        self.adj: dict[Label, list[Edge]] = {}
        self._build()

    # -- labels -----------------------------------------------------------
    def lattice(self, row: int, col: int) -> int:
        return row * self.cols + col + 1

    def bay_cell(self, bay: int) -> tuple[int, int]:
        return (bay - 1) // (self.cols - 1), (bay - 1) % (self.cols - 1)

    def entry_of(self, bay: int) -> int:
        return self.n_lattice + self.n_bays + bay

    def ws_of(self, bay: int) -> int:
        return self.n_lattice + bay

    def kind(self, label: Label) -> str:
        if is_slot(label):
            return "slot"
        if is_depot_label(label):
            return "lane"
        if label == 0:
            return "node0"
        if 1 <= label <= self.n_lattice:
            return "grid"
        if label <= self.n_lattice + self.n_bays:
            return "workstation"
        if label <= self.n_lattice + 2 * self.n_bays:
            return "entry"
        raise ValueError(f"unknown label {label!r}")

    def bay_of(self, label: Label) -> int | None:
        k = self.kind(label)
        if k == "workstation":
            return label - self.n_lattice
        if k == "entry":
            return label - self.n_lattice - self.n_bays
        return None

    def flanks(self, entry: int) -> tuple[int, int]:
        r, c = self.bay_cell(self.bay_of(entry))
        return self.lattice(r + 1, c), self.lattice(r + 1, c + 1)

    # -- geometry ---------------------------------------------------------
    def heading(self, a: Label, b: Label) -> str:
        (ax, ay), (bx, by) = self.pos[a], self.pos[b]
        dx, dy = bx - ax, by - ay
        if abs(dx) >= abs(dy):
            return "E" if dx >= 0 else "W"
        return "N" if dy > 0 else "S"

    @staticmethod
    def turn_seconds(prev: str | None, nxt: str) -> float:
        if prev is None or prev == nxt:
            return 0.0
        delta = (HEADINGS.index(nxt) - HEADINGS.index(prev)) % 4
        return TURN_180_SEC if delta == 2 else TURN_90_SEC

    @staticmethod
    def reverses(prev: str | None, nxt: str) -> bool:
        return prev is not None and \
            (HEADINGS.index(nxt) - HEADINGS.index(prev)) % 4 == 2

    def dead_end(self, label: Label) -> bool:
        """Where an about-face is allowed: the bay spur and a depot slot.
        Everywhere else a ROTATE_180 is banned (solver.js mergeServiceVisits /
        countMidGridFlips)."""
        return self.kind(label) in ("workstation", "slot")

    def corridor(self, a: Label, b: Label) -> str:
        """solver.js corridorKey(): an entry shares the lattice lane it is on."""
        ka, kb = self.kind(a), self.kind(b)
        if ka == "entry" and kb == "grid":
            f = self.flanks(a)
            return self._key(*f)
        if kb == "entry" and ka == "grid":
            f = self.flanks(b)
            return self._key(*f)
        return self._key(a, b)

    @staticmethod
    def _key(a: Label, b: Label) -> str:
        sa, sb = str(a), str(b)
        return f"{sa}<>{sb}" if sa <= sb else f"{sb}<>{sa}"

    def entry_on_corridor(self, key: str) -> int | None:
        return self._entry_by_key.get(key)

    def squeeze_partners(self, label: Label) -> set:
        """Nodes inside the 5.625in turning circle of `label` that make a
        turn there unsafe: an entry with its two lattice neighbours (5.00in)
        and its workstation (4.99in). Lattice-lattice is 10in and
        workstation-lattice 7.07in -- room to turn."""
        return self._partners.get(label, set())

    def neighbours(self, label: Label) -> list[Edge]:
        return self.adj.get(label, [])

    # -- construction -----------------------------------------------------
    @staticmethod
    def _measured_complete(a: Label, b: Label) -> bool:
        """solver.js applyRouteTiming() charges no turn on a move whose type
        starts with DEPOT (every move OUT of a DE# or D# node) or on 0<->1.
        NODE_0_TO_DEPOT_ENTRY (0 -> DE1, on the way home) is NOT exempt: the
        page charges the turn at node 0, so this must too."""
        return is_depot_label(a) or {a, b} == {0, 1}

    def _add(self, a: Label, b: Label, ab: float, ba: float | None = None):
        key = self.corridor(a, b)
        self.adj.setdefault(a, []).append(
            Edge(b, ab, self.heading(a, b), key, self._measured_complete(a, b)))
        self.adj.setdefault(b, []).append(
            Edge(a, ab if ba is None else ba, self.heading(b, a), key,
                 self._measured_complete(b, a)))

    def _build(self):
        R, C = self.rows, self.cols
        for r in range(R):
            for c in range(C):
                self.pos[self.lattice(r, c)] = (float(c), float(r))
        self._entry_by_key: dict[str, int] = {}
        self._partners: dict[Label, set] = {}
        for bay in range(1, self.n_bays + 1):
            r, c = self.bay_cell(bay)
            e, w = self.entry_of(bay), self.ws_of(bay)
            self.pos[e] = (c + 0.5, r + 0.85)
            self.pos[w] = (c + 0.5, r + 0.5)
        # depot: node 0 straight below node 1, lane running east, slots north
        self.pos[0] = (0.0, -1.35)
        for k in range(1, self.slots + 1):
            self.pos[f"DE{k}"] = (0.64 * k, -1.35)
            self.pos[f"D{k}"] = (0.64 * k, -0.75)
        for r in range(R):
            for c in range(C):
                n = self.lattice(r, c)
                if c + 1 < C:
                    self._add(n, self.lattice(r, c + 1), GRID_GRID_SEC)
                if r + 1 < R:
                    self._add(n, self.lattice(r + 1, c), GRID_GRID_SEC)
        for bay in range(1, self.n_bays + 1):
            e, w = self.entry_of(bay), self.ws_of(bay)
            f1, f2 = self.flanks(e)
            self._entry_by_key[self._key(f1, f2)] = e
            for f in (f1, f2):
                self._add(f, e, GRID_ENTRY_SEC, ENTRY_GRID_SEC)
            self._add(e, w, ENTRY_WS_SEC)
            for p in (f1, f2, w):
                self._partners.setdefault(e, set()).add(p)
                self._partners.setdefault(p, set()).add(e)
        self._add(0, 1, NODE0_NODE1_SEC)
        self._add("DE1", 0, DE1_NODE0_SEC)
        for k in range(1, self.slots + 1):
            if k > 1:
                self._add(f"DE{k}", f"DE{k - 1}", DE_DE_SEC)
            self._add(f"D{k}", f"DE{k}", DE_SLOT_SEC)

    # -- distances (heuristics) ------------------------------------------
    def time_to(self, goal: Label) -> dict[Label, float]:
        """Lower-bound travel seconds from every node to `goal` (turns and
        other robots ignored, so admissible for A*)."""
        return _time_to(self, goal)


_TT_CACHE: dict[tuple, dict] = {}


def _time_to(tb: Testbed, goal: Label) -> dict:
    key = (id(tb), goal)
    hit = _TT_CACHE.get(key)
    if hit is not None:
        return hit
    dist = {goal: 0.0}
    pq = [(0.0, 0, goal)]
    seq = 1
    while pq:
        d, _s, u = heapq.heappop(pq)
        if d > dist.get(u, math.inf):
            continue
        # reverse edges: who can reach u, and how fast
        for e in tb.adj.get(u, []):
            v = e.to
            back = next((x for x in tb.adj.get(v, []) if x.to == u), None)
            if back is None:
                continue
            nd = d + back.seconds
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, seq, v))
                seq += 1
    _TT_CACHE[key] = dist
    return dist
