#!/usr/bin/env python3
"""
dispatch_advisor.py — deterministic core of the LLM-gated collision advisor.

Ground truth is the plan exported by agv_grid_workstation_solver.html
(downloadJson() payload). This module:

  1. Aligns each robot's flat firmware command list against its exported
     schedule route at load time (fails loudly if they don't match).
  2. Re-anchors every robot's remaining schedule to live completion times
     each time a robot finishes a command.
  3. Simulates turns as node occupancy and translation as edge occupancy,
     with separate following, opposite-direction, node, intersection,
     shared-depot, and depot-slot clearances.
  4. Gates only the immediate command. Immediate moves acquire deterministic
     resource reservations; collision waits keep the robot supervisor-idle.
     The optional LLM may select only a prevalidated wait horizon and cannot
     change ownership, priority, or the command sent to a robot.

No ROS and no LLM imports here — this module is pure stdlib so it can be
unit-tested anywhere. The LLM client is injected (see ollama_advisor.py;
haiku_advisor.py is the retired Anthropic-API backend, kept for reference).

Physical clearances are named below so hardware tests can tune each resource
interaction independently.
"""

from __future__ import annotations

import json
import math
import os
import re
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

# --- Tunable physical safety policy -----------------------------------------
# These values describe different physical interactions.  They intentionally
# replace the old single seven-second rule; tune them from fleet tests without
# changing the priority algorithm.
TURN_SEC = {
    "RIGHT": 2.65,
    "LEFT": 2.65,
    "ROTATE_180": 5.00,
}
ENTRY_TIME_EPSILON_SEC = 0.05
# When two robots contend for the same resource with entry times inside this
# window AND no hard discriminator (movement, node-clearing, reservation)
# separates them, the deterministic order falls to an arbitrary robot-name
# sort. That is the one place the choice of who yields is genuinely ambiguous
# and makespan-relevant, so the LLM may break it (cached per pair for
# consistency; guarded so it can never invert a clear temporal lead). Outside
# this window priority stays fully deterministic.
PRIORITY_TIE_WINDOW_SEC = 0.75
NODE_CLEARANCE_SEC = 1.0
INTERSECTION_CLEARANCE_SEC = 1.0
SAME_DIRECTION_HEADWAY_SEC = 1.0
OPPOSITE_DIRECTION_CLEARANCE_SEC = 0.5
DEPOT_ENTRANCE_CLEARANCE_SEC = 1.0
DEPOT_SLOT_CLEARANCE_SEC = 0.5
EDGE_RELEASE_BUFFER_SEC = 0.1
COLLISION_RECHECK_SEC = 0.20

# Backward-compatible names for callers that imported the solver-era values.
# They now map to the resource-specific node/following policy above.
SAFETY_WINDOW_SEC = NODE_CLEARANCE_SEC
EDGE_SAFETY_WINDOW_SEC = SAME_DIRECTION_HEADWAY_SEC

# firmware DWELL clamp (MIN_DWELL_MS / MAX_DWELL_MS in the sketch)
MIN_DWELL_MS = 200
MAX_DWELL_MS = 30000
DEFAULT_DWELL_MS = 2000  # WORKSTATION_WAIT_MS in firmware

# --- firmware command vocabulary --------------------------------------------
# Movement commands mapped to the marker color that terminates them.
MOVE_COMMANDS = {
    "FORWARD_UNTIL_RED": {"RED"},
    "FORWARD_UNTIL_YELLOW": {"YELLOW"},
    "FORWARD_UNTIL_BLUE": {"BLUE"},
    "FORWARD_UNTIL_COLOR": {"RED", "YELLOW", "BLUE"},
    "BACKWARD_UNTIL_COLOR": {"RED", "YELLOW", "BLUE"},
    "BACKWARD_UNTIL_YELLOW": {"YELLOW"},
    "BACKWARD_UNTIL_BLUE": {"BLUE"},
}
TURN_COMMANDS = {
    "RIGHT_UNTIL_COLOR": TURN_SEC["RIGHT"],
    "LEFT_UNTIL_COLOR": TURN_SEC["LEFT"],
    "ROTATE_180": TURN_SEC["ROTATE_180"],
}
NOOP_COMMANDS = {"STOP", "RESET_POSE", "GET_STATUS"}


class AlignmentError(Exception):
    """A robot's command list doesn't match its exported schedule route."""


def classify_label(label: Any, rows: int, cols: int) -> str:
    """Marker color at a schedule path node (labels from pathLabel() in the
    HTML). Physical layout after the 2026-07-14 re-stickering (node 0 changed
    to BLUE by the user — it belongs to the depot, not the grid): grid nodes
    1..rows*cols are RED; node 0, the depot-lane junctions ('DE#') and the
    parking stickers ('D#') are BLUE; bay entries/workstations are YELLOW."""
    if isinstance(label, str):
        return "BLUE"
    n = int(label)
    if n == 0:
        return "BLUE"
    if 1 <= n <= rows * cols:
        return "RED"
    return "YELLOW"


def parse_dwell_ms(cmd: str) -> int:
    parts = cmd.split()
    if len(parts) >= 2:
        try:
            ms = int(parts[1])
            if ms > 0:
                return max(MIN_DWELL_MS, min(MAX_DWELL_MS, ms))
        except ValueError:
            pass
    return DEFAULT_DWELL_MS


# --- Grid geometry + vision-guided recovery ----------------------------------
# The supervisor recovers a LINE_LOST / runaway-stopped robot with the existing
# command vocabulary: ROTATE_180 puts the just-followed line back in front of
# the robot, FORWARD_UNTIL_RED lands it on a lattice node, camera vision names
# that node and (from the position delta) the travel heading, and a bridge of
# turns + FORWARD_UNTIL_REDs walks it back to where its plan can resume. Pure
# grid math lives here so it stays unit-testable without ROS.

CARDINALS = ["N", "E", "S", "W"]


def label_to_cell(label: Any, rows: int, cols: int):
    """Schedule node label -> vision grid cell (cells relative to node 1, +x
    east, +y north — the frame apriltag_localize.py publishes). Mirrors
    dispatchLabelToCell in the HTML. Non-numeric depot-lane labels ('DE1',
    'D2') return None; numeric labels — including numeric STRINGS like '18'
    that the reservation layer produces via str(node) — map to a cell."""
    try:
        n = int(label)
    except (TypeError, ValueError):
        return None  # 'DE1', 'D2', etc. — no grid coordinate
    if n == 0:
        return (0.0, -1.35)
    nodes = rows * cols
    if 1 <= n <= nodes:
        idx = n - 1
        return (float(idx % cols), float(idx // cols))
    bays = (rows - 1) * (cols - 1)
    is_workstation = n <= nodes + bays
    bay_num = (n - nodes) if is_workstation else (n - nodes - bays)
    b = bay_num - 1
    # Bay numbering flipped 2026-07-17 to a bottom-up origin (B1 = bay row
    # nearest the depot), matching the node numbering. A bay's entry sits on
    # its north node row; the workstation is half a cell south of it.
    row_up = b // (cols - 1)
    c = b % (cols - 1)
    gy = (row_up + 1) - (0.5 if is_workstation else 0.15)
    return (c + 0.5, gy)


def node_at_cell(gx: float, gy: float, rows: int, cols: int,
                 tol: float = 0.35):
    """Nearest lattice node label for a vision cell, or None when the point
    is farther than tol cells from every node or outside the grid."""
    c = round(gx)
    r = round(gy)
    if not (0 <= c < cols and 0 <= r < rows):
        return None
    if abs(gx - c) > tol or abs(gy - r) > tol:
        return None
    return r * cols + c + 1


def heading_between_cells(a, b) -> str:
    """Dominant-axis cardinal from cell a to cell b (ties go E/W, matching
    the HTML's headingBetween). Callers must guard near-zero displacement."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if abs(dx) >= abs(dy):
        return "E" if dx >= 0 else "W"
    return "N" if dy > 0 else "S"


def turn_commands(from_heading: str, to_heading: str) -> list[str]:
    delta = (CARDINALS.index(to_heading) - CARDINALS.index(from_heading)) % 4
    if delta == 1:
        return ["RIGHT_UNTIL_COLOR"]
    if delta == 3:
        return ["LEFT_UNTIL_COLOR"]
    if delta == 2:
        return ["ROTATE_180"]
    return []


def resync_point(plan: list, failed_index: int, rows: int, cols: int):
    """Where a recovered robot can rejoin its plan: the first move item at or
    after the failed one that starts from a red lattice node. Items before it
    are considered physically consumed (e.g. a bay exit the robot already
    blundered through). Returns (plan_index, node_label, heading) or None
    when the remaining plan never re-anchors on the lattice (depot legs)."""
    nodes = rows * cols
    for idx in range(failed_index, len(plan)):
        item = plan[idx]
        if item.kind != "move":
            continue
        fr = item.from_node
        if isinstance(fr, str):
            return None  # depot lane — no vision anchor there, give up
        fr = int(fr)
        if not (1 <= fr <= nodes):
            continue  # starts at an entry/workstation — try the next move
        a = label_to_cell(fr, rows, cols)
        b = label_to_cell(item.to_node, rows, cols)
        if a is None or b is None:
            return None
        return (idx, fr, heading_between_cells(a, b))
    return None


def bridge_commands(from_node: int, from_heading: str,
                    to_node: int, to_heading: str,
                    rows: int, cols: int) -> list[tuple]:
    """Grid-only command sequence taking a recovered robot from
    (from_node, from_heading) to (to_node, to_heading): turns plus one
    FORWARD_UNTIL_RED per lattice hop, as (command, target_node|None) so the
    caller can vision-check each hop for occupancy. Leg order (rows-first vs
    cols-first) is chosen to minimise about-faces, then length."""
    fc = ((from_node - 1) % cols, (from_node - 1) // cols)
    tc = ((to_node - 1) % cols, (to_node - 1) // cols)

    def build(axis_order):
        steps: list[tuple] = []
        cur = [fc[0], fc[1]]
        heading = from_heading
        for axis in axis_order:
            while cur[axis] != tc[axis]:
                step = 1 if tc[axis] > cur[axis] else -1
                hop = ("E" if step > 0 else "W") if axis == 0 else \
                      ("N" if step > 0 else "S")
                for t in turn_commands(heading, hop):
                    steps.append((t, None))
                heading = hop
                cur[axis] += step
                steps.append(("FORWARD_UNTIL_RED", cur[1] * cols + cur[0] + 1))
        for t in turn_commands(heading, to_heading):
            steps.append((t, None))
        return steps

    candidates = [build([0, 1]), build([1, 0])]
    flips = lambda s: sum(1 for cmd, _ in s if cmd == "ROTATE_180")  # noqa: E731
    return min(candidates, key=lambda s: (flips(s), len(s)))


# --- Plan items --------------------------------------------------------------

@dataclass
class PlanItem:
    command: str
    kind: str                      # "move" | "turn" | "dwell" | "noop"
    duration_sec: float = 0.0      # for moves: move time only (turns are separate items)
    from_node: Any = None          # moves only
    to_node: Any = None            # moves only
    segment_index: Optional[int] = None
    # Depot-lane segments traversed implicitly as part of this move,
    # as (from, to, duration_sec) tuples emitted before the move itself.
    implicit_segs: list = field(default_factory=list)


def align_commands_to_route(route: dict, commands: list[str],
                            rows: int, cols: int) -> list[PlanItem]:
    """Walk the command list and the exported segments together, producing one
    PlanItem per command with durations taken from the export.

    Rules:
      - Each movement command consumes the next marker-terminated segment
        whose destination color matches; depot-lane segments (type DEPOT_*)
        in between are consumed implicitly and their time attributed to the
        move. If the colors don't match but a depot segment is next, the
        command consumes that single depot segment instead (route files may
        encode the depot maneuver as explicit marker commands).
      - Turn commands sit at the current node; if exactly one turn precedes
        a matched segment with turn_sec > 0, the exported turn_sec is used
        as its duration, otherwise the TURN_SEC constant.
      - DWELL / STOP / RESET_POSE / GET_STATUS never consume segments.
    A movement command with no consumable segment left raises AlignmentError.
    """
    segs = sorted(route.get("segments", []), key=lambda s: s["segment_index"])
    agv = route.get("agv", f"agent{route.get('agent', '?')}")
    plan: list[PlanItem] = []
    si = 0
    pending_turn_idxs: list[int] = []

    def seg_duration(seg: dict) -> float:
        return float(seg.get("move_sec", 0.0)) + float(seg.get("turn_sec", 0.0))

    for raw in commands:
        cmd = raw.strip().upper()
        if not cmd:
            continue
        base = cmd.split()[0]

        if base in TURN_COMMANDS or cmd in TURN_COMMANDS:
            key = base if base in TURN_COMMANDS else cmd
            plan.append(PlanItem(command=cmd, kind="turn",
                                 duration_sec=TURN_COMMANDS[key]))
            pending_turn_idxs.append(len(plan) - 1)
            continue

        if base == "DWELL":
            plan.append(PlanItem(command=cmd, kind="dwell",
                                 duration_sec=parse_dwell_ms(cmd) / 1000.0))
            continue

        if cmd in NOOP_COMMANDS:
            plan.append(PlanItem(command=cmd, kind="noop", duration_sec=0.0))
            continue

        if cmd in MOVE_COMMANDS:
            colors = MOVE_COMMANDS[cmd]
            # Scan forward to the segment this command physically stops at.
            j = si
            implicit: list[dict] = []
            matched = None
            while j < len(segs):
                seg = segs[j]
                # Color match first: with node 0 blue, both the exit hop into
                # node 0 and the returning node1->node0 hop match a
                # FORWARD_UNTIL_BLUE one-to-one.
                if classify_label(seg["to"], rows, cols) in colors:
                    matched = seg
                    j += 1
                    break
                is_depot = str(seg.get("type", "")).startswith("DEPOT")
                if is_depot:
                    implicit.append(seg)
                    j += 1
                    continue
                break

            if matched is None and implicit:
                # Command drives a depot-lane hop directly (color check
                # doesn't apply — no schedule marker there).
                matched = implicit[0]
                implicit = []
                j = si + 1

            if matched is None:
                nxt = segs[si] if si < len(segs) else None
                nxt_desc = (f"{nxt['from']}->{nxt['to']} ({nxt.get('type')})"
                            if nxt else "none left")
                raise AlignmentError(
                    f"[{agv}] command #{len(plan) + 1} '{cmd}' has no matching "
                    f"schedule segment (next segment: {nxt_desc}). Expected a "
                    f"destination with marker color in {sorted(colors)}; check "
                    f"that the route file was authored from this exported plan.")

            si = j
            turn_sec = float(matched.get("turn_sec", 0.0))
            move_sec = float(matched.get("move_sec", 0.0))
            if pending_turn_idxs:
                if len(pending_turn_idxs) == 1 and turn_sec > 0:
                    plan[pending_turn_idxs[0]].duration_sec = turn_sec
                # multiple queued turns keep their TURN_SEC constants
            elif turn_sec > 0:
                # Schedule expects an in-place turn but the route file has no
                # turn command here — keep the plan's timing anyway.
                move_sec += turn_sec
            pending_turn_idxs = []

            plan.append(PlanItem(
                command=cmd, kind="move",
                duration_sec=move_sec,
                from_node=matched["from"], to_node=matched["to"],
                segment_index=matched["segment_index"],
                implicit_segs=[(s["from"], s["to"], seg_duration(s)) for s in implicit],
            ))
            continue

        raise AlignmentError(f"[{agv}] unknown command '{cmd}' in route file")

    return plan


# --- Fleet simulation ---------------------------------------------------------

@dataclass
class NodeEvent:
    node: str
    time_sec: float
    plan_index: int
    from_node: Optional[str] = None


@dataclass
class NodeInterval:
    node: str
    start_sec: float
    end_sec: float
    plan_index: int


@dataclass
class EdgeInterval:
    from_node: str
    to_node: str
    key: str
    start_sec: float
    end_sec: float
    plan_index: int
    shared_depot_entrance: bool = False
    depot_slot: Optional[str] = None


@dataclass
class RobotSim:
    events: list = field(default_factory=list)     # list[NodeEvent]
    node_intervals: list = field(default_factory=list)  # list[NodeInterval]
    intervals: list = field(default_factory=list)  # list[EdgeInterval]
    end_sec: float = 0.0


def edge_key(a: Any, b: Any) -> str:
    return "<>".join(sorted([str(a), str(b)]))


def is_workstation_node(label: Any, rows: int, cols: int) -> bool:
    """True if the label is a WORKSTATION node — the dead-end spur where a robot
    physically services a bay, past the entry sticker (numbered
    nodes+1 .. nodes+bays). A robot parked here is OFF the through-lane, so it
    should not block another robot traversing the entry-node corridor."""
    try:
        n = int(label)
    except (TypeError, ValueError):
        return False
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    return nodes < n <= nodes + bays


def is_depot_lane_label(label) -> bool:
    """A node on the single-file depot lane: DE1..DEn, or node 0."""
    t = str(label).strip().upper()
    return t == "0" or bool(re.fullmatch(r"DE\d+", t))

def entry_rotation_footprint(label: Any, rows: int, cols: int):
    """Every node a rotation AT a bay entry physically sweeps into.

    MEASURED 2026-09-30, bay 1 (entry 114, between lattice 9 and 10, serving
    workstation 65), against the robot's 5.625in corner-to-corner diameter:

        114 <-> 9    5.00in   -0.62in
        114 <-> 10   5.00in   -0.62in
        114 <-> 65   4.99in   -0.63in

    All three are INSIDE the turning circle. A robot at a bay entry cannot
    rotate without overlapping a robot on either adjacent lattice node or on
    its own workstation -- that is the table's geometry, not a threshold to
    tune. Everywhere else has room: lattice-to-lattice is 10.00in (+4.38),
    and the workstation itself clears its neighbours by +1.30 to +1.58in, so
    rotating at 65 is safe while rotating at 114 never is.

    This is what made contact on 2026-09-30: Alvik3 sat at entry 123
    (stationary, so the closing-speed guard saw nothing to do), then turned,
    and swept into Alvik4 driving to the adjacent lattice node.

    Returns a set of str labels including the entry itself, or None when
    `label` is not a bay entry.
    """
    pair = entry_edge_nodes(label, rows, cols)
    if pair is None:
        return None
    out = {str(label), str(pair[0]), str(pair[1])}
    # The workstation this entry serves sits one bay-block below it.
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    ws = int(label) - bays
    if nodes < ws <= nodes + bays:
        out.add(str(ws))
    return out


def entry_edge_nodes(label: Any, rows: int, cols: int):
    """The two grid nodes a workstation ENTRY node sits between, or None.

    A bay entry sticker lies mid-edge on the grid lattice (e.g. entry 122 is on
    the corridor between grid nodes 18 and 19). Traversing that corridor and
    servicing that bay therefore share the same physical lane, even though the
    labels differ. Workstation nodes (the dead-end spur past the entry) are NOT
    on the corridor and return None. Returns (west_node, east_node) as ints."""
    try:
        n = int(label)
    except (TypeError, ValueError):
        return None
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    # Only entry nodes: nodes+bays+1 .. nodes+2*bays (workstations are below).
    if not (nodes + bays < n <= nodes + 2 * bays):
        return None
    cell = label_to_cell(n, rows, cols)
    if cell is None:
        return None
    gx, gy = cell
    r = round(gy)
    wc = int(round(gx - 0.5))
    west = node_at_cell(wc, r, rows, cols, tol=0.5)
    east = node_at_cell(wc + 1, r, rows, cols, tol=0.5)
    if west is None or east is None:
        return None
    return west, east


def corridor_key(a: Any, b: Any, rows: int, cols: int) -> str:
    """Canonical key for the physical lane an edge occupies, unifying a full
    grid edge (G1<>G2) with the two half-edges that a bay entry splits it into
    (G1<>E and E<>G2). All three collapse to edge_key(G1, G2) so the reservation
    and conflict layers see them as ONE resource. Any other edge returns its
    plain edge_key."""
    # If either endpoint is an entry node, map to the grid edge it lies on.
    for endpoint, other in ((a, b), (b, a)):
        flank = entry_edge_nodes(endpoint, rows, cols)
        if flank is not None:
            west, east = flank
            # The other endpoint should be one of the flanking grid nodes; the
            # corridor is the full grid edge between them.
            if str(other) in (str(west), str(east)):
                return edge_key(west, east)
    return edge_key(a, b)


def directed_edge_key(a: Any, b: Any) -> str:
    return f"{a}->{b}"


def _depot_slot_label(a: Any, b: Any) -> Optional[str]:
    for value in (a, b):
        label = str(value).upper()
        if re.fullmatch(r"D\d+", label):
            return label
    return None


def _uses_shared_depot_entrance(a: Any, b: Any) -> bool:
    """Shared merge resource for movements *entering* node 0.

    A movement leaving node 0 for its own DE# branch does not retain this
    resource. Its node-0 departure and a follower's later arrival are instead
    ordered by the normal node/intersection space-time clearances.
    """
    return str(b) == "0"


def movement_segments(item: PlanItem) -> list[tuple[Any, Any, float]]:
    """Every continuously traversed segment in one movement command."""
    if item.kind != "move":
        return []
    return [*item.implicit_segs,
            (item.from_node, item.to_node, item.duration_sec)]


def simulate_plan(plan: list[PlanItem], start_index: int, anchor_sec: float,
                  lead_dwell_ms: int = 0, current_node: Any = None,
                  move_scale: float = 1.0, turn_scale: float = 1.0,
                  rows: int = 8, cols: int = 8) -> RobotSim:
    """Simulate physical occupancy from ``anchor_sec``.

    Turns, planned dwells, and pre-departure waiting occupy the current node.
    An edge interval begins only when translation begins and ends on arrival.
    Implicit depot segments remain consecutive parts of the same command.
    The final node is occupied indefinitely because a completed/idle robot
    does not disappear from the floor.
    """
    sim = RobotSim()
    t = anchor_sec + lead_dwell_ms / 1000.0
    if current_node is None:
        for future in plan[start_index:]:
            segs = movement_segments(future)
            if segs:
                current_node = segs[0][0]
                break
    current_node = None if current_node is None else str(current_node)
    node_enter_sec = anchor_sec
    last_plan_index = max(0, min(start_index, len(plan) - 1)) if plan else 0

    def add_node_interval(end_sec: float, plan_index: int) -> None:
        nonlocal node_enter_sec
        if current_node is not None and end_sec > node_enter_sec:
            sim.node_intervals.append(NodeInterval(
                current_node, node_enter_sec, end_sec, plan_index))

    def traverse(frm: Any, to: Any, duration: float, plan_index: int) -> None:
        nonlocal t, current_node, node_enter_sec
        add_node_interval(t, plan_index)
        start = t
        t += duration
        frm_s, to_s = str(frm), str(to)
        # Corridor key, not the raw edge key: a grid edge and the two half-edges
        # a bay entry splits it into share one physical lane, so a robot
        # servicing a bay conflicts with another traversing that grid edge
        # (e.g. entry 122 lane == grid edge 18<>19).
        sim.intervals.append(EdgeInterval(
            frm_s, to_s, corridor_key(frm, to, rows, cols), start, t, plan_index,
            shared_depot_entrance=_uses_shared_depot_entrance(frm, to),
            depot_slot=_depot_slot_label(frm, to)))
        sim.events.append(NodeEvent(to_s, t, plan_index, from_node=frm_s))
        current_node = to_s
        node_enter_sec = t

    for idx in range(start_index, len(plan)):
        item = plan[idx]
        last_plan_index = idx
        if item.kind == "turn":
            t += item.duration_sec * turn_scale
        elif item.kind in ("dwell", "noop"):
            t += item.duration_sec  # dwells are firmware timers — exact
        elif item.kind == "move":
            for frm, to, duration in movement_segments(item):
                traverse(frm, to, duration * move_scale, idx)
    if current_node is not None:
        sim.node_intervals.append(NodeInterval(
            current_node, node_enter_sec, math.inf, last_plan_index))
    sim.end_sec = t
    return sim


# --- Online timing calibration (digital-twin self-calibration) ---------------

CALIBRATION_ALPHA = 0.3                 # EWMA weight of each new sample
CALIBRATION_MIN_PREDICTED_SEC = 0.3     # ignore near-zero predictions
CALIBRATION_SAMPLE_CLAMP = (0.5, 2.0)   # per-sample ratio clamp (outliers,
                                        # e.g. a lost first send + resend)
CALIBRATION_FACTOR_CLAMP = (0.7, 1.5)   # applied factor stays near 1.0


class TimingCalibration:
    """Per-robot, per-kind execution-time correction factors.

    Plan durations come from bench-measured constants, but real robots run a
    few percent fast or slow (battery level, motor spread, floor condition).
    Every completed move/turn yields a sample ratio actual/predicted; an EWMA
    per (robot, kind) is applied when simulating remaining plans, so conflict
    prediction tightens as the mission runs. Factors persist across missions
    via a small JSON file — the model learns its physical parameters from
    operation (the classic digital-twin self-calibration property).
    """

    KINDS = ("move", "turn")

    def __init__(self, path: Optional[str] = None):
        self.path = path
        self.factors: dict[str, dict[str, dict]] = {}
        if path and os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    self.factors = json.load(f)
            except (OSError, json.JSONDecodeError):
                self.factors = {}

    def factor(self, robot: str, kind: str) -> float:
        entry = self.factors.get(robot, {}).get(kind)
        if not entry:
            return 1.0
        lo, hi = CALIBRATION_FACTOR_CLAMP
        return min(hi, max(lo, float(entry.get("ratio", 1.0))))

    def observe(self, robot: str, kind: str, predicted_sec: float,
                actual_sec: float) -> Optional[float]:
        """Fold one completed command into the model; returns the (clamped)
        sample ratio, or None when the sample is unusable."""
        if kind not in self.KINDS or predicted_sec < CALIBRATION_MIN_PREDICTED_SEC:
            return None
        lo, hi = CALIBRATION_SAMPLE_CLAMP
        sample = min(hi, max(lo, actual_sec / predicted_sec))
        entry = self.factors.setdefault(robot, {}).setdefault(
            kind, {"ratio": 1.0, "n": 0})
        entry["ratio"] = round((1 - CALIBRATION_ALPHA) * float(entry["ratio"])
                               + CALIBRATION_ALPHA * sample, 4)
        entry["n"] = int(entry.get("n", 0)) + 1
        self._save()
        return sample

    def _save(self) -> None:
        if not self.path:
            return
        try:
            tmp = f"{self.path}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.factors, f, indent=2, sort_keys=True)
            os.replace(tmp, self.path)
        except OSError:
            pass

    def summary(self) -> str:
        parts = []
        for robot in sorted(self.factors):
            for kind in sorted(self.factors[robot]):
                entry = self.factors[robot][kind]
                parts.append(f"{robot}.{kind}={entry['ratio']:.2f}"
                             f"(n={entry['n']})")
        return ", ".join(parts) or "no factors yet (defaults 1.0)"


# --- Conflict detection (port of detectConflicts() in the HTML) ---------------

@dataclass
class Conflict:
    type: str
    time_sec: float
    robots: tuple                  # (name_a, name_b)
    key: str                       # node label or edge key
    gap_sec: float
    detail: str
    clearance_sec: float = 0.0
    # per-robot info always includes entry and plan_index; interval resources
    # also include start/end.
    info: dict = field(default_factory=dict)

    @property
    def is_edge(self) -> bool:
        return "edge" in self.type


def detect_conflicts(fleet: dict[str, RobotSim]) -> list[Conflict]:
    """Direction-aware space-time conflict detection.

    Same-direction followers, opposite-direction users, stationary node
    occupancy, intersection passages, the depot funnel, and depot slots have
    separate tunable clearances.  The function reports ordering conflicts;
    :class:`DispatchAdvisor` decides which robot owns each resource.
    """
    conflicts: list[Conflict] = []
    names = list(fleet)
    seen: set[tuple] = set()

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a_name, b_name = names[i], names[j]
            a_sim, b_sim = fleet[a_name], fleet[b_name]

            def add(conflict: Conflict) -> None:
                sig = (
                    a_name, b_name, conflict.type, conflict.key,
                    conflict.info[a_name]["plan_index"],
                    conflict.info[b_name]["plan_index"],
                    round(conflict.info[a_name]["entry"], 6),
                    round(conflict.info[b_name]["entry"], 6),
                )
                if sig not in seen:
                    seen.add(sig)
                    conflicts.append(conflict)

            def interval_gap(a0: float, a1: float,
                             b0: float, b1: float) -> tuple[bool, float]:
                overlap = max(a0, b0) < min(a1, b1)
                if overlap:
                    return True, 0.0
                return False, max(a0, b0) - min(a1, b1)

            # Two translation arrivals at the same intersection.
            for ea in a_sim.events:
                for eb in b_sim.events:
                    if ea.node != eb.node:
                        continue
                    gap = abs(ea.time_sec - eb.time_sec)
                    if gap <= INTERSECTION_CLEARANCE_SEC:
                        add(Conflict(
                            type="intersecting edges",
                            time_sec=min(ea.time_sec, eb.time_sec),
                            robots=(a_name, b_name), key=ea.node,
                            gap_sec=gap,
                            clearance_sec=INTERSECTION_CLEARANCE_SEC,
                            detail=(f"Intersection {ea.node}: {a_name} arrives "
                                    f"{ea.time_sec:.1f}s, {b_name} arrives "
                                    f"{eb.time_sec:.1f}s (gap {gap:.1f}s <= "
                                    f"{INTERSECTION_CLEARANCE_SEC:.1f}s)"),
                            info={
                                a_name: {"time": ea.time_sec, "entry": ea.time_sec,
                                         "end": ea.time_sec,
                                         "plan_index": ea.plan_index},
                                b_name: {"time": eb.time_sec, "entry": eb.time_sec,
                                         "end": eb.time_sec,
                                         "plan_index": eb.plan_index},
                            }))

            # Arrival/passage into a node which the other robot occupies.
            for event_name, event_sim, occupied_name, occupied_sim in (
                    (a_name, a_sim, b_name, b_sim),
                    (b_name, b_sim, a_name, a_sim)):
                for event in event_sim.events:
                    for occupancy in occupied_sim.node_intervals:
                        if event.node != occupancy.node:
                            continue
                        if event.time_sec < occupancy.start_sec:
                            gap = occupancy.start_sec - event.time_sec
                        elif event.time_sec > occupancy.end_sec:
                            gap = event.time_sec - occupancy.end_sec
                        else:
                            gap = 0.0
                        if gap > NODE_CLEARANCE_SEC:
                            continue
                        info = {
                            event_name: {
                                "time": event.time_sec, "entry": event.time_sec,
                                "start": event.time_sec, "end": event.time_sec,
                                "plan_index": event.plan_index,
                            },
                            occupied_name: {
                                "time": occupancy.start_sec,
                                "entry": occupancy.start_sec,
                                "start": occupancy.start_sec,
                                "end": occupancy.end_sec,
                                "plan_index": occupancy.plan_index,
                            },
                        }
                        add(Conflict(
                            type="same node occupancy",
                            time_sec=min(event.time_sec, occupancy.start_sec),
                            robots=(a_name, b_name), key=event.node,
                            gap_sec=gap, clearance_sec=NODE_CLEARANCE_SEC,
                            detail=(f"Node {event.node}: {event_name} passage "
                                    f"{event.time_sec:.1f}s conflicts with "
                                    f"{occupied_name} occupancy "
                                    f"{occupancy.start_sec:.1f}-"
                                    f"{occupancy.end_sec:.1f}s"),
                            info=info))

            # Stationary occupancy windows at the same node.
            for na in a_sim.node_intervals:
                for nb in b_sim.node_intervals:
                    if na.node != nb.node:
                        continue
                    overlap, gap = interval_gap(
                        na.start_sec, na.end_sec, nb.start_sec, nb.end_sec)
                    if not overlap and gap > NODE_CLEARANCE_SEC:
                        continue
                    add(Conflict(
                        type="same node occupancy",
                        time_sec=max(na.start_sec, nb.start_sec) if overlap
                        else min(na.end_sec, nb.end_sec),
                        robots=(a_name, b_name), key=na.node,
                        gap_sec=gap, clearance_sec=NODE_CLEARANCE_SEC,
                        detail=(f"Node {na.node}: {a_name} occupancy "
                                f"{na.start_sec:.1f}-{na.end_sec:.1f}s vs "
                                f"{b_name} {nb.start_sec:.1f}-{nb.end_sec:.1f}s"),
                        info={
                            a_name: {"entry": na.start_sec, "start": na.start_sec,
                                     "end": na.end_sec,
                                     "plan_index": na.plan_index},
                            b_name: {"entry": nb.start_sec, "start": nb.start_sec,
                                     "end": nb.end_sec,
                                     "plan_index": nb.plan_index},
                        }))

            # Edge/corridor resources.
            for ia in a_sim.intervals:
                for ib in b_sim.intervals:
                    if ia.from_node == ia.to_node or ib.from_node == ib.to_node:
                        continue
                    overlap, gap = interval_gap(
                        ia.start_sec, ia.end_sec, ib.start_sec, ib.end_sec)
                    conflict_type = None
                    key = ia.key
                    clearance = 0.0
                    if ia.depot_slot and ia.depot_slot == ib.depot_slot:
                        conflict_type = "depot slot"
                        key = ia.depot_slot
                        clearance = DEPOT_SLOT_CLEARANCE_SEC
                    elif ia.key == ib.key:
                        same_direction = (
                            ia.from_node == ib.from_node
                            and ia.to_node == ib.to_node)
                        conflict_type = (
                            "same-direction edge"
                            if same_direction else "opposite-direction edge")
                        clearance = (
                            SAME_DIRECTION_HEADWAY_SEC if same_direction
                            else OPPOSITE_DIRECTION_CLEARANCE_SEC)
                    elif (ia.shared_depot_entrance
                          and ib.shared_depot_entrance):
                        conflict_type = "shared depot entrance"
                        key = "depot:shared-entrance"
                        clearance = DEPOT_ENTRANCE_CLEARANCE_SEC
                    if conflict_type is not None and (
                            overlap or gap <= clearance):
                        add(Conflict(
                            type=conflict_type,
                            time_sec=max(ia.start_sec, ib.start_sec) if overlap
                            else min(ia.end_sec, ib.end_sec),
                            robots=(a_name, b_name), key=key,
                            gap_sec=gap, clearance_sec=clearance,
                            detail=(f"{conflict_type} {key}: {a_name} "
                                    f"{ia.start_sec:.1f}-{ia.end_sec:.1f}s vs "
                                    f"{b_name} {ib.start_sec:.1f}-"
                                    f"{ib.end_sec:.1f}s (gap {gap:.1f}s, "
                                    f"clearance {clearance:.1f}s)"),
                            info={
                                a_name: {
                                    "entry": ia.start_sec, "start": ia.start_sec,
                                    "end": ia.end_sec,
                                    "from": ia.from_node, "to": ia.to_node,
                                    "plan_index": ia.plan_index,
                                },
                                b_name: {
                                    "entry": ib.start_sec, "start": ib.start_sec,
                                    "end": ib.end_sec,
                                    "from": ib.from_node, "to": ib.to_node,
                                    "plan_index": ib.plan_index,
                                },
                            }))

                    # Different physical edges can still meet at one node.
                    if ia.key == ib.key:
                        continue
                    shared_nodes = ({ia.from_node, ia.to_node}
                                    & {ib.from_node, ib.to_node})
                    for node in shared_nodes:
                        ta = ia.start_sec if node == ia.from_node else ia.end_sec
                        tb = ib.start_sec if node == ib.from_node else ib.end_sec
                        passage_gap = abs(ta - tb)
                        if passage_gap > INTERSECTION_CLEARANCE_SEC:
                            continue
                        add(Conflict(
                            type="intersecting edges",
                            time_sec=min(ta, tb), robots=(a_name, b_name),
                            key=node, gap_sec=passage_gap,
                            clearance_sec=INTERSECTION_CLEARANCE_SEC,
                            detail=(f"Edges meet at node {node}: {a_name} "
                                    f"passes {ta:.1f}s, {b_name} passes "
                                    f"{tb:.1f}s"),
                            info={
                                a_name: {"entry": ta, "start": ta, "end": ta,
                                         "plan_index": ia.plan_index},
                                b_name: {"entry": tb, "start": tb, "end": tb,
                                         "plan_index": ib.plan_index},
                            }))

    conflicts.sort(key=lambda c: c.time_sec)
    return conflicts


# --- Advisor -------------------------------------------------------------------

@dataclass
class YieldPlan:
    """One robot's step-aside, chosen by resolve_deadlock().

    Deliberately inert: it names a robot and a node and explains why. The
    supervisor reserves the node and drives the maneuver; keeping the
    decision free of side effects is what lets the whole policy be tested
    without ROS or robots.
    """
    robot: str
    kind: str               # "sidestep" | "backtrack"
    from_node: object
    to_node: object
    blocked_robot: str
    reason: str


@dataclass
class Decision:
    command: Optional[str]         # what to publish next (None = plan complete)
    # Legacy API field. Collision waits no longer issue inserted DWELLs.
    inserted_dwell_ms: Optional[int] = None
    reason: str = ""
    conflicts: list = field(default_factory=list)
    llm_used: bool = False
    supervisor_wait: bool = False  # idle locally; publish no robot command
    retry_after_sec: float = COLLISION_RECHECK_SEC
    selected_candidate_ms: Optional[int] = None
    deadlock: bool = False


#: A robot is judged to have physically left a node once its measured centre
#: is further than this from it, in INCHES. Contact between two robots is
#: 5.04in centre-to-centre (a 15cm-hypotenuse robot, see
#: camera_grid_navigate.py's depot note), so this clears contact with margin.
#: The measurement itself is made by the supervisor, which is the side that
#: can resolve depot labels to world coordinates -- label_to_cell() returns
#: None for 'DE3' and friends.
STANDING_VACATE_IN = 6.0
#: How long AFTER the robot is first seen clear before its hold is dropped.
#: Requested explicitly by the operator on 2026-09-29 following the Alvik5
#: into Alvik3 collision at DE3.
STANDING_LINGER_SEC = 0.5


@dataclass(frozen=True)
class MovementRequest:
    segments: tuple
    destination_nodes: frozenset
    directed_edges: frozenset
    edge_keys: frozenset
    shared_depot_entrance: bool
    depot_slots: frozenset


@dataclass
class Reservation:
    robot: str
    plan_index: int
    request: MovementRequest
    granted_seq: int
    granted_sec: float


@dataclass
class RobotState:
    name: str
    plan: list                      # list[PlanItem]
    launch_offset_sec: float
    next_index: int = 0
    current_node: Any = "depot"
    last_done_sec: Optional[float] = None
    dispatched_sec: Optional[float] = None  # when the pending command was sent
                                            # (feeds timing calibration)
    launched: bool = False
    # what decide() handed out and is awaiting completion for:
    pending: Optional[tuple] = None  # ("plan", idx) | ("dwell", ms)
    last_fallback_sig: Optional[frozenset] = None
    current_edge: Optional[tuple[str, str]] = None
    pending_edges: tuple = ()
    request_seq: Optional[int] = None
    request_plan_index: Optional[int] = None
    last_wait_sig: Optional[frozenset] = None
    last_selected_candidate_ms: Optional[int] = None
    #: The node this robot is PHYSICALLY standing on after completing a move,
    #: held until vision shows it has left plus STANDING_LINGER_SEC. None when
    #: it holds nothing. See _hold_standing()/note_standing_clearance().
    standing_node: Any = None
    #: When vision FIRST showed it clear of standing_node; None while it is
    #: still sitting there.
    standing_clear_since: Optional[float] = None
    #: While a robot moves from one node to the next, the node it LEFT stays
    #: held for STANDING_LINGER_SEC past its arrival at the new one, so the
    #: vacated node is not handed over the instant the mover touches down.
    standing_linger_node: Any = None
    standing_linger_until: Optional[float] = None


class DispatchAdvisor:
    """Event-driven advisor. The supervisor calls decide(robot, now) whenever
    a robot is idle and ready for its next command, and record_done(robot,
    now) when the command it was given completes."""

    def __init__(self, schedule: dict, robot_commands: dict[str, list[str]],
                 llm=None, log_path: Optional[str] = None,
                 fallback_dwell_ms: int = DEFAULT_DWELL_MS,
                 calibration_path: Optional[str] = None,
                 log_fn=None):
        self.schedule = schedule
        self.llm = llm
        self.log_path = log_path
        self.fallback_dwell_ms = fallback_dwell_ms
        self.calibration = TimingCalibration(calibration_path)
        # Optional human-readable sink (e.g. the supervisor's ROS logger) so LLM
        # activity is visible in the terminal, not only in advisor_audit.jsonl.
        # Counters back an end-of-run summary answering "did the LLM do anything".
        self.log_fn = log_fn
        self.llm_stats = {
            "consultations": 0, "llm_pass": 0, "llm_reject": 0,
            "gate_fallback": 0, "llm_error": 0, "no_llm_wait": 0,
            "tie_breaks": 0,
        }
        # Cache of LLM priority tie-breaks, keyed by the unordered robot pair +
        # resource, so _priority_owner returns the SAME owner no matter which
        # robot is currently deciding. Without this, A could yield to B while B
        # yields to A -> deadlock. Cleared implicitly per mission (new advisor).
        self._tie_break_cache: dict[tuple, str] = {}
        self.reservations: dict[str, Reservation] = {}
        self.wait_for: dict[str, str] = {}
        # Corridor blocks seen by the SUPERVISOR'S camera gate, as
        # robot -> (blocker, first_seen, last_seen). Kept separate from
        # wait_for on purpose: wait_for is the reservation model's own
        # bookkeeping and is popped in a dozen places during decide(), so
        # camera-derived edges written there would be erased on the next
        # tick. These are merged only when walking for cycles.
        #
        # WHY THIS EXISTS (2026-09-29). The vision corridor gate is perfectly
        # symmetric and has no tie-breaker: if Alvik2 sits near node 19 and
        # Alvik5 wants node 19, and vice versa, each holds for the other
        # forever. Observed live -- the pair alternated "still on/near node
        # 19 in my path" every ~1.5s for 70 seconds until the operator
        # aborted. deadlock_cycles() could not see it, because a corridor
        # hold is not a reservation: the wait-for graph was EMPTY for that
        # pair while the robots were plainly deadlocked.
        self.vision_wait_for: dict[str, tuple] = {}
        self._request_counter = 0
        self._reservation_counter = 0
        rows = schedule.get("grid", {}).get("rows", 8)
        cols = schedule.get("grid", {}).get("cols", 8)
        self.rows, self.cols = rows, cols

        routes_by_agv = {r["agv"]: r for r in schedule.get("routes", [])}
        # Robots currently standing aside for a deadlock yield. Their PLAN is
        # deliberately left untouched (see end_yield), so the plan still says
        # they will traverse their next edge on the original schedule -- which
        # is exactly what _simulate_fleet() would otherwise feed into
        # detect_conflicts(). See _simulate_fleet for the livelock that caused.
        self.yielding_robots: set[str] = set()
        self.robots: dict[str, RobotState] = {}
        for name, commands in robot_commands.items():
            if name not in routes_by_agv:
                raise AlignmentError(
                    f"robot '{name}' has no route in the schedule export "
                    f"(available: {sorted(routes_by_agv)})")
            route = routes_by_agv[name]
            plan = align_commands_to_route(route, commands, rows, cols)
            initial_node: Any = "depot"
            for item in plan:
                segs = movement_segments(item)
                if segs:
                    initial_node = segs[0][0]
                    break
            self.robots[name] = RobotState(
                name=name, plan=plan,
                launch_offset_sec=float(route.get("launch_offset_sec", 0.0)),
                current_node=initial_node)
        for agv in routes_by_agv:
            if agv not in self.robots:
                self._log({"event": "warn",
                           "detail": f"schedule route {agv} has no live robot; "
                                     f"it is NOT simulated"})

    # -- supervisor-facing API -------------------------------------------------

    def launch_offset(self, robot: str) -> float:
        return self.robots[robot].launch_offset_sec

    def is_finished(self, robot: str) -> bool:
        st = self.robots[robot]
        return st.pending is None and st.next_index >= len(st.plan)

    def record_done(self, robot: str, now_sec: float) -> None:
        st = self.robots[robot]
        if st.pending is None:
            return
        kind, val = st.pending
        if kind == "plan":
            item = st.plan[val]
            # Online self-calibration: fold predicted-vs-actual duration into
            # the per-robot timing model (moves/turns only; dwells are exact).
            if item.kind in ("move", "turn") and st.dispatched_sec is not None:
                predicted = (sum(d for _f, _t, d in movement_segments(item))
                             if item.kind == "move" else item.duration_sec)
                actual = now_sec - st.dispatched_sec
                sample = self.calibration.observe(
                    robot, item.kind, predicted, actual)
                if sample is not None:
                    self._log({"event": "calibration", "robot": robot,
                               "command": item.command, "kind": item.kind,
                               "predicted_sec": round(predicted, 2),
                               "actual_sec": round(actual, 2),
                               "sample_ratio": round(sample, 3),
                               "factor": self.calibration.factor(robot, item.kind),
                               "t": round(now_sec, 2)})
            if item.kind == "move":
                st.current_node = item.to_node
                self._hold_standing(robot, item.to_node, now_sec)
            st.next_index = val + 1
        st.pending = None
        st.dispatched_sec = None
        st.current_edge = None
        st.pending_edges = ()
        st.request_seq = None
        st.request_plan_index = None
        st.last_wait_sig = None
        st.last_selected_candidate_ms = None
        self.wait_for.pop(robot, None)
        st.last_done_sec = now_sec
        st.launched = True

    def cancel_dispatch(self, robot: str) -> None:
        """Undo a just-granted movement before it is published.

        The supervisor uses this when fresh vision rejects a deterministic
        candidate. No command was sent, so the plan index and schedule anchor
        remain unchanged.

        Also reused (2026-07-31) when a fused turn-then-drive job times out
        or fails: try_authorize_next_move() grants a real reservation for
        the next move ahead of it actually being dispatched/completed, and
        this is the release valve for that grant on failure. Safe here
        specifically because the caller always also sets ch.errored = True
        in the same branch, permanently excluding the robot from
        run_advised_vision()'s unfinished list -- nothing ever reads this
        robot's advisor state again, so clearing st.pending along with the
        reservation (this method's normal behavior) has no other effect.
        """
        st = self.robots[robot]
        if st.pending and st.pending[0] == "plan":
            st.pending = None
        st.dispatched_sec = None  # nothing was sent — no calibration sample
        self._release_reservation(robot)
        st.current_edge = None
        st.pending_edges = ()

    def suspend(self, robot: str):
        """A command failed mid-flight (LINE_LOST / runaway stop) and the
        supervisor is starting vision-guided recovery. Detach the in-flight
        plan item so record_done() no-ops for the recovery maneuvers, and
        return its plan index (None if nothing plan-tracked was in flight)."""
        st = self.robots[robot]
        idx = st.pending[1] if st.pending and st.pending[0] == "plan" else None
        st.pending = None
        st.dispatched_sec = None  # failed command — never a calibration sample
        self._release_reservation(robot)
        st.current_edge = None
        st.pending_edges = ()
        st.request_seq = None
        st.request_plan_index = None
        self.wait_for.pop(robot, None)
        self._log({"event": "recovery_suspend", "robot": robot,
                   "failed_index": idx})
        return idx

    def resync(self, robot: str, plan_index: int, node_label: Any,
               now_sec: float) -> None:
        """Recovery finished: the robot is verified (by vision) at node_label
        with the right heading. Resume the plan at plan_index, re-anchored to
        now — the next decide() call re-runs the full conflict gate as if the
        robot had just completed a command there."""
        st = self.robots[robot]
        st.pending = None
        st.next_index = plan_index
        st.current_node = node_label
        self._release_reservation(robot)
        st.current_edge = None
        st.pending_edges = ()
        st.request_seq = None
        st.request_plan_index = None
        st.last_wait_sig = None
        st.last_selected_candidate_ms = None
        self.wait_for.pop(robot, None)
        st.last_done_sec = now_sec
        st.last_fallback_sig = None
        self._log({"event": "recovery_resync", "robot": robot,
                   "resume_index": plan_index, "node": node_label,
                   "t": round(now_sec, 2)})

    def decide(self, robot: str, now_sec: float) -> Decision:
        # Expire lazily rather than relying on anyone to tick us: every path
        # that can be blocked by a standing hold comes through here or
        # try_authorize_next_move(), so the linger cannot outlive its deadline
        # just because a caller forgot. The supervisor also ticks it, for the
        # log line; that is belt and braces, not the mechanism.
        self.expire_standing_linger(now_sec)
        st = self.robots[robot]
        if st.next_index >= len(st.plan):
            return Decision(command=None, reason="plan complete")

        item = st.plan[st.next_index]
        if item.kind != "move":
            # A TURN AT A BAY ENTRY IS NOT FREE. Everything below this used to
            # dispatch every non-move immediately with "movement gate
            # deferred" -- no resource check of any kind. That is exactly how
            # the 2026-09-30 collision happened: the rotating robot was
            # stationary, so the closing-speed guard had nothing to act on,
            # and the reservation layer was never consulted because a turn is
            # not a move. See entry_rotation_footprint() for the measurements.
            if item.kind == "turn":
                blocked_by = self._entry_rotation_blocker(st)
                if blocked_by is not None:
                    other, node = blocked_by
                    # Record it so deadlock_cycles() can see a standoff of two
                    # robots each waiting to rotate; without the edge the graph
                    # is empty while both are stuck.
                    self.wait_for[st.name] = other
                    return self._wait_decision(
                        st,
                        f"rotation at entry {st.current_node} would sweep "
                        f"into {other} on node {node}",
                        conflicts=[{"type": "entry_rotation",
                                    "robot": st.name, "with": other,
                                    "node": str(node)}])
            # Gate exactly one command. A turn remains node occupancy and is
            # dispatched before the following movement is considered.
            self._clear_request(st)
            st.last_wait_sig = None
            st.last_selected_candidate_ms = None
            return self._send_planned(
                st, f"immediate {item.kind}; movement gate deferred",
                now_sec=now_sec)

        self._register_request(st)
        request = self._movement_request(item)

        # Stand on the current node while this move's own fate is decided
        # below -- see _grant_standing_reservation()'s docstring for the
        # real hardware collision this closes. Overwritten below by
        # _grant_reservation() the moment this move itself is cleared, so a
        # robot that's about to move immediately gets its real (in-flight)
        # reservation instead, same as before this fix.
        self._grant_standing_reservation(robot, now_sec)

        lane_block = self._depot_lane_blocker(robot, request)
        if lane_block is not None:
            other, node = lane_block
            self.wait_for[robot] = other
            return self._wait_decision(
                st, f"depot lane: {other} has not cleared {node} yet",
                conflicts=[{"type": "depot_lane", "robot": robot,
                            "with": other, "node": str(node)}])

        swap_with = self._opposite_edge_swap(robot, request)
        if swap_with is not None:
            self.wait_for[robot] = swap_with
            reason = (
                f"deadlock: opposite-direction edge swap with {swap_with}; "
                "rerouting or backtracking required")
            self._log({"event": "deadlock", "robot": robot, "t": now_sec,
                       "waits_for": swap_with, "reason": reason})
            return self._wait_decision(st, reason, deadlock=True)

        reserved = self._reservation_blocker(robot, request)
        if reserved is not None:
            other, resource = reserved
            self.wait_for[robot] = other
            return self._wait_decision(
                st, f"resource {resource} reserved by {other}")

        fleet = self._simulate_fleet(deciding=robot, now_sec=now_sec)
        conflicts = detect_conflicts(fleet)
        mine = [c for c in conflicts
                if robot in c.robots
                and c.info[robot]["plan_index"] == st.next_index]
        losing = self._losing_conflicts(robot, mine)

        if not losing:
            st.last_fallback_sig = None
            st.last_wait_sig = None
            st.last_selected_candidate_ms = None
            self.wait_for.pop(robot, None)
            self._grant_reservation(robot, st.next_index, request, now_sec)
            self._log({"event": "clear", "robot": robot, "t": now_sec,
                       "next": st.plan[st.next_index].command,
                       "owned_conflicts": [c.detail for c in mine],
                       "fleet_conflicts": [c.detail for c in conflicts],
                       "reservation": self._request_summary(request)})
            reason = ("resource priority granted"
                      if mine else "immediate movement resources clear")
            return self._send_planned(st, reason, now_sec=now_sec)

        sig = frozenset(
            (c.type, c.key,
             c.robots[0] if c.robots[1] == robot else c.robots[1])
            for c in losing)

        if st.last_wait_sig == sig:
            self._log({"event": "hold_repeat", "robot": robot, "t": now_sec,
                       "conflicts": [c.detail for c in losing]})
            return self._wait_decision(
                st, "same conflict; supervisor recheck (LLM skipped)",
                conflicts=losing,
                selected_candidate_ms=st.last_selected_candidate_ms)

        candidates = self._build_candidates(robot, now_sec, losing)
        return self._consult_llm(st, now_sec, losing, candidates, sig)

    # -- internals ---------------------------------------------------------------

    def _send_planned(self, st: RobotState, reason: str,
                      now_sec: Optional[float] = None) -> Decision:
        idx = st.next_index
        st.pending = ("plan", idx)
        st.dispatched_sec = now_sec  # calibration measures from here to done
        self.wait_for.pop(st.name, None)
        if st.plan[idx].kind != "move":
            self._clear_request(st)
        return Decision(command=st.plan[idx].command, reason=reason)

    def _wait_decision(self, st: RobotState, reason: str, conflicts=None,
                       llm_used: bool = False,
                       selected_candidate_ms: Optional[int] = None,
                       deadlock: bool = False) -> Decision:
        # No pending command: the supervisor must not publish or record a
        # completion for this local wait.
        st.last_selected_candidate_ms = selected_candidate_ms
        return Decision(
            command=None, reason=reason, conflicts=conflicts or [],
            llm_used=llm_used, supervisor_wait=True,
            retry_after_sec=COLLISION_RECHECK_SEC,
            selected_candidate_ms=selected_candidate_ms, deadlock=deadlock)

    def _entry_rotation_blocker(self, st):
        """Who, if anyone, a rotation at st.current_node would sweep into.

        Returns (robot, node) or None. Considers both where other robots
        ARE and where they are heading: a robot already granted a move onto
        an adjacent lattice node will be there before the turn finishes, so
        letting the turn start would just move the collision a second later.
        """
        foot = entry_rotation_footprint(st.current_node, self.rows, self.cols)
        if not foot:
            return None
        here = str(st.current_node)
        for other, ost in self.robots.items():
            if other == st.name:
                continue
            occupied = {str(ost.current_node)} if ost.current_node is not None else set()
            res = self.reservations.get(other)
            if res is not None:
                occupied |= {str(x) for x in res.request.destination_nodes}
            hit = (occupied & foot) - {here}
            if hit:
                return other, sorted(hit)[0]
        return None


    def _depot_lane_blocker(self, robot: str, request):
        """Who, if anyone, is still on a depot-lane node this move wants.

        THE DEPOT LANE IS SINGLE FILE. It is one straight corridor with the
        slots hanging off it, so a robot may enter a lane node only once the
        robot ahead has ARRIVED at the next one -- not merely started to
        leave. Operator, 2026-09-30:

            Alvik6 cannot move forward to DE5 until Alvik5 has reached DE4,
            Alvik5 cannot move forward to DE4 until Alvik4 has reached DE3,
            ... and once Alvik1 has reached node 1, Alvik2 moves to node 0.

        WHY THE EXISTING MACHINERY DID NOT DO THIS. Two separate gaps.
        First, the HTML cascade expressed the sequencing as per-segment
        waits (`wait_before_segment_sec`) and NOTHING IN THE SUPERVISOR
        READS THAT FIELD -- it is in the mission payload and ignored, so the
        whole schedule was simulation-only and never reached a robot.
        Second, a reservation follows the move: the instant a robot is
        granted DE5 -> DE4 its hold shifts to DE4, freeing DE5 while it is
        still physically sitting there. The follower then sees a free node
        and drives into it. On hardware 2026-09-30 that put Alvik3, 4, 5 and
        6 within 3.8-5.3in of each other in the lane -- inside the 5.625in
        contact distance.

        So a robot in flight keeps occupying the node it LEFT until it has
        arrived. That is the whole rule, and it is what makes the cascade a
        physical guarantee rather than a timing hope.
        """
        wanted = {str(d) for d in request.destination_nodes
                  if is_depot_lane_label(d)}
        if not wanted:
            return None
        # SCOPED TO THE LAUNCH. Only a robot that is itself still in the depot
        # region is sequenced this way -- that is the cascade the operator
        # specified, and the case that bunched four robots together on
        # hardware 2026-09-30.
        #
        # A robot arriving from the GRID is left to the space-time model as
        # before (see test_depot_follower_can_approach_node_zero_while_
        # leader_clears_it): that path predicts the leader will have cleared
        # node 0 by the time the follower covers its 4.2s approach, and it is
        # deliberate, tested behaviour. Worth revisiting -- the same
        # prediction is what this gate exists to stop trusting -- but not
        # worth changing silently as a side effect of the launch work.
        st = self.robots.get(robot)
        here = str(st.current_node) if st is not None else ""
        if not (is_depot_lane_label(here) or re.fullmatch(r"D\d+", here.upper())):
            return None
        for other, ost in self.robots.items():
            if other == robot:
                continue
            held = set()
            if ost.current_node is not None:
                held.add(str(ost.current_node))
            # In flight: BOTH ends are occupied until the move completes.
            for edge in (ost.pending_edges or ()):
                try:
                    held.add(str(edge[0]))
                    held.add(str(edge[1]))
                except (TypeError, IndexError):
                    continue
            if ost.current_edge:
                held.add(str(ost.current_edge[0]))
                held.add(str(ost.current_edge[1]))
            hit = held & wanted
            if hit:
                return other, sorted(hit)[0]
        return None

    def _movement_request(self, item: PlanItem) -> MovementRequest:
        segments = tuple(
            (str(frm), str(to), float(duration))
            for frm, to, duration in movement_segments(item))
        return MovementRequest(
            segments=segments,
            destination_nodes=frozenset(to for _frm, to, _dur in segments),
            directed_edges=frozenset(
                directed_edge_key(frm, to) for frm, to, _dur in segments),
            edge_keys=frozenset(
                corridor_key(frm, to, self.rows, self.cols)
                for frm, to, _dur in segments),
            shared_depot_entrance=any(
                _uses_shared_depot_entrance(frm, to)
                for frm, to, _dur in segments),
            depot_slots=frozenset(
                slot for frm, to, _dur in segments
                if (slot := _depot_slot_label(frm, to)) is not None),
        )

    def _standing_request(self, node: Any) -> MovementRequest:
        """A zero-length MovementRequest representing a robot PARKED at
        `node` (not traveling) -- destination_nodes is the only populated
        field that matters here, since a stationary robot occupies exactly
        that one node, no edge or depot-entrance resource. Used by
        _grant_standing_reservation() to close the gap described there."""
        label = str(node)
        return MovementRequest(
            segments=(), destination_nodes=frozenset({label}),
            directed_edges=frozenset(), edge_keys=frozenset(),
            shared_depot_entrance=False,
            depot_slots=frozenset({label}) if re.fullmatch(r"D\d+", label.upper())
            else frozenset(),
        )

    def _grant_standing_reservation(self, robot: str, now_sec: float) -> None:
        """ROOT-CAUSED 2026-08-24 (Alvik2/Alvik3 collision, 4-robot hardware
        run): self.reservations only ever held a robot's IN-FLIGHT move --
        the moment a move completes, record_done() calls
        _release_reservation() unconditionally, even if the robot is now
        sitting stationary, blocked from its NEXT move by another robot's
        reservation (exactly what happened to Alvik3, parked at DE2 and
        SUPERVISOR-WAITing on Alvik1 for 34+ seconds). With no reservation
        on the node it physically occupies, nothing in _reservation_blocker()
        can see Alvik3 there -- Alvik2 was granted a move whose destination
        was DE2 while Alvik3 sat on it. Compounding this: the vision-based
        corridor gate (_vision_hold_reason() in fleetSupervisor.py)
        deliberately skips depot-region nodes by design (2026-07-21, to
        avoid deadlocking normal depot-lane funneling) -- so a parked robot
        in the depot lane had NEITHER a logical reservation NOR a live
        vision check protecting it. This closes the logical half of that
        gap: any time a robot is genuinely stationary (not mid-move, i.e.
        st.pending is None) with a real current_node, it holds a standing
        reservation on that node, refreshed every decide() call, so
        _reservation_blocker() sees it exactly like an in-flight move would.
        Cleared automatically the instant a REAL move reservation is granted
        (_grant_reservation() overwrites self.reservations[robot], the same
        one-entry-per-robot slot) -- no separate release needed, and this
        never fires for a robot with pending_edges (already covered by its
        real reservation), so it can't shadow or conflict with that path.
        Does NOT touch the vision corridor gate's depot-region exclusion --
        a separate, larger design decision from 2026-07-21 this fix
        deliberately leaves alone."""
        st = self.robots[robot]
        if st.pending is not None or st.pending_edges:
            return  # already covered by a real in-flight reservation
        if st.current_node is None:
            return  # never launched -- nothing to stand on
        self._reservation_counter += 1
        self.reservations[robot] = Reservation(
            robot=robot, plan_index=st.next_index,
            request=self._standing_request(st.current_node),
            granted_seq=self._reservation_counter, granted_sec=now_sec)

    def immediate_movement_request(self, robot: str) -> Optional[MovementRequest]:
        """Read-only view used by the supervisor's physical occupancy gate."""
        return self._immediate_request_for_state(self.robots[robot])

    def next_move_request(self, robot: str) -> Optional[MovementRequest]:
        """The move AFTER the current one (skipping turns/dwells), for the
        supervisor's two-node exit lookahead. None if there is no further move."""
        st = self.robots[robot]
        seen_move = False
        for idx in range(st.next_index, len(st.plan)):
            if st.plan[idx].kind != "move":
                continue
            if not seen_move:
                seen_move = True   # this is the current move; skip it
                continue
            return self._movement_request(st.plan[idx])
        return None

    def try_authorize_next_move(self, robot: str, next_plan_index: int,
                                now_sec: float) -> bool:
        """While ROBOT's CURRENT move is still pending (mid-flight, not yet
        record_done()'d), check whether the move at NEXT_PLAN_INDEX clears
        the deterministic conflict gate RIGHT NOW (assuming the current move
        completes exactly as planned) -- and if so, GRANT it a real
        reservation immediately, under the current move's own in-flight
        reservation slot, so no other robot can be granted that resource
        during the fused window. Returns True iff authorized (and granted).

        Added 2026-07-31 for fleetSupervisor.py's vision-drive turn-then-
        drive fusion: a worker hands off directly from a turn into the NEXT
        leg's drive_leg() (no brake/settle in between -- see
        turn_then_drive_leg() in camera_grid_navigate.py). That must not
        bypass the collision-safety reservation system, so the supervisor
        calls this FIRST; only on True does it tell the worker to fuse, and
        it MUST call record_done_fused() (not two separate record_done()
        calls) once the fused drive actually completes -- see that method's
        docstring for why a plain record_done() would silently release the
        reservation this grants.

        IMPORTANT: on True, this leaves the advisor in a state where BOTH
        the current move's original reservation-tracking AND the granted
        next-move reservation coexist under self.reservations[robot] (the
        next-move one overwrites/represents both, since only one robot can
        hold one reservation slot at a time and nothing else may be
        reserved for this robot while a move is pending anyway). On False,
        EVERY field touched is restored exactly, whether by the conflict
        check or by anything upstream, so a caller that never fuses (the
        overwhelming common case: two-robot+ missions, until this is tested
        against real contention) sees zero side effects.

        Deterministic-only: if the real decide() would need LLM tie-break
        consultation, this returns False (safe default). Never calls the
        LLM and never writes to the audit log."""
        self.expire_standing_linger(now_sec)
        st = self.robots[robot]
        if next_plan_index >= len(st.plan):
            return False
        item = st.plan[next_plan_index]
        if item.kind != "move":
            return False

        # Snapshot every field the deterministic gate (and a real grant) can
        # touch, on both this RobotState and the advisor itself, so a False
        # result leaves everything exactly as found.
        st_snapshot = {
            "next_index": st.next_index, "current_node": st.current_node,
            "current_edge": st.current_edge, "pending_edges": st.pending_edges,
            "request_seq": st.request_seq,
            "request_plan_index": st.request_plan_index,
            "last_wait_sig": st.last_wait_sig,
            "last_fallback_sig": st.last_fallback_sig,
            "last_selected_candidate_ms": st.last_selected_candidate_ms,
        }
        reservation_snapshot = self.reservations.get(robot)
        wait_for_snapshot = self.wait_for.get(robot)
        request_counter_snapshot = self._request_counter
        reservation_counter_snapshot = self._reservation_counter

        # Pretend the current move already landed at its own to_node --
        # movement_segments()[-1] is that move's final destination, matching
        # what record_done() itself sets current_node to.
        current_item = st.plan[st.next_index]
        segs = movement_segments(current_item)
        if segs:
            st.current_node = segs[-1][1]
        st.next_index = next_plan_index
        st.current_edge = None
        st.pending_edges = ()
        st.request_seq = None
        st.request_plan_index = None
        st.last_wait_sig = None

        self._register_request(st)
        request = self._movement_request(item)

        clear = (self._opposite_edge_swap(robot, request) is None
                 and self._reservation_blocker(robot, request) is None)
        if clear:
            fleet = self._simulate_fleet(deciding=robot, now_sec=now_sec)
            conflicts = detect_conflicts(fleet)
            mine = [c for c in conflicts
                   if robot in c.robots
                   and c.info[robot]["plan_index"] == next_plan_index]
            # A real decide() falls back to LLM consultation here if this is
            # non-empty -- this method never does; any conflict at all means
            # "not clear," full stop.
            clear = not self._losing_conflicts(robot, mine)

        if clear:
            # GRANT for real -- this is what makes the fused window safe.
            # st.next_index is already advanced to next_plan_index above, so
            # record_done_fused() (called once the driven leg finishes) can
            # release it exactly like a normal single-move completion.
            self._grant_reservation(robot, next_plan_index, request, now_sec)
            self.wait_for.pop(robot, None)
            self._log({"event": "fused_grant", "robot": robot, "t": now_sec,
                       "next": item.command,
                       "reservation": self._request_summary(request)})
            return True

        for name, value in st_snapshot.items():
            setattr(st, name, value)
        if reservation_snapshot is None:
            self.reservations.pop(robot, None)
        else:
            self.reservations[robot] = reservation_snapshot
        if wait_for_snapshot is None:
            self.wait_for.pop(robot, None)
        else:
            self.wait_for[robot] = wait_for_snapshot
        self._request_counter = request_counter_snapshot
        self._reservation_counter = reservation_counter_snapshot
        return False

    def record_done_fused(self, robot: str, current_plan_index: int,
                          fused_plan_index: int, now_sec: float) -> None:
        """Companion to try_authorize_next_move(): call this ONCE, instead
        of record_done(), when a turn-then-drive fusion actually completed
        (both the original pending move AND the move try_authorize_next_move
        granted). Folds BOTH items into timing calibration (mirroring
        record_done()'s own per-item calibration exactly) and advances
        next_index past both in one atomic step.

        Why not just call record_done() twice: the first call would see
        st.pending == ("plan", current_plan_index), correctly calibrate and
        advance to current_plan_index + 1 -- but it would ALSO call
        _release_reservation(robot), destroying the real reservation
        try_authorize_next_move() just granted for fused_plan_index before
        the robot has even finished physically driving it. A second robot's
        decide() running in the gap between those two record_done() calls
        could then be granted that same resource. This method releases the
        reservation exactly once, after both items are accounted for."""
        st = self.robots[robot]
        if st.pending is None or st.pending != ("plan", current_plan_index):
            return
        for idx in (current_plan_index, fused_plan_index):
            item = st.plan[idx]
            if item.kind == "move" and st.dispatched_sec is not None:
                predicted = sum(d for _f, _t, d in movement_segments(item))
                # Only the FIRST item's duration was actually timed from
                # dispatched_sec -- the fused second item has no independent
                # dispatch timestamp of its own (it was never separately
                # "sent"), so it is not fed into online calibration; folding
                # its predicted duration against the same elapsed window
                # would double-count time already attributed to the first.
                if idx == current_plan_index:
                    actual = now_sec - st.dispatched_sec
                    sample = self.calibration.observe(robot, "move", predicted, actual)
                    if sample is not None:
                        self._log({"event": "calibration", "robot": robot,
                                   "command": item.command, "kind": "move",
                                   "predicted_sec": round(predicted, 2),
                                   "actual_sec": round(actual, 2),
                                   "sample_ratio": round(sample, 3),
                                   "factor": self.calibration.factor(robot, "move"),
                                   "t": round(now_sec, 2), "fused": True})
            if item.kind == "move":
                st.current_node = item.to_node
        self._hold_standing(robot, st.current_node, now_sec)
        st.next_index = fused_plan_index + 1
        st.pending = None
        st.dispatched_sec = None
        st.current_edge = None
        st.pending_edges = ()
        st.request_seq = None
        st.request_plan_index = None
        st.last_wait_sig = None
        st.last_selected_candidate_ms = None
        self.wait_for.pop(robot, None)
        st.last_done_sec = now_sec
        st.launched = True
        self._log({"event": "fused_done", "robot": robot,
                   "current_index": current_plan_index,
                   "fused_index": fused_plan_index, "t": round(now_sec, 2)})

    def _request_summary(self, request: MovementRequest) -> dict:
        return {
            "destination_nodes": sorted(request.destination_nodes),
            "directed_edges": sorted(request.directed_edges),
            "shared_depot_entrance": request.shared_depot_entrance,
            "depot_slots": sorted(request.depot_slots),
        }

    def _register_request(self, st: RobotState) -> None:
        if (st.request_seq is not None
                and st.request_plan_index == st.next_index):
            return
        self._request_counter += 1
        st.request_seq = self._request_counter
        st.request_plan_index = st.next_index

    def _clear_request(self, st: RobotState) -> None:
        st.request_seq = None
        st.request_plan_index = None
        self.wait_for.pop(st.name, None)

    def _grant_reservation(self, robot: str, plan_index: int,
                           request: MovementRequest, now_sec: float) -> None:
        self._reservation_counter += 1
        self.reservations[robot] = Reservation(
            robot=robot, plan_index=plan_index, request=request,
            granted_seq=self._reservation_counter, granted_sec=now_sec)
        st = self.robots[robot]
        st.pending_edges = tuple(
            (frm, to) for frm, to, _dur in request.segments)
        st.current_edge = st.pending_edges[0] if st.pending_edges else None

    def _release_reservation(self, robot: str) -> None:
        self.reservations.pop(robot, None)
        st = self.robots.get(robot)
        if st is not None:
            st.standing_node = None
            st.standing_clear_since = None
            st.standing_linger_node = None
            st.standing_linger_until = None

    def _hold_standing(self, robot: str, node, now_sec: float) -> None:
        """Arriving at a node does NOT free it -- the robot is standing on it.

        Replaces the completed move's reservation with a node-only hold on
        where the robot actually is, kept until vision shows it has physically
        left (note_standing_clearance()).

        WHY. Until 2026-09-29 record_done() called _release_reservation() the
        moment a move completed, dropping the robot's whole reservation while
        it was still sitting on the destination. The next plan item is often a
        turn, which takes no reservation at all ("movement gate deferred"), so
        the robot held nothing while physically occupying the node. Hardware,
        that day, in the depot lane:

            .627  Alvik5 WAIT: resource edge DE2<>DE3 reserved by Alvik3
            .754  Alvik3 plan advances to a turn -- reservation dropped
            .757  Alvik5 granted move DE2->DE3, Alvik3 still standing on DE3
            .800  Alvik3 only NOW starts to leave

        Alvik5 drove into it and ended up at (36.2, 8.7), five inches off a
        lane whose y is 3. On the lattice a vision proximity gate usually
        covers that window ("still on/near node N ... rotating or not yet
        clear"); in the depot region that gate is deliberately skipped, so
        nothing did.

        The edge and corridor claims ARE dropped here -- the robot has
        finished traversing them and holding them would block the lane behind
        it. Only the node it is standing on is kept.
        """
        st = self.robots.get(robot)
        if st is None:
            return
        previous = self.reservations.get(robot)
        if node is None:
            self.reservations.pop(robot, None)
            st.standing_node = None
            st.standing_clear_since = None
            return
        # The node just vacated stays held for the linger. Arrival at the
        # next node is NOT proof the previous one is safe to hand over -- the
        # operator's requirement, 2026-09-29, after Alvik5 was granted DE3
        # 43ms before Alvik3 started to leave it.
        held = {str(node)}
        previous_node = st.standing_node
        if previous_node is not None and str(previous_node) != str(node):
            held.add(str(previous_node))
            st.standing_linger_node = str(previous_node)
            st.standing_linger_until = now_sec + STANDING_LINGER_SEC
        st.standing_node = node
        st.standing_clear_since = None
        self.reservations[robot] = Reservation(
            robot=robot,
            plan_index=previous.plan_index if previous is not None else -1,
            request=MovementRequest(
                segments=(),
                destination_nodes=frozenset(held),
                directed_edges=frozenset(),
                edge_keys=frozenset(),
                shared_depot_entrance=False,
                depot_slots=frozenset()),
            granted_seq=(previous.granted_seq if previous is not None
                         else self._reservation_counter),
            granted_sec=now_sec)

    def note_standing_clearance(self, robot: str, clear: bool, now_sec: float,
                                linger_sec: float = STANDING_LINGER_SEC
                                ) -> bool:
        """Feed MEASURED occupancy of the robot's standing node.

        `clear` is True when vision puts the robot more than
        STANDING_VACATE_IN from the node it is holding. The hold is dropped
        only once that has been true for linger_sec continuously -- a single
        frame saying "clear" is not enough to hand the node to a robot already
        driving at it. Returns True on the tick the hold is released.

        Anything that is not clear resets the timer, so a robot that drifts
        back over the node (or a momentary bad fix) re-arms the wait rather
        than banking partial credit.
        """
        st = self.robots.get(robot)
        if st is None or st.standing_node is None:
            return False
        if not clear:
            st.standing_clear_since = None
            return False
        if st.standing_clear_since is None:
            st.standing_clear_since = now_sec
            return False
        if now_sec - st.standing_clear_since < linger_sec:
            return False
        held = st.standing_node
        st.standing_node = None
        st.standing_clear_since = None
        reservation = self.reservations.get(robot)
        # Only drop it if it is still the node-only standing hold; a real
        # movement reservation granted since then must not be thrown away.
        if (reservation is not None
                and not reservation.request.edge_keys
                and reservation.request.destination_nodes == frozenset({str(held)})):
            self.reservations.pop(robot, None)
        self._log({"event": "standing_released", "robot": robot,
                   "node": str(held), "t": round(now_sec, 2)})
        return True

    def expire_standing_linger(self, now_sec: float) -> list:
        """Drop the extra hold on each robot's previously-vacated node once
        STANDING_LINGER_SEC has passed. Call once per supervisor tick.

        Returns the (robot, node) pairs released, for logging.
        """
        released = []
        for name, st in self.robots.items():
            if (st.standing_linger_until is None
                    or now_sec < st.standing_linger_until):
                continue
            node, st.standing_linger_node = st.standing_linger_node, None
            st.standing_linger_until = None
            reservation = self.reservations.get(name)
            # Only shrink a node-only standing hold; a real movement
            # reservation granted since then must be left alone.
            if (reservation is not None
                    and not reservation.request.edge_keys
                    and st.standing_node is not None):
                self.reservations[name] = Reservation(
                    robot=name,
                    plan_index=reservation.plan_index,
                    request=MovementRequest(
                        segments=(),
                        destination_nodes=frozenset({str(st.standing_node)}),
                        directed_edges=frozenset(),
                        edge_keys=frozenset(),
                        shared_depot_entrance=False,
                        depot_slots=frozenset()),
                    granted_seq=reservation.granted_seq,
                    granted_sec=reservation.granted_sec)
            if node is not None:
                released.append((name, node))
        return released

    def standing_holds(self) -> dict:
        """{robot: node} for every robot currently holding where it stands."""
        return {name: st.standing_node
                for name, st in self.robots.items()
                if st.standing_node is not None}

    def _reservation_overlap(self, a: MovementRequest,
                             b: MovementRequest) -> Optional[str]:
        if a.depot_slots & b.depot_slots:
            return f"depot slot {sorted(a.depot_slots & b.depot_slots)[0]}"
        if a.shared_depot_entrance and b.shared_depot_entrance:
            return "shared depot entrance"
        if a.edge_keys & b.edge_keys:
            return f"edge {sorted(a.edge_keys & b.edge_keys)[0]}"
        if a.destination_nodes & b.destination_nodes:
            return f"node {sorted(a.destination_nodes & b.destination_nodes)[0]}"
        return None

    def _reservation_blocker(
            self, robot: str,
            request: MovementRequest) -> Optional[tuple[str, str]]:
        for other, reservation in sorted(
                self.reservations.items(),
                key=lambda pair: pair[1].granted_seq):
            if other == robot:
                continue
            resource = self._reservation_overlap(request, reservation.request)
            if resource is not None:
                return other, resource
        return None

    def request_corridor_nodes(self, request: MovementRequest) -> set[str]:
        """Every grid node the requested move physically passes through — the
        segment endpoints PLUS entry nodes that sit mid-edge on a traversed
        grid lane (18->19 physically also occupies entry 122 on that lane)."""
        nodes: set[str] = set()
        for frm, to, _dur in request.segments:
            nodes.add(str(frm))
            nodes.add(str(to))
            flank = entry_edge_nodes(frm, self.rows, self.cols) or \
                entry_edge_nodes(to, self.rows, self.cols)
            # If this is a plain grid edge, add the entry node on it (if any).
            en = self._entry_on_edge(frm, to)
            if en is not None:
                nodes.add(str(en))
        return nodes

    def _entry_on_edge(self, a: Any, b: Any) -> Optional[int]:
        """The bay-entry node sitting on grid edge a<>b, or None. Inverse of
        entry_edge_nodes: scan the entry range for one whose flanks are {a,b}."""
        try:
            ai, bi = int(a), int(b)
        except (TypeError, ValueError):
            return None
        nodes = self.rows * self.cols
        bays = (self.rows - 1) * (self.cols - 1)
        for entry in range(nodes + bays + 1, nodes + 2 * bays + 1):
            flank = entry_edge_nodes(entry, self.rows, self.cols)
            if flank and set(flank) == {ai, bi}:
                return entry
        return None

    def is_workstation_or_depot_exit(self, robot: str) -> bool:
        """True when the robot's current move departs a workstation entry node
        or the depot (node 0 / a D# slot) — the dead-end corridors."""
        st = self.robots[robot]
        if st.next_index >= len(st.plan):
            return False
        item = st.plan[st.next_index]
        if item.kind != "move":
            return False
        frm = str(item.from_node)
        if frm in ("0",) or frm.upper().startswith("D"):
            return True
        try:
            n = int(frm)
        except (TypeError, ValueError):
            return False
        nodes = self.rows * self.cols
        bays = (self.rows - 1) * (self.cols - 1)
        return nodes + bays < n <= nodes + 2 * bays  # from an entry node

    def _immediate_request_for_state(
            self, st: RobotState) -> Optional[MovementRequest]:
        if st.next_index >= len(st.plan):
            return None
        item = st.plan[st.next_index]
        return self._movement_request(item) if item.kind == "move" else None

    def _opposite_edge_swap(
            self, robot: str,
            request: MovementRequest) -> Optional[str]:
        st = self.robots[robot]
        my_node = str(st.current_node)
        for other, ost in sorted(self.robots.items()):
            if other == robot or self._is_moving(ost):
                continue
            other_request = self._immediate_request_for_state(ost)
            if other_request is None:
                continue
            other_node = str(ost.current_node)
            for frm, to, _dur in request.segments:
                if frm != my_node:
                    continue
                for ofrm, oto, _odur in other_request.segments:
                    if (ofrm == other_node and frm == oto and to == ofrm
                            and to == other_node):
                        return other
        return None

    #: A camera hold must persist this long before it counts as a deadlock
    #: edge. The gates are re-evaluated every SUPERVISOR_WAIT_RECHECK_SEC
    #: (0.20s), so 1.0s is roughly five consecutive re-assertions -- enough
    #: to tell a standoff from ordinary traffic, where the lead robot really
    #: is still clearing.
    #:
    #: Lowered 6.0 -> 3.0 -> 1.0 across 2026-09-29 and 2026-10-06, each time
    #: after a live deadlock that ran to a manual abort: 70s, then 45s. The
    #: operator's requirement is that two robots never contend for the same
    #: node for more than about a second.
    #:
    #: It should not go much lower. A resolution drives a real robot past
    #: another one, so firing on ordinary traffic trades a brief wait for a
    #: collision risk.
    VISION_BLOCK_DEADLOCK_SEC = 1.0
    #: Drop a hold nobody has re-asserted; the gate re-states it every tick,
    #: so silence means it cleared.
    VISION_BLOCK_STALE_SEC = 4.0

    def note_vision_block(self, robot: str, blocker: str) -> None:
        """Record that the camera corridor gate is holding `robot` for
        `blocker`. Idempotent -- re-stating the same pair EXTENDS it rather
        than restarting the clock, which is what makes the age meaningful."""
        now = time.monotonic()
        prev = self.vision_wait_for.get(robot)
        if prev is not None and prev[0] == blocker:
            self.vision_wait_for[robot] = (blocker, prev[1], now)
        else:
            self.vision_wait_for[robot] = (blocker, now, now)

    def clear_vision_block(self, robot: str) -> None:
        self.vision_wait_for.pop(robot, None)

    def _effective_wait_for(self, robot: str):
        """wait_for, plus camera corridor holds that are both fresh and old
        enough to be a standoff rather than ordinary traffic.

        Reservation edges win: they are the model's own arbitration, and a
        robot held by BOTH should be attributed to the reservation so the
        resolution matches whatever actually gates it."""
        direct = self.wait_for.get(robot)
        if direct is not None:
            return direct
        entry = self.vision_wait_for.get(robot)
        if entry is None:
            return None
        blocker, first_seen, last_seen = entry
        now = time.monotonic()
        if now - last_seen > self.VISION_BLOCK_STALE_SEC:
            self.vision_wait_for.pop(robot, None)
            return None
        if now - first_seen < self.VISION_BLOCK_DEADLOCK_SEC:
            return None
        return blocker

    def _wait_cycle(self, robot: str) -> list[str]:
        path = [robot]
        current = self._effective_wait_for(robot)
        while current is not None and current not in path:
            path.append(current)
            current = self._effective_wait_for(current)
        if current is None:
            return []
        start = path.index(current)
        return path[start:] + [current]

    YIELD_HOP_SEC = 4.0

    def begin_yield(self, plan, now_sec: float):
        """Reserve a yield maneuver and return the two PlanItems to drive.

        Returns (out_item, back_item), or None if the target is no longer
        free. The caller must drive out_item, wait for the blocked robot to
        clear, drive back_item, then call end_yield().

        Re-checking here is not belt-and-braces. resolve_deadlock() decides
        from a snapshot; the reservation layer is what actually arbitrates,
        and between the two another robot can legitimately be granted the
        node we chose. Granting through the same path as any ordinary move
        is what makes the yield safe rather than merely careful.
        """
        st = self.robots.get(plan.robot)
        if st is None:
            return None
        here, target = st.current_node, plan.to_node
        if here is None or str(here) == str(target):
            return None

        occupied = self._occupied_nodes()
        holder = occupied.get(str(target))
        if holder is not None and holder != plan.robot:
            return None

        out_item = PlanItem(command="YIELD_OUT", kind="move",
                            duration_sec=self.YIELD_HOP_SEC,
                            from_node=here, to_node=target)
        back_item = PlanItem(command="YIELD_BACK", kind="move",
                             duration_sec=self.YIELD_HOP_SEC,
                             from_node=target, to_node=here)

        # Detach whatever the plan thought this robot was doing, then take a
        # real reservation on the leg. Without the release, the standing
        # reservation on `here` would keep the node the blocked robot needs.
        self.cancel_dispatch(plan.robot)
        request = self._movement_request(out_item)
        blocker = self._reservation_blocker(plan.robot, request)
        if blocker is not None:
            return None
        self._grant_reservation(plan.robot, -1, request, now_sec)
        st.current_edge = (str(here), str(target))
        self.yielding_robots.add(plan.robot)
        return out_item, back_item

    def finish_yield_leg(self, robot: str, at_node, now_sec: float) -> None:
        """Called after each yield leg lands, to move the reservation with
        the robot rather than leaving it on the leg it has finished."""
        st = self.robots.get(robot)
        if st is None:
            return
        st.current_node = at_node
        st.current_edge = None
        self._release_reservation(robot)
        self._grant_reservation(robot, -1, self._standing_request(at_node),
                                now_sec)

    def end_yield(self, robot: str, now_sec: float) -> None:
        """Hand the robot back to normal dispatch.

        The plan itself was never modified -- the robot is standing on the
        node it started from with the same next_index it had, so decide()
        resumes exactly where it left off.
        """
        self.yielding_robots.discard(robot)
        st = self.robots.get(robot)
        if st is None:
            return
        st.current_edge = None
        st.pending = None
        st.dispatched_sec = None
        self.wait_for.pop(robot, None)
        self._release_reservation(robot)
        if st.current_node is not None:
            self._grant_reservation(
                robot, -1, self._standing_request(st.current_node), now_sec)

    def strand(self, robot: str, now_sec: float) -> None:
        """Excluded robot: drop its movement claims, keep its footprint.

        An excluded robot is stationary and is never coming back, but it is
        still PHYSICALLY THERE. Those two facts need opposite handling and
        getting either one wrong breaks the fleet:

          * its edge/corridor reservations must go, or every robot behind it
            waits on a resource nothing will ever release;
          * its node reservation must STAY, or the planner happily routes a
            live robot through the square it is parked on.

        cancel_dispatch() alone does only the first half, which is why this
        exists separately.

        Hardware failure this fixes (2026-09-24 run, 13:58-13:59): Alvik5 was
        excluded on 'drive_leg failed on leg 1->0'. The exclusion path only
        called cancel_dispatch() when the failed job was a FUSED turn-then-
        drive; this was a plain drive, so nothing was released. Alvik3 then
        waited on "shared depot entrance reserved by Alvik5" for 38 seconds,
        Alvik6 queued behind Alvik3, and the run had to be aborted. One
        robot's failure took out three.
        """
        st = self.robots.get(robot)
        if st is None:
            return
        self.cancel_dispatch(robot)
        self.wait_for.pop(robot, None)
        self.yielding_robots.discard(robot)
        if st.current_node is not None:
            self._grant_reservation(
                robot, -1, self._standing_request(st.current_node), now_sec)

    def _occupied_nodes(self) -> dict:
        """node label -> robot, for every node currently held or promised.

        Both halves matter. A robot's current_node is where it physically is;
        a granted reservation's destination_nodes are where somebody is about
        to be. Sidestepping onto a node that is merely reserved trades one
        deadlock for a collision, which is strictly worse.
        """
        held: dict = {}
        for name, st in self.robots.items():
            if st.current_node is not None:
                held[str(st.current_node)] = name
        for name, res in self.reservations.items():
            for node in res.request.destination_nodes:
                held.setdefault(str(node), name)
        return held

    def _grid_neighbours(self, label) -> list:
        """Orthogonally adjacent GRID nodes (1..rows*cols), as ints.

        Entry, workstation and depot labels have no lattice neighbours and
        return []: an entry node is a mid-edge stub whose only exits are the
        two grid nodes it sits between, which is what entry_edge_nodes() is
        for.
        """
        try:
            n = int(label)
        except (TypeError, ValueError):
            return []
        if not (1 <= n <= self.rows * self.cols):
            return []
        row, col = (n - 1) // self.cols, (n - 1) % self.cols
        out = []
        if col > 0:
            out.append(n - 1)
        if col < self.cols - 1:
            out.append(n + 1)
        if row > 0:
            out.append(n - self.cols)
        if row < self.rows - 1:
            out.append(n + self.cols)
        return out

    def _planned_nodes(self, robot: str, horizon: int = 6) -> list:
        """The next few nodes on a robot's remaining plan."""
        st = self.robots.get(robot)
        if st is None:
            return []
        nodes = []
        for item in st.plan[st.next_index:st.next_index + horizon]:
            for attr in ("to_node", "to", "node"):
                value = getattr(item, attr, None)
                if value is not None:
                    nodes.append(str(value))
                    break
        return nodes

    def _next_grid_waypoint(self, robot: str):
        """The next lattice node the robot still has to reach, or None."""
        for node in self._planned_nodes(robot, horizon=8):
            try:
                n = int(node)
            except (TypeError, ValueError):
                continue
            if 1 <= n <= self.rows * self.cols:
                return n
        return None

    def _lattice_distance(self, a, goal) -> int:
        """Manhattan steps between two grid nodes; 0 when goal is unknown."""
        if goal is None:
            return 0
        try:
            x, y = int(a) - 1, int(goal) - 1
        except (TypeError, ValueError):
            return 0
        return (abs(x // self.cols - y // self.cols)
                + abs(x % self.cols - y % self.cols))

    def is_returning_to_depot(self, robot: str) -> bool:
        """True when a robot has no productive work left -- every remaining
        stop is depot-bound.

        This is the FIRST thing consulted when choosing a deadlock yielder,
        ahead of slack. A robot on its way home has already delivered
        everything it was going to deliver, so delaying it cannot delay any
        pickup or drop; the worst case is that it parks a few seconds later.
        A robot still carrying work can be on the critical path, and slack is
        only an estimate of that. Operator's call, 2026-09-29.

        Deliberately "no workstations left" rather than "next stop is a
        depot": a robot passing THROUGH the depot lane between two
        workstations is still working and must not be treated as home-bound.
        """
        st = self.robots.get(robot)
        if st is None:
            return False
        remaining = st.plan[st.next_index:]
        if not remaining:
            return False
        saw_depot = False
        for item in remaining:
            for attr in ("to_node", "to", "node"):
                value = getattr(item, attr, None)
                if value is None:
                    continue
                if is_workstation_node(value, self.rows, self.cols):
                    return False
                if re.fullmatch(r"D\d+", str(value).upper()):
                    saw_depot = True
                break
        return saw_depot

    def slack_by_robot(self, now_sec: float) -> dict:
        """seconds of slack against the fleet's CURRENT critical path.

        Live re-anchored end_sec, not the solver's original finish times. By
        the time a deadlock happens the plan is minutes old and robots are
        ahead of or behind it; picking a yielder from stale numbers can
        delay the one robot that actually sets the makespan.

        Fleet makespan is max(finish), so delaying a robot that is not on the
        critical path costs the fleet nothing. That is the whole basis for
        choosing who yields.
        """
        fleet = self._simulate_fleet("", now_sec)
        if not fleet:
            return {}
        makespan = max(sim.end_sec for sim in fleet.values())
        return {name: makespan - sim.end_sec for name, sim in fleet.items()}

    def resolve_deadlock(self, cycle: list, now_sec: float):
        """Pick ONE robot to step aside so a wait-for cycle unwinds.

        One is provably enough for any cycle length: a wait-for cycle is a
        simple cycle, deleting one edge leaves a path, and the head of a path
        of waiters is by definition unblocked. Moving two robots at once
        would double the chance of creating a fresh conflict during the
        recovery and make the maneuver much harder to verify, for no gain --
        so 3-, 4- and 5-robot cycles are handled exactly like pairs.

        Candidates are tried depot-bound first (a robot with no work left
        costs the fleet nothing by waiting), then by slack, most first, with
        a name tiebreak so the same deadlock always resolves the same way --
        reproducibility matters more here than fairness. Returns
        None when no member has a legal move, in which case the caller's
        existing abort is the correct fallback.
        """
        members = [r for r in cycle if r in self.robots]
        if len(members) < 2:
            return None
        slack = self.slack_by_robot(now_sec)
        occupied = self._occupied_nodes()
        # Depot-bound robots yield FIRST, whatever the slack model thinks.
        # Slack is a simulation of a plan that is minutes old by the time a
        # deadlock happens; "has no work left" is a fact about the plan
        # itself and does not drift. When both are heading home the ordering
        # falls through to slack and then to the name.
        #
        # The name tiebreak is deliberate and NOT randomised: the same
        # deadlock must resolve the same way every time, or a hang that
        # shows up once becomes impossible to reproduce and the recovery
        # itself becomes untestable.
        homeward = {r: self.is_returning_to_depot(r) for r in members}
        order = sorted(members,
                       key=lambda r: (not homeward[r], -slack.get(r, 0.0), r))
        for robot in order:
            blocked = self._cycle_successor(cycle, robot)
            plan = self._yield_move(robot, blocked, occupied, slack)
            if plan is not None:
                return plan
        return None

    @staticmethod
    def _cycle_successor(cycle: list, robot: str) -> str:
        """The robot immediately unblocked by `robot` stepping aside."""
        if robot not in cycle:
            return cycle[0]
        return cycle[(cycle.index(robot) + 1) % len(cycle)]

    def _yield_move(self, robot: str, blocked: str, occupied: dict,
                    slack: dict):
        """A legal step-aside for `robot`, or None.

        Two shapes, decided by where the yielder is standing:

        SIDESTEP -- at a grid node, move to a free orthogonal neighbour that
        is not on the blocked robot's path. One node, one turn, one move.

        BACKTRACK -- at a bay entry. An entry is a mid-edge stub: there is no
        sideways move, and there is no reverse gear on this testbed (the
        drive law is constant forward cruise), so clearing it means turning
        180, driving to a flanking grid node, and turning 180 back. That is
        two rotations, the most failure-prone primitive here, which is why it
        is the fallback rather than the default.
        """
        st = self.robots.get(robot)
        if st is None or st.current_node is None:
            return None
        here = st.current_node
        avoid = set(self._planned_nodes(blocked, horizon=3))

        flank = entry_edge_nodes(here, self.rows, self.cols)
        if flank is not None:
            # On a bay entry. The blocked robot may want this very corridor,
            # or the bay behind it; either way the entry must be vacated
            # completely rather than shuffled along.
            for target in flank:
                key = str(target)
                if key in occupied:
                    continue
                if key in avoid:
                    # Backing into the blocked robot's own path just moves
                    # the conflict one node along.
                    continue
                return YieldPlan(
                    robot=robot, kind="backtrack", from_node=here,
                    to_node=target, blocked_robot=blocked,
                    reason=(f"{robot} holds bay entry {here} that {blocked} "
                            f"needs; backing out to {target} "
                            f"(slack {slack.get(robot, 0.0):.1f}s)"))
            return None

        # Among the legal neighbours, take the one that costs the yielder
        # least: fewest extra steps back to where it was already going. A
        # free neighbour in the wrong direction resolves the deadlock just as
        # well but pays for it twice, once stepping out and again coming back.
        legal = [t for t in self._grid_neighbours(here)
                 if str(t) not in occupied and str(t) not in avoid]
        goal = self._next_grid_waypoint(robot)
        legal.sort(key=lambda t: (self._lattice_distance(t, goal), int(t)))
        for target in legal:
            return YieldPlan(
                robot=robot, kind="sidestep", from_node=here,
                to_node=target, blocked_robot=blocked,
                reason=(f"{robot} steps aside {here}->{target} to clear "
                        f"{blocked} (slack {slack.get(robot, 0.0):.1f}s)"))
        return None

    def deadlock_cycles(self) -> list[list[str]]:
        """Every distinct wait-for cycle currently held, each as a sorted
        robot list.

        ADDED 2026-08-31. _wait_cycle() has existed since this advisor was
        written and was never called by anything -- so a deadlock was
        detected pairwise (_opposite_edge_swap), logged, and then held
        forever with nobody looking at the wait-for graph as a whole. In the
        6-robot run on 2026-08-31 that meant Alvik3 and Alvik5 sat in an
        opposite-direction edge swap while Alvik1/4 and Alvik2/6 queued
        behind them, for 78s, until the operator aborted by hand.

        This does NOT resolve anything -- deliberately. It reports, so the
        supervisor can record the deadlock as a real conflict (the metrics
        deadlock_count read 0 for that run) and can fail fast instead of
        hanging. Choosing a resolution policy (priority yield, backtrack,
        reroute) is a separate decision, and reporting has to come first
        either way so its effect can be measured.

        Deduplicated by the frozen member set: A->B->A and B->A->B are one
        deadlock, not two, and the caller keys conflict ids off this."""
        seen: set[frozenset[str]] = set()
        cycles: list[list[str]] = []
        # Both graphs: a pure camera standoff has no reservation edge at
        # all, so iterating wait_for alone would never even start the walk.
        for robot in list(self.wait_for) + list(self.vision_wait_for):
            path = self._wait_cycle(robot)
            if not path:
                continue
            members = frozenset(path)
            if members in seen:
                continue
            seen.add(members)
            cycles.append(sorted(members))
        return cycles

    def _remaining_work_sec(self, st: RobotState) -> float:
        """Seconds of plan left, used only after all resource-order ties."""
        start = st.pending[1] if (st.pending and st.pending[0] == "plan") else st.next_index
        total = 0.0
        for item in st.plan[start:]:
            total += item.duration_sec
            for (_f, _t, dur) in item.implicit_segs:
                total += dur
        return total

    def _is_moving(self, st: RobotState) -> bool:
        """True while the robot is executing a move (translating); dwelling,
        turning in place, or idling counts as stationary."""
        if st.pending is None or st.pending[0] != "plan":
            return False
        return st.plan[st.pending[1]].kind == "move"

    def _must_clear_node(self, robot: str) -> bool:
        """Whether another robot's immediate movement needs this robot's
        current node.  A movable occupant gets clearance priority."""
        st = self.robots[robot]
        if self._is_moving(st) or self._immediate_request_for_state(st) is None:
            return False
        node = str(st.current_node)
        for other, ost in self.robots.items():
            if other == robot or self._is_moving(ost):
                continue
            request = self._immediate_request_for_state(ost)
            if request is not None and node in request.destination_nodes:
                return True
        return False

    def _reservation_owns_conflict(self, robot: str,
                                   conflict: Conflict) -> bool:
        reservation = self.reservations.get(robot)
        if reservation is None:
            return False
        request = reservation.request
        if conflict.key == "depot:shared-entrance":
            return request.shared_depot_entrance
        if conflict.key in request.depot_slots:
            return True
        if conflict.key in request.edge_keys:
            return True
        return conflict.key in request.destination_nodes

    def _moving_through_conflict(self, robot: str,
                                 conflict: Conflict) -> bool:
        st = self.robots[robot]
        return (
            self._is_moving(st)
            and conflict.info[robot]["plan_index"] == st.pending[1]
            and self._reservation_owns_conflict(robot, conflict)
        )

    def _priority_owner(self, deciding: str, conflict: Conflict) -> str:
        """Return the deterministic owner in the required priority order."""
        other = (conflict.robots[0] if conflict.robots[1] == deciding
                 else conflict.robots[1])
        mine, theirs = self.robots[deciding], self.robots[other]

        my_moving = self._moving_through_conflict(deciding, conflict)
        other_moving = self._moving_through_conflict(other, conflict)
        if my_moving != other_moving:
            return deciding if my_moving else other

        my_clearing = self._must_clear_node(deciding)
        other_clearing = self._must_clear_node(other)
        if my_clearing != other_clearing:
            return deciding if my_clearing else other

        my_reserved = self._reservation_owns_conflict(deciding, conflict)
        other_reserved = self._reservation_owns_conflict(other, conflict)
        if my_reserved != other_reserved:
            return deciding if my_reserved else other

        my_entry = float(conflict.info[deciding]["entry"])
        other_entry = float(conflict.info[other]["entry"])
        if my_entry + ENTRY_TIME_EPSILON_SEC < other_entry:
            return deciding
        if other_entry + ENTRY_TIME_EPSILON_SEC < my_entry:
            return other

        if mine.request_seq is not None or theirs.request_seq is not None:
            my_seq = mine.request_seq if mine.request_seq is not None else math.inf
            other_seq = (theirs.request_seq
                         if theirs.request_seq is not None else math.inf)
            if my_seq != other_seq:
                return deciding if my_seq < other_seq else other

        # Genuine near-tie: both robots are moving, neither must-clear nor
        # reservation-owns, entry times are within ENTRY_TIME_EPSILON, and the
        # request sequence didn't separate them. The deterministic answer here
        # is an arbitrary robot-name sort. Let the LLM break it (cached per
        # pair for consistency), guarded so it can only pick one of the two and
        # only inside the tie window.
        if deciding != other:
            llm_owner = self._llm_tie_break(deciding, other, conflict)
            if llm_owner is not None:
                return llm_owner
            return min(deciding, other)
        my_work = self._remaining_work_sec(mine)
        other_work = self._remaining_work_sec(theirs)
        return deciding if my_work >= other_work else other

    def _llm_tie_break(self, a: str, b: str,
                       conflict: Conflict) -> Optional[str]:
        """Ask the LLM which of two genuinely-tied robots should go first.

        Returns the owning robot, or None to fall back to the deterministic
        name sort. Only fires when the LLM is present and the entry-time gap is
        inside PRIORITY_TIE_WINDOW_SEC (a real ambiguity). The result is cached
        on the unordered pair + resource so both robots see the same owner —
        the property that prevents a mutual-yield deadlock. Guards: the LLM's
        pick must be one of {a, b}; anything else falls back deterministically."""
        if self.llm is None:
            return None
        try:
            a_entry = float(conflict.info[a]["entry"])
            b_entry = float(conflict.info[b]["entry"])
        except (KeyError, TypeError, ValueError):
            return None
        if abs(a_entry - b_entry) > PRIORITY_TIE_WINDOW_SEC:
            return None  # not a genuine tie — keep it deterministic
        cache_key = (tuple(sorted((a, b))), conflict.type, str(conflict.key))
        if cache_key in self._tie_break_cache:
            return self._tie_break_cache[cache_key]

        owner: Optional[str] = None
        if hasattr(self.llm, "break_tie"):
            payload = self._tie_payload(a, b, conflict)
            try:
                choice = self.llm.break_tie(payload)
            except Exception as exc:  # never let the LLM stall priority
                self._log({"event": "tie_break_error", "robots": [a, b],
                           "error": repr(exc)})
                choice = None
            if choice in (a, b):
                owner = choice
                self.llm_stats["tie_breaks"] += 1
                self._log({"event": "tie_break", "robots": [a, b],
                           "resource": str(conflict.key), "owner": owner,
                           "detail": conflict.detail})
                self._note(f"[advisor] LLM tie-break: {owner} goes first over "
                           f"{b if owner == a else a} at {conflict.key} "
                           f"(entry gap {abs(a_entry - b_entry):.2f}s)")
        # Cache even a None so a flaky LLM can't flip the pair mid-standoff;
        # None means "use the deterministic sort", cached as the sorted min.
        resolved = owner if owner is not None else min(a, b)
        self._tie_break_cache[cache_key] = resolved
        return resolved

    def _tie_payload(self, a: str, b: str, conflict: Conflict) -> dict:
        def robot_view(name: str) -> dict:
            st = self.robots[name]
            return {
                "robot": name,
                "current_node": str(st.current_node),
                "entry_time_sec": round(float(conflict.info[name]["entry"]), 2),
                "remaining_work_sec": round(self._remaining_work_sec(st), 1),
                "commands_remaining": len(st.plan) - st.next_index,
            }
        return {
            "situation": (
                "Two AGVs contend for the same resource and arrive at nearly "
                "the same time; no rule separates them. Choose which goes "
                "FIRST to keep the fleet moving; the other waits briefly."),
            "resource": {"type": conflict.type, "key": str(conflict.key),
                         "detail": conflict.detail},
            "robots": [robot_view(a), robot_view(b)],
            "how_to_choose": [
                "Prefer letting the robot with MORE remaining_work_sec go first "
                "so the longer critical path is not delayed (shortens makespan).",
                "Break exact ties by lower current entry_time_sec.",
            ],
            "answer_field": "owner",
            "allowed_owners": [a, b],
        }

    def _losing_conflicts(self, robot: str,
                          conflicts: list[Conflict]) -> list[Conflict]:
        return [c for c in conflicts if self._priority_owner(robot, c) != robot]

    def _next_motion_end(self, st: RobotState) -> Optional[int]:
        """Compatibility helper: only an immediate movement is gateable."""
        if (st.next_index < len(st.plan)
                and st.plan[st.next_index].kind == "move"):
            return st.next_index
        return None

    def _anchor(self, st: RobotState, now_sec: float, deciding: bool) -> tuple[int, float]:
        """(start_index, anchor_sec) for simulating a robot's remaining plan."""
        if not st.launched and st.pending is None:
            return 0, max(now_sec, st.launch_offset_sec)
        if st.pending is not None:
            # mid-command: remaining plan starts at the in-flight item,
            # anchored at the time it was sent (last_done or launch)
            kind, val = st.pending
            anchor = st.last_done_sec if st.last_done_sec is not None else st.launch_offset_sec
            if kind == "plan":
                return val, anchor
            # in-flight inserted dwell: next planned item starts when it ends
            return st.next_index, anchor + val / 1000.0
        # idle at a node — its future starts no earlier than now
        anchor = st.last_done_sec if st.last_done_sec is not None else now_sec
        return st.next_index, max(anchor, now_sec)

    def _simulate_fleet(self, deciding: str, now_sec: float,
                        lead_dwell_ms: int = 0) -> dict[str, RobotSim]:
        fleet = {}
        for name, st in self.robots.items():
            # A robot standing aside for a yield is NOT going to execute its
            # planned windows -- it is parked off-plan waiting for the robot
            # it unblocked to pass. Its plan was never rewritten, so including
            # it here reports the phantom conflict that it is still about to
            # traverse the very edge it just vacated.
            #
            # Hardware livelock this fixes (base_0924_1336.log): Alvik1 and
            # Alvik3 deadlocked on edge 10<>18. The supervisor correctly
            # sidestepped Alvik1 18->17, which released the reservation and
            # cleared the edge. Then at t=81.1s, with Alvik1 physically parked
            # at 17, this simulation still showed "Alvik1 81.1-86.0s" on
            # 10<>18, the advisor told Alvik3 to WAIT for it, Alvik3 never
            # moved, Alvik1 timed out after 42.7s and returned to 18 -- and
            # the identical deadlock and identical sidestep fired again at
            # t=118.1s. The run had to be aborted by hand.
            #
            # Safety is unaffected: this feeds the ADVISORY dwell path only.
            # The reservation layer still holds this robot's standing
            # reservation on the node it stepped aside to, and that is what
            # actually arbitrates movement (see begin_yield's docstring).
            if name in self.yielding_robots and name != deciding:
                continue
            start, anchor = self._anchor(st, now_sec, deciding=(name == deciding))
            dwell = lead_dwell_ms if name == deciding else 0
            fleet[name] = simulate_plan(
                st.plan, start, anchor, lead_dwell_ms=dwell,
                current_node=st.current_node,
                move_scale=self.calibration.factor(name, "move"),
                turn_scale=self.calibration.factor(name, "turn"),
                rows=self.rows, cols=self.cols)
        return fleet

    def _needed_dwell_sec(self, robot: str, conflict: Conflict) -> float:
        """Delay only a robot which already lost resource priority.

        The entry-order guard prevents the old 19-second inversion where an
        earlier traversal was pushed behind a later one.
        """
        other = conflict.robots[0] if conflict.robots[1] == robot else conflict.robots[1]
        mine, theirs = conflict.info[robot], conflict.info[other]
        if self._priority_owner(robot, conflict) == robot:
            raise ValueError(
                f"{robot} owns {conflict.type} {conflict.key}; refusing to "
                "calculate a priority-reversing delay")
        my_entry = float(mine["entry"])
        other_entry = float(theirs["entry"])
        if my_entry + ENTRY_TIME_EPSILON_SEC < other_entry:
            raise ValueError(
                f"{robot} enters {conflict.key} at {my_entry:.3f}s before "
                f"{other} at {other_entry:.3f}s")
        other_end = float(theirs.get("end", other_entry))
        return (other_end + conflict.clearance_sec
                + EDGE_RELEASE_BUFFER_SEC - my_entry)

    def _build_candidates(self, robot: str, now_sec: float,
                          mine: list[Conflict]) -> list[dict]:
        needed_values = []
        for conflict in mine:
            try:
                needed_values.append(self._needed_dwell_sec(robot, conflict))
            except ValueError as exc:
                self._log({"event": "dwell_guard", "robot": robot,
                           "t": now_sec, "detail": str(exc)})
        if not needed_values:
            return []
        needed = max(needed_values)
        if not math.isfinite(needed):
            return []
        base_ms = int(math.ceil(max(needed, MIN_DWELL_MS / 1000.0) * 10) * 100)
        base_ms = max(MIN_DWELL_MS, min(MAX_DWELL_MS, base_ms))
        raw = sorted({base_ms,
                      min(MAX_DWELL_MS, base_ms + 1000),
                      min(MAX_DWELL_MS, base_ms + 2000),
                      DEFAULT_DWELL_MS})
        candidates = []
        makespans = []
        for ms in raw:
            ok, detail, makespan, later_holds = self._evaluate_dwell(
                robot, now_sec, ms)
            candidates.append({"dwell_ms": ms, "conflict_free": ok,
                               "fleet_makespan_sec": round(makespan, 2),
                               "later_holds_for_me": later_holds,
                               "detail": detail})
            if ok:
                makespans.append(makespan)
        # Relative cost vs the cheapest safe option, so the LLM reasons about a
        # concrete tradeoff ("+3.2s of fleet time but avoids 1 later hold")
        # instead of raw absolute seconds it has to difference itself.
        best_makespan = min(makespans) if makespans else None
        for cand in candidates:
            if cand["conflict_free"] and best_makespan is not None:
                cand["extra_fleet_sec_vs_best"] = round(
                    cand["fleet_makespan_sec"] - best_makespan, 2)
        return candidates

    def _evaluate_dwell(self, robot: str, now_sec: float,
                        ms: int) -> tuple[bool, str, float, int]:
        st = self.robots[robot]
        window_end = self._next_motion_end(st)
        fleet = self._simulate_fleet(deciding=robot, now_sec=now_sec, lead_dwell_ms=ms)
        conflicts = detect_conflicts(fleet)
        mine_now = [c for c in conflicts
                    if robot in c.robots
                    and window_end is not None
                    and c.info[robot]["plan_index"] == window_end]
        mine_now = self._losing_conflicts(robot, mine_now)
        # Conflicts this robot still loses LATER in the plan (beyond the move
        # this dwell gates). A longer dwell that also clears the next one is
        # worth more than one that barely clears the immediate conflict.
        mine_later = self._losing_conflicts(robot, [
            c for c in conflicts
            if robot in c.robots
            and window_end is not None
            and c.info[robot]["plan_index"] != window_end])
        later_holds = len({c.info[robot]["plan_index"] for c in mine_later})
        makespan = max((sim.end_sec for sim in fleet.values()), default=now_sec)
        if mine_now:
            return (False, "; ".join(c.detail for c in mine_now[:3]),
                    makespan, later_holds)
        return True, "next motion conflict-free", makespan, later_holds

    def validate_dwell(self, robot: str, now_sec: float, ms: Any,
                       allowed_candidates: Optional[set[int]] = None
                       ) -> tuple[bool, str]:
        """Validate a deterministic candidate selected by the optional LLM."""
        if not isinstance(ms, int) or isinstance(ms, bool):
            return False, f"dwell_ms must be an integer, got {ms!r}"
        if ms < MIN_DWELL_MS or ms > MAX_DWELL_MS:
            return False, (f"dwell_ms {ms} outside firmware clamp "
                           f"[{MIN_DWELL_MS}, {MAX_DWELL_MS}]")
        if allowed_candidates is not None and ms not in allowed_candidates:
            return False, (
                f"dwell_ms {ms} was not one of the deterministically generated "
                f"safe candidates {sorted(allowed_candidates)}")
        ok, detail, _makespan, _later = self._evaluate_dwell(robot, now_sec, ms)
        if not ok:
            return False, f"deterministic re-check still predicts: {detail}"
        return True, "pass"

    def _consult_llm(self, st: RobotState, now_sec: float, mine: list[Conflict],
                     candidates: list[dict], sig: frozenset) -> Decision:
        robot = st.name
        safe = [candidate for candidate in candidates
                if candidate["conflict_free"]]
        allowed = {candidate["dwell_ms"] for candidate in safe}
        payload = self._llm_payload(robot, now_sec, mine, safe)
        blocker = self._priority_owner(robot, mine[0])
        self.wait_for[robot] = blocker
        st.last_wait_sig = sig

        if self.llm is None:
            selected = min(allowed) if allowed else None
            self.llm_stats["no_llm_wait"] += 1
            self._log({"event": "no_llm_wait", "robot": robot, "t": now_sec,
                       "selected_candidate_ms": selected,
                       "conflicts": [c.detail for c in mine]})
            return self._wait_decision(
                st, "no-LLM mode: supervisor-side collision wait",
                conflicts=mine, selected_candidate_ms=selected)

        if not allowed:
            return self._wait_decision(
                st, "no finite safe delay; waiting for fleet state change",
                conflicts=mine)

        self.llm_stats["consultations"] += 1
        self._note(
            f"[advisor] LLM consult: {robot} at t={now_sec:.1f}s — "
            f"conflict {mine[0].detail}; candidate dwells (ms)="
            f"{sorted(allowed)}")

        rejection = None
        proposal = None
        for attempt in (1, 2):
            try:
                proposal = self.llm.propose(payload, rejection=rejection,
                                            prior=proposal)
            except Exception as exc:  # API/network failure — never drop the command
                self.llm_stats["llm_error"] += 1
                self._log({"event": "llm_error", "robot": robot, "t": now_sec,
                           "attempt": attempt, "error": repr(exc)})
                self._note(f"[advisor] LLM error for {robot}: {exc!r} — "
                           "falling back to a plain supervisor wait")
                return self._wait_decision(
                    st, f"LLM error ({exc!r}); supervisor-side wait",
                    conflicts=mine, llm_used=True)
            ms = proposal.get("dwell_ms")
            ok, why = self.validate_dwell(
                robot, now_sec, ms, allowed_candidates=allowed)
            self._log({"event": "llm_proposal", "robot": robot, "t": now_sec,
                       "attempt": attempt, "proposal": proposal,
                       "gate": "pass" if ok else "reject", "gate_detail": why,
                       "conflicts": [c.detail for c in mine],
                       "candidates": candidates})
            if ok:
                self.llm_stats["llm_pass"] += 1
                st.last_fallback_sig = None
                self._note(
                    f"[advisor] LLM chose dwell {ms}ms for {robot} "
                    f"(attempt {attempt}): "
                    f"{proposal.get('rationale', '') or 'no rationale'}")
                return self._wait_decision(
                    st, f"LLM selected safe wait horizon: "
                    f"{proposal.get('rationale', '')}",
                    conflicts=mine, llm_used=True,
                    selected_candidate_ms=ms)
            self.llm_stats["llm_reject"] += 1
            self._note(f"[advisor] LLM proposal {ms}ms for {robot} REJECTED by "
                       f"gate (attempt {attempt}): {why}")
            rejection = why

        self.llm_stats["gate_fallback"] += 1
        self._log({"event": "gate_fallback", "robot": robot, "t": now_sec,
                   "last_rejection": rejection})
        self._note(f"[advisor] LLM gate-rejected twice for {robot} "
                   f"({rejection}) — using a plain supervisor wait")
        return self._wait_decision(
            st, f"gate rejected LLM twice ({rejection}); supervisor-side wait",
            conflicts=mine, llm_used=True)

    def _llm_payload(self, robot: str, now_sec: float, mine: list[Conflict],
                     candidates: list[dict]) -> dict:
        fleet_state = []
        for name, st in self.robots.items():
            nxt = st.plan[st.next_index].command if st.next_index < len(st.plan) else None
            fleet_state.append({
                "robot": name,
                "current_node": str(st.current_node),
                "launched": st.launched,
                "commands_remaining": len(st.plan) - st.next_index,
                "next_command": nxt,
            })
        return {
            "deciding_robot": robot,
            "now_sec": round(now_sec, 2),
            "fleet": fleet_state,
            "predicted_conflicts": [
                {"type": c.type, "key": c.key, "robots": list(c.robots),
                 "time_sec": round(c.time_sec, 2), "gap_sec": round(c.gap_sec, 2),
                 "deterministic_owner": self._priority_owner(robot, c),
                 "detail": c.detail}
                for c in mine],
            "candidate_dwells": [
                {k: v for k, v in cand.items() if k != "detail"} | {"note": cand["detail"]}
                for cand in candidates],
            "reservations": [
                {"robot": reservation.robot,
                 **self._request_summary(reservation.request)}
                for reservation in sorted(
                    self.reservations.values(),
                    key=lambda value: value.granted_seq)
            ],
            "objective": (
                "Keep the whole fleet moving: minimise fleet makespan while "
                "avoiding repeated stop-and-go. Each candidate is already "
                "collision-safe; choose the one with the best tradeoff."),
            "how_to_choose": [
                "Prefer the smallest fleet_makespan_sec (least total delay).",
                "If a slightly longer dwell has later_holds_for_me = 0 while a "
                "shorter one leaves later_holds_for_me > 0, prefer the longer "
                "one: paying a little extra_fleet_sec_vs_best now avoids "
                "another stop for this robot soon.",
                "Never invent a value: dwell_ms MUST be one listed in "
                "candidate_dwells with conflict_free = true.",
            ],
            "field_guide": {
                "fleet_makespan_sec": "predicted finish time of the LATEST robot "
                                      "if this dwell is used (lower is better)",
                "extra_fleet_sec_vs_best": "how many more seconds of fleet time "
                                           "this costs vs the cheapest safe option",
                "later_holds_for_me": "how many MORE times this robot is predicted "
                                      "to have to stop later if this dwell is used",
            },
            "dwell_bounds_ms": [MIN_DWELL_MS, MAX_DWELL_MS],
        }

    def _log(self, record: dict) -> None:
        if not self.log_path:
            return
        record = {"wall_time": time.time(), **record}
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, default=str) + "\n")
        except OSError:
            pass

    def _note(self, message: str) -> None:
        """Human-readable line to the terminal sink, if the supervisor set one."""
        if self.log_fn:
            try:
                self.log_fn(message)
            except Exception:
                pass

    def llm_activity_summary(self) -> str:
        """One-line recap of whether/how the LLM advisor was used this run."""
        s = self.llm_stats
        if self.llm is None:
            return (f"LLM advisor: DISABLED (--no-llm). "
                    f"{s['no_llm_wait']} deterministic collision wait(s).")
        ties = s.get("tie_breaks", 0)
        tie_note = f" {ties} priority tie-break(s)." if ties else ""
        if s["consultations"] == 0:
            base = ("LLM advisor: enabled; no yield-dwell consultation was "
                    "needed this run")
            return base + (f" but broke {ties} priority tie(s)." if ties
                           else " (no resource conflict required a wait).")
        return (f"LLM advisor: {s['consultations']} dwell consultation(s) — "
                f"{s['llm_pass']} accepted, {s['llm_reject']} gate-rejected, "
                f"{s['gate_fallback']} fell back to a plain wait, "
                f"{s['llm_error']} error(s)." + tie_note)


def load_schedule(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)
