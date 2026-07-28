#!/usr/bin/env python3
"""
route_planner.py  —  Off-robot routing engine for AGV_MULTI_WS_DISPATCH

This module contains ALL grid / map intelligence.  The Arduino sketch is a
dumb token executor; every routing decision is made here and sent as a flat
token string.

Token vocabulary (matches the Arduino sketch)
---------------------------------------------
  RED     drive forward to the next red intersection marker
  CLEAR   creep forward off the current marker before the next RED
  R       turn 90° right (absolute yaw decreases by 90)
  L       turn 90° left  (absolute yaw increases by 90)
  YENTRY  drive slowly until the yellow entry sticker (side of the aisle)
  R_SPUR  right-turn version of the YENTRY→spur turn (same as R with short
          yellow-ignore window — emitted separately so callers can read the
          intent, collapsed to R in to_token_string)
  YWORK   drive slowly until the yellow workstation stop marker
  YAW0    rotate to absolute yaw 0 (face north) before reversing into dock
  DOCK    reverse slowly until the yellow dock marker
  DWELL   wait at the workstation (WORKSTATION_WAIT_MS on the robot)
  EXIT    drive forward until back at the yellow entry sticker
  BLUE    drive forward until the blue depot marker
  YAW0    (also used at the end of the return leg to face north at depot)

Grid conventions  (same as the Arduino sketch and workstations.json)
--------------------------------------------------------------------
  8×8 grid, 64 nodes numbered row-major from the BOTTOM-LEFT.
  Row 1 = nodes  1-8   (bottom row)
  Row 2 = nodes  9-16
  …
  Row 8 = nodes 57-64  (top row)

  Depot is at the bottom-left corner (node 1, column 1).
  The AGV starts facing NORTH (yaw = 0) on the blue sticker.

  Red intersection markers sit at every grid node.
  Yellow entry stickers sit between two adjacent nodes (workstation spurs).
"""

from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

GRID_COLS = 8


def node_row(node: int) -> int:
    """1-based row of a node (row 1 = bottom)."""
    return (node - 1) // GRID_COLS + 1


def node_col(node: int) -> int:
    """1-based column of a node (col 1 = leftmost)."""
    return (node - 1) % GRID_COLS + 1


# ---------------------------------------------------------------------------
# Workstation data class
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Workstation:
    id: str            # e.g. "WS01"
    left_node: int     # lower-numbered node of the pair
    right_node: int    # higher-numbered node of the pair

    @property
    def row(self) -> int:
        return node_row(self.left_node)

    @property
    def left_col(self) -> int:
        return node_col(self.left_node)

    @property
    def right_col(self) -> int:
        return node_col(self.right_node)

    # --- approach from depot (north-bound then east-bound) ------------------

    @property
    def northbound_red_count(self) -> int:
        """Red markers to count going north from depot before the east turn."""
        return self.row - 1

    @property
    def eastbound_red_count(self) -> int:
        """Red markers to count going east before the yellow entry (APPROACH_EAST)."""
        return self.left_col - 1

    @property
    def westbound_red_count(self) -> int:
        """Red markers to count going west from the entry segment back to depot col."""
        return self.left_col


# ---------------------------------------------------------------------------
# Load workstations from JSON
# ---------------------------------------------------------------------------

_DEFAULT_JSON = Path(__file__).parent / "workstations.json"


def load_workstations(path: Path = _DEFAULT_JSON) -> dict[str, Workstation]:
    data = json.loads(path.read_text(encoding="utf-8"))
    registry: dict[str, Workstation] = {}
    for row in data["workstations"]:
        a, b = int(row["between_nodes"][0]), int(row["between_nodes"][1])
        ws = Workstation(
            id=str(row["id"]),
            left_node=min(a, b),
            right_node=max(a, b),
        )
        registry[ws.id] = ws
    return registry


# ---------------------------------------------------------------------------
# Token sequence builders
# ---------------------------------------------------------------------------

def _red_tokens(count: int) -> List[str]:
    """Tokens to traverse `count` red intersection markers in one direction."""
    tokens: List[str] = []
    for i in range(count):
        if i > 0:
            tokens.append("CLEAR")
        tokens.append("RED")
    return tokens


def depot_to_workstation_tokens(ws: Workstation) -> List[str]:
    """
    Token sequence: blue depot → docked at workstation.

    The robot starts at the depot facing NORTH.
    Steps:
      1. Drive north, counting red markers until entry row.
      2. Turn east.
      3. Drive east, counting red markers until entry column.
      4. Drive slowly to yellow entry sticker.
      5. Turn south (right turn) into the workstation spur.
      6. Drive slowly to workstation yellow marker.
      7. Align yaw to north (YAW0) so we can reverse straight.
      8. Reverse to dock yellow marker.
    """
    tokens: List[str] = []

    north_count = ws.northbound_red_count
    east_count  = ws.eastbound_red_count

    if north_count > 0:
        tokens.extend(_red_tokens(north_count))

    tokens.append("R")          # face east

    if east_count > 0:
        tokens.extend(_red_tokens(east_count))

    tokens.append("YENTRY")     # creep east to yellow entry sticker
    tokens.append("R")          # face south into the spur
    tokens.append("YWORK")      # approach workstation yellow
    tokens.append("YAW0")       # align north before reversing
    tokens.append("DOCK")       # reverse to dock

    return tokens


def workstation_to_next_tokens(from_ws: Workstation, to_ws: Workstation) -> List[str]:
    """
    Token sequence: docked at from_ws → docked at to_ws.

    Heading turn table (CW = right, CCW = left):
      R from N → E    L from N → W
      R from E → S    L from E → N
      R from S → W    L from S → E
      R from W → N    L from W → S

    After EXIT the robot faces NORTH at the yellow entry sticker.
    Strategy: move vertically first (N/S), then horizontally (E/W) to
    arrive at to_ws.left_col facing EAST, then do YENTRY→spur→dock.
    """
    tokens: List[str] = []

    tokens.append("EXIT")           # drive N, stop at yellow entry (heading = N)

    # --- Vertical move: current row → to_ws.row ---
    delta_row = to_ws.row - from_ws.row   # positive = go north

    if delta_row > 0:
        # already facing N — count reds northward
        tokens.extend(_red_tokens(delta_row))
    elif delta_row < 0:
        # N→R→E→R→S
        tokens.append("R")          # face E
        tokens.append("R")          # face S
        tokens.extend(_red_tokens(-delta_row))
        # now at correct row, heading S → turn to face E
        tokens.append("L")          # S→L→E
    # if delta_row == 0: still heading N, turn E below

    # --- Horizontal move: current col → to_ws.left_col ---
    # Heading is N (delta_row==0) or E (delta_row!=0) at this point.
    # Normalise to E first, then handle the column delta.
    delta_col = to_ws.left_col - from_ws.left_col   # positive = go east

    if delta_row == 0:
        # heading is still N
        if delta_col >= 0:
            tokens.append("R")      # N→R→E
            if delta_col > 0:
                tokens.extend(_red_tokens(delta_col))
            # heading = E, at correct column
        else:
            tokens.append("L")      # N→L→W
            tokens.extend(_red_tokens(-delta_col))
            # heading W → turn to E: W→R→N→R→E
            tokens.append("R")      # W→R→N
            tokens.append("R")      # N→R→E
    else:
        # heading is already E (after vertical move)
        if delta_col > 0:
            tokens.extend(_red_tokens(delta_col))
            # heading stays E
        elif delta_col < 0:
            # E→L→N→L→W
            tokens.append("L")      # E→L→N
            tokens.append("L")      # N→L→W
            tokens.extend(_red_tokens(-delta_col))
            # W→R→N→R→E
            tokens.append("R")      # W→R→N
            tokens.append("R")      # N→R→E

    # --- Dock at to_ws (heading = E, at to_ws.left_col, to_ws.row) ---
    tokens.append("YENTRY")         # creep E to yellow entry sticker
    tokens.append("R")              # E→R→S (into spur)
    tokens.append("YWORK")
    tokens.append("YAW0")           # align N before reversing
    tokens.append("DOCK")

    return tokens


def workstation_to_depot_tokens(ws: Workstation) -> List[str]:
    """
    Token sequence: docked at workstation → blue depot.

    After EXIT the robot faces NORTH at the yellow entry sticker.
    Turn west (N→L→W), count reds west to col 1, turn south (W→L→S),
    drive to BLUE, then YAW0 to face north ready for the next mission.
    """
    tokens: List[str] = []
    tokens.append("EXIT")           # heading = N
    tokens.append("L")              # N→L→W
    tokens.extend(_red_tokens(ws.westbound_red_count))
    tokens.append("L")              # W→L→S
    tokens.append("BLUE")
    tokens.append("YAW0")
    return tokens


# ---------------------------------------------------------------------------
# High-level route builders
# ---------------------------------------------------------------------------

def build_route_tokens(
    workstation_ids: List[str],
    registry: Optional[dict[str, Workstation]] = None,
    return_to_depot: bool = True,
) -> List[str]:
    """
    Build the complete flat token list for an ordered workstation route.

    Parameters
    ----------
    workstation_ids : e.g. ["WS01", "WS04", "WS07"]
    registry        : workstation registry (loaded from JSON if None)
    return_to_depot : whether to append return-to-depot tokens after last stop

    Returns
    -------
    Flat list of token strings ready to join with commas and send via ROS.
    """
    if registry is None:
        registry = load_workstations()

    if not workstation_ids:
        raise ValueError("workstation_ids must not be empty")

    tokens: List[str] = []

    # First stop: depot → workstation
    first_ws = registry[workstation_ids[0]]
    tokens.extend(depot_to_workstation_tokens(first_ws))
    tokens.append("DWELL")

    # Subsequent stops: workstation → workstation
    for prev_id, next_id in zip(workstation_ids, workstation_ids[1:]):
        prev_ws = registry[prev_id]
        next_ws = registry[next_id]
        tokens.extend(workstation_to_next_tokens(prev_ws, next_ws))
        tokens.append("DWELL")

    # Return leg
    if return_to_depot:
        last_ws = registry[workstation_ids[-1]]
        tokens.extend(workstation_to_depot_tokens(last_ws))

    return tokens


def to_token_string(tokens: List[str]) -> str:
    """Join tokens into the comma-separated string expected by the Arduino."""
    return ",".join(tokens)


def to_run_command(tokens: List[str]) -> str:
    """Format tokens as a 'run ...' ROS command string."""
    return "run " + to_token_string(tokens)


# ---------------------------------------------------------------------------
# Random route generator  (useful for testing and scheduling experiments)
# ---------------------------------------------------------------------------

def random_route(
    n_stops: int,
    registry: Optional[dict[str, Workstation]] = None,
    seed: Optional[int] = None,
    return_to_depot: bool = True,
) -> Tuple[List[str], List[str]]:
    """
    Generate a random ordered route visiting `n_stops` distinct workstations.

    Returns (workstation_id_list, token_list).
    """
    if registry is None:
        registry = load_workstations()

    rng = random.Random(seed)
    ws_ids = rng.sample(sorted(registry.keys()), k=min(n_stops, len(registry)))
    tokens = build_route_tokens(ws_ids, registry=registry, return_to_depot=return_to_depot)
    return ws_ids, tokens


# ---------------------------------------------------------------------------
# Route plan dataclass  (for scheduling integration)
# ---------------------------------------------------------------------------

@dataclass
class RoutePlan:
    """A resolved route for one AGV, ready to dispatch."""
    agv_id: str
    workstation_ids: List[str]
    tokens: List[str] = field(default_factory=list)
    return_to_depot: bool = True

    def build(self, registry: Optional[dict[str, Workstation]] = None) -> "RoutePlan":
        """Compute tokens from workstation_ids. Returns self for chaining."""
        self.tokens = build_route_tokens(
            self.workstation_ids,
            registry=registry,
            return_to_depot=self.return_to_depot,
        )
        return self

    @property
    def run_command(self) -> str:
        return to_run_command(self.tokens)

    def to_dict(self) -> dict:
        return {
            "agv_id": self.agv_id,
            "workstation_ids": self.workstation_ids,
            "token_count": len(self.tokens),
            "tokens": self.tokens,
            "run_command": self.run_command,
        }


# ---------------------------------------------------------------------------
# Token stream inspector  (debugging helper)
# ---------------------------------------------------------------------------

def explain_tokens(tokens: List[str]) -> str:
    """
    Return a human-readable summary of a token list, grouping consecutive
    movement tokens and annotating each DWELL.
    """
    lines: List[str] = []
    step = 0
    for i, tok in enumerate(tokens):
        lines.append(f"  {i:3d}  {tok}")
        if tok == "DWELL":
            step += 1
            lines[-1] += f"  ← stop #{step}"
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Compute token routes for AGV_MULTI_WS_DISPATCH."
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_route = sub.add_parser("route", help="Build route for given workstation list.")
    p_route.add_argument("workstations", nargs="+",
                         help="Workstation IDs in visit order, e.g. WS01 WS04 WS07")
    p_route.add_argument("--agv", default="Alvik1", help="AGV name (for ROS topic prefix)")
    p_route.add_argument("--no-return", action="store_true",
                         help="Do not append return-to-depot tokens")
    p_route.add_argument("--format", choices=("tokens", "command", "explain", "json"),
                         default="command")
    p_route.add_argument("--ws-json", type=Path, default=_DEFAULT_JSON,
                         help="Path to workstations.json")

    p_rand = sub.add_parser("random", help="Build a random route.")
    p_rand.add_argument("n", type=int, help="Number of workstations to visit")
    p_rand.add_argument("--seed", type=int, default=None)
    p_rand.add_argument("--agv", default="Alvik1")
    p_rand.add_argument("--no-return", action="store_true")
    p_rand.add_argument("--format", choices=("tokens", "command", "explain", "json"),
                        default="command")
    p_rand.add_argument("--ws-json", type=Path, default=_DEFAULT_JSON)

    args = parser.parse_args()
    registry = load_workstations(args.ws_json)

    if args.cmd == "route":
        ids   = [i.upper() if i.upper().startswith("WS") else f"WS{int(i):02d}"
                 for i in args.workstations]
        plan  = RoutePlan(agv_id=args.agv, workstation_ids=ids,
                          return_to_depot=not args.no_return).build(registry)
    else:
        ids, tokens = random_route(args.n, registry=registry, seed=args.seed,
                                   return_to_depot=not args.no_return)
        plan = RoutePlan(agv_id=args.agv, workstation_ids=ids,
                         return_to_depot=not args.no_return)
        plan.tokens = tokens

    fmt = args.format
    if fmt == "tokens":
        print(to_token_string(plan.tokens))
    elif fmt == "command":
        print(plan.run_command)
    elif fmt == "explain":
        print(f"Route: {plan.workstation_ids}")
        print(f"Tokens ({len(plan.tokens)}):")
        print(explain_tokens(plan.tokens))
    elif fmt == "json":
        print(json.dumps(plan.to_dict(), indent=2))
