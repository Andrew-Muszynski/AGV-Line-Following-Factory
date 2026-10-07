#!/usr/bin/env python3
"""The depot lane is single file: enter a node only once the robot ahead has
ARRIVED at the next one.

Operator's specification, 2026-09-30:

    Alvik6 cannot move forward to DE5 until Alvik5 has reached DE4,
    Alvik5 cannot move forward to DE4 until Alvik4 has reached DE3,
    Alvik3 cannot move forward to DE2 until Alvik2 has reached DE1,
    Alvik2 cannot move forward to DE1 until Alvik1 has reached node 0.
    Once Alvik1 has reached node 1, Alvik2 moves to node 0. When Alvik2 has
    reached node 0, Alvik3 can move forward to DE1. ...

WHY THIS IS A RUNTIME RULE AND NOT A PLAN-TIME DELAY. The HTML cascade
expressed the sequencing as per-segment waits in `wait_before_segment_sec`,
and NOTHING IN THE SUPERVISOR READS THAT FIELD -- it travels in the mission
payload and is ignored. The whole schedule was therefore simulation-only and
never reached a robot, which is why the plan looked right while the hardware
bunched up.

The second gap was in the reservation layer: a reservation follows the move,
so the instant a robot is granted DE5 -> DE4 its hold shifts to DE4 and DE5
reads free while the robot is still physically standing on it. On hardware
2026-09-30 that put Alvik3, 4, 5 and 6 within 3.8-5.3in of each other --
inside the 5.625in contact distance.

A robot in flight therefore keeps occupying the node it LEFT until it has
arrived. That single rule is what turns the cascade from a timing hope into
a physical guarantee.

    python3 test_depot_lane.py
"""
import sys

import dispatch_advisor as da
from dispatch_advisor import is_depot_lane_label as is_lane

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _St:
    def __init__(self, node, pend=(), edge=None):
        self.current_node = node
        self.pending_edges = pend
        self.current_edge = edge


class _Req:
    def __init__(self, dests):
        self.destination_nodes = frozenset(dests)


def blocker(positions, who, dest):
    a = object.__new__(da.DispatchAdvisor)
    a.rows = a.cols = 8
    a.robots = {n: (v if isinstance(v, _St) else _St(v))
                for n, v in positions.items()}
    return a._depot_lane_blocker(who, _Req({dest}))


print("which labels are on the lane")
for lab in ("DE1", "DE6", "de3", "0", 0):
    check(f"{lab!r} is a lane node", is_lane(lab))
for lab in ("D1", "1", "65", "114", "", "DE", None):
    check(f"{lab!r} is not", not is_lane(lab))

print("\nthe operator's chain, case by case")
cases = [
    ("Alvik6 -> DE5, Alvik5 still ON DE5",
     {"Alvik5": "DE5", "Alvik6": "DE6"}, "Alvik6", "DE5", True),
    ("Alvik6 -> DE5, Alvik5 IN FLIGHT to DE4",
     {"Alvik5": _St("DE5", pend=(("DE5", "DE4"),)), "Alvik6": "DE6"},
     "Alvik6", "DE5", True),
    ("Alvik6 -> DE5, Alvik5 HAS REACHED DE4",
     {"Alvik5": "DE4", "Alvik6": "DE6"}, "Alvik6", "DE5", False),
    ("Alvik5 -> DE4, Alvik4 still on DE4",
     {"Alvik4": "DE4", "Alvik5": "DE5"}, "Alvik5", "DE4", True),
    ("Alvik5 -> DE4, Alvik4 reached DE3",
     {"Alvik4": "DE3", "Alvik5": "DE5"}, "Alvik5", "DE4", False),
    ("Alvik2 -> DE1, Alvik1 still on DE1",
     {"Alvik1": "DE1", "Alvik2": "DE2"}, "Alvik2", "DE1", True),
    ("Alvik2 -> DE1, Alvik1 reached node 0",
     {"Alvik1": "0", "Alvik2": "DE2"}, "Alvik2", "DE1", False),
    ("Alvik2 -> node 0, Alvik1 still leaving it",
     {"Alvik1": _St("0", pend=(("0", "1"),)), "Alvik2": "DE1"},
     "Alvik2", "0", True),
    ("Alvik2 -> node 0, Alvik1 reached node 1",
     {"Alvik1": "1", "Alvik2": "DE1"}, "Alvik2", "0", False),
    ("Alvik3 -> DE1, Alvik2 reached node 0",
     {"Alvik2": "0", "Alvik3": "DE2"}, "Alvik3", "DE1", False),
]
for label, pos, who, dest, want in cases:
    got = blocker(pos, who, dest)
    check(label, (got is not None) == want,
          f"BLOCKED by {got[0]} on {got[1]}" if got else "allowed")

print("\nthe in-flight rule specifically")
# current_edge is the other way a move is recorded; both must count.
check("current_edge also holds the origin",
      blocker({"Alvik5": _St("DE4", edge=("DE5", "DE4")), "Alvik6": "DE6"},
              "Alvik6", "DE5") is not None,
      "a robot that has updated current_node but is still traversing")

print("\nthis gate touches ONLY the lane")
check("a lattice move is never gated by it",
      blocker({"Alvik1": "9", "Alvik2": "17"}, "Alvik2", "9") is None,
      "the grid has its own reservation and corridor gates")
check("a depot SLOT is not a lane node",
      blocker({"Alvik1": "D1", "Alvik2": "D2"}, "Alvik2", "D1") is None,
      "slots are private -- each robot parks in its own")
check("a workstation is not a lane node",
      blocker({"Alvik1": "65", "Alvik2": "114"}, "Alvik2", "65") is None)

print("\nno robots, no block")
check("alone in the lane", blocker({"Alvik1": "DE1"}, "Alvik1", "0") is None)
check("a robot with no known position blocks nothing",
      blocker({"Alvik1": _St(None), "Alvik2": "DE2"}, "Alvik2", "DE1") is None)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
