#!/usr/bin/env python3
"""A robot parked too deep in its bay is an obstacle to the row behind it.

    python3 test_parked_intrusion.py

2026-10-23 COLLISION, Alvik2 and Alvik5, 0.22in overlap. Alvik5 came to rest
at (38.73, 40.47), 1.38in past workstation 81 (38.30, 41.82) -- its deepest
stop of 304 -- and dwelt there 30 s. Alvik2 was granted 20->19 along the row
behind the bay (y 36.75). The corridor gate skipped Alvik5 as "off the lane",
and the guard exempted it as "parked", so nothing predicted the contact.
"""
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Vis:
    def __init__(self, x, y, yaw):
        self.p = (x, y, yaw, 0.05)

    def snapshot_in(self):
        return {"Alvik5": self.p}

    def is_turning(self, _n):
        return False


sup = object.__new__(F.FleetSupervisor)

print("\n1. the dispatcher holds a move past a robot parked too deep")
sup.vision = _Vis(38.73, 40.47, 179.9)
check("today's pose blocks 20->19",
      not sup._path_clears("Alvik5", 20, 19, 8, 8, margin=F.PARKED_INTRUSION_MARGIN_IN))
sup.vision = _Vis(38.30, 41.82, 180.0)
check("parked ON workstation 81: 20->19 is clear (no hold, ~1in to spare)",
      sup._path_clears("Alvik5", 20, 19, 8, 8, margin=F.PARKED_INTRUSION_MARGIN_IN))
sup.vision = _Vis(38.30, 41.32, 180.0)
check("0.5in deep: still clear (holds only past ~0.8in)",
      sup._path_clears("Alvik5", 20, 19, 8, 8, margin=F.PARKED_INTRUSION_MARGIN_IN))
sup.vision = _Vis(38.30, 40.90, 180.0)
check("0.92in deep: held",
      not sup._path_clears("Alvik5", 20, 19, 8, 8, margin=F.PARKED_INTRUSION_MARGIN_IN))
src = open(F.__file__, encoding="utf-8").read()
check("the corridor gate checks off-lane robots by footprint",
      "margin=PARKED_INTRUSION_MARGIN_IN" in src and "is parked too" in src)

print("\n2. the guard predicts it, and stops only the mover")
a, b = (43.4, 36.6), (38.73, 40.47)
stop = (math.hypot(33.5 - a[0], 0.15) + F.LEG_OVERSHOOT_IN) / 3.5
gap = F.footprint_gap_in(b[0] - a[0], b[1] - a[1], -90.0, 179.9)
pred = F.predicted_min_gap_legs_in(a, (-3.5, 0.0), stop, b, (0.0, 0.0), math.inf, -90.0, 179.9)
check("Alvik2 stopped at the start of 20->19, before contact",
      F.motion_verdict(gap, pred, 3.5, math.hypot(b[0] - a[0], b[1] - a[1]),
                       margin=F.COLLISION_PASSBY_MARGIN_IN) == "avoid",
      f"now {gap:.2f}in, predicted {pred:.2f}in")
check("parked pairs no longer exempt from the prediction",
      "parked_pair=parked(a) or parked(b)" not in src)
check("only the moving robot is stopped", "movers = [n for n, p in ((a, pa), (b, pb)) if not p]" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
