#!/usr/bin/env python3
"""Robots sliding past each other in parallel lanes are not emergency-stopped.

    python3 test_lane_passby.py

2026-10-22 SHAKE PP, operator: "robot 2 trying to enter workstation 1 and not
able to get to its node because alvik 3 is driving to node 9 next to it.
This shouldn't stop these robots as they are driving parallel to one another
and won't collide if they just drive straight to their node and then rotate
after they both get there."

The poses are the real ones at the first EMERGENCY STOP (t=838.29): Alvik2
leaving entry 114 south for workstation 65, Alvik3 leaving node 1 north for
node 9, 4.8in apart across the lanes, both tilted 5-7deg.
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


def judge(pa, va, ya, ta, pb, vb, yb, tb):
    """What check_collisions() decides for one pair, old margin vs new."""
    dx, dy = pb[0] - pa[0], pb[1] - pa[1]
    sa = ((math.hypot(ta[0] - pa[0], ta[1] - pa[1]) + F.LEG_OVERSHOOT_IN)
          / math.hypot(*va) if ta else math.inf)
    sb = ((math.hypot(tb[0] - pb[0], tb[1] - pb[1]) + F.LEG_OVERSHOOT_IN)
          / math.hypot(*vb) if tb else math.inf)
    gap = F.footprint_gap_in(dx, dy, ya, yb)
    pred = F.predicted_min_gap_legs_in(pa, va, sa, pb, vb, sb, ya, yb)
    rvx, rvy = vb[0] - va[0], vb[1] - va[1]
    rel = math.hypot(rvx, rvy)
    passby = F.lane_passby(ya, yb, rvx, rvy, dx, dy)
    old = F.motion_verdict(gap, pred, rel, math.hypot(dx, dy))
    new = F.motion_verdict(gap, pred, rel, math.hypot(dx, dy),
                           margin=(F.COLLISION_PASSBY_MARGIN_IN if passby
                                   else F.COLLISION_PREDICT_MARGIN_IN))
    return old, new, passby, pred


print("\n1. the 2026-10-22 stop")
a2 = ((18.60, 25.65), (-0.34, -3.46), -6.7, (18.30, 21.76))   # 114 -> 65
a3 = ((13.63, 18.57), (0.34, 3.51), 174.1, (13.50, 26.75))    # 1 -> 9
old, new, passby, pred = judge(*a2, *a3)
check("recognised as a lane pass-by", passby)
check("the old 0.25in margin stopped them", old == "avoid", f"predicted {pred:.2f}in")
check("the pass-by margin lets them drive on", new is None)

print("\n2. still stopped")
# Same lane, head-on: never a pass-by.
old, new, passby, _ = judge((13.5, 20.0), (0.0, 3.5), 180.0, (13.5, 26.75),
                            (13.5, 27.0), (0.0, -3.5), 0.0, (13.5, 17.0))
check("head-on in one lane", not passby and new == "avoid")
# Neighbouring lanes, but one robot really veering into the other.
old, new, passby, pred = judge((18.4, 25.0), (-1.2, -3.3), -20.0, None,
                               (13.6, 22.0), (0.0, 3.5), 180.0, None)
check("a robot 20deg off its lane is not a pass-by", not passby and new == "avoid",
      f"predicted {pred:.2f}in")
# Lanes so close the footprints will genuinely meet.
old, new, passby, pred = judge((17.6, 25.0), (0.0, -3.5), 0.0, None,
                               (13.5, 22.0), (0.0, 3.5), 180.0, None)
check("4.1in apart and tilted only by noise: gap under 0.10 -> stopped",
      new == "avoid" or pred > F.COLLISION_PASSBY_MARGIN_IN,
      f"predicted {pred:.2f}in, passby={passby}")
old, new, passby, pred = judge((17.4, 25.0), (0.0, -3.5), 4.0, None,
                               (13.5, 22.0), (0.0, 3.5), 176.0, None)
check("3.9in apart: not a pass-by, and stopped", not passby and new is not None,
      f"predicted {pred:.2f}in")
check("a turning robot (yaw None) is never a pass-by",
      not F.lane_passby(None, 0.0, 0.0, 3.0, 5.0, 0.0))
check("crossing paths (relative motion diagonal) is not a pass-by",
      not F.lane_passby(0.0, 90.0, 2.5, 2.5, 5.0, 3.0))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
