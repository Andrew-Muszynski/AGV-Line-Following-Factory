#!/usr/bin/env python3
"""The collision guard does not extrapolate a braking robot past its target.

    python3 test_guard_leg_targets.py

2026-10-21 14:25: Alvik2 braked onto DE1 while Alvik1 pulled off node 0
northbound, 2.52 in between footprints. Constant-velocity extrapolation over
the 0.56 s reaction window carried Alvik2 ~2.5 in past DE1 into Alvik1, the
guard predicted 0.21 in and emergency-stopped both, and Alvik1 then failed its
leg. Robots driving a leg now stop at its end (+0.75 in of coast).
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


INF = math.inf
# The incident, both robots axis-aligned (yaw 0 / 90 make the same square).
a, va = (13.5, 4.0), (0.0, 2.5)      # Alvik1 leaving node 0 northbound
b, vb = (20.0, 3.0), (-4.5, 0.0)     # Alvik2 braking onto DE1 at (19.7, 2.9)
dx, dy = b[0] - a[0], b[1] - a[1]
now_gap = F.footprint_gap_in(dx, dy, 90.0, 270.0)
old = F.predicted_min_gap_in(dx, dy, vb[0] - va[0], vb[1] - va[1], 90.0, 270.0)
check("the incident reproduces under the old prediction",
      F.motion_verdict(now_gap, old, 5.2, math.hypot(dx, dy)) == "avoid",
      f"now {now_gap:.2f} in, predicted {old:.2f} in")

stop_b = (math.hypot(19.7 - b[0], 2.9 - b[1]) + F.LEG_OVERSHOOT_IN) / math.hypot(*vb)
new = F.predicted_min_gap_legs_in(a, va, INF, b, vb, stop_b, 90.0, 270.0)
check("knowing Alvik2 stops at DE1, no stop is called",
      F.motion_verdict(now_gap, new, 5.2, math.hypot(dx, dy)) is None,
      f"predicted {new:.2f} in")

# A genuine head-on: both legs end beyond each other -> still stopped.
p, vp = (10.0, 20.0), (3.2, 0.0)
q, vq = (16.0, 20.0), (-3.2, 0.0)
stop_p = (math.hypot(30.0 - 10.0, 0) + F.LEG_OVERSHOOT_IN) / 3.2
stop_q = (math.hypot(16.0 - 0.0, 0) + F.LEG_OVERSHOOT_IN) / 3.2
gap0 = F.footprint_gap_in(6.0, 0.0, 0.0, 180.0)
pred = F.predicted_min_gap_legs_in(p, vp, stop_p, q, vq, stop_q, 0.0, 180.0)
check("two robots driving at each other are still stopped",
      F.motion_verdict(gap0, pred, 6.4, 6.0) == "avoid", f"predicted {pred:.2f} in")

check("no leg known for either robot: identical to the old prediction",
      abs(F.predicted_min_gap_legs_in(a, va, INF, b, vb, INF, 90.0, 270.0) - old) < 1e-9)

src = open(F.__file__, encoding="utf-8").read()
check("the worker publishes its leg target around drive_leg",
      "_set(self.name, x1, y1)" in src
      and "_clear(self.name)" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
