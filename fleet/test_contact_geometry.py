#!/usr/bin/env python3
"""How close two robots may get depends on their HEADINGS, not just distance.

The robot is 4.0 x 4.0 in on top (operator measurement, 2026-10-06). That
same robot measures 5.625 in corner to corner, and 4.0 * sqrt(2) = 5.657 --
the two figures are one robot seen from two angles. Which applies depends
entirely on whether anybody is turning:

    squared up (parallel, perpendicular, nose to tail)   4.00 in
    both at 45 degrees to the line between them          5.66 in
    either robot ROTATING                                5.625 in circle

THE FAILURE THIS FIXES. On 2026-10-06 Alvik1 held 180 degrees at a
workstation while Alvik3 drove past it. The guard called a collision at
4.74 in against a flat 5.625 in threshold and emergency-stopped both,
failing the run. Physically the pair had room: neither was turning, and
Alvik1 was not going to until Alvik3 had cleared the adjacent node.

Raising the contact distance to the corner-to-corner figure was my own
over-reach when fixing the 2026-09-30 rotation collision. The corner figure
is right for a rotation and wrong for a pass.

ROTATION IS COVERED ELSEWHERE, before the turn starts --
test_rotation_clearance.py (positional) and test_entry_rotation.py
(symbolic). Keeping the two apart is what lets a pass-by be judged on the
real footprint without weakening the rotation case.

    python3 test_contact_geometry.py
"""
import math
import sys
import types

for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import fleetSupervisor as F          # noqa: E402

cd = F.contact_distance_in
PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


print("the two measurements describe one robot")
check("robot side is the measured 4.0 in",
      abs(F.ROBOT_SIDE_IN - 4.0) < 1e-9, f"{F.ROBOT_SIDE_IN}")
check("side x sqrt(2) reproduces the corner measurement",
      abs(F.ROBOT_SIDE_IN * math.sqrt(2) - 5.657) < 0.001,
      f"{F.ROBOT_SIDE_IN * math.sqrt(2):.3f} in vs 5.625 in measured")
check("the rotation sweep stays larger than the footprint",
      F.ROTATION_SWEEP_IN > F.ROBOT_SIDE_IN,
      f"{F.ROTATION_SWEEP_IN} vs {F.ROBOT_SIDE_IN}")

print("\nsquared up -- how these robots sit almost all of the time")
for ya, yb, lbl in ((0, 0, "nose to tail"),
                    (90, 90, "side by side"),
                    (0, 90, "perpendicular"),
                    (180, 90, "the 2026-10-06 pair"),
                    (270, 180, "any multiple of 90")):
    got = cd(ya, yb, 1, 0)
    check(f"squared up, {lbl} -> 4.00 in", abs(got - 4.0) < 1e-6, f"{got:.3f} in")

print("\noff-axis")
worst = cd(45, 45, 1, 0)
check("both at 45 degrees is the worst case",
      abs(worst - F.ROBOT_SIDE_IN * math.sqrt(2)) < 1e-6, f"{worst:.3f} in")
check("a few degrees of slop stays near 4 in",
      cd(5, -5, 1, 0) < 4.5, f"{cd(5, -5, 1, 0):.3f} in")
check("contact never exceeds the rotation circle",
      max(cd(a, b, 1, 0) for a in range(0, 360, 5) for b in range(0, 360, 5))
      <= F.ROTATION_SWEEP_IN + 0.04,
      "swept every heading pair at 5 degree steps")
check("contact never drops below the robot's own width",
      min(cd(a, b, 1, 0) for a in range(0, 360, 5) for b in range(0, 360, 5))
      >= F.ROBOT_SIDE_IN - 1e-6)

print("\nmissing data falls back to the worst case, never the best")
check("one heading unknown", cd(0, None, 1, 0) > cd(0, 0, 1, 0),
      f"{cd(0, None, 1, 0):.3f} in")
check("both unknown -> the full rotation circle",
      abs(cd(None, None, 1, 0) - F.ROTATION_SWEEP_IN) < 1e-9)
check("coincident centres cannot define a direction -> worst case",
      abs(cd(0, 0, 0, 0) - F.ROTATION_SWEEP_IN) < 1e-9)

print("\nthe pass that was wrongly stopped (2026-10-06)")
SEP, CLOSING = 4.74, 0.5
check("a flat corner-to-corner threshold calls it a collision",
      F.collision_verdict(SEP, CLOSING, contact_in=5.625) == "collision",
      "this is the behaviour being fixed")
for ang in (0, 90):
    dx, dy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    c = cd(180, 90, dx, dy)
    check(f"heading-aware ({ang} deg between centres) lets the pass proceed",
          F.collision_verdict(SEP, CLOSING, contact_in=c) is None,
          f"contact {c:.2f} in")

d45 = cd(225, 135, 1, 0)
check("the SAME separation at 45 degrees is still stopped",
      F.collision_verdict(SEP, CLOSING, contact_in=d45) == "collision",
      f"contact {d45:.2f} in -- their corners really are nearest then")

print("\nreal overlap is still a collision at any heading")
for ya, yb in ((0, 0), (45, 45), (0, 90)):
    check(f"3.5 in apart at headings {ya}/{yb}",
          F.collision_verdict(3.5, 0.0, contact_in=cd(ya, yb, 1, 0)) == "collision")

print("\nthe default is unchanged when no heading is supplied")
check("omitting contact_in uses COLLISION_CONTACT_IN",
      F.collision_verdict(5.0, 0.0) ==
      F.collision_verdict(5.0, 0.0, contact_in=F.COLLISION_CONTACT_IN))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
