#!/usr/bin/env python3
"""Each robot is aimed by its own measured resting offset.

    python3 test_aim_correction.py

Operator, 2026-10-22 (SHAKE ASTAR): "alvik 3 parked close to alvik 2 again
... Alvik 2 and Alvik 3 would collide if they both rotated". Alvik3 rested
0.81in to its left of D3 on 16 of 16 returns (sd 0.15), at the old D3 and
the new one alike. drive_leg now aims that far to its right.
"""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import camera_grid_navigate as N  # noqa: E402
import testbed_coordinates as tc  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


aim3 = {"depot_slot": 0.81, "workstation": 0.34}
de3, d3 = tc.NODES["DE3"], tc.NODES["D3"]

print("\n1. Alvik3 into D3 (driving north; its left is -x, toward Alvik2)")
r = N.aim_corrected_target(de3[0], de3[1], d3[0], d3[1], "leg DE3->D3", aim3)
check("aims 0.81in toward +x (its right, toward Alvik4)",
      r is not None and abs(r[0] - (d3[0] + 0.81)) < 0.01 and abs(r[1] - d3[1]) < 0.05,
      f"{d3} -> ({r[0]:.2f}, {r[1]:.2f})" if r else "None")
# Resting 0.81in to its left of the aim point lands it on D3.
check("so its measured resting offset puts it on D3",
      r is not None and abs((r[0] - 0.81) - d3[0]) < 0.01)
d2 = tc.NODES["D2"]
check("Alvik2-Alvik3 centre spacing back above the 5.625in turning circle",
      (d3[0] - d2[0]) > 5.625 and (31.4 - d2[0]) < 5.625,
      f"on D3: {d3[0] - d2[0]:.2f}in; where it parked today: {31.4 - d2[0]:.2f}in")

print("\n2. Alvik3 into a workstation (driving south; its left is +x)")
e, w = tc.NODES["123"], tc.NODES["74"]
r = N.aim_corrected_target(e[0], e[1], w[0], w[1], "leg 123->74", aim3)
check("aims 0.34in toward -x", r is not None and abs(r[0] - (w[0] - 0.34)) < 0.02,
      f"{w} -> ({r[0]:.2f}, {r[1]:.2f})" if r else "None")

print("\n2b. along the approach (2026-10-23: Alvik3 0.57-0.81in past D3)")
r = N.aim_corrected_target(de3[0], de3[1], d3[0], d3[1], "leg DE3->D3", aim3,
                           {"depot_slot": 0.64})
check("aims 0.64in short of D3 along the approach, and still 0.81in to its right",
      r is not None and abs(r[1] - (d3[1] - 0.64)) < 0.08 and abs(r[0] - (d3[0] + 0.81)) < 0.1,
      f"{d3} -> ({r[0]:.2f}, {r[1]:.2f})" if r else "None")
r = N.aim_corrected_target(de3[0], de3[1], d3[0], d3[1], "leg DE3->D3", {}, {"depot_slot": 0.64})
check("an along correction alone also applies", r is not None and r[1] < d3[1] - 0.5)
cal3 = json.load(open(os.path.join(HERE, "robot_drive_calibration.json"),
                      encoding="utf-8"))["robots"]["Alvik3"]
check("Alvik3 carries its along aim (0.30in after its lead went to 1.67in)",
      0.2 < cal3.get("aim_along_in", {}).get("depot_slot", 0) < 0.5 and cal3["brake_lead_in"] > 1.6)

print("\n3. never applied anywhere else")
check("lattice leg untouched", N.aim_corrected_target(13.5, 16.75, 23.5, 16.75, "leg 1->2", aim3) is None)
check("depot entry / lane untouched",
      N.aim_corrected_target(25.9, 3.0, 32.1, 3.1, "leg DE2->DE3", aim3) is None)
check("a robot with no aim untouched",
      N.aim_corrected_target(de3[0], de3[1], d3[0], d3[1], "leg DE3->D3", {}) is None)
check("departure-turn label suffix still parsed",
      N.aim_corrected_target(e[0], e[1], w[0], w[1], "leg 123->74: departure", aim3) is not None)

print("\n4. calibration file and wiring")
cal = json.load(open(os.path.join(HERE, "robot_drive_calibration.json"), encoding="utf-8"))["robots"]
check("Alvik3 carries its depot and workstation aims",
      cal["Alvik3"].get("aim_lateral_in", {}).get("depot_slot", 0) > 0.5)
src = open(os.path.join(HERE, "fleetSupervisor.py"), encoding="utf-8").read()
check("the supervisor hands the aim to each navigator", "args.aim_lateral_in =" in src)
nsrc = open(N.__file__, encoding="utf-8").read()
check("drive_leg applies it", "aim_corrected_target(" in nsrc.split("def drive_leg", 1)[1][:8000])

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
