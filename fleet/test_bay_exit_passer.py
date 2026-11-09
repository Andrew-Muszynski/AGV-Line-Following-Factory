#!/usr/bin/env python3
"""A robot leaving a workstation waits for a robot driving past the bay,
and a robot just dispatched out of a bay is not "parked" to the guard.

    python3 test_bay_exit_passer.py

ALNS24_seq, November 3, 2026, pair 1: the bay-exit gate released Alvik3
from workstation 78 65ms after Alvik4 was granted 23->15 -- a robot whose
move is in flight counts as at its destination. Alvik4 drove down the lane
beside the bay while Alvik3 pulled out; the guard judged Alvik3 "parked"
(no speed yet), stopped only Alvik4, and the pair touched (0.02in overlap).
"""
import sys
import threading
import time
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

import fleetSupervisor as F         # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8


class _Vis:
    def __init__(self, poses, speeds, driving=()):
        self.p, self.s, self.d = poses, speeds, set(driving)

    def snapshot_in(self):
        return self.p

    def speed_in(self, n):
        return self.s.get(n)

    def driving_leg(self, n):
        return n in self.d

    def is_turning(self, _n):
        return False


def sup(vis):
    s = object.__new__(F.FleetSupervisor)
    s.vision = vis
    return s


adv = types.SimpleNamespace(rows=R, cols=C)
req = types.SimpleNamespace(segments=[(78, 127, 2.0)])
x78, y78 = F.nav_node_to_world(78, R, C)
x127, y127 = F.nav_node_to_world(127, R, C)
x23, y23 = F.nav_node_to_world(23, R, C)
x15, y15 = F.nav_node_to_world(15, R, C)
x31, y31 = F.nav_node_to_world(31, R, C)
print(f"nodes: 78 ({x78:.1f},{y78:.1f}) 127 ({x127:.1f},{y127:.1f}) "
      f"23 ({x23:.1f},{y23:.1f}) 15 ({x15:.1f},{y15:.1f})")

# Measured as Alvik3 was released, t=99.9s into the 12:58 run (inches).
A3 = (78.1, 32.1, 180.0, 0.0)
A4_start = (73.7, 36.8, 2.9, 0.0)
A4_mid = (73.7, 32.0, 2.9, 0.0)

print("1. the exit waits for a robot driving past")
fn = F.FleetSupervisor._bay_exit_passer
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": A4_start}, {"Alvik4": 0.4},
                driving={"Alvik4"})), adv, "Alvik3", req)
check("the 2026-11-03 case: Alvik4 just left 23 down the lane -> hold",
      r is not None and "Alvik4" in r, str(r))
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": A4_mid}, {"Alvik4": 3.2})),
       adv, "Alvik3", req)
check("alongside the bay at cruise -> hold", r is not None, str(r))
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": (x15, y15 - 2.0, 2.9, 0.0)},
                {"Alvik4": 3.2}, driving={"Alvik4"})), adv, "Alvik3", req)
check("gone past 15 -> go", r is None, str(r))
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": A4_mid}, {"Alvik4": 0.0})),
       adv, "Alvik3", req)
check("a robot standing beside the bay is left to the node gate", r is None)
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": (x31 + 10.0, y31, 90.0, 0.0)},
                {"Alvik4": 3.2}, driving={"Alvik4"})), adv, "Alvik3", req)
check("driving a lane a bay away -> go", r is None, str(r))
r = fn(sup(_Vis({"Alvik3": A3, "Alvik4": A4_start}, {"Alvik4": 3.2},
                driving={"Alvik4"})), adv, "Alvik3",
       types.SimpleNamespace(segments=[(23, 24, 2.0)]))
check("not a workstation exit -> not this rule", r is None)

print("\n2. a robot dispatched out of its bay is not parked")
v = object.__new__(F.VisionMonitor)
v._lock = threading.Lock()
v._poses_in = {"Alvik3": (78.1, 32.1, 180.0)}
check("no leg target: not driving", not v.driving_leg("Alvik3"))
v.set_leg_target("Alvik3", x127, y127)
check("granted 78->127, 4.7in to go: driving", v.driving_leg("Alvik3"))
v._poses_in["Alvik3"] = (x127, y127 - 0.5, 180.0)
check("within 1in of the target: done driving", not v.driving_leg("Alvik3"))
v.clear_leg_target("Alvik3")
check("leg over: not driving", not v.driving_leg("Alvik3"))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
