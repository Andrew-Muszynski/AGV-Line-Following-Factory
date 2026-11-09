#!/usr/bin/env python3
"""When the guard stops a pair that are both driving, the one pulling
clear keeps going; only the one closing in is stopped.

    python3 test_guard_keep_moving.py

2026-11-03 shakedown: Alvik5 had just left node 42 northbound for 50 when
Alvik3, westbound 43->143, closed on it. Both were stopped. Alvik5 sat 2.6in
off 42, inside the turning circle Alvik3 needed at entry 143; Alvik3's turn
was held for Alvik5 and Alvik5's retry for Alvik3, until Alvik3 timed out.
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


def monitor(robots, turning=()):
    """robots: name -> (x, y, yaw, vx, vy, target or None)."""
    v = object.__new__(F.VisionMonitor)
    v._lock = threading.Lock()
    now = time.time()
    v._poses_in = {n: (r[0], r[1], r[2]) for n, r in robots.items()}
    v._pose_in_t = {n: now for n in robots}
    v._motion = {n: (r[3], r[4], 0.0, now) for n, r in robots.items()}
    v._leg_targets = {n: r[5] for n, r in robots.items() if r[5]}
    v._update_motion = lambda: None
    v._turning_locked = lambda n, t: n in turning
    return v


# Navigator convention: dx = sin(yaw), dy = -cos(yaw); 180 faces +y,
# 270 faces -x (west).
A5 = (23.5, 68.6, 180.0, 0.0, 3.0, (23.5, 76.8))   # left 42, going north
A3 = (31.6, 66.8, 270.0, -4.0, 0.0, (28.5, 66.8))  # 43 -> 143, westbound
M = F.COLLISION_PREDICT_MARGIN_IN

print("1. who is stopped")
k = monitor({"Alvik5": A5, "Alvik3": A3}).keep_moving("Alvik3", "Alvik5", M)
check("the 2026-11-03 case: Alvik5 drives on, only Alvik3 stops",
      k == "Alvik5", str(k))
k = monitor({"Alvik5": A5, "Alvik3": A3}).keep_moving("Alvik5", "Alvik3", M)
check("same answer whichever way round the pair is named", k == "Alvik5")

# Head-on along one lane: neither can drive on safely.
H1 = (40.0, 36.8, 90.0, 4.0, 0.0, (43.5, 36.8))
H2 = (47.5, 36.8, 270.0, -4.0, 0.0, (43.5, 36.8))
check("head-on into the same node: stop both",
      monitor({"Alvik1": H1, "Alvik2": H2}).keep_moving("Alvik1", "Alvik2", M)
      is None)

# The mover would still sweep into the other: stop both.
S5 = (23.5, 62.0, 180.0, 0.0, 3.0, (23.5, 76.8))   # still SOUTH of 42
S3 = (27.8, 66.8, 270.0, -1.0, 0.0, (28.5, 66.8))
check("a 'mover' that would pass within the margin does not keep going",
      monitor({"Alvik5": S5, "Alvik3": S3}).keep_moving("Alvik3", "Alvik5", M)
      is None)

check("a robot turning in place is never the one kept moving",
      monitor({"Alvik5": A5, "Alvik3": A3}, turning={"Alvik5"})
      .keep_moving("Alvik3", "Alvik5", M) is None)
crawl = (23.5, 68.6, 180.0, 0.0, 0.2, (23.5, 76.8))
check("a robot barely moving is not 'pulling clear'",
      monitor({"Alvik5": crawl, "Alvik3": A3}).keep_moving("Alvik3", "Alvik5", M)
      != "Alvik5")

print("\n2. driving into a workstation")
# 2026-11-03 10:24: Alvik2 heading into workstation 88 while Alvik4 drove
# 27->35 along the bay's west edge. Both could be let go; the one entering
# the bay must be, or it stops half out in the lane.
B2 = (38.6, 55.4, -2.0, -0.3, -3.0, (38.0, 51.5))
B4 = (33.5, 47.6, 178.0, 0.0, 3.1, (33.5, 56.8))
v = monitor({"Alvik2": B2, "Alvik4": B4})
plain = v.keep_moving("Alvik2", "Alvik4", M)
pref = v.keep_moving("Alvik2", "Alvik4", M, prefer={"Alvik2"})
check("without the preference the guard kept Alvik4 going", plain == "Alvik4",
      str(plain))
check("with it, Alvik2 finishes driving into the bay", pref == "Alvik2",
      str(pref))
x88, y88 = F.nav_node_to_world(88, 8, 8)
vis = types.SimpleNamespace(snapshot_in=lambda: {
    "Alvik2": (38.31, 54.01, -13.5, 0.0), "Alvik3": (x88 + 0.4, y88, 0.0, 0.0)})
check("2.5in short of workstation 88 is NOT arrived (no banking)",
      not F.at_workstation_centre(vis, "Alvik2", 88, 8, 8))
check("0.4in off its centre is arrived",
      F.at_workstation_centre(vis, "Alvik3", 88, 8, 8))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
