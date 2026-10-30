#!/usr/bin/env python3
"""STOP is judged by the camera AND the robot's own encoders.

    python3 test_stop_encoders.py

Operator, 2026-10-22: use the robot's own wheel readings, as well as the
camera, to know whether it has stopped. The firmware already publishes encoder
odometry (alvik.get_pose()) every 50 ms; still wheels give a constant pose.
2026-10-22: Alvik1 kept turning ~12deg/s through four STOPs, and the log could
not say whether the STOPs arrived or whether the wheels were turning. Now
every STOP line carries both.
"""
import math
import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy"].ok = lambda: True
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import camera_grid_navigate as N  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m): self.lines.append(m)
    error = warning = info


def robot(camera, encoders, ack=True):
    """camera/encoders: functions of elapsed seconds -> (x, y, yaw) or None."""
    nav = object.__new__(N.CameraGridNavigator)
    log = _Log()
    nav.get_logger = lambda: log
    t0 = time.monotonic()

    def send_cmd(c):
        if c == "STOP" and ack:
            nav._on_status(types.SimpleNamespace(data="STOPPED"))
    nav.send_cmd = send_cmd
    nav.args = types.SimpleNamespace(verbose=False)
    nav._spin_once = lambda s: time.sleep(0.02)

    def pose(fn):
        def get():
            now = time.monotonic()
            v = fn(now - t0)
            return None if v is None else N.Pose(v[0], v[1], v[2], now)
        return get
    nav.fresh_pose = pose(camera)
    nav.fresh_odom_pose = pose(encoders)
    return nav, log


print("\n1. camera sees it still: verified, as before")
nav, log = robot(lambda t: (10.0, 10.0, 90.0), lambda t: (5.0, 5.0, 12.0))
check("verified", nav.stop_and_verify("leg", timeout_sec=1.0) is True, log.lines[-1])

print("\n2. no camera frames: the encoders alone confirm it")
nav, log = robot(lambda t: None, lambda t: (5.0, 5.0, 12.0))
ok = nav.stop_and_verify("leg", timeout_sec=1.0)
check("verified by encoders", ok is True and "by the robot's encoders" in log.lines[-1],
      log.lines[-1])

print("\n3. the 2026-10-22 Alvik1 case: still turning ~12deg/s through every STOP")
nav, log = robot(lambda t: (10.0, 10.0, 90.0 + 12.0 * t),
                 lambda t: (5.0, 5.0, 12.0 + 12.0 * t))
ok = nav.stop_and_verify("leg", timeout_sec=1.0)
line = log.lines[-1]
check("not verified", ok is False)
check("the log says the WHEELS are still turning", "encoders: MOVING" in line, line[-150:])
check("and how many STOPs the robot acknowledged", "STOPPED acks 3 of 3 sent" in line)

print("\n4. STOPs never acknowledged are visible")
nav, log = robot(lambda t: (10.0, 10.0, 90.0 + 12.0 * t),
                 lambda t: (5.0, 5.0, 12.0 + 12.0 * t), ack=False)
nav.stop_and_verify("leg", timeout_sec=1.0)
check("0 acks reported", "STOPPED acks 0 of 3 sent" in log.lines[-1])

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
