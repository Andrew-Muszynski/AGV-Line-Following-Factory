#!/usr/bin/env python3
"""A late WHEEL_FOLLOW_MODE acknowledgement is waited for, watchdog fed.

    python3 test_mode_ack.py

2026-10-26 Round 2 run 10: Alvik1's WHEEL_FOLLOW_MODE reached it ~3 s late,
twice. drive_leg gave up after 3 s (MODE_ACK_LOST), stopped feeding the
robot just as it entered the mode, and the firmware's 1 s watchdog raised
ERROR WHEEL_CMD_TIMEOUT: a robot whose command had arrived was excluded.
"""
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


def robot(ack_after):
    n = object.__new__(N.CameraGridNavigator)
    n.args = types.SimpleNamespace(verbose=False)
    n.errored = False
    n.last_wheel_cmd_at = None
    n.wait_for_cmd_match = lambda: True
    feeds = []
    t0 = time.monotonic()
    n.send_cmd = lambda c: None

    def send_wheel(l, r):
        feeds.append(time.monotonic() - t0)
        n.last_wheel_cmd_at = time.monotonic()
    n.send_wheel = send_wheel

    def spin(s):
        time.sleep(0.05)
        if time.monotonic() - t0 >= ack_after:
            n.mode_ack_seen = True
    n._spin_once = spin
    return n, feeds


print("\n1. a reply ~4 s late is waited for")
n, feeds = robot(ack_after=4.0)
ok = n.enter_wheel_follow_mode(timeout_sec=N.WHEEL_FOLLOW_ACK_TIMEOUT_SEC)
gaps = [b - a for a, b in zip(feeds, feeds[1:])]
check("armed instead of MODE_ACK_LOST", ok is True)
check("the watchdog was fed throughout (no gap near the 300 ms brake)",
      gaps and max(gaps) < 0.3, f"largest gap {max(gaps) * 1000:.0f} ms")

print("\n2. a robot that never answers still fails, after the longer wait")
n, _ = robot(ack_after=99)
t = time.monotonic()
saved = N.WHEEL_FOLLOW_ACK_TIMEOUT_SEC
ok = n.enter_wheel_follow_mode(timeout_sec=1.0)
check("no reply -> False", ok is False and time.monotonic() - t < 1.5)

src = open(N.__file__, encoding="utf-8").read()
leg = src.split("def drive_leg", 1)[1][:6000]
check("drive_leg waits WHEEL_FOLLOW_ACK_TIMEOUT_SEC (8 s), not the 3 s default",
      "enter_wheel_follow_mode(timeout_sec=WHEEL_FOLLOW_ACK_TIMEOUT_SEC)" in leg
      and N.WHEEL_FOLLOW_ACK_TIMEOUT_SEC >= 6.0)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
