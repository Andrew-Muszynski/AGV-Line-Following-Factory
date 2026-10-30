#!/usr/bin/env python3
"""A camera outage stops the robot; the backlog that follows is not trusted.

    python3 test_camera_outage.py

2026-10-23 10:44: every robot's camera poses stopped reaching the supervisor
for 2.35 s (Alvik1's wheel commands were 322 ms late at the same moment).
Alvik2, mid-leg 11->12 in camera_assist, kept steering on odometry, then took
the ~2 s-old poses that arrived in a burst as current, overshot node 12 and
ended 3.3in off the path -- excluded, run aborted.
"""
import json
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
    warning = error = info


def nav():
    n = object.__new__(N.CameraGridNavigator)
    n.get_logger = lambda: (n.__dict__.setdefault("_log", _Log()))
    n.localization_mode = "camera"          # keep the filter branch out of it
    n.last_pose = None
    n.vision_msgs = 0
    n.vision_max_age = 0.0
    return n


def msg(x, y, capture_age):
    """A vision pose captured `capture_age` seconds ago (camera clock 7 s off)."""
    ms = (time.time() - 7.0 - capture_age) * 1000.0
    return types.SimpleNamespace(data=json.dumps(
        {"x_in": x, "y_in": y, "yaw_deg": 90.0, "ms": ms}))


def src_nav_leg():
    return open(N.__file__, encoding="utf-8").read().split("def drive_leg", 1)[1][:16000]


print("\n1. poses that arrive with the usual delay are used")
n = nav()
for k in range(30):
    n._on_vision_pose(msg(30.0 + 0.1 * k, 26.8, 0.06 + 0.02 * (k % 3)))
check("all accepted", n.vision_msgs == 30 and abs(n.last_pose.x - 32.9) < 1e-6)

print("\n2. the burst after a 2.35 s stall: old poses are dropped")
before = n.last_pose.x
# Nothing has arrived for 2.35 s, as in the real stall.
n.last_pose = N.Pose(n.last_pose.x, n.last_pose.y, n.last_pose.yaw,
                     time.monotonic() - 2.35)
for age, x in ((2.3, 36.9), (1.6, 39.0), (0.9, 41.5)):
    n._on_vision_pose(msg(x, 26.9, age))
check("2 s-old pose not used as current", n.last_pose.x == before,
      f"dropped {getattr(n, 'vision_backlog_dropped', 0)}")
n._on_vision_pose(msg(46.8, 29.5, 0.07))
check("the first current pose is used", abs(n.last_pose.x - 46.8) < 1e-6)

print("\n2b. a delay SHIFT is not a backlog (2026-10-23 Round 1 run 1)")
# The supervisor received every Alvik3 pose; the navigator dropped them for
# ~6 s because their (arrival - capture) moved past the threshold.
n = nav()
for k in range(30):
    n._on_vision_pose(msg(60.0, 56.8, 0.06))
used = n.vision_msgs
for k in range(12):                       # the clock/delay steps up 0.4 s
    n._on_vision_pose(msg(60.0 - 0.2 * k, 56.8, 0.46))
check("at most a handful dropped, then the shifted poses are used",
      n.vision_msgs - used >= 12 - N.VISION_BACKLOG_MAX_RUN
      and abs(n.last_pose.x - (60.0 - 0.2 * 11)) < 1e-6,
      f"used {n.vision_msgs - used} of 12, re-baselined {getattr(n, 'vision_rebaselines', 0)}x")

print("\n2c. drive_leg stops when it overshoots, never U-turns back")
check("overshoot check in drive_leg",
      "dist_to_target > best_dist + LEG_OVERSHOOT_ABORT_IN" in src_nav_leg()
      and "FAIL_OVERSHOT" in src_nav_leg())

print("\n3. camera lost mid-leg: finish the leg on the encoders, wait on the node")
src = open(N.__file__, encoding="utf-8").read()
leg = src.split("def drive_leg", 1)[1][:14000]
check("mid-leg: keeps driving to the node on encoder odometry (logged once)",
      "finishing \"\n                    \"this leg on encoder odometry" in leg
      or "this leg on encoder odometry" in leg)
check("stops where it is only past CAMERA_ASSIST_BLIND_FINISH_SEC (one leg's time)",
      "blind > CAMERA_ASSIST_BLIND_FINISH_SEC" in leg and N.CAMERA_ASSIST_BLIND_FINISH_SEC >= 2.5)
check("never STARTS a leg blind", "if not self._wait_for_camera(leg_label):" in leg[:2500])
turn = src.split("def turn_to_heading_rotate_rel", 1)[1][:1500]
check("never starts a turn blind", "self._wait_for_camera(leg_label)" in turn)

n = nav()
n.localization_mode = "camera_assist"
n.last_pose = N.Pose(43.5, 26.8, 90.0, time.monotonic())
check("camera fine -> no wait at all", n._wait_for_camera("leg 12->20") is True)
calls = []
n.stop_only = lambda: calls.append("stop")
t_start = time.monotonic()
n.last_pose = N.Pose(43.5, 26.8, 90.0, time.monotonic() - 2.0)


def spin2(_s):
    time.sleep(0.02)
    if time.monotonic() - t_start > 0.2:
        n.last_pose = N.Pose(43.6, 26.8, 90.0, time.monotonic())


n._spin_once = spin2
check("on the node with the camera out: waits (already stopped, no STOP sent)",
      n._wait_for_camera("leg 12->20") is True and calls == [], str(calls))
check("and re-arms wheel-follow before continuing the same leg",
      "after a \"\n                        \"camera outage" in leg or "camera outage -- aborting route" in leg)

n = nav()
n.localization_mode = "camera_assist"
calls = []
n.stop_only = lambda: calls.append("stop")
n.stop_and_verify = lambda *a, **k: calls.append("verify")
n.fail = lambda *a, **k: calls.append(("fail", a[0]))
t_start = time.monotonic()


def spin(_s):
    time.sleep(0.02)
    if time.monotonic() - t_start > 0.3:              # camera returns
        n.last_pose = N.Pose(40.0, 26.8, 90.0, time.monotonic())


n._spin_once = spin
n.last_pose = N.Pose(35.0, 26.8, 90.0, time.monotonic() - 0.5)
ok = n._hold_for_camera("leg 11->12")
check("stops, then resumes once fresh poses are back",
      ok is True and calls == ["stop"], str(calls))

n = nav()
n.localization_mode = "camera_assist"
calls = []
n.stop_only = lambda: calls.append("stop")
n.stop_and_verify = lambda *a, **k: calls.append("verify")
n.fail = lambda *a, **k: calls.append(("fail", a[0]))
n._spin_once = lambda s: time.sleep(0.02)
n.last_pose = N.Pose(35.0, 26.8, 90.0, time.monotonic() - 0.5)
saved, N.CAMERA_OUTAGE_HOLD_SEC = N.CAMERA_OUTAGE_HOLD_SEC, 0.4
try:
    ok = n._hold_for_camera("leg 11->12")
finally:
    N.CAMERA_OUTAGE_HOLD_SEC = saved
check("a camera that never returns fails the leg (stopped, not driving)",
      ok is False and ("fail", N.FAIL_VISION_LOST) in calls and calls[0] == "stop")

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
