#!/usr/bin/env python3
"""The Run tab's Drive mode button chooses the turn method, per mission.

    python3 test_turn_method_selection.py

Until 2026-10-16 the turn method could only be switched by launching
START_TESTBED.bat with an argument (AGV_TURN_METHOD), and the navigator read
that environment variable BEFORE its own arguments -- so nothing a mission
said could change it. The Run tab now sends turn_method with each mission
("Camera only" -> staged, "Camera + Encoder" -> rotate_rel).

These tests pin the precedence, in the navigator AND in the tuning snapshot,
because compare_turn_arms.py groups runs by the snapshot's value: if the two
ever disagree, a run is filed under the arm it did not run.

    mission choice  >  AGV_TURN_METHOD  >  navigator argument
"""
import argparse
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Same ROS stub the other guard tests use.
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
import tuning_snapshot as T  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def nav_ran(mission, env, arg):
    """Which turn law _turn_to_heading_core() dispatches to."""
    nav = object.__new__(N.CameraGridNavigator)
    nav.args = argparse.Namespace(turn_method=arg, mission_turn_method=mission)
    nav.turn_to_heading_staged = lambda *a, **k: "staged"
    nav.turn_to_heading = lambda *a, **k: "wheel"
    # rotate_rel is the fall-through branch: it asks for a pose first. A
    # non-pose here makes it raise straight after, which identifies the
    # branch without driving anything -- staged and wheel return earlier.
    nav.control_pose = lambda: "rotate_rel"
    saved = os.environ.pop("AGV_TURN_METHOD", None)
    try:
        if env:
            os.environ["AGV_TURN_METHOD"] = env
        try:
            out = nav._turn_to_heading_core(90.0, "test")
        except Exception:
            out = "rotate_rel"   # got past both early returns
        return out if out in ("staged", "wheel") else "rotate_rel"
    finally:
        os.environ.pop("AGV_TURN_METHOD", None)
        if saved is not None:
            os.environ["AGV_TURN_METHOD"] = saved


def snap_says(mission, env, arg):
    saved = os.environ.pop("AGV_TURN_METHOD", None)
    try:
        if env:
            os.environ["AGV_TURN_METHOD"] = env
        return T.effective_turn_method(
            argparse.Namespace(turn_method=arg, mission_turn_method=mission))
    finally:
        os.environ.pop("AGV_TURN_METHOD", None)
        if saved is not None:
            os.environ["AGV_TURN_METHOD"] = saved


CASES = [
    # (label, mission, env, arg, expected)
    ("Camera + Encoder button, no launch override", "rotate_rel", None, "staged", "rotate_rel"),
    ("Camera only button, no launch override", "staged", None, "staged", "staged"),
    ("button beats a launch-line override", "rotate_rel", "staged", "staged", "rotate_rel"),
    ("...in the other direction too", "staged", "rotate_rel", "staged", "staged"),
    ("no mission choice -> launch override applies", None, "rotate_rel", "staged", "rotate_rel"),
    ("no mission choice, no override -> argument", None, None, "staged", "staged"),
]

print("navigator and tuning snapshot resolve the same way")
for label, mission, env, arg, want in CASES:
    ran = nav_ran(mission, env, arg)
    rec = snap_says(mission, env, arg)
    check(label, ran == want and rec == want,
          f"ran={ran} recorded={rec} want={want}")

print("\nold navigator args (no mission field) still work")
saved = os.environ.pop("AGV_TURN_METHOD", None)
old = argparse.Namespace(turn_method="staged")
check("snapshot falls back to the argument", T.effective_turn_method(old) == "staged")
if saved is not None:
    os.environ["AGV_TURN_METHOD"] = saved

print("\nthe supervisor hands the mission's choice to every navigator")
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8").read()
check("the worker sets args.mission_turn_method",
      "args.mission_turn_method = turn_method" in src)
check("workers are built with the mission's turn_method",
      "turn_method=turn_method)" in src)
check("the mission loop reads turn_method from the mission",
      'mission.get("turn_method")' in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
