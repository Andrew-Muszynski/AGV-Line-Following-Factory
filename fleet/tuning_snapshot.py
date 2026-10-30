#!/usr/bin/env python3
"""tuning_snapshot.py -- what the fleet was tuned to, captured per run.

WHY THIS EXISTS. The testbed carries ~127 tuned constants across four layers:
the 47 motion parameters handed to each navigator, the advisor's coordination
clearances, the supervisor's vision and collision guards, and the navigator's
localization filters. None of them were recorded with a run.

That is not a cosmetic gap. The design of experiments states that motion
control is held constant while the traffic planner varies -- and the runs
recorded before 2026-10-14 cannot support that claim, because the motion
controller changed twice inside them:

    before 2026-09-24   51 runs   pre-staged turn, kp_yaw 0.6
    2026-09-24..09-25   20 runs   staged turn, approach taper trialled
    after 2026-09-25    35 runs   current: taper off, kp_yaw 1.625

Three controllers pooled into one makespan aggregate, with nothing in the
data to separate them. From now on each run stores the values it actually
executed under, so a later comparison can prove the arms matched instead of
assuming it.

The snapshot is read from the live modules rather than a hand-maintained
list, so a constant that is renamed or retuned cannot silently drift out of
the record.
"""

from __future__ import annotations

import os

#: Module-level constants are captured from these, by name prefix. A plain
#: upper-case numeric module global is tuning by convention in this codebase.
_MODULES = (
    ("coordination", "dispatch_advisor"),
    ("supervisor", "fleetSupervisor"),
    ("localization", "camera_grid_navigate"),
)

#: Prefixes worth capturing per module. Everything else at module scope is
#: geometry or protocol, not tuning, and would bury the values that matter.
_PREFIXES = {
    "coordination": ("ENTRY_", "PRIORITY_", "NODE_", "INTERSECTION_",
                     "SAME_DIRECTION_", "OPPOSITE_", "DEPOT_", "EDGE_",
                     "COLLISION_", "MIN_DWELL", "MAX_DWELL", "DEFAULT_DWELL",
                     "CALIBRATION_", "STANDING_", "VISION_"),
    "supervisor": ("VISION_", "COLLISION_", "EMERGENCY_", "ROTATION_",
                   "RECOVERY_", "YIELD_", "ACK_", "MAX_SEND", "DRIFT_",
                   "ROBOT_SIDE"),
    "localization": ("POSE_", "ODOM_", "CAMERA_ASSIST_", "YAW_JUMP",
                     "CAPTURE_", "VISION_", "CONTROL_HZ", "DEG_PER_SEC"),
}


def _module_constants(module_name: str, prefixes: tuple) -> dict:
    """Numeric upper-case globals matching `prefixes`.

    A module that will not import records WHY under `_unavailable` rather
    than returning silently empty -- "no constants captured" and "this layer
    could not be read" are different claims, and only one of them means the
    run's configuration is unknown. On a laptop without ROS2 or cv2 the
    supervisor and navigator modules do not import at all, which is exactly
    when a blank section would be most misleading.

    Never raises: a snapshot is diagnostic, and failing to take one must not
    stop a run from being dispatched.
    """
    try:
        mod = __import__(module_name)
    except Exception as exc:
        return {"_unavailable": f"{module_name}: {type(exc).__name__}: {exc}"}
    out = {}
    for name in dir(mod):
        if not name.isupper() or not name.startswith(prefixes):
            continue
        val = getattr(mod, name)
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            continue
        out[name] = val
    return out


def navigator_tuning(nav_args) -> dict:
    """The motion parameters actually handed to a navigator.

    Taken from the argparse Namespace the supervisor builds, so it records
    what this run used rather than what the defaults say -- an override from
    the environment (AGV_TURN_METHOD) or a CLI flag is captured too.
    """
    out = {}
    for name in dir(nav_args):
        if name.startswith("_"):
            continue
        val = getattr(nav_args, name)
        if isinstance(val, bool) or isinstance(val, (int, float, str)):
            out[name] = val
    return out


def effective_turn_method(nav_args=None) -> str:
    """Which turn law will actually run.

    camera_grid_navigate._turn_to_heading_core() reads the mission's choice
    (args.mission_turn_method) FIRST, then the environment, and only then
    args.turn_method, so during an A/B they disagree: the args still say
    "staged" while the fleet is running "rotate_rel". Record the resolved
    value, in the same order, because an arm you cannot identify afterwards
    is an arm you did not run.
    """
    mission = getattr(nav_args, "mission_turn_method", None) if nav_args else None
    if mission:
        return mission
    env = os.environ.get("AGV_TURN_METHOD")
    if env:
        return env
    return getattr(nav_args, "turn_method", "rotate_rel") if nav_args else "unknown"


def collect(nav_args=None) -> dict:
    """The whole snapshot. Safe to call at any time; never raises."""
    snap: dict = {"schema": 1}
    try:
        for label, module_name in _MODULES:
            snap[label] = _module_constants(module_name, _PREFIXES[label])
        snap["effective_turn_method"] = effective_turn_method(nav_args)
        if nav_args is not None:
            snap["motion"] = navigator_tuning(nav_args)
        # "motion" is one navigator's args; the per-robot values differ by
        # robot, so record the whole table this run drove with.
        try:
            import json
            with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "robot_drive_calibration.json"),
                      encoding="utf-8") as fh:
                cal = json.load(fh).get("robots")
        except Exception:
            cal = None
        if cal:
            snap["drive_calibration"] = {
                r: {k: v.get(k) for k in ("brake_lead_in", "steer_trim_rpm",
                                          "aim_lateral_in", "aim_along_in")}
                for r, v in cal.items()}
        # Environment overrides that change behaviour fleet-wide without a
        # code edit; absent means the code default was in force.
        env = {k: v for k, v in os.environ.items()
               if k.startswith("AGV_") and len(v) < 200}
        if env:
            snap["env_overrides"] = env
    except Exception as exc:                       # never block a run
        snap["error"] = f"{type(exc).__name__}: {exc}"
    return snap


def summary(snap: dict) -> str:
    """One line for the console, so the operator sees it was captured."""
    if not snap:
        return "no tuning captured"
    n = sum(len(v) for v in snap.values() if isinstance(v, dict))
    motion = snap.get("motion") or {}
    bits = []
    eff = snap.get("effective_turn_method")
    if eff:
        bits.append(f"turn={eff}")
    for key in ("kp_yaw", "cruise_rpm", "staged_fast_rpm",
                "final_yaw_tol_deg"):
        if key in motion:
            bits.append(f"{key}={motion[key]}")
    return f"{n} tuned values" + (f"  ({', '.join(bits)})" if bits else "")


if __name__ == "__main__":
    import json
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    snap = collect()
    print(json.dumps(snap, indent=2, sort_keys=True))
    print("\n" + summary(snap))
