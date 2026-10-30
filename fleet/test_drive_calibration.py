#!/usr/bin/env python3
"""One node table for both programs; per-robot brake lead and steering trim.

    python3 test_drive_calibration.py

Operator, 2026-10-22: "We must put them into a single file that they both use
so that it is consistent", and "Alvik 3 will consistently return to its depot
node and stop a bit closer to alvik 2 than 4", and "fix the stopping distance
per robot with the real data that we have gathered".

Measured (tools/analyze_stops.py, 3019 legs 2026-10-15..21): Alvik3 held a
-4.75deg standing heading error on every lattice leg and stopped 0.52in to its
own left -- facing north into D3, that is toward D2. Coast after the STOP ran
1.23 (Alvik2) to 1.43in (Alvik5) against one fleet-wide 1.38in brake lead.
"""
import json
import math
import os
import shutil
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import camera_grid_navigate as nav  # noqa: E402
import testbed_coordinates as tc  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


print("\n1. one node table")
labels = ([str(i) for i in range(1, 163)] + ["0"]
          + [f"D{i}" for i in range(1, 8)] + [f"DE{i}" for i in range(1, 8)])
check("all 177 nodes are listed", all(l in tc.NODES for l in labels))
check("the navigator drives to exactly the listed positions",
      all(nav.node_to_world(l, 8, 8) == tc.NODES[l] for l in labels))
check("node 74 is still where it was measured", tc.NODES["74"] == (38.32, 31.72))
check("D3 sits between D2 and D4",
      tc.NODES["D2"][0] < tc.NODES["D3"][0] < tc.NODES["D4"][0])
try:
    import apriltag_localize as loc
except Exception as exc:          # cv2 / pupil_apriltags absent on this machine
    loc = None
    print(f"  (skipping the localiser half: {type(exc).__name__}: {exc})")
if loc is not None:
    check("the localiser reads the same file",
          loc.COORDINATES_ID == nav.COORDINATES_ID)
    check("...and puts every numbered node in the same place",
          all(loc.node_to_world(i, 8, 8) == nav.node_to_world(i, 8, 8)
              for i in range(1, 163)))
check("other grid sizes keep the formula", nav.node_to_world(5, 4, 4) == (13.5, 26.75))
for name in ("camera_grid_navigate.py", os.path.join("..", "apriltag_localize.py")):
    src = open(os.path.join(HERE, name), encoding="utf-8").read()
    check(f"{os.path.basename(name)} carries no literal table any more",
          "65: (18.30, 21.76)" not in src and '"D3": (32.40, 10.70)' not in src)

tmp = tempfile.mkdtemp()
try:
    p = os.path.join(tmp, tc.FILENAME)
    shutil.copy(tc.PATH, p)
    tc.save({"74": (38.40, 31.70)}, "2099-01-01", path=p)
    blob = json.load(open(p, encoding="utf-8"))
    check("calibrate_workstations --apply path: one node changes, the rest stay",
          blob["nodes"]["74"] == [38.4, 31.7] and blob["nodes"]["75"] == [48.07, 31.76]
          and len(blob["nodes"]) == len(tc.NODES) and blob["updated"] == "2099-01-01")
finally:
    shutil.rmtree(tmp)

print("\n2. per-robot calibration reaches each navigator")
cal = json.load(open(os.path.join(HERE, "robot_drive_calibration.json"),
                     encoding="utf-8"))["robots"]
check("all six robots calibrated", sorted(cal) == [f"Alvik{i}" for i in range(1, 7)])
check("brake leads are the measured coasts (1.1-1.8in)",
      all(1.1 <= v["brake_lead_in"] <= 1.8 for v in cal.values()))
check("Alvik3 carries the largest steering trim",
      max(cal, key=lambda r: abs(cal[r]["steer_trim_rpm"])) == "Alvik3")

import fleetSupervisor as F  # noqa: E402

built = {}


class _FakeNav:
    def __init__(self, name, args):
        built[name] = args


nav.CameraGridNavigator, _real = _FakeNav, nav.CameraGridNavigator
try:
    for r in ("Alvik3", "Alvik5"):
        F.VisionLegWorker(r, supervisor=None)
finally:
    nav.CameraGridNavigator = _real
check("Alvik3's navigator gets its own lead and trim",
      built["Alvik3"].brake_lead_in == cal["Alvik3"]["brake_lead_in"]
      and built["Alvik3"].steer_trim_rpm == cal["Alvik3"]["steer_trim_rpm"])
check("Alvik5's differs from Alvik3's",
      built["Alvik5"].brake_lead_in != built["Alvik3"].brake_lead_in)
saved = F.ROBOT_DRIVE_CALIBRATION_FILE
F.ROBOT_DRIVE_CALIBRATION_FILE = os.path.join(HERE, "no_such_file.json")
try:
    check("no calibration file -> fleet defaults, not a crash",
          F.robot_drive_calibration() == {})
finally:
    F.ROBOT_DRIVE_CALIBRATION_FILE = saved

import tuning_snapshot  # noqa: E402
snap = tuning_snapshot.collect(built["Alvik3"])
check("every run records all six robots' calibration",
      snap.get("drive_calibration", {}).get("Alvik6", {}).get("brake_lead_in")
      == cal["Alvik6"]["brake_lead_in"])

print("\n3. the trim removes the bow (drive_leg's own law on a pulling robot)")


KP = 1.625


def leg(trim, pull_rpm=7.72, clamp=10.0):
    """A 10in leg toward +y on a robot whose wheels pull: the yaw rate is
    proportional to (correction + pull), so a P-only loop settles where
    KP*err = -pull. Returns (median mid-leg heading error, sideways offset
    at the brake point), the two things analyze_stops.py measures."""
    x, y, yaw = 0.0, 0.0, 0.0                  # yaw measured from +y, + toward +x
    errs = []
    for _ in range(5000):
        dx, dy = 0.0 - x, 10.0 - y
        if math.hypot(dx, dy) <= 1.38:
            break
        err = math.degrees(math.atan2(dx, dy)) - yaw
        if math.hypot(dx, dy) >= 2.5 and y > 1.0:
            errs.append(err)
        u = max(-clamp, min(clamp, KP * err)) + trim
        yaw += (u + pull_rpm) * 1.8 * 0.02     # 1.8 deg/s per RPM, 50 Hz
        x += 2.5 * math.sin(math.radians(yaw)) * 0.02
        y += 2.5 * math.cos(math.radians(yaw)) * 0.02
    errs.sort()
    return errs[len(errs) // 2], x


e0, bow0 = leg(trim=0.0)
trim = KP * e0                                 # analyze_stops.py's formula
e1, bow1 = leg(trim=trim)
check("untrimmed: a standing heading error and a sideways offset",
      abs(e0) > 3.0 and abs(bow0) > 0.2, f"err {e0:+.2f}deg, offset {bow0:+.2f}in")
# The mid-leg median includes the start-of-leg transient, so it over-reads
# the true standing error by ~15% and one pass slightly over-corrects (well
# under a degree); a second analyze_stops.py pass takes out the remainder.
check("trim = kp_yaw x measured error removes both",
      abs(e1) < 1.0 and abs(bow1) < abs(bow0) / 4,
      f"trim {trim:+.2f}RPM -> err {e1:+.2f}deg, offset {bow1:+.2f}in")
src = open(os.path.join(HERE, "camera_grid_navigate.py"), encoding="utf-8").read()
check("drive_leg adds the trim outside the clamp",
      'turn_adjust += getattr(self.args, "steer_trim_rpm", 0.0)' in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
