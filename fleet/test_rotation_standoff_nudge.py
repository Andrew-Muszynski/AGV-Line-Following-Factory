#!/usr/bin/env python3
"""Two stopped robots each holding a turn for the other: one backs up just
far enough for the other to turn, then drives back onto its node. And a
yield does not go home into a neighbour's turning circle.

    python3 test_rotation_standoff_nudge.py

ALNS24_joint, November 2, 2026: Alvik2 at entry 137 had to turn into bay
88. Alvik4 came home from a yield to node 35, 4.87in away, and had to turn
toward 27. Each turn was held for the other; later Alvik5 came home to 36
beside it too. Nothing moved until the run was ended by hand.
"""
import sys
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


class _Vis:
    def __init__(self, poses, turning=()):
        self.p, self.t = poses, set(turning)

    def snapshot_in(self):
        return self.p

    def is_turning(self, n):
        return n in self.t


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, t):
        self.lines.append(t)

    warning = error = info


def sup(poses, holds=None, turning=()):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses, turning)
    s._rot_holds = dict(holds or {})
    s.advisor = None
    log = _Log()
    s.get_logger = lambda: log
    s._event = lambda *a, **k: None
    return s


# Navigator convention: dx = sin(yaw), dy = -cos(yaw). 180 faces +y.
A4 = (33.5, 56.9, 180.0, 0.0)     # at 35, came north from 27
A2 = (38.4, 56.8, 90.0, 0.0)      # at entry 137, came east from 35
MUTUAL = {"Alvik4": "Alvik2", "Alvik2": "Alvik4"}

print("1. who backs up, and how far")
s = sup({"Alvik4": A4, "Alvik2": A2}, MUTUAL)
p4 = F.FleetSupervisor._standoff_nudge(s, "Alvik4", "Alvik2", 3.0)
p2 = F.FleetSupervisor._standoff_nudge(s, "Alvik2", "Alvik4", 3.0)
check("the 2026-11-02 case: Alvik4 backs up toward 27", p4 is not None, str(p4))
check("by about 3.5in, inside the 4in limit",
      p4 is not None and 3.2 <= p4[0] <= 4.0, f"{p4[0]:.1f}in" if p4 else "")
check("Alvik2 does not (backing up would bring it closer)", p2 is None)
check("not before the hold has lasted 2.5s",
      F.FleetSupervisor._standoff_nudge(s, "Alvik4", "Alvik2", 1.0) is None)
s = sup({"Alvik4": A4, "Alvik2": A2}, {"Alvik4": "Alvik2"})
check("not when only ONE robot is holding (the other will move on)",
      F.FleetSupervisor._standoff_nudge(s, "Alvik4", "Alvik2", 9.0) is None)
s = sup({"Alvik4": A4, "Alvik2": A2,
         "Alvik6": (33.5, 51.0, 180.0, 0.0)}, MUTUAL)
check("not into a robot behind it",
      F.FleetSupervisor._standoff_nudge(s, "Alvik4", "Alvik2", 3.0) is None)
s = sup({"Alvik4": A4, "Alvik2": (A2[0], A2[1], A2[2], 5.0)}, MUTUAL)
check("not without a fresh fix of the other robot",
      F.FleetSupervisor._standoff_nudge(s, "Alvik4", "Alvik2", 3.0) is None)
# Both can back up (facing apart): only the higher name does.
B1 = (33.5, 56.9, 0.0, 0.0)       # tail toward +y
B2 = (38.4, 56.8, 0.0, 0.0)
s = sup({"Alvik1": B1, "Alvik3": B2}, {"Alvik1": "Alvik3", "Alvik3": "Alvik1"})
a = F.FleetSupervisor._standoff_nudge(s, "Alvik1", "Alvik3", 3.0)
b = F.FleetSupervisor._standoff_nudge(s, "Alvik3", "Alvik1", 3.0)
check("both able: exactly one backs up (the higher name)",
      a is None and b is not None, f"{a} / {b}")

print("\n2. the hold loop backs up, waits for the turn, and drives back")


class _Loop:
    """await_rotation_clearance() on a stub: Alvik4 is blocked while
    Alvik2 is within the sweep; Alvik2 turns once Alvik4 is clear."""

    def __init__(self):
        self.poses = {"Alvik4": list(A4), "Alvik2": list(A2)}
        self.s = sup({}, {"Alvik2": "Alvik4"})
        self.s.vision.snapshot_in = lambda: {k: tuple(v) for k, v in self.poses.items()}
        self.s.rotation_blocker = self.blocker
        self.drives = []
        self.t0 = time.monotonic()

    def gap(self):
        a, b = self.poses["Alvik4"], self.poses["Alvik2"]
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def blocker(self, name, at_node):
        if self.gap() < F.ROTATION_SWEEP_IN:
            return "Alvik2", f"Alvik2 is {self.gap():.2f}in away"
        # Alvik4 is clear: Alvik2 turns and drives into its bay.
        self.s._rot_holds.pop("Alvik2", None)
        self.poses["Alvik2"] = [38.4, 51.6, 0.0, 0.0]
        return None

    def nudge(self, dist, label):
        self.drives.append(("back", round(dist, 1)))
        self.poses["Alvik4"][1] -= dist
        return True

    def drive_to(self, x, y, label):
        self.drives.append(("return", round(x, 1), round(y, 1)))
        self.poses["Alvik4"][0], self.poses["Alvik4"][1] = x, y
        return True


F.STANDOFF_NUDGE_SEC = 0.05
F.ROTATION_CLEAR_POLL_SEC = 0.01
L = _Loop()
ok, why = F.FleetSupervisor.await_rotation_clearance(
    L.s, "Alvik4", "turn at 35 toward 35->27", timeout_sec=None,
    at_node=35, idle=lambda t: time.sleep(t), nudge=L.nudge,
    drive_to=L.drive_to)
check("the turn is cleared", ok, why)
check("Alvik4 backed up once, then drove back onto 35",
      [d[0] for d in L.drives] == ["back", "return"], str(L.drives))
check("ending where it started", tuple(L.poses["Alvik4"][:2]) == (33.5, 56.9))
check("and no hold is left registered", "Alvik4" not in L.s._rot_holds)

print("\n3. a yield does not go home into a turning circle")
s = sup({"Alvik4": (33.5, 46.8, 0.0, 0.0), "Alvik2": A2})
adv = types.SimpleNamespace(rows=8, cols=8)
r = F.FleetSupervisor._turning_circle_crowd(s, adv, "Alvik4", ["35"])
check("Alvik2 at entry 137, 5in from 35: stay out", r is not None, str(r))
s = sup({"Alvik4": (33.5, 46.8, 0.0, 0.0), "Alvik2": (38.4, 51.6, 0.0, 0.0)})
check("Alvik2 down in bay 88 (7in away): go home",
      F.FleetSupervisor._turning_circle_crowd(s, adv, "Alvik4", ["35"]) is None)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
