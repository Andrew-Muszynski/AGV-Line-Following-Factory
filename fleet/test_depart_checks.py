#!/usr/bin/env python3
"""A robot never drives a leg it is not on, or not pointed down.

    python3 test_depart_checks.py

2026-10-16, run on a new BRKGA route (13:57). Alvik4 reached entry 137 and
was correctly held from turning toward bay 88 -- Alvik6 was 4.90in away at
node 36. Then, in order:

  1. the turn failed the instant the hold ended: "no fresh encoder +
     camera-assisted odometry yaw". The hold SLEPT, and a navigator only
     reads its topics while its worker spins it -- after 0.75s its odometry
     is stale. The same thing failed every held turn that day (D2, D3, 137);
  2. the drive to 137 was banked (correctly -- the robot was there), and
     the next move 137->88 was dispatched with Alvik4 still facing east.
     drive_leg() only trims heading: it arced off the tape in a circle;
  3. vision resynced it to node 29, and the retry of "137->88" drove from
     there -- 13in off that leg, diagonally across the grid -- until the
     guard stopped it beside Alvik3.

Also: a rotation claim outlived its 0.46s turn and held Alvik3 in its bay.
"""
import os
import sys
import threading
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
import dispatch_advisor as da  # noqa: E402
import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
W = lambda n: F.nav_node_to_world(n, R, C)  # noqa: E731
P = lambda x, y, yaw: types.SimpleNamespace(x=x, y=y, yaw=yaw)  # noqa: E731


class _Nav:
    def __init__(self, cam=None, yaw=None, yaw_after_spin=None):
        self.cam, self.yaw, self.yaw_after_spin = cam, yaw, yaw_after_spin
        self.args = types.SimpleNamespace(rows=R, cols=C, turn_tol_deg=4.0,
                                          final_yaw_tol_deg=2.0)
        self.localization_mode = "camera"
        self.turns, self.drives, self.spins, self.stops = [], [], 0, 0
        self.last_failure = None

    def fresh_pose(self):
        return self.cam

    def control_pose(self):
        return None if self.yaw is None else types.SimpleNamespace(yaw=self.yaw)

    def _spin_once(self, t):
        self.spins += 1
        if self.yaw is None and self.yaw_after_spin is not None:
            self.yaw = self.yaw_after_spin

    def stop_only(self, *a):
        self.stops += 1

    def drive_leg(self, x, y, label):
        self.drives.append(label)
        return True

    def turn_to_heading_rotate_rel(self, heading, label, tol_deg=None):
        self.turns.append((label, round(heading)))
        self.yaw = heading
        return True


class _Sup:
    def __init__(self):
        self.gates, self.warnings, self.released = [], [], []
        self.vision = types.SimpleNamespace(
            release_rotation=lambda n: self.released.append(n))

    def await_rotation_clearance(self, name, label, timeout_sec=None,
                                 cancelled=None, at_node=None, idle=None):
        self.gates.append((label, at_node, idle is not None))
        return True, "clear"

    def get_logger(self):
        return types.SimpleNamespace(warning=lambda m: self.warnings.append(m),
                                     info=lambda m: None, error=lambda m: None)


def worker(nav, sup=None):
    w = object.__new__(F.VisionLegWorker)
    w.name, w.nav, w.supervisor = "Alvik4", nav, sup or _Sup()
    w._cancel = threading.Event()
    return w


x137, y137 = W(137)
x88, y88 = W(88)
h_bay = N.heading_between(x137, y137, x88, y88)

# --------------------------------------------------------------------------
print("1. a rotation hold keeps the navigator reading its topics")


class _Vis:
    def __init__(self, poses):
        self.poses = poses

    def snapshot_in(self):
        return self.poses


s = object.__new__(F.FleetSupervisor)
s.vision = _Vis({"Alvik4": (x137, y137, 90.0, 0.0),
                 "Alvik6": (x137 + 4.9, y137, 90.0, 0.0)})
s.advisor = None
s.get_logger = lambda: types.SimpleNamespace(info=lambda m: None)
s._event = lambda *a, **k: None
ticks = []


def idle(sec):
    ticks.append(sec)
    if len(ticks) >= 4:                       # Alvik6 moves on
        s.vision.poses["Alvik6"] = (x137 + 20.0, y137, 90.0, 0.0)


ok, why = s.await_rotation_clearance("Alvik4", "turn at 137", timeout_sec=None,
                                     idle=idle)
check("the hold calls idle() instead of sleeping, every poll",
      ok and len(ticks) >= 4, f"{len(ticks)} polls")
check("the worker hands its spinner to the gate",
      worker(_Nav(cam=None, yaw=90.0))._clear_to_rotate is not None
      and "idle=self._idle" in open(os.path.join(os.path.dirname(
          os.path.abspath(__file__)), "fleetSupervisor.py"),
          encoding="utf-8").read())
nav = _Nav()
worker(nav)._idle(0.05)
check("_idle spins the navigator in short slices", nav.spins >= 3,
      f"{nav.spins} spins in 50ms")

# --------------------------------------------------------------------------
print("\n2. the rotation claim ends with the turn")
sup = _Sup()
w = worker(_Nav(yaw=4.7), sup)
w._turn(0.0, "turn at 18 toward 18->10")
check("released when the turn returns", sup.released == ["Alvik4"])


class _Boom(_Nav):
    def turn_to_heading_rotate_rel(self, *a, **k):
        raise RuntimeError("firmware went away")


sup = _Sup()
try:
    worker(_Boom(), sup)._turn(0.0, "x")
except RuntimeError:
    pass
check("...and released even if the turn raises", sup.released == ["Alvik4"])

# --------------------------------------------------------------------------
print("\n3. never drive a leg you are not pointed down")
nav = _Nav(cam=P(x137, y137, 90.0), yaw=90.0)        # Alvik4, facing east
sup = _Sup()
ok, why = worker(nav, sup)._ready_to_depart(137, x137, y137, x88, y88,
                                            "leg 137->88", N.heading_between)
check("Alvik4 at 137 facing east, leg into bay 88: turns FIRST",
      ok and why == "turned" and nav.turns == [("leg 137->88: departure turn",
                                                round(h_bay))], str(nav.turns))
check("...through the rotation gate, at the entry",
      sup.gates and sup.gates[0][1] == 137 and sup.gates[0][2], str(sup.gates))
check("...and says so in the log", any("turning before driving" in m
                                       for m in sup.warnings))
nav = _Nav(cam=P(x137, y137, h_bay + 3.0), yaw=h_bay + 3.0)
ok, why = worker(nav)._ready_to_depart(137, x137, y137, x88, y88,
                                       "leg 137->88", N.heading_between)
check("an aimed robot (3deg off) just drives", ok and why == "aimed"
      and nav.turns == [])
nav = _Nav(cam=P(x137, y137, 90.0), yaw=None, yaw_after_spin=90.0)
ok, why = worker(nav)._ready_to_depart(137, x137, y137, x88, y88,
                                       "leg 137->88", N.heading_between)
check("stale heading at the start of a job: reads topics, then decides",
      ok and why == "turned" and nav.spins > 0, why)

# --------------------------------------------------------------------------
print("\n4. never drive a leg you are not on")
nav = _Nav(cam=P(51.0, 47.0, 90.0), yaw=90.0)       # where vision put it
ok, why = worker(nav)._ready_to_depart(137, x137, y137, x88, y88,
                                       "leg 137->88", N.heading_between)
check("Alvik4 resynced to node 29: '137->88' from there is REFUSED",
      not ok and why.startswith("OFF_PATH") and nav.turns == []
      and nav.stops == 1, why)
x17, y17 = W(17)
x18, y18 = W(18)
x26, y26 = W(26)
nav = _Nav(cam=P(x18 - 1.3, y18, 90.0), yaw=N.heading_between(x18, y18, x26, y26))
ok, why = worker(nav)._ready_to_depart(18, x18, y18, x26, y26, "leg 18->26",
                                       N.heading_between)
check("stopped 1.3in short of 18 (normal brake) -> may drive 18->26", ok, why)
nav = _Nav(cam=P(x17 + 4.7, y17 + 0.2, 90.0), yaw=90.0)
ok, why = worker(nav)._ready_to_depart(17, x17, y17, x18, y18, "leg 17->18",
                                       N.heading_between)
check("guard-stopped partway ALONG its own leg -> may resume it", ok, why)

# --------------------------------------------------------------------------
print("\n5. end to end: the banked-then-misaimed move turns before it drives")
nav = _Nav(cam=P(x137, y137, 90.0), yaw=90.0)
w = worker(nav)
item = da.PlanItem(command="FWD", kind="move", from_node=137, to_node=88)
ok, note, _ = w._execute(item, None, False, da.movement_segments,
                         N.heading_between)
order = [t[0] for t in nav.turns]
check("move 137->88 from an east-facing robot: turn, then drive",
      ok and order and order[0] == "leg 137->88: departure turn"
      and nav.drives == ["leg 137->88"], f"turns={order} drives={nav.drives}")

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
