#!/usr/bin/env python3
"""The rotation gate guards the ROTATION, not the no-op that follows it.

    python3 test_rotation_gate.py

WHY THIS FILE EXISTS. Until 2026-10-15 every rotation guard keyed on
`PlanItem.kind == "turn"`. In vision mode -- the default, and the mode every
recorded run used -- that item does nothing: fleetSupervisor._execute() says
so outright ("there is nothing for a bare turn/noop item to do here"), and
the rotation is performed at the END of the preceding MOVE.

So the guards fired on a no-op, AFTER the rotation they meant to prevent.
Measured on run 5 of 2026-10-15: Alvik3's rotation finished at t=88.2s and
it was stationary (yaw sd 0.17deg) from t=95s, while the sweep guard refused
to let it "start" that rotation every ~3 s for the next 115 s. The run died
with five robots standing still.

These tests pin both halves of the fix:
  * await_rotation_clearance() measures the turning circle at the moment of
    rotation and holds until it is clear;
  * the old turn-item hold is inert in vision mode, so the no-op can no
    longer livelock a run.
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Same ROS stub the other guard tests use: this file exercises pure
# measurement and gating logic, which must be testable without a robot.
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Vis:
    """snapshot_in() -> {robot: (x_in, y_in, yaw, age_sec)}"""
    def __init__(self, poses):
        self._p = dict(poses)

    def snapshot_in(self):
        return self._p


class _Adv:
    def __init__(self):
        self.blocks, self.cleared = [], []

    def note_vision_block(self, robot, blocker):
        self.blocks.append((robot, blocker))

    def clear_vision_block(self, robot):
        self.cleared.append(robot)


def sup(poses, advisor=None, drive_mode="vision"):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses) if poses is not None else None
    s.advisor = advisor
    s.drive_mode = drive_mode
    s._logged = []
    s.get_logger = lambda: type("L", (), {
        "info": lambda _self, m: s._logged.append(m),
        "error": lambda _self, m: s._logged.append(m)})()
    s._event = lambda *a, **k: None
    return s


print("the measured geometry this gate exists for")
check("a turn sweeps the robot's corner-to-corner diameter",
      abs(F.ROTATION_SWEEP_IN - 5.625) < 1e-9, f"{F.ROTATION_SWEEP_IN}in")
check("a bay entry and its flanking lattice node are INSIDE that",
      5.00 < F.ROTATION_SWEEP_IN,
      "5.00in measured 2026-09-30 -> neither robot can rotate")

print("\n1. rotation_sweep_blocker measures, it does not consult the plan")
clear = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0),
             "Alvik2": (20.0, 0.0, 0.0, 0.0)})
check("alone / far apart -> nothing blocking",
      clear.rotation_sweep_blocker("Alvik1") is None)

tight = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0),
             "Alvik2": (5.00, 0.0, 0.0, 0.0)})
got = tight.rotation_sweep_blocker("Alvik1")
check("entry-to-lattice spacing IS blocking",
      got is not None and got[0] == "Alvik2" and abs(got[1] - 5.0) < 1e-6,
      str(got))

lat = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0),
           "Alvik2": (10.0, 0.0, 0.0, 0.0)})
check("lattice-to-lattice spacing is NOT blocking",
      lat.rotation_sweep_blocker("Alvik1") is None, "10.00in, +4.38in clear")

nearest = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0),
               "Alvik2": (5.5, 0.0, 0.0, 0.0),
               "Alvik3": (4.0, 0.0, 0.0, 0.0)})
got = nearest.rotation_sweep_blocker("Alvik1")
check("reports the closest offender", got is not None and got[0] == "Alvik3",
      str(got))

stale = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0),
             "Alvik2": (4.0, 0.0, 0.0, 99.0)})
check("a stale fix is not evidence of a blocker",
      stale.rotation_sweep_blocker("Alvik1") is None)
check("no vision at all -> no opinion",
      sup(None).rotation_sweep_blocker("Alvik1") is None)

print("\n2. await_rotation_clearance holds, then releases")
adv = _Adv()
s = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (20.0, 0.0, 0.0, 0.0)},
        advisor=adv)
ok, why = s.await_rotation_clearance("Alvik1", "leg 9->1")
check("a clear turning circle passes straight through", ok and why == "clear")
check("and registers no standoff", adv.blocks == [])

adv = _Adv()
s = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (4.0, 0.0, 0.0, 0.0)},
        advisor=adv)
ok, why = s.await_rotation_clearance("Alvik1", "turn at 9", timeout_sec=0.25)
check("a blocked turn is REFUSED, not performed", not ok, why)
check("the reason names the blocker and the distance",
      "Alvik2" in why and "4.00in" in why, why)
check("the standoff is registered for the deadlock resolver",
      ("Alvik1", "Alvik2") in adv.blocks, str(adv.blocks[:2]))
check("and cleared on the way out so it cannot linger",
      "Alvik1" in adv.cleared)

# the neighbour moves away mid-wait
adv = _Adv()
poses = {"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (4.0, 0.0, 0.0, 0.0)}
s = sup(poses, advisor=adv)
calls = {"n": 0}
real = s.vision.snapshot_in


def moving():
    calls["n"] += 1
    if calls["n"] > 2:
        return {"Alvik1": (0.0, 0.0, 0.0, 0.0),
                "Alvik2": (20.0, 0.0, 0.0, 0.0)}
    return real()


s.vision.snapshot_in = moving
ok, why = s.await_rotation_clearance("Alvik1", "turn at 9", timeout_sec=5.0)
check("it waits for the neighbour and then proceeds", ok and why == "clear",
      f"released after {calls['n']} polls")
check("the wait was registered while it lasted", adv.blocks != [])
check("and cleared once the circle was free", "Alvik1" in adv.cleared)

print("\n3. the no-op turn item can no longer livelock a vision run")
# Run 5, 2026-10-15: Alvik3 had ALREADY rotated; the held item was a no-op.


class _Item:
    def __init__(self, kind):
        self.kind = kind


class _St:
    def __init__(self, kind="turn"):
        self.next_index = 0
        self.plan = [_Item(kind)]


class _PlanAdv(_Adv):
    def __init__(self, names):
        super().__init__()
        self.robots = {n: _St() for n in names}


poses = {"Alvik3": (0.0, 0.0, 0.0, 0.0), "Alvik1": (4.0, 0.0, 0.0, 0.0)}
vis = sup(poses, drive_mode="vision")
check("vision mode: the no-op turn item is NOT held",
      vis._rotation_clearance_hold(_PlanAdv(poses), "Alvik3") is None,
      "the rotation is gated at its real call site instead")

col = sup(poses, drive_mode="color")
r = col._rotation_clearance_hold(_PlanAdv(poses), "Alvik3")
check("colour mode: the turn item really does turn, so it IS still held",
      r is not None and "Alvik1" in r, r or "")

print("\n4. an abort does not have to wait out the deadline")
import time as _t  # noqa: E402

adv = _Adv()
s_ab = sup({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (4.0, 0.0, 0.0, 0.0)},
           advisor=adv)
s_ab._abort = True
_t0 = _t.monotonic()
ok, why = s_ab.await_rotation_clearance("Alvik1", "turn at 9", timeout_sec=30.0)
_elapsed = _t.monotonic() - _t0
check("abort releases the hold immediately",
      not ok and "abort" in why.lower(), why)
check("and does not sit out the 30s deadline", _elapsed < 1.0,
      f"{_elapsed:.2f}s")
check("the standoff is still cleared on the way out", "Alvik1" in adv.cleared)

print("\n5. the gate is reachable from the worker")
check("the worker retains the supervisor",
      "self.supervisor = supervisor" in open(
          os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fleetSupervisor.py"), encoding="utf-8",
          errors="replace").read())
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8",
           errors="replace").read()
check("all four real rotation sites are gated",
      src.count("self._clear_to_rotate(") >= 4,
      f"{src.count('self._clear_to_rotate(')} call sites "
      "(square-up + next-leg + yield + turn after dwell)")
check("and the helper reaches the gate", "self.supervisor.await_rotation_clearance(" in src)
check("the yield uses the short deadline",
      "YIELD_ROTATION_CLEAR_TIMEOUT_SEC" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
