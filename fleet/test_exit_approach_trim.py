#!/usr/bin/env python3
"""A robot leaving a workstation waits for a robot about to drive onto a
node beside its entry; a straight-on robot may skip a small blocked trim.

    python3 test_exit_approach_trim.py

ALNS24_joint shakedown, 2026-10-30, second run: Alvik4 left bay 30 (94 ->
entry 143) while Alvik5, at 35, was turning toward 43 -- allowed, because
43 was still empty. Alvik5 then drove 35 -> 43 on a straight route to 51,
arrived 5.1deg off, and its trim was held by Alvik4 4.82in away; Alvik4's
90deg turn at 143 toward 43 was held by Alvik5. Deadlock abort.

  A. Operator: the robot in the workstation should wait -- it blocks nobody
     there. The bay-exit rule now also counts a robot whose very next move
     is onto a flank node.
  B. Backstop for the other order (Alvik4 already on 143): a straight-on
     robot skips a small blocked trim when the exact footprints say the
     unsteered drive keeps the guard's margin.
"""
import sys
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

import dispatch_advisor as da      # noqa: E402
import fleetSupervisor as F         # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
DWELL = da.PlanItem(command="DWELL 2000", kind="dwell")


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


class _St:
    def __init__(self, name, node, plan, next_index, pending=None):
        self.name, self.current_node = name, node
        self.plan, self.next_index, self.pending = plan, next_index, pending


def adv(*states):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {s.name: s for s in states}
    a.reservations, a.yielding_robots = {}, set()
    return a


def in_bay(name, ws, entry, *then):
    plan = [mv(entry, ws), DWELL, mv(ws, entry)] + [mv(a, b) for a, b in then]
    return _St(name, ws, plan, 2)


def exit_block(a, name):
    st = a.robots[name]
    return a._bay_exit_blocker(name, st.plan[st.next_index])


print("geometry: entry 143 serves workstation 94 between lattice "
      f"{da.entry_edge_nodes(143, R, C)}")

print("\nA. leaving a workstation while a robot is about to reach a flank")
# Alvik5 still finishing 36 -> 35 (turning at 35 toward 43): in flight.
alvik5 = _St("Alvik5", 36, [mv(36, 35), mv(35, 43), mv(43, 51)], 0,
             pending=("plan", 0))
a = adv(alvik5, in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
r = exit_block(a, "Alvik4")
check("the 2026-10-30 case: Alvik4 waits in bay 30 for Alvik5",
      r is not None and r[0] == "Alvik5" and "about to drive onto 43" in r[1],
      str(r))
alvik5 = _St("Alvik5", 35, [mv(36, 35), mv(35, 43), mv(43, 51)], 1)
a = adv(alvik5, in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
check("also once Alvik5 is recorded at 35",
      (exit_block(a, "Alvik4") or ("",))[0] == "Alvik5")
alvik5 = _St("Alvik5", 35, [mv(36, 35), mv(35, 34), mv(34, 33)], 1)
a = adv(alvik5, in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
check("not held when Alvik5's next move goes elsewhere (35 -> 34)",
      exit_block(a, "Alvik4") is None)
alvik5 = _St("Alvik5", 35, [mv(36, 35), mv(35, 43), mv(43, 143)], 1)
a = adv(alvik5, in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
check("not held by THIS rule when Alvik5 is heading for bay 30's entry",
      exit_block(a, "Alvik4") is None)
a = adv(_St("Alvik5", 43, [mv(35, 43), mv(43, 51)], 1),
        in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
r = exit_block(a, "Alvik4")
check("standing on 43 still holds, as before",
      r is not None and "Alvik5 is at 43" in r[1], str(r))
a = adv(_St("Alvik5", 51, [mv(43, 51), mv(51, 59)], 1),
        in_bay("Alvik4", 94, 143, (143, 43), (43, 44)))
check("once Alvik5 is past (at 51, going 59): Alvik4 leaves",
      exit_block(a, "Alvik4") is None)
a = adv(_St("Alvik5", 35, [mv(36, 35), mv(35, 43)], 1),
        in_bay("Alvik4", 94, 143, (143, 43)))
a.stranded_robots = {"Alvik5"}
check("a stranded robot is not 'about to' move anywhere",
      exit_block(a, "Alvik4") is None)


print("\nB. a straight-on robot may skip a small blocked trim")


class _Vis:
    def __init__(self, poses, turning=()):
        self._p, self._t = poses, set(turning)

    def snapshot_in(self):
        return self._p

    def is_turning(self, n):
        return n in self._t


def trim_clear(poses, nav_yaw, turning=()):
    sup = object.__new__(F.FleetSupervisor)
    sup.vision = _Vis(poses, turning)
    return F.FleetSupervisor.straight_trim_clear(sup, "Alvik5", nav_yaw)


# Navigator convention: dx = sin(yaw), dy = -cos(yaw); 180 = +y (north).
# Measured 2026-10-30 at the abort: Alvik5 (33.3, 66.7) tag yaw -174.94,
# Alvik4 (28.5, 67.0) tag yaw -178.78, 4.82in apart at the same row.
cam = {"Alvik5": (33.3, 66.7, -174.94, 0.0), "Alvik4": (28.5, 67.0, -178.78, 0.0)}
check("nose 5deg AWAY from Alvik4 (east): clear to drive on",
      trim_clear(cam, 175.0) is True)
check("dead straight north: clear to drive on",
      trim_clear(cam, 180.0) is True)
check("nose 5deg TOWARD Alvik4 (the abort, as measured): still clear -- "
      "exact footprints keep 0.63in, unsteered", trim_clear(cam, -175.0) is True)
check("nose 10deg toward Alvik4: refused (0.24in, inside the guard's 0.25in)",
      trim_clear(cam, -170.0) is False)
roomy = dict(cam, Alvik4=(27.8, 67.0, -178.78, 0.0))
check("same 10deg, with Alvik4 on its sticker 5.5in away: clear",
      trim_clear(roomy, -170.0) is True)
check("a neighbour that is turning counts as its swept disc: refused",
      trim_clear(cam, 175.0, turning={"Alvik4"}) is False)
stale = dict(cam, Alvik4=(28.5, 67.0, -178.78, 5.0))
check("a nearby neighbour we cannot see right now: refused",
      trim_clear(stale, 175.0) is False)
far_stale = dict(cam, Alvik4=(80.0, 10.0, 0.0, 5.0))
check("a stale robot far away does not matter", trim_clear(far_stale, 175.0))
own_stale = dict(cam, Alvik5=(33.3, 66.7, -174.94, 5.0))
check("no fresh fix of its own: refused", trim_clear(own_stale, 175.0) is False)


class _Nav:
    class args:
        turn_tol_deg = 4.0

    def __init__(self, yaw):
        self.yaw = yaw

    def control_pose(self):
        return types.SimpleNamespace(yaw=self.yaw)


class _Sup:
    def __init__(self, blocked, clear):
        self._b, self._c, self.lines = blocked, clear, []

    def rotation_blocker(self, name, at_node):
        return ("Alvik4", "Alvik4 is 4.82in away") if self._b else None

    def straight_trim_clear(self, name, yaw, length_in=None):
        return self._c

    def get_logger(self):
        return types.SimpleNamespace(info=self.lines.append)


def skip(yaw, incoming=180.0, nxt=180.0, blocked=True, clear=True):
    w = object.__new__(F.VisionLegWorker)
    w.name, w.nav, w.supervisor = "Alvik5", _Nav(yaw), _Sup(blocked, clear)
    return F.VisionLegWorker._straight_trim_skip(w, 43, incoming, nxt, "turn at 43")


check("5.1deg trim on a straight route, blocked, clear: skipped",
      skip(-174.9) is True)
check("within turn_tol (3deg): normal path, nothing to skip",
      skip(177.0) is False)
check("not blocked: trims as usual", skip(-174.9, blocked=False) is False)
check("drive not clear: waits for the gate as before",
      skip(-174.9, clear=False) is False)
check("a real 90deg turn is never skipped",
      skip(-174.9, incoming=90.0, nxt=180.0) is False)
check("more than 10deg off: waits for the gate", skip(-168.0) is False)

src = open(F.__file__, encoding="utf-8").read()
check("the trim skip sits before step (3)'s gate",
      src.index("if self._straight_trim_skip(") <
      src.index('next_heading, f"turn at {to_label} toward {nfrm}->{nto}",'))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
