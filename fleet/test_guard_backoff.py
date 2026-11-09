#!/usr/bin/env python3
"""A guard-stopped robot backs out to its leg's start node and holds,
instead of retrying the same leg into the robot it was stopped against.

    python3 test_guard_backoff.py

ALNS24_seq, November 2, 2026, pair 1: Alvik4 was granted 31->23 while
Alvik3 stood at entry 127 (beside 23), turned to leave toward 23. The guard
stopped Alvik4 4.4in short. Each retry was resynced to 23 -- so the corridor
gate skipped 23 -- and drove further in; the third approach was a contact
(5.60in apart, 0.02in overlap) and the run ended.

  1. the back-off decision, on the measured poses
  2. the hold: until the other robot moves, fed to the wait-for graph
  3. the reservation for the reverse hop
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
from camera_grid_navigate import heading_between  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


def xy(n):
    return F.nav_node_to_world(n, R, C)


class _Vis:
    def __init__(self, poses):
        self.p = poses

    def snapshot_in(self):
        return self.p

    def is_turning(self, _n):
        return False


class _Adv:
    rows, cols = R, C

    def __init__(self, occupied=None):
        self.occ = occupied or {}
        self.blocks = {}

    def _occupied_nodes(self):
        return self.occ

    def note_vision_block(self, robot, blocker):
        self.blocks[robot] = blocker

    def clear_vision_block(self, robot):
        self.blocks.pop(robot, None)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, t):
        self.lines.append(t)

    warning = error = info


def sup(poses):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses)
    s._guard_holds = {}
    log = _Log()
    s.get_logger = lambda: log
    return s


x23, y23 = xy(23)
x31, y31 = xy(31)
x127, y127 = xy(127)
h_in = heading_between(x31, y31, x23, y23)      # Alvik4 facing 23
h_out = heading_between(x127, y127, x23, y23)   # Alvik3 turned toward 23
print(f"nodes: 23 {xy(23)}, 31 {xy(31)}, 127 {xy(127)}")

# Measured at the second guard stop (telemetry, in inches).
A4 = (73.72, 40.50, h_in + 9.2, 0.0)
A3 = (78.21, 36.71, h_out + 2.3, 0.0)

print("\n1. what to do after a guard stop")
s = sup({"Alvik4": A4, "Alvik3": A3})
plan = F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(31, 23),
                                             "Alvik3")
check("the 2026-11-02 case: back straight out from 23's side to 31",
      plan == ("reverse", (23, 31)), str(plan))
gap_now = F.footprint_gap_in(A3[0] - A4[0], A3[1] - A4[1], h_in, A3[2])
gap_31 = F.footprint_gap_in(A3[0] - x31, A3[1] - y31, h_in, A3[2])
check("and backing out only opens the gap to Alvik3",
      gap_31 > gap_now, f"{gap_now:.2f}in now, {gap_31:.2f}in at 31")

s = sup({"Alvik4": A4, "Alvik3": A3, "Alvik6": (x31, y31 + 3.0, h_in, 0.0)})
check("a robot just behind it: no reverse (ordinary retry path)",
      F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(31, 23),
                                            "Alvik3") is None)
s = sup({"Alvik4": A4, "Alvik3": A3})
check("31 held by another robot: no reverse",
      F.FleetSupervisor._guard_backoff_plan(
          s, _Adv({"31": "Alvik6"}), "Alvik4", mv(31, 23), "Alvik3") is None)
s = sup({"Alvik4": (x31, y31 - 1.0, h_in, 0.0), "Alvik3": A3})
check("stopped within 1.5in of 31: no reverse, just hold",
      F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(31, 23),
                                            "Alvik3") == ("hold", None))
# Partner BEHIND (stopped against a robot coming up from 31's side).
s = sup({"Alvik4": A4, "Alvik3": (x31, y31 - 1.0, h_in + 180.0, 0.0)})
check("the robot it was stopped against is behind it: no reverse into it",
      F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(31, 23),
                                            "Alvik3") is None)
s = sup({"Alvik4": A4, "Alvik3": A3})
check("a workstation leg keeps the old path",
      F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(127, 78),
                                            "Alvik3") is None)
s = sup({"Alvik4": (A4[0], A4[1], A4[2], 5.0), "Alvik3": A3})
check("no fresh pose of its own: no decision",
      F.FleetSupervisor._guard_backoff_plan(s, _Adv(), "Alvik4", mv(31, 23),
                                            "Alvik3") is None)

print("\n2. the hold")
poses = {"Alvik4": (x31, y31, h_in, 0.0), "Alvik3": A3}
s = sup(poses)
a = _Adv()
F.FleetSupervisor._start_guard_hold(s, "Alvik4", "Alvik3")
r = F.FleetSupervisor._guard_hold_reason(s, a, "Alvik4")
check("held while Alvik3 stands at 127", r is not None and "Alvik3" in r, str(r))
check("and the hold is in the wait-for graph", a.blocks.get("Alvik4") == "Alvik3")
poses["Alvik3"] = (A3[0] - 1.0, A3[1], h_out, 0.0)
check("a 1in creep is not 'moved'",
      F.FleetSupervisor._guard_hold_reason(s, a, "Alvik4") is not None)
poses["Alvik3"] = (x23, y23, h_out, 0.0)
check("Alvik3 drives to 23: the hold ends",
      F.FleetSupervisor._guard_hold_reason(s, a, "Alvik4") is None)
check("and leaves the graph", "Alvik4" not in a.blocks)
check("and is gone for good", "Alvik4" not in s._guard_holds)
s = sup(dict(poses, Alvik3=A3))
F.FleetSupervisor._start_guard_hold(s, "Alvik4", "Alvik3")
s._guard_holds["Alvik4"]["since"] -= F.GUARD_HOLD_MAX_SEC + 1.0
check(f"bounded: lifted after {F.GUARD_HOLD_MAX_SEC:.0f}s even if it never moves",
      F.FleetSupervisor._guard_hold_reason(s, _Adv(), "Alvik4") is None)
s = sup(poses)
check("a robot with no hold is not held",
      F.FleetSupervisor._guard_hold_reason(s, _Adv(), "Alvik1") is None)

print("\n3. the reverse hop is reserved like any move")


def seg(i, frm, to):
    return {"segment_index": i, "from": frm, "to": to, "start_sec": 0.0,
            "end_sec": 0.0, "wait_before_sec": 0.0, "move_sec": 3.56,
            "turn_sec": 0.0, "type": "RED_TO_RED"}


sched = {"grid": {"rows": R, "cols": C}, "agents": 2, "routes": [
    {"agent": 4, "agv": "Alvik4", "launch_offset_sec": 0.0,
     "segments": [seg(1, 30, 31), seg(2, 31, 23), seg(3, 23, 15)]},
    {"agent": 6, "agv": "Alvik6", "launch_offset_sec": 0.0,
     "segments": [seg(1, 39, 40)]}]}
cmds = {"Alvik4": ["FORWARD_UNTIL_RED"] * 3, "Alvik6": ["FORWARD_UNTIL_RED"]}
adv = da.DispatchAdvisor(sched, cmds, llm=None)
item = adv.begin_backoff("Alvik4", 23, 31, 0.0)
check("granted: a tail-first hop 23->31",
      item is not None and item.command == "YIELD_OUT_REV"
      and (item.from_node, item.to_node) == (23, 31), str(item))
check("holding a reservation", "Alvik4" in adv.reservations)
check("and the edge it is on", adv.robots["Alvik4"].current_edge == ("23", "31"))
adv = da.DispatchAdvisor(sched, cmds, llm=None)
adv._grant_reservation("Alvik6", -1, adv._standing_request(31), 0.0)
check("refused while another robot holds 31",
      adv.begin_backoff("Alvik4", 23, 31, 0.0) is None)
check("same node both ends: refused", adv.begin_backoff("Alvik4", 31, 31, 0.0) is None)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
