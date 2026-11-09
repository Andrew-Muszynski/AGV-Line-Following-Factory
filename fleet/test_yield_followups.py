#!/usr/bin/env python3
"""Recovery fixes held back during Experiment 2 (applied November 9, 2026).

    python3 test_yield_followups.py

  1. A robot's turn is not held by a reservation segment the other robot
     has already driven (Alvik1's square-up in ws 73, held 57 s for
     "Alvik6 is driving 11->19" with Alvik6 at 122; twice on Nov 9).
  2. A two-hop yield's HOME is where the way back ends, not its middle node.
  3. An abandoned yield is booked where the camera sees the robot.
  4. The resolver hears from the camera that a robot has no room to turn,
     and backs it out instead of retrying a sidestep the rotation gate will
     never allow (Alvik3 at node 9 beside Alvik5 in bay 1, 53 s on Nov 9).
"""
import os
import sys
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

import dispatch_advisor as da      # noqa: E402
import fleetSupervisor as F         # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
xy = lambda n: F.nav_node_to_world(n, R, C)


class _Vis:
    def __init__(self, poses):
        self.p = poses

    def snapshot_in(self):
        return self.p


def sup(poses, advisor=None):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses)
    s.advisor = advisor
    return s


def at(n, dx=0.0, dy=0.0, yaw=0.0):
    x, y = xy(n)
    return (x + dx, y + dy, yaw, 0.0)


adv = types.SimpleNamespace(rows=R, cols=C)
SEGS = [(11, 19, 2.0), (19, 122, 2.0)]

print("1. a reservation segment already driven does not hold a turn")
left = F.FleetSupervisor._undriven_segments(sup({"Alvik6": at(122)}, adv),
                                            "Alvik6", SEGS)
check("Nov 9: Alvik6 at 122 after 11->19->122: nothing left", left == [],
      str(left))
left = F.FleetSupervisor._undriven_segments(sup({"Alvik6": at(11)}, adv),
                                            "Alvik6", SEGS)
check("still at 11: the whole way is ahead of it", left == SEGS, str(left))
x11, y11 = xy(11)
x19, y19 = xy(19)
left = F.FleetSupervisor._undriven_segments(
    sup({"Alvik6": ((x11 + x19) / 2, (y11 + y19) / 2, 180.0, 0.0)}, adv),
    "Alvik6", SEGS)
check("halfway up 11->19: that segment and the next", left == SEGS, str(left))
left = F.FleetSupervisor._undriven_segments(sup({"Alvik6": at(19)}, adv),
                                            "Alvik6", SEGS)
check("at 19: only 19->122 is left", left == SEGS[1:], str(left))
left = F.FleetSupervisor._undriven_segments(sup({}, adv), "Alvik6", SEGS)
check("not seen: assume none of it is driven (safe side)", left == SEGS)

edges = F.workstation_rotation_edges(73, R, C)
res = types.SimpleNamespace(request=types.SimpleNamespace(segments=SEGS))
a = types.SimpleNamespace(rows=R, cols=C, robots={"Alvik1": 0, "Alvik6": 0},
                          reservations={"Alvik6": res})
blk = F.FleetSupervisor.workstation_lane_blocker(
    sup({"Alvik6": at(122)}, a), "Alvik1", 73)
check("Alvik1's square-up in ws 73 is no longer held by it", blk is None,
      f"{blk} (bay edges {sorted(tuple(sorted(e)) for e in edges)})")
blk = F.FleetSupervisor.workstation_lane_blocker(
    sup({"Alvik6": at(11)}, a), "Alvik1", 73)
check("but Alvik6 still waiting at 11 to drive 11->19 does hold it",
      blk is not None, str(blk))

print("\n2. a two-hop yield's home is where the way back ends")
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8").read()
check("home = back_item.to_node", '"home": back_item.to_node' in src)
plan = da.YieldPlan(robot="Alvik6", kind="backtrack", from_node=122,
                    to_node=11, blocked_robot="Alvik1", reason="test",
                    via=19, reverse=False)
adv2 = object.__new__(da.DispatchAdvisor)
adv2.rows, adv2.cols = R, C
st = object.__new__(da.RobotState)
st.name, st.current_node, st.plan, st.next_index = "Alvik6", 122, [], 0
st.pending, st.current_edge = None, None
adv2.robots = {"Alvik6": st}
adv2.reservations, adv2.yielding_robots, adv2.wait_for = {}, set(), {}
adv2._reservation_counter = 0
pair = adv2.begin_yield(plan, 0.0)
check("122 -> 19 -> 11: the way back ends at 122, its middle is 19",
      pair is not None and str(pair[1].to_node) == "122"
      and str(pair[0].from_node) == "19",
      f"out {pair[0].from_node}->{pair[0].to_node}, back ->{pair[1].to_node}"
      if pair else "no pair")

print("\n3. an abandoned yield is booked where the robot is")
y = {"home": 122, "out": types.SimpleNamespace(from_node=19, to_node=11)}
n = F.FleetSupervisor._nearest_yield_node(sup({"Alvik6": at(122, 0.4)}, adv),
                                          adv, "Alvik6", y)
check("Nov 9: Alvik6 standing at 122 is booked at 122, not 11", n == 122,
      str(n))
n = F.FleetSupervisor._nearest_yield_node(sup({"Alvik6": at(11)}, adv),
                                          adv, "Alvik6", y)
check("standing aside at 11: booked at 11", n == 11, str(n))
n = F.FleetSupervisor._nearest_yield_node(
    sup({"Alvik6": at(11, dx=-10.0)}, adv), adv, "Alvik6", y)
check("between nodes: not booked anywhere (left as it was)", n is None)

print("\n4. no room to turn, measured: back out, do not sidestep")
T = da.PlanItem(command="T", kind="turn")
mv = lambda a_, b_: da.PlanItem(command="F", kind="move", from_node=a_,
                                to_node=b_)


class _Sim:
    def __init__(self, end):
        self.end_sec = end


def fleet():
    a_ = object.__new__(da.DispatchAdvisor)
    a_.rows, a_.cols = R, C
    a_.robots = {}
    # Alvik3 on 9 heading south to 1 (came from 17); Alvik5 in bay 1's
    # workstation (65) waiting to leave through entry 114.
    for name, node, plan in (("Alvik3", 9, [mv(9, 1), T, mv(1, 0)]),
                             ("Alvik5", 65, [mv(65, 114), T, mv(114, 10)])):
        s = object.__new__(da.RobotState)
        s.name, s.current_node, s.plan, s.next_index = name, node, plan, 0
        s.pending, s.current_edge = None, None
        a_.robots[name] = s
    a_.reservations, a_.yielding_robots, a_.wait_for = {}, set(), {}
    a_._reservation_counter = 0
    a_._simulate_fleet = lambda d, n_: {"Alvik3": _Sim(300.0),
                                        "Alvik5": _Sim(340.0)}
    return a_


tail_to_17 = lambda r, n_, reverse=False: reverse and (r, str(n_)) == ("Alvik3", "17")
p = fleet().resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                             can_yield=lambda r: r == "Alvik3",
                             drives_straight=tail_to_17)
check("without the camera's answer: a sidestep that needs a turn (the bug)",
      p is not None and not p.reverse,
      f"{p.robot}->{p.to_node} reverse={p.reverse}" if p else "None")
p = fleet().resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                             can_yield=lambda r: r == "Alvik3",
                             drives_straight=tail_to_17,
                             can_rotate=lambda r: r != "Alvik3")
check("with it: Alvik3 backs straight out to 17, no turn",
      p is not None and p.robot == "Alvik3" and p.reverse
      and str(p.to_node) in ("17",) + tuple(str(n_) for n_ in
                                            F.FleetSupervisor.__dict__ and ()),
      f"{p.robot}->{p.to_node} via {p.via} reverse={p.reverse}" if p else "None")
p = fleet().resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                             can_yield=lambda r: r == "Alvik3",
                             drives_straight=lambda r, n_, reverse=False: False,
                             can_rotate=lambda r: r != "Alvik3")
check("nothing free behind or ahead either: declined, not a doomed turn",
      p is None)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
