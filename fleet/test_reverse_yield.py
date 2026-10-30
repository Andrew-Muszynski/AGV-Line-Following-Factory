#!/usr/bin/env python3
"""A deadlock yield may back a robot STRAIGHT out when nothing else works.

    python3 test_reverse_yield.py

Operator, 2026-10-30: "We could always have the robot drive straight back
to move to a free node if it is available and there are no other available
movements. The wheel speed would work just the same as it does forward, but
just with negative numbers." The firmware's WHEEL_FOLLOW mode already
clamps each wheel to +/-WHEEL_FOLLOW_MAX_RPM, so no firmware change.

  1. drive_leg(reverse=True), in a kinematic simulation whose wheel->yaw
     sign is taken from how the FORWARD law corrects heading: it backs onto
     the target, tail first, from a tilted start, with both wheels negative.
  2. the resolver uses reverse only as the LAST resort.
  3. the worker drives that hop tail first, with no departure turn.
"""
import math
import os
import sys
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

import camera_grid_navigate as N   # noqa: E402
import dispatch_advisor as da      # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, t):
        self.lines.append(t)

    warning = error = info


class SimNav:
    """Just enough of CameraGridNavigator for drive_leg(), on a diff-drive
    model. Convention as heading_between(): dx = sin(yaw), dy = -cos(yaw).
    Yaw rate is +K*(right-left): with the forward law (left = c - adj,
    right = c + adj, adj = kp * (target - yaw)) that is what makes a
    positive error shrink -- the sign the real robot has."""
    IN_PER_RPM_S = 0.05        # 60 RPM -> 3 in/s
    DEG_PER_RPM_S = 1.5

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.l = self.r = 0.0
        self.t = 0.0
        self.sent = []
        self.args = types.SimpleNamespace(
            cruise_rpm=60.0, brake_lead_in=1.2, max_turn_adjust=20.0,
            kp_yaw=1.5, steer_trim_rpm=3.0, verbose=False,
            aim_lateral_in=None, aim_along_in=None)
        self.errored = False
        self.localization_mode = "camera"
        self.last_pose = None
        self.last_status = "BUSY WHEEL_FOLLOW"
        self.last_failure = None
        self._log = _Log()

    # -- what drive_leg calls --
    def get_logger(self):
        return self._log

    def _wait_for_camera(self, label):
        return True

    def enter_wheel_follow_mode(self, timeout_sec=None):
        return True

    def localization_source_label(self):
        return "camera"

    def fail(self, reason, msg):
        self.last_failure = reason

    def stop_only(self):
        self.l = self.r = 0.0

    def stop_and_verify(self, *a, **k):
        self.stop_only()

    def send_wheel(self, left, right):
        self.l, self.r = left, right
        self.sent.append((left, right))

    def _spin_once(self, _t):
        dt = 1.0 / N.CONTROL_HZ
        v = (self.l + self.r) / 2.0 * self.IN_PER_RPM_S
        self.yaw += (self.r - self.l) * self.DEG_PER_RPM_S * dt
        t = math.radians(self.yaw)
        self.x += v * math.sin(t) * dt
        self.y += -v * math.cos(t) * dt
        self.t += dt

    def control_pose(self):
        return N.Pose(x=self.x, y=self.y, yaw=self.yaw, t=self.t)


N.time.sleep = lambda s: None      # run the 50 Hz loop flat out

print("1. drive_leg(reverse=True) backs onto the target")
# Robot on an entry at (28.5, 36.8) FACING EAST (yaw 90 = +x), 6deg off;
# the free node is 18, 5in WEST at (23.5, 36.8) -- behind it.
nav = SimNav(28.5, 36.8, 96.0)
ok = N.CameraGridNavigator.drive_leg(nav, 23.5, 36.8, "leg 122->18", reverse=True)
end_err = math.hypot(nav.x - 23.5, nav.y - 36.8)
check("it arrives (brakes at the lead, short of the node by about that)",
      ok and end_err <= nav.args.brake_lead_in + 0.3,
      f"ended {end_err:.2f}in from 18, failure={nav.last_failure}")
check("both wheels were negative the whole leg",
      nav.sent and all(l < 0 and r < 0 for l, r in nav.sent),
      f"{len(nav.sent)} setpoints")
check("it stayed on the lane (y within 0.5in of 36.8)",
      abs(nav.y - 36.8) < 0.5, f"y={nav.y:.2f}")
check("and still FACES east -- it backed, it did not turn round",
      abs(((nav.yaw - 90.0 + 180) % 360) - 180) < 6.0, f"yaw={nav.yaw:.1f}")
check("the 6deg tilt was steered out, not grown",
      abs(((nav.yaw - 90.0 + 180) % 360) - 180) < 6.0)
check("the log says REVERSING", any("REVERSING" in t for t in nav._log.lines))

nav = SimNav(28.5, 36.8, 84.0)    # tilted the other way
ok = N.CameraGridNavigator.drive_leg(nav, 23.5, 36.8, "leg 122->18", reverse=True)
check("tilted the other way: also arrives on the lane",
      ok and abs(nav.y - 36.8) < 0.5, f"y={nav.y:.2f} failure={nav.last_failure}")

nav = SimNav(23.5, 36.8, 90.0)
ok = N.CameraGridNavigator.drive_leg(nav, 28.5, 36.8, "leg 18->122")
check("forward driving is unchanged (positive wheels, arrives)",
      ok and all(l > 0 and r > 0 for l, r in nav.sent))


print("\n2. the resolver backs out only when nothing else moves")
R = C = 8
T = da.PlanItem(command="T", kind="turn")


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


def vision_plan(*legs):
    out = []
    for a, b in legs:
        out += [mv(a, b), T]
    return out


class Sim:
    def __init__(self, end):
        self.end_sec = end


def advisor(states, fin):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {}
    for name, (node, plan) in states.items():
        st = object.__new__(da.RobotState)
        st.name, st.current_node, st.plan, st.next_index = name, node, plan, 0
        st.pending, st.current_edge = None, None
        a.robots[name] = st
    a.reservations, a.yielding_robots, a.wait_for = {}, set(), {}
    a._reservation_counter = 0
    a._simulate_fleet = lambda d, n: {k: Sim(v) for k, v in fin.items()}
    return a


# Alvik5 on entry 122 FACING Alvik3, who is on 19 coming through. Neither
# can turn (5.00in apart), Alvik5 faces only Alvik3, Alvik3 faces Alvik5.
# Alvik5's tail points at 18, which is free.
def facing_pair():
    return advisor({"Alvik5": (122, vision_plan((122, 19), (19, 20))),
                    "Alvik3": (19, vision_plan((19, 122), (122, 18), (18, 17)))},
                   {"Alvik3": 340.0, "Alvik5": 300.0})


def drives(r, n, reverse=False):
    return reverse and (r, str(n)) == ("Alvik5", "18")


a = facing_pair()
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0, drives_straight=drives)
check("head to head, no room to turn: Alvik5 backs out", plan is not None
      and plan.robot == "Alvik5" and plan.reverse,
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}"
      f" reverse={getattr(plan, 'reverse', None)}")
check("through 18 to a node off Alvik3's way (18 itself is on it)",
      plan is not None and str(plan.via) == "18"
      and str(plan.to_node) in {"10", "26"},
      f"{plan.to_node if plan else None} via {plan.via if plan else None}")
check("the reason says REVERSING", plan is not None and "REVERSING" in plan.reason)
pair = a.begin_yield(plan, 50.0)
check("the out leg is marked for the worker (YIELD_OUT_REV)",
      pair is not None and pair[0].command == "YIELD_OUT_REV")
check("and the way home is a normal forward leg",
      pair is not None and pair[1].command == "YIELD_BACK")

a = facing_pair()
both = lambda r, n, reverse=False: ((r, str(n)) == ("Alvik5", "18") if reverse
                                    else (r, str(n)) == ("Alvik5", "73"))
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0, drives_straight=both)
check("forward first: facing its free workstation, it pulls in rather than reversing",
      plan is not None and not plan.reverse and str(plan.to_node) == "73",
      f"{plan.to_node if plan else None} reverse={getattr(plan, 'reverse', None)}")

a = facing_pair()
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                          drives_straight=lambda r, n, reverse=False: False)
check("nothing free ahead or behind: declined", plan is None)

a = facing_pair()
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                          drives_straight=lambda r, n: False)
check("a caller without reverse support is handled (declined, no crash)",
      plan is None)


print("\n3. the worker drives that hop tail first")
import fleetSupervisor as F  # noqa: E402
src = open(F.__file__, encoding="utf-8").read()
check("YIELD_OUT_REV's first hop is reversed",
      'rev = (str(item.command) == "YIELD_OUT_REV" and seg_index == 0)' in src)
check("...with no departure turn", "if rev:\n" in src
      and "elif str(item.command).startswith(\"YIELD\"):" in src)
check("...driven with reverse=True",
      "nav.drive_leg(x1, y1, leg_label, reverse=True) if rev" in src)
check("...and on arrival it keeps the way it faces", "heading = facing" in src)
check("the supervisor's heading check knows reverse",
      "travel = (pose.yaw + 180.0) % 360.0 if reverse else pose.yaw" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
