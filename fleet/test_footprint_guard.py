#!/usr/bin/env python3
"""Robots are judged by their footprints and where they are actually going.

    python3 test_footprint_guard.py

2026-10-16, afternoon run. Alvik1 stood at entry 118 (between nodes 13 and
14, in front of workstation 69). Alvik3 drove north up the east leg of that
cell, 6->14, passing 5in to Alvik1's side. Operator: "the robots would still
have an inch in between them if they are perpendicular or parallel and 5
inches apart." The supervisor nevertheless:

  1. emergency-stopped both at 7.18in -- the closing-rate model extrapolated
     the diagonal approach and took contact along the diagonal (5.66in);
  2. held Alvik3 off node 14 and Alvik1 out of its own bay, each "near" a
     node on the other's path by a flat 0.6-cell radius -- a deadlock;
  3. "banked" both failed legs, which recorded nothing (record_done after
     cancel_dispatch is a no-op), so the plan still had Alvik1 at 13 and
     Alvik3 at 6;
  4. resolved the deadlock with yields "13->5" and "6->5" from those wrong
     nodes -- two diagonal drives across the grid, into each other.

It also let Alvik1 START its 95deg turn at entry 118 while Alvik3 was
already granted node 14, 5.00in away. Each section pins one fix.
"""
import os
import sys
import threading
import time
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

import dispatch_advisor as da  # noqa: E402
import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
g = F.footprint_gap_in
W = lambda n: F.nav_node_to_world(n, R, C)  # noqa: E731

# --------------------------------------------------------------------------
print("1. footprint gap: the operator's 1 inch")
for stagger in (0.0, 1.0, 2.0, 3.0, 4.0):
    check(f"perpendicular, 5in to the side, {stagger:.0f}in staggered: 1.00in",
          abs(g(5.0, stagger, 90.0, 180.0) - 1.0) < 1e-9,
          f"{g(5.0, stagger, 90.0, 180.0):.3f}in")
check("parallel, 5in apart: 1.00in", abs(g(5.0, 0.0, 90.0, 90.0) - 1.0) < 1e-9)
check("nose to tail 6in apart: 2.00in", abs(g(0.0, 6.0, 0.0, 0.0) - 2.0) < 1e-9)
check("square-on at 4.0in: touching", abs(g(4.0, 0.0, 0.0, 0.0)) < 1e-9)
check("both at 45deg, 5in apart: overlapping", g(5.0, 0.0, 45.0, 45.0) < 0.0)
check("a turning robot (disc) beside a square one at 5.0in: 0.17in",
      abs(g(5.0, 0.0, None, 90.0) - (5.0 - 2.0 - F.ROBOT_SIDE_IN / 2 ** 0.5))
      < 1e-9, f"{g(5.0, 0.0, None, 90.0):.3f}in")
check("argument order does not matter",
      abs(g(3.0, 4.0, 10.0, None) - g(-3.0, -4.0, None, 10.0)) < 1e-9)
check("the old centre-line model called the 2in stagger 'needs 5.14in'",
      F.contact_distance_in(93.0, 180.0, 5.0, 2.0) > 5.0)

# --------------------------------------------------------------------------
print("\n2. the guard predicts along the real motion")
# Alvik1 still at entry 118, yaw 93; Alvik3 4.9in east and 5.2in south of
# it, heading north (yaw 180) at 4.2in/s -- the first stop, measured.
pg = F.predicted_min_gap_in(4.9, -5.2, 0.0, 4.2, 93.0, 180.0)
v = F.motion_verdict(g(4.9, -5.2, 93.0, 180.0), pg, 4.2, 7.14)
check("today's pass-by: no stop", v is None, f"min gap {pg:.2f}in, verdict {v}")
pg_t = F.predicted_min_gap_in(4.9, -5.2, 0.0, 4.2, None, 180.0)
v_t = F.motion_verdict(g(4.9, -5.2, None, 180.0), pg_t, 4.2, 7.14)
check("the same pass-by WHILE Alvik1 turns: stop", v_t == "avoid",
      f"min gap {pg_t:.2f}in")
head = F.predicted_min_gap_in(7.0, 0.0, -8.0, 0.0, 90.0, 90.0)
check("head-on at 8in/s relative, 7in apart: stop",
      F.motion_verdict(g(7.0, 0, 90, 90), head, 8.0, 7.0) == "avoid")
sep = F.predicted_min_gap_in(4.3, 0.0, 3.0, 0.0, 90.0, 90.0)
check("close but SEPARATING: never stopped",
      F.motion_verdict(g(4.3, 0, 90, 90), sep, 3.0, 4.3) is None)
check("overlap now: collision",
      F.motion_verdict(-0.1, -0.2, 0.0, 3.9) == "collision")
check("barely moving (noise): no stop",
      F.motion_verdict(0.3, 0.1, 0.2, 4.3) is None)
check("parked pair keeps its own rule (6.16in slots never trip)",
      F.motion_verdict(-0.5, -0.5, 3.0, 5.2, parked_pair=True) is None
      and F.motion_verdict(-1.0, -1.0, 0.0, 3.9, parked_pair=True)
      == "collision")

# --------------------------------------------------------------------------
print("\n3. VisionMonitor measures velocity and turning from poses")
vm = object.__new__(F.VisionMonitor)
vm._lock = threading.Lock()
vm._poses, vm._prev_sep = {}, {}
now = time.time()
vm._poses_in = {"Alvik1": (58.5, 26.8, 93.0), "Alvik3": (63.5, 20.0, 180.0)}
vm._pose_in_t = {"Alvik1": now - 0.30, "Alvik3": now - 0.30}
vm.motion_pairs_in()                                   # baseline
vm._poses_in["Alvik3"] = (63.5, 21.26, 180.0)          # +1.26in in 0.30s
vm._pose_in_t = {"Alvik1": now, "Alvik3": now}
pairs = vm.motion_pairs_in()
a, b, s, gap, mg, rel = pairs[0]
check("relative speed measured from the pose stream",
      abs(rel - 4.2) < 0.2, f"{rel:.2f}in/s")
check("Alvik3 driving past Alvik1 is NOT predicted to touch",
      mg > F.COLLISION_PREDICT_MARGIN_IN, f"min gap {mg:.2f}in")
vm.claim_rotation("Alvik1")
_, _, _, gap2, mg2, _ = vm.motion_pairs_in()[0]
check("once Alvik1 is cleared to turn it is judged as its swept disc",
      mg2 < mg and vm.is_turning("Alvik1"), f"{mg:.2f} -> {mg2:.2f}in")
vm.release_rotation("Alvik1")
vm._poses_in["Alvik1"] = (58.5, 26.8, 140.0)            # +47deg in 0.3s
vm._pose_in_t["Alvik1"] = now + 0.30
vm._pose_in_t["Alvik3"] = now + 0.30
vm.motion_pairs_in()
check("a measured yaw rate also counts as turning", vm.is_turning("Alvik1"))

# --------------------------------------------------------------------------
print("\n4. vision holds judge footprints, not a 0.6-cell radius")


class _Vis:
    def __init__(self, poses, turning=()):
        self.p, self.t = poses, set(turning)

    def snapshot_in(self):
        return self.p

    def is_turning(self, n):
        return n in self.t


def sup(poses, turning=()):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses, turning)
    return s


x118, y118 = W(118)
x69, y69 = W(69)
at_entry = {"Alvik1": (x118, y118, 93.0, 0.0)}
check("Alvik3 may drive 6->14 past Alvik1 standing at entry 118",
      sup(at_entry)._path_clears("Alvik1", 6, 14, R, C, node=14))
check("...and along the whole leg",
      sup(at_entry)._path_clears("Alvik1", 6, 14, R, C))
check("but NOT while Alvik1 is turning there",
      not sup(at_entry, turning={"Alvik1"})._path_clears(
          "Alvik1", 6, 14, R, C, node=14))
on_leg = {"Alvik3": (63.1, 22.1, 180.0, 0.0)}            # where it stopped
check("Alvik1 may drive 118->69 into its bay past Alvik3 on the leg",
      sup(on_leg)._path_clears("Alvik3", 118, 69, R, C))
check("a robot genuinely on my lane still blocks it",
      not sup({"Alvik3": (W(14)[0], W(14)[1] - 3.0, 180.0, 0.0)})._path_clears(
          "Alvik3", 6, 14, R, C))
check("a robot passing THROUGH the entry still blocks the corridor 13-14",
      not sup(at_entry)._path_clears("Alvik1", 13, 14, R, C))
check("unknown pose -> not clear (hold)",
      not sup({})._path_clears("Alvik1", 6, 14, R, C, node=14))

# --------------------------------------------------------------------------
print("\n5. a turn at an entry waits for robots already headed into it")


class _Res:
    def __init__(self, dest):
        self.request = types.SimpleNamespace(destination_nodes=frozenset(dest))


def with_reservations(res):
    s = object.__new__(F.FleetSupervisor)
    s.advisor = types.SimpleNamespace(
        rows=R, cols=C, robots={"Alvik1": None, "Alvik3": None},
        reservations=res)
    return s


s = with_reservations({"Alvik3": _Res({"14"})})
check("Alvik3 granted node 14 -> Alvik1's turn at entry 118 waits",
      s.entry_inbound_blocker("Alvik1", 118) == ("Alvik3", "14"))
s = with_reservations({"Alvik3": _Res({"22"})})
check("Alvik3 granted node 22, outside the sweep -> no wait",
      s.entry_inbound_blocker("Alvik1", 118) is None)
s = with_reservations({"Alvik3": _Res({"14"})})
check("a turn at a LATTICE node is not held for this (it has room)",
      s.entry_inbound_blocker("Alvik1", 13) is None)
check("a robot's own reservation never blocks it",
      with_reservations({"Alvik1": _Res({"69"})}).entry_inbound_blocker(
          "Alvik1", 118) is None)

# --------------------------------------------------------------------------
print("\n6. banking a finished drive actually records it")


class _St:
    def __init__(self, name, node, plan):
        self.name, self.current_node, self.plan = name, node, plan
        self.next_index, self.pending = 0, ("plan", 0)
        self.dispatched_sec = 1.0
        for k in ("current_edge", "request_seq", "request_plan_index",
                  "last_wait_sig", "last_selected_candidate_ms",
                  "last_done_sec"):
            setattr(self, k, None)
        self.pending_edges, self.launched = (), False


def adv_with(node, plan):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {"Alvik1": _St("Alvik1", node, plan)}
    a.reservations, a.wait_for = {}, {}
    a._release_reservation = lambda r: None
    a._hold_standing = lambda r, n, t: None
    a._log = lambda e: None
    return a


mv = da.PlanItem(command="F", kind="move", from_node=13, to_node=118)
a = adv_with(13, [mv])
a.cancel_dispatch("Alvik1")
a.record_done("Alvik1", 5.0)
check("the old path: record_done after cancel_dispatch records NOTHING",
      a.robots["Alvik1"].current_node == 13)
a = adv_with(13, [mv])
a.cancel_dispatch("Alvik1")
ok = a.bank_completed_move("Alvik1", 5.0)
check("bank_completed_move records the arrival at 118",
      ok and a.robots["Alvik1"].current_node == 118
      and a.robots["Alvik1"].next_index == 1)
check("banking needs the current item to be a move",
      not adv_with(13, [da.PlanItem(command="T", kind="turn")])
      .bank_completed_move("Alvik1", 5.0))
check("arrival is judged at 2.5in, not the 7.5in off-route band",
      F.ARRIVAL_BANK_CELLS <= 0.3 < 0.469 < F.VISION_OFFROUTE_CELLS,
      "Alvik3 stopped 4.69in short must NOT count as arrived")

# --------------------------------------------------------------------------
print("\n7. yields only from the node the plan says the robot is on")
s = object.__new__(F.FleetSupervisor)
s.vision = _Vis({"Alvik1": (x118, y118, 0.0, 0.0)})
plan_at_13 = types.SimpleNamespace(rows=R, cols=C, robots={
    "Alvik1": types.SimpleNamespace(current_node=13)})
plan_at_118 = types.SimpleNamespace(rows=R, cols=C, robots={
    "Alvik1": types.SimpleNamespace(current_node=118)})
check("plan says 13, robot stands at entry 118: may NOT yield",
      not s._on_planned_node(plan_at_13, "Alvik1"))
check("plan says 118 and it is there: may yield",
      s._on_planned_node(plan_at_118, "Alvik1"))
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8").read()
check("the supervisor hands that veto to the resolver",
      "can_yield=lambda r: self._on_planned_node(" in src)


ra = object.__new__(da.DispatchAdvisor)
ra.robots = {"Alvik1": None, "Alvik3": None}
ra.slack_by_robot = lambda now: {"Alvik1": 80.0, "Alvik3": 2.0}
ra._occupied_nodes = lambda: {}
ra.is_returning_to_depot = lambda r: False
tried = []
ra._yield_move = lambda r, b, o, s: tried.append(r) or (
    types.SimpleNamespace(robot=r) if r == "Alvik3" else None)
got = ra.resolve_deadlock(["Alvik1", "Alvik3"], 1.0,
                          can_yield=lambda r: r != "Alvik1")
check("a vetoed robot is skipped, and the next one is tried",
      tried == ["Alvik3"] and got.robot == "Alvik3", str(tried))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
