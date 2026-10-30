#!/usr/bin/env python3
"""Fixes from the 2026-10-22 afternoon runs (8 missions, 5 completed).

    python3 test_run_review_2026_10_22.py

1. COLLISION at the depot (CBS run, t=16s): Alvik1 arrived at DE1, its turn
   failed, the retry could neither bank the arrival nor resync it (depot
   nodes have no grid cell), so the advisor still had it at D1 and granted
   DE1 to Alvik2. The guard then treated Alvik1, standing ON the lane, as
   "parked off the lane" and never predicted the contact. 0.83in overlap.
2. A turning robot's tag swings around the rotation centre; the guard read
   that swing as travel and stopped three pairs.
3. Alvik2 started a 170deg square-up in workstation 65 while Alvik3 drove
   1->9 up the bay's west edge (~0.3in clear), three times.
"""
import math
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


class _Vis:
    def __init__(self, poses):
        self.poses = poses

    def snapshot_in(self):
        return {n: (x, y, yaw, 0.05) for n, (x, y, yaw) in self.poses.items()}


print("\n1. depot arrivals are recognised by world position")
vis = _Vis({"Alvik1": (19.72, 2.78, 4.4), "Alvik2": (22.8, 3.0, -92.4)})
check("Alvik1 at (19.72, 2.78) has arrived at DE1", F.arrived_world_in(vis, "Alvik1", "DE1"))
check("...and not at D1", not F.arrived_world_in(vis, "Alvik1", "D1"))
check("resync places it on DE1", F.nearest_depot_label(vis, "Alvik1") == "DE1")
check("a robot between DE nodes is placed nowhere",
      F.nearest_depot_label(vis, "Alvik2") is None)
check("depot labels recognised", all(F.depot_label(l) for l in ("D1", "DE7", "0", "de2"))
      and not any(F.depot_label(l) for l in ("1", "65", "114")))
src = open(F.__file__, encoding="utf-8").read()
check("the retry path uses them", "arrived_world_in(self.vision, name," in src
      and "nearest_depot_label(" in src)

print("\n2. standing on the depot lane is not 'parked'")
cell_y = lambda y_in: (y_in - 16.75) / 10.0
check("a robot in slot D1 (y 10.58) is in the parked band",
      F.DEPOT_SLOT_MIN_Y_CELLS < cell_y(10.58) < F.DEPOT_LANE_Y_CUTOFF)
check("a robot on DE1 (y 2.78) is not",
      not (F.DEPOT_SLOT_MIN_Y_CELLS < cell_y(2.78) < F.DEPOT_LANE_Y_CUTOFF))
check("check_collisions uses the band", "DEPOT_SLOT_MIN_Y_CELLS < p[1] < DEPOT_LANE_Y_CUTOFF" in src)
# The collision itself: Alvik2 driving DE2->DE1 at 3.5in/s onto standing Alvik1.
a, b = (19.72, 2.78), (25.0, 2.9)
dx, dy = b[0] - a[0], b[1] - a[1]
stop_b = (math.hypot(19.71 - b[0], 2.94 - b[1]) + F.LEG_OVERSHOOT_IN) / 3.5
gap = F.footprint_gap_in(dx, dy, 4.4, -90.0)
pred = F.predicted_min_gap_legs_in(a, (0.0, 0.0), math.inf, b, (-3.5, 0.0), stop_b, 4.4, -90.0)
check("judged normally, the approach is stopped before contact",
      F.motion_verdict(gap, pred, 3.5, math.hypot(dx, dy)) in ("avoid", "collision"),
      f"gap now {gap:.2f}in, predicted {pred:.2f}in")

print("\n3. a turning robot does not 'travel'")
vm = object.__new__(F.VisionMonitor)
vm._lock = threading.Lock()
vm._poses, vm._prev_sep = {}, {}
now = time.time()
# Alvik2 squaring up at D2 (tag swinging), Alvik3 standing on node 2's lane.
vm._poses_in = {"Alvik2": (25.99, 10.49, 175.0), "Alvik3": (27.17, 16.17, -90.6)}
vm._pose_in_t = {"Alvik2": now - 0.3, "Alvik3": now - 0.3}
vm.motion_pairs_in()
vm._poses_in["Alvik2"] = (26.4, 10.9, 120.0)       # tag swung 0.6in in 0.3s
vm._pose_in_t = {"Alvik2": now, "Alvik3": now}
vm.claim_rotation("Alvik2")
(_, _, _, gap, mg, rel) = vm.motion_pairs_in()[0]
check("the swept disc is judged where it is, not extrapolated",
      abs(mg - gap) < 1e-6 and F.motion_verdict(gap, mg, rel, 5.0) is None,
      f"gap {gap:.2f}in, predicted {mg:.2f}in")

print("\n4. no turning in a workstation while a robot drives its cell's edges")
e = da.workstation_rotation_edges(65, 8, 8)
check("workstation 65's edges are 1-9, 2-10, 1-2, 9-10 (split at entry 114)",
      e == {frozenset(p) for p in (("1", "9"), ("2", "10"), ("1", "2"), ("9", "10"),
                                   ("9", "114"), ("114", "10"))})
check("workstation 74 (bay 10): 11-19, 12-20, 11-12, 19-20",
      frozenset(("11", "19")) in da.workstation_rotation_edges(74, 8, 8)
      and frozenset(("12", "20")) in da.workstation_rotation_edges(74, 8, 8))
check("not a workstation -> None", da.workstation_rotation_edges(114, 8, 8) is None)


class _Res:
    def __init__(self, segs):
        self.request = types.SimpleNamespace(segments=segs)


sup = object.__new__(F.FleetSupervisor)
sup.advisor = types.SimpleNamespace(rows=8, cols=8, robots={"Alvik2": 0, "Alvik3": 0},
                                    reservations={"Alvik3": _Res((("1", "9", 3.0),))})
check("Alvik3 granted 1->9: Alvik2's turn at 65 waits",
      sup.workstation_lane_blocker("Alvik2", 65) == ("Alvik3", "1->9"))
sup.advisor.reservations = {"Alvik3": _Res((("9", "17", 3.0),))}
check("once Alvik3 is driving 9->17, beyond the cell, Alvik2 may turn",
      sup.workstation_lane_blocker("Alvik2", 65) is None)
sup.advisor.reservations = {"Alvik3": _Res((("9", "9", 0.0),))}
check("a robot merely standing on corner node 9 does not block (room: +1.3in)",
      sup.workstation_lane_blocker("Alvik2", 65) is None)
check("turns at lattice nodes are unaffected",
      sup.workstation_lane_blocker("Alvik2", 9) is None)
check("await_rotation_clearance consults it",
      "self.workstation_lane_blocker(name, at_node)" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
