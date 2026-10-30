#!/usr/bin/env python3
"""Leaving a bay claims the lane its entry sits on; a robot leaving a bay is
not "parked".

    python3 test_bay_exit_lane.py

2026-10-19 16:07, Alvik1/Alvik2 collision. Alvik1 had dropped off in bay 73
(entry 122, which sits ON lane 18<>19). The squeeze rule held it so Alvik2
could go first; Alvik2 was granted 18->19, and 46 ms later Alvik1 was
granted 73->122 with "immediate movement resources clear" -- the bay-stub
move claimed node 122 and edge 73<>122, Alvik2 held node 19 and lane
18<>19, and nothing overlapped. Alvik1 backed out into the lane Alvik2 was
driving along. The collision guard did not stop either robot first because
Alvik1, still inside the bay spur, counted as parked, which limits the
guard to actual contact.
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# fleetSupervisor imports ROS at module level; stand-ins are enough here.
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


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


a = object.__new__(da.DispatchAdvisor)
a.rows = a.cols = 8
lane = da.edge_key(18, 19)

print("\n1. moves that end on a bay entry claim the entry's lane")
out_of_bay = a._movement_request(mv(73, 122))
check("73 -> 122 (backing out of the bay) claims lane 18<>19",
      lane in out_of_bay.edge_keys, str(sorted(out_of_bay.edge_keys)))
check("...and still claims the bay stub itself",
      da.corridor_key(73, 122, 8, 8) in out_of_bay.edge_keys)
along = a._movement_request(mv(18, 122))
check("18 -> 122 (arriving along the lane) is unchanged: lane only",
      along.edge_keys == frozenset({lane}), str(sorted(along.edge_keys)))
into_bay = a._movement_request(mv(122, 73))
check("122 -> 73 (into the bay) does not claim the lane",
      lane not in into_bay.edge_keys, str(sorted(into_bay.edge_keys)))
check("a plain lattice move is unchanged",
      a._movement_request(mv(9, 10)).edge_keys == frozenset({da.edge_key(9, 10)}))

print("\n2. the 2026-10-19 pair now conflicts, in either order")
alvik2 = a._movement_request(mv(18, 19))
check("Alvik2 18->19 granted first -> Alvik1 73->122 refused",
      a._reservation_overlap(out_of_bay, alvik2) is not None,
      str(a._reservation_overlap(out_of_bay, alvik2)))
check("Alvik1 73->122 granted first -> Alvik2 18->19 refused",
      a._reservation_overlap(alvik2, out_of_bay) is not None)
check("a robot on a different lane is unaffected",
      a._reservation_overlap(out_of_bay, a._movement_request(mv(26, 27))) is None)

print("\n3. parked means stopped")
check("PARKED_MAX_SPEED_IN_S sits between vision jitter and cruise",
      0.2 < F.PARKED_MAX_SPEED_IN_S < 3.2, str(F.PARKED_MAX_SPEED_IN_S))
# The incident pair, judged as a MOVING pair (Alvik1 leaving the bay at ~2in/s,
# Alvik2 driving the lane at ~3.2in/s): footprints 1.0in apart and closing.
gap_now, min_gap, rel = 1.0, -0.6, 3.2
check("a moving pair 1.0in apart and closing is stopped before contact",
      F.motion_verdict(gap_now, min_gap, rel, 5.0, parked_pair=False) == "avoid")
check("the same pair judged as 'parked' (the old rule) was NOT stopped",
      F.motion_verdict(gap_now, min_gap, rel, 5.0, parked_pair=True) is None)
src = open(F.__file__, encoding="utf-8").read()
body = src[src.index("def parked(nm: str)"):src.index("for (a, b, sep, gap, min_gap,")]
check("parked() consults the robot's speed",
      "speed_in(" in body and "PARKED_MAX_SPEED_IN_S" in body)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
