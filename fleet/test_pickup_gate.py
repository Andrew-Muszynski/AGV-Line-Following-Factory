#!/usr/bin/env python3
"""Workstation waits follow the robots, not the plan's clock.

    python3 test_pickup_gate.py

Operator, 2026-10-21: "the time spent waiting at a workstation must be done
when the robot is at the workstation". A robot collecting a part that
another robot dropped may not enter that bay before the drop has physically
happened, and may not leave it before drop arrival + processing time has
physically elapsed -- the planner's readiness rule, on measured times.

Geometry (8x8): workstation node = 64 + bay, entry node = 64 + 49 + bay.
Bay 10 -> workstation 74, entry 123 (between lattice 19 and 20).
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

import dispatch_advisor as da  # noqa: E402
import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


schedule = {"rpd": {"enabled": True, "process_sec": 30},
            "routes": [{"agv": "Alvik1", "visits": [{"bay": 9, "mode": "drop"},
                                                    {"bay": 10, "mode": "pick"}]},
                       {"agv": "Alvik5", "visits": [{"bay": 10, "mode": "drop"},
                                                    {"bay": 9, "mode": "pick"}]}]}
sup = object.__new__(F.FleetSupervisor)
sup._jobs = F.JobRegistry(schedule)
sup._rpd_process_sec = 30.0
sup._drop_arrival = {}

print("\n1. node labels")
check("74 is bay 10's workstation", sup._ws_bay(74) == 10 and sup._ws_bay("74") == 10)
check("123 is bay 10's entry", sup._entry_bay(123) == 10)
check("a lattice node is neither", sup._ws_bay(19) is None and sup._entry_bay(19) is None)

print("\n2. not a pick-up: never held")
check("Alvik1 on its way to drop at bay 9", sup._rpd_hold("Alvik1", mv(18, 122), 5.0) is None)
sup._drop_arrival[9] = 40.0              # Alvik1 arrives at bay 9 and drops

print("\n2b. wait in the PREVIOUS bay, not on the lane (2026-10-21 deadlock)")
h = sup._rpd_hold("Alvik1", mv(73, 122), 45.0)
check("leaving bay 9 while bay 10's part is not dropped -> stays in bay 9",
      h is not None and "waiting in bay 9" in h, h or "")
sup._drop_arrival[10] = 46.0             # Alvik5 drops at bay 10
check("once it is dropped -> leaves bay 9",
      sup._rpd_hold("Alvik1", mv(73, 122), 47.0) is None)
del sup._drop_arrival[10]                # rewind for the fallback cases below
sup._jobs.complete_next("Alvik1")          # Alvik1 has left bay 9

print("\n3. entering: wait outside until the part has been dropped")
h = sup._rpd_hold("Alvik1", mv(19, 123), 50.0)
check("onto bay 10's entry before Alvik5's drop -> held", h is not None, h or "")
check("a lattice move elsewhere is not held", sup._rpd_hold("Alvik1", mv(18, 19), 50.0) is None)
sup._drop_arrival[10] = 55.0             # Alvik5 arrives at bay 10 and drops
check("after the drop -> released", sup._rpd_hold("Alvik1", mv(19, 123), 56.0) is None)
check("into the workstation after the drop -> released",
      sup._rpd_hold("Alvik1", mv(123, 74), 70.0) is None)

print("\n4. leaving: not before drop arrival + 30 s, measured")
h = sup._rpd_hold("Alvik1", mv(74, 123), 80.0)
check("25 s after the drop -> held", h is not None and "ready in 5.0s" in h, h or "")
check("30 s after the drop -> released", sup._rpd_hold("Alvik1", mv(74, 123), 85.0) is None)

print("\n5. off when pickup-and-delivery is off")
sup._rpd_process_sec = None
check("no RPD -> never held", sup._rpd_hold("Alvik1", mv(74, 123), 60.0) is None)

print("\n6. jobs: next_pending follows the plan order")
j = F.JobRegistry(schedule)
check("Alvik5's next job is its drop at 10",
      j.next_pending("Alvik5").workstation == 10 and j.next_pending("Alvik5").mode == "drop")
j.complete_next("Alvik5")
check("then its pick-up at 9", j.next_pending("Alvik5").mode == "pick")

src = open(F.__file__, encoding="utf-8").read()
check("jobs close when a robot LEAVES a workstation, not on every DWELL",
      "LEAVING A WORKSTATION ends the visit" in src
      and "The dwell IS the service visit" not in src)

print("\n7. 2026-10-29 drop rule: processing starts when the drop's DWELL ends")
sup._rpd_process_sec = 30.0
sup._rpd_after_drop = True
sup._drop_arrival = {10: 55.0}           # Alvik5 arrived at bay 10 ...
sup._drop_done = {}
h = sup._rpd_hold("Alvik1", mv(74, 123), 90.0)
check("drop arrived but its DWELL not finished -> not ready", h is not None, h or "")
sup._drop_done[10] = 62.0                # ... turned, DWELLed 2 s, done at 62
h = sup._rpd_hold("Alvik1", mv(74, 123), 90.0)
check("28 s after the drop ended -> held", h is not None and "ready in 2.0s" in h, h or "")
check("30 s after the drop ended -> released",
      sup._rpd_hold("Alvik1", mv(74, 123), 92.0) is None)
sup._rpd_after_drop = False
check("older plans still count from arrival (55 + 30 = 85)",
      sup._rpd_hold("Alvik1", mv(74, 123), 86.0) is None)
check("the DWELL completion records the drop end",
      'self._drop_done[bay] = now' in src
      and 'bay not in self._drop_done' in src
      and 'rpd.get("processing_starts")' in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
