#!/usr/bin/env python3
"""A retried workstation exit closes only the job at that bay, nobody waits
for a drop that can no longer happen, and the decision log keeps one line
per decision.

    python3 test_drop_bookkeeping.py

ALNS24_joint, November 3, 2026, 15:46: Alvik3's exit from bay 27 failed and
was retried; each dispatch closed a job, so the second closed its bay-25
drop before it happened. The drop never registered and Alvik2 waited in
bay 42 for it until the run was aborted by hand.
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

import fleetSupervisor as F         # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def job(jid, robot, bay, mode, status="pending"):
    return types.SimpleNamespace(job_id=jid, robot=robot, workstation=bay,
                                 mode=mode, status=status)


class _Jobs:
    def __init__(self, jobs):
        self.jobs = {j.job_id: j for j in jobs}

    def _q(self, robot):
        return [j for j in self.jobs.values() if j.robot == robot]

    def next_pending(self, robot):
        return next((j for j in self._q(robot) if j.status == "pending"), None)

    def pending_after(self, robot):
        p = [j for j in self._q(robot) if j.status == "pending"]
        return p[1] if len(p) > 1 else None


def sup(jobs, drops=None):
    s = object.__new__(F.FleetSupervisor)
    s._jobs = jobs
    s._rpd_process_sec = 30.0
    s._drop_arrival = drops or {}
    s._drop_done = {}
    s._rpd_after_drop = False
    return s


R, C = 8, 8
WS = lambda bay: str(R * C + bay)          # workstation node of a bay
mv = lambda a, b: types.SimpleNamespace(kind="move", from_node=a, to_node=b)

print("1. a retried exit closes only the job at the bay it leaves")
a3 = [job("j1", "Alvik3", 27, "pick"), job("j2", "Alvik3", 25, "drop")]
check("leaving bay 27 with bay 27's pick next: closes it",
      F.job_at_bay(a3[0], 27))
a3[0].status = "done"
check("the retry, bay 25's drop next: does not close it",
      not F.job_at_bay(a3[1], 27))
check("not a workstation: never", not F.job_at_bay(a3[1], None))

print("\n2. waiting for a drop")
jobs = _Jobs([job("a", "Alvik2", 42, "pick"), job("b", "Alvik2", 25, "pick"),
              job("c", "Alvik3", 25, "drop")])
s = sup(jobs, drops={42: 0.0})
r = F.FleetSupervisor._rpd_hold(s, "Alvik2", mv(WS(42), "150"), 100.0)
check("bay-25 drop still pending: Alvik2 waits in bay 42",
      r is not None and "bay 25" in r, str(r))
jobs.jobs["c"].status = "done"
r = F.FleetSupervisor._rpd_hold(s, "Alvik2", mv(WS(42), "150"), 100.0)
check("bay-25 drop closed but never recorded: no endless wait", r is None,
      str(r))
jobs.jobs["c"].status = "orphaned"
check("dropper excluded (orphaned job): no endless wait",
      F.FleetSupervisor._rpd_hold(s, "Alvik2", mv(WS(42), "150"), 100.0)
      is None)
jobs = _Jobs([job("b", "Alvik2", 25, "pick"), job("c", "Alvik3", 25, "drop")])
s = sup(jobs, drops={})
check("leaving the pick bay before the drop: held",
      F.FleetSupervisor._rpd_hold(s, "Alvik2", mv(WS(25), "150"), 100.0)
      is not None)
s._drop_arrival[25] = 90.0
r = F.FleetSupervisor._rpd_hold(s, "Alvik2", mv(WS(25), "150"), 100.0)
check("dropped 10s ago: held for the processing time", r is not None
      and "ready in 20.0s" in r, str(r))

print("\n3. one log line per decision")
k = F.wait_key
check("countdown ticks are one decision",
      k("pick-up at bay 9: part ready in 3.8s (processing 30s from the "
        "measured drop)") == k("pick-up at bay 9: part ready in 0.2s "
                               "(processing 30s from the measured drop)"))
check("changing distances are one decision",
      k("bay exit: Alvik6 is driving past, 0.88in from the way out to 114") ==
      k("bay exit: Alvik6 is driving past, 1.02in from the way out to 114"))
check("a different node is a different decision",
      k("Alvik5 still on/near node 1 in my path") !=
      k("Alvik5 still on/near node 9 in my path"))
check("a different robot is a different decision",
      k("Alvik6 is driving past, 0.88in") != k("Alvik4 is driving past, 0.88in"))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
