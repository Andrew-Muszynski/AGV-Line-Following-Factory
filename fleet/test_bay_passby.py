#!/usr/bin/env python3
"""A robot standing in a workstation blocks nobody; leaving it needs the lane clear.

    python3 test_bay_passby.py

Operator, 2026-10-21: "While a robot is sitting in the workstation node like
Alvik5 sitting at node 74, robots can freely move to/through nodes 11, 12, 19
and 20 without any hindrance or rotation hazard." The entry-squeeze rule held
Alvik2 at node 18 for Alvik5's entire 30 s of processing, because a lane
through-route passes over the bay entry as a waypoint (19 -> 123 -> 20) and
the rule tested "touches the entry" instead of "goes into the workstation".

And the other half of the same rule: "The robot that was in the workstation
should then wait until both node 18 and 19 are clear."

Geometry (8x8): bay 10 = workstation 74, entry 123 between lattice 19 and 20.
"""
import sys

import dispatch_advisor as da

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


DWELL = da.PlanItem(command="DWELL 30000", kind="dwell")


class _St:
    def __init__(self, name, node, plan, next_index, pending=None):
        self.name, self.current_node = name, node
        self.plan, self.next_index, self.pending = plan, next_index, pending


def processing_at_74():
    """Alvik5 arrived at 74 from 123 and is dwelling; then it leaves via 19."""
    return _St("Alvik5", 74, [mv(123, 74), DWELL, mv(74, 123), mv(123, 19)], 1)


def at(name, node, came_from, *then):
    return _St(name, node, [mv(came_from, node)] + [mv(a, b) for a, b in then], 1)


def requester(name, frm, to, *then):
    return _St(name, frm, [mv(frm, to)] + [mv(a, b) for a, b in then], 0)


def adv(*states):
    a = object.__new__(da.DispatchAdvisor)
    a.rows = a.cols = 8
    a.robots = {s.name: s for s in states}
    a.reservations, a.yielding_robots = {}, set()
    return a


print("\n1. driving PAST a bay whose robot is processing: never held")
a = adv(processing_at_74(), requester("Alvik2", 18, 19, (19, 20), (20, 21)))
r = a._entry_squeeze_blocker("Alvik2", a.robots["Alvik2"].plan[0])
check("Alvik2 18->19 on its way 19->20->21 past B10 (the 2026-10-21 hold)",
      r is None, str(r))
a = adv(processing_at_74(), requester("Alvik2", 19, 20, (20, 21), (21, 22)))
r = a._entry_squeeze_blocker("Alvik2", a.robots["Alvik2"].plan[0])
check("...and onward 19->20, over B10's entry (the planners' real path)",
      r is None, str(r))
a = adv(processing_at_74(), requester("Alvik2", 11, 12, (12, 13)))
check("nodes 11/12 below the bay: never held",
      a._entry_squeeze_blocker("Alvik2", a.robots["Alvik2"].plan[0]) is None)

print("\n2. going INTO that bay while its robot still has to leave: held")
a = adv(processing_at_74(), requester("Alvik2", 18, 19, (19, 123), (123, 74)))
r = a._entry_squeeze_blocker("Alvik2", a.robots["Alvik2"].plan[0])
check("Alvik2 heading 19->123->74 waits for Alvik5 to come out", r is not None, str(r))

print("\n3. leaving the workstation: both lattice nodes beside the entry clear")
leaving = requester("Alvik5", 74, 123, (123, 19))
a = adv(leaving, at("Alvik2", 19, 18, (19, 20)))
r = a._bay_exit_blocker("Alvik5", leaving.plan[0])
check("a robot standing at 19 -> Alvik5 stays in the bay", r is not None, str(r))
a = adv(requester("Alvik5", 74, 123, (123, 19)), at("Alvik2", 20, 19, (20, 21)))
r = a._bay_exit_blocker("Alvik5", a.robots["Alvik5"].plan[0])
check("a robot standing at 20 -> Alvik5 stays in the bay", r is not None, str(r))
a = adv(requester("Alvik5", 74, 123, (123, 19)), at("Alvik2", 21, 20, (21, 22)))
check("the robot has moved on to 21 -> Alvik5 may leave",
      a._bay_exit_blocker("Alvik5", a.robots["Alvik5"].plan[0]) is None)
arriving = _St("Alvik2", 18, [mv(18, 19), mv(19, 20)], 0, pending=("plan", 0))
a = adv(requester("Alvik5", 74, 123, (123, 19)), arriving)
r = a._bay_exit_blocker("Alvik5", a.robots["Alvik5"].plan[0])
check("a robot already granted the move onto 19 counts as there", r is not None, str(r))
check("an ordinary lattice move is not an exit",
      adv(at("Alvik2", 19, 18))._bay_exit_blocker("X", mv(18, 19)) is None)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
