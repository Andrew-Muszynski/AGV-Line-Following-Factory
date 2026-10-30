#!/usr/bin/env python3
"""Two robots may not face each other across the lattice node between two
bay entries.

    python3 test_lane_head_on.py

ALNS24_joint shakedown, 2026-10-30 14:59: Alvik6 left bay 9 onto entry 122
heading 122->19->123; Alvik1 then left bay 10 onto entry 123 heading
123->19. With both entries taken nobody can use 19 (whoever enters cannot
turn there, or cannot go on), and the run aborted on a deadlock. The second
robot must wait where it is until the first has gone through.

Geometry (8x8): entry 122 sits between lattice 18 and 19 and serves
workstation 73 (bay 9); entry 123 sits between 19 and 20 and serves
workstation 74 (bay 10). 11 is directly below 19, 27 above it.
"""
import sys

import dispatch_advisor as da

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


def recorded_at(name, node, came_from, *then):
    return _St(name, node, [mv(came_from, node)] + [mv(a, b) for a, b in then], 1)


def arriving_at(name, frm, node, *then):
    return _St(name, frm, [mv(frm, node)] + [mv(a, b) for a, b in then], 0,
               pending=("plan", 0))


def in_bay(name, ws, entry, *then):
    """Dwelt in workstation `ws`, about to ask for ws -> entry."""
    plan = [mv(entry, ws), DWELL, mv(ws, entry)] + [mv(a, b) for a, b in then]
    return _St(name, ws, plan, 2)


def adv(*states):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {s.name: s for s in states}
    a.reservations, a.yielding_robots = {}, set()
    return a


def ask(a, name):
    st = a.robots[name]
    return a._lane_head_on_blocker(name, st.plan[st.next_index])


print("\n1. the 2026-10-30 abort, in the order it happened")
a = adv(recorded_at("Alvik6", 122, 73, (122, 19), (19, 123), (123, 20)),
        in_bay("Alvik1", 74, 123, (123, 19), (19, 11)))
r = ask(a, "Alvik1")
check("Alvik1 may not leave bay 10 while Alvik6 on 122 heads into 19",
      r is not None and r[0] == "Alvik6", str(r))
check("Alvik6 itself is not held by this rule (Alvik1 is in its bay)",
      a._lane_head_on_blocker("Alvik6", a.robots["Alvik6"].plan[1]) is None)

print("\n2. the other order: whoever is on the lane first goes first")
a = adv(arriving_at("Alvik1", 74, 123, (123, 19), (19, 11)),
        in_bay("Alvik6", 73, 122, (122, 19), (19, 123)))
r = ask(a, "Alvik6")
check("Alvik6 waits in bay 9 while Alvik1 is granted 123 toward 19",
      r is not None and r[0] == "Alvik1", str(r))

print("\n3. from the lattice side too")
a = adv(recorded_at("Alvik6", 122, 73, (122, 19), (19, 27)),
        _St("Alvik1", 20, [mv(20, 123), mv(123, 19), mv(19, 11)], 0))
r = ask(a, "Alvik1")
check("Alvik1 waits at 20 rather than driving onto 123 toward 19",
      r is not None and r[0] == "Alvik6", str(r))

print("\n4. not held when nobody meets at the shared node")
a = adv(recorded_at("Alvik6", 122, 73, (122, 18), (18, 10)),
        in_bay("Alvik1", 74, 123, (123, 19), (19, 11)))
check("Alvik6 on 122 heading AWAY (to 18): Alvik1 leaves freely",
      ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik6", 122, 73, (122, 19), (19, 27)),
        in_bay("Alvik1", 74, 123, (123, 20), (20, 12)))
check("Alvik1 leaving bay 10 eastward (to 20): not held",
      ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik6", 122, 18, (122, 73)),
        in_bay("Alvik1", 74, 123, (123, 19), (19, 11)))
check("Alvik6 on 122 going INTO bay 9: Alvik1 leaves freely",
      ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik6", 18, 10, (18, 122), (122, 19)),
        in_bay("Alvik1", 74, 123, (123, 19), (19, 11)))
check("Alvik6 still at lattice 18 (not on the entry yet): Alvik1 goes first",
      ask(a, "Alvik1") is None)
r = adv(arriving_at("Alvik1", 74, 123, (123, 19), (19, 11)),
        _St("Alvik6", 18, [mv(18, 122), mv(122, 19), mv(19, 27)], 0))
r = ask(r, "Alvik6")
check("...and then Alvik6 waits at 18 until Alvik1 is through",
      r is not None and r[0] == "Alvik1", str(r))

print("\n5. only bay-entry moves are judged")
a = adv(recorded_at("Alvik6", 122, 73, (122, 19), (19, 123)),
        _St("Alvik1", 28, [mv(28, 20), mv(20, 19)], 0))
check("a lattice move is left to the other rules", ask(a, "Alvik1") is None)
check("non-move items are not judged",
      a._lane_head_on_blocker("Alvik1", DWELL) is None)

src = open(da.__file__, encoding="utf-8").read()
check("the rule is wired into the decision before the squeeze rule",
      src.index("self._lane_head_on_blocker(robot, item)")
      < src.index("squeeze = self._entry_squeeze_blocker(robot, item)"))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
