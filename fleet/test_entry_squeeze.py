#!/usr/bin/env python3
"""Robots are not allowed to form a pair around a bay entry that cannot unwind.

    python3 test_entry_squeeze.py

A bay entry is 5.00in from both neighbouring lattice nodes and 4.99in from its
workstation, inside a 5.625in turning circle: a robot on the entry and a robot
on any of those three cannot turn while the other is there. The rotation gate
keeps that safe; this rule keeps it moving, by refusing the move that would
create a pair that cannot unwind. The robot already in place goes first.

Operator's rules (2026-10-15/16):
  * "the robot at the workstation entry should move first and clear the
    workstation entry node before the other robot begins to move to the
    adjacent intersection node"
  * "If Alvik1's next node is node 1 [straight through], then it should not
    be forced to stop ... it should continue while Alvik2 waits in place"

Geometry used below (8x8): entry 114 sits between lattice 9 and 10 and serves
workstation 65; entry 122 sits between 18 and 19 and serves workstation 73.
Lattice rows run 1-8, 9-16, 17-24, ... so 17 is directly above 9, 1 below it.
"""
import sys

import dispatch_advisor as da

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


DWELL = da.PlanItem(command="DWELL 2000", kind="dwell")


class _St:
    def __init__(self, name, node, plan, next_index, pending=None):
        self.name, self.current_node = name, node
        self.plan, self.next_index, self.pending = plan, next_index, pending


def recorded_at(name, node, came_from, *then, dwell_first=False):
    """Arrived at `node` from `came_from`, move recorded; plan continues."""
    plan = [mv(came_from, node)] + ([DWELL] if dwell_first else []) + \
        [mv(a, b) for a, b in then]
    return _St(name, node, plan, 1)


def arriving_at(name, frm, node, *then):
    """Granted and driving frm -> node right now."""
    plan = [mv(frm, node)] + [mv(a, b) for a, b in then]
    return _St(name, frm, plan, 0, pending=("plan", 0))


def adv(*states):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {s.name: s for s in states}
    a.reservations, a.yielding_robots = {}, set()
    return a


def ask(a, name):
    st = a.robots[name]
    return a._entry_squeeze_blocker(name, st.plan[st.next_index])


def requester(name, frm, to, *then):
    return _St(name, frm, [mv(frm, to)] + [mv(a, b) for a, b in then], 0)


check("geometry: entry 114 sits between 9 and 10", da.entry_edge_nodes(114, R, C)
      == (9, 10))
check("geometry: entry 122 sits between 18 and 19", da.entry_edge_nodes(122, R, C)
      == (18, 19))

# --------------------------------------------------------------------------
print("\n1. the run-ending standoff (2026-10-16): Alvik5 leaving bay 73, "
      "Alvik1 coming in")
a = adv(recorded_at("Alvik5", 73, 122, (73, 122), (122, 18)),
        requester("Alvik1", 26, 18, (18, 122), (122, 73)))
got = ask(a, "Alvik1")
check("Alvik1 may not take 18 while Alvik5 still has to leave bay 73 through it",
      got is not None and got[0] == "Alvik5", got[1] if got else "")
a = adv(arriving_at("Alvik5", 73, 122, (122, 18)),
        requester("Alvik1", 26, 18, (18, 122), (122, 73)))
got = ask(a, "Alvik1")
check("...nor once Alvik5 is already on its way onto entry 122",
      got is not None and got[0] == "Alvik5", got[1] if got else "")
a = adv(recorded_at("Alvik5", 73, 122, (73, 122), (122, 19)),
        requester("Alvik1", 26, 18, (18, 17)))
check("but a robot only passing 18, not entering that bay, is not held "
      "by a robot inside it", ask(a, "Alvik1") is None)

# --------------------------------------------------------------------------
print("\n2. the operator's two cases: Alvik2 on entry 114, Alvik1 coming to 9")
a = adv(recorded_at("Alvik2", 114, 10, (114, 65)),
        requester("Alvik1", 17, 9, (9, 1)))
check("next node 1, straight on: Alvik1 is NOT stopped", ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik2", 114, 65, (114, 9), (9, 17)),
        requester("Alvik1", 17, 9, (9, 10)))
got = ask(a, "Alvik1")
check("next node 10, a turn at 9 beside Alvik2 who needs 9: Alvik1 waits",
      got is not None and got[0] == "Alvik2", got[1] if got else "")
a = adv(arriving_at("Alvik2", 10, 114, (114, 65)),
        requester("Alvik1", 17, 9, (9, 1)))
check("Alvik2 still turning in at 114, Alvik1 straight through 9: allowed "
      "(Alvik2 turns once 9 is clear)", ask(a, "Alvik1") is None)
a = adv(arriving_at("Alvik2", 10, 114, (114, 65)),
        requester("Alvik1", 1, 9, (9, 17), (17, 18)))
check("coming up from 1 and carrying straight on to 17: also allowed",
      ask(a, "Alvik1") is None)
a = adv(arriving_at("Alvik2", 10, 114, (114, 65)),
        requester("Alvik1", 1, 9, (9, 10)))
got = ask(a, "Alvik1")
check("...but coming up from 1 to turn east at 9: the entry robot goes first",
      got is not None and got[0] == "Alvik2", got[1] if got else "")
a = adv(arriving_at("Alvik2", 10, 114, (114, 65)),
        requester("Alvik1", 17, 9, (9, 10)))
got = ask(a, "Alvik1")
check("both would have to turn (Alvik2 into its bay, Alvik1 east): Alvik1 waits",
      got is not None and got[0] == "Alvik2", got[1] if got else "")
a = adv(recorded_at("Alvik2", 114, 10, (114, 65)),
        requester("Alvik1", 17, 9, (9, 10)))
check("Alvik2 already facing its bay and leaving into it: Alvik1 may come "
      "(it turns once Alvik2 has gone in)", ask(a, "Alvik1") is None)

# --------------------------------------------------------------------------
print("\n3. moving ONTO the entry beside a robot that still has to turn")
a = adv(recorded_at("Alvik3", 9, 10, (9, 17), dwell_first=True),
        requester("Alvik2", 10, 114, (114, 65)))
got = ask(a, "Alvik2")
check("Alvik3 at 9 turns north after its dwell; Alvik2 would turn into the "
      "bay at 114: Alvik2 waits", got is not None and got[0] == "Alvik3",
      got[1] if got else "")
a = adv(recorded_at("Alvik3", 9, 1, (9, 17), dwell_first=True),
        requester("Alvik2", 10, 114, (114, 65)))
check("Alvik3 dwelling but leaving straight on (1->9->17): Alvik2 may come",
      ask(a, "Alvik2") is None)

# --------------------------------------------------------------------------
print("\n4. the dead-end bay")
a = adv(recorded_at("Alvik3", 65, 114, (65, 114), (114, 10)),
        requester("Alvik2", 10, 114, (114, 65)))
got = ask(a, "Alvik2")
check("Alvik3 still in bay 65: Alvik2 may not take its entry to go in",
      got is not None and got[0] == "Alvik3", got[1] if got else "")
a = adv(recorded_at("Alvik3", 65, 114, (65, 114), (114, 9)),
        requester("Alvik2", 17, 9, (9, 114), (114, 65)))
got = ask(a, "Alvik2")
check("...nor wait on the flank it will leave through", got is not None
      and got[0] == "Alvik3", got[1] if got else "")

# --------------------------------------------------------------------------
print("\n5. things the rule must leave alone")
a = adv(recorded_at("Alvik2", 114, 10, (114, 65)),
        requester("Alvik1", 27, 28, (28, 29)))
check("a move nowhere near an entry", ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik2", 9, 1, (9, 17)),
        requester("Alvik1", 18, 10, (10, 2)))
check("two lattice robots 10in apart around an EMPTY entry", ask(a, "Alvik1")
      is None)
a = adv(recorded_at("Alvik4", 114, 10, (114, 9)),
        requester("Alvik1", 17, 9, (9, 10)))
a.__dict__["stranded_robots"] = {"Alvik4"}
check("an excluded robot is never waited on to 'go first'",
      ask(a, "Alvik1") is None)
a = adv(recorded_at("Alvik2", 114, 10, (114, 65)),
        requester("Alvik1", 17, 9, (9, 1)))
st = a.robots["Alvik1"]
check("non-move items are not judged", a._entry_squeeze_blocker(
    "Alvik1", da.PlanItem(command="T", kind="turn")) is None)

# --------------------------------------------------------------------------
print("\n6. wired into the decision, and visible to the deadlock detector")
src = open(da.__file__, encoding="utf-8").read()
check("decide() consults it for every move",
      "squeeze = self._entry_squeeze_blocker(robot, item)" in src)
check("and registers the wait-for edge",
      "self.wait_for[robot] = other\n            return self._wait_decision(\n"
      "                st, f\"entry squeeze: {why}\"" in src.replace("\r\n", "\n"))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
