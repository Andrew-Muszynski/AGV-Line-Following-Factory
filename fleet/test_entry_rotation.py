#!/usr/bin/env python3
"""A rotation at a bay entry must be gated; everywhere else must not be.

THE COLLISION THIS PREVENTS (hardware, 2026-09-30):

    ...474.14  Alvik4  leg 18->19: driving from (23.9,36.9) to (33.5,36.8)
    ...475.34  EMERGENCY STOP ['Alvik3','Alvik4'] 6.47in left, needs 6.55in
    ...477.75  Alvik3  turn at 123 toward 123->74: staged turn -84.8deg
    ...491.45  Alvik4  leg 19->27: no progress, stuck 11.98in from target

Alvik3 sat at entry 123 -- STATIONARY, so the closing-speed guard had
nothing to act on -- then rotated and swept into Alvik4, which was driving
to node 19. Entry 123's footprint is {19, 20, 74, 123}: node 19 is in it.

The reason nothing stopped it is that `decide()` dispatched every non-move
immediately with "movement gate deferred" -- a turn was never checked
against any resource, because a turn is not a move.

Geometry, measured, against the 5.625in corner-to-corner diameter:

    entry <-> adjacent lattice   5.00in   -0.62in   OVERLAPS
    entry <-> its workstation    4.99in   -0.63in   OVERLAPS
    lattice <-> lattice         10.00in   +4.38in   clear
    workstation <-> lattice   6.92-7.21in +1.30in   clear

So the entry node is the ONLY place on the table where a robot cannot turn
in place next to an occupied neighbour.

    python3 test_entry_rotation.py
"""
import sys

import dispatch_advisor as da
from dispatch_advisor import entry_rotation_footprint as foot

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8

print("footprint geometry")
check("entry 114 -> its two lattice nodes, its workstation, itself",
      foot(114, R, C) == {"9", "10", "65", "114"},
      str(sorted(foot(114, R, C), key=int)))
check("entry 123 (the one that made contact) includes node 19",
      "19" in foot(123, R, C), str(sorted(foot(123, R, C), key=int)))
check("all 49 entries resolve",
      all(foot(n, R, C) for n in range(114, 163)))
check("every footprint is exactly 4 nodes",
      {len(foot(n, R, C)) for n in range(114, 163)} == {4})

print("\nnon-entries are NOT gated -- they have room to turn")
for n in (1, 9, 10, 17, 64):
    check(f"lattice node {n} is ungated", foot(n, R, C) is None)
for n in (65, 74, 113):
    check(f"workstation {n} is ungated (+1.30in clearance)", foot(n, R, C) is None)
for n in ("D1", "DE1", "0", 163, 200, None, "junk"):
    check(f"{n!r} is ungated", foot(n, R, C) is None)


class _St:
    def __init__(self, name, node):
        self.name, self.current_node = name, node


def adv(positions, reservations=None):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {n: _St(n, node) for n, node in positions.items()}
    a.reservations = reservations or {}
    return a


print("\nthe gate itself")
a = adv({"Alvik3": 123, "Alvik4": 19})
got = a._entry_rotation_blocker(a.robots["Alvik3"])
check("Alvik3 turning at 123 is blocked by Alvik4 on node 19",
      got is not None and got[0] == "Alvik4" and got[1] == "19", str(got))

a = adv({"Alvik3": 123, "Alvik4": 74})
check("also blocked by a robot on its own workstation",
      a._entry_rotation_blocker(a.robots["Alvik3"]) is not None)

a = adv({"Alvik3": 123, "Alvik4": 27})
check("NOT blocked by a robot two nodes away",
      a._entry_rotation_blocker(a.robots["Alvik3"]) is None,
      "node 27 is outside the footprint")

a = adv({"Alvik3": 19, "Alvik4": 20})
check("a robot on a LATTICE node is never gated",
      a._entry_rotation_blocker(a.robots["Alvik3"]) is None,
      "10.00in apart, +4.38in clear")

a = adv({"Alvik3": 123})
check("alone at the entry: free to rotate",
      a._entry_rotation_blocker(a.robots["Alvik3"]) is None)

print("\nwhere a robot is HEADING counts, not just where it is")
# The 2026-09-30 case in its true form: Alvik4 was still on 18, driving to
# 19. Gating only on current position would have let the turn start and
# moved the collision a second later.
class _Res:
    def __init__(self, dests):
        self.request = type("R", (), {"destination_nodes": frozenset(dests)})()

a = adv({"Alvik3": 123, "Alvik4": 18}, {"Alvik4": _Res({"19"})})
got = a._entry_rotation_blocker(a.robots["Alvik3"])
check("blocked by a robot RESERVED onto node 19 but not yet there",
      got is not None and got[0] == "Alvik4", str(got))

a = adv({"Alvik3": 123, "Alvik4": 18}, {"Alvik4": _Res({"26"})})
check("not blocked when its reservation avoids the footprint",
      a._entry_rotation_blocker(a.robots["Alvik3"]) is None)

print("\nthe robot's own node never blocks itself")
a = adv({"Alvik3": 123, "Alvik4": 123})
got = a._entry_rotation_blocker(a.robots["Alvik3"])
check("two robots reported on the SAME entry is not self-block",
      got is None or got[1] != "123",
      "sharing a node is a localization fault, not a rotation conflict")

print()
print("a robot AT an entry blocks only the corridor it sits on")
# An entry sits 5.00in from its neighbouring lattice node -- 0.50 cells --
# against a VISION_EDGE_CORRIDOR_CELLS of 0.5. A robot parked there projects
# ~0.49 cells onto EVERY edge leaving that node, so without this rule it
# refuses all of them. The geometry guarantees that false positive.
#
# Deadlocked a run on 2026-10-06: Alvik4 waited at entry 122 to rotate until
# Alvik5 left node 19, while Alvik5 was refused edge 19->11 because Alvik4
# "projects 0.49 cells" from it. The pair alternated those two messages for
# 45 seconds until the run was aborted by hand.
from dispatch_advisor import corridor_key

_pair = da.entry_edge_nodes(122, R, C)
check("entry 122 is flanked by lattice nodes 18 and 19",
      _pair == (18, 19), str(_pair))
_own = corridor_key(_pair[0], _pair[1], R, C)
check("the corridor it sits on is STILL blocked (18-19)",
      corridor_key(18, 19, R, C) == _own, str(_own))
for frm, to, lbl in ((19, 11, "the edge that deadlocked"),
                     (19, 27, "perpendicular, the other way"),
                     (19, 20, "along the row, past the entry"),
                     (18, 17, "away from the entry")):
    check(f"{frm}-{to} is exempt: {lbl}",
          corridor_key(frm, to, R, C) != _own,
          str(corridor_key(frm, to, R, C)))

_bad = [e for e in range(114, 163)
        if da.entry_edge_nodes(e, R, C) is None
        or corridor_key(*da.entry_edge_nodes(e, R, C), R, C)
        != corridor_key(*reversed(da.entry_edge_nodes(e, R, C)), R, C)]
check("every entry has a direction-independent corridor key",
      not _bad, str(_bad[:5]))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
