#!/usr/bin/env python3
"""The camera corridor gate is symmetric and has no tie-breaker, so two
robots converging on one node hold for each other forever.

Seen live 2026-09-29, third run of the evening: Alvik2 and Alvik5 both
wanted node 19 and alternated

    Alvik5 vision corridor: Alvik2 still on/near node 19 in my path
    Alvik2 vision corridor: Alvik5 still on/near node 19 in my path

every ~1.5s for 70 seconds, until the operator aborted. deadlock_cycles()
could not see it: a corridor hold is not a reservation, so the wait-for
graph was EMPTY for that pair while the robots were visibly deadlocked.

The fix records corridor holds in a parallel graph that the existing cycle
walker merges, so the sidestep resolver that already works on reservation
deadlocks handles camera standoffs too. These tests pin the merge, because
the failure mode it guards against is a run that hangs until someone
notices -- exactly what is NOT obvious from a passing run.

    python3 test_vision_deadlock.py
"""
import sys
import time

from dispatch_advisor import DispatchAdvisor, RobotState
from test_dispatch_advisor import crossing_schedule, CROSSING_CMDS

PASS, FAIL = [], []


def check(label, cond):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")


def adv():
    return DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)


def age(a, robot, seconds):
    """Backdate a hold's first_seen so the age threshold is reached without
    the test actually sleeping for it."""
    blocker, first, last = a.vision_wait_for[robot]
    a.vision_wait_for[robot] = (blocker, first - seconds, last)


print(__doc__.strip().splitlines()[0])
print()

# ---------------------------------------------------------------- transient
print("a fresh mutual hold is NOT a deadlock (ordinary traffic)")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
a.note_vision_block("Alvik5", "Alvik2")
check("no cycle reported immediately", a.deadlock_cycles() == [])
check("both edges are held, just not yet counted",
      set(a.vision_wait_for) == {"Alvik2", "Alvik5"})

# ---------------------------------------------------------------- standoff
print("\na mutual hold older than the threshold IS a deadlock")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
a.note_vision_block("Alvik5", "Alvik2")
age(a, "Alvik2", a.VISION_BLOCK_DEADLOCK_SEC + 1)
age(a, "Alvik5", a.VISION_BLOCK_DEADLOCK_SEC + 1)
cycles = a.deadlock_cycles()
check("exactly one cycle (A->B->A and B->A->B are one deadlock)",
      len(cycles) == 1)
check("and it names both robots",
      cycles and sorted(cycles[0]) == ["Alvik2", "Alvik5"])

# ------------------------------------------------------------- re-statement
print("\nre-stating the same hold extends it, it does not restart the clock")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
age(a, "Alvik2", a.VISION_BLOCK_DEADLOCK_SEC + 1)
first_before = a.vision_wait_for["Alvik2"][1]
a.note_vision_block("Alvik2", "Alvik5")          # the gate re-fires
check("first_seen preserved across re-statement",
      a.vision_wait_for["Alvik2"][1] == first_before)
check("last_seen advanced", a.vision_wait_for["Alvik2"][2] >= first_before)
# ...but a DIFFERENT blocker is a new standoff and must restart the clock,
# or robot-hopping would accumulate age it never actually spent stuck.
a.note_vision_block("Alvik2", "Alvik6")
check("a different blocker restarts the clock",
      a.vision_wait_for["Alvik2"][1] > first_before)

# ------------------------------------------------------------------- stale
print("\na hold nobody re-asserts decays")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
a.note_vision_block("Alvik5", "Alvik2")
for r in ("Alvik2", "Alvik5"):
    age(a, r, a.VISION_BLOCK_DEADLOCK_SEC + 1)
    blocker, first, last = a.vision_wait_for[r]
    a.vision_wait_for[r] = (blocker, first,
                            last - (a.VISION_BLOCK_STALE_SEC + 1))
check("stale edges report no cycle", a.deadlock_cycles() == [])
check("and are dropped from the graph", a.vision_wait_for == {})

print("\nclear_vision_block removes it outright")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
a.clear_vision_block("Alvik2")
check("cleared", "Alvik2" not in a.vision_wait_for)
check("clearing an unknown robot is harmless",
      a.clear_vision_block("Alvik9") is None)

# ------------------------------------------------- reservations take priority
print("\na reservation edge wins over a camera edge for the same robot")
a = adv()
a.wait_for["Alvik2"] = "Alvik6"
a.note_vision_block("Alvik2", "Alvik5")
age(a, "Alvik2", a.VISION_BLOCK_DEADLOCK_SEC + 1)
check("reservation edge is the effective one",
      a._effective_wait_for("Alvik2") == "Alvik6")

# ------------------------------------------------------------ one-sided hold
print("\na one-sided hold is not a deadlock -- the lead robot is free to go")
a = adv()
a.note_vision_block("Alvik2", "Alvik5")
age(a, "Alvik2", a.VISION_BLOCK_DEADLOCK_SEC + 1)
check("no cycle from a single edge", a.deadlock_cycles() == [])

# ------------------------------------------------------------ 3-robot cycle
print("\na three-robot camera standoff is still found")
a = adv()
for r, b in (("Alvik1", "Alvik2"), ("Alvik2", "Alvik3"), ("Alvik3", "Alvik1")):
    a.note_vision_block(r, b)
    age(a, r, a.VISION_BLOCK_DEADLOCK_SEC + 1)
cycles = a.deadlock_cycles()
check("one cycle naming all three",
      len(cycles) == 1 and sorted(cycles[0]) == ["Alvik1", "Alvik2", "Alvik3"])

# --------------------------------------------------------------- mixed graph
print("\na cycle that crosses both graphs is found")
a = adv()
a.wait_for["Alvik2"] = "Alvik5"          # reservation edge one way
a.note_vision_block("Alvik5", "Alvik2")  # camera edge back
age(a, "Alvik5", a.VISION_BLOCK_DEADLOCK_SEC + 1)
cycles = a.deadlock_cycles()
check("mixed reservation/camera cycle detected",
      len(cycles) == 1 and sorted(cycles[0]) == ["Alvik2", "Alvik5"])

# ============================================================ depot yielding
# "If a robot is returning to the depot, it should yield to the other
# robot." -- operator, 2026-09-29. A robot on its way home has already
# delivered everything it was going to deliver, so delaying it cannot delay
# any pickup or drop; the worst case is that it parks a few seconds later.
# Local harness rather than importing test_deadlock_resolution's: that
# module is a SCRIPT and calls sys.exit() at the end, so importing it ran
# its suite and killed this process before a single test here executed.
#
# NOTE the same caveat it carries: this bypasses __init__ and builds only
# the state resolve_deadlock() reads, so any new __init__ field has to be
# mirrored here or the failure looks like a production bug and is not one.


class _Sim:
    def __init__(self, end_sec):
        self.end_sec = end_sec


class _Item:
    """Minimal stand-in for PlanItem: only to_node is read."""

    def __init__(self, to_node):
        self.to_node = to_node


def da_advisor(positions, finishes, plans=None, rows=8, cols=8):
    adv = object.__new__(DispatchAdvisor)
    adv.rows, adv.cols = rows, cols
    adv.robots = {}
    for nm, node in positions.items():
        st = object.__new__(RobotState)
        st.name = nm
        st.current_node = node
        st.plan = [_Item(n) for n in (plans or {}).get(nm, [])]
        st.next_index = 0
        adv.robots[nm] = st
    adv.reservations = {}
    adv.yielding_robots = set()
    adv._simulate_fleet = lambda deciding, now: {
        n: _Sim(f) for n, f in finishes.items()}
    return adv

print()
print("is_returning_to_depot")
a = da_advisor({"Alvik1": 11}, {"Alvik1": 100.0},
               plans={"Alvik1": ["10", "9", "0", "DE1", "D1"]})
check("home-bound: nothing left but the depot lane",
      a.is_returning_to_depot("Alvik1") is True)

a = da_advisor({"Alvik1": 11}, {"Alvik1": 100.0},
               plans={"Alvik1": ["10", "73", "9", "D1"]})
check("still working: a workstation remains",
      a.is_returning_to_depot("Alvik1") is False)

# The case a naive 'next stop is a depot node' test gets wrong: crossing
# the depot lane between two workstations is NOT going home.
a = da_advisor({"Alvik1": 11}, {"Alvik1": 100.0},
               plans={"Alvik1": ["0", "DE1", "0", "88"]})
check("passing THROUGH the depot lane is not going home",
      a.is_returning_to_depot("Alvik1") is False)

a = da_advisor({"Alvik1": 11}, {"Alvik1": 100.0}, plans={"Alvik1": []})
check("empty plan is not returning",
      a.is_returning_to_depot("Alvik1") is False)

a = da_advisor({"Alvik1": 11}, {"Alvik1": 100.0},
               plans={"Alvik1": ["10", "9"]})
check("grid moves with neither depot nor workstation is not returning",
      a.is_returning_to_depot("Alvik1") is False)

print()
print("the depot-bound robot yields even with LESS slack")
# Alvik2 is home-bound but has the TIGHTER finish (less slack); under the
# old slack-only ordering Alvik5 -- which still has work to do -- would
# have been picked to step aside. That inversion is the point of the change.
a = da_advisor({"Alvik2": 19, "Alvik5": 20},
               {"Alvik2": 200.0, "Alvik5": 100.0},
               plans={"Alvik2": ["18", "0", "DE2", "D2"],
                      "Alvik5": ["21", "74"]})
plan = a.resolve_deadlock(["Alvik2", "Alvik5"], now_sec=0.0)
check("a yielder was found", plan is not None)
check("and it is the home-bound robot, not the working one",
      plan is not None and plan.robot == "Alvik2")

print()
print("both home-bound: resolves deterministically, not at random")
a = da_advisor({"Alvik2": 19, "Alvik5": 20},
               {"Alvik2": 100.0, "Alvik5": 100.0},
               plans={"Alvik2": ["18", "0", "DE2", "D2"],
                      "Alvik5": ["21", "0", "DE5", "D5"]})
picks = set()
for _ in range(5):
    p = a.resolve_deadlock(["Alvik2", "Alvik5"], now_sec=0.0)
    picks.add(p.robot if p is not None else None)
check("same deadlock resolves the same way every time", len(picks) == 1)
check("and it picked one of the pair", picks <= {"Alvik2", "Alvik5"})

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
