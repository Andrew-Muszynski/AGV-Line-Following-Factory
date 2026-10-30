#!/usr/bin/env python3
"""Tests for resolve_deadlock() -- the policy that replaces aborting the run.

The case these are built around is real: on 2026-09-09 Alvik3 and Alvik5 met
head-on between grid nodes 11 and 3, neither could yield, and 20 seconds
later the supervisor stopped all six robots including the four that were
running perfectly. Planned finishes from that run's own plan
(runs/nearest_2026-09-09_20-53-58.json):

    Alvik1 165.03   Alvik4 186.48   Alvik2 197.04
    Alvik5 216.58   Alvik3 227.49   Alvik6 248.78  <- critical path

Alvik5 finishes earlier than Alvik3, so Alvik5 carries more slack and is the
one that should step aside. Delaying it cannot move fleet makespan, which is
max(finish) and belongs to Alvik6.

No ROS, no robots: the advisor's decision half is pure, so the whole policy
is checked here in under a second.
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dispatch_advisor as da  # noqa: E402

FAILS = []


def check(label, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}"
          + (f"   {detail}" if detail else ""))
    if not cond:
        FAILS.append(label)


class Sim:
    def __init__(self, end_sec):
        self.end_sec = end_sec


class Item:
    """Minimal stand-in for PlanItem: only to_node is read."""

    def __init__(self, to_node):
        self.to_node = to_node


def advisor(positions, finishes, plans=None, reservations=None,
            rows=8, cols=8):
    """A DispatchAdvisor with only the state resolve_deadlock() reads."""
    adv = object.__new__(da.DispatchAdvisor)
    adv.rows, adv.cols = rows, cols
    adv.robots = {}
    for name, node in positions.items():
        st = object.__new__(da.RobotState)
        st.name = name
        st.current_node = node
        st.plan = [Item(n) for n in (plans or {}).get(name, [])]
        st.next_index = 0
        adv.robots[name] = st
    adv.reservations = reservations or {}
    # This helper deliberately bypasses __init__ to build only the state
    # resolve_deadlock() reads, which means every field added to __init__ has
    # to be mirrored here or the test dies with an AttributeError that looks
    # like a production bug and is not one. yielding_robots arrived with the
    # yield-livelock fix; begin_yield()/end_yield() maintain it.
    adv.yielding_robots = set()
    adv._simulate_fleet = lambda deciding, now: {
        n: Sim(f) for n, f in finishes.items()}
    return adv


# Finishes from the real plan.
REAL = {"Alvik1": 165.03, "Alvik2": 197.04, "Alvik3": 227.49,
        "Alvik4": 186.48, "Alvik5": 216.58, "Alvik6": 248.78}

print("1. the deadlock that actually happened (Alvik3 at 11, Alvik5 at 3)")
adv = advisor({"Alvik3": 11, "Alvik5": 3}, REAL,
              plans={"Alvik3": [3, 2, 1], "Alvik5": [11, 19, 27]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], now_sec=109.1)
check("a resolution exists (run no longer has to abort)", plan is not None)
check("the robot with MORE slack yields",
      plan and plan.robot == "Alvik5",
      f"Alvik5 slack {248.78 - 216.58:.1f}s vs Alvik3 {248.78 - 227.49:.1f}s")
check("it is a sidestep, not a backtrack", plan and plan.kind == "sidestep")
check("it does not step onto the blocked robot's path",
      plan and str(plan.to_node) not in {"3", "2", "1"},
      f"chose {plan.to_node if plan else None}")
check("it does not step onto an occupied node",
      plan and str(plan.to_node) != "11")
check("the reason names both robots",
      plan and "Alvik5" in plan.reason and "Alvik3" in plan.reason)

print("2. slack comes from LIVE simulation, not the original plan")
live = {**REAL, "Alvik5": 260.0}      # Alvik5 has fallen behind mid-run
adv = advisor({"Alvik3": 11, "Alvik5": 3}, live,
              plans={"Alvik3": [3, 2], "Alvik5": [11, 19]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], now_sec=109.1)
check("the yielder flips when live timing flips the slack",
      plan and plan.robot == "Alvik3",
      "Alvik5 is now the later finisher, so Alvik3 yields instead")

print("3. shortest detour is preferred among legal neighbours")
# Alvik5 at 3; free neighbours are 2 and 4. Its own next waypoint is 11,
# reachable from either, but 4 is no closer than 2 -- the tiebreak is the
# lower node id, so this asserts determinism rather than a direction.
adv = advisor({"Alvik3": 11, "Alvik5": 3}, REAL,
              plans={"Alvik3": [3], "Alvik5": [11]})
first = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
second = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
check("the same deadlock always resolves the same way",
      first.to_node == second.to_node, f"chose {first.to_node} twice")
adv = advisor({"Alvik3": 11, "Alvik5": 3, "Alvik2": 2}, REAL,
              plans={"Alvik3": [3], "Alvik5": [11]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
check("a neighbour occupied by a third robot is skipped",
      plan and str(plan.to_node) == "4", f"chose {plan.to_node if plan else None}")

print("4. reserved-but-not-yet-occupied nodes are respected")
req = da.MovementRequest(
    segments=(), destination_nodes=frozenset({"4"}),
    directed_edges=frozenset(), edge_keys=frozenset(),
    shared_depot_entrance=False, depot_slots=frozenset())
res = da.Reservation(robot="Alvik1", plan_index=0, request=req,
                     granted_seq=1, granted_sec=0.0)
adv = advisor({"Alvik3": 11, "Alvik5": 3, "Alvik2": 2}, REAL,
              plans={"Alvik3": [3], "Alvik5": [11]},
              reservations={"Alvik1": res})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
# Alvik5 is boxed in: 2 is occupied, 4 is reserved by a robot about to enter
# it, and 11 is the other deadlocker. The policy must not squeeze onto any of
# them -- and must not give up either, since the next candidate can still
# move. Falling through to the lower-slack robot is the correct answer.
check("the boxed-in robot is not squeezed onto a reserved node",
      plan is None or plan.robot != "Alvik5",
      f"yielded {plan.robot if plan else None}")
check("it falls through to the next candidate rather than aborting",
      plan is not None and plan.robot == "Alvik3",
      f"chose {plan.robot if plan else None} -> {plan.to_node if plan else None}")
check("and that robot's target is genuinely free",
      plan and str(plan.to_node) in {"10", "12", "19"},
      f"{plan.to_node if plan else None}")

# Now box in BOTH: only then should it decline and leave the abort in place.
adv = advisor({"Alvik3": 1, "Alvik5": 2, "Alvik1": 9, "Alvik2": 3,
               "Alvik4": 10}, REAL,
              plans={"Alvik3": ["2"], "Alvik5": ["1"]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
check("with every neighbour taken, it declines and the abort stands",
      plan is None, f"got {plan}")

print("5. bay entry -> backtrack, because there is no reverse gear")
entry = 8 * 8 + 7 * 7 + 1        # first entry label on an 8x8 grid
flank = da.entry_edge_nodes(entry, 8, 8)
check("entry_edge_nodes gives the two flanking grid nodes",
      flank is not None and len(flank) == 2, f"{entry} -> {flank}")
# Alvik3 needs the ENTRY itself. Backing Alvik5 out to the other flank would
# park it 5.00in from that entry, inside the 5.625in sweep of the turn
# Alvik3 makes there -- the 2026-10-16 Alvik2/Alvik3 standoff at 137/36. So
# the entry robot must NOT take the flank; the robot that can step clear
# yields instead.
adv = advisor({"Alvik5": entry, "Alvik3": flank[0]}, REAL,
              plans={"Alvik3": [str(entry)], "Alvik5": [flank[1]]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
foot = da.entry_rotation_footprint(entry, 8, 8)
check("the entry robot is not parked inside the turn the other must make",
      not (plan and plan.robot == "Alvik5" and str(plan.to_node) in foot),
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}")
# 2026-10-30: and Alvik3 cannot step clear EITHER. It stands on the flank,
# 5.00in from the occupied entry, so its own departure turn sweeps into
# Alvik5 -- the rotation gate holds it and the yield fails on its 3 s
# deadline. Until 2026-10-30 this test accepted that doomed sidestep. Neither
# robot can turn: declining and leaving it to the abort is the truth.
check("neither can turn (each 5.00in from the other): declined",
      plan is None,
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}")

# One node further back, Alvik5 CAN turn on the entry: both flanks are
# inside Alvik3's path or turn, so it passes through the free flank to a
# lattice node off the lane and waits there (two hops).
back = [n for n in adv._grid_neighbours(flank[0])
        if n not in flank and da.entry_edge_nodes(entry, 8, 8)][0]
adv = advisor({"Alvik5": entry, "Alvik3": back}, REAL,
              plans={"Alvik3": [str(flank[0]), str(entry)],
                     "Alvik5": [flank[1]]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
check("Alvik3 one node back: the entry robot escapes in two hops",
      plan is not None and plan.robot == "Alvik5"
      and str(plan.via) == str(flank[1]) and str(plan.to_node) not in foot,
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}"
      f" via {getattr(plan, 'via', None)}")

# Alvik3 needs only the FLANK -- it is not entering the bay. Backing out to
# the far flank puts Alvik5 10in from where Alvik3 will be: the backtrack is
# still the right answer here, and still the only shape an entry robot has.
adv = advisor({"Alvik5": entry, "Alvik3": 17}, REAL,
              plans={"Alvik3": [str(flank[0])], "Alvik5": [flank[1]]})
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
check("the robot on the entry backs out", plan and plan.kind == "backtrack")
check("it backs to the free flank away from the blocker's target",
      plan and str(plan.to_node) == str(flank[1]),
      f"{flank[0]} is Alvik3's target, so it must take {flank[1]}")

print("6. cycles of 3, 4 and 5 -- exactly one robot yields")
for size in (3, 4, 5):
    names = [f"Alvik{i}" for i in range(1, size + 1)]
    pos = {n: 10 + 2 * i for i, n in enumerate(names)}
    adv = advisor(pos, REAL, plans={n: [] for n in names})
    plan = adv.resolve_deadlock(names, 109.1)
    slack = {n: 248.78 - REAL[n] for n in names}
    most = max(slack, key=lambda n: (slack[n], n))
    check(f"{size}-robot cycle resolves", plan is not None)
    check(f"{size}-robot cycle yields the most-slack robot",
          plan and plan.robot == most,
          f"{plan.robot if plan else None} (slack {slack.get(most, 0):.1f}s)")

print("7. no legal move -> None, so the existing abort still protects us")
adv = advisor({"Alvik3": 1, "Alvik5": 2, "Alvik1": 9}, REAL,
              plans={"Alvik3": ["2"], "Alvik5": ["1"]})
# node 1's neighbours are 2 (Alvik5) and 9 (Alvik1); node 2's are 1, 3, 10.
plan = adv.resolve_deadlock(["Alvik3"], 109.1)
check("a cycle of one is not a deadlock", plan is None)

print("8. geometry helpers agree with the module's own label mapping")
adv = advisor({}, {})
check("corner node 1 has two neighbours",
      sorted(adv._grid_neighbours(1)) == [2, 9], adv._grid_neighbours(1))
check("interior node 11 has four",
      sorted(adv._grid_neighbours(11)) == [3, 10, 12, 19])
check("row wrap is not a neighbour (8 and 9 are different rows)",
      9 not in adv._grid_neighbours(8), adv._grid_neighbours(8))
check("entry/depot labels have no lattice neighbours",
      adv._grid_neighbours("DE1") == [] and adv._grid_neighbours(entry) == [])
# node 1 is (row 0, col 0); node 11 is (row 1, col 2) -> 1 + 2 = 3 steps.
check("lattice distance is Manhattan",
      adv._lattice_distance(1, 11) == 3, adv._lattice_distance(1, 11))
check("unknown goal costs nothing rather than crashing",
      adv._lattice_distance(1, None) == 0
      and adv._lattice_distance("DE1", 11) == 0)



# --------------------------------------------------------------------------
# The execution half: begin_yield / finish_yield_leg / end_yield.
# --------------------------------------------------------------------------
print("9. begin_yield reserves the leg and returns both halves")
adv = advisor({"Alvik3": 11, "Alvik5": 3}, REAL,
              plans={"Alvik3": [3, 2, 1], "Alvik5": [11, 19]})
adv._reservation_counter = 0
adv.wait_for = {}
plan = adv.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
pair = adv.begin_yield(plan, 109.1)
check("begin_yield returns an out and a back item", pair is not None
      and len(pair) == 2)
out_item, back_item = pair
check("out leg goes where the decision said",
      str(out_item.from_node) == "3" and str(out_item.to_node) == "4")
check("back leg is the exact reverse -- yield and RETURN",
      str(back_item.from_node) == str(out_item.to_node)
      and str(back_item.to_node) == str(out_item.from_node))
check("both are move items the worker can drive",
      out_item.kind == "move" and back_item.kind == "move")
check("the leg is reserved so nobody else is granted it",
      "Alvik5" in adv.reservations
      and "4" in adv.reservations["Alvik5"].request.destination_nodes)

print("10. the plan is never edited -- that is why no replan is needed")
st5 = adv.robots["Alvik5"]
check("next_index is untouched by the maneuver", st5.next_index == 0)
check("the robot's plan is the same object it started with",
      len(st5.plan) == 2)

print("11. the target is re-checked at grant time, not trusted")
adv2 = advisor({"Alvik3": 11, "Alvik5": 3}, REAL,
               plans={"Alvik3": [3], "Alvik5": [11]})
adv2._reservation_counter = 0
adv2.wait_for = {}
plan2 = adv2.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
# Somebody parks on the chosen node between the decision and the grant.
adv2.robots["Alvik3"].current_node = plan2.to_node
check("a target taken after the decision is refused",
      adv2.begin_yield(plan2, 109.1) is None,
      f"target {plan2.to_node} was claimed in the gap")

print("12. the reservation follows the robot leg by leg")
adv.finish_yield_leg("Alvik5", 4, 110.0)
res = adv.reservations.get("Alvik5")
check("after the out leg it holds the node it is standing on",
      res is not None and "4" in res.request.destination_nodes)
check("and no longer holds the node it vacated",
      res is not None and "3" not in res.request.destination_nodes,
      "this is what unblocks the other robot")
print("12b. the way home is RESERVED before it is driven (2026-10-22)")
# Another robot standing on home: the return must be refused, nothing changed.
adv.robots["Alvik3"].current_node, saved = 3, adv.robots["Alvik3"].current_node
adv._grant_reservation("Alvik3", -1, adv._standing_request(3), 111.0)
check("home occupied -> begin_yield_return refuses",
      adv.begin_yield_return("Alvik5", back_item, 111.0) is False
      and "4" in adv.reservations["Alvik5"].request.destination_nodes)
adv._release_reservation("Alvik3")
adv.robots["Alvik3"].current_node = saved
adv._grant_reservation("Alvik3", -1, adv._standing_request(saved), 111.5)
check("home free -> the return leg is reserved",
      adv.begin_yield_return("Alvik5", back_item, 112.0) is True
      and "3" in adv.reservations["Alvik5"].request.destination_nodes)
blocked = adv._reservation_blocker(
    "Alvik3", adv._movement_request(da.PlanItem(
        command="F", kind="move", from_node=2, to_node=3)))
check("so nobody else can be granted a move onto home while it drives back",
      blocked is not None, str(blocked))
adv.finish_yield_leg("Alvik5", 3, 114.0)
adv.end_yield("Alvik5", 114.0)
res = adv.reservations.get("Alvik5")
check("end_yield leaves a standing reservation on the original node",
      res is not None and "3" in res.request.destination_nodes)
check("and clears the in-flight edge",
      adv.robots["Alvik5"].current_edge is None)
check("robot is back where it started, plan intact",
      str(adv.robots["Alvik5"].current_node) == "3"
      and adv.robots["Alvik5"].next_index == 0)

print()
print("FAILED:", FAILS if FAILS else "none")
sys.exit(1 if FAILS else 0)
