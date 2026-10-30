#!/usr/bin/env python3
"""Where a deadlock yield goes is chosen from BOTH robots' next nodes.

    python3 test_yield_choice.py

ALNS24_joint shakedown, 2026-10-30, 14:21 run: Alvik6 on entry 122 (going
122 -> 19 -> 123 -> 20) and Alvik1 in bay 10 (going 74 -> 123 -> 19 -> 122
-> 73, into bay 9) deadlocked. The resolver backed Alvik6 out to 18 --
inside the turn Alvik1 then had to make at 122 -- and the pair stood for
the whole 30 s yield timeout, then nearly collided when Alvik6 drove home.

Two causes, both fixed:
  * _planned_nodes() counted plan ITEMS, and a vision plan has a turn item
    between moves: "the next 3 nodes" were 2, and 122 was never seen.
  * only one-hop spots were considered. Both of Alvik6's (18, 19) are on
    Alvik1's path or inside its turns; two hops (122 -> 19 -> 11/27) clear.
"""
import sys

import dispatch_advisor as da

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
T = da.PlanItem(command="T", kind="turn")
DWELL = da.PlanItem(command="DWELL 2000", kind="dwell")


def mv(a, b):
    return da.PlanItem(command="F", kind="move", from_node=a, to_node=b)


def vision_plan(*legs):
    """Moves with a turn item after each, as vision plans have."""
    out = []
    for a, b in legs:
        out += [mv(a, b), T]
    return out


class Sim:
    def __init__(self, finish):
        self.end_sec = finish


def advisor(states, finishes):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    a.robots = {}
    for name, (node, plan) in states.items():
        st = object.__new__(da.RobotState)
        st.name, st.current_node, st.plan, st.next_index = name, node, plan, 0
        st.pending, st.current_edge = None, None
        a.robots[name] = st
    a.reservations, a.yielding_robots, a.wait_for = {}, set(), {}
    a._reservation_counter = 0
    a._simulate_fleet = lambda deciding, now: {n: Sim(f) for n, f in finishes.items()}
    return a


print("1. _planned_nodes counts nodes, not items")
a = advisor({"Alvik1": (74, vision_plan((74, 123), (123, 19), (19, 122),
                                        (122, 73)))}, {"Alvik1": 300.0})
check("turn items between moves no longer eat the horizon",
      a._planned_nodes("Alvik1", horizon=4) == ["123", "19", "122", "73"],
      str(a._planned_nodes("Alvik1", horizon=4)))
a = advisor({"Alvik2": (18, [mv(18, 19), T, mv(19, 20)])}, {"Alvik2": 300.0})
check("a lane move counts the entry it passes over (18->19 crosses 122)",
      a._planned_nodes("Alvik2", horizon=4) == ["122", "19", "123", "20"],
      str(a._planned_nodes("Alvik2", horizon=4)))

print("\n2. the 2026-10-30 standoff: Alvik6 on 122, Alvik1 in bay 10")
# Alvik6 has more slack, so it is the one asked to yield.
fin = {"Alvik1": 340.0, "Alvik6": 300.0}
a = advisor({"Alvik6": (122, vision_plan((122, 19), (19, 123), (123, 20))),
             "Alvik1": (74, vision_plan((74, 123), (123, 19), (19, 122),
                                        (122, 73)))}, fin)
plan = a.resolve_deadlock(["Alvik1", "Alvik6"], 103.5)
foot122 = da.entry_rotation_footprint(122, R, C)
check("a resolution exists", plan is not None)
check("Alvik6 yields", plan and plan.robot == "Alvik6", str(plan))
check("NOT to 18 (inside the turn Alvik1 makes at 122)",
      plan and str(plan.to_node) != "18" and str(plan.to_node) not in foot122,
      f"to {plan.to_node if plan else None}")
check("two hops through 19 to a node off the lane (11 or 27)",
      plan and str(plan.via) == "19" and str(plan.to_node) in {"11", "27"},
      f"{plan.via if plan else None} -> {plan.to_node if plan else None}")

pair = a.begin_yield(plan, 103.5)
check("begin_yield builds it", pair is not None)
out_item, back_item = pair
check("out drives 122 -> 19 -> target as one item",
      [(str(f), str(t)) for f, t, _ in da.movement_segments(out_item)]
      == [("122", "19"), ("19", str(plan.to_node))])
check("back is the exact reverse",
      [(str(f), str(t)) for f, t, _ in da.movement_segments(back_item)]
      == [(str(plan.to_node), "19"), ("19", "122")])
res = a.reservations.get("Alvik6")
check("the reservation covers the middle node and the target",
      res is not None and {"19", str(plan.to_node)}
      <= set(res.request.destination_nodes),
      str(res.request.destination_nodes if res else None))

print("\n3. it compares other robots' next nodes too")
fin3 = {"Alvik1": 340.0, "Alvik6": 300.0, "Alvik3": 320.0}
a = advisor({"Alvik6": (122, vision_plan((122, 19), (19, 123), (123, 20))),
             "Alvik1": (74, vision_plan((74, 123), (123, 19), (19, 122),
                                        (122, 73))),
             "Alvik3": (35, vision_plan((35, 27), (27, 26)))}, fin3)
plan = a.resolve_deadlock(["Alvik1", "Alvik6"], 103.5)
check("27 is next on Alvik3's route, so the yield takes 11 instead",
      plan and str(plan.to_node) == "11", f"to {plan.to_node if plan else None}")
a = advisor({"Alvik6": (122, vision_plan((122, 19), (19, 123), (123, 20))),
             "Alvik1": (74, vision_plan((74, 123), (123, 19), (19, 122),
                                        (122, 73))),
             "Alvik3": (11, vision_plan((11, 3)))}, fin3)
plan = a.resolve_deadlock(["Alvik1", "Alvik6"], 103.5)
check("11 occupied: the yield takes 27",
      plan and str(plan.to_node) == "27", f"to {plan.to_node if plan else None}")

print("\n4. one hop still wins when it is legal")
fin4 = {"Alvik3": 227.49, "Alvik5": 248.78}
a = advisor({"Alvik3": (11, vision_plan((11, 3), (3, 2))),
             "Alvik5": (3, vision_plan((3, 11), (11, 19)))}, fin4)
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 109.1)
# Alvik3 finishes first, so it has the slack and yields; Alvik5 needs 3,
# 11, 19 next, so 10 or 12 are the one-hop spots off its path.
check("a free neighbour off the path: one hop, no via",
      plan and plan.robot == "Alvik3" and plan.via is None
      and str(plan.to_node) in {"10", "12"},
      f"{plan.to_node if plan else None} via {plan.via if plan else None}")

print("\n5. never through or onto anyone")
a = advisor({"Alvik6": (122, vision_plan((122, 19), (19, 123), (123, 20))),
             "Alvik1": (74, vision_plan((74, 123), (123, 19), (19, 122),
                                        (122, 73))),
             "Alvik2": (19, vision_plan((19, 27)))},
            {"Alvik1": 340.0, "Alvik6": 300.0, "Alvik2": 310.0})
plan = a.resolve_deadlock(["Alvik1", "Alvik6"], 103.5)
check("19 occupied: Alvik6 cannot pass through it (and 18 stays refused)",
      plan is None or (plan.robot != "Alvik6"),
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}")

print("\n6. no room to turn: leave STRAIGHT out (operator, 2026-10-30)")
# Alvik5 on entry 122 heading east, Alvik3 on flank 18 (5.00in away) going
# into bay 9. Neither can turn. Alvik5 already points at 19.
fin6 = {"Alvik3": 340.0, "Alvik5": 300.0}


def pair6():
    return advisor({"Alvik5": (122, vision_plan((122, 19), (19, 20))),
                    "Alvik3": (18, vision_plan((18, 122), (122, 73)))}, fin6)


a = pair6()
check("without heading information: declined (the old answer)",
      a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0) is None)
a = pair6()
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                          drives_straight=lambda r, n: (r, str(n)) == ("Alvik5", "19"))
check("Alvik5, facing 19, drives straight through it to a node off the lane",
      plan is not None and plan.robot == "Alvik5" and str(plan.via) == "19"
      and str(plan.to_node) in {"11", "27", "20"},
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}"
      f" via {plan.via if plan else None}")
check("the reason says it is a straight-out",
      plan is not None and "straight out" in plan.reason, plan.reason if plan else "")
pair = a.begin_yield(plan, 50.0)
check("and it can be granted", pair is not None)

# Same pair, Alvik3 passing west along the lane (19 -> 122 -> 18) instead,
# and Alvik5 facing its own workstation: it pulls straight into bay 9.
a = advisor({"Alvik5": (122, vision_plan((122, 18), (18, 10))),
             "Alvik3": (19, vision_plan((19, 122), (122, 18), (18, 17)))},
            {"Alvik3": 340.0, "Alvik5": 300.0})
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                          drives_straight=lambda r, n: (r, str(n)) == ("Alvik5", "73"))
check("facing its workstation: pulls straight into bay 9 to let Alvik3 pass",
      plan is not None and plan.robot == "Alvik5" and str(plan.to_node) == "73"
      and plan.via is None,
      f"{plan.robot if plan else None} -> {plan.to_node if plan else None}")
# ...but never into a bay the other robot is going into.
a = pair6()
plan = a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                          drives_straight=lambda r, n: (r, str(n)) == ("Alvik5", "73"))
check("not into bay 9 when Alvik3 is heading into bay 9",
      plan is None or str(plan.to_node) != "73",
      f"{plan.to_node if plan else None}")

a = pair6()
check("neither robot facing anywhere free: declined, abort stands",
      a.resolve_deadlock(["Alvik3", "Alvik5"], 50.0,
                         drives_straight=lambda r, n: False) is None)

import os  # noqa: E402
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8").read()
check("the supervisor hands the resolver its heading check",
      "drives_straight=lambda r, n, reverse=False:" in src
      and "self._drives_straight(advisor, workers, r, n," in src)
check("a straight-out yield skips its blocked departure trim",
      'straight_out = self._blocked_trim_skip(' in src
      and "if not straight_out:" in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
