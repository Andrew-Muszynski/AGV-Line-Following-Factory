#!/usr/bin/env python3
"""Depot launch scheduling: what the funnel costs, and what a cascade saves.

THE PROBLEM
-----------
The solver staggers departures with launch_offset_sec so robots do not
collide leaving the depot. On nearest_2026-09-09_20-53-58 those offsets are
0, 23.2, 37.9, 60.2, 80.4 and 105.0 seconds -- 307 seconds of queueing in
total. Measured against the actual drive, that is wildly conservative: the
whole depot lane is only 46 inches end to end and no robot needs more than
16 seconds of it.

Alvik6 carries 105.0s of offset and set the planned 248.8s makespan, so
roughly 42% of the critical path is a robot sitting still in its slot.

THE CASCADE
-----------
Instead of a fixed wait per robot:

  1. all six move D# -> DE# at once (distinct nodes, no contention), and
     turn west while they are there, so every turn is paid in parallel;
  2. Alvik1 runs DE1 -> 0, then 0 -> 1, freeing the lane behind it;
  3. each robot advances one lane node as the one ahead vacates it.

A shift register, nearest-to-node-0 first. Nearest-first is the order that
works when the robots pre-stage in the lane: no robot ever has to pass a
stationary one. (Farthest-first also works, but only if robots stay in
their slots until released, because otherwise Alvik6 would have to drive
through five parked robots.)

WHERE THE NUMBERS COME FROM
---------------------------
Every constant below is measured from runs_linux/fleet_log_20260915.txt --
each leg's "driving from" and "brake fired" pair, and the gap to the next
leg, which is where the turns live. Nothing here is estimated from the
plan; the plan is what we are checking.

    python3 tools/launch_cascade.py
    python3 tools/launch_cascade.py --headway 10
"""
from __future__ import annotations

import argparse

# ---- measured, 2026-09-15 (n=6 robots unless noted) ------------------------
#: D# -> DE#, southbound out of the parking slot. 2.03-2.33s.
SLOT_TO_ENTRY = 2.22
#: Turning south -> west at the entry node, brake-to-next-leg gap.
TURN_AT_ENTRY = 1.94
#: One lane hop DE(n+1) -> DE(n): 1.74s driving + 0.38s settling. n=15.
LANE_HOP = 1.74 + 0.38
#: DE1 -> node 0, the last westward hop.
ENTRY_TO_NODE0 = 1.82
#: Turning west -> north on node 0.
TURN_AT_NODE0 = 1.90
#: node 0 -> node 1, onto the lattice. 3.59-3.93s.
NODE0_TO_NODE1 = 3.78

#: Measured arrival at node 0 in r4, by robot (LAUNCH_COMPLETED), and the
#: schedule's launch_offset_sec that each was waiting out.
R4_ARRIVE_NODE0 = {"Alvik4": 12.92, "Alvik1": 28.64, "Alvik3": 51.58,
                   "Alvik2": 69.54, "Alvik5": 93.81, "Alvik6": 120.01}
PLAN_OFFSET = {"Alvik1": 23.19, "Alvik2": 60.18, "Alvik3": 37.89,
               "Alvik4": 0.0, "Alvik5": 80.37, "Alvik6": 104.96}
#: What the planner thinks D# -> 0 costs (the solver page's "D# to 0"),
#: against what the log says it actually costs.
PLAN_SLOT_TO_NODE0 = {"Alvik4": 19.0, "Alvik1": 14.7, "Alvik3": 18.1,
                      "Alvik2": 16.0, "Alvik5": 20.4, "Alvik6": 21.9}
MEASURED_SLOT_TO_NODE0 = {"Alvik1": 6.26, "Alvik2": 8.44, "Alvik3": 10.88,
                          "Alvik4": 12.59, "Alvik5": 13.90, "Alvik6": 15.67}

#: Slot number per robot: Alvik N parks in D N, so its distance from node 0
#: is (N-1) lane hops plus the final DE1 -> 0.
ROBOTS = [f"Alvik{i}" for i in range(1, 7)]


def unobstructed(robot: str) -> float:
    """Seconds from leaving the slot to standing on node 0, lane empty."""
    hops = int(robot[-1]) - 1
    return (SLOT_TO_ENTRY + TURN_AT_ENTRY + hops * LANE_HOP + ENTRY_TO_NODE0)


def cascade(headway: float | None = None) -> dict:
    """Simulate the shift-register launch. Returns {robot: (arrive0, free0)}.

    Node 0 is the only real bottleneck: a robot holds it for the turn plus
    the drive onto node 1, and the next robot cannot enter until then. The
    lane behind it drains faster than node 0 can accept, so every robot
    after the first is gated by node 0 alone.
    """
    # Everyone reaches its own entry node and turns west in parallel.
    ready = SLOT_TO_ENTRY + TURN_AT_ENTRY
    out: dict = {}
    free_node0 = 0.0
    lane_free: dict[int, float] = {}       # slot index -> when DE# is vacated
    for i, robot in enumerate(ROBOTS):
        # walk the lane, one node at a time, behind the robot ahead
        t = ready
        for hop in range(i, 0, -1):        # DE(i+1) -> ... -> DE1
            t = max(t, lane_free.get(hop, 0.0)) + LANE_HOP
            lane_free[hop + 1] = t         # vacated the node it came from
        t = max(t, free_node0) + ENTRY_TO_NODE0
        lane_free[1] = t                   # DE1 is now clear
        arrive0 = t
        free0 = arrive0 + TURN_AT_NODE0 + NODE0_TO_NODE1
        if headway is not None:
            free0 = max(free0, arrive0 + headway)
        free_node0 = free0
        out[robot] = (arrive0, free0)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--headway", type=float, default=None,
                    help="minimum seconds between successive robots entering "
                         "node 0; the physical floor is %.2fs (turn + drive "
                         "onto node 1). Raise it to trade launch speed for "
                         "grid separation." % (TURN_AT_NODE0 + NODE0_TO_NODE1))
    args = ap.parse_args()

    print("PLANNER vs MEASURED, depot slot to node 0")
    print(f"  {'robot':<9}{'planner':>9}{'measured':>10}{'over by':>9}")
    for r in ROBOTS:
        p, m = PLAN_SLOT_TO_NODE0[r], MEASURED_SLOT_TO_NODE0[r]
        print(f"  {r:<9}{p:>8.1f}s{m:>9.2f}s{p - m:>8.2f}s")
    over = [PLAN_SLOT_TO_NODE0[r] - MEASURED_SLOT_TO_NODE0[r] for r in ROBOTS]
    print(f"  the planner is {min(over):.1f}-{max(over):.1f}s long on every "
          f"robot, which inflates every offset behind it\n")

    sim = cascade(args.headway)
    print("CASCADE vs r4, arrival at node 0")
    print(f"  {'robot':<9}{'r4 actual':>11}{'cascade':>10}{'saved':>9}")
    saved = []
    for r in ROBOTS:
        now, then = R4_ARRIVE_NODE0[r], sim[r][0]
        saved.append(now - then)
        print(f"  {r:<9}{now:>10.1f}s{then:>9.1f}s{now - then:>8.1f}s")
    last_now = max(R4_ARRIVE_NODE0.values())
    last_new = max(v[0] for v in sim.values())
    print(f"\n  last robot onto node 0: {last_now:.1f}s -> {last_new:.1f}s "
          f"({last_now - last_new:.1f}s earlier)")
    print(f"  total queueing removed: {sum(PLAN_OFFSET.values()):.0f}s of "
          f"offset -> {sum(max(0.0, sim[r][0] - unobstructed(r)) for r in ROBOTS):.0f}s "
          f"of waiting in the lane")
    period = TURN_AT_NODE0 + NODE0_TO_NODE1 if args.headway is None else args.headway
    print(f"\n  node 0 is the only bottleneck: each robot holds it for "
          f"{TURN_AT_NODE0:.2f}s of turn plus {NODE0_TO_NODE1:.2f}s onto "
          f"node 1,\n  so the floor is one robot every {period:.2f}s no matter "
          f"how the lane behind is scheduled.")


if __name__ == "__main__":
    main()
