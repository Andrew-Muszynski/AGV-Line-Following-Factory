#!/usr/bin/env python3
"""Build static/data/launch_replay.json -- the two depot-launch schemes on
the SAME route, for the side-by-side replay drawn on the solver grid.

THE EXPERIMENT
--------------
Six robots leave their depot slots and climb the west column, one terminal
node each: Alvik1->41, Alvik2->33, Alvik3->25, Alvik4->17, Alvik5->9,
Alvik6->1. Identical work per robot; the only thing that changes between the
two arms is when each robot is released.

WHERE EACH ARM COMES FROM
-------------------------
cascade : runs_linux/launch_cascade_20260915T174634.json. Measured. Every
          node arrival is a timestamp from the rig, including the time spent
          blocked behind the robot ahead -- that queueing IS the result.

offsets : the launch schedule the supervisor actually uses, read out of run
          r4's stored plan (launch_offset_sec per robot: 0, 23.19, 37.89,
          60.18, 80.37, 104.96 s), combined with the SAME rig's measured
          per-leg motion (turn_s + drive_s), with blocked_s excluded.
          Excluding it is the point rather than a convenience: fixed offsets
          are sized so that no robot ever waits for another, so under this
          scheme there is no queueing to reproduce. A robot's trajectory is
          therefore its release time plus its own measured driving.

So both arms use measured motion and a real release schedule. What the
offsets arm is NOT is a single recorded run of this route -- the rig only
ever ran the cascade (all 24 of its logs). The closest hardware measurement
of the offsets schedule is run r4, a 12-job mission that shares the launch
but then diverges to the bays; its node-0 arrivals are reported alongside
for comparison.

Node geometry comes from fleet/testbed_coordinates.json (via
testbed_coordinates.py, which needs no rclpy), the same file the navigator
drives to.
"""
from __future__ import annotations

import ast
import glob
import io
import json
import os
import re
import sqlite3
import sys
import statistics
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV = os.path.join(ROOT, "fleet", "camera_grid_navigate.py")
CASCADE = os.path.join(ROOT, "runs_linux",
                       "launch_cascade_20260915T174634.json")
DB = os.path.join(ROOT, "data", "agv_metrics.sqlite3")
OUT = os.path.join(ROOT, "static", "data", "launch_replay.json")

R4 = "b7ebe53f-cc89-4c6f-8d32-410693583a3f"
# The order the launch experiment actually released them in
# (launch_cascade_test.py staggers on int(robot[-1]) - 1).
TESTED_ORDER = ["Alvik%d" % i for i in range(1, 7)]
# The planner's own default, which run r4 executed: Alvik4 first.
R4_ORDER = ["Alvik4", "Alvik1", "Alvik3", "Alvik2", "Alvik5", "Alvik6"]
M_TO_IN = 1.0 / 0.0254
WANT = ("DEPOT_SLOT_WORLD_IN", "DEPOT_ENTRY_WORLD_IN", "NODE0_WORLD_IN",
        "GRID_NODE1_WORLD_IN", "GRID_PITCH_IN")


def nav_constants() -> dict:
    """The navigator's node tables, from the one shared file (2026-10-22;
    these used to be parsed out of camera_grid_navigate.py's literals)."""
    sys.path.insert(0, os.path.join(ROOT, "fleet"))
    import testbed_coordinates as tc
    out = {name: getattr(tc, name) for name in WANT}
    return out


K = nav_constants()
ROWS = COLS = 8


def node_xy(label) -> tuple[float, float]:
    """Node label -> world inches. Mirrors node_to_world() for the labels a
    launch path can contain: depot slots, depot entries, node 0, and lattice
    nodes. Bay and entry nodes never appear in a launch path."""
    s = str(label)
    if s in K["DEPOT_SLOT_WORLD_IN"]:
        return tuple(K["DEPOT_SLOT_WORLD_IN"][s])
    if s in K["DEPOT_ENTRY_WORLD_IN"]:
        return tuple(K["DEPOT_ENTRY_WORLD_IN"][s])
    if s == "0":
        return tuple(K["NODE0_WORLD_IN"])
    n = int(s)
    idx = n - 1
    gx, gy = float(idx % COLS), float(idx // COLS)
    return (K["GRID_NODE1_WORLD_IN"][0] + gx * K["GRID_PITCH_IN"],
            K["GRID_NODE1_WORLD_IN"][1] + gy * K["GRID_PITCH_IN"])


def cascade_arm() -> dict:
    d = json.load(open(CASCADE, encoding="utf-8"))
    robots, entry, paths = {}, {}, {}
    for name, rb in sorted(d["robots"].items()):
        path = rb["path"]
        pts = [[0.0, str(path[0])]]
        for leg in rb["legs"]:
            # Leg labels read "1->41 (through 9, 17, 25, 33)" -- a run
            # of collinear nodes driven as one leg, so the through
            # nodes lie on the straight line and interpolation is exact.
            dest = leg["leg"].split("->")[-1].split(" (")[0].strip()
            pts.append([round(float(leg["arrived_s"]), 2), dest])
            if dest == "0":
                entry[name] = round(float(leg["arrived_s"]), 2)
        robots[name] = pts
        paths[name] = [str(n) for n in path]
    return {"robots": robots, "entry_s": entry, "paths": paths,
            "summary": d.get("summary", {}),
            "kind": "measured node arrivals, including time spent blocked"}


def measured_clearances() -> tuple[dict, float]:
    """Depot-corridor clearance per robot, MEASURED from the rig logs.

    Definition: unblocked transit from the depot slot to node 1, i.e. the
    sum of turn_s + drive_s over a robot's legs up to and including its
    arrival at node 1, with blocked_s excluded. Node 0 is the shared gate, so
    the corridor is clear for the next robot once the previous one is off it
    and onto node 1.

    That definition was chosen because it reproduces the four values in
    solver.js's DEPOT_ENTRY_CLEARANCE_SEC that WERE hardware-measured
    (Alvik1-4: 14.7 / 16.0 / 18.1 / 19.0) to a mean absolute error of 1.5 s,
    far closer than any other quantity available in the logs -- the next best
    candidate is out by 5.1 s. Alvik5 and Alvik6 have no measured entry in
    that table at all; solver.js carries extrapolated placeholders and warns
    against trusting them. These are measurements instead, averaged over
    every six-robot rig run.

    Note the extrapolation's premise does not survive contact with the data:
    it assumed Alvik5/6 must be SLOWER than Alvik4 because their slots are
    farther out, and the logs show Alvik5 consistently a little faster.
    """
    clear = defaultdict(list)
    for f in sorted(glob.glob(os.path.join(ROOT, "runs_linux",
                                           "launch_cascade_*.json"))):
        d = json.load(open(f, encoding="utf-8"))
        if len(d.get("robots", {})) != 6:
            continue
        for name, rb in d["robots"].items():
            unblocked = 0.0
            for leg in rb["legs"]:
                unblocked += (float(leg.get("turn_s") or 0.0)
                              + float(leg.get("drive_s") or 0.0))
                if leg["leg"].split("->")[-1].split(" (")[0].strip() == "1":
                    clear[name].append(unblocked)
                    break
    out = {k: round(statistics.fmean(v), 2) for k, v in clear.items()}
    js = io.open(os.path.join(ROOT, "static", "js", "solver.js"),
                 encoding="utf-8").read()
    blue_red = float(re.search(r"BLUE_RED:\s*([0-9.]+)", js).group(1))
    return out, blue_red


def launch_offsets(order: list[str]) -> dict:
    """computeLaunchOffsets() from solver.js, evaluated for a given launch
    order. Each robot waits for the one before it to clear the depot
    corridor, plus one blue-to-red hop.

    The ORDER matters and is the thing that was wrong first time round. The
    planner's own default is DEPOT_LAUNCH_ORDER = [3, 0, 2, 1] -- Alvik4
    first -- which is what run r4 executed. The launch experiment on the rig
    released Alvik1 first and then straight down the line, so that is the
    order used for the replay.
    """
    clear, blue_red = measured_clearances()
    offs, t = {order[0]: 0.0}, 0.0
    for prev, nxt in zip(order, order[1:]):
        # solver.js carries one special case: Alvik1 -> Alvik3 skips the
        # blue-to-red hop. Without it this does not reproduce the planner.
        hop = 0.0 if (prev == "Alvik1" and nxt == "Alvik3") else blue_red
        t += clear[prev] + hop
        offs[nxt] = round(t, 2)
    return offs


def r4_node0() -> dict:
    con = sqlite3.connect(DB)
    rows = con.execute(
        """SELECT robot_id, elapsed_ms FROM events
            WHERE run_id = ? AND event_type = 'LAUNCH_COMPLETED'""",
        (R4,)).fetchall()
    con.close()
    return {r[0]: round(r[1] / 1000.0, 2) for r in rows}


def offsets_arm() -> dict:
    """Same route and same measured motion as the cascade arm, released on
    the fixed-offset schedule. blocked_s is dropped because fixed offsets
    exist precisely so that nothing blocks."""
    d = json.load(open(CASCADE, encoding="utf-8"))
    offs = launch_offsets(TESTED_ORDER)
    robots, entry, paths = {}, {}, {}
    for name, rb in sorted(d["robots"].items()):
        t = float(offs.get(name, 0.0))
        start = str(rb["path"][0])
        pts = [[0.0, start]]                       # parked until released
        if t > 0:
            pts.append([round(t, 2), start])
        for leg in rb["legs"]:
            t += float(leg.get("turn_s") or 0.0) + float(leg.get("drive_s") or 0.0)
            dest = leg["leg"].split("->")[-1].split(" (")[0].strip()
            pts.append([round(t, 2), dest])
            if dest == "0":
                entry[name] = round(t, 2)
        robots[name] = pts
        paths[name] = [str(n) for n in rb["path"]]
    # Same construction run against r4's launch order, so the printout can
    # check the method against an independently measured run.
    chk_offs = launch_offsets(R4_ORDER)
    chk = {}
    for name, rb in d["robots"].items():
        t = chk_offs.get(name, 0.0)
        for leg in rb["legs"]:
            t += float(leg.get("turn_s") or 0.0) + float(leg.get("drive_s") or 0.0)
            if leg["leg"].split("->")[-1].split(" (")[0].strip() == "0":
                chk[name] = round(t, 2)
                break
    return {"robots": robots, "entry_s": entry, "paths": paths,
            "r4_order_check": chk,
            "launch_order": TESTED_ORDER,
            "schedule_s": {k: round(v, 2) for k, v in offs.items()},
            "measured_r4_node0_s": r4_node0(),
            "kind": "real offset schedule + the same measured leg motion"}


def main() -> None:
    casc, offs = cascade_arm(), offsets_arm()
    doc = {
        "generated_by": "tools/make_launch_replay.py",
        "node0_in": [round(v, 2) for v in node_xy("0")],
        # the replay draws the same lattice the robots were driven on
        "grid": {"node1": [round(v, 2) for v in K["GRID_NODE1_WORLD_IN"]],
                 "pitch": K["GRID_PITCH_IN"], "rows": ROWS, "cols": COLS},
        "arms": [
            {"id": "offsets", "label": "Fixed launch offsets",
             "source": "schedule from run r4's plan + measured rig motion",
             **offs},
            {"id": "cascade", "label": "Rolling node reservation",
             "source": "launch rig, 2026-09-15T17:46 (measured)",
             **casc},
        ],
    }
    for a in doc["arms"]:
        a["last_entry_s"] = max(a["entry_s"].values()) if a["entry_s"] else None
        a["end_s"] = max(p[-1][0] for p in a["robots"].values())
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, separators=(",", ":"))
    clear, _ = measured_clearances()
    print("measured depot clearance (s), unblocked slot -> node 1:")
    print("   " + "  ".join(f"{k.replace('Alvik', 'A')}={v:.1f}"
                            for k, v in sorted(clear.items())))
    print("wrote", OUT, "(%.0f KB)" % (os.path.getsize(OUT) / 1024))
    for a in doc["arms"]:
        pts = sum(len(v) for v in a["robots"].values())
        print("  %-8s %d robots, %5d points, last onto node 0 at %.1f s"
              % (a["id"], len(a["robots"]), pts, a["last_entry_s"]))
        print("           onto node 0: " + ", ".join(
            f"{k.replace('Alvik', 'A')}={v:.1f}"
            for k, v in sorted(a["entry_s"].items())))
        if a.get("r4_order_check"):
            print("           method check, r4's own order 4-1-3-2-5-6:")
            print("             reconstructed " + ", ".join(
                f"{k.replace('Alvik', 'A')}={v:.1f}"
                for k, v in sorted(a["r4_order_check"].items())))
        if a.get("measured_r4_node0_s"):
            print("           r4 measured : " + ", ".join(
                f"{k.replace('Alvik', 'A')}={v:.1f}"
                for k, v in sorted(a["measured_r4_node0_s"].items())))


if __name__ == "__main__":
    main()
