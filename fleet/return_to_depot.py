#!/usr/bin/env python3
"""Recall robots to their own depot slots from wherever they are standing.

WHY
---
When a run dies mid-grid the fleet is left scattered. The supervisor cannot
help -- its plan is gone -- but the robots themselves are fine and still answer
camera_grid_navigate.py normally. Recovering by hand means reading each robot's
position off the overlay, working out a legal path back down the lane, and
typing a --route per robot without mixing two of them up in the depot funnel.
This does that.

It plans; camera_grid_navigate.py drives. Nothing here talks to a motor.

TOPOLOGY (read off 1,500+ real legs in runs_linux/supervisor_live.log, not
assumed)
------------------------------------------------------------------------
    D<N>  <-> DE<N>            in and out of a parking slot
    DE<N> <-> DE<N+-1>         laterally along the depot lane
    DE1   <-> 0                THE ONLY WAY IN OR OUT OF THE LANE
    0     <-> 1                the funnel mouth onto the lattice

Every robot returning to any slot therefore drives the same DE1 funnel and
shares the same lane. That is why this tool is strictly sequential and has no
option to be otherwise -- see SAFETY.

On the lattice: orthogonal neighbours only. A bay entry node is a mid-edge stub
joining the two lattice nodes it sits between (dispatch_advisor.entry_edge_nodes),
and a workstation node is a dead-end spur whose only exit is its own entry, so
a robot parked in a bay is routed out north through the entry automatically.

SAFETY
------
One robot moves at a time, and the tool waits for each to finish before
starting the next. This is not caution for its own sake:

  * camera_grid_navigate.py's own header records that depot slots measure
    6.14-6.38in apart and that "both-robots-rotating clearance at the tightest
    pair is only +0.23in, inside placement error. Sequenced (one-at-a-time)
    depot rotations are fine; simultaneous adjacent ones are not."
  * The same header records a real collision from getting this wrong: "Alvik3
    struck a parked Alvik2 on the 4-robot depot return."

There is no reservation system here. Sequencing IS the collision avoidance.

Robots are recalled furthest-slot-first (D6 before D1) so each one drives a
lane that is as empty as possible; a robot already parked sits in its D slot,
~7.9in off the DE lane the next one travels.

Routes are planned around the other robots, not just around the walls. Each
robot is routed clear of everyone who has not moved yet and of the slots of
everyone already home, and the plan is built in execution order so each route
accounts for the ones before it having parked. This is not theoretical: the
first version of this tool planned each path in isolation and, on the first
real fleet it was pointed at, routed Alvik6 straight through the node Alvik2
was standing on. Driving one at a time does not help if the route goes through
someone. A robot with no clear path is left alone and reported by name of
whoever is in its way -- two robots in the single-file lane facing opposite
ways genuinely cannot resolve themselves and need a hand.

USE
---
Plan only -- prints what it would do and changes nothing. Always start here:

    python3 fleet/return_to_depot.py

Then actually drive them, one at a time, on the machine that runs the fleet:

    python3 fleet/return_to_depot.py --execute

Subsets and re-runs are fine; a robot already in its slot is skipped:

    python3 fleet/return_to_depot.py --robots Alvik2,Alvik5,Alvik6 --execute

Requires rosbridge (the fleet's, on the Linux laptop) for the pose read, and
--execute additionally requires camera_grid_navigate.py to be runnable here,
so --execute belongs on the Linux laptop. Planning works from anywhere.

Exit codes: 0 all good, 2 at least one robot needs a human before it can be
recalled, anything else the exit code of the camera_grid_navigate.py run that
failed.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# Alvik1..Alvik6. Tag 7 exists on the table and has a measured D7/DE7, but
# apriltag_localize.py maps tags 1-6 to robot names only, so a 7th robot has no
# pose topic to recall from.
DEPOT_SLOTS = 6

#: A pose older than this is not evidence of where the robot is now.
POSE_STALE_SEC = 2.0
#: How long to listen before deciding a robot is not publishing at all.
POSE_WAIT_SEC = 3.0
#: Furthest a measured position may sit from the node it is snapped to. Grid
#: pitch is 10in, so 6in still resolves a robot stopped mid-leg to one end of
#: the lane it is already on, while refusing anything genuinely off-table.
SNAP_MAX_IN = 6.0
#: Tie-break width between a bay's entry and its own workstation, which are
#: 4.63in apart -- the closest pair of labels on the table. Over half that
#: separation, so the workstation wins only when the robot is south of the
#: entry (in the bay). See snap_to_node().
BAY_BIAS_IN = 2.5
#: Centre-to-centre distance below which two robots foul each other, used to
#: route a moving robot around the ones standing still. camera_grid_navigate.py
#: records that a 15cm-hypotenuse robot needs 5.04in; 5.4 adds a little for
#: placement error. It has to stay under 5.74in (the closest a depot slot comes
#: to a lattice node) and under 6.16in (the depot slot pitch), or a parked
#: robot would block nodes and slots the fleet legitimately uses.
CLEARANCE_IN = 5.4


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


# --------------------------------------------------------------------------
# Graph. Pure -- no ROS, no hardware, and unit-testable on its own.
# --------------------------------------------------------------------------
def depot_slot(robot: str) -> str:
    """'Alvik5' -> 'D5'. Each robot only ever parks in its own slot."""
    digits = "".join(c for c in robot if c.isdigit())
    if not digits or not (1 <= int(digits) <= DEPOT_SLOTS):
        raise ValueError(f"{robot!r} has no depot slot (expected Alvik1..{DEPOT_SLOTS})")
    return f"D{int(digits)}"


def build_graph(rows: int, cols: int) -> dict[str, list[str]]:
    """Undirected adjacency over every label a robot can legally occupy."""
    from dispatch_advisor import entry_edge_nodes

    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    g: dict[str, list[str]] = {}

    def link(a: str, b: str) -> None:
        g.setdefault(a, [])
        g.setdefault(b, [])
        if b not in g[a]:
            g[a].append(b)
        if a not in g[b]:
            g[b].append(a)

    # Lattice: orthogonal neighbours.
    for n in range(1, nodes + 1):
        row, col = (n - 1) // cols, (n - 1) % cols
        if col < cols - 1:
            link(str(n), str(n + 1))
        if row < rows - 1:
            link(str(n), str(n + cols))

    # Bays: workstation -- entry -- the two lattice nodes the entry sits between.
    for b in range(1, bays + 1):
        workstation, entry = nodes + b, nodes + bays + b
        link(str(workstation), str(entry))
        edge = entry_edge_nodes(entry, rows, cols)
        if edge is None:
            # No lattice pair resolved: leave the bay hanging off its
            # workstation rather than inventing an edge. A robot there simply
            # reports no route instead of being driven somewhere invented.
            continue
        west, east = edge
        link(str(entry), str(west))
        link(str(entry), str(east))

    # Depot lane.
    link("0", "1")
    link("DE1", "0")
    for i in range(1, DEPOT_SLOTS + 1):
        link(f"D{i}", f"DE{i}")
        if i < DEPOT_SLOTS:
            link(f"DE{i}", f"DE{i + 1}")
    return g


def shortest_path(graph: dict[str, list[str]], start: str, goal: str,
                  blocked: set[str] | None = None) -> list[str] | None:
    """Fewest-hops path inclusive of both ends, or None if unreachable.

    `blocked` labels are not entered. The start is always enterable (the robot
    is already standing there) and the goal never should be blocked -- callers
    check that first so they can say WHO is in the slot.
    """
    blocked = blocked or set()
    if start == goal:
        return [start]
    if start not in graph or goal not in graph:
        return None
    prev: dict[str, str | None] = {start: None}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        for nxt in graph[cur]:
            if nxt in prev or (nxt in blocked and nxt != goal):
                continue
            prev[nxt] = cur
            if nxt == goal:
                path, node = [], nxt
                while node is not None:
                    path.append(node)
                    node = prev[node]
                return path[::-1]
            queue.append(nxt)
    return None


def snap_to_node(x_in: float, y_in: float, graph: dict[str, list[str]],
                 rows: int, cols: int) -> tuple[str, float]:
    """Nearest occupiable label to a measured position, and the distance to it.

    Nearest-node rather than any dead-reckoning of intent: a robot stopped
    mid-leg lands within half a pitch of one end of the lane it is already on,
    and driving to that end is a legal move along that same lane.

    One deliberate exception. A bay's workstation and its entry are the closest
    pair of labels on the whole table (4.63in), and the two mis-snaps are not
    equally bad:

      * snapping to the WORKSTATION when the robot is really at the entry adds
        a leg that drives it from the entry to the entry. Harmless.
      * snapping to the ENTRY when the robot is really inside the bay drops the
        workstation->entry exit leg, so the first thing the robot does is drive
        from inside the bay straight at a lattice node -- diagonally across the
        bay mouth.

    So when the choice is between a bay's entry and that bay's own workstation,
    this breaks the tie toward the workstation. It only fires when the
    workstation is genuinely close, which happens when the robot is south of
    the entry, i.e. in the bay.
    """
    from camera_grid_navigate import node_to_world

    dist: dict[str, float] = {}
    for label in graph:
        try:
            nx, ny = node_to_world(label, rows, cols)
        except (ValueError, KeyError):
            continue
        dist[label] = math.hypot(nx - x_in, ny - y_in)
    if not dist:
        raise RuntimeError("no node positions available to snap against")

    best = min(dist, key=dist.get)
    nodes, bays = rows * cols, (rows - 1) * (cols - 1)
    try:
        n = int(best)
    except ValueError:
        return best, dist[best]
    if nodes + bays < n <= nodes + 2 * bays:
        workstation = str(nodes + (n - nodes - bays))
        if workstation in dist and dist[workstation] < dist[best] + BAY_BIAS_IN:
            return workstation, dist[workstation]
    return best, dist[best]


# --------------------------------------------------------------------------
# Pose read
# --------------------------------------------------------------------------
def read_poses(host: str, port: int, robots: list[str],
               listen_sec: float) -> dict[str, tuple[float, float, float]]:
    """Latest /<robot>_vision_pose per robot. Missing robots are absent."""
    try:
        import roslibpy
    except ImportError:                                        # pragma: no cover
        sys.exit("roslibpy is required:  python3 -m pip install roslibpy")

    latest: dict[str, tuple[float, float, float]] = {}
    seen_at: dict[str, float] = {}
    ros = roslibpy.Ros(host=host, port=port)
    ros.run()
    if not ros.is_connected:
        sys.exit(f"could not reach rosbridge at {host}:{port} -- is the fleet "
                 "stack up on the Linux laptop?")

    topics = []
    for robot in robots:
        topic = roslibpy.Topic(ros, f"/{robot}_vision_pose", "std_msgs/String")

        def on_pose(msg, r=robot):
            try:
                d = json.loads(msg["data"])
                latest[r] = (float(d["x_in"]), float(d["y_in"]),
                             float(d["yaw_deg"]))
                seen_at[r] = time.monotonic()
            except (ValueError, KeyError, TypeError):
                pass

        topic.subscribe(on_pose)
        topics.append(topic)

    log(f"listening {listen_sec:.1f}s for vision poses on {host}:{port}")
    t0 = time.monotonic()
    while time.monotonic() - t0 < listen_sec:
        time.sleep(0.1)
        if len(latest) == len(robots) and time.monotonic() - t0 > 0.6:
            break
    for topic in topics:
        try:
            topic.unsubscribe()
        except Exception:
            pass
    ros.terminate()

    now = time.monotonic()
    return {r: p for r, p in latest.items()
            if now - seen_at.get(r, 0.0) <= POSE_STALE_SEC}


# --------------------------------------------------------------------------
# Planning
# --------------------------------------------------------------------------
def clearance_zone(occupied: dict[str, str], graph: dict[str, list[str]],
                   rows: int, cols: int,
                   exclude: str | None = None) -> dict[str, str]:
    """label -> the robot whose body would be in the way of anything there.

    A moving robot must not be routed onto a node another robot is standing on
    OR onto one so close that the two overlap. camera_grid_navigate.py's own
    depot note gives the figure: a 15cm-hypotenuse robot needs 5.04in
    centre-to-centre, which is why CLEARANCE_IN is what it is.
    """
    from camera_grid_navigate import node_to_world

    zone: dict[str, str] = {}
    for robot, label in occupied.items():
        if robot == exclude:
            continue
        try:
            ox, oy = node_to_world(label, rows, cols)
        except (ValueError, KeyError):
            continue
        for cand in graph:
            try:
                nx, ny = node_to_world(cand, rows, cols)
            except (ValueError, KeyError):
                continue
            if math.hypot(nx - ox, ny - oy) < CLEARANCE_IN:
                zone.setdefault(cand, robot)
    return zone


class Recall:
    def __init__(self, robot: str, status: str, detail: str = "",
                 route: list[str] | None = None, at: str = "",
                 snap_in: float = 0.0, order: int = 0) -> None:
        self.robot, self.status, self.detail = robot, status, detail
        self.route, self.at, self.snap_in = route or [], at, snap_in
        self.order = order

    @property
    def route_str(self) -> str:
        return ",".join(self.route)


def plan_recall(poses: dict[str, tuple[float, float, float]],
                robots: list[str], rows: int, cols: int) -> list[Recall]:
    """Plan every robot's way home, treating the OTHERS as obstacles.

    Planning each robot's shortest path on its own is wrong and was wrong here
    on the first hardware run: it routed Alvik6 straight through node 27 where
    Alvik2 was standing, and within 5.00in of Alvik5 (5.04in is contact).
    Driving one robot at a time stops two of them moving at once; it does
    nothing whatever about a moving robot hitting a parked one.

    So the plan is built in execution order, carrying the occupancy forward: a
    robot is routed around everyone who has not moved yet, and around the slots
    of everyone already home or already recalled earlier in the sequence. A
    robot that has no clear path is left where it is and reported, and the
    others still go -- then the pass repeats, because recalling one robot often
    clears the way for another.
    """
    graph = build_graph(rows, cols)
    results: dict[str, Recall] = {}
    snapped: dict[str, tuple[str, float]] = {}

    for robot in robots:
        pose = poses.get(robot)
        if pose is None:
            results[robot] = Recall(robot, "NO POSE",
                                    f"no fresh /{robot}_vision_pose -- is the "
                                    "tag visible?")
            continue
        x, y, _yaw = pose
        here, dist = snap_to_node(x, y, graph, rows, cols)
        if dist > SNAP_MAX_IN:
            results[robot] = Recall(robot, "OFF GRID",
                                    f"measured ({x:.1f},{y:.1f}) is {dist:.1f}in "
                                    f"from the nearest node ({here}) -- too far "
                                    "to trust; move it onto tape by hand",
                                    at=here, snap_in=dist)
            continue
        snapped[robot] = (here, dist)

    # Where every robot we can see is standing right now. A robot we cannot
    # see is NOT in here, which is the honest thing to do but does mean an
    # unseen robot is not routed around -- said plainly in the report.
    occupied = {r: lab for r, (lab, _d) in snapped.items()}

    pending = sorted((r for r in snapped if snapped[r][0] != depot_slot(r)),
                     key=lambda r: int(depot_slot(r)[1:]), reverse=True)
    for robot in snapped:
        if robot not in pending:
            lab, dist = snapped[robot]
            results[robot] = Recall(robot, "HOME", f"already at {lab}",
                                    at=lab, snap_in=dist)

    step = 0
    while pending:
        progressed = False
        for robot in list(pending):
            here, dist = snapped[robot]
            goal = depot_slot(robot)
            zone = clearance_zone(occupied, graph, rows, cols, exclude=robot)
            zone.pop(here, None)          # it is already standing there
            if goal in zone:
                continue                  # maybe the blocker moves later
            path = shortest_path(graph, here, goal, set(zone))
            if path is None:
                continue
            results[robot] = Recall(robot, "PLAN", f"{len(path) - 1} legs",
                                    route=path, at=here, snap_in=dist,
                                    order=step)
            # It will be parked in its slot before the next one sets off.
            occupied[robot] = goal
            pending.remove(robot)
            step += 1
            progressed = True
        if not progressed:
            break

    # Whatever is still pending cannot be freed by moving anything else.
    for robot in pending:
        here, dist = snapped[robot]
        goal = depot_slot(robot)
        zone = clearance_zone(occupied, graph, rows, cols, exclude=robot)
        zone.pop(here, None)
        if goal in zone:
            detail = f"{zone[goal]} is in or beside {goal}"
        else:
            free = shortest_path(graph, here, goal)
            culprits = sorted({zone[n] for n in (free or []) if n in zone})
            detail = (f"no clear path from {here} to {goal}"
                      + (" -- " + ", ".join(culprits) + " in the way"
                         if culprits else ""))
        results[robot] = Recall(robot, "BLOCKED", detail, at=here, snap_in=dist)

    return [results[r] for r in robots]


def recall_order(plans: list[Recall]) -> list[Recall]:
    """Execution order. Set by plan_recall(), which built each robot's route
    assuming the ones before it are already parked -- so this order is part of
    the plan's correctness, not a preference."""
    return sorted([p for p in plans if p.status == "PLAN"],
                  key=lambda p: p.order)


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", default=",".join(
        f"Alvik{i}" for i in range(1, DEPOT_SLOTS + 1)),
        help="comma-separated subset to recall")
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--cols", type=int, default=8)
    ap.add_argument("--localization-mode", default="camera_assist",
                    choices=["encoder", "camera_assist", "camera"])
    ap.add_argument("--turn-law", choices=["ladder", "taper"], default="ladder",
                    help="which turn law camera_grid_navigate.py uses. "
                         "'ladder' is the pre-2026-09-25 behaviour "
                         "(--staged-approach-kp 0 --staged-kick-max 0) and is "
                         "the DEFAULT here because it is the one with known "
                         "hardware behaviour. 'taper' is the 2026-09-25 "
                         "change, which on its first hardware run overshot "
                         "every depot square-up by 11-16deg.")
    ap.add_argument("--execute", action="store_true",
                    help="actually drive them, ONE AT A TIME. Without this the "
                         "tool only prints the plan and the commands.")
    ap.add_argument("--listen-sec", type=float, default=POSE_WAIT_SEC)
    ap.add_argument("--python", default="python3",
                    help="interpreter used to run camera_grid_navigate.py. "
                         "python3 on purpose, on Windows too: the Store "
                         "Python has roslibpy and conda's does not.")
    args = ap.parse_args()

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    for robot in robots:
        depot_slot(robot)          # fail fast on a typo, before any I/O

    poses = read_poses(args.host, args.port, robots, args.listen_sec)
    plans = plan_recall(poses, robots, args.rows, args.cols)

    print()
    print(f"  {'robot':8} {'at':>6} {'snap':>7}  {'status':9} route")
    for p in plans:
        route = p.route_str if p.route else p.detail
        snap = f"{p.snap_in:.1f}in" if p.at else ""
        print(f"  {p.robot:8} {p.at:>6} {snap:>7}  {p.status:9} {route}")
    print()

    todo = recall_order(plans)
    blocked = [p for p in plans
               if p.status in ("NO POSE", "OFF GRID", "BLOCKED")]
    if blocked:
        print("  needs a human first:")
        for p in blocked:
            print(f"    {p.robot}: {p.detail}")
        print()
    if not todo:
        print("  nothing to recall.")
        return 2 if blocked else 0

    # Bare script name, run with cwd=HERE. The printed commands are then
    # copy-pasteable on the machine that drives the fleet, which is not
    # necessarily the machine this planned on.
    script = "camera_grid_navigate.py"
    # The depot arrival ends in a ~180deg square-up to heading 0, the longest
    # and least forgiving turn in the system, so which turn law is in force
    # matters more here than anywhere else on the route.
    law = ([] if args.turn_law == "taper"
           else ["--staged-approach-kp", "0", "--staged-kick-max", "0"])
    cmds = [[args.python, script, "--robot", p.robot,
             "--localization-mode", args.localization_mode]
            + law + ["--route", p.route_str] for p in todo]

    if not args.execute:
        print("  DRY RUN -- nothing has moved. Run these ONE AT A TIME and IN "
              "THIS ORDER, from\n  the directory holding "
              "camera_grid_navigate.py. Each route is planned around\n  where "
              "the earlier ones will have parked, so the order is part of the "
              "plan,\n  not a preference. Or re-run with --execute and let "
              "this tool sequence them:")
        for cmd in cmds:
            print("    " + " ".join(cmd))
        print()
        return 2 if blocked else 0

    if not os.path.exists(os.path.join(HERE, script)):
        sys.exit(f"{script} not found in {HERE} -- --execute has to run on "
                 "the machine that drives the fleet")

    log(f"recalling {len(todo)} robot(s), one at a time: "
        + ", ".join(p.robot for p in todo))
    for p, cmd in zip(todo, cmds):
        log(f"{p.robot}: {p.at} -> {depot_slot(p.robot)}  ({p.route_str})")
        rc = subprocess.call(cmd, cwd=HERE)
        if rc != 0:
            # Stop rather than send the next robot into a lane that may now
            # hold a stranded one. Sequencing is the only collision avoidance
            # this tool has, and it depends on the lane being as expected.
            log(f"{p.robot} FAILED (exit {rc}) -- stopping here. The lane may "
                "be occupied; check the table before recalling the rest.")
            return rc
        log(f"{p.robot} parked.")
    log("all recalled.")
    return 2 if blocked else 0


if __name__ == "__main__":
    sys.exit(main())
