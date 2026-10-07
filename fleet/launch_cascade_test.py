#!/usr/bin/env python3
"""Depot launch cascade: six robots up the west column, no supervisor.

WHAT THIS IS
------------
A standalone rig for one question: how fast can six robots leave the depot
and get onto the grid? The full stack answers it in 120.0s for the last
robot (measured r4, 2026-09-15), but most of that is the solver's launch
offsets -- 307 seconds of scheduled waiting for 64 seconds of driving.
This removes the solver, the advisor, the LLM and the job model, and leaves
only the funnel.

THE ROUTE
---------
Every robot drives its slot, down to its entry node, west along the depot
lane to node 0, then north up the west column. They stop one node apart:

    Alvik1  D1 -> DE1 ->                             0 -> 1 -> 9 -> 17 -> 25 -> 33 -> 41
    Alvik2  D2 -> DE2 -> DE1 ->                      0 -> 1 -> 9 -> 17 -> 25 -> 33
    Alvik3  D3 -> DE3 -> DE2 -> DE1 ->               0 -> 1 -> 9 -> 17 -> 25
    Alvik4  D4 -> DE4 -> DE3 -> DE2 -> DE1 ->        0 -> 1 -> 9 -> 17
    Alvik5  D5 -> DE5 -> DE4 -> DE3 -> DE2 -> DE1 -> 0 -> 1 -> 9
    Alvik6  D6 -> DE6 -> DE5 -> DE4 -> DE3 -> DE2 -> DE1 -> 0 -> 1

Eight legs each, which is a useful property: every robot does the same
amount of work, so the spread in finishing times is entirely queueing.

HOW THE CASCADE IS PRODUCED
---------------------------
Not by timers. Each robot takes the node ahead before it moves and releases
the one behind after it arrives -- hand over hand. The cascade falls out of
that: Alvik2 cannot enter DE1 until Alvik1 has left it, which is exactly
"wait for the robot ahead to reach the next node".

The reservation protocol is deadlock-free FOR THESE FIXED,
UNIDIRECTIONAL LAUNCH PATHS: every robot only ever wants a node further
along one line in one direction, and the lead robot's next node is never
held, so the wait-for chain always terminates at a robot that can move.
That is a property of this path set, not of the mechanism. On the general
grid, with opposing and crossing traffic, reservations can absolutely form
cycles and need the supervisor's deadlock detection. It also says nothing
about physical failures -- lost localization, a refused turn, a robot
dropping off the agent -- which is what --leg-timeout is for.

SAFETY
------
Nothing here plans around obstacles or watches for collisions; it assumes
the six robots start parked in their own slots and the lane is clear.
Ctrl-C sends STOP to all six. A leg that overruns --leg-timeout stops the
whole fleet rather than letting one robot keep driving into a queue.

RUN IT (Linux laptop, ROS 2 sourced, micro-ROS agent up, robots green,
apriltag_localize.py --rosbridge publishing):

    python3 launch_cascade_test.py --dry-run      # plan + locking, no motion
    python3 launch_cascade_test.py                # the real thing
    python3 launch_cascade_test.py --robots Alvik1 Alvik2

Writes launch_cascade_<timestamp>.json with every leg's start and end, so
runs can be compared.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time

# ROS and the navigator are imported lazily in main(), so --dry-run (which
# touches no hardware) works on any machine -- including the Windows laptop
# where the paths are edited.
rclpy = None
CameraGridNavigator = heading_between = node_to_world = None
mode_requires_odom = None


def _load_ros() -> None:
    global rclpy, CameraGridNavigator, heading_between, node_to_world
    global mode_requires_odom
    import rclpy as _rclpy
    from camera_grid_navigate import (CameraGridNavigator as _Nav,
                                      heading_between as _hb,
                                      node_to_world as _n2w)
    from localization_modes import mode_requires_odom as _mro
    rclpy, CameraGridNavigator = _rclpy, _Nav
    heading_between, node_to_world, mode_requires_odom = _hb, _n2w, _mro

ROWS, COLS = 8, 8

#: Node sequences, written out rather than generated: this is the thing
#: under test, and it should be readable at a glance and editable without
#: re-deriving an index scheme.
PATHS: dict[str, list[str]] = {
    "Alvik1": ["D1", "DE1", "0", "1", "9", "17", "25", "33", "41"],
    "Alvik2": ["D2", "DE2", "DE1", "0", "1", "9", "17", "25", "33"],
    "Alvik3": ["D3", "DE3", "DE2", "DE1", "0", "1", "9", "17", "25"],
    "Alvik4": ["D4", "DE4", "DE3", "DE2", "DE1", "0", "1", "9", "17"],
    "Alvik5": ["D5", "DE5", "DE4", "DE3", "DE2", "DE1", "0", "1", "9"],
    "Alvik6": ["D6", "DE6", "DE5", "DE4", "DE3", "DE2", "DE1", "0", "1"],
}


class NodeLocks:
    """Hand-over-hand exclusion over the shared nodes.

    One lock per node, held from before the robot starts moving toward it
    until after the robot has left it. Deliberately not a timed headway: a
    robot that brakes late or needs a camera correction holds its node for
    exactly as long as it is really there, which a fixed gap cannot express.
    """

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._held: dict[str, str] = {}      # node -> robot
        self.waits: list[tuple[str, str, float]] = []   # robot, node, seconds

    def seed(self, robot: str, node: str) -> None:
        with self._cv:
            self._held[node] = robot

    def acquire(self, robot: str, node: str, timeout: float) -> bool:
        """Take `node`, or return False if it is still held when time runs out.

        timeout=0.0 is the opportunistic probe the rolling reservation uses
        to extend a run: never block, just take it if it happens to be free.
        """
        t0 = time.monotonic()
        with self._cv:
            while self._held.get(node) not in (None, robot):
                if not self._cv.wait(timeout=max(0.05, timeout -
                                                 (time.monotonic() - t0))):
                    if time.monotonic() - t0 >= timeout:
                        return False
            self._held[node] = robot
        waited = time.monotonic() - t0
        if waited > 0.05:
            self.waits.append((robot, node, round(waited, 2)))
        return True

    def release(self, robot: str, node: str) -> None:
        with self._cv:
            if self._held.get(node) == robot:
                del self._held[node]
                self._cv.notify_all()

    def holder(self, node: str) -> str | None:
        with self._cv:
            return self._held.get(node)


#: How far past a node the AprilTag centre must be before that node is
#: released. The tag sits near the robot's centre, so crossing a node is not
#: the same as clearing it -- the back of the robot is still in the
#: intersection.
#:
#: NOT YET MEASURED. It should be
#:     C = tag-centre-to-rear + localization uncertainty + safety margin
#: and 5.0in is a placeholder chosen to be comfortably larger than any of
#: those. Measure the first term with a ruler and the second from the
#: spread in a stationary robot's reported pose, then set this from the sum
#: rather than by taste -- it is the one number here that trades throughput
#: against physical separation, so it should be defensible.
CLEARANCE_IN = 5.0


def min_separation(runners: list) -> tuple:
    """Closest pair of robots right now, as (inches, a, b), or (inf, ...).

    Node exclusion protects the logical grid, but continuous driving means
    physical following distance is now its own question: a robot released
    past node 9 can keep closing on the one ahead if that one brakes or
    stops for a camera correction. This does not control anything yet -- it
    is measured so the launch throughput number comes with the separation it
    was bought at.
    """
    best, pair = float("inf"), (None, None)
    poses = [(r.robot, r.nav.last_pose) for r in runners
             if r.nav is not None and r.nav.last_pose is not None]
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            (na, pa), (nb, pb) = poses[i], poses[j]
            d = math.hypot(pa.x - pb.x, pa.y - pb.y)
            if d < best:
                best, pair = d, (na, nb)
    return best, pair[0], pair[1]


def release_passed(runners: list, locks: "NodeLocks", clearance: float) -> None:
    """Give back every reserved node the robot has physically cleared.

    Called while drives are in flight. Without this a robot driving 0 -> 41
    in one continuous leg would hold node 0 for the whole column and the
    funnel would be worse than one-hop-at-a-time, which defeats the point.

    Uses the raw camera pose rather than a node label: progress is measured
    as distance along the direction of travel past the node, so a node is
    only released once the footprint is genuinely out of it.
    """
    for run in runners:
        fl, nav = run.flight, run.nav
        if not fl or nav is None:
            continue
        pose = nav.last_pose
        if pose is None:
            continue
        ux, uy = fl["dir"]
        for entry in list(fl["pending"]):
            node, (nx, ny) = entry
            if (pose.x - nx) * ux + (pose.y - ny) * uy > clearance:
                locks.release(run.robot, node)
                try:
                    fl["pending"].remove(entry)
                except ValueError:
                    pass


def pump(navs: list, seconds: float) -> None:
    """Spin every navigator's executor for a while.

    A CameraGridNavigator only pumps its own executor from inside a blocking
    primitive (drive_leg, turn_*, wait_for_*). Any wall-clock wait on the
    main thread therefore starves ALL of them at once, and ODOM_STALE_SEC is
    0.75s -- so a bare sleep(3) between setup and the first turn guarantees
    "no fresh odometry yaw" on every robot. Measured exactly that on
    2026-09-15, first hardware attempt.
    """
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for nav in navs:
            nav._spin_once(0.005)


def all_fresh(navs: list, need_odom: bool) -> list:
    """Names of navigators whose streams are NOT currently fresh."""
    stale = []
    for nav in navs:
        if nav.fresh_pose() is None:
            stale.append(f"{nav.robot}(vision)")
        elif need_odom and nav.fresh_odom_pose() is None:
            stale.append(f"{nav.robot}(odom)")
    return stale


def brake_lead_for(args, robot: str) -> float:
    """Per-robot brake lead, falling back to the fleet default.

    Stopping distance is a property of the individual robot, not the fleet.
    Measured 2026-09-15 at a uniform 0.45in lead: five robots landed 0.3-0.4in
    short while Alvik4 ran 1.3in PAST every node -- it slides ~1.55in where
    the others slide ~0.6in. That overshoot is not cosmetic: arriving past a
    node skews the next leg, and Alvik4 reached node 1 at +167deg instead of
    180, needing a 12.9deg correction -- exactly the size of turn that does
    not reliably execute.
    """
    return args.brake_lead_for.get(robot, args.brake_lead)


def make_nav(name: str, mode: str, brake_lead: float = 1.9):
    """Same Namespace the supervisor's vision worker builds, so this rig and
    a real mission drive the robots identically -- otherwise a timing
    difference here would say nothing about the mission."""
    import argparse as _ap
    args = _ap.Namespace(
        verbose=False, rows=ROWS, cols=COLS, localization_mode=mode,
        turn_tol_deg=4.0, turn_settle_count=3, turn_min_speed=20.0,
        turn_decel_zone_deg=35.0, turn_creep_speed=4.0,
        turn_scale_deg=60.0, turn_max_speed=35.0,
        turn_rpm=35.0, turn_brake_lead_deg=30.0, turn_creep_rpm=10.0,
        cruise_rpm=60.0, brake_lead_in=brake_lead,
        kp_yaw=0.6, max_turn_adjust=10.0,
    )
    return CameraGridNavigator(name, args)


class Runner:
    def __init__(self, robot: str, path: list[str], locks: NodeLocks,
                 args: argparse.Namespace, t0: float, abort: threading.Event):
        self.robot, self.path, self.locks = robot, path, locks
        self.args, self.t0, self.abort = args, t0, abort
        self.nav: CameraGridNavigator | None = None
        self.legs: list[dict] = []
        self.error: str | None = None
        #: set while a drive is in flight, read by the release monitor
        self.flight: dict | None = None
        #: one diagnostic record per rotation, from the navigator
        self.turns: list[dict] = []
        #: along-path shortfall at the end of each drive, inches
        self.stop_errors: list[float] = []
        self.thread = threading.Thread(target=self._run, name=f"cascade-{robot}",
                                       daemon=True)

    # -- setup is done on the main thread so a failure stops the run before
    #    anything moves, rather than half the fleet driving off.
    def prepare(self) -> str | None:
        self.nav = make_nav(self.robot, self.args.localization_mode,
                            brake_lead_for(self.args, self.robot))
        failure = self.nav.wait_for_localization_ready()
        if failure:
            return failure
        if mode_requires_odom(self.args.localization_mode):
            if not self.nav.reset_onboard_pose("cascade start"):
                return "onboard pose reset failed"
            if not self.nav.wait_for_fresh_odom(timeout_sec=3.0):
                return "no fresh onboard pose after reset"
        return None

    def _stamp(self) -> float:
        return round(time.monotonic() - self.t0, 2)
    def _run(self) -> None:
        """Drive the path as a few long straight runs, not many short hops.

        ROLLING RESERVATION
        -------------------
        The first version took one node at a time and braked at every one.
        That is wrong twice over: the depot lane and the west column are
        straight lines, so a robot braking at DE5, DE4, DE3, DE2 and DE1 on
        its way to node 0 is stopping five times for no reason; and holding
        each node until arrival forced the robot behind to wait for a full
        stop-and-go cycle rather than for clear space.

        So: take the longest run of consecutive nodes that are (a) collinear
        with the first step and (b) not reserved by anyone else, then drive
        the whole stretch with ONE drive_leg. drive_leg steers to a world
        point and brakes once at the end -- it has no concept of nodes -- so
        a five-node straight run costs one brake instead of five.

        The far end of the reserved run is the guaranteed stopping point:
        the robot always knows where it may come to rest without further
        permission. Only the next node is blocking; everything beyond it is
        opportunistic, so a busy lane degrades to the old one-hop behaviour
        instead of stalling.

        Nodes behind are released by the monitor as the robot's footprint
        clears them (see release_passed), not when the whole run finishes --
        otherwise a robot driving 0 -> 41 would hold node 0 for the entire
        column and the funnel would be worse than before.
        """
        nav, path = self.nav, self.path
        if self.args.stagger:
            idx = int(self.robot[-1]) - 1
            deadline = self.t0 + idx * self.args.stagger
            while time.monotonic() < deadline and not self.abort.is_set():
                nav._spin_once(0.05)

        i = 0
        while i < len(path) - 1 and not self.abort.is_set():
            frm = path[i]
            fx, fy = node_to_world(frm, ROWS, COLS)
            nx, ny = node_to_world(path[i + 1], ROWS, COLS)
            heading = heading_between(fx, fy, nx, ny)
            # unit vector of travel, taken from the coordinates rather than
            # the heading angle so the yaw convention cannot creep in here
            span = math.hypot(nx - fx, ny - fy)
            ux, uy = (nx - fx) / span, (ny - fy) / span

            # how far along the path stays on this same straight line
            last_collinear = i + 1
            while last_collinear + 1 < len(path):
                ax, ay = node_to_world(path[last_collinear], ROWS, COLS)
                bx, by = node_to_world(path[last_collinear + 1], ROWS, COLS)
                d = math.hypot(bx - ax, by - ay)
                if d < 1e-6 or abs((bx - ax) / d - ux) > 1e-3                         or abs((by - ay) / d - uy) > 1e-3:
                    break
                last_collinear += 1

            def freshen(limit: float = 2.0) -> None:
                end = time.monotonic() + limit
                while (nav.fresh_pose() is None and time.monotonic() < end
                       and not self.abort.is_set()):
                    nav._spin_once(0.02)

            # Aim before reserving: a rotation in place never enters the
            # node ahead, so it never needed that permission, and doing it
            # while still waiting keeps it off the critical path.
            t_turn0 = self._stamp()
            freshen()
            if (mode_requires_odom(self.args.localization_mode)
                    and not nav.ensure_odom_anchor(fx, fy, heading)):
                self.error = f"no fresh odometry to anchor at {frm}"
                self.abort.set()
                return
            ok_turn = nav.turn_to_heading_rotate_rel(
                heading, f"{frm}->{path[i + 1]}")
            rec = dict(getattr(nav, "last_rotation", {}) or {})
            if rec:
                rec.setdefault("result", "SUCCESS" if ok_turn else "FAILED")
                rec["at_s"] = self._stamp()
                self.turns.append(rec)
            if not ok_turn:
                self.error = f"turn failed at {frm}"
                self.abort.set()
                return
            t_ready = self._stamp()

            # The next node is required; everything past it is a bonus.
            while not self.locks.acquire(self.robot, path[i + 1], timeout=0.2):
                if self.abort.is_set():
                    return
                nav._spin_once(0.05)
                if self._stamp() - t_ready > self.args.leg_timeout:
                    self.error = (f"waited >{self.args.leg_timeout:.0f}s for "
                                  f"{path[i + 1]}")
                    self.abort.set()
                    return
            far = i + 1
            while far < last_collinear and (far - i) < self.args.max_run:
                if not self.locks.acquire(self.robot, path[far + 1],
                                          timeout=0.0):
                    break
                far += 1
            t_go = self._stamp()

            tx, ty = node_to_world(path[far], ROWS, COLS)
            label = f"{frm}->{path[far]}"
            if far > i + 1:
                label += f" (through {', '.join(path[i + 1:far])})"
            # Hand the monitor the nodes to release as the footprint clears
            # them, newest last, with the travel direction.
            self.flight = {
                "dir": (ux, uy),
                "pending": [(n, node_to_world(n, ROWS, COLS))
                            for n in path[i:far]],
            }
            ok = nav.drive_leg(tx, ty, label)
            self.flight = None
            for n in path[i:far]:
                self.locks.release(self.robot, n)
            if not ok:
                self.error = f"drive failed on {label}"
                self.abort.set()
                return
            t_done = self._stamp()
            landed = nav.last_pose
            if landed is not None:
                self.stop_errors.append(
                    round(math.hypot(landed.x - tx, landed.y - ty), 2))

            self.legs.append({
                "leg": label, "through": far - i - 1,
                "turn_s": round(t_ready - t_turn0, 2),
                "blocked_s": round(t_go - t_ready, 2),
                "drive_s": round(t_done - t_go, 2),
                "arrived_s": t_done,
            })
            print(f"  [{self._stamp():7.2f}] {self.robot} {label:<28} "
                  f"turn {t_ready - t_turn0:4.2f}s  drive {t_done - t_go:4.2f}s"
                  + (f"  (waited {t_go - t_ready:.2f}s)"
                     if t_go - t_ready > 0.05 else ""))
            i = far


    def arrival_at(self, node: str) -> float | None:
        for leg in self.legs:
            if leg["leg"].endswith(f"->{node}"):
                return leg["arrived_s"]
        return None


def dry_run(paths: dict[str, list[str]]) -> None:
    """Replay the locking protocol with no ROS and no robots.

    Confirms the sequence reaches the end for every robot and reports the
    order nodes change hands -- which is the cascade, written down.
    """
    holder = {p[0]: r for r, p in paths.items()}
    idx = {r: 0 for r in paths}
    order, step = [], 0
    while any(idx[r] < len(paths[r]) - 1 for r in paths) and step < 500:
        step += 1
        moved = False
        for r in sorted(paths):
            i = idx[r]
            if i >= len(paths[r]) - 1:
                continue
            frm, to = paths[r][i], paths[r][i + 1]
            if holder.get(to) in (None, r):
                holder.pop(frm, None)
                holder[to] = r
                idx[r] = i + 1
                order.append((step, r, f"{frm}->{to}"))
                moved = True
        if not moved:
            print("  DEADLOCK: no robot can move -- the path set is wrong")
            return
    print(f"  completed in {step} rounds of hand-over-hand movement")
    for s, r, leg in order:
        print(f"    round {s:>2}  {r} {leg}")
    ends = {r: paths[r][-1] for r in sorted(paths)}
    print(f"  final: {ends}")
    assert len(set(ends.values())) == len(ends), "two robots end on one node"


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--robots", nargs="+", default=sorted(PATHS))
    ap.add_argument("--localization-mode", default="camera_assist")
    ap.add_argument("--leg-timeout", type=float, default=60.0,
                    help="a robot blocked this long on one node stops the run")
    ap.add_argument("--stagger", type=float, default=0.0,
                    help="seconds between robot N and N+1 starting; 0 = all "
                         "at once, which is the point of the test")
    ap.add_argument("--brake-lead", type=float, default=1.9, metavar="IN",
                    help="how far before the target the hard brake fires. "
                         "The 1.9 default was measured 2026-07-28 against a "
                         "LAGGIER pose stream; with the background spinner "
                         "poses are fresh and robots now stop ~1.3in short, "
                         "so this needs re-measuring (--stop-test).")
    ap.add_argument("--brake-lead-for", action="append", default=[],
                    metavar="ROBOT=IN",
                    help="per-robot brake lead, repeatable, e.g. "
                         "--brake-lead-for Alvik4=1.5. Overrides "
                         "--brake-lead for that robot.")
    ap.add_argument("--max-run", type=int, default=8, metavar="N",
                    help="most nodes to reserve ahead in one straight run "
                         "(default 8 = the whole path). 1 reproduces the old "
                         "stop-at-every-node behaviour for comparison.")
    ap.add_argument("--clearance", type=float, default=CLEARANCE_IN,
                    metavar="IN",
                    help="how far past a node the robot must be before that "
                         "node is released (default %(default)s in)")
    ap.add_argument("--dry-run", action="store_true",
                    help="replay the locking with no ROS and no motion")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    overrides = {}
    for item in args.brake_lead_for:
        name, _, value = item.partition("=")
        if not value:
            raise SystemExit(f"--brake-lead-for wants ROBOT=INCHES, got {item!r}")
        overrides[name] = float(value)
    args.brake_lead_for = overrides

    paths = {r: PATHS[r] for r in args.robots if r in PATHS}
    missing = [r for r in args.robots if r not in PATHS]
    if missing:
        raise SystemExit(f"no path defined for {missing}")

    print(f"launch cascade: {', '.join(sorted(paths))}")
    for r in sorted(paths):
        print(f"  {r}: {' -> '.join(paths[r])}")
    if args.dry_run:
        print("\ndry run -- protocol only, nothing moves:")
        dry_run(paths)
        return

    _load_ros()
    rclpy.init()
    locks = NodeLocks()
    abort = threading.Event()
    t0 = time.monotonic()
    runners = [Runner(r, paths[r], locks, args, t0, abort)
               for r in sorted(paths)]
    try:
        print("\npreparing navigators (localization must be live)...")
        for run in runners:
            failure = run.prepare()
            if failure:
                raise SystemExit(f"{run.robot}: {failure}")
            locks.seed(run.robot, run.path[0])
            # Callbacks now get a dedicated thread per robot. Without this,
            # they are only serviced when a motion primitive calls
            # _spin_once(), which handles ONE callback per call and cannot
            # keep up with 60Hz vision once several navigators share the
            # laptop -- measured 2026-09-15 as total pose starvation on two
            # robots and a collision.
            run.nav.start_background_spin()
            print(f"  {run.robot} ready, holding {run.path[0]}")

        navs = [r.nav for r in runners]
        need_odom = mode_requires_odom(args.localization_mode)
        # Re-establish freshness on every robot at once: each prepare() above
        # left the previously-prepared navigators unspun, so by the time the
        # last is ready the first one's odometry has aged out.
        print()
        for remaining in (3, 2, 1):
            print(f"  starting in {remaining}s -- ctrl-c to stop")
            pump(navs, 1.0)
        stale = all_fresh(navs, need_odom)
        deadline = time.monotonic() + 5.0
        while stale and time.monotonic() < deadline:
            pump(navs, 0.2)
            stale = all_fresh(navs, need_odom)
        if stale:
            raise SystemExit("stale localization at start: " + ", ".join(stale)
                             + " -- is apriltag_localize publishing, and are "
                               "the robots green?")
        t0 = time.monotonic()
        stop_monitor = threading.Event()

        closest = {"in": float("inf"), "pair": None, "at": None}

        def monitor() -> None:
            # ~20Hz. Camera localization is faster, so this adds up to 50ms
            # of release latency -- negligible against 2-4s legs, but it is
            # why a measured headway can read ~50ms long.
            while not stop_monitor.is_set():
                release_passed(runners, locks, args.clearance)
                d, a, b = min_separation(runners)
                if d < closest["in"]:
                    closest.update({"in": round(d, 2), "pair": [a, b],
                                    "at": round(time.monotonic() - t0, 2)})
                time.sleep(0.05)

        mon = threading.Thread(target=monitor, name="release-monitor",
                               daemon=True)
        mon.start()
        for run in runners:
            run.t0 = t0
            run.thread.start()
        for run in runners:
            run.thread.join()
        stop_monitor.set()
        mon.join(timeout=1.0)
    except KeyboardInterrupt:
        print("\ninterrupted -- STOP to all robots")
        abort.set()
    finally:
        for run in runners:
            if run.nav is not None:
                try:
                    run.nav.send_cmd("STOP")
                except Exception:
                    pass
        time.sleep(0.3)
        for run in runners:
            if run.nav is not None:
                try:
                    run.nav.destroy_node()
                except Exception:
                    pass
        rclpy.shutdown()

    print("=" * 70)
    for run in runners:
        if run.error:
            print(f"  {run.robot}: FAILED -- {run.error}")

    def tot(run, key):
        return round(sum(l[key] for l in run.legs), 2)

    print(f"  {'robot':<9}{'to node 0':>10}{'finished':>10}{'stops':>7}"
          f"{'thru':>6}{'turn':>8}{'drive':>8}{'blocked':>9}")
    for run in runners:
        zero = run.arrival_at("0")
        end = run.legs[-1]["arrived_s"] if run.legs else None
        print(f"  {run.robot:<9}"
              f"{(f'{zero:.2f}s' if zero is not None else '-'):>10}"
              f"{(f'{end:.2f}s' if end is not None else '-'):>10}"
              f"{len(run.legs):>7}"
              f"{sum(l['through'] for l in run.legs):>6}"
              f"{tot(run, 'turn_s'):>7.2f}s{tot(run, 'drive_s'):>7.2f}s"
              f"{tot(run, 'blocked_s'):>8.2f}s")

    # Rotation health. The intermittent multi-robot failure was dropped
    # ROTATE_REL commands on the BEST_EFFORT command topic -- invisible
    # except as a robot that never moved. `resent` counts commands that
    # needed more than one send to be acknowledged: it should be 0 on a
    # quiet link and rise with the number of active navigators.
    print()
    print(f"  {'robot':<9}{'turns':>7}{'1st try':>9}{'resent':>8}"
          f"{'no-ack':>8}{'no-motion':>11}{'timeout':>9}")
    for run in runners:
        t = run.turns
        print(f"  {run.robot:<9}{len(t):>7}"
              f"{sum(1 for x in t if x.get('sends') == 1):>9}"
              f"{sum(1 for x in t if (x.get('sends') or 1) > 1):>8}"
              f"{sum(1 for x in t if x.get('result') == 'COMMAND_NOT_ACKNOWLEDGED'):>8}"
              f"{sum(1 for x in t if x.get('result') == 'NO_MOTION'):>11}"
              f"{sum(1 for x in t if x.get('result') == 'FIRMWARE_TIMEOUT'):>9}")
    # Pose-stream health per robot. This is the number that distinguishes
    # "the algorithm is slow" from "the robot could not see where it was".
    print()
    print(f"  {'robot':<9}{'vision Hz':>11}{'odom Hz':>9}"
          f"{'worst vis gap':>15}{'worst odom gap':>16}")
    elapsed = max((r.legs[-1]["arrived_s"] for r in runners if r.legs),
                  default=1.0) or 1.0
    for run in runners:
        nav = run.nav
        if nav is None:
            continue
        print(f"  {run.robot:<9}{nav.vision_msgs / elapsed:>10.1f}"
              f"{nav.odom_msgs / elapsed:>9.1f}"
              f"{nav.vision_max_age * 1000:>13.0f}ms"
              f"{nav.odom_max_age * 1000:>14.0f}ms")

    # Where each leg actually stopped versus where it aimed. This is the
    # measurement that sets --brake-lead: the whole point of a hard brake is
    # landing on the node, and the slide distance changed when pose latency
    # did.
    for run in runners:
        if run.stop_errors:
            e = run.stop_errors
            print(f"  {run.robot:<9}stopping error mean {sum(e)/len(e):.2f}in  "
                  f"max {max(e):.2f}in  (lead "
                  f"{brake_lead_for(args, run.robot)}in)")
    errs = [e for r in runners for e in r.stop_errors]
    if errs:
        print()
        print(f"  stopping error: mean {sum(errs) / len(errs):.2f}in  "
              f"range {min(errs):.2f}-{max(errs):.2f}in  over {len(errs)} leg(s)"
              f"   (brake-lead {args.brake_lead}in)")

    acks = [x["ack_latency_ms"] for r in runners for x in r.turns
            if x.get("ack_latency_ms") is not None]
    if acks:
        print(f"  BUSY ack latency: mean {sum(acks) / len(acks):.0f}ms  "
              f"max {max(acks)}ms  over {len(acks)} command(s)")

    # Headway at a node is the throughput of the funnel: the interval between
    # successive robots passing the same point. Node 0 is the bottleneck, so
    # its headway is what decides whether a launch policy is fast -- more
    # informative than "the sixth robot arrived at 31s".
    def headways(node):
        times = sorted(r.arrival_at(node) for r in runners
                       if r.arrival_at(node) is not None)
        return [round(b - a, 2) for a, b in zip(times, times[1:])]

    summary = {
        "max_run": args.max_run,
        "clearance_in": args.clearance,
        "stagger_sec": args.stagger,
        "grid_entry_makespan_s": max(
            (r.arrival_at("0") for r in runners
             if r.arrival_at("0") is not None), default=None),
        "launch_makespan_s": max(
            (r.legs[-1]["arrived_s"] for r in runners if r.legs), default=None),
        "headway_node0_s": headways("0"),
        "headway_node1_s": headways("1"),
        "stops_total": sum(len(r.legs) for r in runners),
        "passthrough_total": sum(l["through"] for r in runners for l in r.legs),
        "blocked_total_s": round(sum(tot(r, "blocked_s") for r in runners), 2),
        "turn_total_s": round(sum(tot(r, "turn_s") for r in runners), 2),
        "drive_total_s": round(sum(tot(r, "drive_s") for r in runners), 2),
        "min_separation_in": (None if closest["in"] == float("inf")
                              else closest["in"]),
        "min_separation_pair": closest["pair"],
        "min_separation_at_s": closest["at"],
        "failed": [r.robot for r in runners if r.error],
        "turn_commands": sum(len(r.turns) for r in runners),
        "turn_resends": sum(1 for r in runners for x in r.turns
                            if (x.get("sends") or 1) > 1),
        "turn_unacked": sum(1 for r in runners for x in r.turns
                            if x.get("result") == "COMMAND_NOT_ACKNOWLEDGED"),
        "stop_errors_in": [e for r in runners for e in r.stop_errors],
        "brake_lead_in": args.brake_lead,
        "turn_no_motion": sum(1 for r in runners for x in r.turns
                              if x.get("result") == "NO_MOTION"),
        "pose_rates": {r.robot: {
            "vision_msgs": getattr(r.nav, "vision_msgs", None),
            "odom_msgs": getattr(r.nav, "odom_msgs", None),
            "vision_max_age_ms": round(
                getattr(r.nav, "vision_max_age", 0.0) * 1000),
            "odom_max_age_ms": round(
                getattr(r.nav, "odom_max_age", 0.0) * 1000),
        } for r in runners if r.nav is not None},
    }
    h0 = summary["headway_node0_s"]
    print()
    print(f"  grid entry (last onto node 0): "
          f"{summary['grid_entry_makespan_s']}s     r4 full stack: 120.01s")
    print(f"  launch makespan (all parked):  {summary['launch_makespan_s']}s")
    if h0:
        print(f"  node 0 headway: {h0}  mean {sum(h0) / len(h0):.2f}s  "
              f"max {max(h0):.2f}s")
    print(f"  max-run {args.max_run}: {summary['stops_total']} stops, "
          f"{summary['passthrough_total']} pass-throughs, "
          f"{summary['blocked_total_s']}s blocked")
    if summary["min_separation_in"] is not None:
        print(f"  closest approach: {summary['min_separation_in']}in between "
              f"{' and '.join(summary['min_separation_pair'])} at "
              f"t={summary['min_separation_at_s']}s")

    out = args.out or f"launch_cascade_{time.strftime('%Y%m%dT%H%M%S')}.json"
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "summary": summary,
                   "localization_mode": args.localization_mode,
                   "robots": {r.robot: {"path": r.path, "legs": r.legs,
                                        "turns": r.turns,
                                        "error": r.error} for r in runners},
                   "lock_waits": locks.waits}, fh, indent=1)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
