#!/usr/bin/env python3
"""
fleetSupervisor.py  —  Fleet-wide, single-step command supervisor for
AGV_Factory_color_pose.ino, built on the same per-robot protocol as Supervisor.py.

Unlike Supervisor.py (one process per robot, each robot runs ahead as
soon as it alone reports done), this script drives every robot from ONE
process and advances the WHOLE FLEET by exactly one command per step:

  1. Send every robot (that still has commands left) its next command.
  2. Wait for ALL of them to report a done marker.
  3. Stop and wait for you to press Enter (or type 'auto' to free-run,
     'quit' to stop) before releasing the next step.

This exists so a human (or later, the solver) can inspect the grid after
every single movement and catch a node/edge collision before it happens,
instead of letting every robot run its whole route unattended.

Usage:
    python3 fleetSupervisor.py \\
        --robot Alvik1 --file route_alvik1.txt \\
        --robot Alvik2 --file route_alvik2.txt \\
        --robot Alvik3 --tokens R,RED,RED,L,RED

Each --robot takes exactly one of --file or --tokens (same formats as
Supervisor.py: one command/token per line for --file, comma-separated
for --tokens). Robots with fewer remaining commands simply sit idle once
their list is exhausted while the others keep stepping.

Advisor mode (LLM-gated collision advisor, see LLM_DISPATCH_ADVISOR_DESIGN.md):

    python3 fleetSupervisor.py \\
        --schedule plan.json \\
        --robot Alvik1 --file route_alvik1.txt \\
        --robot Alvik2 --file route_alvik2.txt

With --schedule (the JSON exported from agv_grid_workstation_solver.html),
the fleet runs autonomously and each robot advances individually the moment
IT reports done — no lockstep, no Enter prompt. At every such trigger the
advisor re-anchors the exported schedule and applies direction-aware
node/edge/depot reservations. Collision waits keep an idle robot idle in the
supervisor and are re-evaluated frequently; they are not synthetic DWELL
commands. Planned workstation DWELL commands remain tracked route commands.
The optional local LLM (Ollama) may rank only deterministically safe wait
horizons and cannot override ownership or priority. Every decision is appended
to a JSONL audit log.

Fault recovery (vision-guided, added 2026-07-16): if a robot reports ERROR
LINE_LOST, or a motion command runs far past its planned duration (runaway
watchdog — a missed marker means the robot line-follows away hunting it),
the supervisor STOPs it and walks it back using only the existing command
vocabulary: ROTATE_180 (the just-followed line is now ahead), FORWARD_UNTIL_
RED onto a lattice node, a camera fix (node from position, heading from the
position delta — no yaw convention needed), then turns + FORWARD_UNTIL_REDs
back to where the plan can resume; the advisor re-anchors and the conflict
gate re-engages. At most RECOVERY_MAX_PER_ROBOT recoveries per robot per
mission; requires vision; the touch e-stop is never auto-recovered.

Extra flags for advisor mode:
    --no-llm             deterministic candidates only, no LLM calls (bench mode)
    --audit-log PATH     JSONL decision log (default: advisor_audit.jsonl)
    --timeout SEC        per-command completion timeout (default: 90)

Vision (camera ground truth):

If apriltag_localize.py --rosbridge is running on the camera laptop, this
supervisor also subscribes to /<Name>_vision_pose and /vision_calib and
reports each robot's measured grid position (cells relative to node 1) at
every lockstep prompt / after every step, plus a WARNING when two robots on
the lattice are closer than VISION_PROX_CELLS. In advisor mode, fresh vision
may conservatively veto an immediate movement; logical occupancy remains
authoritative if a camera update is stale. --no-vision disables it.
"""

from __future__ import annotations

import atexit
import itertools
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from localization_modes import (
    mode_requires_odom,
    normalize_localization_mode,
)

# Structured metrics events (Page-4 backend on the Windows laptop, see
# metrics_events.py). STRICTLY optional: every call is a no-op unless a
# mission arrives carrying metrics_run_id + a reachable backend URL, the
# emitter runs on its own daemon thread with a bounded queue, and it can
# never raise into control code. Missing module == metrics disabled.
try:
    from metrics_events import metrics_emitter
except Exception:  # noqa: BLE001 - emitter is optional by design
    metrics_emitter = None

# Same done-marker / token-mapping rules as Supervisor.py — AGV_Factory_color_pose.ino
# always follows "TURN COMPLETE"/"ROTATE_180 COMPLETE" with a bare "IDLE" in
# the same step, so "IDLE" alone is the real done signal.
DONE_MARKERS = ("IDLE", "STOPPED", "POSE_RESET", "ERROR")

# Command delivery is best-effort (the sketch subscribes best-effort), so a
# published command CAN be silently lost — seen 2026-07-13: a lost first
# FORWARD_UNTIL_BLUE let the robot's 1 Hz idle heartbeat pass as completion
# and the following turn fired at the depot. The robot's immediate
# "BUSY <cmd>" status is our delivery ack: resend until it appears, and
# never count a done-marker before it.
ACK_TIMEOUT_SEC = 2.0
MAX_SEND_ATTEMPTS = 4

TOKEN_TO_CMD = {
    "RED":    "FORWARD_UNTIL_RED",
    "YELLOW": "FORWARD_UNTIL_YELLOW",
    "BLUE":   "FORWARD_UNTIL_BLUE",
    "R":      "RIGHT_UNTIL_COLOR",
    "L":      "LEFT_UNTIL_COLOR",
}


def commands_from_file(path: str) -> list[str]:
    commands = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            commands.append(line.upper())
    return commands


def tokens_to_commands(token_csv: str) -> list[str]:
    tokens = [t.strip().upper() for t in token_csv.split(",") if t.strip()]
    commands = []
    for tok in tokens:
        if tok not in TOKEN_TO_CMD:
            raise ValueError(
                f"Token '{tok}' has no AGV_Factory_color_pose.ino equivalent "
                f"(supported: {sorted(TOKEN_TO_CMD)})")
        commands.append(TOKEN_TO_CMD[tok])
    return commands


def _is_depot_region_label(label) -> bool:
    """Node 0 and the depot-lane labels (DE#, D#) are the off-grid funnel — the
    corridor gate skips them (dense parking; robots sit close by design). Node 1
    is a real grid node and is NOT skipped, so the returning-into-node-1 guard
    stays. See the depot deadlock fix (2026-07-21)."""
    try:
        return int(label) == 0
    except (TypeError, ValueError):
        return True  # DE#, D#, etc.


def emergency_brake_targets(pair, distance, phase, in_recovery,
                            threshold=None):
    """Which robots in a too-close pair must be emergency-stopped.

    Pure decision helper (unit-testable without ROS). Returns the moving,
    non-recovering members of the pair when they are closer than the emergency
    threshold, else []. A robot already stopped (not 'busy') or already
    recovering is left alone — braking it again is pointless and would fight
    the recovery machinery."""
    if threshold is None:
        threshold = VISION_EMERGENCY_CELLS
    if distance >= threshold:
        return []
    a, b = pair
    return [n for n in (a, b)
            if phase.get(n) == "busy" and n not in in_recovery]


# ---- vision (ground truth from apriltag_localize.py) ----
VISION_STALE_SEC = 1.5      # pose older than this is reported as stale
VISION_PROX_CELLS = 0.75    # proximity warning threshold on the lattice
# Below this, two robots are about to touch — an active e-stop, not a warning.
# The advisor's space-time model can mispredict when robots run slower than
# their learned calibration (the 2026-07-21 collision: warnings climbed 0.41 ->
# 0.75 while the model thought the robots were separated). This is a hard
# physical backstop independent of the plan: STOP the moving robot immediately.
VISION_EMERGENCY_CELLS = 0.40
EMERGENCY_BRAKE_COOLDOWN_SEC = 3.0   # don't re-STOP the same pair every tick
VISION_TARGET_BLOCK_CELLS = 0.6  # another robot this close to a move target = hold
#: Inside those radii, a hold now needs the footprints to actually come
#: within this of each other along the move (FleetSupervisor._path_clears).
#: Square-on robots 5.0in apart have 1.0in; this allows for lane tracking
#: error and arrival offset, and still holds beside a robot that is turning.
CORRIDOR_FOOTPRINT_MARGIN_IN = 0.5
#: A robot PARKED off the lane (bay spur, depot slot) holds a move only if its
#: real footprint comes within this of the move's path. A correctly parked
#: robot leaves ~1in; 0.25in fires only when it rests ~0.8in+ too deep (2 of
#: 304 workstation stops, 2026-10-15..23).
PARKED_INTRUSION_MARGIN_IN = 0.25
# Corridor-gate radius: how close another robot must be to a node in this move's
# path to count as physically occupying it. Catches a robot ROTATING in place on
# a shared node (2026-07-21 depot collision: Alvik2 turning at node 1 blocked
# Alvik3) — a rotating robot stays within ~0.2 cells of the node center.
# MUST stay below ~0.71, the distance from a workstation/bay to its adjacent
# grid nodes: at 0.95 a robot parked in a bay (0.71 from the neighbouring node)
# falsely blocked traffic through that node (2026-07-21 workstation false-hold).
# 0.6 catches an on-node robot but excludes a robot tucked into a bay/workstation.
VISION_CORRIDOR_CLEAR_CELLS = 0.6
#: Lattice size. Single source for the geometry helpers here; the robot
#: channel builds its navigator with the same 8x8 (see rows=8, cols=8).
GRID_ROWS = 8
GRID_COLS = 8

# The depot funnel (node 0 + parking lane) sits BELOW the grid at grid_y < 0.
# A robot measured below this line is parked/funneling in the depot, not
# standing on a grid node, so the corridor gate ignores it (node 1 is at y=0).
DEPOT_LANE_Y_CUTOFF = -0.25

# --- collision detection and avoidance (2026-09-29) -----------------------
# Centre-to-centre distance at which two robots CAN touch. The robot was
# measured corner to corner at 5.625in (operator, 2026-09-30), so its worst-
# case radius is 2.81in and two of them touch at 5.625in centre-to-centre
# when both present a corner.
#
# RAISED 2026-09-30 from 5.04. That figure was never a measurement: the
# comment here claimed "15cm across the diagonal, so 5.04in", but 15cm is
# 5.91in, so the number and its justification never agreed. 5.04 sat between
# the robot's two real widths -- ~3.94in across the flats (see
# COLLISION_PARKED_CONTACT_IN) and 5.625in across the diagonal -- which means
# the guard modelled a robot that was square-on to the lane and went blind
# the moment one rotated. A rotation at a workstation entry node is exactly
# what made physical contact on 2026-09-30: Alvik3 was stationary (so not
# "closing"), then turned, and its corner swept into Alvik4.
#
# Sanity check on the geometry: 3.94in x sqrt(2) = 5.57in, against the 5.625in
# measured. The robot is near-square and the two figures agree.
#
# This raises the AVOID threshold too, since avoid = contact + closing *
# reaction -- which helps, but does NOT fix the separate finding that the
# brake fires too late (three of four events on 2026-09-30 needed more
# stopping distance than was left). See PLAN_2026-09-30.md item 1.
#
# NOT applied to COLLISION_PARKED_CONTACT_IN, deliberately: the table geometry
# is tighter than 5.625in in places (a workstation sits 5.22in from the
# adjacent bay entry, the bottom lane passes 5.71in from a depot slot), so a
# 5.625in threshold on parked pairs would fire continuously on legal states.
COLLISION_CONTACT_IN = 5.625

# A robot turning in place sweeps a circle of its corner-to-corner diameter,
# so two robots closer than this centre-to-centre cannot BOTH be where they
# are while either rotates. Same 5.625in measurement as COLLISION_CONTACT_IN
# -- named separately because it gates a different decision (may this robot
# start turning?) and should not silently follow a change made for the
# closing-speed guard.
#
# At the depot this permits what the operator asked for and blocks what they
# were worried about, with no separate rule: measured slot pitch is 6.30-
# 6.50in, so six correctly parked robots all rotate at once (+0.675in at the
# tightest pair). One robot 0.5in off its slot toward a neighbour still
# clears (5.80in). BOTH 0.5in off toward each other closes to 5.30in and is
# refused -- which is right, that overlaps by 0.325in.
ROTATION_SWEEP_IN = 5.625
# How long await_rotation_clearance() will hold a rotation before failing the
# leg. Matches RECOVERY_STAGE_TIMEOUT_SEC: long enough for a neighbour to
# finish a leg and move on, short enough that a genuine standoff surfaces as
# a failed leg rather than a silent stall.
ROTATION_CLEAR_TIMEOUT_SEC = 20.0
#: How long a rotation the gate has just allowed is treated as in progress
#: before the yaw rate can show it (see VisionMonitor.claim_rotation). A turn
#: and its settle take ~1.3-2.5s; past that the measured yaw rate takes over.
ROTATION_CLAIM_SEC = 3.0
#: A drive leg starting more than this off its own line is refused rather
#: than driven -- see VisionLegWorker._ready_to_depart(). Legs brake 1.0-1.4in
#: short ALONG the line; sideways a robot on the tape is within ~0.5in.
LEG_START_LATERAL_IN = 3.0
#: A drive leg starting more than this off its heading turns first.
DEPART_ALIGN_DEG = 20.0
#: A robot going STRAIGHT ON through a node whose only "turn" there is a
#: trim of arrival drift no bigger than this may skip it while the rotation
#: gate is blocked, and let drive_leg() steer it out -- if the exact
#: footprints say the drive stays clear (straight_trim_clear). 2026-10-30:
#: Alvik5 reached 43 5.1deg off on a straight 35->43->51, was held for that
#: trim by Alvik4 4.8in away at entry 143, and Alvik4's real 90deg turn was
#: held by Alvik5 -- deadlock abort. Well inside DEPART_ALIGN_DEG.
STRAIGHT_TRIM_SKIP_MAX_DEG = 10.0
#: How far along the leg straight_trim_clear() checks: a robot side (4in)
#: plus the 5in entry spacing clears anything on a neighbouring node.
STRAIGHT_TRIM_CHECK_IN = 9.0
ROTATION_CLEAR_POLL_SEC = 0.10
# A yield is the deadlock resolver's own escape move, so it cannot afford to
# wait out a 20 s hold -- if its departure turn is blocked, the sidestep was
# the wrong choice and failing fast returns control to the resolver.
YIELD_ROTATION_CLEAR_TIMEOUT_SEC = 3.0
# Everything here is measured in INCHES, deliberately. The lattice-only
# proximity_pairs() works in cells and filters to GRID_LO..GRID_HI, which
# excludes the depot lane entirely -- and the depot lane is exactly where
# Alvik5 drove into Alvik3 on 2026-09-29 without a single event being raised.
#
# The avoid threshold cannot simply be "contact plus a margin": adjacent
# depot slots are only 6.16in apart, so any fixed threshold above that fires
# continuously on two correctly parked robots. It is therefore based on
# CLOSING SPEED -- two robots sitting 6.2in apart are not closing and are
# fine, two robots closing at 6in/s at the same separation are not.
# 0.35 -> 0.56 after the 2026-10-15 contact (Alvik1/Alvik3, run 9ab100ec).
#
# THE OLD VALUE BUDGETED ONLY THE ROBOT'S STOPPING TIME, NOT THE SYSTEM'S
# TIME TO NOTICE. 0.35 came from turn coast (~4deg at 6 RPM is ~0.37s),
# which is how long the CHASSIS takes to stop once told. But the guard can
# only act on a pose sample it has received, and the measured pose cadence
# in that run was 209ms median (p90 219ms, max 233ms). So the earliest the
# guard can possibly react is one sample interval after the threshold is
# crossed, and that interval was nowhere in the budget.
#
# What it cost, from the recorded trace:
#     88.65s  sep 7.29in  closing ~3.0in/s  trigger 6.59  -> no fire
#     88.86s  sep 6.28in                    trigger 6.29  -> fires (0.49in
#                                            of braking still to come)
#     89.27s  sep 5.31in  stopped
#   true contact for their actual orientation was 5.32in, so they stopped
#   0.04in inside it -- inside the 0.06in vision rms, i.e. touching.
#
# 0.56 = 0.35 stopping + 0.21 measured sensing interval. On this trace it
# fires at 88.65 (trigger 7.31 >= sep 7.29) and, with the same 0.49in of
# braking, stops near 6.8in -- 1.5in clear.
#
# COST, measured over 42 six-robot vision runs / 561 fleet-minutes: avoid
# episodes rise from 0.69 to 0.97 per fleet-minute, 1.39x. An avoid is a
# brief mutual stop that resumes; a contact aborts the run. 0.70 was also
# tried (1.72x) and is not justified by the measured cadence.
COLLISION_REACTION_SEC = 0.56   # robot stop latency (0.35) + one pose
                                # interval (0.21), both measured
COLLISION_MIN_CLOSING_IN_S = 0.5  # below this, treat as not closing (noise)
# Closing rate is measured over AT LEAST this long. Consecutive vision frames
# are ~20ms apart and the two robots in a pair do not update on the same
# frame, so a per-frame difference divides a real change in separation by an
# almost-zero dt. 2026-09-29 produced "closing at 61.1in/s at 24.3in" that
# way and emergency-stopped two robots two feet apart.
# Contact threshold for a pair where one robot is PARKED OFF THE LANE -- in a
# depot slot or a bay spur. Those are dead ends the table is deliberately
# built tight around: the bottom lane passes 5.71in from a depot slot, and a
# workstation sits 5.22in from the adjacent bay entry. Judging those pairs by
# the normal avoid threshold makes the lane unusable the moment anything is
# parked beside it -- on 2026-09-29 it emergency-stopped Alvik3 mid-leg at
# "0.6in/s with 5.23in left", which dropped it out of wheel-follow and failed
# the leg. A parked robot is not closing on anyone; only genuine overlap
# counts. 4.0in is two axis-aligned robots (~3.94in across) actually touching.
COLLISION_PARKED_CONTACT_IN = 4.0
# ...but PARKED MEANS STOPPED, not merely located in a slot or bay spur. The
# position-only test exempted Alvik1 while it BACKED OUT of bay 73 into
# Alvik2's lane on 2026-10-19: the pair was judged on overlap alone, the
# predictive stop never fired, and they touched. Above this speed (in/s) a
# robot in a dead end is leaving it and gets the normal predictive check.
# 1.0 sits well above vision jitter (~0.2in/s) and well below cruise (~3.2).
PARKED_MAX_SPEED_IN_S = 1.0
# ...and PARKED MEANS IN A SLOT, not on the depot lane. The slots sit at
# y ~10.7in (-0.6 cells), the lane (DE1..DE7, node 0) at y ~3in (-1.37). The
# old test, "anywhere below the grid", exempted a robot STANDING ON THE LANE:
# 2026-10-22 Alvik1's turn at DE1 failed, it stood there, Alvik2 was sent
# DE2->DE1 and drove into it -- 0.83in overlap -- with no predictive stop,
# because the pair was judged on overlap alone. Midway between the rows:
DEPOT_SLOT_MIN_Y_CELLS = -1.0
COLLISION_MIN_DT_SEC = 0.15
# Above this, the "closing rate" is a vision glitch, not two robots. Cruise is
# ~3.2in/s, so two robots driving at each other close at ~6.4in/s; 10 leaves
# headroom for a genuinely fast approach while rejecting the 27 and 61in/s
# readings that triggered spurious stops.
COLLISION_MAX_CLOSING_IN_S = 10.0
COLLISION_MAX_AGE_SEC = 0.5     # poses older than this cannot be trusted
                                # for a safety decision


def in_bay_spur(grid_x: float, grid_y: float) -> bool:
    """Is this MEASURED position inside a workstation spur, past the entry?

    Module level and pure so it can be unit-tested; the gate that uses it is a
    closure and could otherwise only be exercised by running a fleet.

    WHY MEASURED AND NOT PLANNED. off_lane() used to decide "is this robot
    servicing a bay" from advisor.current_node. On 2026-09-29 Alvik1 was
    physically parked in bay 5 while a retry resynced its current_node from
    69 (the workstation) to 14 (a lattice node):

        [Alvik1] advisor had it at 69, vision says 14; resyncing before retry

    From that moment it was no longer "off lane", and since a workstation sits
    exactly 0.5 cells from the adjacent vertical grid edge it permanently
    blocked edge 6->14 that Alvik3 needed -- ~200 repeated holds and a
    deadlock. Geometry does not go stale; current_node did.

    THE TEST. In grid cells, within any cell:
        bay spur       gx fraction ~0.5, gy fraction ~0.5
        bay ENTRY      gx fraction ~0.5, gy fraction ~0.85  <- ON the corridor
        lattice node   gx fraction ~0.0, gy fraction ~0.0
        lane traffic   gy fraction ~0.0 (along a row) or gx ~0.0 (along a col)

    So a robot is in a spur when it sits mid-column AND mid-row. Entry nodes
    are deliberately NOT included: an entry lies mid-edge on the lattice and a
    robot there really does block the corridor.
    """
    fx = grid_x - math.floor(grid_x)
    fy = grid_y - math.floor(grid_y)
    return abs(fx - 0.5) <= 0.2 and 0.3 <= fy <= 0.7


#: The robot is 4.0 x 4.0 in on top (operator measurement, 2026-10-06).
#: 4.0 * sqrt(2) = 5.657 in, which is the 5.625 in corner-to-corner figure
#: measured separately -- the same robot from two angles.
ROBOT_SIDE_IN = 4.0


def contact_distance_in(yaw_a_deg, yaw_b_deg, dx: float, dy: float) -> float:
    """Centre separation at which two robots touch, GIVEN THEIR HEADINGS.

    WHY THIS EXISTS. A single contact distance cannot be right for both
    cases. Two robots squared up to each other -- parallel or perpendicular,
    which is how they sit on this grid almost all of the time -- touch at
    4.0 in. Two robots at 45 degrees touch at 5.657 in. Using the larger
    figure everywhere treats a legal pass as a collision.

    That is exactly what happened on 2026-10-06: Alvik1 held 180 degrees at a
    workstation while Alvik3 drove past it, and the guard called a collision
    at 4.74 in against a flat 5.625 in threshold. Physically the pair had
    room -- neither was turning, and neither intended to until the other had
    cleared.

    The projection is a separating-axis test along the line joining the two
    centres: a square of side S at angle phi to that line reaches
    (S/2)(|cos phi| + |sin phi|) along it, which is S/2 when squared up and
    S/sqrt(2) at 45 degrees. Summing both robots gives 4.0 in and 5.657 in
    respectively, so the model reproduces both measurements rather than
    interpolating between them.

    ROTATION IS NOT COVERED HERE, deliberately. A turning robot sweeps every
    orientation, so its clearance is the full 5.625 in circle -- enforced
    separately and BEFORE the turn starts, by the advisor's entry-rotation
    footprint and the supervisor's rotation-clearance gate. Keeping the two
    apart is what lets a pass-by be judged on the real footprint without
    weakening the rotation case.

    An unknown heading falls back to the worst case for that robot.
    """
    if dx == 0.0 and dy == 0.0:
        return ROTATION_SWEEP_IN
    alpha = math.degrees(math.atan2(dy, dx))
    half = 0.0
    for yaw in (yaw_a_deg, yaw_b_deg):
        if yaw is None:
            half += ROTATION_SWEEP_IN / 2.0
            continue
        phi = math.radians(alpha - float(yaw))
        half += (ROBOT_SIDE_IN / 2.0) * (abs(math.cos(phi)) + abs(math.sin(phi)))
    return half


#: Yaw rate above which a robot counts as TURNING. A turning robot is judged
#: as the disc its corners sweep, not as the square it happened to be in the
#: last frame. Drive legs trim heading by a few deg/s; turns run 60-150.
COLLISION_TURNING_DEG_S = 30.0
#: The predicted gap must stay above this through the stop horizon. Covers
#: pose noise (~0.05in) and heading noise (~1deg, ~0.05in at a corner).
COLLISION_PREDICT_MARGIN_IN = 0.25


def footprint_gap_in(dx: float, dy: float, yaw_a, yaw_b) -> float:
    """Clearance between two robots' footprints, in inches; <= 0 is contact.

    (dx, dy) runs from A's centre to B's. A yaw of None means that robot is
    turning: it is the disc of its corner radius, every orientation at once.

    WHY NOT contact_distance_in(). That projects onto ONE axis, the line
    between the centres, which for two squares is only one of the candidate
    separating axes -- and at a diagonal bearing it is the worst one, so it
    reports contact where there is none. Two square-on robots 5.0in apart
    side to side have exactly 1.0in between them wherever one is along the
    other (operator, 2026-10-16), but the centre-line projection called it
    5.20in "needed" at a 2in stagger. That is how a robot driving up the
    east leg of a workstation cell was emergency-stopped beside a robot
    standing at the bay entry, 5in away, on 2026-10-16.

    Exact for the shapes involved: the separating-axis test over all four
    face normals for square/square, point-to-square distance for disc/square.
    """
    half = ROBOT_SIDE_IN / 2.0
    radius = ROBOT_SIDE_IN / math.sqrt(2.0)
    if yaw_a is None and yaw_b is None:
        return math.hypot(dx, dy) - 2.0 * radius
    if yaw_a is None or yaw_b is None:
        sq_yaw = yaw_b if yaw_a is None else yaw_a
        # Vector from the square's centre to the disc's centre.
        vx, vy = (-dx, -dy) if yaw_a is None else (dx, dy)
        t = math.radians(sq_yaw)
        lx = abs(vx * math.cos(t) + vy * math.sin(t)) - half
        ly = abs(-vx * math.sin(t) + vy * math.cos(t)) - half
        if lx > 0.0 or ly > 0.0:
            d = math.hypot(max(lx, 0.0), max(ly, 0.0))
        else:
            d = max(lx, ly)
        return d - radius
    best = -math.inf
    for axis in (yaw_a, yaw_a + 90.0, yaw_b, yaw_b + 90.0):
        a = math.radians(axis)
        proj = abs(dx * math.cos(a) + dy * math.sin(a))
        reach = 0.0
        for yaw in (yaw_a, yaw_b):
            p = math.radians(yaw - axis)
            reach += half * (abs(math.cos(p)) + abs(math.sin(p)))
        best = max(best, proj - reach)
    return best


def predicted_min_gap_in(dx: float, dy: float, rvx: float, rvy: float,
                         yaw_a, yaw_b,
                         horizon_sec: float = None, steps: int = 10) -> float:
    """Smallest footprint gap over the next `horizon_sec`, both robots holding
    their current velocity. (rvx, rvy) is B's velocity relative to A.

    This is what the guard should have asked all along: not "how fast is the
    distance between centres shrinking", but "do the two footprints meet
    before we could stop them". A robot driving PAST another closes the
    centre distance quickly at first and then not at all; extrapolating that
    initial rate is what stopped legal pass-bys. Excludes t=0, so a pair that
    is close but separating is never stopped.
    """
    horizon = COLLISION_REACTION_SEC if horizon_sec is None else horizon_sec
    best = math.inf
    for k in range(1, steps + 1):
        t = horizon * k / steps
        best = min(best, footprint_gap_in(dx + rvx * t, dy + rvy * t,
                                          yaw_a, yaw_b))
    return best


#: How far past the end of its current leg a driving robot is assumed able to
#: run before it stops (brake lead is applied BEFORE the target; this covers
#: the coast). See predicted_min_gap_legs_in().
LEG_OVERSHOOT_IN = 0.75


def predicted_min_gap_legs_in(pa, va, stop_a, pb, vb, stop_b, yaw_a, yaw_b,
                              horizon_sec: float = None,
                              steps: int = 10) -> float:
    """predicted_min_gap_in(), but each robot stops where its leg ends.

    pa/pb: positions (in); va/vb: velocities (in/s); stop_a/stop_b: seconds
    until that robot reaches the end of its current leg (math.inf if it is
    not driving a known leg). A robot braking onto its target is not going
    to keep going. 2026-10-21: Alvik2 braking at DE1 while Alvik1 pulled off
    node 0 northbound -- 2.52 in apart -- was extrapolated 2.9 in past DE1
    into Alvik1's footprint and both were emergency-stopped; Alvik1 then
    failed its leg. The same false stop hit Alvik3/Alvik4 in the depot
    funnel that morning.
    """
    horizon = COLLISION_REACTION_SEC if horizon_sec is None else horizon_sec
    best = math.inf
    for k in range(1, steps + 1):
        tau = horizon * k / steps
        ta, tb = min(tau, stop_a), min(tau, stop_b)
        dx = (pb[0] + vb[0] * tb) - (pa[0] + va[0] * ta)
        dy = (pb[1] + vb[1] * tb) - (pa[1] + va[1] * ta)
        best = min(best, footprint_gap_in(dx, dy, yaw_a, yaw_b))
    return best


#: Margin for two robots sliding past each other in neighbouring parallel
#: lanes (see lane_passby()). Only their sideways gap matters there, and it
#: does not shrink by itself; pose noise is ~0.05in. 2026-10-22 SHAKE PP:
#: Alvik2 driving south into bay 1 (x 18.5) and Alvik3 north up the west
#: column (x 13.6), 4.8in centre to centre, 0.8in apart square-on, were
#: emergency-stopped at a predicted 0.25in because both were tilted 5-7deg;
#: neither could resume, Alvik3 was excluded, and the run deadlocked.
COLLISION_PASSBY_MARGIN_IN = 0.10
#: How square to a lane both robots and their relative motion must be.
PASSBY_ALIGN_DEG = 12.0


def lane_passby(yaw_a, yaw_b, rvx: float, rvy: float,
                dx: float, dy: float) -> bool:
    """True when two robots are passing each other in DIFFERENT parallel
    lanes: neither turning, both within PASSBY_ALIGN_DEG of the grid axes,
    their relative motion along one axis, and their centres at least a
    robot's width apart across it (so they are not in the same lane)."""
    if yaw_a is None or yaw_b is None:
        return False
    for yaw in (yaw_a, yaw_b):
        m = yaw % 90.0
        if min(m, 90.0 - m) > PASSBY_ALIGN_DEG:
            return False
    along, across, off = ((rvy, rvx, dx) if abs(rvy) >= abs(rvx)
                          else (rvx, rvy, dy))
    if abs(along) < 1e-6:
        return False
    if math.degrees(math.atan2(abs(across), abs(along))) > PASSBY_ALIGN_DEG:
        return False
    return abs(off) >= ROBOT_SIDE_IN


def motion_verdict(gap_now: float, min_gap: float, rel_speed: float,
                   sep_in: float, parked_pair: bool = False,
                   margin: float | None = None) -> str | None:
    """'collision' | 'avoid' | None, from footprints and relative motion.

    collision: the footprints overlap now.
    avoid:     they will come within `margin` (COLLISION_PREDICT_MARGIN_IN by
               default) of each other inside COLLISION_REACTION_SEC, and are
               getting closer.
    A parked pair keeps its own rule -- see COLLISION_PARKED_CONTACT_IN.
    """
    if parked_pair:
        return "collision" if sep_in <= COLLISION_PARKED_CONTACT_IN else None
    if gap_now <= 0.0:
        return "collision"
    if rel_speed < COLLISION_MIN_CLOSING_IN_S:
        return None
    m = COLLISION_PREDICT_MARGIN_IN if margin is None else margin
    if min_gap <= m and min_gap < gap_now:
        return "avoid"
    return None


def collision_verdict(sep_in: float, closing_in_s: float,
                      parked_pair: bool = False,
                      contact_in: float | None = None) -> str | None:
    """'collision' | 'avoid' | None for one pair of robots.

    Module level and pure so the safety rule can actually be unit-tested --
    buried in the supervisor's tick closure it could only be exercised by
    running a fleet, which is how the 2026-09-29 depot collision reached the
    table unnoticed in the first place.

    Contact is a fixed distance. The AVOID threshold is not: it is contact
    plus whatever the pair's closing speed will cover before they can stop.
    A fixed margin cannot work here, because adjacent depot slots are 6.16in
    apart and two correctly parked robots would trip it forever.
    """
    # Heading-aware when the caller can supply it; the worst case otherwise.
    contact = COLLISION_CONTACT_IN if contact_in is None else contact_in
    if parked_pair:
        # One of them is in a depot slot or a bay spur. See
        # COLLISION_PARKED_CONTACT_IN: that geometry is tight ON PURPOSE and a
        # parked robot is not closing on anybody, so only real overlap counts.
        # Stopping traffic for it strands the mover mid-leg, which is worse
        # than the thing being prevented.
        return "collision" if sep_in <= COLLISION_PARKED_CONTACT_IN else None
    if sep_in <= contact:
        return "collision"
    if closing_in_s < COLLISION_MIN_CLOSING_IN_S:
        return None
    if sep_in <= contact + closing_in_s * COLLISION_REACTION_SEC:
        return "avoid"
    return None
VISION_OFFROUTE_CELLS = 0.75     # own vision vs expected node farther than this = hold
#: A failed leg counts as having ARRIVED (and is banked) only this close to
#: its destination. Legs brake 1.0-1.4in short by design; 2.5in covers that,
#: and a robot stopped mid-leg (4.69in short on 2026-10-16) is not "there".
ARRIVAL_BANK_CELLS = 0.25


def depot_label(label) -> bool:
    """D1..Dn, DE1..DEn or node 0: the nodes below the grid."""
    return bool(re.fullmatch(r"(D|DE)\d+|0", str(label).strip().upper()))


def _depot_world():
    from camera_grid_navigate import (DEPOT_ENTRY_WORLD_IN,
                                      DEPOT_SLOT_WORLD_IN, NODE0_WORLD_IN)
    out = {"0": NODE0_WORLD_IN}
    out.update(DEPOT_SLOT_WORLD_IN)
    out.update(DEPOT_ENTRY_WORLD_IN)
    return out


def arrived_world_in(vision, name: str, label) -> bool:
    """`name`'s fresh pose is within ARRIVAL_BANK_CELLS of depot node `label`."""
    pose = vision.snapshot_in().get(name)
    target = _depot_world().get(str(label).strip().upper())
    if pose is None or target is None or pose[3] > VISION_STALE_SEC:
        return False
    return (math.hypot(pose[0] - target[0], pose[1] - target[1])
            <= ARRIVAL_BANK_CELLS * GRID_PITCH_WORLD_IN)


def nearest_depot_label(vision, name: str):
    """The depot node `name` is standing on (within ARRIVAL_BANK_CELLS), or
    None."""
    pose = vision.snapshot_in().get(name)
    if pose is None or pose[3] > VISION_STALE_SEC:
        return None
    best = None
    for label, (x, y) in _depot_world().items():
        d = math.hypot(pose[0] - x, pose[1] - y)
        if d <= ARRIVAL_BANK_CELLS * GRID_PITCH_WORLD_IN and (
                best is None or d < best[1]):
            best = (label, d)
    return best[0] if best else None
#: A yield is only started from a robot standing within this of the node the
#: plan has it on. A yield is "drive from your node to the next one"; from
#: anywhere else it is a diagonal drive across the grid -- on 2026-10-16 two
#: such yields (Alvik1 "13->5" from entry 118, Alvik3 "6->5" from mid-leg)
#: drove the pair into each other.
YIELD_START_TOL_IN = 3.0
VISION_EDGE_CORRIDOR_CELLS = 0.5   # projection distance from a requested edge.
# Raised from 0.35 after the 2026-07-20 collision, where Alvik3 projected 0.43
# cells from Alvik1's leaving-the-bay edge (entry 122 on the 18<>19 lane) and
# slipped under the old threshold. 0.5 = half a cell: a robot within half a
# lane-width of a requested edge blocks entry to it.
# proximity is only checked inside the 8x8 lattice; the depot row and node 0
# are off-lattice and robots legitimately park ~0.55 cells apart there
GRID_LO, GRID_HI = -0.3, 7.3

# ---- vision-guided recovery (LINE_LOST / runaway watchdog) ----
# A motion command that misses its marker line-follows away hunting for it
# (seen 2026-07-16: two robots crossed the whole grid and stopped off the
# boundary on the 400 ms failsafe). The watchdog STOPs a motion command that
# runs far past its planned duration; recovery then uses the camera to walk
# the robot back onto its plan with the existing vocabulary (ROTATE_180 +
# FORWARD_UNTIL_RED + turns). Firmware needs no changes: the cmd callback
# accepts commands in STATE_ERROR.
RECOVERY_MAX_PER_ROBOT = 2       # recoveries per robot per mission
RECOVERY_STAGE_TIMEOUT_SEC = 20.0
RECOVERY_SETTLE_SEC = 1.0        # let the robot brake + vision refresh
RECOVERY_MIN_DISP_CELLS = 0.25   # min displacement to trust a motion heading
RUNAWAY_FACTOR = 2.5             # STOP when elapsed > factor*planned + grace
RUNAWAY_GRACE_SEC = 3.0

ROBOT_DRIVE_CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "robot_drive_calibration.json")


def robot_drive_calibration() -> dict:
    """{robot: {"brake_lead_in", "steer_trim_rpm", ...}} from
    robot_drive_calibration.json, read fresh on every call so a redeployed
    file takes effect at the next mission. Missing or unreadable -> {} and
    every robot drives on the fleet-wide defaults, said loudly once."""
    try:
        with open(ROBOT_DRIVE_CALIBRATION_FILE, encoding="utf-8") as fh:
            return dict(json.load(fh).get("robots") or {})
    except (OSError, ValueError) as exc:
        if not getattr(robot_drive_calibration, "_warned", False):
            robot_drive_calibration._warned = True
            print(f"[supervisor] no per-robot drive calibration ({exc}); "
                  "using the fleet-wide brake lead and no steering trim")
        return {}
RUNAWAY_MIN_CAP_SEC = 6.0

# Collision/vision waits are supervisor state, not robot commands.
SUPERVISOR_WAIT_RECHECK_SEC = 0.20
# How long a wait-for CYCLE may persist before the run is aborted.
# A cycle is not ordinary queueing -- nothing in it can ever progress
# without an outside change, so waiting longer only delays the
# inevitable. 20s is comfortably past any transient the recheck loop
# might create while still ending the run in seconds rather than the
# 78s of hand-watched standstill that motivated it.
#: How far past a depot-lane node another robot's AprilTag centre must be
#: before this robot is released into it. 5.0 in is what the standalone
#: cascade rig used on 2026-09-15 to reach a 5.09 in measured closest
#: approach across 44 legs -- the tag sits near the robot's centre, so a
#: node is not clear the instant the centre passes it.
LAUNCH_CLEARANCE_IN = 5.0

DEADLOCK_ABORT_SEC = 20.0
#: A wait-for cycle must persist this long before it is declared. 2026-10-16:
#: four of the five DEADLOCKs in one run were one-tick artefacts as each
#: robot left the depot lane -- "deadlock cleared after 0.0s" every time --
#: and each wrote a DEADLOCK_DETECTED row, inflating deadlock_count. A real
#: deadlock persists by definition, so a second of patience costs nothing.
DEADLOCK_CONFIRM_SEC = 1.0
# How long to stand aside waiting for the blocked robot to pass, and the hard
# ceiling on a whole yield. Beyond the ceiling the robot is off its planned
# node with no way back, which is worse than the abort it was avoiding -- so
# the abort is what happens.
YIELD_CLEAR_TIMEOUT_SEC = 30.0
YIELD_TOTAL_TIMEOUT_SEC = 75.0

# Per-leg retry budget before a robot is excluded. Two, because every vision
# leg failure observed on this testbed has been transient and single: a turn
# that overshot, or one that never started. The navigator re-derives the whole
# maneuver from the CURRENT pose, so attempt two is a fresh attempt from
# wherever the robot actually is, not a replay. Set to 0 to restore the old
# exclude-on-first-failure behaviour.
MAX_LEG_RETRIES = 2
# Let the fleet settle before re-dispatching -- the robot has just braked and
# a neighbour may have been waiting on the resource it is about to re-request.
LEG_RETRY_PAUSE_SEC = 1.0
# Longest a robot may sit off-route before it is excluded. Unlike every other
# wait here, an off-route hold cannot be cleared by another robot moving, so
# without a bound it is a permanent fleet-wide stall (2026-09-25 run 4).
# Generous enough that a genuine transient -- a vision glitch, a robot still
# settling -- resolves well inside it.
# Consecutive off-route observations before excluding. At the ~0.2s recheck
# cadence this is roughly 20s of UNINTERRUPTED off-route, but unlike a wall
# clock it cannot be run up by ordinary queueing.
OFFROUTE_MAX_HITS = 100
# Main-loop ticks (~20ms) of a live robot waiting on an excluded robot's
# reservation before the run is ended as unrecoverable. ~10s, long enough that
# a robot excluded while others happen to be momentarily queued is not
# mistaken for a genuine block.
BLOCKED_ON_DEAD_TICKS = 500
# Global liveness bound. Seconds with NO robot advancing its plan before the
# run is ended as stalled. This exists because every stall this testbed has
# hit -- deadlock cycles, off-route holds, waiting on an excluded robot's
# reservation -- needed its own bespoke bound, and each was added only after
# it had already cost a run that had to be killed by hand. This is the
# catch-all: whatever the next novel stall turns out to be, the run ends by
# itself and says so. Measured against real behaviour it is generous -- the
# slowest legitimate fleet-wide pause is a deadlock yield, which completes in
# 12-45s, and during depot launch some robot is always moving even while
# others queue for over a minute.
RUN_STALL_SEC = 90.0
# How long a deadlock may stay open while one of ITS OWN members is mid-yield.
# Measured 2026-09-15: a complete yield (turn out, drive, stand aside, wait,
# turn back, drive home) took 47.7s -- every departure turn hit the firmware
# hard deadline and needed a settled camera verification plus a corrective
# ROTATE_REL. DEADLOCK_ABORT_SEC is 20s, so an executing yield cannot finish
# inside the plain abort window and the run would be killed mid-maneuver.
# Set above YIELD_TOTAL_TIMEOUT_SEC so the yield's own timeout fires first
# and hands the decision back to the abort timer, rather than the two racing.
DEADLOCK_YIELD_GRACE_SEC = YIELD_TOTAL_TIMEOUT_SEC + 5.0
# How often a deadlock whose resolution was DECLINED is
# reconsidered. A decline is usually temporary -- another yield in
# flight, or a neighbour held by a robot about to move -- so
# attempting once at detection throws away every later chance to
# recover while the abort timer keeps running. Slow enough that a
# permanently stuck cycle does not spin the resolver, which
# re-simulates the whole fleet on each call.
DEADLOCK_RETRY_SEC = 3.0

class RunTranscript:
    """Mirror this process's stdout+stderr into a per-run .txt file.

    ROS logging is emitted by rcutils at the C level, straight to file
    descriptors 1 and 2 -- reassigning sys.stdout would capture none of it.
    So the descriptors themselves are duplicated into a `tee`, which keeps
    the terminal live and writes the file at the same time.

    Nothing here is allowed to take a run down: if the transcript cannot be
    opened the run proceeds exactly as before, terminal only.
    """

    DIR = "runs"

    def __init__(self, path: str):
        self.path = path
        self._saved = None
        self._tee = None
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            self._tee = subprocess.Popen(["tee", path],
                                         stdin=subprocess.PIPE)
            sys.stdout.flush()
            sys.stderr.flush()
            self._saved = (os.dup(1), os.dup(2))
            os.dup2(self._tee.stdin.fileno(), 1)
            os.dup2(self._tee.stdin.fileno(), 2)
            atexit.register(self.close)
            print(f"run transcript: {os.path.abspath(path)}", flush=True)
        except Exception as exc:
            print(f"run transcript unavailable ({exc!r}); terminal only",
                  flush=True)
            self.close()

    @classmethod
    def for_mission(cls, mission: dict) -> "RunTranscript":
        """runs/<YYYYMMDD>-<HHMM> <N>agents <algorithm>.txt

        The algorithm is whatever the UI put in the mission; the solver page
        already computes it for the metrics payload as
        `state.planSource || route-mode`. Older pages do not send it, so a
        run from one of those is labelled unknown-algorithm rather than
        failing to be recorded.
        """
        agents = len(mission.get("commands") or {})
        algo = str(mission.get("algorithm")
                   or (mission.get("schedule") or {}).get("algorithm")
                   or "unknown-algorithm")
        algo = re.sub("[^A-Za-z0-9._-]+", "_", algo).strip("_") or "unknown"
        stamp = datetime.now().strftime("%Y%m%d-%H%M")
        return cls(os.path.join(cls.DIR,
                                f"{stamp} {agents}agents {algo}.txt"))

    def close(self) -> None:
        """Restore the real terminal. Safe to call twice."""
        if self._saved is not None:
            try:
                sys.stdout.flush()
                sys.stderr.flush()
            except Exception:
                pass
            try:
                os.dup2(self._saved[0], 1)
                os.dup2(self._saved[1], 2)
                os.close(self._saved[0])
                os.close(self._saved[1])
            except Exception:
                pass
            self._saved = None
        if self._tee is not None:
            try:
                self._tee.stdin.close()
                self._tee.wait(timeout=5)
            except Exception:
                pass
            self._tee = None


HOLD_LOG_THROTTLE_SEC = 3.0

# ---- proactive drift correction (turn commands only) ----
# The firmware's turn primitives (RIGHT_UNTIL_COLOR/LEFT_UNTIL_COLOR/
# ROTATE_180) are RELATIVE deltas computed from the robot's own wheel-odometry
# yaw, which drifts under wheel slip with no shared reference to true heading.
# Confirmed 2026-07-24: a robot's odometry reported a "complete" turn to what
# it believed was 180 deg while the camera measured ~170 deg physically --
# the turn was internally self-consistent but the odometry reference frame
# had drifted ~10 deg over the route, and this caused an intermittent
# LINE_LOST at the next intersection. Cardinal yaw convention (independently
# confirmed against firmware boot yaw, same sign/zero): S=0, E=90, N=180,
# W=270, positive=CCW.
DRIFT_CARDINALS_DEG = (0.0, 90.0, 180.0, 270.0)
TURN_RELATIVE_DEG = {
    "RIGHT_UNTIL_COLOR": -90.0,
    "LEFT_UNTIL_COLOR": 90.0,
    "ROTATE_180": 180.0,
}
DRIFT_CORRECT_MIN_DEG = 5.0    # snap-worthy: correct anything past this
DRIFT_CORRECT_MAX_DEG = 27.5   # past this, vision is untrusted -- don't guess
DRIFT_VISION_MAX_AGE_SEC = 1.0  # stale vision -- fall back to the plain turn
DRIFT_ODOM_MAX_AGE_SEC = 1.0    # stale odometry sample -- fall back too
# Confirmed 2026-07-24 (fleet run): a correction can be individually "valid"
# (fresh sensors, off-cardinal within the trust band) while still commanding
# an abnormal PHYSICAL ROTATION DISTANCE -- e.g. a LEFT turn's target_odom
# worked out to only ~76 deg of travel instead of the tuned ~90 deg, which
# left the robot short of the workstation-entry line (visually confirmed: a
# "sudden" turn that missed the color sensor). Off-cardinal alone doesn't
# bound this -- a large offset can shrink or stretch the commanded rotation
# without necessarily failing the off-cardinal check. Guard the rotation
# distance itself, not just the endpoint.
DRIFT_ROTATION_TOLERANCE_DEG = 10.0


def _nearest_cardinal_deg(yaw_deg: float) -> float:
    yaw_deg %= 360.0
    return min(DRIFT_CARDINALS_DEG,
               key=lambda c: min(abs(yaw_deg - c), 360.0 - abs(yaw_deg - c)))


def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def drift_corrected_command(cmd: str, vision_yaw_deg: float | None,
                            vision_age_sec: float, odom_yaw_deg: float | None,
                            odom_age_sec: float) -> tuple[str, str | None]:
    """For a turn command, return (possibly-ROTATE_TO command, note).

    The firmware's ROTATE_TO still measures its own progress with get_yaw()
    (odometry) -- it has no camera. So a target computed purely from vision
    would not actually correct anything: telling a robot whose odometry has
    drifted 10 deg from truth to "reach 90 deg" just makes it drive until ITS
    OWN (still-drifted) yaw reads 90, landing at true ~80 or ~100 again. The
    target must instead be expressed in ODOMETRY's frame: figure out the
    desired TRUE heading from vision (nearest cardinal + this turn's relative
    delta), then translate it back into odometry's frame using the current
    vision/odometry offset (both read at the same moment) --
        target_odom = desired_true_heading + (odom_yaw - vision_yaw)
    so the firmware's own convergence loop still lands on the right physical
    heading despite measuring in a drifted frame.

    Falls back to the plain command (note=None) when the command isn't a
    turn, either sensor is stale/missing, or vision yaw is too far from any
    cardinal to trust (robot not roughly grid-aligned yet, or a bad reading)
    -- deliberately conservative: never invent a target from noise."""
    delta = TURN_RELATIVE_DEG.get(cmd)
    if delta is None or vision_yaw_deg is None or odom_yaw_deg is None:
        return cmd, None
    if vision_age_sec > DRIFT_VISION_MAX_AGE_SEC:
        return cmd, None
    if odom_age_sec > DRIFT_ODOM_MAX_AGE_SEC:
        return cmd, None
    nearest = _nearest_cardinal_deg(vision_yaw_deg)
    off = min(abs(vision_yaw_deg - nearest), 360.0 - abs(vision_yaw_deg - nearest))
    if off <= DRIFT_CORRECT_MIN_DEG or off > DRIFT_CORRECT_MAX_DEG:
        return cmd, None
    desired_true_heading = (nearest + delta) % 360.0
    offset = _wrap180(odom_yaw_deg - vision_yaw_deg)
    target_odom = (desired_true_heading + offset) % 360.0
    # The endpoint can look individually trustworthy while still commanding
    # an abnormal physical rotation (e.g. ~76 deg for what should be a ~90
    # deg turn) -- verify the ACTUAL rotation the firmware will execute
    # (target_odom, measured from odom_yaw_deg -- the frame get_yaw() itself
    # moves through) is close to this turn's tuned delta before trusting it.
    actual_rotation = _wrap180(target_odom - odom_yaw_deg)
    if abs(_wrap180(actual_rotation - delta)) > DRIFT_ROTATION_TOLERANCE_DEG:
        return cmd, None
    note = (f"drift correction: vision {vision_yaw_deg:.1f} vs odom "
            f"{odom_yaw_deg:.1f} (offset {offset:+.1f}), {off:.1f} off "
            f"cardinal {nearest:.0f}, rotation {actual_rotation:+.1f} "
            f"(expected {delta:+.0f}) -> ROTATE_TO {target_odom:.1f} (was {cmd})")
    return f"ROTATE_TO {target_odom:.1f}", note

# Grid geometry (label_to_cell & friends) is shared with the advisor so the
# recovery math stays unit-testable without ROS.
from dispatch_advisor import (  # noqa: E402
    STANDING_LINGER_SEC, STANDING_VACATE_IN,
    bridge_commands, corridor_key, entry_edge_nodes, entry_rotation_footprint,
    heading_between_cells, is_workstation_node, label_to_cell,
    movement_segments, node_at_cell, resync_point, workstation_rotation_edges)
from testbed_coordinates import GRID_PITCH_IN as GRID_PITCH_WORLD_IN  # noqa: E402


class VisionMonitor:
    """Subscribes to the /<Name>_vision_pose JSON topics published (via
    rosbridge) by apriltag_localize.py on the camera laptop. Warn-only:
    consumers read positions and warnings, nothing here touches commands."""

    def __init__(self, node: Node, names, qos) -> None:
        self._lock = threading.Lock()
        self._owner_node = node          # for metrics emission only
        self._poses: dict[str, tuple[float, float, float, float]] = {}
        self._poses_in: dict[str, tuple[float, float, float]] = {}
        self._pose_in_t: dict[str, float] = {}
        self._prev_sep: dict[tuple[str, str], tuple[float, float]] = {}
        self._metrics_last: dict[str, float] = {}  # POSE_SAMPLE throttle
        self.calib: dict | None = None
        for name in names:
            node.create_subscription(
                String, f"{name}_vision_pose",
                lambda msg, n=name: self._on_pose(n, msg), qos)
        node.create_subscription(String, "vision_calib", self._on_calib, qos)

    def _on_pose(self, name: str, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            entry = (float(d["grid_x"]), float(d["grid_y"]),
                     float(d["yaw_deg"]), time.time())
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self._poses[name] = entry
            # Inch coordinates kept beside the grid ones purely so a terminal
            # pose can be attached to DEPOT_ARRIVED; the metrics schema stores
            # meters and only this message carries x_in/y_in.
            try:
                self._poses_in[name] = (float(d["x_in"]), float(d["y_in"]),
                                        float(d["yaw_deg"]))
                self._pose_in_t[name] = entry[3]
            except (ValueError, KeyError, TypeError):
                pass
        # Launch timing: first moment this robot's pose reaches node 0.
        # Detected from the pose because the depot-lane hops (D1 -> DE1 -> 0)
        # are implicit_segs folded into the first move and never surface as
        # a PlanItem to_node.
        owner = getattr(self, "_owner_node", None)
        timer = getattr(owner, "_launch", None) if owner is not None else None
        if timer is not None:
            secs = timer.note_pose(name, entry[0], entry[1], entry[3])
            if secs is not None:
                owner._memit("LAUNCH_COMPLETED", robot_id=name,
                             details={"launch_to_node0_sec": round(secs, 2)})
                owner.get_logger().info(
                    f"[{name}] launch: reached node 0 in {secs:.1f}s")
        # Page-4 metrics telemetry (additive): POSE_SAMPLE events feed the
        # distance/separation/vision-availability metrics. Throttled to
        # ~5Hz/robot (the full ~59Hz stream would bloat the events table
        # for no metric gain -- separation is bucketed at 250ms anyway).
        # No-op unless a metrics run is active; never raises into this
        # ROS callback.
        if metrics_emitter is not None and metrics_emitter.enabled:
            try:
                now_ts = entry[3]
                if now_ts - self._metrics_last.get(name, 0.0) >= 0.2:
                    self._metrics_last[name] = now_ts
                    metrics_emitter.emit_pose_in(
                        name, d.get("x_in"), d.get("y_in"),
                        yaw_deg=entry[2])
            except Exception:
                pass

    def _on_calib(self, msg: String) -> None:
        try:
            self.calib = json.loads(msg.data)
        except ValueError:
            pass

    def snapshot(self) -> dict[str, tuple[float, float, float, float]]:
        """{name: (grid_x, grid_y, yaw_deg, age_sec)} for robots seen so far."""
        now = time.time()
        with self._lock:
            return {n: (gx, gy, yaw, now - t)
                    for n, (gx, gy, yaw, t) in self._poses.items()}

    def snapshot_in(self) -> dict[str, tuple[float, float, float, float]]:
        """{name: (x_in, y_in, yaw_deg, age_sec)} for robots seen so far.

        Inches, not grid cells, because the standing-hold check has to measure
        distance to nodes like 'DE3' and label_to_cell() returns None for
        every depot label. nav_node_to_world() resolves those; grid cells
        cannot.
        """
        now = time.time()
        with self._lock:
            ages = {n: now - t for n, (_gx, _gy, _yaw, t) in self._poses.items()}
            return {n: (x, y, yaw, ages.get(n, 1e9))
                    for n, (x, y, yaw) in self._poses_in.items()}

    def closing_pairs_in(self) -> list[tuple[str, str, float, float]]:
        """[(a, b, separation_in, closing_in_per_sec, contact_in), ...].

        contact_in is the heading-aware distance at which THAT pair would
        actually touch -- see contact_distance_in(). A flat threshold called a
        legal pass a collision on 2026-10-06.


        Inches and no lattice filter, unlike proximity_pairs(): the depot lane
        is off the lattice and is where the 2026-09-29 collision happened.
        Closing rate comes from the previous sample of the same pair, so it is
        whatever the vision rate gives -- good enough to tell a pair that is
        driving together from two robots parked side by side.
        """
        now = time.time()
        with self._lock:
            poses = {n: (x, y, yaw, self._pose_in_t.get(n, 0.0))
                     for n, (x, y, yaw) in self._poses_in.items()}
            prev = dict(self._prev_sep)
            out = []
            names = sorted(poses)
            fresh = {}
            for name in names:
                x, y, yaw, t = poses[name]
                if now - t <= COLLISION_MAX_AGE_SEC:
                    fresh[name] = (x, y, yaw, t)
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    if a not in fresh or b not in fresh:
                        continue
                    ax, ay, ayaw, at = fresh[a]
                    bx, by, byaw, bt = fresh[b]
                    sep = math.hypot(ax - bx, ay - by)
                    # Heading-aware: 4.0in squared up, 5.657in at 45 degrees.
                    contact = contact_distance_in(ayaw, byaw,
                                                  bx - ax, by - ay)
                    key = (a, b)
                    was = prev.get(key)
                    stamp = max(at, bt)
                    closing = 0.0
                    if was is None:
                        self._prev_sep[key] = (sep, stamp)
                    else:
                        old_sep, old_t = was
                        dt = stamp - old_t
                        if dt >= COLLISION_MIN_DT_SEC:
                            rate = (old_sep - sep) / dt
                            # A rate no pair of these robots can produce is a
                            # bad fix, not an emergency. Ignore it rather than
                            # let it inflate the stop threshold.
                            if abs(rate) <= COLLISION_MAX_CLOSING_IN_S:
                                closing = rate
                            # Advance the baseline only once it has spanned a
                            # real interval; otherwise KEEP the old sample so
                            # the next comparison still has one to measure
                            # against. Advancing every frame is what made dt
                            # tiny in the first place.
                            self._prev_sep[key] = (sep, stamp)
                    out.append((a, b, sep, closing, contact))
            return out

    # -- motion: velocity, turning, and the footprint guard (2026-10-16) -----
    def _update_motion(self) -> None:
        """Per-robot velocity (in/s) and yaw rate (deg/s) from the pose
        stream. Call with self._lock held. The baseline advances only after
        COLLISION_MIN_DT_SEC, for the same reason as the closing rate above:
        frame-to-frame differences at 50Hz are mostly noise."""
        base = self.__dict__.setdefault("_motion_base", {})
        est = self.__dict__.setdefault("_motion", {})
        for name, (x, y, yaw) in self._poses_in.items():
            t = self._pose_in_t.get(name, 0.0)
            b = base.get(name)
            if b is None or t - b[3] > 2.0:
                base[name] = (x, y, yaw, t)
                est.pop(name, None)
                continue
            dt = t - b[3]
            if dt < COLLISION_MIN_DT_SEC:
                continue
            vx, vy = (x - b[0]) / dt, (y - b[1]) / dt
            dyaw = (yaw - b[2] + 180.0) % 360.0 - 180.0
            # A speed no Alvik can reach is a bad fix, not a velocity.
            if math.hypot(vx, vy) <= COLLISION_MAX_CLOSING_IN_S:
                est[name] = (vx, vy, dyaw / dt, t)
            base[name] = (x, y, yaw, t)

    def claim_rotation(self, name: str,
                       sec: float = None) -> None:
        """Mark `name` as about to turn, before the first frame shows it.

        The rotation gate calls this the moment it lets a turn start, so the
        guard and the corridor gate treat the robot as the disc it is about
        to sweep, rather than waiting ~0.2s for the yaw rate to show it."""
        hold = ROTATION_CLAIM_SEC if sec is None else sec
        with self._lock:
            self.__dict__.setdefault("_rot_claims", {})[name] = time.time() + hold

    def release_rotation(self, name: str) -> None:
        with self._lock:
            self.__dict__.setdefault("_rot_claims", {}).pop(name, None)

    def _turning_locked(self, name: str, now: float) -> bool:
        if self.__dict__.get("_rot_claims", {}).get(name, 0.0) > now:
            return True
        m = self.__dict__.get("_motion", {}).get(name)
        return (m is not None and now - m[3] <= COLLISION_MAX_AGE_SEC
                and abs(m[2]) >= COLLISION_TURNING_DEG_S)

    def is_turning(self, name: str) -> bool:
        with self._lock:
            self._update_motion()
            return self._turning_locked(name, time.time())

    def speed_in(self, name: str) -> float | None:
        """Current speed (in/s) from the pose stream, or None if unknown."""
        now = time.time()
        with self._lock:
            self._update_motion()
            m = self.__dict__.get("_motion", {}).get(name)
        if m is None or now - m[3] > COLLISION_MAX_AGE_SEC:
            return None
        return math.hypot(m[0], m[1])

    def set_leg_target(self, name: str, x_in: float, y_in: float) -> None:
        """Where the leg this robot is driving ends (world inches)."""
        with self._lock:
            self.__dict__.setdefault("_leg_targets", {})[name] = (x_in, y_in)

    def clear_leg_target(self, name: str) -> None:
        with self._lock:
            self.__dict__.setdefault("_leg_targets", {}).pop(name, None)

    def motion_pairs_in(self) -> list[tuple]:
        """motion_pairs_ext() without the pass-by flag."""
        return [p[:6] for p in self.motion_pairs_ext()]

    def motion_pairs_ext(self) -> list[tuple]:
        """[(a, b, sep_in, gap_now_in, min_gap_in, rel_speed_in_s, passby)].

        passby: lane_passby() -- the pair is sliding past in parallel lanes.

        gap_now_in is the real clearance between the two footprints at their
        current headings (footprint_gap_in); min_gap_in is the smallest it
        will get in the next COLLISION_REACTION_SEC if both hold their
        current velocity. A robot that is turning is judged as its swept
        disc. Feeds motion_verdict(); replaces the closing-rate model, which
        stopped legal pass-bys on 2026-10-16.
        """
        now = time.time()
        with self._lock:
            self._update_motion()
            est = dict(self.__dict__.get("_motion", {}))
            fresh = {n: p for n, p in self._poses_in.items()
                     if now - self._pose_in_t.get(n, 0.0) <= COLLISION_MAX_AGE_SEC}
            turning = {n for n in fresh if self._turning_locked(n, now)}
            targets = dict(self.__dict__.get("_leg_targets", {}))

        def vel(n):
            # A turning robot spins in place; it is judged as its swept
            # disc, which already covers every orientation. Its tag sits off
            # the rotation centre, so the pose stream shows the tag swinging
            # at several in/s, and extrapolating that swing as travel stopped
            # three pairs on 2026-10-22 (e.g. Alvik2 squaring up at D2 vs
            # Alvik3 on node 2's lane, "-1.06in within 0.56s at 5.0in/s"),
            # one of which cascaded into the WAIT run's abort.
            if n in turning:
                return 0.0, 0.0
            m = est.get(n)
            if m is None or now - m[3] > COLLISION_MAX_AGE_SEC:
                return 0.0, 0.0
            return m[0], m[1]

        def stop_after(n, v):
            """Seconds until n reaches the end of its leg (+ coast)."""
            tgt = targets.get(n)
            speed = math.hypot(v[0], v[1])
            if tgt is None or speed < 1e-6 or n in turning:
                return math.inf
            x, y, _ = fresh[n]
            dist = math.hypot(tgt[0] - x, tgt[1] - y) + LEG_OVERSHOOT_IN
            return dist / speed

        out = []
        names = sorted(fresh)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                ax, ay, ayaw = fresh[a]
                bx, by, byaw = fresh[b]
                dx, dy = bx - ax, by - ay
                ya = None if a in turning else ayaw
                yb = None if b in turning else byaw
                (avx, avy), (bvx, bvy) = vel(a), vel(b)
                rvx, rvy = bvx - avx, bvy - avy
                sa, sb = stop_after(a, (avx, avy)), stop_after(b, (bvx, bvy))
                if math.isinf(sa) and math.isinf(sb):
                    pred = predicted_min_gap_in(dx, dy, rvx, rvy, ya, yb)
                else:
                    pred = predicted_min_gap_legs_in(
                        (ax, ay), (avx, avy), sa, (bx, by), (bvx, bvy), sb,
                        ya, yb)
                out.append((a, b, math.hypot(dx, dy),
                            footprint_gap_in(dx, dy, ya, yb),
                            pred, math.hypot(rvx, rvy),
                            lane_passby(ya, yb, rvx, rvy, dx, dy)))
        return out

    def proximity_pairs(self) -> list[tuple[str, str, float]]:
        """Pairs of robots with fresh vision, both on the lattice, closer
        than VISION_PROX_CELLS (center-to-center, in cells)."""
        fresh = {n: p for n, p in self.snapshot().items()
                 if p[3] <= VISION_STALE_SEC
                 and GRID_LO <= p[0] <= GRID_HI and GRID_LO <= p[1] <= GRID_HI}
        pairs = []
        for (a, pa), (b, pb) in itertools.combinations(sorted(fresh.items()), 2):
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if d < VISION_PROX_CELLS:
                pairs.append((a, b, d))
        return pairs

    def report_lines(self, names) -> list[str]:
        """Human-readable status: one line of positions + one per warning."""
        snap = self.snapshot()
        if not snap:
            return ["vision: no data yet (apriltag_localize.py --rosbridge "
                    "running on the camera laptop?)"]
        parts = []
        for n in names:
            if n in snap:
                gx, gy, yaw, age = snap[n]
                s = f"{n} grid=({gx:+.2f},{gy:+.2f}) yaw={yaw:+.0f}"
                if age > VISION_STALE_SEC:
                    s += f" [STALE {age:.0f}s]"
                parts.append(s)
            else:
                parts.append(f"{n} [no vision]")
        lines = ["vision: " + "; ".join(parts)]
        for a, b, d in self.proximity_pairs():
            lines.append(f"vision WARNING: {a} and {b} are only "
                         f"{d:.2f} cells apart")
        return lines


@dataclass
class RobotChannel:
    name: str
    commands: list[str]
    index: int = 0
    done_event: threading.Event = field(default_factory=threading.Event)
    ack_event: threading.Event = field(default_factory=threading.Event)
    ack_seen: bool = False
    skip_next_idle: bool = False
    last_status: str = ""
    last_seen: float = 0.0   # wall time of the last status message (heartbeat)
    errored: bool = False
    battery_pct: float | None = None  # latest from <name>_pose (odometry JSON)
    odom_yaw_deg: float | None = None  # latest odometry yaw from <name>_pose
    odom_yaw_seen: float = 0.0        # wall time odom_yaw_deg was last updated
    #: Wall time of the last <name>_pose message of ANY shape. Separate from
    #: odom_yaw_seen, which only advances when the yaw field parses -- this is
    #: about the robot being alive, not about the contents being usable.
    pose_seen: float = 0.0

    @property
    def finished(self) -> bool:
        return self.index >= len(self.commands)

    @property
    def next_command(self) -> str | None:
        return None if self.finished else self.commands[self.index]


class FleetSupervisor(Node):
    #: Set when a mission is accepted. Class-level defaults so every gate can
    #: read them unconditionally -- the rotation gate runs on a worker thread
    #: that may reach it before a mission has ever been configured, and
    #: "color" is the conservative default (it keeps the old turn-item hold
    #: active rather than silently disabling a guard).
    advisor = None
    drive_mode = "color"

    def __init__(self, robots: dict[str, list[str]], use_vision: bool = True,
                 launch_mode: str = "offsets"):
        super().__init__("fleet_supervisor")
        self._qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # the sketch publishes status RELIABLE; subscribe reliable so BUSY
        # acks and done markers can't be dropped on our side
        self._qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._use_vision = use_vision
        # "offsets" (default) or "cascade" -- see _launch_cascade_hold().
        # Set here, in __init__, and NOT inside _on_mission()'s guarded
        # registry block: that try/except deliberately swallows anything it
        # raises so a bad schedule costs the job ids rather than the whole
        # metrics run, which would have turned a missing launch_mode into a
        # silent "job registry not built" on every mission.
        self.launch_mode = launch_mode
        self.vision: VisionMonitor | None = None
        self._channels: dict[str, RobotChannel] = {}
        self._cmd_pubs = {}
        self._status_subbed: set[str] = set()
        # Saved (not discarded) so wait_for_status_match() can poll
        # get_publisher_count() -- see that method's docstring.
        self._status_subs = {}
        self._vision_names: set[str] = set()
        if robots:
            self.configure_robots(robots)

        # ---- mission handoff from the HTML (over rosbridge) ----
        # /fleet_mission: std_msgs/String JSON
        #   {"schedule": <downloadJson payload>, "commands": {"Alvik1": [...]}}
        # /fleet_control: "START" begins the loaded mission, "ABORT" stops it
        #   and broadcasts STOP to every robot.
        # /fleet_status: JSON progress feedback the HTML displays.
        self._pending_mission: dict | None = None
        self._mission_event = threading.Event()
        self._start_event = threading.Event()
        self._abort = False
        self.create_subscription(String, "fleet_mission", self._on_mission,
                                 self._qos_status)
        self.create_subscription(String, "fleet_control", self._on_control,
                                 self._qos_status)
        self._status_pub = self.create_publisher(String, "fleet_status",
                                                 self._qos_status)
        # /fleet_events: discrete run events (vision warnings, errors, recovery,
        # waits, LLM/mission milestones) the HTML accumulates into a Decision
        # Log on the Run & Monitor screen. Each is a small JSON object; the log
        # is reconstructable from these alone.
        self._event_pub = self.create_publisher(String, "fleet_events",
                                                self._qos_status)
        self._event_seq = 0
        # Which supervisor process sent an event: seq restarts at 1 on every
        # restart, so the page dedupes on (boot, seq), not seq alone.
        self._event_boot = int(time.time() * 1000)

        self.get_logger().info(
            "Fleet supervisor ready"
            + (f" for: {', '.join(self._channels)}" if self._channels else " (listen mode)")
            + ("" if use_vision else " (vision disabled)"))

    def configure_robots(self, robots: dict[str, list[str]]) -> None:
        """(Re)configure the robot set — reuses existing pubs/subs on repeat
        missions, resets per-robot state."""
        self._channels = {
            name: RobotChannel(name=name, commands=cmds)
            for name, cmds in robots.items()
        }
        for name in self._channels:
            if name not in self._cmd_pubs:
                self._cmd_pubs[name] = self.create_publisher(
                    String, f"{name}_cmd", self._qos_cmd)
            if name not in self._status_subbed:
                self._status_subs[name] = self.create_subscription(
                    String, f"{name}_status",
                    lambda msg, n=name: self._on_status(n, msg), self._qos_status)
                # Odometry pose carries battery %; we only read it at run
                # start/end but keep the latest value updated cheaply here.
                self.create_subscription(
                    String, f"{name}_pose",
                    lambda msg, n=name: self._on_pose(n, msg), self._qos_cmd)
                self._status_subbed.add(name)
        if self._use_vision:
            new_names = set(self._channels) - self._vision_names
            if self.vision is None:
                self.vision = VisionMonitor(self, self._channels, self._qos_status)
                self._vision_names = set(self._channels)
            elif new_names:
                for name in new_names:
                    self.create_subscription(
                        String, f"{name}_vision_pose",
                        lambda msg, n=name: self.vision._on_pose(n, msg),
                        self._qos_status)
                self._vision_names |= new_names

    def _on_mission(self, msg: String) -> None:
        try:
            mission = json.loads(msg.data)
            assert isinstance(mission.get("schedule"), dict)
            assert isinstance(mission.get("commands"), dict) and mission["commands"]
            drive_mode = str(mission.get("drive_mode", "color")).lower()
            if drive_mode not in ("color", "vision"):
                raise ValueError(
                    f"drive_mode must be 'color' or 'vision', got {drive_mode!r}")
            expected_node_mode = (
                "color_sensor" if drive_mode == "color" else "position_target")
            node_detection_mode = str(mission.get(
                "node_detection_mode", expected_node_mode)).lower()
            if node_detection_mode != expected_node_mode:
                raise ValueError(
                    "node_detection_mode conflicts with drive_mode: "
                    f"{node_detection_mode!r} vs {drive_mode!r}")
            localization_mode = normalize_localization_mode(
                mission.get("localization_mode", "camera_assist"))
            mission["drive_mode"] = drive_mode
            mission["node_detection_mode"] = node_detection_mode
            mission["localization_mode"] = localization_mode
        except (ValueError, AssertionError) as exc:
            self.get_logger().error(f"bad /fleet_mission payload: {exc!r}")
            self.publish_fleet_status("error", note=f"bad mission payload: {exc!r}")
            return
        self._pending_mission = mission
        self._mission_event.set()
        # Job identity and launch timing live on the node so the START
        # handler, the pose bridge and the dispatch loop all see one copy.
        # Built from the schedule, which is the only place that knows what
        # the work IS as opposed to what the robots were told to drive.
        # Guarded as a whole. This sits AHEAD of configure_from_mission()
        # below, so anything raising here would skip metrics configuration
        # entirely and the run would record zero events -- a much worse
        # failure than having no job registry, and a silent one. An odd
        # schedule shape must cost the job ids, not the whole metrics run.
        try:
            self._jobs = JobRegistry(mission.get("schedule"))
            # Pickup readiness on REAL times (see _rpd_hold): bay -> mission
            # time the drop's robot arrived at that workstation.
            rpd = (mission.get("schedule") or {}).get("rpd") or {}
            self._rpd_process_sec = (float(rpd.get("process_sec") or 0.0)
                                     if rpd.get("enabled") else None)
            self._drop_arrival = {}
            # Plans made under the 2026-10-29 drop rule say so; older plans
            # (the DOE's) keep processing from the drop's arrival.
            self._rpd_after_drop = (rpd.get("processing_starts")
                                    == "after the drop dwell")
            self._drop_done = {}
            # 8x8 lattice, the same dimensions the navigator is built with
            # (see the argparse Namespace in the robot channel below).
            self._launch = LaunchTimer(GRID_ROWS, GRID_COLS)
        except Exception as exc:
            self._jobs, self._launch = None, None
            self._rpd_process_sec, self._drop_arrival = None, {}
            self._rpd_after_drop, self._drop_done = False, {}
            self.get_logger().error(
                f"job registry not built ({exc!r}); job metrics and launch "
                "timing are off for this run, dispatch is unaffected")
        if self._jobs:
            self.get_logger().info(
                f"jobs: {len(self._jobs)} from schedule "
                + ", ".join(f"{j.job_id}={j.robot}/ws{j.workstation}"
                            for j in list(self._jobs.jobs.values())[:6])
                + (" ..." if len(self._jobs) > 6 else ""))
        if metrics_emitter is not None:
            try:
                metrics_emitter.configure_from_mission(mission)
            except Exception:
                pass
        self.get_logger().info(
            f"mission received for {sorted(mission['commands'])} "
            f"({sum(len(v) for v in mission['commands'].values())} commands total, "
            f"drive_mode={drive_mode}, localization_mode={localization_mode})")

    #: Seconds of silence on BOTH proof-of-life topics before a robot counts
    #: as offline. _pose runs at 20Hz, so this is 60 missed messages.
    ONLINE_SILENCE_SEC = 3.0

    def robot_online(self, ch) -> bool:
        """Recent evidence of life on EITHER the status topic or the odometry
        pose topic.

        Single definition on purpose: the Run & Monitor markers and the START
        gate used to compute this separately, and the gate still looked only
        at ch.last_seen -- the status topic, which the firmware publishes ONLY
        on events, never as a heartbeat. So an idle robot could read offline in
        one place and online in the other.

        Deliberately NOT vision pose: the overhead camera sees a powered-off
        robot's AprilTag perfectly well, so vision freshness would report a
        dead robot as online, which is the opposite of the point.
        """
        last_life = max(ch.last_seen, ch.pose_seen)
        return last_life > 0 and (time.time() - last_life) < self.ONLINE_SILENCE_SEC

    def ping_robots(self) -> list:
        """Actively poll every robot with GET_STATUS instead of waiting.

        WHY. Passive listening cannot tell "idle and healthy" from "session
        died" -- the firmware only publishes status on events, and a stale
        micro-ROS session leaves the TOPICS registered with nothing behind
        them, so `ros2 topic info` still reports a publisher while nothing
        is ever sent. That is exactly the state seen 2026-09-29: all six
        showing OFFLINE, green LEDs, Publisher count 1, and no message in 5s.
        A green LED only means the robot reached SOME agent once; after an
        agent restart the robots must be power-cycled.

        GET_STATUS forces a reply, so a robot that answers is provably alive
        and a robot that stays silent needs a power cycle. Non-blocking: the
        replies land in _on_status and the markers update on their own.
        """
        asked = sorted(self._channels)
        for name in asked:
            pub = self._cmd_pubs.get(name)
            if pub is None:
                continue
            m = String()
            m.data = "GET_STATUS"
            pub.publish(m)
        silent = [n for n, ch in self._channels.items() if not self.robot_online(ch)]
        self.get_logger().info(
            f"PING sent to {asked}. Currently silent: {silent or 'none'} — "
            "markers update as replies arrive; anything still silent after a "
            "few seconds needs a POWER CYCLE (a green LED only means it "
            "reached an agent once).")
        self._event("mission",
                    f"PING sent to {len(asked)} robot(s)"
                    + (f"; silent: {', '.join(silent)}" if silent else ""),
                    severity="warn" if silent else "info")
        self.publish_fleet_status(
            "ping", note=f"pinged {len(asked)}; silent: {silent or 'none'}")
        return silent

    def _on_control(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd.startswith("PING") or cmd.startswith("RECONNECT"):
            self.ping_robots()
            return
        if cmd.startswith("START"):
            force = "FORCE" in cmd
            offline = [n for n, ch in self._channels.items()
                       if not self.robot_online(ch)]
            if offline and not force:
                # Refusing is deliberate: with no heartbeat there are no BUSY
                # acks, so resends arrive as duplicate motion commands
                # (observed 2026-07-16: a robot consumed 4x FORWARD_UNTIL_BLUE
                # and walked itself to node 0). Note a green robot LED can be
                # stale — it only means the robot connected to SOME agent
                # once; after an agent restart the robots must be
                # power-cycled to reconnect.
                self.get_logger().error(
                    f"START refused — no heartbeat from: {offline}. "
                    "Try PING first (publish 'PING' on /fleet_control, or "
                    "the button on the Run & Monitor page) — it asks every "
                    "robot to answer. Anything still silent needs a POWER "
                    "CYCLE, done AFTER the micro-ROS agent is up: a green LED "
                    "only means the robot reached an agent once, and a stale "
                    "session leaves the topics registered with nothing behind "
                    "them. Override: publish 'START FORCE' on "
                    "/fleet_control.")
                self.publish_fleet_status(
                    "blocked", note=f"START refused — offline: {offline}")
                return
            self._start_event.set()
            self.get_logger().info("fleet_control: START"
                                   + (" (FORCE)" if force and offline else ""))
            self._memit("RUN_STARTED")
            # JOB_RELEASED sets release_elapsed_ms, which is half of flow
            # time -- completion alone would leave every flow statistic null,
            # which is exactly the state the metrics page was in.
            jobs = getattr(self, "_jobs", None)
            if jobs is not None:
                for job in jobs.jobs.values():
                    self._memit("JOB_RELEASED", robot_id=job.robot,
                                job_id=job.job_id,
                                workstation_id=job.workstation,
                                details={"mode": job.mode})
            timer = getattr(self, "_launch", None)
            if timer is not None:
                timer.start(list(self._channels), time.time())
        elif cmd == "ABORT":
            self._abort = True
            self._start_event.set()  # release any waiter so it sees the abort
            for name in self._channels:
                m = String()
                m.data = "STOP"
                self._cmd_pubs[name].publish(m)
            self.get_logger().warning("fleet_control: ABORT — STOP sent to all robots")
            self._event("mission", "ABORT — STOP sent to all robots",
                        severity="error")
            self._memit("RUN_ABORTED", reason="OPERATOR_ABORT")
        else:
            self.get_logger().warning(f"fleet_control: unknown command '{cmd}'")

    def _event(self, kind: str, text: str, robot: str | None = None,
               severity: str = "info", data: dict | None = None) -> None:
        """Emit a discrete Decision-Log event to /fleet_events for the HTML.

        kind: vision | error | recovery | wait | llm | mission (categories the
        Run & Monitor Decision Log filters/colours), or bay (workstation state
        for the page's bay colours, see _bay_event; not shown in the log).
        severity: info|warn|error. `data` rides along as structured fields.
        Fire-and-forget; the terminal logging at each call site is unchanged."""
        self._event_seq += 1
        msg = String()
        msg.data = json.dumps({
            "seq": self._event_seq,
            "boot": getattr(self, "_event_boot", 0),
            "ms": int(time.time() * 1000),
            "kind": kind,
            "severity": severity,
            "robot": robot,
            "text": text,
            **({"data": data} if data else {}),
        })
        try:
            self._event_pub.publish(msg)
        except Exception:
            pass

    def _bay_event(self, bay, state: str, robot: str | None = None) -> None:
        """Tell the page what a workstation is ACTUALLY doing.

        Operator, 2026-10-30: the page coloured bays from the plan's clock,
        so a bay turned PROC before any robot had reached it. These events
        come from the moments the supervisor itself acts on:
          processing  -- the processing clock started (end of the drop's
                         DWELL under the 2026-10-29 drop rule, else on
                         arrival -- the same start _rpd_hold() counts from);
                         carries process_sec so the page can turn it READY.
          picked      -- the robot collecting the part left the workstation.
          delivered   -- a drop with no processing (pickup-and-delivery off).
        """
        data = {"bay": int(bay), "state": state}
        if state == "processing":
            data["process_sec"] = float(self._rpd_process_sec or 0.0)
        self._event("bay", f"bay {bay}: {state}", robot=robot, data=data)

    def _memit(self, event_type: str, details: dict | None = None,
               **fields) -> None:
        """Forward one structured metrics event (see metrics_events.py).
        Fire-and-forget and double-guarded: the emitter is a no-op when the
        mission carried no metrics_run_id, and metrics can never affect
        dispatch -- any exception is swallowed here."""
        if metrics_emitter is None:
            return
        try:
            metrics_emitter.emit(event_type, details=details, **fields)
        except Exception:
            pass

    def advisor_log_sink(self):
        """A log_fn for DispatchAdvisor: prints to the terminal AND forwards
        advisor '[advisor] …' lines into the Decision Log as 'llm' events."""
        def sink(line: str) -> None:
            self.get_logger().info(line)
            if line.startswith("[advisor]"):
                self._event("llm", line.replace("[advisor] ", "", 1),
                            severity="info")
        return sink

    def publish_fleet_status(self, state: str, advisor=None, phase=None,
                             note: str = "", now_sec: float | None = None
                             ) -> None:
        # Decision state for the live VLM experiment: who is waiting on whom,
        # how much slack each robot has against the CURRENT critical path, and
        # what work each has left. Computed once per publish rather than per
        # robot -- slack_by_robot() re-simulates the whole fleet.
        # None (not {}) means "not computed", which the consumer must not read
        # as "nothing is blocked".
        slack: dict = {}
        deadlocks: list = []
        if advisor is not None and now_sec is not None:
            try:
                slack = advisor.slack_by_robot(now_sec)
                deadlocks = advisor.deadlock_cycles()
            except Exception:       # never let telemetry break the publish
                slack, deadlocks = {}, []
        jobs = getattr(self, "_jobs", None)
        robots = {}
        for name, ch in self._channels.items():
            # ONLINE = recent evidence of life on EITHER the status topic or
            # the odometry pose topic.
            #
            # It used to be status alone, with a comment claiming "heartbeats
            # are 1 Hz while idle". There is no such heartbeat: the firmware
            # calls publish_status() only on events (state transitions, IDLE
            # after a command completes), so a robot sitting idle waiting for
            # a mission publishes NOTHING on _status. After 3s it was reported
            # OFFLINE, then flicked back the moment anything made it publish
            # -- the flapping seen on the Run & Monitor page 2026-09-29, with
            # every robot actually connected the whole time.
            #
            # _pose runs continuously at POSE_PERIOD_MS (50ms / 20Hz), so 3s
            # is 60 missed messages -- a real disconnection, not a quiet
            # moment.
            #
            # Deliberately NOT vision pose: the overhead camera still sees a
            # powered-off robot's AprilTag, so vision freshness would report
            # a dead robot as online -- the exact opposite of what this is
            # for.
            online = self.robot_online(ch)
            entry = {"phase": (phase or {}).get(name, "idle"),
                     "online": online,
                     "last_status": ch.last_status, "errored": ch.errored}
            if advisor is not None and name in advisor.robots:
                st = advisor.robots[name]
                entry.update(step=st.next_index, total=len(st.plan),
                             node=str(st.current_node))
                # Where this robot is trying to go next. turn/dwell/noop
                # items carry no to_node of their own, so skip to the first
                # real move -- the same lookahead the dispatch loop does.
                for item in st.plan[st.next_index:]:
                    if item.kind == "move" and item.to_node is not None:
                        entry["next_node"] = str(item.to_node)
                        break
                # The authoritative answer to "is this robot blocked, and by
                # whom" -- the advisor's wait-for graph, the same structure
                # deadlock_cycles() walks. A VLM asked that question is
                # graded against this.
                blocker = advisor.wait_for.get(name)
                if blocker is not None:
                    entry["blocked_by"] = str(blocker)
                if name in slack:
                    entry["slack_sec"] = round(float(slack[name]), 1)
            if jobs is not None:
                try:
                    pend = jobs.pending_for(name)
                    entry["jobs_pending"] = [j.job_id for j in pend]
                    entry["jobs_left"] = len(pend)
                    if pend:
                        entry["next_ws"] = pend[0].workstation
                except Exception:
                    pass
            robots[name] = entry
        msg = String()
        payload = {"state": state, "note": note, "robots": robots,
                   "ms": int(time.time() * 1000)}
        if deadlocks:
            payload["deadlocks"] = deadlocks
        if jobs is not None:
            try:
                done = sum(1 for j in jobs.jobs.values() if j.status == "done")
                orph = sum(1 for j in jobs.jobs.values()
                           if j.status == "orphaned")
                payload["jobs"] = {"total": len(jobs), "done": done,
                                   "orphaned": orph,
                                   "pending": len(jobs) - done - orph}
            except Exception:
                pass
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _launch_cascade_hold(self, advisor, name: str) -> str | None:
        """Physical clearance gate for releasing a robot from its depot slot.

        Returns a hold reason, or None to release. Only meaningful before a
        robot has launched; after that the normal advisor + vision path
        applies.

        This is the physical half of the depot-lane check that
        _vision_hold_reason() skips by design. The advisor's standing
        reservations cover the logical half, but a logical release can run
        ahead of the robot actually clearing the node -- which is the gap
        this closes, and the reason the standalone rig measured clearance
        from the camera rather than trusting node labels.

        Conservative by construction: any missing information (no vision, no
        pose, no coordinates for a node) holds the robot rather than
        releasing it. A stale launch is cheap; a collision in the funnel is
        not.
        """
        if self.vision is None:
            return "cascade launch requires vision (--no-vision given?)"
        request = advisor.immediate_movement_request(name)
        if request is None:
            return None          # nothing queued to move into yet
        rows, cols = advisor.rows, advisor.cols
        snap = self.vision.snapshot()
        # Local import, matching LaunchTimer: these live in
        # camera_grid_navigate and are not module-level here.
        from camera_grid_navigate import (GRID_NODE1_WORLD_IN,
                                          GRID_PITCH_IN)

        # Destination nodes -> the vision frame, via the same conversion
        # LaunchTimer uses (node_to_world is inches; snapshot() is cells).
        targets = []
        for node in request.destination_nodes:
            try:
                wx, wy = nav_node_to_world(node, rows, cols)
            except Exception:
                return f"cascade: no measured coordinates for node {node}"
            targets.append((node,
                            (wx - GRID_NODE1_WORLD_IN[0]) / GRID_PITCH_IN,
                            (wy - GRID_NODE1_WORLD_IN[1]) / GRID_PITCH_IN))
        if not targets:
            return None

        clearance_cells = LAUNCH_CLEARANCE_IN / GRID_PITCH_IN
        for other, pose in snap.items():
            if other == name:
                continue
            ox, oy, _oyaw, age = pose
            if age > VISION_STALE_SEC:
                # A robot we cannot currently see might be standing in the
                # lane. Hold rather than guess.
                return (f"cascade: {other} pose is {age:.1f}s stale "
                        f"(limit {VISION_STALE_SEC:.1f}s)")
            for node, tx, ty in targets:
                if math.hypot(ox - tx, oy - ty) < clearance_cells:
                    return (f"cascade: {other} is within "
                            f"{LAUNCH_CLEARANCE_IN:.1f}in of {node}")
        return None

    def rotation_sweep_blocker(self, name: str):
        """Who is inside `name`'s turning circle RIGHT NOW, or None.

        Pure positional and free of any plan state, so the same measurement
        can gate the REAL rotation (see await_rotation_clearance, called
        from the worker immediately before each turn_to_heading_rotate_rel)
        as well as the plan-level hold below. Returns (robot, separation_in).
        """
        if self.vision is None:
            return None
        snap = self.vision.snapshot_in()
        mine = snap.get(name)
        if mine is None or mine[3] > VISION_STALE_SEC:
            return None          # no fresh fix: fall through to the other gates
        worst = None
        for other, pose in snap.items():
            if other == name or pose[3] > VISION_STALE_SEC:
                continue
            d = math.hypot(mine[0] - pose[0], mine[1] - pose[1])
            if d < ROTATION_SWEEP_IN and (worst is None or d < worst[1]):
                worst = (other, d)
        return worst

    def workstation_lane_blocker(self, name: str, at_node):
        """In a workstation: another robot granted a move along one of the
        bay cell's edges, which a turn here sweeps over. (robot, "a->b") or
        None. Operator, 2026-10-22: drive straight to the node, "then rotate
        after they both get there". See workstation_rotation_edges()."""
        adv = self.advisor
        if adv is None or at_node is None:
            return None
        edges = workstation_rotation_edges(at_node, adv.rows, adv.cols)
        if not edges:
            return None
        try:
            for other in list(adv.robots):
                if other == name:
                    continue
                res = adv.reservations.get(other)
                if res is None:
                    continue
                for seg in res.request.segments:
                    a, b = str(seg[0]), str(seg[1])
                    if a != b and frozenset((a, b)) in edges:
                        return other, f"{a}->{b}"
        except RuntimeError:
            return "(advisor busy)", str(at_node)
        return None

    def entry_inbound_blocker(self, name: str, at_node):
        """At a bay entry: another robot already granted a move ONTO a node
        inside this entry's turning circle. (robot, node) or None.

        The positional check above only sees where robots ARE. On 2026-10-16
        Alvik1 began a 95deg turn at entry 118 while Alvik3, 7.4in away and
        so outside the sweep, was driving to node 14 -- 5.00in from 118 --
        and the two met mid-turn. A turn at an entry takes ~1.5-2s; a robot
        granted a footprint node will be there before it ends. Only entries:
        everywhere else on the table has room to turn (entry_rotation_footprint).
        """
        adv = self.advisor
        if adv is None or at_node is None:
            return None
        foot = entry_rotation_footprint(at_node, adv.rows, adv.cols)
        if not foot:
            return None
        here = str(at_node)
        try:
            for other in list(adv.robots):
                if other == name:
                    continue
                res = adv.reservations.get(other)
                if res is None:
                    continue
                hit = ({str(x) for x in res.request.destination_nodes}
                       & foot) - {here}
                if hit:
                    return other, sorted(hit)[0]
        except RuntimeError:
            # The main loop changed advisor state mid-read. Assume blocked;
            # the next poll, 0.1s later, reads it again.
            return "(advisor busy)", here
        return None

    def rotation_blocker(self, name: str, at_node) -> tuple[str, str] | None:
        """(other, why) if a turn by `name` at `at_node` must wait right now,
        else None. One check of the three gates await_rotation_clearance()
        polls."""
        worst = self.rotation_sweep_blocker(name)
        if worst is not None:
            other, d = worst
            return other, (f"{other} is {d:.2f}in away, a turn sweeps "
                           f"{ROTATION_SWEEP_IN:.3f}in")
        inbound = self.entry_inbound_blocker(name, at_node)
        if inbound is not None:
            other, node = inbound
            return other, (f"{other} is heading onto {node}, inside the "
                           f"turning circle of entry {at_node}")
        lane = self.workstation_lane_blocker(name, at_node)
        if lane is not None:
            other, edge = lane
            return other, (f"{other} is driving {edge} beside workstation "
                           f"{at_node}; turn after it has passed")
        return None

    def straight_trim_clear(self, name: str, nav_yaw_deg: float,
                            length_in: float | None = None) -> bool:
        """May `name` skip a small heading trim and drive on as it points
        now without its footprint coming within COLLISION_PREDICT_MARGIN_IN
        of anyone -- the margin the collision guard itself holds? False
        whenever unknown.

        Worst case, deliberately: no steering at all. The robot travels
        STRAIGHT_TRIM_CHECK_IN along its CURRENT heading (navigator
        convention: dx = sin(yaw), dy = -cos(yaw)), keeping its measured
        orientation since no rotation happens; drive_leg() steering back
        toward the line only widens the gap. A neighbour that is turning
        counts as its full swept disc. `length_in` shortens the check to a
        drive that ends sooner (a 5in hop to a bay entry's flank).
        """
        if self.vision is None:
            return False
        snap = self.vision.snapshot_in()
        mine = snap.get(name)
        if mine is None or mine[3] > VISION_STALE_SEC:
            return False
        mx, my, myaw = mine[0], mine[1], mine[2]
        t = math.radians(nav_yaw_deg)
        ux, uy = math.sin(t), -math.cos(t)
        reach = (STRAIGHT_TRIM_CHECK_IN if length_in is None
                 else min(STRAIGHT_TRIM_CHECK_IN, max(0.0, length_in)))
        steps = int(reach / 0.5)
        for other, p in snap.items():
            if other == name:
                continue
            if p[3] > VISION_STALE_SEC:
                if math.hypot(p[0] - mx, p[1] - my) < 2 * STRAIGHT_TRIM_CHECK_IN:
                    return False     # nearby and not seen right now
                continue
            turning = bool(getattr(self.vision, "is_turning",
                                   lambda _n: False)(other))
            oyaw = None if turning else p[2]
            for k in range(steps + 1):
                px, py = mx + k * 0.5 * ux, my + k * 0.5 * uy
                if footprint_gap_in(p[0] - px, p[1] - py, myaw,
                                    oyaw) < COLLISION_PREDICT_MARGIN_IN:
                    return False
        return True

    def await_rotation_clearance(self, name: str, leg_label: str,
                                 timeout_sec: float | None
                                 = ROTATION_CLEAR_TIMEOUT_SEC,
                                 cancelled=None,
                                 at_node=None,
                                 idle=None) -> tuple[bool, str]:
        """Hold until rotating here is clear of other robots.

        THIS IS THE GATE THAT ACTUALLY GUARDS A ROTATION. Everything before
        2026-10-15 gated `PlanItem.kind == "turn"` -- and in vision mode, the
        mode every recorded run uses, a standalone turn item DOES NOTHING
        (see _execute: "there is nothing for a bare turn/noop item to do
        here"). The rotation is performed at the END of the preceding MOVE,
        by its square-up and turn-toward-next-leg steps. So both the
        symbolic gate (advisor.entry_rotation_footprint) and the positional
        one (_rotation_clearance_hold) were guarding a no-op that runs AFTER
        the rotation they meant to prevent.

        Measured on 2026-10-15 run 5: Alvik3's rotation finished at t=88.2s
        (yaw +14.9 -> +91.9, then sd 0.17deg from t=95s, i.e. stationary).
        The sweep-guard refusals began at t~91s and repeated every ~3s for
        115s. The guard spent the whole run refusing to let it START a
        rotation it had already FINISHED, which is why the livelock could
        never break: the held item was a no-op, so nothing it did could
        satisfy the gate.

        Waiting, not failing: a rotation that is unsafe now is usually safe
        a second later once the neighbour moves on. The wait is registered
        with the advisor so deadlock_cycles() can see a genuine standoff.

        timeout_sec=None holds with no deadline. Use it for any rotation
        AFTER the robot has arrived somewhere: failing there re-dispatches
        the whole leg from its start node, which the robot has already left,
        and the vision gate then reads it as off-route and excludes it
        (2026-10-16, Alvik4 at node 18). The backstops for an endless hold
        are the deadlock abort, once the standoff closes a cycle, and the
        per-leg timeout, which calls the worker's abort() and so `cancelled`.

        `cancelled` is polled like the operator abort, so an excluded or
        aborted robot's worker thread stops waiting instead of turning
        later, after everyone has stopped watching it.
        """
        deadline = (None if timeout_sec is None
                    else time.monotonic() + timeout_sec)
        blocked_by = None
        while True:
            other, why = self.rotation_blocker(name, at_node) or (None, "")
            if other is None:
                if blocked_by is not None and self.advisor is not None:
                    self.advisor.clear_vision_block(name)
                # Tell the guard and the corridor gate now, before the first
                # frame of the turn shows a yaw rate.
                claim = getattr(self.vision, "claim_rotation", None)
                if claim is not None:
                    claim(name)
                return True, "clear"
            if blocked_by != other:
                blocked_by = other
                self.get_logger().info(
                    f"[{name}] {leg_label}: holding rotation -- {why}")
                self._event("wait", f"{name} rotation hold: {why}",
                            robot=name)
            if self.advisor is not None and not other.startswith("("):
                # Same registration the corridor gate uses, so a mutual
                # rotation standoff is visible to the existing resolver
                # instead of being an invisible wait.
                self.advisor.note_vision_block(name, other)
            # An operator ABORT must not have to wait out the deadline. This
            # runs on a worker thread, so without the check the fleet would
            # keep holding for up to 20 s after STOP was pressed.
            if (getattr(self, "_abort", False)
                    or (cancelled is not None and cancelled())):
                if self.advisor is not None:
                    self.advisor.clear_vision_block(name)
                return False, "aborted while holding for rotation clearance"
            if deadline is not None and time.monotonic() >= deadline:
                if self.advisor is not None:
                    self.advisor.clear_vision_block(name)
                return False, f"rotation blocked for {timeout_sec:g}s: {why}"
            # `idle`, not sleep, when the caller has a node to keep alive:
            # a navigator only reads its odometry and pose topics while its
            # worker spins it, and a turn refuses odometry older than 0.75s.
            # Sleeping here made EVERY held turn fail the moment the hold
            # ended (2026-10-16: D2, D3, entry 137 -- "no fresh encoder +
            # camera-assisted odometry yaw").
            (idle or time.sleep)(ROTATION_CLEAR_POLL_SEC)

    def _rotation_clearance_hold(self, advisor, name: str) -> str | None:
        """Refuse to start a turn that would sweep into another robot.

        MEASURED, not modelled: compares live camera positions, so it needs
        no assumption about where a robot "should" be. A robot drifted off
        its node is judged on where it actually is.

        SCOPE, NARROWED 2026-10-15. This now applies only where a standalone
        turn item really does turn the robot -- colour-sensor mode. In vision
        mode the item is a no-op and the real rotation is gated at its own
        call site by await_rotation_clearance(); holding the no-op there
        bought nothing and produced the 115-second livelock described in
        that method. See drive_mode below.

        WHY A TURN NEEDS ITS OWN GATE. Until 2026-09-30 a turn was
        dispatched with "movement gate deferred" and checked against
        nothing: it is not a move, so it took no reservation, and a
        stationary robot has no closing speed for the emergency brake to
        act on. A robot could therefore rotate into a neighbour with every
        guard in the system looking straight at it. That is what happened
        to Alvik3 and Alvik4.
        """
        if self.drive_mode == "vision":
            return None
        st = advisor.robots.get(name)
        if st is None or st.next_index >= len(st.plan):
            return None
        if st.plan[st.next_index].kind != "turn":
            return None
        if self.vision is None:
            return None
        snap = self.vision.snapshot_in()
        mine = snap.get(name)
        if mine is None or mine[3] > VISION_STALE_SEC:
            return None          # no fresh fix: fall through to the other gates
        worst = None
        for other, pose in snap.items():
            if other == name or pose[3] > VISION_STALE_SEC:
                continue
            d = math.hypot(mine[0] - pose[0], mine[1] - pose[1])
            if d < ROTATION_SWEEP_IN and (worst is None or d < worst[1]):
                worst = (other, d)
        if worst is None:
            return None
        other, d = worst
        # Register the hold in the wait-for graph, exactly as the corridor
        # gate above does. THE SAME BUG, ONE GUARD LATER: the corridor hold
        # was made visible to deadlock_cycles() on 2026-09-29 because a hold
        # that is not a reservation left the detector looking at an empty
        # graph; this guard was added on 2026-09-30 and never got it.
        #
        # Measured cost, 2026-10-15 run 5: Alvik1 hit WHEEL_CMD_TIMEOUT, went
        # IDLE and sat motionless on node 18 for 115 s. Alvik3 refused to
        # rotate at entry 122 about every 3 s for the whole of it, Alvik5,
        # Alvik2, Alvik4 and Alvik6 queued behind in turn, and the run
        # reached a five-robot standstill with deadlocks_detected = 0.
        # Nothing was wrong with the refusal -- rotating would have struck
        # Alvik1. What was missing is that no resolver could see the standoff.
        #
        # This does NOT weaken the guard: the turn is still refused. It only
        # lets the EXISTING detector and sidestep resolver act on it.
        advisor.note_vision_block(name, other)
        return (f"rotation clearance: {other} is {d:.2f}in away, a turn "
                f"sweeps {ROTATION_SWEEP_IN:.3f}in "
                f"(short by {ROTATION_SWEEP_IN - d:.2f}in)")

    def _drives_straight(self, advisor, workers, robot: str, node,
                         reverse: bool = False) -> bool:
        """Could `robot` leave for `node` with NO rotation? It must already
        point there (within STRAIGHT_TRIM_SKIP_MAX_DEG, which the yield's
        drive steers out) and the straight drive there must keep the
        guard's margin from everyone. False whenever unknown.

        reverse=True: its TAIL must point there, and the drive is checked
        backwards (camera_grid_navigate.drive_leg(reverse=True)).

        The deadlock resolver asks this for a robot with no room to turn
        where it stands (see DispatchAdvisor._yield_move)."""
        worker = (workers or {}).get(robot)
        st = advisor.robots.get(robot)
        if worker is None or st is None or st.current_node is None:
            return False
        pose = worker.nav.control_pose()
        if pose is None:
            return False
        try:
            from camera_grid_navigate import heading_between
            x0, y0 = nav_node_to_world(st.current_node, advisor.rows,
                                       advisor.cols)
            x1, y1 = nav_node_to_world(node, advisor.rows, advisor.cols)
        except (ValueError, KeyError, TypeError, ImportError):
            return False
        if (x0, y0) == (x1, y1):
            return False
        heading = heading_between(x0, y0, x1, y1)
        travel = (pose.yaw + 180.0) % 360.0 if reverse else pose.yaw
        err = abs((heading - travel + 180.0) % 360.0 - 180.0)
        if err > STRAIGHT_TRIM_SKIP_MAX_DEG:
            return False
        return self.straight_trim_clear(robot, travel,
                                        length_in=math.hypot(x1 - x0, y1 - y0))

    def _on_planned_node(self, advisor, robot: str) -> bool:
        """Is `robot` physically within YIELD_START_TOL_IN of the node the
        advisor has it on? False when that cannot be measured."""
        st = advisor.robots.get(robot)
        if st is None or self.vision is None:
            return False
        p = self.vision.snapshot_in().get(robot)
        if p is None or p[3] > VISION_STALE_SEC:
            return False
        try:
            nx, ny = nav_node_to_world(st.current_node, advisor.rows,
                                       advisor.cols)
        except (ValueError, KeyError, TypeError):
            return False
        return math.hypot(p[0] - nx, p[1] - ny) <= YIELD_START_TOL_IN

    def _path_clears(self, other: str, frm, to, rows: int, cols: int,
                     node=None, margin: float | None = None) -> bool:
        """Would this move's footprint clear `other`'s by
        CORRIDOR_FOOTPRINT_MARGIN_IN? Judged at `node` if given, else along
        the whole straight segment frm->to.

        THE FIFTH ENTRY-NODE FALSE HOLD (2026-10-16). Every vision hold below
        used a flat radius of 0.6 cells, but this table puts a bay entry, its
        two neighbouring lattice nodes and its workstation all exactly 0.50
        cells from the next lane over. Square-on, that is 1.0in of real
        clearance (operator: "the robots would still have an inch in between
        them"), yet the radius called it blocked. Alvik3 was refused the east
        leg 6->14 because Alvik1 stood at entry 118, and Alvik1 was refused
        its own bay because Alvik3 stood on that leg -- a deadlock made of
        two legal positions.

        So: if the other robot is inside the radius, measure footprints --
        this robot at the segment's heading, the other at its measured yaw,
        or as its swept disc if it is turning (a robot mid-turn really does
        block, and is still held). False whenever anything is unknown.
        """
        if self.vision is None:
            return False
        p = self.vision.snapshot_in().get(other)
        if p is None or p[3] > VISION_STALE_SEC:
            return False
        try:
            from camera_grid_navigate import heading_between
            ax, ay = nav_node_to_world(frm, rows, cols)
            bx, by = nav_node_to_world(to, rows, cols)
            points = ([nav_node_to_world(node, rows, cols)] if node is not None
                      else [(ax + (bx - ax) * k / 10.0, ay + (by - ay) * k / 10.0)
                            for k in range(11)])
        except (ValueError, KeyError, TypeError):
            return False
        if (ax, ay) == (bx, by):
            return False
        heading = heading_between(ax, ay, bx, by)
        turning = bool(getattr(self.vision, "is_turning",
                               lambda _n: False)(other))
        other_yaw = None if turning else p[2]
        gap = min(footprint_gap_in(p[0] - px, p[1] - py, heading, other_yaw)
                  for px, py in points)
        return gap >= (CORRIDOR_FOOTPRINT_MARGIN_IN if margin is None else margin)

    def _ws_bay(self, node) -> int | None:
        """Solver bay number of a WORKSTATION node label, else None."""
        try:
            n = int(node)
        except (TypeError, ValueError):
            return None
        lattice = GRID_ROWS * GRID_COLS
        bays = (GRID_ROWS - 1) * (GRID_COLS - 1)
        return n - lattice if lattice < n <= lattice + bays else None

    def _entry_bay(self, node) -> int | None:
        """Solver bay number of a bay ENTRY node label, else None."""
        try:
            n = int(node)
        except (TypeError, ValueError):
            return None
        lattice = GRID_ROWS * GRID_COLS
        bays = (GRID_ROWS - 1) * (GRID_COLS - 1)
        return n - lattice - bays if lattice + bays < n <= lattice + 2 * bays else None

    def _rpd_hold(self, name: str, item, now: float) -> str | None:
        """Hold a move that would break PICKUP READINESS on real times.

        WORKSTATION WAITS FOLLOW THE ROBOTS, NOT THE PLAN'S CLOCK (operator,
        2026-10-21): "the time spent waiting at a workstation must be done
        when the robot is at the workstation". A pick-up of a part another
        robot dropped was a fixed planned DWELL, so a late dropper meant the
        collector left before the part was ready, and an early one meant
        waiting for nothing. The planner's rule is: ready = the dropper's
        arrival at the bay + processing time, and the collector may not
        leave before it. Same rule here, on measured times. Plans made under
        the 2026-10-29 drop rule (rpd.processing_starts == "after the drop
        dwell") start the clock when the drop's DWELL ends instead:

          * WAIT IN THE PREVIOUS BAY. A robot whose NEXT visit is a pick-up
            stays in the workstation it is leaving until that part has been
            dropped. The first version held it "outside the bay" instead --
            on 2026-10-21 that parked Alvik1 on lattice node 19, the only
            way for Alvik5 to reach bay 10 and drop the part Alvik1 was
            waiting for: a deadlock the gate itself created. A bay is a dead
            end nobody else needs until the part there is ready.
          * Fallback, only for a pick-up with no previous bay (first stop):
            no move onto the bay's entry or workstation before the drop.
          * LEAVING the pick-up bay: not before drop arrival + processing
            time has actually elapsed.

        Service visits (drop, wait, collect by the same robot) need nothing:
        their dwell already starts on arrival. Planner-independent: applies
        to every traffic method alike.
        """
        jobs = getattr(self, "_jobs", None)
        proc = getattr(self, "_rpd_process_sec", None)
        if jobs is None or proc is None or item.kind != "move":
            return None
        job = jobs.next_pending(name)
        if job is None:
            return None
        drops = getattr(self, "_drop_arrival", {})
        from_ws = self._ws_bay(item.from_node)
        to_ws, to_entry = self._ws_bay(item.to_node), self._entry_bay(item.to_node)
        if from_ws is not None and int(job.workstation) == from_ws:
            # Leaving the bay of the current visit.
            if job.mode == "pick":
                started = (getattr(self, "_drop_done", {}).get(from_ws)
                           if getattr(self, "_rpd_after_drop", False)
                           else drops.get(from_ws))
                ready = None if started is None else started + proc
                if ready is None or now < ready:
                    left = "" if ready is None else f" in {ready - now:.1f}s"
                    return (f"pick-up at bay {from_ws}: part ready{left} "
                            f"(processing {proc:.0f}s from the measured drop)")
            after = jobs.pending_after(name)
            if after is not None and after.mode == "pick" and \
                    drops.get(int(after.workstation)) is None:
                return (f"next stop is a pick-up at bay {after.workstation}: "
                        f"waiting in bay {from_ws} until that part has been "
                        "dropped")
            return None
        if job.mode == "pick" and drops.get(int(job.workstation)) is None:
            bay = int(job.workstation)
            if to_ws == bay or (to_entry == bay and from_ws != bay):
                return (f"pick-up at bay {bay}: the part has not been "
                        "dropped there yet -- waiting outside the bay")
        return None

    def _vision_hold_reason(self, advisor, name: str) -> str | None:
        """Validate an advisor-approved movement against visual state.

        The advisor's space-time simulation already handles logical node
        occupancy, including a robot that will clear a node before this move
        arrives. Pending edges survive a temporary camera dropout. Fresh
        vision adds edge-corridor projection for unexpected physical blockers
        where schedule coordinates exist. DE#/D# stay graph resources because
        the schedule export does not provide coordinates for those labels.
        """
        request = advisor.immediate_movement_request(name)
        if request is None:
            return None
        st = advisor.robots.get(name)
        if st is None:
            return None
        rows, cols = advisor.rows, advisor.cols

        for other, other_state in advisor.robots.items():
            if other == name:
                continue
            for frm, to in other_state.pending_edges:
                # Corridor key, so another robot leaving a bay (e.g. edge
                # 122<>19) is recognised as being on the grid lane 18<>19 that
                # this move wants — the entry-node-on-a-grid-edge case.
                if corridor_key(frm, to, rows, cols) in request.edge_keys:
                    return (f"logical: {other} is moving on requested edge "
                            f"{frm}<>{to}")

        if self.vision is None:
            return None
        snap = self.vision.snapshot()
        mine = snap.get(name)
        my_cell = label_to_cell(st.current_node, rows, cols)
        if mine and mine[3] <= VISION_STALE_SEC and my_cell is not None:
            off = math.hypot(mine[0] - my_cell[0], mine[1] - my_cell[1])
            if off > VISION_OFFROUTE_CELLS:
                return (f"vision: off-route — measured ({mine[0]:.2f},{mine[1]:.2f}) "
                        f"vs expected node {st.current_node} ({off:.2f} cells)")

        # Corridor gate (2026-07-21 workstation-lane rule): a robot may enter a
        # corridor node only after the lead robot has PHYSICALLY advanced off
        # it. Unlike the advisor's space-time model (which mispredicts when
        # robots run slower than calibration), this reads the camera: if another
        # robot is physically parked on a node this move would occupy, hold.
        # When leaving a workstation/depot dead-end, extend to the next two
        # nodes (the lane a lead robot must fully clear before we exit into it).
        corridor_nodes = set(advisor.request_corridor_nodes(request))
        all_segments = list(request.segments)
        if advisor.is_workstation_or_depot_exit(name):
            # Two-node lookahead: also require the lane BEYOND this move to be
            # clear, so a robot exiting a dead-end waits for a passer to fully
            # leave the shared lane (rule #2, 2026-07-21).
            nxt = advisor.next_move_request(name)
            if nxt is not None:
                corridor_nodes |= advisor.request_corridor_nodes(nxt)
                all_segments += list(nxt.segments)

        def segment_through(lbl):
            """The segment of this move that passes through node `lbl` --
            as an endpoint, or as the bay entry lying mid-edge on it."""
            for frm, to, _d in all_segments:
                if str(lbl) in (str(frm), str(to)):
                    return frm, to
                if str(advisor._entry_on_edge(frm, to)) == str(lbl):
                    return frm, to
            return None
        # The DEPOT region (node 0, node 1, and the parking lane below the grid)
        # is a dense funnel where robots legitimately sit ~0.5-0.8 cells apart —
        # the whole lane is only 1.35 cells long. Applying the wide grid margin
        # there flags every parked robot as "occupying" node 0/1 and deadlocks
        # departures (2026-07-21). The depot funnel is coordinated by the
        # model's shared-depot-entrance reservation + the emergency brake, so the
        # corridor gate skips depot-region targets and only guards grid nodes.
        gate_cells = [(lbl, label_to_cell(lbl, rows, cols))
                      for lbl in corridor_nodes
                      if not _is_depot_region_label(lbl)]

        def off_lane(other_name, pose):
            """A robot that does NOT obstruct the through-lane, so the vision
            holds below must ignore it: (a) parked/funneling in the depot lane
            (grid_y below the grid), or (b) SERVICING a workstation — parked in
            the dead-end bay spur off the lane, even though its camera position
            reads ~0.35 cells from its own entry node. Such robots' departures
            are gated separately (is_workstation_or_depot_exit + lookahead), so
            they can't pull out into a passer."""
            if pose[1] < DEPOT_LANE_Y_CUTOFF:
                return True
            # MEASURED position first. A robot physically in a bay spur is off
            # the lane whatever the plan believes -- see in_bay_spur() for the
            # 2026-09-29 deadlock this prevents, where a position resync moved
            # current_node off the workstation and the robot started blocking
            # the edge it was parked beside.
            if in_bay_spur(pose[0], pose[1]):
                return True
            ost = advisor.robots.get(other_name)
            return ost is not None and is_workstation_node(
                ost.current_node, rows, cols)

        for other, pose in snap.items():
            if other == name or pose[3] > VISION_STALE_SEC:
                continue
            if off_lane(other, pose):
                # Off the lane by position -- but a robot that came to rest
                # too deep in its bay, or out of its slot, can still reach
                # into an edge this move drives. 2026-10-23: Alvik5 stopped
                # 1.38in deep in workstation 81 and Alvik2 was granted the row
                # behind it (20->19); the guard found the overlap only once
                # contact was made. Judge by real footprints along this
                # move. A robot parked where it should be leaves ~1in and
                # never holds anyone; this fires only past ~0.8in deep.
                # (Depot robots are left to the depot-lane rule: the slots sit
                # ~6in below row 1, so they cannot reach a grid edge.)
                if pose[1] >= DEPOT_LANE_Y_CUTOFF:
                    for frm, to, _d in all_segments:
                        if str(frm) == str(to):
                            continue
                        if not self._path_clears(other, frm, to, rows, cols,
                                                 margin=PARKED_INTRUSION_MARGIN_IN):
                            advisor.note_vision_block(name, other)
                            return (f"vision corridor: {other} is parked too "
                                    f"close to {frm}->{to} for this robot to "
                                    "pass; waiting for it to leave")
                continue
            for lbl, cell in gate_cells:
                if cell is None or str(lbl) == str(st.current_node):
                    continue
                # Use the wider CLEAR margin, not just the block radius: the
                # lead robot must have physically pulled away (finished rotating
                # in place and started moving) before this robot may enter the
                # node — a robot mid-rotation still occupies its node.
                if math.hypot(pose[0] - cell[0],
                              pose[1] - cell[1]) <= VISION_CORRIDOR_CLEAR_CELLS:
                    # Inside the radius is not the same as in the way: a
                    # robot at the next lane over is 0.50 cells off with 1in
                    # to spare. Judge by footprints. See _path_clears().
                    seg = segment_through(lbl)
                    if seg is not None and self._path_clears(
                            other, seg[0], seg[1], rows, cols, node=lbl):
                        continue
                    # Feed the wait-for graph. This gate is SYMMETRIC and has
                    # no tie-breaker, so two robots converging on one node
                    # hold for each other indefinitely -- seen live on
                    # 2026-09-29, Alvik2 and Alvik5 alternating on node 19
                    # for 70s until the run was aborted by hand. A corridor
                    # hold is not a reservation, so deadlock_cycles() saw an
                    # empty graph for the pair. Recording it here lets the
                    # EXISTING detector and sidestep resolver break it
                    # instead of adding a second, unproven recovery path.
                    advisor.note_vision_block(name, other)
                    return (f"vision corridor: {other} still on/near node {lbl} "
                            f"in my path (rotating or not yet clear)")

        # Nothing on the corridor: drop any standing camera block for this
        # robot so a cleared standoff cannot linger in the graph and get
        # resolved after the fact.
        advisor.clear_vision_block(name)

        def distance_to_segment(px, py, a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            denom = dx * dx + dy * dy
            if denom <= 1e-12:
                return math.hypot(px - a[0], py - a[1]), 0.0
            along = ((px - a[0]) * dx + (py - a[1]) * dy) / denom
            clamped = min(1.0, max(0.0, along))
            nearest = (a[0] + clamped * dx, a[1] + clamped * dy)
            return math.hypot(px - nearest[0], py - nearest[1]), along

        for other, pose in snap.items():
            if other == name or pose[3] > VISION_STALE_SEC:
                continue
            # Off-lane robots (depot-parked or servicing a workstation) don't
            # obstruct a passer even though they project close to the entry-node
            # edge (2026-07-21: Alvik4 servicing bay 73 projected 0.48 cells onto
            # Alvik1's 18->19 edge and wrongly held it). Same rule as the gate.
            if off_lane(other, pose):
                continue
            other_state = advisor.robots.get(other)
            other_cell = (
                label_to_cell(other_state.current_node, rows, cols)
                if other_state is not None else None
            )
            expected_target_occupant = (
                other_state is not None
                and str(other_state.current_node) in request.destination_nodes
                and other_cell is not None
                and math.hypot(
                    pose[0] - other_cell[0], pose[1] - other_cell[1]
                ) <= VISION_TARGET_BLOCK_CELLS
            )
            if expected_target_occupant:
                # This robot is where the schedule says it is. The advisor
                # granted the move only because its predicted departure clears
                # the node before the requester arrives.
                continue

            # A robot standing ON a bay entry blocks the corridor that entry
            # lies on -- that is exactly why off_lane() does not exempt it.
            # It does NOT block the edges running PERPENDICULAR out of its
            # neighbouring lattice nodes, and the geometry makes that a
            # guaranteed false positive if left alone: an entry sits 5.00in
            # from its neighbour, which is 0.50 cells, against a
            # VISION_EDGE_CORRIDOR_CELLS of 0.5. Every perpendicular edge
            # therefore projects at ~0.49 cells and is refused.
            #
            # Deadlocked a run on 2026-10-06: Alvik4 held at entry 122 waiting
            # to rotate until Alvik5 left node 19, while Alvik5 was refused
            # edge 19->11 because Alvik4 "projects 0.49 cells" from it. Neither
            # could move; the pair alternated the same two messages for 45s
            # until the run was aborted by hand.
            other_entry_corridor = None
            if other_state is not None:
                _pair = entry_edge_nodes(other_state.current_node, rows, cols)
                if _pair is not None:
                    other_entry_corridor = corridor_key(
                        _pair[0], _pair[1], rows, cols)

            for frm, to, _duration in request.segments:
                if (other_entry_corridor is not None
                        and corridor_key(frm, to, rows, cols)
                        != other_entry_corridor):
                    continue
                start = label_to_cell(frm, rows, cols)
                end = label_to_cell(to, rows, cols)
                if start is None or end is None:
                    continue
                distance, along = distance_to_segment(
                    pose[0], pose[1], start, end)
                if (distance < VISION_EDGE_CORRIDOR_CELLS
                        and -0.05 <= along <= 1.05
                        and not self._path_clears(other, frm, to, rows, cols)):
                    # Feed the wait-for graph. Without this the edge gate is
                    # invisible to deadlock_cycles(): on 2026-10-06 the
                    # rotation gate recorded Alvik4 -> Alvik5 while this gate
                    # recorded nothing back, so the cycle never closed and
                    # nothing ever fired.
                    advisor.note_vision_block(name, other)
                    return (f"vision: {other} projects {distance:.2f} cells "
                            f"from requested edge {frm}->{to}")

            # Circular proximity remains only a final conservative safeguard.
            for target in request.destination_nodes:
                cell = label_to_cell(target, rows, cols)
                if cell is None:
                    continue
                distance = math.hypot(pose[0] - cell[0], pose[1] - cell[1])
                seg = segment_through(target)
                if (distance < VISION_TARGET_BLOCK_CELLS
                        and not (seg is not None and self._path_clears(
                            other, seg[0], seg[1], rows, cols, node=target))):
                    return (f"vision: {other} is {distance:.2f} cells from "
                            f"requested node {target}")
        return None

    def _print_vision(self) -> None:
        if self.vision is None:
            return
        for line in self.vision.report_lines(list(self._channels)):
            if "WARNING" in line:
                self.get_logger().warning(line)
            else:
                self.get_logger().info(line)

    def _on_status(self, name: str, msg: String) -> None:
        # A robot outside the CURRENT mission still has its status
        # subscription (subscriptions are never torn down between missions).
        # After a 4-robot mission, Alvik5/6 powering on or reconnecting
        # publish IDLE here; indexing _channels raised KeyError, which killed
        # the ROS spin thread, and every later mission/START went unheard
        # until a restart (DOE, 2026-10-23..27). Not our robot right now:
        # ignore it. It is picked up again when a mission names it.
        ch = self._channels.get(name)
        if ch is None:
            return
        text = msg.data.strip()
        # ROBOT_STATUS capture (2026-09-22). Every string the firmware
        # publishes is recorded against the run, so a breakdown can be
        # reconstructed afterwards from what the robot actually said rather
        # than from what the supervisor chose to log. This is the stream that
        # carried "ROTATE_REL DIAG ..." -- which existed for a month before
        # anyone could read it, because raw statuses only ever reached a
        # --verbose console and were never stored anywhere.
        #
        # Deduplicated on change: the firmware republishes a steady state
        # (IDLE, BUSY ...) repeatedly, and only transitions carry
        # information. Volume is then a few hundred rows per run against
        # ~390k POSE_SAMPLEs, so this is free by comparison.
        if text and text != getattr(ch, "last_status", None):
            self._memit("ROBOT_STATUS", robot_id=name, reason=text)
        ch.last_status = text
        ch.last_seen = time.time()

        if text.startswith("ERROR"):
            self.get_logger().error(f"[{name}] reported error: {text}")
            self._event("error", f"reported {text}", robot=name,
                        severity="error")
            ch.errored = True
            ch.done_event.set()
            return

        # LINE_GAP <ms>: telemetry only (how long the line was lost on the
        # last sticker/marker crossing), published by followLine() whenever a
        # gap exceeds LINE_GAP_REPORT_MS. Not part of the ack/done handshake —
        # surfaced unconditionally so gaps approaching LOST_LINE_FAILSAFE_MS
        # (400ms) are visible instead of silently falling through the status
        # handler (previously not logged at all).
        if text.startswith("LINE_GAP"):
            self.get_logger().info(f"[{name}] {text}")
            self._event("telemetry", text, robot=name, severity="info")
            return

        # The firmware brakes at a 300 ms wheel-command receive gap but now
        # allows a healthy best-effort stream to recover before the 1.0 s
        # terminal timeout. Surface every recovery so transport jitter stays
        # measurable instead of becoming an invisible mid-leg pause.
        if text.startswith("WHEEL_CMD_STALL_RECOVERED"):
            self.get_logger().warning(f"[{name}] {text}")
            self._event("telemetry", text, robot=name, severity="warning")
            return

        # "BUSY <cmd>" is the robot confirming it received our command.
        if text.startswith("BUSY"):
            ch.ack_seen = True
            ch.ack_event.set()
            return

        # Discard exactly one bare "IDLE" right after sending a command —
        # it can be the tail of the PREVIOUS command's double-publish,
        # arriving just after the new command went out.
        if ch.skip_next_idle and text == "IDLE":
            ch.skip_next_idle = False
            return
        ch.skip_next_idle = False

        # Until the robot has ACKed the current command, any "IDLE" is just
        # its 1 Hz idle heartbeat — counting it as done would advance the
        # route past a command the robot never received.
        if not ch.ack_seen:
            return

        if any(text == m or text.startswith(m + " ") for m in DONE_MARKERS):
            ch.done_event.set()

    def _on_pose(self, name: str, msg: String) -> None:
        """Odometry pose JSON {x,y,yaw,battery,ms}. Battery is read at run
        start/end. yaw is also kept fresh (no per-update logging, it's noisy)
        — needed alongside vision yaw for drift_corrected_command(), which
        must know the CURRENT odometry/vision offset, not just vision alone:
        the firmware's ROTATE_TO still measures its own progress with
        get_yaw() (odometry), so a target computed purely in vision's frame
        would not actually correct anything."""
        ch = self._channels.get(name)
        if ch is None:
            return
        try:
            pose = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # A well-formed pose message is proof the robot is powered and still
        # attached to the micro-ROS agent, whatever is in it.
        ch.pose_seen = time.time()
        battery = pose.get("battery")
        if isinstance(battery, (int, float)):
            ch.battery_pct = float(battery)
        yaw = pose.get("yaw")
        if isinstance(yaw, (int, float)):
            ch.odom_yaw_deg = float(yaw)
            ch.odom_yaw_seen = time.time()

    def _battery_report(self) -> str:
        """One-line battery snapshot across the fleet, for run start/end."""
        parts = []
        for name in sorted(self._channels):
            pct = self._channels[name].battery_pct
            parts.append(f"{name} {pct:.0f}%" if pct is not None
                         else f"{name} n/a")
        return "; ".join(parts) if parts else "no robots"

    def _publish_cmd(self, name: str, cmd: str, attempt: int = 1) -> None:
        ch = self._channels[name]
        ch.done_event.clear()
        ch.ack_event.clear()
        ch.ack_seen = False
        ch.skip_next_idle = True
        msg = String()
        msg.data = cmd
        self._cmd_pubs[name].publish(msg)
        note = f" (resend #{attempt - 1})" if attempt > 1 else ""
        self.get_logger().info(
            f"[{name}] step {ch.index + 1}/{len(ch.commands)} -> {cmd}{note}")

    def _wait_for_robots(self, timeout_sec: float = 10.0) -> None:
        """Publishing before publisher/subscriber matching completes silently
        drops the message — wait for every robot's cmd subscriber first."""
        import time as _time
        t0 = _time.monotonic()
        while _time.monotonic() - t0 < timeout_sec:
            missing = [n for n, p in self._cmd_pubs.items()
                       if p.get_subscription_count() == 0]
            if not missing:
                _time.sleep(0.3)  # settle
                return
            _time.sleep(0.1)
        self.get_logger().warning(
            f"cmd topic not matched for: {missing} — robots offline? "
            "Proceeding; the BUSY-ack resend will retry deliveries.")

    def wait_for_status_match(self, names, timeout_sec: float = 5.0) -> None:
        """Block until each of NAMES' <name>_status subscription has a
        matched publisher (the robot's own status publisher) before
        reporting "loaded" -- REAL BUG confirmed on hardware 2026-07-31: a
        robot that had been on and publishing for a while, well before this
        mission was ever sent, was shown "offline" on the dashboard the
        instant Send mission was clicked. Root cause: configure_robots()
        creates each robot's status subscription for the FIRST time a
        mission names it (this supervisor was never subscribed before that
        -- it only just learned this robot exists), and _on_status() had
        never fired yet, so ch.last_seen was still 0.0 (never updated) at
        the moment publish_fleet_status("loaded", ...) ran right after --
        nothing to do with the robot's real connectivity, purely DDS/
        rosbridge discovery not having completed yet for OUR side's brand
        new subscription. Same class of bug already fixed for cmd_pub
        (_wait_for_robots(), the outbound direction) and for
        camera_grid_navigate.py's status_sub (the same inbound direction,
        different node) -- this is the matching fix for THIS node's inbound
        status subscriptions. Best-effort: a genuinely offline robot will
        still correctly show offline after the timeout, this only removes
        the false negative for one that's actually fine."""
        import time as _time
        t0 = _time.monotonic()
        while _time.monotonic() - t0 < timeout_sec:
            missing = [n for n in names
                      if n in self._status_subs
                      and self._status_subs[n].get_publisher_count() == 0]
            if not missing:
                return
            _time.sleep(0.1)
        self.get_logger().warning(
            f"status topic not matched for: {missing} within "
            f"{timeout_sec:.0f}s — may show offline briefly even if the "
            "robot is actually on")

    def _send_step(self) -> list[str]:
        """Send the next command to every unfinished robot, resending until
        each robot ACKs with 'BUSY <cmd>'. Returns the names actually sent."""
        sent = []
        for name, ch in self._channels.items():
            if ch.finished or ch.errored:
                continue
            self._publish_cmd(name, ch.next_command)
            sent.append(name)
        for name in sent:
            ch = self._channels[name]
            attempt = 1
            while not ch.ack_event.wait(timeout=ACK_TIMEOUT_SEC):
                attempt += 1
                if attempt > MAX_SEND_ATTEMPTS:
                    self.get_logger().error(
                        f"[{name}] never acknowledged '{ch.next_command}' after "
                        f"{MAX_SEND_ATTEMPTS} attempts; excluded from stepping")
                    ch.errored = True
                    break
                self.get_logger().warning(
                    f"[{name}] no BUSY ack within {ACK_TIMEOUT_SEC:.0f}s — resending")
                self._publish_cmd(name, ch.next_command, attempt)
        return [n for n in sent if not self._channels[n].errored]

    def _wait_for_step(self, sent: list[str], timeout_sec: float) -> bool:
        """Wait for every robot sent a command this step to report done.
        Returns True if all finished (or errored) within the timeout."""
        ok = True
        for name in sent:
            ch = self._channels[name]
            if not ch.done_event.wait(timeout=timeout_sec):
                self.get_logger().error(f"[{name}] timed out waiting for: {ch.next_command}")
                ok = False
                continue
            if not ch.errored:
                ch.index += 1
        return ok

    def run_stepped(self, timeout_sec: float = 30.0) -> None:
        """Prompt-driven loop: Enter = one step, 'auto' = free-run to the
        end, 'quit' = stop early."""
        auto = False
        self._wait_for_robots()
        while any(not ch.finished and not ch.errored for ch in self._channels.values()):
            if not auto:
                remaining = ", ".join(
                    f"{name}={ch.index}/{len(ch.commands)}"
                    for name, ch in self._channels.items()
                )
                choice = input(
                    f"[{remaining}] Press Enter for next step, "
                    f"'auto' to free-run, 'quit' to stop: "
                ).strip().lower()
                if choice == "quit":
                    self.get_logger().info("Stopped by user.")
                    return
                if choice == "auto":
                    auto = True

            sent = self._send_step()
            if not sent:
                break
            self._wait_for_step(sent, timeout_sec)
            # ground-truth check for the human (or the eventual solver) at
            # the step boundary — covers both prompted and 'auto' runs
            self._print_vision()

        if all(ch.errored for ch in self._channels.values() if ch.commands):
            self.get_logger().error("Fleet stopped: all robots errored.")
        else:
            self.get_logger().info("Fleet step run complete.")

    def run_advised(self, advisor, timeout_sec: float = 90.0) -> None:
        """Autonomous per-robot loop for advisor mode. Each robot is advanced
        the moment IT reports done: record completion with the advisor, ask
        it for the next action (planned command, or a gated DWELL hold), and
        publish. Robots that error out stop being advanced; the rest keep
        going. Ctrl+C stops the run."""
        import time as _time

        self._wait_for_robots()
        t0 = _time.monotonic()
        phase = {name: "idle" for name in self._channels}
        # idle | waiting (no command outstanding) | busy | done
        sent_at = {name: 0.0 for name in self._channels}
        current_cmd = {name: "" for name in self._channels}
        resends = {name: 0 for name in self._channels}
        next_gate_at = {name: 0.0 for name in self._channels}
        last_hold_log = {name: ("", -math.inf) for name in self._channels}
        launched: set[str] = set()
        last_vision_check = 0.0
        last_status_pub = 0.0
        last_emergency_brake = {}   # (a,b) pair -> monotonic time of last STOP
        self._abort = False

        # -- vision-guided recovery (LINE_LOST / runaway watchdog) -----------
        # stage machine per robot: pose0 (read camera at the stop point) ->
        # back (ROTATE_180 + FORWARD_UNTIL_RED onto a lattice node) -> fix
        # (camera names the node; displacement names the heading) -> bridge
        # (turns + FORWARD_UNTIL_REDs to the plan's resume point) -> resync.
        recovery: dict[str, dict] = {}
        recovery_counts = {name: 0 for name in self._channels}
        recovered_indexes: dict[str, set] = {name: set() for name in self._channels}
        recovery_declined: set[str] = set()

        def send_cmd(name: str, cmd: str) -> None:
            ch = self._channels[name]
            ch.done_event.clear()
            ch.ack_event.clear()
            ch.ack_seen = False
            ch.skip_next_idle = True
            msg = String()
            msg.data = cmd
            self._cmd_pubs[name].publish(msg)
            current_cmd[name] = cmd
            sent_at[name] = now
            resends[name] = 0
            phase[name] = "busy"

        def send_stop(name: str) -> None:
            msg = String()
            msg.data = "STOP"
            self._cmd_pubs[name].publish(msg)

        def send_reset_pose(name: str) -> None:
            # Best-effort, no ack tracked -- same pattern as send_stop(). Only
            # called once a robot's plan is fully complete (missions in this
            # system always finish with the robot parked back on its own
            # depot sticker, confirmed 2026-07-24), since RESET_POSE zeroes
            # x/y/yaw together (alvik.reset_pose(0,0,0)) and would corrupt
            # odometry if sent anywhere else on the grid. Fixes the cross-run
            # odometry drift seen in back-to-back send_route.py tests without
            # a reset between them (offsets kept growing run over run instead
            # of starting fresh).
            msg = String()
            msg.data = "RESET_POSE"
            self._cmd_pubs[name].publish(msg)
            self.get_logger().info(f"[{name}] RESET_POSE sent (plan complete, "
                                   "parked at depot)")

        def log_wait(name: str, reason: str, *, deadlock: bool = False) -> None:
            previous, logged_at = last_hold_log[name]
            if previous == reason and now - logged_at < HOLD_LOG_THROTTLE_SEC:
                return
            last_hold_log[name] = (reason, now)
            text = f"[{name}] SUPERVISOR WAIT t={now:.1f}s ({reason})"
            if deadlock:
                self.get_logger().error(text)
                self._event("wait", f"DEADLOCK hold: {reason}", robot=name,
                            severity="error")
            else:
                self.get_logger().warning(text)
                self._event("wait", reason, robot=name, severity="warn")

        def fresh_pose(name: str, max_age: float = 1.0):
            if self.vision is None:
                return None
            pose = self.vision.snapshot().get(name)
            if pose is None or pose[3] > max_age:
                return None
            return pose

        def node_occupied(name: str, node_label) -> bool:
            if self.vision is None:
                return False
            cell = label_to_cell(node_label, advisor.rows, advisor.cols)
            if cell is None:
                return False
            for other, pose in self.vision.snapshot().items():
                if other == name or pose[3] > VISION_STALE_SEC:
                    continue
                if math.hypot(pose[0] - cell[0],
                              pose[1] - cell[1]) < VISION_TARGET_BLOCK_CELLS:
                    return True
            return False

        def runaway_cap(name: str) -> float:
            expected = 4.0  # recovery moves / unknown: about one grid hop
            st = advisor.robots.get(name)
            if st is not None and st.pending and st.pending[0] == "plan":
                item = st.plan[st.pending[1]]
                if item.duration_sec > 0:
                    expected = item.duration_sec + sum(
                        d for _, _, d in item.implicit_segs)
            return max(RUNAWAY_MIN_CAP_SEC,
                       RUNAWAY_FACTOR * expected + RUNAWAY_GRACE_SEC)

        def fail_recovery(name: str, reason: str) -> None:
            send_stop(name)
            recovery.pop(name, None)
            recovery_declined.add(name)
            self._channels[name].errored = True
            self.get_logger().error(
                f"[{name}] RECOVERY failed ({reason}); robot excluded")
            self._event("recovery", f"failed ({reason}); robot excluded",
                        robot=name, severity="error")

        def start_recovery(name: str, why: str) -> None:
            ch = self._channels[name]
            failed_idx = advisor.suspend(name)
            target = None
            if failed_idx is not None:
                target = resync_point(advisor.robots[name].plan, failed_idx,
                                      advisor.rows, advisor.cols)
            if (self.vision is None or failed_idx is None or target is None
                    or recovery_counts[name] >= RECOVERY_MAX_PER_ROBOT
                    or failed_idx in recovered_indexes[name]):
                send_stop(name)
                recovery_declined.add(name)
                ch.errored = True
                self.get_logger().error(
                    f"[{name}] {why} — not recoverable (vision "
                    f"{'on' if self.vision else 'OFF'}, "
                    f"failed_index={failed_idx}, "
                    f"resync={'found' if target else 'none'}, "
                    f"prior_attempts={recovery_counts[name]}); robot excluded")
                self._event("recovery", f"{why} — not recoverable; excluded",
                            robot=name, severity="error")
                return
            ch.errored = False
            recovery_counts[name] += 1
            recovered_indexes[name].add(failed_idx)
            recovery[name] = {"stage": "pose0", "since": now, "target": target,
                              "pos0": None, "queue": []}
            phase[name] = "idle"
            self.get_logger().warning(
                f"[{name}] RECOVERY started ({why}): plan resumes at item "
                f"#{target[0]}, node {target[1]}, heading {target[2]}")
            self._event("recovery",
                        f"started ({why}); resumes at node {target[1]}",
                        robot=name, severity="warn")

        self.get_logger().info(
            "Advisor mode: autonomous, per-robot advancement. Ctrl+C to stop.")
        start_battery = self._battery_report()
        self.get_logger().info(f"battery at start: {start_battery}")
        self._event("mission", f"run started — {len(self._channels)} robot(s); "
                    f"battery {start_battery}", severity="info")
        while True:
            now = _time.monotonic() - t0

            if self._abort:
                self.get_logger().warning("mission aborted via /fleet_control")
                self.publish_fleet_status("aborted", advisor, phase)
                return

            if now - last_status_pub >= 1.0:
                last_status_pub = now
                self.publish_fleet_status("running", advisor, phase,
                                          now_sec=now)

            # Vision proximity: warn tier (logged/streamed) + emergency-brake
            # tier (an active e-stop). Runs every tick so the brake can't be
            # missed; warnings stay throttled to ~2 s.
            if self.vision is not None:
                pairs = self.vision.proximity_pairs()
                warn_due = now - last_vision_check >= 2.0
                if warn_due:
                    last_vision_check = now
                for a, b, d in pairs:
                    if d < VISION_EMERGENCY_CELLS:
                        # Imminent contact: STOP whichever of the pair is still
                        # moving (both if both are). This overrides the plan —
                        # the space-time model has mispredicted physical
                        # separation. Cooldown per pair so we don't re-STOP a
                        # robot that has already braked and is recovering.
                        key = tuple(sorted((a, b)))
                        if (now - last_emergency_brake.get(key, -math.inf)
                                < EMERGENCY_BRAKE_COOLDOWN_SEC):
                            continue
                        movers = emergency_brake_targets(
                            (a, b), d, phase, recovery)
                        if not movers:
                            continue
                        last_emergency_brake[key] = now
                        self.get_logger().error(
                            f"[EMERGENCY] {a} and {b} only {d:.2f} cells apart "
                            f"(< {VISION_EMERGENCY_CELLS}); braking {movers}")
                        self._event("error",
                                    f"EMERGENCY brake — {a} & {b} {d:.2f} cells "
                                    f"apart; stopped {', '.join(movers)}",
                                    severity="error")
                        for n in movers:
                            send_stop(n)
                            # Route into vision recovery so the robot re-finds
                            # its plan from where it actually stopped, instead
                            # of blindly resuming a command mid-corridor.
                            # start_recovery self-declines (STOP + exclude) when
                            # the robot can't be recovered (vision off / attempts
                            # exhausted), so this is always safe to call.
                            start_recovery(n, "emergency vision brake")
                        continue
                    if warn_due:
                        self.get_logger().warning(
                            f"vision WARNING: {a} and {b} are only "
                            f"{d:.2f} cells apart")
                        # robot=None: the text already names both robots, and
                        # the HTML bolds a robot= prefix (would double up).
                        self._event("vision",
                                    f"{a} and {b} only {d:.2f} cells apart",
                                    severity="warn")

            # LINE_LOST -> vision-guided recovery instead of exclusion. Only
            # line-lost errors: the touch e-stop (EMERGENCY_STOP) stays manual.
            for name, ch in self._channels.items():
                if (ch.errored and name not in recovery
                        and name not in recovery_declined
                        and name in launched
                        and "LINE_LOST" in ch.last_status):
                    start_recovery(name, "line lost")

            unfinished = [n for n, ch in self._channels.items()
                          if not ch.errored and phase[n] != "done"]
            if not unfinished:
                break

            for name in unfinished:
                ch = self._channels[name]

                if phase[name] in ("idle", "waiting"):
                    if now < next_gate_at[name]:
                        continue
                    rec = recovery.get(name)
                    if rec is not None:
                        if now - rec["since"] > RECOVERY_STAGE_TIMEOUT_SEC:
                            fail_recovery(name, f"stage '{rec['stage']}' timed out")
                            continue
                        if rec["stage"] == "pose0":
                            if now - rec["since"] < RECOVERY_SETTLE_SEC:
                                continue
                            pose = fresh_pose(name)
                            if pose is None:
                                continue  # waiting on the camera; stage timeout guards
                            rec["pos0"] = (pose[0], pose[1])
                            rec["queue"] = [("FORWARD_UNTIL_RED", None)]
                            rec["stage"] = "back"
                            rec["since"] = now
                            send_cmd(name, "ROTATE_180")
                            self.get_logger().info(
                                f"[{name}] RECOVERY t={now:.1f}s -> ROTATE_180 "
                                f"(stopped at grid ({pose[0]:+.2f},{pose[1]:+.2f}))")
                        elif rec["stage"] == "back":
                            if rec["queue"]:
                                cmd, _ = rec["queue"].pop(0)
                                rec["since"] = now
                                send_cmd(name, cmd)
                                self.get_logger().info(
                                    f"[{name}] RECOVERY t={now:.1f}s -> {cmd}")
                            else:
                                rec["stage"] = "fix"
                                rec["since"] = now
                        elif rec["stage"] == "fix":
                            if now - rec["since"] < RECOVERY_SETTLE_SEC:
                                continue
                            pose = fresh_pose(name)
                            if pose is None:
                                continue
                            node = node_at_cell(pose[0], pose[1],
                                                advisor.rows, advisor.cols)
                            disp = math.hypot(pose[0] - rec["pos0"][0],
                                              pose[1] - rec["pos0"][1])
                            if node is None or disp < RECOVERY_MIN_DISP_CELLS:
                                fail_recovery(name, f"cannot fix pose (node={node}, "
                                                    f"moved {disp:.2f} cells)")
                                continue
                            heading = heading_between_cells(
                                rec["pos0"], (pose[0], pose[1]))
                            idx, rnode, rheading = rec["target"]
                            rec["queue"] = bridge_commands(
                                node, heading, rnode, rheading,
                                advisor.rows, advisor.cols)
                            rec["stage"] = "bridge"
                            rec["since"] = now
                            self.get_logger().info(
                                f"[{name}] RECOVERY fixed at node {node} heading "
                                f"{heading}; bridging {len(rec['queue'])} cmd(s) "
                                f"to node {rnode}")
                        elif rec["stage"] == "bridge":
                            if not rec["queue"]:
                                idx, rnode, _ = rec["target"]
                                advisor.resync(name, idx, rnode, now)
                                del recovery[name]
                                self.get_logger().info(
                                    f"[{name}] RECOVERY complete t={now:.1f}s — "
                                    f"resuming plan at item #{idx} (node {rnode})")
                                self._event("recovery",
                                            f"complete; resuming at node {rnode}",
                                            robot=name, severity="info")
                            else:
                                cmd, tnode = rec["queue"][0]
                                if tnode is not None and node_occupied(name, tnode):
                                    phase[name] = "waiting"
                                    next_gate_at[name] = (
                                        now + SUPERVISOR_WAIT_RECHECK_SEC)
                                    log_wait(
                                        name,
                                        f"recovery node {tnode} occupied")
                                else:
                                    rec["queue"].pop(0)
                                    rec["since"] = now
                                    send_cmd(name, cmd)
                                    self.get_logger().info(
                                        f"[{name}] RECOVERY t={now:.1f}s -> {cmd}")
                        continue
                    # Honor the plan's staggered depot launch for the first send.
                    # Colour-sensor path: offsets only. The cascade gate
                    # needs live vision to measure clearance, and this path
                    # runs without it, so --launch-mode cascade is ignored
                    # here rather than silently releasing every robot at once.
                    if name not in launched and now < advisor.launch_offset(name):
                        continue
                    decision = advisor.decide(name, now)
                    if decision.supervisor_wait:
                        phase[name] = "waiting"
                        next_gate_at[name] = now + max(
                            SUPERVISOR_WAIT_RECHECK_SEC,
                            decision.retry_after_sec)
                        log_wait(name, decision.reason,
                                 deadlock=decision.deadlock)
                        continue
                    if decision.command is None:
                        phase[name] = "done"
                        self.get_logger().info(f"[{name}] plan complete ({decision.reason}).")
                        send_reset_pose(name)
                        continue

                    # Deterministic ownership is established first. Fresh
                    # vision may conservatively veto the physical dispatch;
                    # cancel_dispatch releases the unsent reservation without
                    # advancing or re-anchoring the plan.
                    # Turns first: _vision_hold_reason() returns None for
                    # them (no movement request), so without this a rotation
                    # passes every gate in the system untested.
                    vision_hold = (self._rotation_clearance_hold(advisor, name)
                                   or self._vision_hold_reason(advisor, name))
                    if vision_hold:
                        advisor.cancel_dispatch(name)
                        phase[name] = "waiting"
                        next_gate_at[name] = (
                            now + SUPERVISOR_WAIT_RECHECK_SEC)
                        log_wait(name, vision_hold)
                        continue

                    out_cmd = decision.command
                    if self.vision is not None:
                        pose = self.vision.snapshot().get(name)
                        vyaw = pose[2] if pose is not None else None
                        vage = pose[3] if pose is not None else math.inf
                        oyaw = ch.odom_yaw_deg
                        oage = (time.time() - ch.odom_yaw_seen
                                if ch.odom_yaw_seen > 0 else math.inf)
                        out_cmd, drift_note = drift_corrected_command(
                            out_cmd, vyaw, vage, oyaw, oage)
                        if drift_note:
                            self.get_logger().info(f"[{name}] {drift_note}")
                            self._event("drift", drift_note, robot=name,
                                        severity="warn")

                    ch.done_event.clear()
                    ch.ack_event.clear()
                    ch.ack_seen = False
                    ch.skip_next_idle = True
                    msg = String()
                    msg.data = out_cmd
                    self._cmd_pubs[name].publish(msg)
                    launched.add(name)
                    current_cmd[name] = out_cmd
                    sent_at[name] = now
                    resends[name] = 0
                    phase[name] = "busy"
                    self.get_logger().info(
                        f"[{name}] PLAN t={now:.1f}s -> {out_cmd} "
                        f"({decision.reason})")

                elif phase[name] == "busy":
                    if ch.done_event.is_set():
                        ch.done_event.clear()
                        advisor.record_done(name, _time.monotonic() - t0)
                        phase[name] = "idle"
                        # A released node/edge/reservation is a fleet-state
                        # change: wake all supervisor-side waiters now.
                        for waiting_name, waiting_phase in phase.items():
                            if waiting_phase == "waiting":
                                next_gate_at[waiting_name] = 0.0
                    elif not ch.ack_seen and now - sent_at[name] > ACK_TIMEOUT_SEC:
                        # command may have been lost (best-effort): resend
                        # until the robot ACKs with "BUSY <cmd>"
                        resends[name] += 1
                        if resends[name] >= MAX_SEND_ATTEMPTS:
                            self.get_logger().error(
                                f"[{name}] never acknowledged {current_cmd[name]} after "
                                f"{MAX_SEND_ATTEMPTS} attempts; excluded from stepping")
                            self._event("error",
                                        f"no ACK for {current_cmd[name]} after "
                                        f"{MAX_SEND_ATTEMPTS} tries; excluded",
                                        robot=name, severity="error")
                            ch.errored = True
                        else:
                            self.get_logger().warning(
                                f"[{name}] no BUSY ack for {current_cmd[name]} — resend "
                                f"#{resends[name]}")
                            ch.ack_event.clear()
                            ch.skip_next_idle = True
                            msg = String()
                            msg.data = current_cmd[name]
                            self._cmd_pubs[name].publish(msg)
                            sent_at[name] = now
                    elif (ch.ack_seen
                          and (current_cmd[name].startswith("FORWARD_UNTIL")
                               or current_cmd[name].startswith("BACKWARD_UNTIL"))
                          and now - sent_at[name] > runaway_cap(name)):
                        # Runaway: the marker was missed and the robot is
                        # line-following away hunting for it (2026-07-16: two
                        # robots crossed the whole grid and stopped on the
                        # boundary failsafe). STOP it near its route while
                        # vision can still walk it back.
                        send_stop(name)
                        self.get_logger().error(
                            f"[{name}] runaway watchdog: {current_cmd[name]} "
                            f"still BUSY after {now - sent_at[name]:.1f}s "
                            f"(cap {runaway_cap(name):.1f}s); STOP sent")
                        self._event("error",
                                    f"runaway watchdog STOP on "
                                    f"{current_cmd[name]}", robot=name,
                                    severity="error")
                        if name in recovery:
                            fail_recovery(name, "runaway during recovery")
                        elif name in recovery_declined:
                            ch.errored = True
                        else:
                            start_recovery(name, f"runaway on {current_cmd[name]}")
                    elif now - sent_at[name] > timeout_sec:
                        # The robot ACKed but never finished — it is likely
                        # physically blocked (e.g. pushing against another
                        # robot, observed 2026-07-14). Stop it before
                        # excluding it, or it keeps driving indefinitely.
                        stop = String()
                        stop.data = "STOP"
                        self._cmd_pubs[name].publish(stop)
                        self.get_logger().error(
                            f"[{name}] timed out ({timeout_sec:.0f}s) on {current_cmd[name]}; "
                            f"STOP sent, robot excluded from further stepping")
                        self._event("error",
                                    f"timed out on {current_cmd[name]}; excluded",
                                    robot=name, severity="error")
                        ch.errored = True

            _time.sleep(0.02)

        done = [n for n, p in phase.items() if p == "done"]
        errored = [n for n, ch in self._channels.items() if ch.errored]
        self.get_logger().info(
            f"Advisor run finished. Complete: {done or 'none'}; errored: {errored or 'none'}.")
        end_battery = self._battery_report()
        self.get_logger().info(f"battery at end:   {end_battery}")
        self._event("mission",
                    f"run finished — complete: {done or 'none'}; "
                    f"errored: {errored or 'none'}; battery {end_battery}",
                    severity="error" if errored else "info")
        self.publish_fleet_status("finished", advisor, phase,
                                  note=f"complete={done} errored={errored}")

    # -- position-target dispatch (drive_mode="vision") ----------------------
    def run_advised_vision(self, advisor, timeout_sec: float = 90.0,
                           fuse_turns: bool = False,
                           localization_mode: str = "camera_assist",
                           turn_method: str | None = None) -> None:
        """Same per-robot advancement as run_advised(), but executes each
        plan item with position/heading control instead of the color-sensor
        command vocabulary — no FORWARD_UNTIL_<color>. localization_mode
        selects encoder-only, encoder + camera correction, or camera-only
        feedback for the closed-loop controller.

        fuse_turns: when True AND this mission has exactly one robot, the
        turn toward a leg's NEXT heading hands off directly into driving
        that leg (turn_then_drive_leg(), no brake/settle in between) instead
        of always braking and settling first. Added 2026-07-31 per explicit
        user direction ("the robot taking too long around the corners").
        FORCED OFF for missions with 2+ robots regardless of this flag: the
        collision-safety pre-authorization (DispatchAdvisor.
        try_authorize_next_move()/record_done_fused()) is implemented and
        unit-tested, but has not been exercised against real multi-robot
        contention on hardware yet -- see run_advised_vision()'s call site
        in _run_listen_loop() and try_authorize_next_move()'s own docstring
        for the full reasoning. Single-robot missions have no other robot to
        collide with, so fusion there carries none of that risk.

        drive_leg()/turn_to_heading_rotate_rel() (camera_grid_navigate.py)
        are blocking, closed-loop functions. drive_leg() continuously
        streams wheel RPMs until arrival; turns send one firmware-controlled
        ROTATE_REL and wait for its target-gated, post-brake completion ack.
        That is the opposite of the fire-and-forget command/ack model
        run_advised() is built around, so they can't share that loop.
        Instead each robot gets its own CameraGridNavigator instance and
        worker thread (VisionLegWorker below); this method only calls
        advisor.decide()/record_done() and blocks on that worker completing
        the handed-off PlanItem. The advisor's node/edge reservation and
        collision logic is unchanged — it only ever consumed from_node/
        to_node, never the command token, so it is correct for either mode.

        Recovery/runaway-watchdog/ack-resend machinery from run_advised() is
        NOT reused here: it exists specifically to detect and repair color-
        sensor line-following failures (LINE_LOST, missed marker). Vision
        driving has no equivalent failure mode — drive_leg()/
        turn_to_heading_rotate_rel() already abort cleanly (return False) on
        lost localization or a stalled approach, which this loop treats as a
        robot exclusion, same as any other error."""
        import time as _time

        localization_mode = normalize_localization_mode(localization_mode)

        # DISABLED 2026-07-31 -- DO NOT RE-ENABLE without a real redesign.
        # fuse_turns caused TWO real hardware collisions on Alvik1 in the
        # same session it was added. Root causes, both confirmed on
        # hardware and both fixed in turn: (1) handoff into drive_leg()
        # while the robot was still physically rotating (fixed: added a
        # deceleration + multi-tick stability check before handoff); (2)
        # turn_then_drive_leg()'s arrival square-up reused the heading it
        # had just turned to/driven (the CURRENT leg's own direction) as if
        # it were the heading toward the NEXT leg after that -- it only ever
        # implemented the equivalent of run()'s "square up to this leg"
        # step, never run()'s separate "turn toward the next leg" step for
        # a fused move's own destination, because that needs a SECOND level
        # of lookahead (the leg after next_move) that neither the original
        # design nor the first patch accounted for. Building that correctly
        # -- and re-verifying the whole thing against real hardware again --
        # is real work that deserves a bench test, not a live route. See
        # turn_then_drive_leg()'s docstring in camera_grid_navigate.py for
        # the full history if this is ever revisited.
        if fuse_turns:
            self.get_logger().warning(
                "fuse_turns requested but is DISABLED (caused two real "
                "hardware collisions 2026-07-31 -- see run_advised_vision()'s "
                "docstring) -- forcing it off")
            fuse_turns = False

        self._wait_for_robots()
        self.get_logger().info(
            f"position-target workers using localization={localization_mode}")
        workers = {
            name: VisionLegWorker(name, self, localization_mode,
                                  turn_method=turn_method)
            for name in self._channels
        }
        for w in workers.values():
            w.start()
        try:
            t0 = _time.monotonic()
            phase = {name: "idle" for name in self._channels}
            # idle | busy | waiting | done
            next_gate_at = {name: 0.0 for name in self._channels}
            last_hold_log = {name: ("", -math.inf) for name in self._channels}
            # plan index of the fused next-move for a robot's current
            # in-flight job, or None -- read back on completion to know
            # whether to call record_done_fused() instead of record_done().
            fused_index: dict[str, int | None] = {name: None for name in self._channels}
            # Bounded per-leg retry. Before 2026-09-25 a single failed vision
            # leg excluded the robot permanently, so one bad turn ended that
            # robot's mission and, with its jobs orphaned, usually the run.
            # Every failure actually observed has been transient -- an
            # overshot turn, a rotation that did not start -- and the
            # navigator re-derives the whole maneuver from the CURRENT pose,
            # so a second attempt is a genuinely fresh one rather than a
            # replay of the same command. Keyed by (robot, plan index) so a
            # robot gets a fresh allowance at every leg instead of a budget
            # that silently runs out across a long route.
            leg_attempts: dict[tuple, int] = {}
            # Consecutive off-route observations per robot; reset by any
            # check that is not off-route. See OFFROUTE_MAX_HITS.
            offroute_hits: dict[str, int] = {n: 0 for n in self._channels}
            blocked_on_dead = 0
            # Total plan progress across the fleet; a stall is this not moving.
            last_progress_total = -1
            last_progress_at = 0.0
            max_leg_retries = getattr(self, "max_leg_retries",
                                      MAX_LEG_RETRIES)
            last_status_pub = 0.0
            self._abort = False
            # Staggered depot launch, mirroring run_advised()'s own use of
            # advisor.launch_offset() -- REAL BUG confirmed on hardware
            # 2026-07-31 (multi-robot fuse_turns bench test, unrelated to
            # fuse_turns itself): a 2-robot mission had both Alvik1 and
            # Alvik2 start moving the instant START was pressed, with no
            # regard for the schedule's per-robot launch_offset_sec --
            # run_advised_vision() never had this check at all (only
            # run_advised(), the color-sensor path, did). The depot lane is
            # a tight shared corridor specifically staggered to avoid two
            # robots leaving it at once; this check was silently missing for
            # every vision-drive mission with 2+ robots until now.
            launched: set[str] = set()

            keepalive_at: dict[str, float] = {}

            def log_wait(name: str, reason: str, *, deadlock: bool = False) -> None:
                # ONCE per wait, not every 3 s (2026-10-22, operator: 64
                # events in one depot launch, the same five waits repeated
                # while Alvik1 had not moved). Logged when a wait starts or
                # its reason changes; log_release() closes it with how long
                # it lasted. The decision is still re-checked every tick --
                # only the repetition in the log is gone.
                previous, since = last_hold_log[name]
                if previous == reason:
                    return
                last_hold_log[name] = (reason, now)
                text = f"[{name}] SUPERVISOR WAIT t={now:.1f}s ({reason})"
                if deadlock:
                    self.get_logger().error(text)
                    self._event("wait", f"DEADLOCK hold: {reason}", robot=name,
                                severity="error")
                else:
                    self.get_logger().warning(text)
                    self._event("wait", reason, robot=name, severity="warn")

            def log_release(name: str) -> None:
                """Close an open wait with its duration, once."""
                previous, since = last_hold_log[name]
                if not previous:
                    return
                last_hold_log[name] = ("", -math.inf)
                waited = now - since
                self.get_logger().info(
                    f"[{name}] wait over after {waited:.1f}s ({previous})")
                self._event("wait", f"released after {waited:.1f}s: {previous}",
                            robot=name, severity="info")

            self.get_logger().info(
                "Advisor mode (VISION DRIVE): autonomous, per-robot "
                "advancement. Ctrl+C to stop.")
            start_battery = self._battery_report()
            self.get_logger().info(f"battery at start: {start_battery}")
            self._event("mission",
                        f"run started (vision drive) — {len(self._channels)} "
                        f"robot(s); battery {start_battery}", severity="info")

            # ---- metrics stream (additive; no-ops without a metrics run) --
            for reg_name, reg_ch in self._channels.items():
                self._memit("ROBOT_REGISTERED", robot_id=reg_name,
                            battery_pct=reg_ch.battery_pct)
                if reg_ch.battery_pct is not None:
                    self._memit("BATTERY_SAMPLE", robot_id=reg_name,
                                battery_pct=reg_ch.battery_pct)
            # Per-robot dedup so the 0.2s waiting/idle recheck loop can't
            # spam ROBOT_STATE_CHANGED with the same state every tick, plus
            # in-flight command tracking for COMMAND_COMPLETED/FAILED and
            # the dwell -> PROCESSING_* mapping (last arrived node).
            last_metric_state: dict[str, str | None] = \
                {n: None for n in self._channels}
            # Open deadlocks, keyed by the cycle's member set. Value is
            # (conflict_id, first_seen_sec) -- see the deadlock block in the
            # main loop below.
            open_deadlocks: dict[frozenset, tuple[str, float]] = {}
            declined_at: dict = {}   # cycle -> when its last attempt was refused
            # Cycles seen but not yet DEADLOCK_CONFIRM_SEC old -> first seen.
            pending_cycles: dict[frozenset, float] = {}
            # Robots currently standing aside. Keyed by robot; see the
            # "yielding" branch in the main loop for the stage machine.
            yielding: dict[str, dict] = {}

            def _next_planned_move(robot: str):
                """The robot's next MOVE plan item, or None.

                Handed to the yield's return leg so _execute()'s step (3)
                turns toward it on arrival. Without this the robot gets home
                facing the way it came back, and the first resumed move
                inherits exactly the wrong-heading bug this fix is for --
                planned moves assume something already aimed them.
                """
                st = advisor.robots.get(robot)
                if st is None:
                    return None
                for item in st.plan[st.next_index:]:
                    if item.kind == "move":
                        return item
                return None
            inflight: dict[str, tuple] = {}
            last_node: dict[str, str] = {}
            in_service: dict[str, str] = {}   # robot -> workstation node

            def emit_state(name: str, new_state: str | None,
                           reason: str | None = None) -> None:
                if new_state and last_metric_state.get(name) != new_state:
                    last_metric_state[name] = new_state
                    self._memit("ROBOT_STATE_CHANGED", robot_id=name,
                                state_to=new_state, reason=reason)

            avoided: dict[tuple, float] = {}

            def emergency_stop(names, why: str) -> None:
                """STOP specific robots immediately, outside the plan.

                Publishes straight to each robot's command topic -- the same
                path ABORT uses -- rather than going through the per-robot
                worker, because a worker mid-drive_leg() is exactly what needs
                interrupting and it will not check in until its leg ends.
                """
                for nm in names:
                    pub = self._cmd_pubs.get(nm)
                    if pub is None:
                        continue
                    m = String()
                    m.data = "STOP"
                    pub.publish(m)
                self.get_logger().error(f"EMERGENCY STOP {sorted(names)}: {why}")

            def check_collisions(now_sec: float) -> bool:
                """Detect contact, and stop robots that are about to make it.

                Two tiers, both in inches over ALL robots including the depot
                lane -- proximity_pairs() is cells-only and lattice-only, and
                on 2026-09-29 Alvik5 drove into Alvik3 at DE3 without raising
                a single event because of exactly that gap.

                  * CLOSING FASTER THAN THEY CAN STOP -> emergency-stop both.
                    The trigger distance is contact plus whatever the closing
                    speed will cover in COLLISION_REACTION_SEC, so two robots
                    parked 6.2in apart in adjacent depot slots (closing speed
                    zero) never trigger, while two driving together do.
                  * ALREADY TOUCHING -> that IS a collision. Label it as one,
                    stop everything and end the run. The supervisor used to
                    report this as a generic ABORT, which is what the operator
                    objected to: a collision must be named.

                Returns True if the run should abort.
                """
                if self.vision is None:
                    return False
                cells = self.vision.snapshot()

                def parked(nm: str) -> bool:
                    p = cells.get(nm)
                    if p is None or p[3] > VISION_STALE_SEC:
                        return False
                    # Same two conditions off_lane() uses in the vision gates:
                    # below the grid is the depot lane/slots, and in_bay_spur
                    # is a workstation dead end.
                    if not (DEPOT_SLOT_MIN_Y_CELLS < p[1] < DEPOT_LANE_Y_CUTOFF
                            or in_bay_spur(p[0], p[1])):
                        return False
                    # ...and actually stopped. A robot driving out of a bay
                    # or slot is not parked (see PARKED_MAX_SPEED_IN_S). An
                    # unknown speed (no estimate yet) keeps the old rule.
                    speed = self.vision.speed_in(nm)
                    return speed is None or speed <= PARKED_MAX_SPEED_IN_S

                # FOOTPRINTS AND MOTION, not centre distance and closing rate
                # (2026-10-16). The old rule extrapolated how fast the
                # distance between centres was shrinking and compared it with
                # a contact distance taken along that same diagonal, so a
                # robot driving PAST another 5in to the side -- 1.0in of real
                # clearance -- was emergency-stopped at 7.2in. See
                # footprint_gap_in() and predicted_min_gap_in().
                for (a, b, sep, gap, min_gap, rel_speed,
                     passby) in self.vision.motion_pairs_ext():
                    margin = (COLLISION_PASSBY_MARGIN_IN if passby
                              else COLLISION_PREDICT_MARGIN_IN)
                    # PARKED ROBOTS ARE JUDGED BY FOOTPRINT TOO (2026-10-23).
                    # The old exemption ("collision only at <= 4.0in centre
                    # to centre") came from the 2026-09-29 contact model; the
                    # exact footprints (2026-10-16) already give a robot
                    # properly parked in a bay ~1in of real clearance to the
                    # lane, so the exemption no longer prevents false stops --
                    # it only hid real ones. 2026-10-23: Alvik5 came to rest
                    # 1.38in deep in workstation 81 (its deepest stop of 304)
                    # and Alvik2, driving the row behind the bay, was never
                    # predicted: the overlap was found only when Alvik5 began
                    # to leave. Parked pairs use the pass-by margin, and only
                    # the MOVING robot is stopped -- a STOP would knock the
                    # parked one out of its dwell.
                    pa, pb = parked(a), parked(b)
                    if pa or pb:
                        margin = min(margin, COLLISION_PASSBY_MARGIN_IN)
                    verdict = motion_verdict(gap, min_gap, rel_speed, sep,
                                             margin=margin)
                    if verdict is None:
                        continue
                    if verdict == "collision":
                        self.get_logger().error(
                            f"COLLISION: {a} and {b} are {sep:.2f}in apart, "
                            f"footprints overlap by {-gap:.2f}in")
                        emergency_stop([a, b], "collision")
                        self._event("error",
                                    f"COLLISION — {a} and {b} at {sep:.1f}in "
                                    f"(overlap {-gap:.1f}in); run aborted",
                                    severity="error")
                        self._memit("COLLISION",
                                    details={"robots": [a, b],
                                             "separation_in": round(sep, 2),
                                             "gap_in": round(gap, 2),
                                             "rel_speed_in_s": round(rel_speed, 2)})
                        self._memit("RUN_ABORTED", reason="COLLISION",
                                    details={"robots": [a, b],
                                             "separation_in": round(sep, 2)})
                        return True
                    # Rate-limit: one stop per pair per second, or a pair that
                    # stays close keeps re-stopping and never recovers.
                    key = (a, b)
                    if now_sec - avoided.get(key, -99.0) < 1.0:
                        continue
                    avoided[key] = now_sec
                    movers = [n for n, p in ((a, pa), (b, pb)) if not p] or [a, b]
                    emergency_stop(
                        movers,
                        (f"{a if pa else b} is parked; " if (pa or pb) else "")
                        + f"{gap:.2f}in between footprints now, "
                        f"{min_gap:.2f}in within {COLLISION_REACTION_SEC:.2f}s "
                        f"at {rel_speed:.1f}in/s relative "
                        f"(margin {margin:.2f}in"
                        + (", lane pass-by" if passby else "") + ")")
                    self._event("error",
                                f"COLLISION AVOIDED — {a} and {b}: "
                                f"{min_gap:.2f}in predicted gap at "
                                f"{rel_speed:.1f}in/s; "
                                + ("both stopped" if len(movers) == 2
                                   else f"{movers[0]} stopped"),
                                severity="error")
                    self._memit("COLLISION_AVOIDED",
                                details={"robots": [a, b],
                                         "separation_in": round(sep, 2),
                                         "gap_in": round(gap, 2),
                                         "min_gap_in": round(min_gap, 2),
                                         "rel_speed_in_s": round(rel_speed, 2)})
                return False

            def update_standing_holds(now_sec: float) -> None:
                """Release each robot's hold on the node it is standing on,
                but only once VISION says it has physically left, and only
                after it has stayed left for STANDING_LINGER_SEC.

                This is the measured half of the fix for the 2026-09-29 depot
                collision. record_done() used to free the node the instant a
                move completed -- i.e. the instant the robot arrived and was
                sitting on it -- and the next plan item is often a turn, which
                takes no reservation at all. Alvik5 was granted DE3 43ms
                before Alvik3 began to leave it, and drove into it.

                A robot whose pose is missing or stale keeps its hold: the
                safe failure is to make others wait, and the run-stall
                watchdog is the backstop if that ever deadlocks.
                """
                for name, node in advisor.expire_standing_linger(now_sec):
                    self.get_logger().info(
                        f"[{name}] released {node} "
                        f"{STANDING_LINGER_SEC:.1f}s after moving off it")
                holds = advisor.standing_holds()
                if not holds:
                    return
                poses = (self.vision.snapshot_in()
                         if self.vision is not None else {})
                for name, node in holds.items():
                    pose = poses.get(name)
                    if pose is None or pose[3] > VISION_STALE_SEC:
                        continue          # unknown position -> keep holding
                    try:
                        nx, ny = nav_node_to_world(
                            node, advisor.rows, advisor.cols)
                    except (ValueError, KeyError, TypeError):
                        # A label with no world position cannot be measured,
                        # so it cannot be proven vacated. Drop the hold rather
                        # than wedge the fleet on a label we do not understand.
                        advisor.note_standing_clearance(name, True, now_sec,
                                                        linger_sec=0.0)
                        continue
                    clear = math.hypot(pose[0] - nx,
                                       pose[1] - ny) > STANDING_VACATE_IN
                    if advisor.note_standing_clearance(name, clear, now_sec):
                        self.get_logger().info(
                            f"[{name}] released {node} "
                            f"{STANDING_LINGER_SEC:.1f}s after leaving it")

            while True:
                now = _time.monotonic() - t0
                # Safety first in the tick: a collision check that runs after
                # dispatch would be reporting history.
                if check_collisions(now):
                    self._abort = True
                    break
                update_standing_holds(now)

                # NOBODY MAY WAIT ON A ROBOT THAT IS ALREADY EXCLUDED.
                #
                # strand() deliberately keeps an excluded robot's NODE
                # reservation, because the robot is still physically on that
                # square and routing a live robot through it would be a
                # collision. The consequence is that anyone whose route needs
                # that node waits forever, and there is no replanning to route
                # around it. On 2026-09-25 Alvik3 was excluded holding node 5
                # and Alvik1 logged "resource node 5 reserved by Alvik3"
                # twenty-four times over 68 seconds until the run was killed
                # by hand.
                #
                # This is genuinely unrecoverable, so end the run promptly and
                # say why, instead of hanging. An unattended session gets a
                # finished run it can move past rather than a silent stall.
                # --- global liveness ------------------------------------
                # next_index only advances when a robot finishes a plan item,
                # so the fleet-wide sum is the one number that rises if and
                # only if SOMETHING is getting done. Yields and sidesteps do
                # not advance it, which is why the bound is generous rather
                # than tight.
                progress_total = sum(
                    st.next_index for st in advisor.robots.values())
                if progress_total != last_progress_total:
                    last_progress_total = progress_total
                    last_progress_at = now
                elif now - last_progress_at > RUN_STALL_SEC:
                    stuck = {n: last_hold_log[n][0] for n in self._channels
                             if phase.get(n) == "waiting"}
                    self.get_logger().error(
                        f"no robot has advanced its plan for "
                        f"{now - last_progress_at:.0f}s -- ending the run as "
                        f"stalled. waiting: {stuck}")
                    self._event("error",
                                f"run stalled: no plan progress for "
                                f"{now - last_progress_at:.0f}s",
                                severity="error")
                    self._memit("RUN_ABORTED", reason="RUN_STALLED",
                                details={"stalled_sec":
                                         round(now - last_progress_at, 1),
                                         "waiting": stuck,
                                         "drive_mode": "vision",
                                         "localization_mode":
                                             localization_mode})
                    self._abort = True

                dead = {n for n, c in self._channels.items() if c.errored}
                if dead:
                    blocked = [n for n in self._channels
                               if phase.get(n) == "waiting"
                               and any(d in last_hold_log[n][0] for d in dead)]
                    if blocked:
                        blocked_on_dead += 1
                        if blocked_on_dead >= BLOCKED_ON_DEAD_TICKS:
                            self.get_logger().error(
                                f"{sorted(blocked)} are waiting on resources "
                                f"held by excluded robot(s) {sorted(dead)}, "
                                "which will never move -- ending the run")
                            self._event(
                                "error",
                                f"unrecoverable: {sorted(blocked)} blocked by "
                                f"excluded {sorted(dead)}", severity="error")
                            # Emit the terminal event HERE. Setting _abort
                            # jumps to the /fleet_control abort branch, which
                            # returns without emitting one -- RUN_ABORTED is
                            # published by the operator-abort handler, not by
                            # that branch. Without this the run would sit at
                            # status 'running' in the metrics DB forever, the
                            # same integrity failure as the RUN_FAILED
                            # misclassification fixed earlier today.
                            self._memit(
                                "RUN_ABORTED",
                                reason="BLOCKED_BY_EXCLUDED_ROBOT",
                                details={"blocked": sorted(blocked),
                                         "excluded": sorted(dead),
                                         "drive_mode": "vision",
                                         "localization_mode":
                                             localization_mode})
                            self._abort = True
                    else:
                        blocked_on_dead = 0
                else:
                    blocked_on_dead = 0

                if self._abort:
                    self.get_logger().warning("mission aborted via /fleet_control")
                    for w in workers.values():
                        w.abort()
                    self.publish_fleet_status("aborted", advisor, phase)
                    return

                # ---- deadlock: record it, then fail fast -------------
                # ADDED 2026-08-31 after a 6-robot run held for 78s and was
                # aborted by hand. The advisor already DETECTED the cycle
                # (Alvik3<->Alvik5 opposite-direction edge swap, with
                # Alvik1/4 and Alvik2/6 queued behind) and logged it ~100
                # times, but nothing looked at the wait-for graph as a
                # whole, so: (a) metrics recorded deadlock_count = 0 for
                # that run, because nothing ever wrote a conflict row, and
                # (b) the run hung indefinitely, since a WAIT decision has
                # no timeout.
                #
                # This does NOT resolve deadlocks -- the advisor has no
                # yield/backtrack/reroute policy and picking one is a
                # separate decision. It makes the deadlock VISIBLE (a real
                # conflict row, so deadlock_count and intervention_free stop
                # being computed against an empty list) and BOUNDED (the run
                # ends by itself, with the cycle named, instead of holding
                # until someone notices). A run that ends in 20s with data
                # beats one that hangs for 78s and reports nothing.
                cycles = advisor.deadlock_cycles()
                seen_now = set()
                for cycle in cycles:
                    key = frozenset(cycle)
                    seen_now.add(key)
                    # A YIELD ALREADY RESOLVING EXACTLY THIS PAIR. A robot
                    # standing aside waits for the one it is letting through,
                    # and that robot waits for it until it is physically
                    # clear, so the pair reads as a cycle for as long as the
                    # maneuver runs. Treating that as a new deadlock is what
                    # sent BOTH robots aside on 2026-10-16: Alvik2 backed out
                    # of 137, the "same" cycle was seen 1.2s later, and
                    # Alvik3 was sidestepped too -- each then saw the other
                    # gone, decided it had passed, went home, and the cycle
                    # came straight back. The open entry stays open (it is in
                    # seen_now), so the yield-grace abort still bounds it.
                    if any(key == frozenset({yname, y["blocked"]})
                           for yname, y in yielding.items()):
                        continue
                    first_seen = key not in open_deadlocks
                    if first_seen:
                        t_first = pending_cycles.setdefault(key, now)
                        if now - t_first < DEADLOCK_CONFIRM_SEC:
                            continue
                    # A declined cycle is reconsidered rather than
                    # abandoned -- see DEADLOCK_RETRY_SEC.
                    if not first_seen:
                        last = declined_at.get(key)
                        if last is None or now - last < DEADLOCK_RETRY_SEC:
                            continue
                        declined_at[key] = now
                        conflict_id = open_deadlocks[key][0]
                    if first_seen:
                        conflict_id = f"dl-{'-'.join(cycle)}-{now:.0f}"
                        # Timed from the first sighting, so the abort and
                        # the recovery metric include the confirm window.
                        open_deadlocks[key] = (conflict_id,
                                               pending_cycles.pop(key, now))
                        self._memit("DEADLOCK_DETECTED",
                                    conflict_id=conflict_id,
                                    details={"type": "deadlock",
                                             "severity": "error",
                                             "robot_ids": list(cycle)})
                        self.get_logger().error(
                            f"DEADLOCK: wait-for cycle {' -> '.join(cycle)}"
                            f" -> {cycle[0]} at t={now:.1f}s; aborting in "
                            f"{DEADLOCK_ABORT_SEC:.0f}s unless it clears")
                        self._event("wait",
                                    f"DEADLOCK cycle {' -> '.join(cycle)}",
                                    severity="error")
                    # EXECUTING since 2026-09-xx -- this drives a real
                    # robot. Deciding and executing stay split: the decision
                    # is pure and unit tested
                    # (fleet/test_deadlock_resolution.py), while the
                    # maneuver moves a real robot past another one, and a
                    # wrong move is a collision rather than a bad number.
                    # Yields may overlap between robots (see below); a
                    # cycle whose resolution is refused is retried rather
                    # than abandoned.
                    try:
                        # Only robots physically ON their planned node may
                        # yield -- see YIELD_START_TOL_IN.
                        yielded = advisor.resolve_deadlock(
                            cycle, now,
                            can_yield=lambda r: self._on_planned_node(
                                advisor, r),
                            drives_straight=lambda r, n, reverse=False:
                                self._drives_straight(advisor, workers, r, n,
                                                      reverse))
                    except Exception as exc:
                        yielded = None
                        self.get_logger().error(
                            f"deadlock resolver raised {exc!r}; "
                            "falling back to the abort timer")
                    started = False
                    # CONCURRENT YIELDS ARE ALLOWED, and that is load
                    # bearing. Do not re-add a fleet-wide "one at a time"
                    # guard: it was tried on 2026-09-15 and turned two runs
                    # that had completed into fleet-wide lockups.
                    #
                    # The evidence, from the metrics DB. Both completed runs
                    # overlap their yields:
                    #   09-14  +109.8 Alvik5 yields ... +128.2 Alvik3 yields
                    #          (Alvik5 still out) ... +140.2 Alvik2 yields
                    #          (Alvik3 still out) -> RUN_COMPLETED 266.8s
                    #   09-09  +109.9 Alvik5 ... +128.2 Alvik4 (Alvik5 still
                    #          out) -> RUN_COMPLETED 244.5s
                    # With the guard in place the second resolution was
                    # refused at the same instant in the same cycle:
                    #   r2     +128.4 DECLINED "already yielding" -> ABORTED
                    #   r3     + 88.7 DECLINED "already yielding" -> ABORTED
                    #
                    # The r1 cascade that motivated the guard was really the
                    # abort suppression (`[] if yielding else`) letting a
                    # hung fleet run forever; that is fixed separately, above.
                    # Concurrency was never the fault.
                    #
                    # The only serialization kept is per robot: a robot
                    # already mid-maneuver cannot be given a second one.
                    if (yielded is not None
                            and yielded.robot not in yielding
                            and phase.get(yielded.robot) in
                            ("waiting", "idle")):
                        # Only ever start from a stationary robot. A
                        # mid-leg robot has a live reservation on the
                        # edge it is crossing, and pre-empting that is a
                        # different and much larger problem than the one
                        # this solves.
                        pair = advisor.begin_yield(yielded, now)
                        if pair is None:
                            self.get_logger().error(
                                f"DEADLOCK RESOLUTION: {yielded.to_node} "
                                "was taken between deciding and "
                                "reserving; abort timer stands")
                        else:
                            out_item, back_item = pair
                            slack = advisor.slack_by_robot(now)
                            blocked_at = advisor.robots[
                                yielded.blocked_robot].current_node
                            yielding[yielded.robot] = {
                                "stage": "out", "out": out_item,
                                "back": back_item, "home": out_item.from_node,
                                "blocked": yielded.blocked_robot,
                                "blocked_at": str(blocked_at),
                                # Plan position of the robot being let
                                # through, so "has passed" can require it
                                # to have actually done something.
                                "blocked_index": advisor.robots[
                                    yielded.blocked_robot].next_index,
                                "conflict_id": conflict_id,
                                "since": now, "stage_since": now,
                            }
                            workers[yielded.robot].submit(out_item, None,
                                                          False)
                            phase[yielded.robot] = "yielding"
                            started = True
                            self.get_logger().warning(
                                f"DEADLOCK RESOLUTION: {yielded.kind} "
                                f"{yielded.robot} {yielded.from_node}->"
                                f"{yielded.to_node} to clear "
                                f"{yielded.blocked_robot} | "
                                f"{yielded.reason}")
                            self._event(
                                "recovery",
                                f"{yielded.kind}: {yielded.robot} "
                                f"{yielded.from_node}->{yielded.to_node}",
                                robot=yielded.robot, severity="warn")
                            self._memit(
                                "DEADLOCK_RESOLUTION_STARTED",
                                conflict_id=conflict_id,
                                robot_id=yielded.robot,
                                details={
                                    "kind": yielded.kind,
                                    "from_node": str(yielded.from_node),
                                    "to_node": str(yielded.to_node),
                                    "blocked_robot":
                                        yielded.blocked_robot,
                                    "reason": yielded.reason,
                                    "slack_sec": {
                                        r: round(slack.get(r, 0.0), 2)
                                        for r in cycle},
                                })
                    if started:
                        declined_at.pop(key, None)
                    if not started:
                        why = ("no legal yield move for any member "
                               "(every neighbour occupied or reserved)"
                               if yielded is None else
                               f"{yielded.robot} is already mid-yield"
                               if yielded.robot in yielding else
                               f"{yielded.robot} was not stationary "
                               f"(phase={phase.get(yielded.robot)})")
                        # Only the FIRST decline per cycle logs and emits.
                        # The retries behind it are silent, or a cycle stuck
                        # for its whole abort window would write a row every
                        # DEADLOCK_RETRY_SEC -- r2 already logged 8307 events
                        # in 160s.
                        repeat = key in declined_at
                        declined_at.setdefault(key, now)
                        if not repeat:
                            self.get_logger().error(
                                f"DEADLOCK RESOLUTION: {why} -- retrying "
                                f"every {DEADLOCK_RETRY_SEC:.0f}s while the "
                                "abort timer runs")
                    if not started and not repeat:
                        self._memit(
                            "DEADLOCK_RESOLUTION_DECLINED",
                            conflict_id=conflict_id,
                            details={"kind": "none", "executed": False,
                                     "reason": why})
                # A cycle that cleared on its own is genuinely resolved --
                # record that rather than leaving the row open, so
                # mean_deadlock_recovery_sec has something to measure.
                # A cycle that vanished inside its confirm window was never a
                # deadlock: forget it without a row or a log line.
                for key in [k for k in pending_cycles if k not in seen_now]:
                    pending_cycles.pop(key, None)
                for key in [k for k in open_deadlocks if k not in seen_now]:
                    conflict_id, since = open_deadlocks.pop(key)
                    self._memit("DEADLOCK_RESOLVED", conflict_id=conflict_id,
                                reason="cycle cleared without intervention")
                    self.get_logger().info(
                        f"deadlock cleared after {now - since:.1f}s "
                        f"({', '.join(sorted(key))})")
                # A yield in flight IS the resolution -- do not abort out
                # from under it. But only for the deadlock THAT yield is
                # resolving, and only for as long as the maneuver may take.
                #
                # This was `[] if yielding else [...]`: ANY robot mid-yield
                # suppressed the abort for EVERY open deadlock. Measured
                # 2026-09-15, that is unbounded in practice -- four deadlocks
                # formed in 48s while yields take 47.7s each, so at least one
                # robot was always yielding, the abort never evaluated, and
                # the run sat in a six-robot hold for 50s until it was ended
                # by hand at t=208s. That is precisely the indefinite hang
                # the 2026-08-31 fail-fast was written to prevent; executing
                # yields reintroduced it through the back door.
                busy = set(yielding) | {y["blocked"] for y in yielding.values()}
                held = [(cid, since) for k, (cid, since) in open_deadlocks.items()
                        if now - since >= (DEADLOCK_YIELD_GRACE_SEC
                                           if (k & busy) else
                                           DEADLOCK_ABORT_SEC)]
                if held:
                    members = sorted(
                        {r for k in open_deadlocks for r in k})
                    self.get_logger().error(
                        f"DEADLOCK unresolved ({', '.join(members)}) -- "
                        f"aborting run. Either no yield was legal, or one ran "
                        f"and did not clear the cycle within "
                        f"{DEADLOCK_YIELD_GRACE_SEC:.0f}s.")
                    self._event("mission",
                                f"ABORT — deadlock unresolved for "
                                f"{DEADLOCK_ABORT_SEC:.0f}s: "
                                f"{', '.join(members)}", severity="error")
                    for w in workers.values():
                        w.abort()
                    self._memit("RUN_ABORTED",
                                reason=f"deadlock: {', '.join(members)}")
                    self.publish_fleet_status("aborted", advisor, phase)
                    return

                if now - last_status_pub >= 1.0:
                    last_status_pub = now
                    self.publish_fleet_status("running", advisor, phase,
                                              now_sec=now)

                unfinished = [n for n, ch in self._channels.items()
                              if not ch.errored and phase[n] != "done"]
                if not unfinished:
                    break

                for name in unfinished:
                    ch = self._channels[name]
                    worker = workers[name]

                    if phase[name] == "yielding":
                        y = yielding[name]
                        if now - y["since"] > YIELD_TOTAL_TIMEOUT_SEC:
                            # Stranded off its planned node. Nothing good is
                            # reachable from here -- the plan expects this
                            # robot at `home` and it cannot get there.
                            self.get_logger().error(
                                f"[{name}] yield exceeded "
                                f"{YIELD_TOTAL_TIMEOUT_SEC:.0f}s; leaving the "
                                "abort timer to end the run")
                            self._memit("DEADLOCK_RESOLUTION_FAILED",
                                        conflict_id=y["conflict_id"],
                                        robot_id=name,
                                        details={"reason": "yield timed out"})
                            yielding.pop(name, None)
                            phase[name] = "waiting"
                            next_gate_at[name] = now + SUPERVISOR_WAIT_RECHECK_SEC
                            continue

                        if y["stage"] == "clear":
                            # Stand aside until the blocked robot has really
                            # moved off the node it was waiting on -- its
                            # reservation clearing is not the same as the
                            # robot being physically gone.
                            bst = advisor.robots[y["blocked"]]
                            # A two-hop yield goes home through its middle
                            # node too: that has to be free and not about to
                            # be needed, exactly like home.
                            way_home = [str(y["home"])] + [
                                str(to) for _f, to, _d in
                                movement_segments(y["back"])[:-1]]
                            held = {k for k, holder in
                                    advisor._occupied_nodes().items()
                                    if holder != name}
                            home_free = not any(n in held for n in way_home)
                            # PASSED means went THROUGH, not merely "left the
                            # node it was waiting on". That was the whole
                            # test, and on 2026-10-16 it was satisfied by the
                            # blocked robot stepping aside itself: both
                            # yielders saw the other gone and went home into
                            # the same standoff. Now the robot let through
                            # must have completed planned work, must not be
                            # off-plan in a yield of its own, and must no
                            # longer be about to need our home node -- or we
                            # walk back into its path.
                            passed = (
                                bst.next_index > y.get("blocked_index", -1)
                                and str(bst.current_node) != y["blocked_at"]
                                and y["blocked"] not in yielding
                                and not any(advisor.needs_node_soon(
                                    y["blocked"], n) for n in way_home))
                            # Reserve the way home before driving it; a
                            # "looks free" check alone let Alvik4 be granted
                            # 3->2 while Alvik3 drove 10->2 (2026-10-22).
                            if (passed and home_free
                                    and advisor.begin_yield_return(
                                        name, y["back"], now)):
                                worker.submit(y["back"],
                                              _next_planned_move(name), False)
                                y["stage"] = "back"
                                y["stage_since"] = now
                                self.get_logger().info(
                                    f"[{name}] yield: {y['blocked']} has "
                                    f"passed; returning to {y['home']}")
                            elif now - y["stage_since"] > YIELD_CLEAR_TIMEOUT_SEC:
                                # Go home anyway if the node is free; the
                                # blocked robot may have been rerouted.
                                if home_free and advisor.begin_yield_return(
                                        name, y["back"], now):
                                    worker.submit(y["back"],
                                                  _next_planned_move(name),
                                                  False)
                                    y["stage"] = "back"
                                    y["stage_since"] = now
                            continue

                        result = worker.poll()
                        if result is None:
                            continue
                        ok, note, _fused = result
                        if not ok:
                            self.get_logger().error(
                                f"[{name}] yield leg failed: {note}")
                            self._memit("DEADLOCK_RESOLUTION_FAILED",
                                        conflict_id=y["conflict_id"],
                                        robot_id=name,
                                        details={"reason": note,
                                                 "stage": y["stage"]})
                            yielding.pop(name, None)
                            phase[name] = "waiting"
                            next_gate_at[name] = now + SUPERVISOR_WAIT_RECHECK_SEC
                            continue

                        if y["stage"] == "out":
                            advisor.finish_yield_leg(name, y["out"].to_node,
                                                     now)
                            y["stage"] = "clear"
                            y["stage_since"] = now
                            self.get_logger().info(
                                f"[{name}] yield: standing aside at "
                                f"{y['out'].to_node}, waiting for "
                                f"{y['blocked']}")
                            # The robot it was blocking can move now.
                            for waiting_name, waiting_phase in phase.items():
                                if waiting_phase == "waiting":
                                    next_gate_at[waiting_name] = 0.0
                            continue

                        # stage == "back": home again, plan untouched.
                        advisor.finish_yield_leg(name, y["home"], now)
                        advisor.end_yield(name, now)
                        took = now - y["since"]
                        self.get_logger().warning(
                            f"[{name}] yield complete in {took:.1f}s; "
                            f"back at {y['home']}, resuming plan")
                        self._memit("DEADLOCK_RESOLVED",
                                    conflict_id=y["conflict_id"],
                                    robot_id=name,
                                    reason=f"yield-and-return in {took:.1f}s")
                        self._event("recovery",
                                    f"yield complete: {name} back at "
                                    f"{y['home']} after {took:.1f}s",
                                    robot=name, severity="info")
                        open_deadlocks.pop(
                            frozenset({name, y["blocked"]}), None)
                        yielding.pop(name, None)
                        phase[name] = "idle"
                        next_gate_at[name] = 0.0
                        continue

                    if phase[name] == "idle":
                        if now < next_gate_at[name]:
                            continue
                        # Depot launch. Two modes, selected by
                        # --launch-mode; see LAUNCH_CLEARANCE_IN and
                        # _launch_cascade_hold().
                        #
                        #   offsets (default) -- wait out the schedule's
                        #     per-robot launch_offset_sec. 306.6s of queueing
                        #     across six robots on the 2026-09-09 export.
                        #   cascade -- release as soon as the node this robot
                        #     is moving into is measurably clear. 36.60s vs
                        #     120.01s to get the last robot onto node 0.
                        if name not in launched:
                            if self.launch_mode == "cascade":
                                hold = self._launch_cascade_hold(advisor, name)
                                if hold is not None:
                                    phase[name] = "waiting"
                                    next_gate_at[name] = (
                                        now + SUPERVISOR_WAIT_RECHECK_SEC)
                                    log_wait(name, hold)
                                    continue
                            elif now < advisor.launch_offset(name):
                                continue
                        decision = advisor.decide(name, now)
                        if decision.supervisor_wait:
                            phase[name] = "waiting"
                            next_gate_at[name] = now + max(
                                SUPERVISOR_WAIT_RECHECK_SEC,
                                decision.retry_after_sec)
                            log_wait(name, decision.reason,
                                     deadlock=decision.deadlock)
                            emit_state(name, "TRAFFIC_WAIT", decision.reason)
                            continue
                        if decision.command is None:
                            phase[name] = "done"
                            self.get_logger().info(
                                f"[{name}] plan complete ({decision.reason}).")
                            _vm = getattr(self, "vision", None)
                            _tp = getattr(_vm, "_poses_in", {}).get(name)
                            if _tp is None:
                                self._memit("DEPOT_ARRIVED", robot_id=name)
                            else:
                                self._memit("DEPOT_ARRIVED", robot_id=name,
                                            x_m=_tp[0] * 0.0254,
                                            y_m=_tp[1] * 0.0254,
                                            yaw_deg=_tp[2])
                            emit_state(name, "TERMINAL_IDLE")
                            continue

                        st = advisor.robots[name]
                        item = st.plan[st.next_index]
                        rpd_hold = self._rpd_hold(name, item, now)
                        if rpd_hold:
                            advisor.cancel_dispatch(name)
                            phase[name] = "waiting"
                            next_gate_at[name] = now + SUPERVISOR_WAIT_RECHECK_SEC
                            log_wait(name, rpd_hold)
                            emit_state(name, "PROCESSING_HOLD"
                                       if self._ws_bay(item.from_node) is not None
                                       else "TRAFFIC_WAIT", rpd_hold)
                            continue
                        vision_hold = (self._vision_hold_reason(advisor, name)
                                       if item.kind == "move" else None)
                        if vision_hold:
                            # BOUND THE OFF-ROUTE HOLD.
                            #
                            # Every other wait in this loop clears when some
                            # other robot moves. "off-route" does not: it means
                            # THIS robot is not where the advisor believes, and
                            # nothing in the wait path ever changes either of
                            # those. Before 2026-09-25 it had no timeout, so on
                            # run 4 Alvik5 logged it about eighty times over 80
                            # seconds while Alvik6, Alvik4, Alvik2, Alvik3 and
                            # Alvik1 queued behind it and the run had to be
                            # aborted by hand. Excluding the one robot strands
                            # it (releasing its corridor claims) and lets the
                            # other five finish.
                            if vision_hold.startswith("vision: off-route"):
                                # CONSECUTIVE observations, not wall clock.
                                # The first version timed from the first
                                # sighting, but this check is only reached
                                # when the advisor GRANTS -- a robot in a
                                # traffic wait never reaches it, so the clock
                                # ran through waits where off-route was not
                                # being evaluated at all. On 2026-09-25 that
                                # excluded Alvik3 after 25s of mostly ordinary
                                # queueing, and because strand() keeps the node
                                # reservation, Alvik1 then waited on node 5
                                # forever. A recoverable traffic wait became a
                                # permanent deadlock. Counting only back-to-back
                                # sightings measures a robot that is actually
                                # stuck rather than one that is merely blocked.
                                offroute_hits[name] = offroute_hits[name] + 1
                                if offroute_hits[name] >= OFFROUTE_MAX_HITS:
                                    self.get_logger().error(
                                        f"[{name}] off-route on "
                                        f"{offroute_hits[name]} consecutive "
                                        f"checks ({vision_hold}); excluding so "
                                        "the rest of the fleet can proceed")
                                    self._event(
                                        "error",
                                        f"off-route on "
                                        f"{offroute_hits[name]} consecutive "
                                        "checks; excluded", robot=name,
                                        severity="error")
                                    ch.errored = True
                                    self._memit("ROBOT_OFFLINE", robot_id=name,
                                                reason=f"off-route: {vision_hold}")
                                    emit_state(name, "FAULT_RECOVERY")
                                    advisor.strand(name, now)
                                    continue
                            else:
                                offroute_hits[name] = 0
                            advisor.cancel_dispatch(name)
                            phase[name] = "waiting"
                            next_gate_at[name] = now + SUPERVISOR_WAIT_RECHECK_SEC
                            log_wait(name, vision_hold)
                            emit_state(name, "TRAFFIC_WAIT", vision_hold)
                            continue
                        offroute_hits[name] = 0

                        # Lookahead so the worker can turn toward the NEXT
                        # leg after finishing this one, same as run()'s own
                        # per-node square-up-then-turn-toward-next structure
                        # (camera_grid_navigate.py). turn/noop items never
                        # carry from_node/to_node of their own -- skip over
                        # them to find the next real move. STOP at a dwell
                        # (do not skip over it): a dwell is a real scheduled
                        # action the advisor hasn't dispatched yet (it's a
                        # separate PlanItem, only sent once this move
                        # completes and next_index advances) -- fusing this
                        # move's turn-toward-next with a drive would skip
                        # straight past that dwell. next_move stays None in
                        # that case, so the worker keeps the safe
                        # target-gated turn_to_heading_rotate_rel() path
                        # (mirrors run()'s own rotate_at/dwell_at exclusion
                        # above).
                        next_move = None
                        next_move_index = None
                        for offset, future in enumerate(
                                st.plan[st.next_index + 1:], start=st.next_index + 1):
                            if future.kind == "move":
                                next_move = future
                                next_move_index = offset
                                break
                            if future.kind == "dwell":
                                break

                        fuse_next = False
                        if (fuse_turns and item.kind == "move"
                                and next_move is not None):
                            # Real collision-safety grant, not just a check --
                            # see try_authorize_next_move()'s own docstring.
                            # Gated behind fuse_turns (single-robot missions
                            # only, until tested against real multi-robot
                            # contention -- see run_advised_vision()'s own
                            # docstring for why).
                            fuse_next = advisor.try_authorize_next_move(
                                name, next_move_index, now)

                        worker.submit(item, next_move, fuse_next)
                        phase[name] = "busy"
                        launched.add(name)
                        fused_index[name] = next_move_index if fuse_next else None
                        # metrics stream (additive)
                        cmd_id = f"{name}:{st.next_index}"
                        inflight[name] = (item, cmd_id)
                        self._memit("COMMAND_SENT", robot_id=name,
                                    command_id=cmd_id,
                                    details={"kind": item.kind,
                                             "from": str(item.from_node),
                                             "to": str(item.to_node),
                                             "drive_mode": "vision",
                                             "localization_mode": localization_mode})
                        if item.kind == "move":
                            ahead = [it.to_node
                                     for it in st.plan[st.next_index:]
                                     if it.kind == "move"]
                            emit_state(name,
                                       metrics_emitter.travel_state(ahead)
                                       if metrics_emitter is not None
                                       else None)
                        elif item.kind == "dwell":
                            # Only a dwell AT A WORKSTATION is service, and
                            # only the first one of a visit starts it: a
                            # visit is sent as several DWELLs (2 s pause +
                            # 30 s chunks), and the delay-insertion plans add
                            # traffic waits as DWELLs on lattice nodes too.
                            here = last_node.get(name)
                            if self._ws_bay(here) is not None:
                                emit_state(name, "PROCESSING_HOLD")
                                if in_service.get(name) != here:
                                    in_service[name] = here
                                    self._memit("PROCESSING_STARTED",
                                                robot_id=name,
                                                workstation_id=here)
                            else:
                                emit_state(name, "TRAFFIC_WAIT",
                                           "planned wait")
                        if item.kind == "move" and \
                                self._ws_bay(item.from_node) is not None:
                            # LEAVING A WORKSTATION ends the visit: one
                            # PROCESSING_COMPLETED per visit, and the job
                            # closes here -- not on every DWELL, which closed
                            # two jobs per visit (2 s pause + 30 s chunk).
                            here = str(item.from_node)
                            in_service.pop(name, None)
                            jobs = getattr(self, "_jobs", None)
                            job = (jobs.complete_next(name)
                                   if jobs is not None else None)
                            self._memit("PROCESSING_COMPLETED", robot_id=name,
                                        workstation_id=here,
                                        job_id=job.job_id if job else None)
                            if job is not None and job.mode in ("pick",
                                                                "service"):
                                self._bay_event(job.workstation, "picked",
                                                name)
                            if job is not None:
                                self._memit(
                                    "PICKUP_COMPLETED" if job.mode == "pick"
                                    else "DROPOFF_COMPLETED",
                                    robot_id=name, job_id=job.job_id,
                                    workstation_id=job.workstation,
                                    details={"mode": job.mode,
                                             "grid_node": here})
                        log_release(name)
                        self.get_logger().info(
                            f"[{name}] PLAN t={now:.1f}s -> {item.kind} "
                            f"{item.from_node}->{item.to_node} "
                            f"({decision.reason})"
                            + (f" [fused with next move #{next_move_index}]"
                               if fuse_next else ""))

                    elif phase[name] == "waiting":
                        # Keep-alive: the firmware's STATE_WHEEL_FOLLOW
                        # watchdog (WHEEL_CMD_BRAKE_MS=300ms, see
                        # AGV_Factory_camera_correction.ino) brakes whenever
                        # it hasn't seen a fresh wheel_cmd in the last 300ms,
                        # then aborts at 1000ms -- it has
                        # no concept of "the supervisor is deliberately
                        # holding me here for another robot," so a robot
                        # legitimately WAITING on a resource/corridor for
                        # longer than 300ms applies its own safety brake and
                        # can still error if the hold stream remains absent.
                        # Confirmed on hardware 2026-08-05 (5-robot mission):
                        # every WHEEL_CMD_TIMEOUT in that run coincided
                        # exactly with a robot entering this "waiting" phase
                        # (blocked by another robot's reservation/corridor),
                        # never with active driving -- cancel_dispatch() (the
                        # only thing that ran before "waiting" was set) never
                        # sends anything, so the worker's PREVIOUS job had
                        # already fully completed and nothing refreshed
                        # wheel_cmd_last_ms after that. Re-send a zero-
                        # velocity hold every tick here (same command
                        # enter_wheel_follow_mode() sends once on rearm) --
                        # SUPERVISOR_WAIT_RECHECK_SEC=0.20s is already well
                        # under the 300ms brake threshold, so piggybacking on the
                        # existing recheck cadence needs no new timer. Only
                        # safe to call directly (not via the worker thread)
                        # because phase=="waiting" means the worker is
                        # guaranteed idle, not mid-job.
                        #
                        # ONLY TO A ROBOT STILL IN WHEEL_FOLLOW, AT 10 Hz
                        # (2026-10-22). Since 2026-08-20 every leg ends in
                        # stop_only(), so a waiting robot's firmware is IDLE
                        # and ignores wheel_cmd entirely -- this went out on
                        # every 20 ms loop pass to every waiting robot for
                        # nothing: ~250 msg/s through the micro-ROS agent while
                        # five robots queue in the depot. Alvik1's turns failed
                        # 3.6% of the time with no robot waiting, 10% with 1-2,
                        # 13% with 3-5 -- and Alvik1 leads every launch.
                        nav = worker.nav
                        if (str(getattr(nav, "last_status", "")).startswith(
                                "BUSY WHEEL_FOLLOW")
                                and now - keepalive_at.get(name, -1.0) >= 0.1):
                            nav.send_wheel(0.0, 0.0)
                            keepalive_at[name] = now
                        if now < next_gate_at[name]:
                            continue
                        phase[name] = "idle"

                    elif phase[name] == "busy":
                        result = worker.poll()
                        if result is None:
                            if now - st_dispatched(advisor, name) > timeout_sec:
                                self.get_logger().error(
                                    f"[{name}] vision leg timed out "
                                    f"({timeout_sec:.0f}s); robot excluded")
                                self._event(
                                    "error",
                                    f"vision leg timed out after "
                                    f"{timeout_sec:.0f}s; excluded",
                                    robot=name, severity="error")
                                worker.abort()
                                ch.errored = True
                                failed_cmd = inflight.pop(name, None)
                                self._memit(
                                    "COMMAND_FAILED", robot_id=name,
                                    command_id=failed_cmd[1] if failed_cmd else None,
                                    reason="COMMAND_TIMEOUT")
                                self._memit("ROBOT_OFFLINE", robot_id=name,
                                            reason="vision leg timeout; excluded")
                                emit_state(name, "FAULT_RECOVERY")
                                # No reservation of ANY kind may outlive the
                                # failed job -- nothing else will ever release
                                # it. strand() also keeps the node reservation,
                                # since the robot is still sitting on it.
                                advisor.strand(name, now)
                            continue
                        ok, note, fused = result
                        if not ok:
                            key = (name, advisor.robots[name].next_index)
                            leg_attempts[key] = leg_attempts.get(key, 0) + 1
                            tries = leg_attempts[key]
                            if tries <= max_leg_retries:
                                # Retry, do not exclude. cancel_dispatch()
                                # not strand(): this releases the movement
                                # grant so the re-dispatch re-requests it
                                # cleanly through the ordinary reservation
                                # path, while leaving the robot a live
                                # participant. next_index has not advanced
                                # (record_done never ran), so going back to
                                # "idle" re-dispatches THIS leg.
                                self.get_logger().warning(
                                    f"[{name}] vision leg failed ({note}); "
                                    f"retry {tries}/{max_leg_retries}")
                                self._event(
                                    "recovery",
                                    f"vision leg failed ({note}); "
                                    f"retry {tries}/{max_leg_retries}",
                                    robot=name, severity="warning")
                                self._memit(
                                    "COMMAND_FAILED", robot_id=name,
                                    command_id=(inflight.get(name) or (None, None))[1],
                                    reason="CONTROL_ERROR_RETRY",
                                    details={"note": note, "attempt": tries})
                                inflight.pop(name, None)
                                fused_index[name] = None
                                advisor.cancel_dispatch(name)
                                emit_state(name, "FAULT_RECOVERY")

                                # RECONCILE POSITION BEFORE RETRYING.
                                #
                                # The worker's job is "drive this leg AND turn
                                # toward the next one", so a failure in the
                                # trailing turn discards the completed drive:
                                # record_done never runs and current_node stays
                                # on the leg's ORIGIN while the robot is
                                # physically at its DESTINATION.
                                #
                                # _vision_hold_reason() compares vision against
                                # current_node, so the retry is then refused as
                                # off-route -- by exactly one grid cell -- and
                                # that gate has no timeout. Observed 2026-09-25
                                # run 4: Alvik5's turn at node 1 failed, the
                                # retry logged "off-route — measured
                                # (0.01,-0.08) vs expected node 0 (1.27 cells)"
                                # about eighty times, and the whole fleet
                                # deadlocked behind it until a manual abort.
                                #
                                # If vision puts the robot at the destination,
                                # the drive really did finish: bank it, so the
                                # retry resumes from where the robot IS. The
                                # next item's own square-up re-does the turn.
                                st_r = advisor.robots[name]
                                item_r = (st_r.plan[st_r.next_index]
                                          if st_r.next_index < len(st_r.plan)
                                          else None)
                                banked = False
                                if (item_r is not None and item_r.kind == "move"
                                        and self.vision is not None
                                        and depot_label(item_r.to_node)):
                                    # DEPOT NODES have no grid cell, so the
                                    # cell test below could never bank them
                                    # and the resync could never place the
                                    # robot: Alvik1 arrived at DE1, its turn
                                    # failed, the advisor kept it at D1, and
                                    # DE1 was granted to Alvik2, which drove
                                    # into it (2026-10-22). World inches.
                                    if (arrived_world_in(self.vision, name,
                                                         item_r.to_node)
                                            and advisor.bank_completed_move(
                                                name, now)):
                                        banked = True
                                        self.get_logger().warning(
                                            f"[{name}] drive to "
                                            f"{item_r.to_node} had completed; "
                                            "banking it so the retry is not "
                                            "refused as off-route")
                                elif (item_r is not None and item_r.kind == "move"
                                        and self.vision is not None):
                                    mine = self.vision.snapshot().get(name)
                                    dest = label_to_cell(item_r.to_node,
                                                         advisor.rows,
                                                         advisor.cols)
                                    # ARRIVED means at the node, not within
                                    # the 0.75-cell off-route band: on
                                    # 2026-10-16 Alvik3, guard-stopped 4.69in
                                    # short mid-leg, was "banked" at 14.
                                    # And bank_completed_move(), not
                                    # record_done(): cancel_dispatch() above
                                    # has already cleared st.pending, and
                                    # record_done() returns at once without
                                    # it -- so this logged "banking it" and
                                    # recorded NOTHING, every time. The robot
                                    # stayed on its origin node in the plan,
                                    # was excluded as off-route (Alvik4, this
                                    # morning) or was sent on a yield from
                                    # the wrong node (Alvik1 13->5 from entry
                                    # 118, ending in a collision).
                                    if (mine and mine[3] <= VISION_STALE_SEC
                                            and dest is not None
                                            and math.hypot(mine[0] - dest[0],
                                                           mine[1] - dest[1])
                                            <= ARRIVAL_BANK_CELLS
                                            and advisor.bank_completed_move(
                                                name, now)):
                                        banked = True
                                        self.get_logger().warning(
                                            f"[{name}] drive to "
                                            f"{item_r.to_node} had completed; "
                                            "banking it so the retry is not "
                                            "refused as off-route")
                                if not banked and self.vision is not None:
                                    # Not at the destination. Make the
                                    # advisor's idea of where the robot is
                                    # match vision anyway, or the same gate
                                    # refuses the retry. finish_yield_leg()
                                    # is exactly the operation wanted here --
                                    # set current_node, clear the edge, and
                                    # re-grant a standing reservation on the
                                    # node actually occupied -- despite the
                                    # name, which comes from its first caller.
                                    mine = self.vision.snapshot().get(name)
                                    if mine and mine[3] <= VISION_STALE_SEC:
                                        here = node_at_cell(mine[0], mine[1],
                                                            advisor.rows,
                                                            advisor.cols,
                                                            tol=0.6)
                                        if here is None:
                                            # Below the grid: the depot lane
                                            # and slots, by world position.
                                            here = nearest_depot_label(
                                                self.vision, name)
                                        if (here is not None
                                                and str(here) != str(
                                                    st_r.current_node)):
                                            self.get_logger().warning(
                                                f"[{name}] advisor had it at "
                                                f"{st_r.current_node}, vision "
                                                f"says {here}; resyncing "
                                                "before retry")
                                            advisor.finish_yield_leg(
                                                name, here, now)

                                phase[name] = "idle"
                                next_gate_at[name] = now + LEG_RETRY_PAUSE_SEC
                                continue
                            self.get_logger().error(
                                f"[{name}] vision leg failed ({note}) after "
                                f"{max_leg_retries} retries; robot excluded")
                            self._event("error",
                                        f"vision leg failed ({note}); excluded "
                                        f"after {max_leg_retries} retries",
                                        robot=name, severity="error")
                            ch.errored = True
                            failed_cmd = inflight.pop(name, None)
                            self._memit(
                                "COMMAND_FAILED", robot_id=name,
                                command_id=failed_cmd[1] if failed_cmd else None,
                                reason="CONTROL_ERROR", details={"note": note})
                            self._memit("ROBOT_OFFLINE", robot_id=name,
                                        reason=f"vision leg failed: {note}")
                            emit_state(name, "FAULT_RECOVERY")
                            # Unconditional, and strand() not cancel_dispatch():
                            # the old `if fused_index[name] is not None` guard
                            # meant a plain drive_leg failure released nothing,
                            # so this robot's corridor reservations outlived it
                            # and deadlocked everyone behind it. See strand().
                            advisor.strand(name, now)
                            continue
                        if fused and fused_index[name] is not None:
                            st = advisor.robots[name]
                            current_idx = st.pending[1] if st.pending else None
                            if current_idx is not None:
                                advisor.record_done_fused(
                                    name, current_idx, fused_index[name], now)
                            else:
                                advisor.record_done(name, now)
                        else:
                            advisor.record_done(name, now)
                        # metrics stream (additive)
                        done_cmd = inflight.pop(name, None)
                        if done_cmd is not None:
                            done_item, done_cmd_id = done_cmd
                            self._memit("COMMAND_COMPLETED", robot_id=name,
                                        command_id=done_cmd_id)
                            if done_item.kind == "move" and \
                                    done_item.to_node is not None:
                                last_node[name] = str(done_item.to_node)
                                # A drop (or a service visit's drop) at this
                                # bay happens on ARRIVAL: start its part's
                                # processing clock -- see _rpd_hold().
                                bay = self._ws_bay(done_item.to_node)
                                jobs = getattr(self, "_jobs", None)
                                nxt = (jobs.next_pending(name)
                                       if jobs is not None else None)
                                if bay is not None and nxt is not None and \
                                        int(nxt.workstation) == bay and \
                                        nxt.mode in ("drop", "service"):
                                    drops = getattr(self, "_drop_arrival", {})
                                    if bay not in drops:
                                        drops[bay] = now
                                        # Processing starts HERE unless the 2026-10-29
                                        # drop rule defers it to the end of the DWELL.
                                        if self._rpd_process_sec is None:
                                            self._bay_event(bay, "delivered", name)
                                        elif not (getattr(self, "_rpd_after_drop", False)
                                                  and nxt.mode == "drop"):
                                            self._bay_event(bay, "processing", name)
                            elif done_item.kind == "dwell" and \
                                    getattr(self, "_rpd_after_drop", False):
                                # Drop rule (operator, 2026-10-29): the drop
                                # is the DWELL after the robot squares up in
                                # the bay; the workstation's processing clock
                                # starts when that DWELL ends, and the robot
                                # is then free to leave.
                                bay = self._ws_bay(last_node.get(name))
                                jobs = getattr(self, "_jobs", None)
                                nxt = (jobs.next_pending(name)
                                       if jobs is not None else None)
                                if bay is not None and nxt is not None and \
                                        int(nxt.workstation) == bay and \
                                        nxt.mode == "drop" and bay not in self._drop_done:
                                    self._drop_done[bay] = now
                                    self._bay_event(bay, "processing", name)
                        fused_index[name] = None
                        phase[name] = "idle"
                        for waiting_name, waiting_phase in phase.items():
                            if waiting_phase == "waiting":
                                next_gate_at[waiting_name] = 0.0

                _time.sleep(0.02)

            done = [n for n, p in phase.items() if p == "done"]
            errored = [n for n, ch in self._channels.items() if ch.errored]
            self.get_logger().info(
                f"Advisor run finished (position-target/{localization_mode}). "
                f"Complete: {done or 'none'}; "
                f"errored: {errored or 'none'}.")
            end_battery = self._battery_report()
            self.get_logger().info(f"battery at end:   {end_battery}")
            for end_name, end_ch in self._channels.items():
                if end_ch.battery_pct is not None:
                    self._memit("BATTERY_SAMPLE", robot_id=end_name,
                                battery_pct=end_ch.battery_pct)
            # RUN STATUS IS THE MISSION OUTCOME, NOT THE ERROR FLAG.
            #
            # This used to be `if errored:` alone, so ANY robot fault marked
            # the whole run failed even when every job had been delivered.
            # Two runs in the database are wrong because of it (2026-09-24
            # 11617f6e and 18e5f0f8): 12/12 jobs, 333/332 and 316/315
            # commands -- a single failed command each, and in both cases it
            # was the LAST one, a cosmetic square-up at the robot's own depot
            # after all its work was done. A run that delivered everything it
            # was asked to deliver is not a failed run, and recording it as
            # one corrupts the completion-rate statistic these runs exist to
            # measure.
            #
            # Nothing is hidden by this: the errored list rides along in
            # details either way, and the COMMAND_FAILED / ROBOT_OFFLINE
            # events remain exactly as before, so a run that finished its work
            # with a fault is still fully reconstructable.
            jobs_reg = getattr(self, "_jobs", None)
            outstanding = ([j.job_id for j in jobs_reg.jobs.values()
                            if j.status != "done"]
                           if jobs_reg is not None else None)
            # No job registry (no schedule, or color mode) means there is no
            # objective to measure, so fall back to the old flag rather than
            # silently calling every such run complete.
            failed = (bool(errored) if outstanding is None
                      else bool(outstanding))
            common = {"complete": list(done),
                      "errored": list(errored),
                      "jobs_outstanding": outstanding,
                      "jobs_total": (len(jobs_reg.jobs)
                                     if jobs_reg is not None else None),
                      "drive_mode": "vision",
                      "localization_mode": localization_mode}
            self._memit("RUN_FAILED" if failed else "RUN_COMPLETED",
                        details=common)
            if errored and not failed:
                self.get_logger().warning(
                    f"run COMPLETED with faults: every job delivered, but "
                    f"{sorted(errored)} errored after finishing their work")
            self._event("mission",
                        f"run finished (position-target/{localization_mode}) — complete: "
                        f"{done or 'none'}; errored: {errored or 'none'}; "
                        f"battery {end_battery}",
                        severity="error" if errored else "info")
            self.publish_fleet_status("finished", advisor, phase,
                                      note=f"complete={done} errored={errored}")
        finally:
            for w in workers.values():
                w.shutdown()


def st_dispatched(advisor, name: str) -> float:
    """dispatched_sec for the robot's current pending plan item, or -inf if
    none (never times out a leg that was never actually handed off)."""
    st = advisor.robots.get(name)
    if st is None or st.dispatched_sec is None:
        return -math.inf
    return st.dispatched_sec


def _leg_note(nav, what: str) -> str:
    """Human note plus the navigator's own failure classification.

    Reads nav.last_failure, set by CameraGridNavigator.fail() at each
    failure site. Produces e.g.

        STALLED_IDLE: drive_leg failed on leg 26->18

    so the Decision Log, the event stream and the metrics row all say WHY,
    not just that something failed. Five distinct causes used to share the
    single label "vision leg failed" -- stopped-short-and-IDLE, blocked-
    while-driving, mode-ack-lost, settle-failed and off-route all looked
    identical from outside.

    Cleared after reading so a later failure cannot inherit a stale reason;
    a failure with nothing recorded says UNKNOWN rather than lying.
    """
    reason = getattr(nav, "last_failure", None) or "UNKNOWN"
    try:
        nav.last_failure = None
    except Exception:
        pass
    return f"{reason}: {what}"


class VisionLegWorker:
    """One robot's dedicated CameraGridNavigator + spin thread, driven by a
    single-slot job queue: run_advised_vision() submit()s one PlanItem at a
    time and poll()s for (ok, note) once drive_leg()/
    turn_to_heading_rotate_rel()/dwell() returns. A CameraGridNavigator is a
    full rclpy.Node with its own <name>_vision_pose subscription — running N
    of them alongside
    FleetSupervisor's own node in one process is ordinary ROS2 (separate
    nodes, separate executors), not a new pattern; camera_grid_navigate.py
    already proved every one of these primitives on real hardware, this
    class only adds the plan-item-at-a-time handoff."""

    def __init__(self, name: str, supervisor: "FleetSupervisor",
                 localization_mode: str = "camera_assist",
                 turn_method: str | None = None) -> None:
        from camera_grid_navigate import CameraGridNavigator
        import argparse as _argparse

        self.name = name
        self.supervisor = supervisor      # for await_rotation_clearance()
        self.localization_mode = normalize_localization_mode(localization_mode)
        args = _argparse.Namespace(
            verbose=False, rows=8, cols=8,
            localization_mode=self.localization_mode,
            # Production turns use turn_to_heading_rotate_rel(): one fast
            # firmware-controlled alvik.rotate(), followed in camera-capable
            # modes by at most one camera correction. 4deg is the accepted
            # final/correction tolerance. The streamed-turn parameters below
            # remain only for disabled bench/fusion code paths.
            turn_tol_deg=4.0, turn_settle_count=3, turn_min_speed=20.0,
            turn_decel_zone_deg=35.0, turn_creep_speed=4.0,
            turn_scale_deg=60.0, turn_max_speed=35.0,
            # turn_then_drive_leg()-ONLY params (DISABLED feature, see
            # run_advised_vision()'s docstring) -- kept only because that
            # method is still reachable code and references these names.
            turn_rpm=35.0, turn_brake_lead_deg=30.0, turn_creep_rpm=10.0,
            # 1.38, re-measured 2026-09-25 by --stop-test: the old 1.9 was
            # landing 0.52in short on every leg. See --brake-lead-in's help.
            cruise_rpm=60.0, brake_lead_in=1.38,
            # kp_yaw 1.75, not 0.6: measured 2026-09-24 against cross-track
            # deviation. drive_leg() has no cross-track term, so a leg entered
            # mis-aimed bows off the tape and a higher gain removes that error
            # sooner. See camera_grid_navigate.py's --kp-yaw help.
            kp_yaw=1.625, max_turn_adjust=10.0,
            # Staged turn (default since 2026-09-24). One wheel-speed
            # paradigm for the whole route in three speeds, replacing both
            # alvik.rotate() and the tapered law: 1.35deg worst case and
            # ~1.3s per 90deg, measured on all six robots. See
            # turn_to_heading_staged() and FINDINGS_ROTATION_2026-09-24.md.
            # AGV_TURN_METHOD=rotate_rel or =wheel reverts fleet-wide without
            # a code change.
            turn_method="staged",
            staged_fast_rpm=70.0, staged_slow_rpm=25.0, staged_final_rpm=6.0,
            staged_handoff1_deg=65.0, staged_handoff2_deg=45.0,
            staged_lead_deg=2.71,
            # 2026-09-25, FINDINGS_TURN_2026-09-25.md. The handoff1/handoff2
            # ladder above is now inert: staged_approach_kp replaces it with a
            # continuous taper, because 1156 turns of supervisor_live.log
            # showed the ladder never executes once a turn reaches 70 RPM, and
            # that 19% of those turns finished over the 5deg bay-collision
            # threshold while logging success. Set staged_approach_kp=0.0 to
            # restore the ladder fleet-wide. staged_kick_* turns the stall that
            # produced 10 of the 12 vision leg failures into a recovery.
            # staged_approach_kp=0.0 -> the handoff ladder above is LIVE and
            # the taper is off. The taper was tried on hardware 2026-09-25 and
            # made the thing it was meant to fix worse: every robot finished
            # its ~180deg depot square-up 11-16deg off. Accuracy is no longer
            # the coarse turn's job at all -- settle_heading() re-approaches
            # from a standstill afterwards and holds final_yaw_tol_deg, which
            # is the one regime that measures accurate (0 of 311 small turns
            # from rest exceeded 5deg). See FINDINGS_TURN_2026-09-25.md.
            staged_approach_kp=0.0,
            staged_kick_rpm=6.0, staged_kick_max=3,
            staged_stall_accept_deg=5.0,
            # Enforced at workstation and depot arrivals: the orientations
            # that cause collisions when they are wrong.
            final_yaw_tol_deg=2.0,
            settle_rpm=6.0, settle_rpm_min=4.0, settle_lead_deg=4.0,
            settle_rpm_step=4.0, settle_progress_deg=0.5,
            settle_moved_eps_deg=0.3,
            settle_rpm_max=18.0, settle_max_passes=4,
            settle_pass_timeout_sec=4.0, settle_no_progress_sec=0.8,
            settle_samples=9, settle_margin_deg=0.6, settle_noise_warn_deg=0.5,
            settle_hard_fail_deg=5.0,
            staged_timeout_sec=15.0, staged_no_progress_sec=1.2,
        )
        # This robot's own brake lead and steering trim, measured from real
        # legs (robot_drive_calibration.json, tools/analyze_stops.py). The
        # fleet-wide 1.38in lead stopped Alvik5/6 0.15in past and Alvik1/2/4
        # 0.05in short, and Alvik3 drove every leg bowed 0.5in to its left.
        args.steer_trim_rpm = 0.0
        cal = robot_drive_calibration().get(name, {})
        for key in ("brake_lead_in", "steer_trim_rpm"):
            if isinstance(cal.get(key), (int, float)):
                setattr(args, key, float(cal[key]))
        # Where this robot rests beside its depot slot / workstation targets,
        # fed forward by drive_leg (see aim_corrected_target()).
        aim = cal.get("aim_lateral_in")
        args.aim_lateral_in = dict(aim) if isinstance(aim, dict) else {}
        along = cal.get("aim_along_in")
        args.aim_along_in = dict(along) if isinstance(along, dict) else {}
        # Chosen per mission by the Run tab's Drive mode buttons: "Camera
        # only" sends staged, "Camera + Encoder" sends rotate_rel. None
        # leaves AGV_TURN_METHOD / turn_method above in charge. It outranks
        # the launch-line override -- see _turn_to_heading_core().
        args.mission_turn_method = turn_method
        # Hand the tuning to the metrics emitter before the navigator is
        # built, so the run records the parameters it actually executed
        # under -- defaults plus any env/CLI override. Guarded: a metrics
        # import problem must never stop a robot being constructed.
        try:
            from metrics_events import metrics_emitter as _me
            _me.remember_nav_args(args)
        except Exception:
            pass
        self.nav = CameraGridNavigator(name, args)
        self._job: tuple | None = None
        self._result: tuple[bool, str] | None = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        # Set by abort(): a rotation hold with no deadline polls it, so a
        # robot excluded mid-hold does not turn later on its own.
        self._cancel = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name=f"vision-leg-{name}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit(self, item, next_move=None, fuse_next: bool = False) -> None:
        """fuse_next: True only when the supervisor already got a real
        reservation grant from advisor.try_authorize_next_move() for
        next_move -- tells _execute() to hand off via turn_then_drive_leg()
        instead of the safe target-gated
        turn_to_heading_rotate_rel()+drive_leg() pair. The caller MUST have
        already granted (not just checked) before setting this; the worker
        does not re-verify collision safety."""
        with self._lock:
            self._result = None
            self._job = (item, next_move, fuse_next)
            self._cancel.clear()
        self._wake.set()

    def poll(self) -> tuple[bool, str, bool] | None:
        """Returns (ok, note, fused) once done -- fused True means next_move
        was ALSO completed as part of this call (the caller must advance the
        advisor with record_done_fused(), not record_done())."""
        with self._lock:
            return self._result

    def abort(self) -> None:
        self._cancel.set()
        self.nav.send_cmd("STOP")

    def _idle(self, seconds: float) -> None:
        """Wait while keeping this robot's navigator reading its topics.

        One spin_once handles one callback, and odometry plus camera pose
        arrive at ~70 msg/s together, so spin in short slices."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.nav._spin_once(0.01)

    def _turn(self, heading: float, label: str, tol_deg=None) -> bool:
        """The gated turn itself, releasing the rotation claim when it ends.

        await_rotation_clearance() claims the rotation for ROTATION_CLAIM_SEC
        so the guard and corridor gate see it before the first frame does;
        without a release the claim outlived the turn. 2026-10-16: Alvik1's
        4.7deg turn at 18 took 0.46s, but its claim held Alvik3 in bay 73 --
        "projects 0.48 cells from requested edge 73->122" -- after it."""
        try:
            return self.nav.turn_to_heading_rotate_rel(heading, label,
                                                       tol_deg=tol_deg)
        finally:
            release = getattr(getattr(self.supervisor, "vision", None),
                              "release_rotation", None)
            if release is not None:
                release(self.name)

    def _ready_to_depart(self, frm, x0: float, y0: float, x1: float,
                         y1: float, leg_label: str,
                         heading_between) -> tuple[bool, str]:
        """Two checks before ANY drive leg starts. (ok, why).

        ON THE LINE. drive_leg() steers toward the target from wherever the
        robot is, so a leg begun off its own line is a diagonal drive across
        the grid. 2026-10-16: Alvik4 was resynced to node 29 and the retry
        of "137->88" set off from there, 13in off that leg, and was stopped
        beside Alvik3. Off by more than LEG_START_LATERAL_IN: refuse; the
        retry/exclusion path then stops the robot instead.

        POINTED DOWN IT. drive_leg() only trims heading. Planned moves are
        aimed by the previous item's turn, but nothing re-aims a robot whose
        trailing turn FAILED and whose drive was then banked -- 2026-10-16,
        Alvik4 at entry 137 still facing east set off for bay 88, arced off
        the tape and drove a circle. More than DEPART_ALIGN_DEG off: make
        the (gated) turn first. Aimed robots are within 4deg, so normal
        moves are untouched.
        """
        nav = self.nav
        cam = nav.fresh_pose()
        if cam is not None:
            dx, dy = x1 - x0, y1 - y0
            denom = dx * dx + dy * dy
            if denom > 1e-9:
                t = max(0.0, min(1.0, ((cam.x - x0) * dx + (cam.y - y0) * dy)
                                 / denom))
                off = math.hypot(cam.x - (x0 + t * dx), cam.y - (y0 + t * dy))
                if off > LEG_START_LATERAL_IN:
                    nav.stop_only()
                    return False, (
                        f"OFF_PATH: {off:.1f}in off the line of {leg_label} "
                        f"at ({cam.x:.1f},{cam.y:.1f}); refusing to drive "
                        "across the grid")
        depart = heading_between(x0, y0, x1, y1)
        pose = nav.control_pose()
        if pose is None:
            self._idle(0.3)          # first job after a wait: read topics
            pose = nav.control_pose()
        if pose is None:
            return True, "heading unknown; drive_leg decides"
        err = abs((depart - pose.yaw + 180.0) % 360.0 - 180.0)
        if err <= DEPART_ALIGN_DEG:
            return True, "aimed"
        self.supervisor.get_logger().warning(
            f"[{self.name}] {leg_label}: facing {pose.yaw:+.0f}deg, leg needs "
            f"{depart:.0f}deg ({err:.0f}deg off) -- turning before driving")
        ok, why = self._clear_to_rotate(depart, f"{leg_label}: departure",
                                        timeout_sec=ROTATION_CLEAR_TIMEOUT_SEC,
                                        at_node=frm)
        if not ok:
            return False, f"{why} (departure turn)"
        if not self._turn(depart, f"{leg_label}: departure turn"):
            return False, f"departure turn failed on {leg_label}"
        return True, "turned"

    def _straight_trim_skip(self, at_node, incoming: float, next_heading: float,
                            label: str) -> bool:
        """True: skip this turn and go on to the next leg as aimed now.

        Only for a route going STRAIGHT ON through `at_node` (the plan makes
        no turn there), whose remaining "turn" is a trim of arrival drift
        above turn_tol_deg but within STRAIGHT_TRIM_SKIP_MAX_DEG, that the
        rotation gate would hold right now, and only if straight_trim_clear()
        shows the unsteered drive keeps the guard's margin. drive_leg()
        steers the trim out on the way; _ready_to_depart() still turns a
        robot more than DEPART_ALIGN_DEG off. Everything else waits for the
        gate as before.
        """
        if abs((incoming - next_heading + 180.0) % 360.0 - 180.0) > 1.0:
            return False                         # a real turn
        return self._blocked_trim_skip(at_node, next_heading, label,
                                       "on a straight route")

    def _blocked_trim_skip(self, at_node, heading: float, label: str,
                           why: str, length_in: float | None = None) -> bool:
        """True: the turn to `heading` is only a trim (above turn_tol_deg,
        within STRAIGHT_TRIM_SKIP_MAX_DEG), the rotation gate would hold it
        right now, and the unsteered drive ahead keeps the guard's margin --
        so drive on and let the leg steer it out."""
        pose = self.nav.control_pose()
        if pose is None:
            return False
        err = abs((heading - pose.yaw + 180.0) % 360.0 - 180.0)
        if (err <= self.nav.args.turn_tol_deg
                or err > STRAIGHT_TRIM_SKIP_MAX_DEG):
            return False
        blocked = self.supervisor.rotation_blocker(self.name, at_node)
        if blocked is None:
            return False                         # free to trim: trim
        if not self.supervisor.straight_trim_clear(self.name, pose.yaw,
                                                   length_in=length_in):
            return False
        self.supervisor.get_logger().info(
            f"[{self.name}] {label}: skipping a {err:.1f}deg trim {why} "
            f"({blocked[1]}); the leg steers it out")
        return True

    def _clear_to_rotate(self, heading: float, label: str,
                         timeout_sec: float | None,
                         at_node=None) -> tuple[bool, str]:
        """Gate the rotation about to happen at `label` -- if one will.

        The navigator skips a turn that is already within turn_tol_deg of
        `heading` (_turn_to_heading_core), and a turn that does not happen
        sweeps nothing. The gate used to be consulted first regardless, so
        "turn at X: already within 4.0 deg ... skipping turn" could follow a
        hold of up to 20s for that same non-turn. With no pose to judge by,
        assume the turn happens and gate it.
        """
        pose = self.nav.control_pose()
        if pose is not None:
            err = abs((heading - pose.yaw + 180.0) % 360.0 - 180.0)
            if err <= self.nav.args.turn_tol_deg:
                return True, "no rotation needed"
        return self.supervisor.await_rotation_clearance(
            self.name, label, timeout_sec=timeout_sec,
            cancelled=self._cancel.is_set, at_node=at_node,
            idle=self._idle)

    def shutdown(self) -> None:
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=5.0)
        try:
            self.nav.destroy_node()
        except Exception:
            pass

    def _run(self) -> None:
        from dispatch_advisor import movement_segments
        from camera_grid_navigate import heading_between

        # Wait only for the streams selected by this experiment. Encoder-only
        # deliberately has no camera dependency; camera-only deliberately has
        # no onboard-odometry dependency.
        setup_failure = self.nav.wait_for_localization_ready()
        if setup_failure is None and mode_requires_odom(self.localization_mode):
            if not self.nav.reset_onboard_pose(
                    f"localization={self.localization_mode} mission start"):
                setup_failure = "onboard pose reset failed at mission start"
            elif not self.nav.wait_for_fresh_odom(timeout_sec=3.0):
                setup_failure = "no fresh onboard pose after mission-start reset"
        if setup_failure is None:
            self.nav.get_logger().info(
                f"localization={self.localization_mode} streams ready")

        # Motion primitives arm WHEEL_FOLLOW_MODE immediately before they
        # begin streaming. Keeping the firmware armed while the supervisor is
        # waiting at a launch gate, collision gate, or dwell would trip its
        # short wheel-command watchdog.

        while not self._stop.is_set():
            if not self._wake.wait(timeout=0.1):
                self.nav._spin_once(0.0)
                continue
            self._wake.clear()
            if self._stop.is_set():
                break
            with self._lock:
                job = self._job
                self._job = None
            if job is None:
                continue
            item, next_move, fuse_next = job
            if setup_failure is not None:
                ok, note, fused = False, setup_failure, False
            else:
                ok, note, fused = self._execute(
                    item, next_move, fuse_next, movement_segments, heading_between)
            with self._lock:
                self._result = (ok, note, fused)

        # A leg thread finishing after an abort has already destroyed the
        # node publishes into a dead handle. The robots are stopped either
        # way -- the abort broadcast STOP before tearing down -- so this is
        # a shutdown race, not a control failure, and it should not surface
        # as a thread traceback that buries the real reason the run ended.
        try:
            self.nav.send_cmd("STOP")
            for _ in range(5):
                self.nav._spin_once(0.05)
        except Exception as exc:
            if type(exc).__name__ != "InvalidHandle":
                raise

    def _execute(self, item, next_move, fuse_next, movement_segments,
                heading_between) -> tuple[bool, str, bool]:
        """next_move: the next "move"-kind PlanItem in this robot's plan
        (skipping over turn/dwell/noop items, which carry no from/to of
        their own), or None if this is the robot's last move. fuse_next:
        True only when the supervisor already holds a real advisor
        reservation for next_move (see VisionLegWorker.submit()) -- when
        True, the turn toward next_move's heading hands off DIRECTLY into
        driving that leg (turn_then_drive_leg(), no brake/settle in
        between); when False (the default/safe path), it executes and settles
        a target-gated turn via turn_to_heading_rotate_rel(), and next_move's
        own drive_leg() happens on a LATER call to this method once the
        advisor separately dispatches it.

        Returns (ok, note, fused) -- fused True means next_move was ALSO
        driven as part of this call; the caller must advance the advisor
        with record_done_fused(), not record_done().

        Mirrors camera_grid_navigate.py's safety intent: (1) drive_leg to the
        target, (2) preserve required arrival orientation at a workstation,
        depot, or route endpoint, and (3) turn directly toward the NEXT leg's
        heading at an ordinary transit node. The following drive_leg() call
        only applies a small proportional heading correction, not a real
        turn, so it must start already facing roughly the right way. Confirmed on hardware
        2026-07-30: omitting step (3) let a robot leave DE1 for node 0 still
        facing DE1's own approach heading; drive_leg's weak correction
        couldn't close that gap and it drove off the table."""
        nav = self.nav
        rows, cols = nav.args.rows, nav.args.cols
        try:
            if item.kind == "dwell":
                ms = None
                parts = item.command.split()
                if len(parts) > 1:
                    try:
                        ms = int(parts[1])
                    except ValueError:
                        ms = None
                if not nav.dwell(ms, f"dwell ({item.command})"):
                    return False, _leg_note(nav, "dwell failed"), False
                # Turn toward the NEXT leg AFTER the dwell, same as every
                # move item's own step (3) above -- REAL BUG confirmed on
                # hardware 2026-07-31: a workstation dwell was followed
                # immediately by the next move's drive_leg() with NO turn in
                # between (Alvik1 stayed facing the depot-approach heading
                # it arrived at the workstation with, then drove straight
                # back out the wrong way and collided with Alvik2). Mirrors
                # camera_grid_navigate.py's run(), which performs this turn
                # unconditionally right after a dwell at the SAME node,
                # never deferring it to the next move's own dispatch -- this
                # method previously had no equivalent for the dwell case at
                # all (only move items ever computed the departure turn).
                # next_move.from_node is the node just dwelt at (the plan is
                # sequential: whatever move follows a dwell always departs
                # from the node the dwell happened at).
                if next_move is not None:
                    fx, fy = nav_node_to_world(next_move.from_node, rows, cols)
                    tx, ty = nav_node_to_world(next_move.to_node, rows, cols)
                    heading = heading_between(fx, fy, tx, ty)
                    if (mode_requires_odom(nav.localization_mode)
                            and not nav.ensure_odom_anchor(fx, fy, heading)):
                        return False, (
                            "no fresh onboard odometry to anchor turn after dwell"
                        ), False
                    # A dwell can end anywhere a planned wait was inserted,
                    # bay entries included -- on 2026-10-16 Alvik2 turned
                    # 92deg at entry 137 after one, the one place on the
                    # table a turn cannot clear an adjacent robot. (Leaving
                    # a workstation needs no turn: its 180deg arrival
                    # square-up already faces the exit.) Ungated until
                    # 2026-10-16. No deadline: the dwell is done, and
                    # failing here would re-run it.
                    ok, why = self._clear_to_rotate(
                        heading,
                        f"turn after dwell toward "
                        f"{next_move.from_node}->{next_move.to_node}",
                        timeout_sec=None, at_node=next_move.from_node)
                    if not ok:
                        return False, f"{why} (turn after dwell)", False
                    if not self._turn(
                            heading,
                            f"turn after dwell toward "
                            f"{next_move.from_node}->{next_move.to_node}"):
                        return False, "turn after dwell failed", False
                return True, "ok", False

            if item.kind in ("turn", "noop"):
                # Standalone turn/no-op plan items (RIGHT_UNTIL_COLOR/
                # LEFT_UNTIL_COLOR/ROTATE_180/STOP/RESET_POSE/GET_STATUS in
                # the color-sensor vocabulary) carry no heading info of their
                # own in vision mode -- turning toward the next leg is
                # handled by the preceding move item's own step (3) above,
                # so there is nothing for a bare turn/noop item to do here.
                return True, "ok", False

            if item.kind != "move":
                return True, "ok", False

            segs = movement_segments(item)
            last_to = None
            for seg_index, (frm, to, _dur) in enumerate(segs):
                x0, y0 = nav_node_to_world(frm, rows, cols)
                x1, y1 = nav_node_to_world(to, rows, cols)
                leg_label = f"leg {frm}->{to}"
                # YIELD_OUT_REV: this hop is driven TAIL FIRST -- a robot with
                # no room to turn and nothing free ahead (2026-10-30). It
                # faces the opposite way to its travel the whole hop.
                rev = (str(item.command) == "YIELD_OUT_REV" and seg_index == 0)
                facing = ((heading_between(x0, y0, x1, y1) + 180.0) % 360.0
                          if rev else heading_between(x0, y0, x1, y1))
                # A yield is the ONE move that cannot assume the robot is
                # already pointing at it. Planned moves are always preceded
                # by the previous item's step (3) turn toward the next leg,
                # which is why drive_leg() only trims heading rather than
                # turning. A yield is injected off-plan, so the robot is
                # still squared up for the leg it was ABOUT to drive.
                # 2026-09-09, real hardware: Alvik5 held node 3 facing north
                # for 3->11, was told to yield west to node 2, and arced into
                # Alvik3 because nothing turned it first. Turn, then drive.
                if rev:
                    # No departure turn: the tail already points at `to`
                    # (_drives_straight checked it within 10deg, and the
                    # reverse leg steers the rest out).
                    if (mode_requires_odom(nav.localization_mode)
                            and not nav.ensure_odom_anchor(x0, y0, facing)):
                        return False, (
                            f"no fresh onboard odometry to anchor the "
                            f"reverse hop {leg_label}"), False
                elif str(item.command).startswith("YIELD"):
                    depart = heading_between(x0, y0, x1, y1)
                    if (mode_requires_odom(nav.localization_mode)
                            and not nav.ensure_odom_anchor(x0, y0, depart)):
                        return False, (
                            f"no fresh onboard odometry to anchor the "
                            f"departure turn for {leg_label}"), False
                    # A yield is injected off-plan by the deadlock resolver
                    # and skips the normal gates on purpose ("the worker
                    # does not re-verify collision safety", submit()). But
                    # its FIRST primitive is a rotation, and a sidestep out
                    # of a lattice node beside an occupied bay entry is the
                    # 2026-09-30 contact geometry exactly. Gated with a
                    # short deadline: a yield that cannot turn safely must
                    # fail fast so the resolver or the abort timer takes
                    # over, rather than turning into somebody.
                    #
                    # A robot with no room to turn is only ever sent STRAIGHT
                    # out (2026-10-30, _yield_move): already pointing there,
                    # so all that is left is a trim the leg steers out.
                    straight_out = self._blocked_trim_skip(
                        frm, depart, f"{leg_label}: yield departure",
                        "leaving straight out",
                        length_in=math.hypot(x1 - x0, y1 - y0))
                    if not straight_out:
                        ok, why = self._clear_to_rotate(
                            depart, f"{leg_label}: yield departure",
                            timeout_sec=YIELD_ROTATION_CLEAR_TIMEOUT_SEC,
                            at_node=frm)
                        if not ok:
                            return False, f"{why} (yield departure)", False
                        if not self._turn(
                                depart, f"{leg_label}: yield departure turn"):
                            return False, (
                                f"yield departure turn failed on {leg_label}"
                            ), False
                if mode_requires_odom(nav.localization_mode):
                    initial_heading = facing
                    if not nav.ensure_odom_anchor(x0, y0, initial_heading):
                        return False, (
                            f"no fresh onboard odometry to anchor {leg_label}"
                        ), False
                if seg_index == 0:
                    # Reversing, "pointed down the leg" means the TAIL is.
                    hb = ((lambda *a: (heading_between(*a) + 180.0) % 360.0)
                          if rev else heading_between)
                    ok, why = self._ready_to_depart(frm, x0, y0, x1, y1,
                                                    leg_label, hb)
                    if not ok:
                        return False, why, False
                # Tell the collision guard where this leg ends, so it does
                # not extrapolate a braking robot past its own target.
                # Optional: a missing hook must never stop a robot driving.
                _vm = getattr(self.supervisor, "vision", None)
                _set = getattr(_vm, "set_leg_target", None)
                _clear = getattr(_vm, "clear_leg_target", None)
                if _set is not None:
                    _set(self.name, x1, y1)
                try:
                    _drove = (nav.drive_leg(x1, y1, leg_label, reverse=True) if rev
                              else nav.drive_leg(x1, y1, leg_label))
                finally:
                    if _clear is not None:
                        _clear(self.name)
                if not _drove:
                    # nav.last_failure carries WHY -- STALLED_IDLE,
                    # STALLED_DRIVING, MODE_ACK_LOST, SETTLE_FAILED,
                    # SETTLE_DIVERGED, VISION_LOST. Without it every one of
                    # these reads as the same "vision leg failed" line and
                    # the only way to tell them apart is reading the
                    # supervisor log by hand (operator request 2026-09-30).
                    return False, _leg_note(nav, f"drive_leg failed on {leg_label}"), False
                to_text = str(to)
                if to_text.startswith("D") and not to_text.startswith("DE"):
                    # Depot SLOT (D1..D6, the parking spot itself, not a DE#
                    # entry-corridor node) -- confirmed on hardware 2026-08-05:
                    # heading_between() below always points along the FINAL
                    # approach leg's own travel direction (e.g. DE2->D2 drives
                    # +y, so heading_between returns 180), which is correct for
                    # every other node (face the way you're about to keep
                    # going) but wrong here, since there's no "next leg" to
                    # face at the depot -- the robot just stops. It must
                    # instead square up to the SAME heading it started the
                    # whole route facing (yaw 0 -- reset_pose(0,0,0) is taken
                    # while parked facing south, see agv_grid_workstation_
                    # solver.html's own comment on this), so a later mission
                    # gets a known-good starting orientation instead of
                    # whatever direction the final approach happened to come
                    # from.
                    heading = 0.0
                elif is_workstation_node(to, rows, cols):
                    # ROOT-CAUSED 2026-08-20 after the user reported a robot
                    # squaring up to 0deg instead of 180deg at a workstation
                    # (real hardware, drop-off/pick-up processing started
                    # facing the wrong way): this branch simply didn't exist
                    # -- workstation arrivals fell through to the plain
                    # heading_between() case below, same as any ordinary
                    # lattice node, which points along the ENTRY leg's own
                    # travel direction (0deg, since every workstation's only
                    # entry/exit is its Northern segment). A prior fix
                    # (2026-08-19) added this same 180deg-on-arrival rule to
                    # agv_grid_workstation_solver.html's generateCommandLines()
                    # -- but that JS function is ONLY used by color-sensor
                    # mode dispatch; vision mode (this file, drive_mode=
                    # "vision") never calls it and has its own completely
                    # separate turn logic here. Mirrors the depot-slot
                    # special case immediately above: a fixed heading, not
                    # derived from travel direction, because the workstation
                    # layout guarantees the exit heading regardless of which
                    # direction the robot arrived from.
                    heading = 180.0
                else:
                    # Reversed in: hold the way it faces, never turn round.
                    heading = facing
                # At the final ordinary transit node, an immediately-following
                # move already causes the absolute departure-heading turn
                # below. Squaring to the incoming heading first is redundant
                # and cost ~1.1-1.2 s for small 6-9 degree rotations in
                # run_1527.log before the robot then turned again. Preserve
                # arrival square-up at workstations/depots (their orientation
                # is semantically required), at a route endpoint, and between
                # multiple physical segments represented by one plan item.
                final_segment = seg_index == len(segs) - 1
                ordinary_transit_with_departure = (
                    final_segment
                    and next_move is not None
                    and not (to_text.startswith("D")
                             and not to_text.startswith("DE"))
                    and not is_workstation_node(to, rows, cols)
                )
                if not ordinary_transit_with_departure:
                    # The depot-slot and workstation branches above set a FIXED
                    # heading because the orientation is semantically required
                    # there -- so those are exactly the two that have to be
                    # held to the tight tolerance, not merely aimed at it.
                    arrival_tol = (
                        nav.args.final_yaw_tol_deg
                        if ((to_text.startswith("D")
                             and not to_text.startswith("DE"))
                            or is_workstation_node(to, rows, cols))
                        else None)
                    # THE REAL ROTATION GATE. Checked here, immediately
                    # before the turn actually happens, because the
                    # standalone turn PlanItem this used to be gated on is
                    # a no-op in vision mode and runs AFTER this rotation.
                    #
                    # The robot has ARRIVED, so this must never fail the leg:
                    # a failure re-dispatches the leg from `frm`, which the
                    # robot has left, and the vision gate then excludes it
                    # as off-route. 2026-10-16: Alvik4 reached 18, was held
                    # 20s here, failed, and was excluded ~23s later.
                    #   * Workstation/depot: the heading is required, so
                    #     hold with no deadline.
                    #   * Anywhere else this only trims arrival drift toward
                    #     the incoming heading; every later turn re-aims to
                    #     an absolute heading. If it stays blocked, skip it.
                    ok, why = self._clear_to_rotate(
                        heading, f"square up at {to}",
                        timeout_sec=(None if arrival_tol is not None
                                     else ROTATION_CLEAR_TIMEOUT_SEC),
                        at_node=to)
                    if not ok and why.startswith("aborted"):
                        return False, f"{why} (square up at {to})", False
                    if not ok:
                        self.supervisor.get_logger().warning(
                            f"[{self.name}] square up at {to} skipped: {why}; "
                            "arrived, keeping the arrival heading")
                    elif not self._turn(
                            heading, f"square up at {to} to {heading:.0f}deg",
                            tol_deg=arrival_tol):
                        return False, (
                            f"turn_to_heading_rotate_rel failed at {to}"
                        ), False
                last_to = (to, x1, y1)

            if next_move is not None and last_to is not None:
                to_label, tx, ty = last_to
                nfrm, nto = next_move.from_node, next_move.to_node
                nx, ny = nav_node_to_world(nto, rows, cols)
                next_heading = heading_between(tx, ty, nx, ny)
                # Step (3), the turn toward the next leg. This is the
                # rotation the entry/lattice geometry actually conflicts
                # over: at 5.00in between a bay entry and its flanking
                # lattice node, against a 5.625in turning circle, neither
                # robot can turn while the other is there. Gated here
                # rather than on the no-op turn item that follows.
                # No deadline: the robot has arrived at `to_label`, so a
                # failure would strand it exactly as at the square-up above,
                # and skipping would send it down the next leg mis-aimed --
                # except a small trim on a straight-on route, which the next
                # leg's own steering absorbs (see _straight_trim_skip).
                if self._straight_trim_skip(
                        to_label, heading_between(x0, y0, x1, y1),
                        next_heading, f"turn at {to_label} toward {nfrm}->{nto}"):
                    return True, "ok", False
                ok, why = self._clear_to_rotate(
                    next_heading, f"turn at {to_label} toward {nfrm}->{nto}",
                    timeout_sec=None, at_node=to_label)
                if not ok:
                    return False, f"{why} (turn at {to_label})", False
                if fuse_next:
                    if not nav.turn_then_drive_leg(
                            next_heading, nx, ny,
                            f"turn at {to_label} toward {nfrm}->{nto}"):
                        return False, _leg_note(
                        nav, f"turn_then_drive_leg failed at {to_label}"), False
                    return True, "ok", True
                if not self._turn(
                        next_heading,
                        f"turn at {to_label} toward {nfrm}->{nto}"):
                    return False, _leg_note(
                        nav, f"turn toward next leg failed at {to_label}"), False
            return True, "ok", False
        except Exception as exc:  # never let a worker thread die silently
            return False, f"exception: {exc!r}", False


@dataclass
class FleetJob:
    """One unit of work, identified independently of who is carrying it.

    job_id is assigned once from the schedule and never changes. robot is
    the CURRENT assignee and is expected to change: the downed-robot
    mechanic orphans a dead robot's pending jobs and hands them to robots
    with spare capacity, and the metrics row for a job has to survive that
    or the reallocation looks like a job failing and a new one appearing.
    """
    job_id: str
    workstation: int          # solver workstation id (bay), NOT the grid node
    mode: str                 # "drop" | "pickup"
    robot: str                # current assignee
    origin_robot: str         # who the schedule planned it for
    status: str = "pending"   # pending | done | orphaned


class JobRegistry:
    """Jobs for one mission, plus each robot's remaining queue in plan order.

    Built from the schedule export, whose routes carry
    visits: [{bay, mode}, ...] in the order the robot serves them -- the
    same order its dwell plan items occur in, which is what lets
    complete_next() pair a finished dwell with the right job without
    threading an id through PlanItem and the advisor's alignment.
    """

    def __init__(self, schedule: dict | None):
        self.jobs: dict[str, FleetJob] = {}
        self.queues: dict[str, list[str]] = {}
        seq = 0
        for route in ((schedule or {}).get("routes") or []):
            robot = route.get("agv") or route.get("agent")
            if robot is None:
                continue
            visits = route.get("visits") or [
                {"bay": b, "mode": "drop"} for b in (route.get("bays") or [])]
            for visit in visits:
                bay = visit.get("bay")
                if bay is None:
                    continue
                seq += 1
                jid = f"J{seq:02d}"
                self.jobs[jid] = FleetJob(
                    job_id=jid, workstation=int(bay),
                    mode=str(visit.get("mode") or "drop"),
                    robot=str(robot), origin_robot=str(robot))
                self.queues.setdefault(str(robot), []).append(jid)

    def __len__(self) -> int:
        return len(self.jobs)

    def next_pending(self, robot: str):
        """This robot's next unfinished job, or None."""
        for jid in self.queues.get(robot, []):
            job = self.jobs[jid]
            if job.status == "pending" and job.robot == robot:
                return job
        return None

    def pending_after(self, robot: str):
        """The unfinished job AFTER this robot's next one, or None."""
        seen = False
        for jid in self.queues.get(robot, []):
            job = self.jobs[jid]
            if job.status == "pending" and job.robot == robot:
                if seen:
                    return job
                seen = True
        return None

    def complete_next(self, robot: str):
        """Mark this robot's next pending job done and return it, or None.

        Returns None rather than raising when a dwell has no job behind it:
        a dwell can legitimately occur without being a service visit, and a
        metrics gap is never worth stalling dispatch over.
        """
        for jid in self.queues.get(robot, []):
            job = self.jobs[jid]
            if job.status == "pending" and job.robot == robot:
                job.status = "done"
                return job
        return None

    def orphan(self, robot: str) -> list:
        """Take every pending job off a robot -- the downed-robot case.

        The jobs keep their ids and become unassigned, so reassign() can
        place them elsewhere and the metrics row continues rather than
        being replaced.
        """
        taken = []
        for jid in self.queues.get(robot, []):
            job = self.jobs[jid]
            if job.status == "pending":
                job.status = "orphaned"
                taken.append(job)
        return taken

    def reassign(self, job_id: str, robot: str) -> bool:
        """Give an orphaned job to another robot, appended to its queue."""
        job = self.jobs.get(job_id)
        if job is None or job.status == "done":
            return False
        job.robot = robot
        job.status = "pending"
        self.queues.setdefault(robot, []).append(job_id)
        return True

    def pending_for(self, robot: str) -> list:
        return [self.jobs[j] for j in self.queues.get(robot, [])
                if self.jobs[j].status == "pending"
                and self.jobs[j].robot == robot]

    def summary(self) -> str:
        done = sum(1 for j in self.jobs.values() if j.status == "done")
        orph = sum(1 for j in self.jobs.values() if j.status == "orphaned")
        return (f"{done}/{len(self.jobs)} jobs done"
                + (f", {orph} orphaned" if orph else ""))


class LaunchTimer:
    """Time from START to each robot's arrival at node 0.

    Kept apart from the job stream deliberately. This measures the depot
    funnel -- six robots leaving a seven-slot lane through one node -- and
    the 2026-09-09 export shows why it matters: Alvik6 set the 248.8s
    makespan while carrying 104.96s of launch offset, so most of the
    critical path was queueing, not driving.

    Before changing the launch order (releasing Alvik1 and Alvik6 together,
    or cascading DE1 -> DE2 -> ...), each robot's current funnel time has to
    be on record, or there is no baseline to compare the change against.

    Arrival is detected from the vision pose rather than a command
    completion, because the depot-lane hops (D1 -> DE1 -> 0) are folded into
    the first move as implicit_segs and never appear as a PlanItem to_node.
    """

    #: grid units; node spacing is 1.0, so this cannot catch a neighbour.
    ARRIVE_RADIUS = 0.45

    def __init__(self, rows: int, cols: int):
        self.started_at: dict[str, float] = {}
        self.arrived: dict[str, float] = {}
        try:
            wx, wy = nav_node_to_world("0", rows, cols)
            from camera_grid_navigate import (GRID_NODE1_WORLD_IN,
                                              GRID_PITCH_IN)
            gx, gy = GRID_NODE1_WORLD_IN
            self.target = ((wx - gx) / GRID_PITCH_IN, (wy - gy) / GRID_PITCH_IN)
        except Exception:
            self.target = None

    def start(self, robots, now: float) -> None:
        for name in robots:
            self.started_at.setdefault(name, now)

    def note_pose(self, name: str, grid_x: float, grid_y: float,
                  now: float):
        """Return elapsed seconds the first time `name` reaches node 0."""
        if self.target is None or name in self.arrived:
            return None
        t0 = self.started_at.get(name)
        if t0 is None:
            return None
        dx = grid_x - self.target[0]
        dy = grid_y - self.target[1]
        if (dx * dx + dy * dy) ** 0.5 > self.ARRIVE_RADIUS:
            return None
        self.arrived[name] = now
        return now - t0


def nav_node_to_world(label, rows: int, cols: int) -> tuple[float, float]:
    """Schedule node label (int-as-str for lattice/workstation/entry nodes,
    or 'D1'..'D6'/'DE1'..'DE6'/'0' for depot labels) -> world (x_in, y_in)
    via camera_grid_navigate.node_to_world(). The advisor stores every label
    as a string (see RobotState.current_node/PlanItem.from_node/to_node);
    node_to_world() accepts numeric labels as either int or numeric string
    directly, only depot labels need the string form."""
    from camera_grid_navigate import node_to_world
    text = str(label)
    if text.isdigit() and text != "0":
        return node_to_world(int(text), rows, cols)
    return node_to_world(text, rows, cols)


def _parse_args(argv: list[str]) -> tuple[dict[str, list[str]], dict]:
    usage = ("usage: fleetSupervisor.py [--schedule plan.json] [--listen] "
             "[--no-llm] [--no-vision] [--audit-log path.jsonl] [--timeout SEC] "
             "[--launch-mode offsets|cascade] "
             "--robot AlvikN --file route.txt | --tokens R,RED,RED "
             "[--robot AlvikM ...]\n"
             "  --listen: no robots/files needed — wait for missions on "
             "/fleet_mission (sent by the HTML), start via /fleet_control START")
    robots: dict[str, list[str]] = {}
    options = {"schedule": None, "no_llm": False, "no_vision": False,
               "listen": False, "audit_log": "advisor_audit.jsonl",
               "calibration_file": "timing_calibration.json", "timeout": 90.0,
               "launch_mode": "offsets",
               "max_leg_retries": MAX_LEG_RETRIES}
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--schedule" and i + 1 < len(argv):
            options["schedule"] = argv[i + 1]
            i += 2
        elif arg == "--listen":
            options["listen"] = True
            i += 1
        elif arg == "--no-llm":
            options["no_llm"] = True
            i += 1
        elif arg == "--no-vision":
            options["no_vision"] = True
            i += 1
        elif arg == "--audit-log" and i + 1 < len(argv):
            options["audit_log"] = argv[i + 1]
            i += 2
        elif arg == "--calibration-file" and i + 1 < len(argv):
            # persisted per-robot timing factors ("" disables calibration I/O)
            options["calibration_file"] = argv[i + 1] or None
            i += 2
        elif arg == "--timeout" and i + 1 < len(argv):
            options["timeout"] = float(argv[i + 1])
            i += 2
        elif arg == "--launch-mode" and i + 1 < len(argv):
            mode = argv[i + 1]
            if mode not in ("offsets", "cascade"):
                raise SystemExit(
                    f"unknown --launch-mode '{mode}' (expected offsets or cascade)")
            options["launch_mode"] = mode
            i += 2
        elif arg == "--max-leg-retries" and i + 1 < len(argv):
            # 0 restores the pre-2026-09-25 behaviour of excluding a robot on
            # its first failed vision leg.
            options["max_leg_retries"] = max(0, int(argv[i + 1]))
            i += 2
        elif arg == "--robot" and i + 3 < len(argv):
            name = argv[i + 1]
            mode = argv[i + 2]
            value = argv[i + 3]
            if mode == "--file":
                robots[name] = commands_from_file(value)
            elif mode == "--tokens":
                robots[name] = tokens_to_commands(value)
            else:
                raise SystemExit(
                    f"unknown mode '{mode}' for --robot {name} (expected --file or --tokens)")
            i += 4
        else:
            raise SystemExit(usage)
    if not robots and not options["listen"]:
        raise SystemExit(usage)
    return robots, options


def _spin_forever(node) -> None:
    """rclpy.spin(), but a callback that raises no longer ends it.

    An exception in ANY subscription callback propagates out of rclpy.spin
    and ends the spin thread; the supervisor then looks alive but never
    hears another mission, START or robot status, and only a restart helps
    (the 4->6 robot switches during the DOE). The operator wants run after
    run without restarts, so: log the traceback, keep spinning. Each
    distinct failure is printed in full a few times, then only counted, so
    a fault repeating at 20 Hz cannot flood the terminal."""
    import traceback
    seen: dict = {}
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:                       # older rclpy
        ExternalShutdownException = KeyboardInterrupt
    while rclpy.ok():
        try:
            rclpy.spin(node)
            return                            # spin returned: shutting down
        except (KeyboardInterrupt, ExternalShutdownException):
            return
        except Exception as exc:              # noqa: BLE001 - that is the point
            if not rclpy.ok():
                return
            tb = traceback.extract_tb(exc.__traceback__)
            where = f"{tb[-1].filename}:{tb[-1].lineno}" if tb else "?"
            key = (type(exc).__name__, where)
            seen[key] = seen.get(key, 0) + 1
            n = seen[key]
            try:
                log = node.get_logger()
                if n <= 3:
                    log.error(f"callback raised {type(exc).__name__}: {exc} "
                              "-- spinning on\n" + traceback.format_exc())
                elif n in (10, 100) or n % 1000 == 0:
                    log.error(f"callback raised {type(exc).__name__} at {where} "
                              f"{n} times so far -- spinning on")
            except Exception:                 # noqa: BLE001
                pass


def main() -> None:
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        sys.exit(1)

    robots, options = _parse_args(argv)

    advisor = None
    if options["schedule"]:
        # Build the advisor BEFORE touching ROS so a command-list/schedule
        # mismatch fails loudly here, not mid-run.
        from dispatch_advisor import DispatchAdvisor, load_schedule
        llm = None
        if not options["no_llm"]:
            # Local Ollama server — free, no API account. Fails loudly here
            # (server down / model not pulled) rather than mid-run.
            from ollama_advisor import OllamaAdvisor
            llm = OllamaAdvisor()
        advisor = DispatchAdvisor(
            load_schedule(options["schedule"]), robots,
            llm=llm, log_path=options["audit_log"],
            calibration_path=options["calibration_file"])

    rclpy.init()
    node = FleetSupervisor(robots, use_vision=not options["no_vision"],
                           launch_mode=options["launch_mode"])
    node.max_leg_retries = int(options["max_leg_retries"])
    if advisor is not None:
        advisor.log_fn = node.advisor_log_sink()  # LLM activity -> terminal + log
        node.advisor = advisor

    spin_thread = threading.Thread(target=_spin_forever, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if options["listen"]:
            _run_listen_loop(node, options)
        elif advisor is not None:
            node.run_advised(advisor, timeout_sec=options["timeout"])
            node.get_logger().info(advisor.llm_activity_summary())
        else:
            node.run_stepped()
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C during spin can shut the context down first — the second
        # shutdown (and destroy) then raises RCLError noise; ignore it.
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


def _run_listen_loop(node: "FleetSupervisor", options: dict) -> None:
    """Listen mode: HTML sends {schedule, commands} on /fleet_mission; each
    mission is aligned, armed, and started via /fleet_control START. The
    conflict advisor runs natively here — one hop from the robots — instead
    of in the browser."""
    from dispatch_advisor import AlignmentError, DispatchAdvisor
    import time as _time

    llm = None
    if not options["no_llm"]:
        try:
            from ollama_advisor import OllamaAdvisor
            llm = OllamaAdvisor()
            node.get_logger().info(
                f"LLM advisor ready: {llm.model} via Ollama at {llm.url}")
        except Exception as exc:
            node.get_logger().warning(
                f"LLM advisor unavailable ({exc}) — running deterministic-only")

    node.get_logger().info(
        "listen mode: waiting for /fleet_mission (send from the HTML solver)...")
    while rclpy.ok():
        if not node._mission_event.wait(timeout=0.5):
            continue
        node._mission_event.clear()
        mission = node._pending_mission
        try:
            advisor = DispatchAdvisor(
                mission["schedule"], mission["commands"],
                llm=llm, log_path=options["audit_log"],
                calibration_path=options["calibration_file"],
                log_fn=node.advisor_log_sink())
            node.get_logger().info(
                f"timing calibration: {advisor.calibration.summary()}")
        except AlignmentError as exc:
            node.get_logger().error(f"mission rejected (alignment): {exc}")
            node.publish_fleet_status("error", note=f"alignment: {exc}")
            continue
        # Retained so the rotation gate can register a standoff with
        # note_vision_block(); see await_rotation_clearance().
        node.advisor = advisor
        node.drive_mode = str(mission.get("drive_mode", "color")).lower()
        # In vision mode a turn plan item is a no-op and the real rotation is
        # gated by await_rotation_clearance(); the advisor must not ALSO hold
        # the no-op (2026-10-16, Alvik3 held 60s after it had already turned).
        advisor.rotation_gated_at_execution = (node.drive_mode == "vision")
        node.configure_robots(mission["commands"])
        node.wait_for_status_match(mission["commands"])
        node._start_event.clear()
        node._abort = False
        node.publish_fleet_status("loaded", advisor,
                                  note="send START on /fleet_control")
        node.get_logger().info(
            f"mission armed for {sorted(mission['commands'])} — waiting for START "
            "(robots must be parked on their depot stickers)")
        last_pub = 0.0
        while rclpy.ok() and not node._start_event.wait(timeout=0.5):
            if node._mission_event.is_set():
                break  # a newer mission replaces this one
            if _time.monotonic() - last_pub >= 1.0:
                # keep the HTML's online/offline markers live while armed
                last_pub = _time.monotonic()
                node.publish_fleet_status("armed", advisor,
                                          note="send START on /fleet_control")
        if node._mission_event.is_set() or node._abort or not rclpy.ok():
            if node._abort:
                node.publish_fleet_status("aborted", advisor)
                node.get_logger().info("mission cancelled before start")
            continue
        transcript = RunTranscript.for_mission(mission)
        drive_mode = mission.get("drive_mode", "color")
        localization_mode = normalize_localization_mode(
            mission.get("localization_mode", "camera_assist"))
        if drive_mode == "vision":
            fuse_turns = bool(mission.get("fuse_turns", False))
            # The Run tab's Drive mode buttons send this; an unknown value is
            # dropped rather than guessed at, which leaves the launch default.
            turn_method = mission.get("turn_method")
            if turn_method not in (None, "staged", "rotate_rel", "wheel"):
                node.get_logger().warn(
                    f"mission turn_method {turn_method!r} is not staged, "
                    "rotate_rel or wheel -- ignored, launch default applies")
                turn_method = None
            launch_turn = os.environ.get("AGV_TURN_METHOD") or "staged"
            node.get_logger().info(
                "drive_mode=vision — dispatching position/heading control; "
                f"localization={localization_mode}; "
                f"turn={turn_method or launch_turn}"
                f"{' (from mission)' if turn_method else ' (launch default)'}; "
                "color sensor not used"
                + (" [fuse_turns requested]" if fuse_turns else ""))
            node.run_advised_vision(advisor, timeout_sec=options["timeout"],
                                    fuse_turns=fuse_turns,
                                    localization_mode=localization_mode,
                                    turn_method=turn_method)
        else:
            node.get_logger().info(
                "drive_mode=color — firmware color markers trigger nodes and "
                "firmware follows tape; position-target localization selector "
                f"({localization_mode}) is inactive")
            node.run_advised(advisor, timeout_sec=options["timeout"])
            # Color-mode runs are not yet instrumented per-event; emit ONE
            # coarse terminal event so a metrics-registered run doesn't stay
            # 'running' forever. Vision mode emits its own detailed stream
            # (see run_advised_vision); an abort already emitted RUN_ABORTED.
            if not node._abort:
                errored_names = [n for n, ch in node._channels.items()
                                 if ch.errored]
                node._memit("RUN_FAILED" if errored_names else "RUN_COMPLETED",
                            details={"drive_mode": "color",
                                     "localization_mode": localization_mode,
                                     "errored": errored_names,
                                     "note": "coarse terminal event; "
                                             "color mode has no per-event "
                                             "metrics instrumentation yet"})
        node.get_logger().info(advisor.llm_activity_summary())
        node.get_logger().info("listen mode: waiting for the next mission...")
        transcript.close()


if __name__ == "__main__":
    main()
