#!/usr/bin/env python3
"""
camera_grid_navigate.py — drive ONE Alvik through a sequence of grid-lattice
waypoints (node numbers, e.g. "1,2,10,18") using a selectable localization
source: onboard odometry only, onboard odometry with AprilTag correction, or
AprilTag camera poses only. The color sensor and tape-follow state machine are
not used by this controller.

drive_leg() steering (rewritten 2026-07-27, ported from a bench-verified
onboard-odometry driveTo() that achieved 0.1in/high repeatability): steers
toward the LIVE target point every tick -- distance and heading-to-target are
recomputed fresh from the current camera pose each iteration, not tracked
against a line fixed at leg start. Heading convention: 0=-y, 90=+x, 180=+y,
270=-x, positive=CCW (see camera_trajectory.TargetLine's docstring for how
this was bench-verified); heading_to_target_deg() below is atan2(dx,-dy),
the equivalent of the reference function's atan2(dy,dx) in THIS robot's
convention -- verified against all four cardinal directions before use.

Speed law REWRITTEN AGAIN 2026-07-28: constant --cruise-rpm the whole leg,
then a hard brake (stop_and_rearm()) once within --brake-lead-in of target --
replaced the distance-proportional kp_dist/drive-min/max-speed law entirely.
User's explicit direction after repeated hardware overshoot/undershoot
tuning: "Instead of slowing down, we just need the alvik to brake" -- a
tapered approach doesn't reproduce the old color-sensor firmware's
demonstrated 60-70RPM constant-speed-then-instant-hard-stop behavior.
--brake-lead-in's default is measured data (see stop_test()/
stop_test_route(), a dedicated constant-cruise-then-brake measurement mode),
not a guess -- re-measure with --stop-test if --cruise-rpm changes.

Turning between legs: turn_to_heading_rotate_rel() sizes one fast ROTATE_REL
from the selected localization yaw; firmware executes alvik.rotate(), waits
for a stable onboard IMU target, brakes, and then acknowledges completion.
Camera-capable modes may send one small correction. A missing selected pose
aborts instead of silently substituting the streamed taper. An EARLIER ROTATE_REL-based turn
(alvik.rotate(), closed-loop on Alvik's own motor-control MCU) was tried
2026-07-30 and reverted the same day after multiple firmware hangs on real
hardware, root cause unresolved at the time. That hang was later
root-caused and FIXED at the firmware level (see AGV_Factory_camera_
correction.ino's ROTATE_REL handler comment) and re-validated 2026-08-13
with a 128-rotation hardware stress test (zero hangs) -- turn_to_heading_
rotate_rel() is the current, actively-used ROTATE_REL turn path: sizes
from the selected localization source, then samples vision in camera-capable
modes and sends ONE corrective ROTATE_REL if still outside
--turn-tol-deg (default 2026-08-13 after a --yaw-stress-test comparison,
Alvik6, 128 rotations/tier: odom-only mean error 2.46deg/1.30s per turn vs.
this camera-assisted approach's 0.91deg/2.01s -- ~2.7x more accurate for
+0.7s/turn, judged worth it for a research testbed). See that method's own
docstring for the full hang-history and design rationale
before ever changing its completion-check pattern.

Grid <-> world conversion mirrors apriltag_localize.py's world_to_grid()
and agv_grid_workstation_solver.html's node numbering (nodeNumber(r,c) =
(rows-1-r)*cols + c + 1, node 1 = bottom-left / depot end). Verified against
two bench points: node 1 -> world (13.5, 16.75); node 8 -> world (83.5,
16.75), matching a real run to within 0.1in. Workstation/entry bay offsets
remeasured on the physical table 2026-07-27 (bay 1, node114/node65) -- see
node_to_world()'s comment for the corrected values vs. the dispatch model's
assumed ones.

Run inside WSL2 (ROS 2 sourced, native rclpy) as of the 2026-07-29/30
migration off the separate Linux laptop -- see wsl2_microros_migration
notes. Needs: the micro-ROS agent up (WSL2), rosbridge up (WSL2), the robot
powered/green running AGV_Factory_camera_correction.ino with its firmware
ROTATE_REL command (added 2026-07-30 -- older firmware without it will
reject ROTATE_REL as ERROR UNKNOWN_COMMAND), and
camera_bridge_windows.py (native Windows) + apriltag_localize.py
--frame-source-port ... --rosbridge=... (WSL2) for vision:

    python3 camera_grid_navigate.py --robot Alvik3 --route 1,8,16

Ctrl+C sends STOP and exits early.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from typing import Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from camera_trajectory import Pose, normalize_deg, yaw_error_deg
from localization_modes import (
    OdomAnchor,
    blended_correction,
    mode_requires_camera,
    mode_requires_odom,
    normalize_localization_mode,
    odom_to_world,
)

CONTROL_HZ = 50.0
# Raised from 30 2026-07-28: alvik.brake() is confirmed instant/unconditional
# in firmware (AGV_Factory_camera_correction.ino STOP handler, no queueing).
# The only real lag between the robot crossing brake_lead_in and STOP being
# issued is this loop's own tick period, since drive_leg()/turn_to_heading()
# only check dist_to_target once per tick. 50Hz matches apriltag_localize.py's
# real sustained pose-publish rate (~48-51Hz on the WSL2 bridge config) -- the
# actual ceiling on how soon a crossed threshold can be seen at all; going
# higher than the vision rate would just re-check a stale pose more often for
# no benefit.
VISION_STALE_SEC = 0.5
ODOM_STALE_SEC = 0.75
# Complementary-filter gains applied per accepted AprilTag update in
# camera_assist mode. Odometry remains the high-rate prediction; camera
# observations pull accumulated position/yaw drift back toward table truth.
CAMERA_ASSIST_POSITION_ALPHA = 0.20
CAMERA_ASSIST_YAW_ALPHA = 0.20
CAPTURE_SETTLE_SEC = 1.0
# Confirmed 2026-07-27 (near-collision on the 8->16 leg): a FIXED jump cap
# compared only against the last ACCEPTED pose is a lockout trap. A dropped
# BEST_EFFORT rosbridge message (normal on websocket transport -- the camera
# itself never lost the tag, confirmed watching apriltag_localize.py's own
# preview) widens the gap since the last accepted pose; once that gap exceeds
# a FIXED cap, the next (correct) reading is rejected too, widening the gap
# further -- a permanent lockout while the robot keeps moving, which looks
# exactly like "vision went stale" even though the camera never lost the tag.
# Fix: scale the allowed jump by elapsed time * a generous max plausible
# speed, so a legitimately-larger gap (because more time passed) is accepted,
# while a same-instant teleport (a genuinely bad AprilTag read) still isn't.
POSE_JUMP_MAX_SPEED_IN_PER_SEC = 20.0  # generous vs. the ~2.5in/s bench speed
POSE_JUMP_MARGIN_IN = 3.0              # slack for a single-tick measurement wobble
YAW_JUMP_REJECT_DEG = 45.0

# Grid <-> world calibration, mirrors apriltag_localize.py.
GRID_NODE1_WORLD_IN = (13.5, 16.75)
GRID_PITCH_IN = 10.0

# MEASURED workstation (bay) positions, world inches, replacing the derived
# grid_y = (row_up + 1) - 0.44 offset for workstation nodes ONLY.
#
# Measured 2026-08-26 with fleet/calibrate_workstations.py: seven robots
# parked one row of bays at a time, all 49 bays, each value the median of
# 60 AprilTag pose samples. Sample spread was 0.01in and every heading
# landed within 2.0deg of the 180deg service heading, so these are clean
# position reads, not averages over a moving or mis-aimed robot.
#
# WHY THIS EXISTS: the 0.44 offset was measured ONCE, on bay 1
# (node114/node65) on 2026-07-27, then applied to all 49 bays. It is
# wrong -- the real mean offset is 0.503 grid units, so every bay sat
# 0.63in further from its row than the formula assumed (worst 0.97in),
# on a 4.4in bay approach leg with POS_TOL_IN at 0.15in. There is also
# genuine per-bay scatter the single constant could never capture:
# ~0.18in mean residual even after removing row and column trends.
#
# ENTRY NODES ARE NOT IN THIS TABLE, deliberately. They sit on the
# lattice row and the routing/reservation model is built on that.
#
# KNOWN LIMITATION: each column was measured by one robot (col 0 =
# Alvik1 ... col 6 = tag7), so a robot's tag-mounting offset and that
# column's true position are confounded. Column means run +0.06 to
# -0.37in in x. Re-measuring one row with the robots shifted a column
# would separate the two; until then up to ~0.4in of the x component
# may be robot mount, not bay.
#
# REGENERATE with: calibrate_workstations.py --emit-python
# (paste into BOTH apriltag_localize.py and fleet/camera_grid_navigate.py
# -- they run on different machines and cannot import each other.)
WORKSTATION_WORLD_IN: dict[int, tuple[float, float]] = {
    65: (18.30, 21.76),  # row 0 col 0, Alvik1
    66: (28.12, 21.81),  # row 0 col 1, Alvik2
    67: (38.19, 21.88),  # row 0 col 2, Alvik3
    68: (48.19, 21.86),  # row 0 col 3, Alvik4
    69: (58.29, 21.85),  # row 0 col 4, Alvik5
    70: (68.25, 21.76),  # row 0 col 5, Alvik6
    71: (77.73, 21.74),  # row 0 col 6, tag7
    72: (18.42, 31.90),  # row 1 col 0, Alvik1
    73: (28.25, 31.78),  # row 1 col 1, Alvik2
    74: (38.32, 31.72),  # row 1 col 2, Alvik3
    75: (48.07, 31.76),  # row 1 col 3, Alvik4
    76: (58.28, 31.73),  # row 1 col 4, Alvik5
    77: (68.08, 31.90),  # row 1 col 5, Alvik6
    78: (78.23, 31.90),  # row 1 col 6, tag7
    79: (18.61, 41.69),  # row 2 col 0, Alvik1
    80: (28.49, 41.75),  # row 2 col 1, Alvik2
    81: (38.30, 41.82),  # row 2 col 2, Alvik3
    82: (48.18, 41.86),  # row 2 col 3, Alvik4
    83: (58.26, 41.83),  # row 2 col 4, Alvik5
    84: (68.03, 41.79),  # row 2 col 5, Alvik6
    85: (78.04, 41.86),  # row 2 col 6, tag7
    86: (18.62, 51.70),  # row 3 col 0, Alvik1
    87: (28.58, 51.55),  # row 3 col 1, Alvik2
    88: (37.98, 51.46),  # row 3 col 2, Alvik3
    89: (48.10, 51.52),  # row 3 col 3, Alvik4
    90: (58.40, 51.54),  # row 3 col 4, Alvik5
    91: (68.19, 51.62),  # row 3 col 5, Alvik6
    92: (78.14, 51.71),  # row 3 col 6, tag7
    93: (18.88, 61.55),  # row 4 col 0, Alvik1
    94: (28.78, 61.67),  # row 4 col 1, Alvik2
    95: (38.48, 61.67),  # row 4 col 2, Alvik3
    96: (48.32, 61.91),  # row 4 col 3, Alvik4
    97: (58.40, 61.80),  # row 4 col 4, Alvik5
    98: (68.38, 61.77),  # row 4 col 5, Alvik6
    99: (78.24, 61.78),  # row 4 col 6, tag7
    100: (18.68, 71.56),  # row 5 col 0, Alvik1
    101: (28.65, 71.38),  # row 5 col 1, Alvik2
    102: (38.22, 71.51),  # row 5 col 2, Alvik3
    103: (48.45, 71.61),  # row 5 col 3, Alvik4
    104: (58.61, 71.53),  # row 5 col 4, Alvik5
    105: (68.30, 71.79),  # row 5 col 5, Alvik6
    106: (78.01, 71.98),  # row 5 col 6, tag7
    107: (18.44, 81.80),  # row 6 col 0, Alvik1
    108: (28.46, 81.69),  # row 6 col 1, Alvik2
    109: (38.44, 81.60),  # row 6 col 2, Alvik3
    110: (48.44, 81.65),  # row 6 col 3, Alvik4
    111: (58.54, 81.50),  # row 6 col 4, Alvik5
    112: (68.56, 81.65),  # row 6 col 5, Alvik6
    113: (78.49, 81.81),  # row 6 col 6, tag7
}

# Table bounds safety stop (2026-08-19, real hardware failure: Alvik4 was
# commanded ROTATE_REL -84deg at DE4 -- a depot-entry node near the table's
# south edge -- and drove forward off the table instead of rotating; no
# ROTATE_REL COMPLETE ack ever arrived and the route aborted on timeout, but
# by then the robot was already off-table. Root cause not yet identified
# (firmware STATE_ROTATE_REL is a simple non-blocking timer that looked
# correct on inspection; Python-side send path is a plain publish, nothing
# stale found) -- this is a BACKSTOP against the hazard recurring while that
# investigation continues, not a fix for the underlying cause. Mirrors
# apriltag_localize.py's TAG20_INSET_IN=2.75/SPAN_IN=91.5 -> 97.0in table
# (kept as a literal here, not imported, matching this file's existing
# style of duplicating apriltag_localize.py's world constants locally --
# see GRID_NODE1_WORLD_IN/GRID_PITCH_IN above and DEPOT_SLOT_WORLD_IN's own
# comment on why). Margin is generous (6in) since DEPOT_ENTRY_WORLD_IN
# points sit as close as y=3.0in to the table's own y=0 edge under normal
# operation -- a tight bound would false-trigger on ordinary depot-entry
# parking. This only checks the fresh vision pose as it arrives (every tick,
# regardless of which command is currently in flight), so it catches a
# runaway robot mid-motion, not just between commands.
TABLE_SIZE_IN = 97.0
TABLE_BOUNDS_MARGIN_IN = 6.0

# Depot-exit waypoints -- outside the rows*cols lattice/workstation/entry
# numbering entirely (a physically separate lane south of the grid), so they
# get reserved node IDs rather than fitting the numeric scheme. All 6 depot
# slots/entries measured on the physical table 2026-07-29 by parking each of
# the 6 Alvik robots in turn and reading camera-tracked position off
# apriltag_localize.py's preview overlay -- SAME real-measured numbers as
# apriltag_localize.py's DEPOT_SLOT_WORLD_IN/DEPOT_ENTRY_WORLD_IN/
# NODE0_WORLD_IN (added there 2026-07-30 for the 'o' overlay); keep both
# copies in sync if either is re-measured. Real spacing is NOT uniform
# (5.0-7.3in between slots) -- confirmed the HTML's nominal DEPOT_SLOT_PITCH
# assumption doesn't match hardware, so these are looked up by label, never
# interpolated from a pitch constant. String tokens "D1".."D6"/"DE1".."DE6"/
# "0" in --route map to these, matching the exact labels already used in
# agv_grid_workstation_solver.html's own route/schedule export (e.g.
# "D3-DE3-0-1-9-114-...") so a route can be copied over with no translation.
# Depot slots and entries REMEASURED 2026-08-26 with
# fleet/calibrate_workstations.py, after the depot tape was physically
# respaced to 16cm centres. The previous values (measured 2026-07-30) were
# left up to 3.2in wrong by that move, and their 5.0in spacing gave a
# rotating robot NEGATIVE clearance against a parked neighbour -- 5.001in
# centre-to-centre against the 5.04in a 15cm-hypotenuse robot needs, which
# is why Alvik3 struck a parked Alvik2 on the 4-robot depot return. The new
# spacing measures 6.14-6.38in, giving +1.10in at the tightest pair.
#
# D7/DE7 are new: a 7th tag position, measured so the table is complete.
# Routing a 7th robot is a separate job -- ROBOT_NAMES in
# apriltag_localize.py still maps tags 1-6 only, so tag 7 publishes as
# "tag7".
#
# NOTE both-robots-rotating clearance at the tightest pair is only +0.23in,
# inside placement error. Sequenced (one-at-a-time) depot rotations are
# fine; simultaneous adjacent ones are not.
#
# Regenerate with: calibrate_workstations.py --emit-python
# (paste into BOTH this file and the other one -- they run on different
# machines and cannot import each other.)
DEPOT_SLOT_WORLD_IN: dict[str, tuple[float, float]] = {
    "D1": (19.64, 10.86),  # Alvik1
    "D2": (25.88, 11.04),  # Alvik2
    "D3": (32.20, 11.08),  # Alvik3
    "D4": (38.58, 11.15),  # Alvik4
    "D5": (44.96, 11.20),  # Alvik5
    "D6": (51.20, 11.19),  # Alvik6
    "D7": (57.34, 11.23),  # tag7
}
DEPOT_ENTRY_WORLD_IN: dict[str, tuple[float, float]] = {
    "DE1": (19.71, 2.94),  # Alvik1
    "DE2": (25.93, 3.07),  # Alvik2
    "DE3": (32.09, 3.15),  # Alvik3
    "DE4": (38.56, 3.21),  # Alvik4
    "DE5": (45.07, 3.27),  # Alvik5
    "DE6": (51.47, 3.36),  # Alvik6
    "DE7": (57.51, 3.46),  # tag7
}
NODE0_WORLD_IN = (13.3, 2.9)

DEPOT_NODE_TOKENS: dict[str, str] = {"0": "0"}
DEPOT_NODE_TOKENS.update({k: k for k in DEPOT_SLOT_WORLD_IN})
DEPOT_NODE_TOKENS.update({k: k for k in DEPOT_ENTRY_WORLD_IN})
# Kept for single-robot CLI callers that still pass bare -1/-2/0 ints.
DEPOT_WORLD_IN = {-1: DEPOT_SLOT_WORLD_IN["D1"], -2: DEPOT_ENTRY_WORLD_IN["DE1"],
                  0: NODE0_WORLD_IN}


def node_to_world(n: int | str, rows: int, cols: int) -> tuple[float, float]:
    """Grid/workstation node number -> world (x_in, y_in). Node numbering
    matches agv_grid_workstation_solver.html's nodeNumber() and
    the dispatch model's label_to_cell() (kept in sync with that function --
    both must agree on the workstation/entry split, see is_workstation_node):
      1..rows*cols                          lattice nodes (RED marker)
      rows*cols+1 .. rows*cols+bays          WORKSTATION nodes (dead-end
                                              spur past the entry, YELLOW)
      rows*cols+bays+1 .. rows*cols+2*bays   ENTRY nodes (bay mouth, on the
                                              north edge of the bay's row)
    where bays = (rows-1)*(cols-1). Node 1 is the bottom-left (depot-adjacent)
    lattice point, numbering increases left to right then bottom to top.
    Depot labels ("D1".."D6", "DE1".."DE6", "0", or the legacy bare -1/-2/0
    ints) -> the real-measured DEPOT_SLOT_WORLD_IN/DEPOT_ENTRY_WORLD_IN/
    NODE0_WORLD_IN tables above."""
    if isinstance(n, str):
        if n in DEPOT_SLOT_WORLD_IN:
            return DEPOT_SLOT_WORLD_IN[n]
        if n in DEPOT_ENTRY_WORLD_IN:
            return DEPOT_ENTRY_WORLD_IN[n]
        if n == "0":
            return NODE0_WORLD_IN
        n = int(n)
    if n in DEPOT_WORLD_IN:
        return DEPOT_WORLD_IN[n]
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    if 1 <= n <= nodes:
        idx = n - 1
        grid_x = float(idx % cols)
        grid_y = float(idx // cols)
    elif nodes < n <= nodes + 2 * bays:
        is_workstation = n <= nodes + bays
        # Measured position wins over the derived offset for WORKSTATION
        # nodes -- see WORKSTATION_WORLD_IN. Entry nodes are never in that
        # table, so they always fall through to the lattice-aligned formula.
        if is_workstation and n in WORKSTATION_WORLD_IN:
            return WORKSTATION_WORLD_IN[n]
        bay_num = (n - nodes) if is_workstation else (n - nodes - bays)
        b = bay_num - 1
        # Bay numbering bottom-up (B1 = bay row nearest the depot), matching
        # the node numbering. Offsets remeasured on the physical table
        # 2026-07-27 (bay 1, node114/node65) -- the dispatch model's
        # label_to_cell() assumes the entry sits 0.15 grid-units south of the
        # lattice row and the workstation 0.50 south; the real table measures
        # the entry essentially ON the row (~0.00-0.01 south, within noise)
        # and the workstation ~0.44 south. Only this script's copy was
        # corrected -- the dispatch model's assumed offsets are unchanged
        # and may need the same fix if anything there ever drives a real
        # robot into a bay using vision (today it only uses these offsets for
        # the advisor's own internal simulation/reservation math).
        row_up = b // (cols - 1)
        c = b % (cols - 1)
        grid_x = c + 0.5
        grid_y = (row_up + 1) - (0.44 if is_workstation else 0.0)
    else:
        raise ValueError(
            f"node {n} outside 1..{nodes + 2 * bays} for a {rows}x{cols} grid "
            f"({bays} bays)")
    x = GRID_NODE1_WORLD_IN[0] + grid_x * GRID_PITCH_IN
    y = GRID_NODE1_WORLD_IN[1] + grid_y * GRID_PITCH_IN
    return x, y


def is_workstation_node(n: int | str, rows: int, cols: int) -> bool:
    """True iff n is a WORKSTATION node (the dead-end spur past a bay's
    entry -- see node_to_world()'s own numbering comment). Depot labels
    ("D1".."D6", "DE1".."DE6", "0") and lattice/entry nodes are all False.

    ADDED 2026-08-20 -- node_to_world()'s own docstring already referenced
    this function by name ("both must agree on the workstation/entry
    split, see is_workstation_node") since it was written, but it was
    never actually implemented. Real hardware consequence: fleetSupervisor.
    py's VisionLegWorker._execute() (vision-mode dispatch) had no way to
    special-case a workstation arrival, so every node -- including
    workstations -- squared up to its ENTRY heading (heading_between the
    approach leg) instead of the fixed 180deg exit heading every
    workstation's single Northern entry/exit requires. Two real hardware
    collisions (Alvik4 off-table, Alvik1 into the corner tag -- see
    stop_only()'s own docstring) were traced to a DIFFERENT bug in the
    same drive-to-turn handoff, but this missing function is a second,
    independent gap in the same area: an HTML-side fix (generateCommandLines()
    in agv_grid_workstation_solver.html, for color-sensor mode) was
    mistakenly believed to also cover vision mode, which never calls that
    JS function at all -- see _execute()'s own use of this function for
    the real vision-mode fix."""
    if isinstance(n, str):
        if n in DEPOT_SLOT_WORLD_IN or n in DEPOT_ENTRY_WORLD_IN or n == "0":
            return False
        n = int(n)
    if n in DEPOT_WORLD_IN:
        return False
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    return nodes < n <= nodes + bays


def heading_between(x0: float, y0: float, x1: float, y1: float) -> float:
    """yaw_deg (this robot's convention: 90=+x, 180=+y, 270=-x, 0=-y --
    see camera_trajectory.TargetLine for how these two verified points were
    reconciled into dx=sin(yaw), dy=-cos(yaw)) pointing from (x0,y0) toward
    (x1,y1). SNAPPED to the nearest cardinal -- used only for the turn
    between legs (which should target an exact cardinal), not for live
    steering during a leg; see heading_to_target_deg() for that."""
    dx, dy = x1 - x0, y1 - y0
    if abs(dx) >= abs(dy):
        return 90.0 if dx >= 0 else 270.0
    return 180.0 if dy >= 0 else 0.0


def heading_to_target_deg(dx: float, dy: float) -> float:
    """Continuous (non-snapped) yaw_deg pointing from a robot toward a
    target offset by (dx, dy), in this robot's convention (0=-y, 90=+x,
    180=+y, 270=-x). Equivalent of atan2(dy, dx) from the bench-verified
    onboard-odometry driveTo() this was ported from, translated into this
    robot's heading convention: since dx=sin(yaw) and dy=-cos(yaw) (see
    camera_trajectory.TargetLine), yaw = atan2(dx, -dy). Verified against
    all four cardinal directions (dx=1,dy=0 -> 90; dx=0,dy=1 -> 180;
    dx=-1,dy=0 -> 270; dx=0,dy=-1 -> 0) before use."""
    return math.degrees(math.atan2(dx, -dy)) % 360.0


def parse_route_token(tok: str) -> int | str:
    """"9" -> 9, "D3"/"DE5"/"0" -> the matching DEPOT_NODE_TOKENS label
    (passed through as a string; node_to_world() resolves it)."""
    tok = tok.strip()
    if tok in DEPOT_NODE_TOKENS:
        return DEPOT_NODE_TOKENS[tok]
    return int(tok)


def parse_route(route_str: str, rows: int, cols: int) -> list[tuple[int | str, float, float]]:
    """"1,8,16" -> [(1, x1, y1), (8, x8, y8), (16, x16, y16)]. Also accepts
    any "D1".."D6"/"DE1".."DE6"/"0" depot token, e.g. "D3,DE3,0,1,9,114"
    (see DEPOT_NODE_TOKENS)."""
    nodes = [parse_route_token(tok) for tok in route_str.split(",") if tok.strip()]
    if len(nodes) < 2:
        raise ValueError("--route needs at least 2 nodes (a start and a destination)")
    return [(n, *node_to_world(n, rows, cols)) for n in nodes]


class CameraGridNavigator(Node):
    def __init__(self, robot: str, args: argparse.Namespace):
        # Per-robot node name (not a shared "camera_grid_navigate") -- required
        # so fleetSupervisor.py's vision drive mode can run one instance per
        # robot in a single process/ROS graph without name collisions; also
        # just more useful in `ros2 node list` for the single-robot CLI case.
        super().__init__(f"camera_grid_navigate_{robot.lower()}")
        self.robot = robot
        self.args = args
        self.localization_mode = normalize_localization_mode(
            getattr(args, "localization_mode", "camera_assist"))

        # Own dedicated executor (added for fleetSupervisor.py's vision drive
        # mode, 2026-07-30): every internal wait/control loop below spins via
        # self._spin_once() instead of the bare rclpy.spin_once(self, ...),
        # which implicitly uses the process's single global default executor.
        # Confirmed on hardware: with multiple CameraGridNavigator instances
        # running on separate threads (one per robot) alongside
        # FleetSupervisor's own rclpy.spin(node) background thread, that
        # shared global executor raised "RuntimeError: Executor is already
        # spinning" the instant a second thread tried to spin_once through
        # it. A private SingleThreadedExecutor bound only to this node is
        # unaffected by any other node/thread's spinning.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # A wheel setpoint supersedes every older setpoint. Keep only the
        # newest sample so a DDS/WiFi recovery cannot replay a local backlog
        # of stale steering commands.
        qos_wheel = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(String, f"{robot}_cmd", qos_best_effort)
        self.wheel_pub = self.create_publisher(String, f"{robot}_wheel_cmd", qos_wheel)
        # Saved (not discarded) so wait_for_cmd_match() can also confirm
        # THIS subscription sees the robot's status publisher -- confirmed
        # 2026-07-31: cmd_pub matching (get_subscription_count() > 0) is not
        # proof status_sub has ALSO matched; the two are independent DDS
        # discovery events even though both concern the same two nodes, and
        # a real hardware run showed cmd_pub matched (WHEEL_FOLLOW_MODE was
        # sent) while status_sub had not (last_status stayed '' for a full
        # 3s poll, zero callbacks fired -- not a slow ack, no ack traffic
        # arrived at all).
        self.status_sub = self.create_subscription(
            String, f"{robot}_status", self._on_status, qos_status)
        # Saved (not discarded) so callers can poll get_publisher_count() --
        # a freshly-created node's subscription is not instantly matched to
        # apriltag_localize.py's publisher (DDS/rosbridge discovery takes a
        # real, variable amount of time), and racing that discovery against
        # wait_for_fresh_vision()'s own data timeout can burn the whole
        # timeout on discovery alone with zero actual messages received.
        # See VisionLegWorker._run() in fleetSupervisor.py, which waits for
        # this match before starting that timeout.
        self.vision_pose_sub = self.create_subscription(
            String, f"{robot}_vision_pose", self._on_vision_pose, qos_best_effort)
        # Onboard odometry (alvik.get_pose(), published by the firmware's
        # own publish_pose() -- {"x":..,"y":..,"yaw":..,"battery":..,"ms":..},
        # CM/DEG, NOT the same JSON shape as _vision_pose's x_in/y_in/
        # yaw_deg). Added 2026-08-13 for turn_to_heading_rotate_rel(): local
        # yaw with no camera round-trip, fast enough to size a single
        # ROTATE_REL accurately (see that method's docstring for why vision
        # yaw alone is too laggy for this). Never used for x/y position --
        # only yaw_offset-corrected yaw, see corrected_odom_yaw().
        self.odom_pose_sub = self.create_subscription(
            String, f"{robot}_pose", self._on_odom_pose, qos_best_effort)

        self.last_status = ""
        self.mode_ack_seen = False
        self.dwell_done_seen = False
        self.errored = False
        self.error_text = ""

        self.last_pose: Pose | None = None
        self._last_stale_warn_at = 0.0
        self.rotate_rel_done_seen = False  # ROTATE_REL COMPLETE, see turn_to_heading()
        # The firmware always brakes at its hard ROTATE_REL deadline. In
        # camera-capable modes, BRAKED_UNCONFIRMED is a provisional terminal
        # result that must be accepted only after settled vision verifies the
        # requested yaw. Encoder-only mode still requires COMPLETE.
        self.rotate_rel_unconfirmed_seen = False
        # Set by _rotate_rel_and_wait_or_abort() when VISION (not firmware
        # status) ended the rotation early -- see that method's CAMERA
        # EARLY-EXIT block. Holds the settled, at-target pose, so the caller
        # can skip a redundant post-turn settle.
        self.rotate_camera_confirmed_pose: Pose | None = None
        self.pose_reset_seen = False  # POSE_RESET, see reset_onboard_pose()

        # Onboard yaw drift correction (2026-08-13, see corrected_odom_yaw()
        # and resync_yaw_offset()): raw_odom_yaw + yaw_offset ~= vision yaw
        # at the moment of the last resync. reset_pose() zeroes the Alvik's
        # own onboard yaw reference on connect/RESET_POSE, which will not in
        # general match vision's absolute heading convention -- yaw_offset
        # is how the two get reconciled WITHOUT a firmware change (no new
        # UART command near the ROTATE_REL hang history, see that method's
        # docstring). None until the first resync succeeds.
        self.last_odom_yaw: float | None = None
        self.last_odom_yaw_at: float = 0.0
        self.yaw_offset: float | None = None
        self.last_odom_pose: Pose | None = None
        self.odom_anchor: OdomAnchor | None = None
        self.odom_correction_x_in = 0.0
        self.odom_correction_y_in = 0.0
        self.odom_correction_yaw_deg = 0.0

        # Diagnostic instrumentation added 2026-08-06 to directly measure
        # (not just infer from log timestamps) the real gap between the
        # last wheel_cmd this side sent and any WHEEL_CMD_TIMEOUT the
        # firmware reports -- see _on_status()'s use of this and
        # send_wheel() below. Confirms or disproves the theory (root-caused
        # 2026-08-05 from reading the firmware source + the supervisor's
        # wait-loop code, but never directly measured against a live
        # failure) that the firmware's unconditional 300ms STATE_WHEEL_
        # FOLLOW watchdog fires because nothing refreshes wheel_cmd_last_ms
        # while a robot legitimately waits on another robot's reservation.
        self.last_wheel_cmd_at: float | None = None

    def _spin_once(self, timeout_sec: float) -> None:
        self._executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self) -> None:
        # Own executor must release this node before the base class destroys
        # it, or the executor is left holding a dangling reference.
        try:
            self._executor.remove_node(self)
            self._executor.shutdown()
        except Exception:
            pass
        super().destroy_node()

    # -- status / vision callbacks (identical to camera_line_follow.py) ---
    def _on_status(self, msg: String) -> None:
        text = msg.data.strip()
        self.last_status = text
        if self.args.verbose:
            self.get_logger().info(f"  [status] {text}")
        if text.startswith("ERROR"):
            self.errored = True
            self.error_text = text
            if "WHEEL_CMD_TIMEOUT" in text:
                # Diagnostic instrumentation, see last_wheel_cmd_at's own
                # comment in __init__ -- this is the DIRECT measurement
                # (not log-timestamp inference) of the gap that theory is
                # about. Logged unconditionally (not gated behind
                # args.verbose) since it's specifically instrumenting the
                # failure this session is trying to root-cause; remove once
                # confirmed on hardware.
                gap = (None if self.last_wheel_cmd_at is None
                       else time.monotonic() - self.last_wheel_cmd_at)
                gap_str = f"{gap*1000:.0f}ms" if gap is not None else "never sent"
                self.get_logger().warning(
                    f"[DIAG] WHEEL_CMD_TIMEOUT: {gap_str} since this side's "
                    f"last send_wheel() call (firmware brake threshold is "
                    "300ms; terminal timeout is 1000ms)")
        elif text.startswith("BUSY WHEEL_FOLLOW_MODE"):
            self.mode_ack_seen = True
        elif text == "DWELL COMPLETE":
            self.dwell_done_seen = True
        elif text == "ROTATE_REL COMPLETE":
            self.rotate_rel_done_seen = True
        elif text == "ROTATE_REL BRAKED_UNCONFIRMED":
            self.rotate_rel_unconfirmed_seen = True
        elif text.startswith("WHEEL_CMD_STALL_RECOVERED"):
            # FleetSupervisor independently subscribes and logs this once.
            # Keep standalone navigator output available under --verbose
            # without duplicating every fleet recovery warning.
            if self.args.verbose:
                self.get_logger().warning(
                    f"wheel-command stream recovered after firmware safety "
                    f"brake: {text}")
        elif text == "POSE_RESET":
            self.pose_reset_seen = True

    def _on_vision_pose(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            x = float(d["x_in"])
            y = float(d["y_in"])
            yaw = float(d["yaw_deg"])
        except (ValueError, KeyError, TypeError):
            return
        now = time.monotonic()
        prev = self.last_pose
        if prev is not None and now - prev.t < VISION_STALE_SEC:
            elapsed = now - prev.t
            max_jump_in = (POSE_JUMP_MAX_SPEED_IN_PER_SEC * elapsed
                           + POSE_JUMP_MARGIN_IN)
            pos_jump = math.hypot(x - prev.x, y - prev.y)
            yaw_jump = abs(normalize_deg(yaw - prev.yaw))
            if pos_jump > max_jump_in or yaw_jump > YAW_JUMP_REJECT_DEG:
                return
        self.last_pose = Pose(x, y, yaw, now)

        # camera_assist is a complementary filter: transformed wheel
        # odometry predicts every control tick; each accepted AprilTag pose
        # removes a bounded fraction of the accumulated world-frame error.
        # encoder mode deliberately does not enter this branch, and camera
        # mode consumes self.last_pose directly in control_pose().
        if self.localization_mode == "camera_assist":
            predicted = self._odom_world_pose()
            if predicted is not None:
                self.odom_correction_x_in = blended_correction(
                    self.odom_correction_x_in, x - predicted.x,
                    CAMERA_ASSIST_POSITION_ALPHA)
                self.odom_correction_y_in = blended_correction(
                    self.odom_correction_y_in, y - predicted.y,
                    CAMERA_ASSIST_POSITION_ALPHA)
                yaw_error = normalize_deg(yaw - predicted.yaw)
                self.odom_correction_yaw_deg = normalize_deg(
                    blended_correction(
                        self.odom_correction_yaw_deg, yaw_error,
                        CAMERA_ASSIST_YAW_ALPHA))

        # Table-bounds safety stop -- see TABLE_BOUNDS_MARGIN_IN's own
        # comment for why this exists. Checked on every accepted pose,
        # independent of whichever command is currently in flight, so a
        # runaway/misbehaving robot gets an immediate STOP instead of
        # waiting for the current command's own timeout to notice.
        lo = -TABLE_BOUNDS_MARGIN_IN
        hi = TABLE_SIZE_IN + TABLE_BOUNDS_MARGIN_IN
        if not (lo <= x <= hi and lo <= y <= hi) and not self.errored:
            self.errored = True
            self.error_text = f"OFF_TABLE x={x:.1f} y={y:.1f}"
            self.send_cmd("STOP")
            self.get_logger().error(
                f"SAFETY STOP: vision pose x={x:.1f} y={y:.1f} is outside "
                f"the table bounds (0-{TABLE_SIZE_IN:.0f}in +/-"
                f"{TABLE_BOUNDS_MARGIN_IN:.0f}in margin) -- sent STOP, "
                "aborting current command")

    def _on_odom_pose(self, msg: String) -> None:
        """Firmware's publish_pose(): {"x":..,"y":..,"yaw":..,"battery":..,
        "ms":..} -- a DIFFERENT key set than _on_vision_pose's x_in/y_in/
        yaw_deg (this one is CM/DEG straight from alvik.get_pose(), no unit
        suffix). Position, yaw, and timestamp feed encoder/assisted route
        localization; corrected_odom_yaw() also retains this raw yaw for the
        older ROTATE_REL measurement tools. No jump-rejection like
        _on_vision_pose's: this is a much higher-trust, lower-latency local
        link (no camera/rosbridge round-trip), and turn_to_heading_rotate_
        rel() only ever reads the single latest value right before a turn,
        never integrates it over time the way drive_leg() does with vision."""
        try:
            d = json.loads(msg.data)
            x_in = float(d["x"]) / 2.54
            y_in = float(d["y"]) / 2.54
            yaw = float(d["yaw"])
        except (ValueError, KeyError, TypeError):
            return
        now = time.monotonic()
        self.last_odom_pose = Pose(x_in, y_in, yaw, now)
        self.last_odom_yaw = yaw
        self.last_odom_yaw_at = now

    def corrected_odom_yaw(self) -> float | None:
        """Onboard yaw translated into vision's absolute heading frame via
        the last resync_yaw_offset() reading. Returns None if no odom
        sample has ever arrived, OR no resync has happened yet (yaw_offset
        is None) -- an uncorrected raw onboard yaw is meaningless here since
        reset_pose()'s zero point has no defined relationship to vision's
        heading convention until offset by a real vision-anchored sample."""
        if self.last_odom_yaw is None or self.yaw_offset is None:
            return None
        return normalize_deg(self.last_odom_yaw + self.yaw_offset)

    def fresh_odom_pose(self) -> Pose | None:
        pose = self.last_odom_pose
        if pose is None or time.monotonic() - pose.t > ODOM_STALE_SEC:
            return None
        return pose

    def ensure_odom_anchor(self, world_x_in: float, world_y_in: float,
                           world_yaw_deg: float) -> bool:
        """Anchor raw onboard odometry to a known route start pose once.

        The first movement segment supplies its known ``from`` node and
        heading. Subsequent calls intentionally keep the original anchor so
        encoder drift remains observable rather than being silently reset at
        every node. camera_assist removes drift only through camera updates.
        """
        if self.odom_anchor is not None:
            return True
        raw = self.fresh_odom_pose()
        if raw is None:
            return False
        self.odom_anchor = OdomAnchor(
            raw_x_in=raw.x, raw_y_in=raw.y, raw_yaw_deg=raw.yaw,
            world_x_in=world_x_in, world_y_in=world_y_in,
            world_yaw_deg=world_yaw_deg)
        self.odom_correction_x_in = 0.0
        self.odom_correction_y_in = 0.0
        self.odom_correction_yaw_deg = 0.0

        # Start fused mode aligned to the current camera observation rather
        # than spending several filter frames converging from the nominal
        # schedule point. This is the only full-strength camera correction;
        # later updates use the bounded gains above.
        if self.localization_mode == "camera_assist":
            vision = self.fresh_pose()
            base = self._odom_world_pose()
            if vision is not None and base is not None:
                self.odom_correction_x_in += vision.x - base.x
                self.odom_correction_y_in += vision.y - base.y
                self.odom_correction_yaw_deg = normalize_deg(
                    self.odom_correction_yaw_deg
                    + normalize_deg(vision.yaw - base.yaw))
        self.get_logger().info(
            f"localization={self.localization_mode}: odometry anchored raw "
            f"({raw.x:.2f},{raw.y:.2f},{raw.yaw:+.1f}) to world "
            f"({world_x_in:.2f},{world_y_in:.2f},{world_yaw_deg:+.1f})")
        return True

    def _odom_world_pose(self) -> Pose | None:
        raw = self.fresh_odom_pose()
        if raw is None or self.odom_anchor is None:
            return None
        x, y, yaw = odom_to_world(raw.x, raw.y, raw.yaw, self.odom_anchor)
        pose_time = raw.t
        if self.localization_mode == "camera_assist":
            vision = self.fresh_pose()
            if vision is not None:
                pose_time = max(pose_time, vision.t)
        return Pose(
            x + self.odom_correction_x_in,
            y + self.odom_correction_y_in,
            normalize_deg(yaw + self.odom_correction_yaw_deg),
            pose_time)

    def control_pose(self) -> Pose | None:
        """Fresh pose used by route steering/arrival for the selected mode."""
        if self.localization_mode == "camera":
            return self.fresh_pose()
        # encoder and camera_assist both predict from onboard odometry.
        return self._odom_world_pose()

    def localization_source_label(self) -> str:
        return {
            "encoder": "onboard encoder odometry",
            "camera_assist": "encoder + camera-assisted odometry",
            "camera": "camera localization",
        }[self.localization_mode]

    def resync_yaw_offset(self) -> bool:
        """Recomputes yaw_offset from the CURRENT vision + odom readings:
        offset = vision_yaw - raw_odom_yaw, so future corrected_odom_yaw()
        calls track vision's absolute frame. Only call this with the robot
        KNOWN STATIONARY (e.g. right after a turn settles, before the next
        drive_leg()/turn_to_heading_rotate_rel() starts) -- vision and odom
        samples are not timestamp-synchronized, so resyncing while moving
        bakes in whatever gap existed between the two most recent samples
        as if it were true drift. Returns False (leaves yaw_offset
        untouched) if either reading is missing/stale, so a resync attempt
        during a vision dropout can't silently corrupt a previously-good
        offset with garbage."""
        vision = self.fresh_pose()
        if vision is None or self.last_odom_yaw is None:
            return False
        if time.monotonic() - self.last_odom_yaw_at > VISION_STALE_SEC:
            return False
        self.yaw_offset = normalize_deg(vision.yaw - self.last_odom_yaw)
        return True

    def reset_onboard_pose(self, leg_label: str, timeout_sec: float = 3.0) -> bool:
        """Sends RESET_POSE (alvik.reset_pose(0,0,0)) and waits for the
        firmware's POSE_RESET ack. Added 2026-08-13 per explicit user
        direction: yaw_stress_test_route() previously ran encoder ->
        camera_assist -> camera_only back-to-back with NO reset between
        tiers, so 256 rotations' worth of onboard-encoder drift from the
        first two tiers could carry into camera_only's readings even though
        that tier ignores odom for CONTROL -- reset_onboard_pose() should be
        called once at the START of each tier (see yaw_stress_test_route())
        so every tier begins from the same known-zero onboard reference,
        making the 3 tiers a fair comparison rather than a de facto 4th
        variable (accumulated drift) riding along with tier order.

        Invalidates yaw_offset (the old offset was computed against the
        PRE-reset onboard zero point, now meaningless) and last_odom_yaw
        (stale until a fresh _on_odom_pose() sample arrives post-reset) --
        caller should resync_yaw_offset() again once vision confirms the
        robot is stationary after this returns."""
        self.pose_reset_seen = False
        self.send_cmd("RESET_POSE")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.pose_reset_seen:
                break
        else:
            self.get_logger().error(
                f"{leg_label}: no POSE_RESET ack within {timeout_sec:.1f}s -- aborting")
            return False
        self.yaw_offset = None
        self.last_odom_yaw = None
        self.last_odom_pose = None
        self.odom_anchor = None
        self.odom_correction_x_in = 0.0
        self.odom_correction_y_in = 0.0
        self.odom_correction_yaw_deg = 0.0
        return True

    # -- lifecycle ----------------------------------------------------
    def send_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def send_wheel(self, left_rpm: float, right_rpm: float) -> None:
        msg = String()
        msg.data = f"{left_rpm:.1f} {right_rpm:.1f}"
        self.wheel_pub.publish(msg)
        self.last_wheel_cmd_at = time.monotonic()

    def spin_for(self, duration_sec: float) -> None:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < duration_sec:
            self._spin_once(0.05)

    def fresh_pose(self) -> Pose | None:
        pose = self.last_pose
        if pose is None or time.monotonic() - pose.t > VISION_STALE_SEC:
            return None
        return pose

    def wait_for_fresh_vision(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.fresh_pose() is not None:
                return True
        return False

    def wait_for_fresh_odom(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.fresh_odom_pose() is not None:
                return True
        return False

    def wait_for_localization_ready(self, match_timeout_sec: float = 8.0,
                                    data_timeout_sec: float = 8.0) -> str | None:
        """Wait only for the sensor streams required by localization_mode."""
        required: list[tuple[str, object, Callable[[float], bool]]] = []
        if mode_requires_camera(self.localization_mode):
            required.append((
                f"{self.robot}_vision_pose", self.vision_pose_sub,
                self.wait_for_fresh_vision))
        if mode_requires_odom(self.localization_mode):
            required.append((
                f"{self.robot}_pose", self.odom_pose_sub,
                self.wait_for_fresh_odom))

        for topic, subscription, wait_for_data in required:
            t0 = time.monotonic()
            while (rclpy.ok()
                   and time.monotonic() - t0 < match_timeout_sec
                   and subscription.get_publisher_count() == 0):
                self._spin_once(0.1)
            if subscription.get_publisher_count() == 0:
                return f"{topic} subscription never matched a publisher"
            if not wait_for_data(data_timeout_sec):
                return f"{topic} matched but no fresh data arrived"
        return None

    def wait_for_vision_ready(self, match_timeout_sec: float = 8.0,
                              data_timeout_sec: float = 8.0) -> str | None:
        """Two-stage vision-ready check: wait for vision_pose_sub to
        DDS-match apriltag_localize.py's publisher, THEN wait for actual
        data -- not a single flat wait_for_fresh_vision() call.

        ROOT-CAUSED 2026-08-25 (found again via --stop-interrupt-test's
        first real dry-run; ORIGINALLY root-caused on hardware 2026-07-31
        in fleetSupervisor.py's VisionLegWorker._run(), see that method's
        matching comment -- this fix was never carried over to this file's
        own entry points until now): a freshly-created node's subscription
        is not instantly matched to apriltag_localize.py's publisher (DDS/
        rosbridge discovery takes a real, variable amount of time). Racing
        that discovery against a single flat data-arrival timeout can burn
        the WHOLE budget on discovery alone, timing out with zero messages
        received even though apriltag_localize.py is confirmed actively
        publishing and seeing the robot's tag -- indistinguishable, from
        the error message alone, from vision genuinely being unavailable.

        Returns None on success, or a short string describing which stage
        failed (for the caller's own error message) -- never raises."""
        match_t0 = time.monotonic()
        matched = False
        while rclpy.ok() and time.monotonic() - match_t0 < match_timeout_sec:
            self._spin_once(0.1)
            if self.vision_pose_sub.get_publisher_count() > 0:
                matched = True
                break
        if not matched:
            return (f"{self.robot}_vision_pose subscription never matched "
                    f"apriltag_localize.py's publisher within "
                    f"{match_timeout_sec:.0f}s")
        if not self.wait_for_fresh_vision(timeout_sec=data_timeout_sec):
            return (f"subscription matched but no fresh {self.robot}_"
                    f"vision_pose data arrived within {data_timeout_sec:.0f}s")
        return None

    def wait_for_cmd_match(self, timeout_sec: float = 5.0,
                           recreate_after_sec: float = 2.5) -> bool:
        """Block until BOTH directions of the command/status link are
        matched: cmd_pub has a subscriber (the robot's /<robot>_cmd
        subscription) AND status_sub has a publisher (the robot's
        /<robot>_status publisher) -- before any send_cmd() call.

        REAL BUG confirmed on hardware 2026-07-31, in three parts:

        (1) enter_wheel_follow_mode() used to call
        send_cmd("WHEEL_FOLLOW_MODE") immediately on node construction, with
        nothing having ever waited for pub/sub discovery first. cmd_pub is
        BEST_EFFORT, so a publish before the match completes is silently
        dropped -- no error, and the poll loop only waits for an ACK, never
        re-sending the original command.

        (2) After fixing (1) to wait for cmd_pub's OWN match, a hardware
        retry still failed the same way -- but this time with --verbose
        proof that WHEEL_FOLLOW_MODE really was sent (cmd_pub matched) and
        the full 3s ack-poll produced ZERO status callbacks, not a slow
        ack. cmd_pub matching does NOT prove status_sub has also matched --
        they are two independent DDS discovery events (subscriber-sees-
        publisher on this node's cmd topic vs. publisher-sees-subscriber on
        the robot's status topic), even though both concern the same two
        ROS2 nodes. Both directions must be confirmed before trusting a
        send/ack round trip.

        (3) After fixing (1) and (2), hardware testing showed a genuinely
        INTERMITTENT failure -- diagnostic logging (added, then removed
        after confirming this) proved the poll loop itself was correct:
        rclpy.ok() stayed True the whole time, ~50 real ticks/second ran for
        the full 5s, yet status_sub's publisher count stayed 0 the entire
        window on a real failing run, while a separate `ros2 topic echo
        /Alvik1_status` in another terminal connected to the SAME topic in
        under a second at the same time. A passive wait cannot recover from
        a lost/dropped discovery announcement -- if the packet that would
        have completed the handshake never arrives, waiting longer for it
        changes nothing. Fixed by, after recreate_after_sec with no match,
        DESTROYING and RECREATING cmd_pub and status_sub (a fresh
        create_publisher()/create_subscription() forces a brand new
        discovery announcement, giving the handshake a real second chance
        instead of waiting on one that may already be lost) -- repeated
        until timeout_sec is exhausted. This is a standard DDS-level
        recovery for exactly this failure mode; a passive-only wait
        (however long) cannot substitute for it."""
        t0 = time.monotonic()
        last_recreate = 0.0
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if (self.cmd_pub.get_subscription_count() > 0
                    and self.status_sub.get_publisher_count() > 0):
                return True
            elapsed = time.monotonic() - t0
            if elapsed - last_recreate >= recreate_after_sec:
                last_recreate = elapsed
                self.get_logger().warning(
                    f"  [rearm] no cmd/status match after {elapsed:.1f}s -- "
                    "recreating cmd_pub/status_sub to force fresh discovery")
                self._recreate_cmd_status_endpoints()
        return False

    def _recreate_cmd_status_endpoints(self) -> None:
        """Destroy and recreate cmd_pub and status_sub with identical
        topic/QoS/callback -- see wait_for_cmd_match()'s docstring, part
        (3), for why a passive wait alone cannot recover a lost discovery
        announcement."""
        qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        try:
            self.destroy_publisher(self.cmd_pub)
        except Exception:
            pass
        try:
            self.destroy_subscription(self.status_sub)
        except Exception:
            pass
        self.cmd_pub = self.create_publisher(
            String, f"{self.robot}_cmd", qos_best_effort)
        self.status_sub = self.create_subscription(
            String, f"{self.robot}_status", self._on_status, qos_status)

    def enter_wheel_follow_mode(self, timeout_sec: float = 3.0) -> bool:
        """Confirmed 2026-07-28 (5th-workstation leg, node35 square-up): the
        firmware stamps wheel_cmd_last_ms the MOMENT it processes
        WHEEL_FOLLOW_MODE (see .ino), not when our ack arrives here -- if the
        BUSY WHEEL_FOLLOW_MODE round trip over rosbridge/websocket is slow
        (observed once alongside an already-flaky 1.18s vision-pose stall),
        the 300ms WHEEL_CMD_BRAKE_MS watchdog can already be most of the
        way elapsed by the time this function returns, and the caller
        (turn_to_heading()/drive_leg()) still has its own tolerance-check +
        logging to do before its first send_wheel() -- enough to trip the
        watchdog again before the robot ever got a fresh command, producing
        an ERROR WHEEL_CMD_TIMEOUT in the same instant as (or moments after)
        a route step that otherwise looked totally clean. Send a hold
        command immediately on ack, before returning, so wheel_cmd_last_ms
        gets refreshed as close as possible to when the mode actually took
        effect rather than whenever the caller gets around to it.

        Waits for cmd_pub's subscriber match first (see wait_for_cmd_match())
        -- cheap/instant once already matched (the common case, every
        re-arm after the first), but real insurance on the very first call
        of a fresh node, where this used to be the actual failure point."""
        if not self.wait_for_cmd_match():
            self.get_logger().error(
                f"{self.robot}_cmd has no matched subscriber -- is the "
                "robot powered on and connected to the micro-ROS agent?")
            return False
        self.mode_ack_seen = False
        self.send_cmd("WHEEL_FOLLOW_MODE")
        if self.args.verbose:
            self.get_logger().info("  [rearm] WHEEL_FOLLOW_MODE sent, polling for ack...")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.mode_ack_seen:
                if self.args.verbose:
                    self.get_logger().info(
                        f"  [rearm] ack seen after {time.monotonic() - t0:.2f}s")
                self.send_wheel(0.0, 0.0)
                return True
            if self.errored:
                if self.args.verbose:
                    self.get_logger().info(
                        f"  [rearm] errored after {time.monotonic() - t0:.2f}s, giving up")
                return False
        if self.args.verbose:
            self.get_logger().info(f"  [rearm] timed out after {timeout_sec:.1f}s, no ack seen")
        return False

    def stop_and_rearm(self) -> bool:
        """Real STOP (confirmed brake, exits STATE_WHEEL_FOLLOW in firmware)
        followed immediately by re-entering WHEEL_FOLLOW_MODE. Confirmed
        2026-07-27: firmware's wheelCmdCallback drops every /wheel_cmd
        message while current_state != STATE_WHEEL_FOLLOW (silent no-op, no
        error), so a bare STOP between chained legs/turns left the next
        turn_to_heading()/drive_leg() commanding speed into a robot that had
        already fallen back to IDLE -- looked like a stalled turn (spd=70
        logged, yaw not moving). Use this instead of a bare send_cmd("STOP")
        at every intermediate stop; the truly-final stop in run()'s finally
        block stays a plain STOP since nothing drives after it.

        Also confirmed 2026-07-27 (node65 workstation turn): the STOP ->
        WHEEL_FOLLOW_MODE round trip briefly leaves STATE_WHEEL_FOLLOW, which
        races the firmware's own 300ms WHEEL_CMD_BRAKE_MS watchdog if that
        round trip is slow (observed a 1.45s stall once, cause unconfirmed --
        rosbridge/websocket hiccup, not reproduced elsewhere). The watchdog
        firing during OUR OWN deliberate stop published a real ERROR
        WHEEL_CMD_TIMEOUT status, which self._on_status latched into
        self.errored -- a sticky flag nothing ever cleared. The next leg then
        drove for a few good ticks before drive_leg()'s `if self.errored`
        check caught that stale flag and aborted a route that was, by then,
        actually fine. Since a watchdog trip strictly inside this function is
        an expected side effect of the stop itself (not a real fault) as long
        as WHEEL_FOLLOW_MODE is then re-acked cleanly, clear the flag here."""
        self.send_cmd("STOP")
        rearmed = self.enter_wheel_follow_mode()
        if rearmed:
            self.errored = False
            self.error_text = ""
        return rearmed

    def stop_only(self, settle_sec: float = 0.3) -> None:
        """Real STOP, deliberately NOT followed by re-entering
        WHEEL_FOLLOW_MODE -- use this (not stop_and_rearm()) right before
        any ROTATE_REL, and ONLY here.

        ROOT-CAUSED 2026-08-20 after two real hardware collisions (Alvik4
        drove off-table; Alvik1 drove straight through node 0 into the
        corner reference tag -- both immediately after a logged, apparently
        "completed" ROTATE_REL that never actually happened physically):
        every drive_leg() arrival called stop_and_rearm(), which leaves the
        robot ARMED in STATE_WHEEL_FOLLOW (actively watching for /wheel_cmd
        setpoints), not truly stationary/IDLE. turn_to_heading_rotate_rel()
        was then called immediately after, sending ROTATE_REL -- whose
        firmware handler (AGV_Factory_camera_correction.ino) deliberately
        skips alvik.brake() first, on the explicit (and, until now,
        untested against this exact sequence) assumption that "the robot is
        ALREADY STATIONARY between commands". If any residual wheel motion
        or a stray leftover setpoint was still live in STATE_WHEEL_FOLLOW at
        that moment, alvik.rotate() could get called on a robot that was
        still physically moving -- and ROTATE_REL's completion is a blind
        millis() timer with NO real ack (see ROTATE_DEG_PER_SEC's comment in
        the .ino for why that design was chosen), so the firmware has no way
        to detect the difference and reports COMPLETE regardless of what
        the robot actually did. The isolated --rotate-test/--yaw-stress-test
        bench tooling that "proved" ROTATE_REL reliable (128/128 clean) never
        exercised this: neither ever calls enter_wheel_follow_mode() at all
        (see their own "No enter_wheel_follow_mode() here" comments) -- the
        bench test's robot starts genuinely idle, which a real route's robot
        is not, between legs.

        Fix: drive_leg() now ends here instead of stop_and_rearm() -- a
        real STOP (exits STATE_WHEEL_FOLLOW to firmware IDLE, matching what
        ROTATE_REL's handler assumes) plus a short settle wait for any
        residual motion to actually stop. WHEEL_FOLLOW_MODE is re-armed
        instead at the START of drive_leg()'s own control loop, i.e. only
        once something is actually about to stream wheel setpoints again --
        so a straight multi-leg run (drive-drive-drive, no rotation between)
        still re-arms every leg as before, just one step later, and a
        drive-then-turn transition (the common case, and the one that
        failed twice) never hands off into ROTATE_REL while still armed.

        settle_sec is a FIXED delay, not vision-verified stationarity --
        deliberate choice (2026-08-20): STOP already calls alvik.brake() in
        firmware, which has no comparable failure history to ROTATE_REL's,
        and a vision-dependent check would add a new failure mode (vision
        briefly stale right at this exact moment) to what is meant to be a
        safety fix. Revisit only with real evidence a fixed delay isn't
        enough.

        Briefly raised to 0.5s on 2026-08-25 to absorb a 200ms firmware
        block introduced by a cancel-then-brake STOP handler; that firmware
        change was bench-measured as a regression (--stop-interrupt-test
        2/9 -> 0/9) and reverted the same day, so this is back to 0.3.
        Do not raise it again without a firmware reason -- see
        ROTATE_REL_HAZARD_2026-08-20.md."""
        # ASSERT the stop for the whole window instead of firing once and
        # idling (2026-08-29). ROOT CAUSE, measured twice:
        #
        #   run_1404, node 143: "brake fired (1.86in short)" logged normally,
        #   then the robot was 5.7in PAST the target and still at cruise when
        #   the next window opened -- it never decelerated at all. 25in of
        #   uncommanded travel followed.
        #   run4_1316, DE4: same signature, but DE4 sits 3.2in from the south
        #   table edge, so Alvik4 went off it.
        #
        # The old body sent ONE "STOP" and then spun for 0.3s doing nothing.
        # Firmware STOP is alvik.brake() -> drive(0,0) -> a 'V' (velocity)
        # packet, while the wheels were being driven by streamed
        # set_wheels_speed() 'J' packets. A single 'V' does not reliably
        # override a standing 'J' setpoint, and STOP moves the firmware to
        # IDLE -- whose case body is `break`, commanding the motors nothing
        # ever again. The WHEEL_CMD_TIMEOUT watchdog cannot save it either,
        # because that watchdog lives INSIDE STATE_WHEEL_FOLLOW and STOP has
        # just left it. So one lost packet = a robot driving with nothing
        # watching.
        #
        # Evidence for asserting repeatedly: the thing that finally stopped
        # Alvik4 in run_1404 was the physical cancel button, which runs
        # alvik.brake() EVERY loop iteration -- 28 consecutive
        # ERROR EMERGENCY_STOP lines in that log. Same primitive, asserted
        # rather than fired once.
        #
        # Order matters. Zero the wheels FIRST on the same 'J' channel that
        # commanded them, while the firmware is still in STATE_WHEEL_FOLLOW
        # and therefore still acting on wheel_cmd (it ignores it in any other
        # state). Only then STOP, repeatedly, which brakes and exits the mode.
        # This costs NO extra time: settle_sec is unchanged and was previously
        # spent idle. The quiet tail keeps UART silent just before the next
        # ROTATE_REL -- see the firmware's own warning about traffic
        # immediately preceding alvik.rotate().
        WHEEL_ZERO_SEC = 0.10
        STOP_BURST_SEC = 0.12
        SEND_INTERVAL_SEC = 0.03
        t0 = time.monotonic()
        last_send = 0.0
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= settle_sec:
                break
            if now - last_send >= SEND_INTERVAL_SEC:
                if elapsed < WHEEL_ZERO_SEC:
                    # Still in STATE_WHEEL_FOLLOW: zero the setpoint on the
                    # channel that is actually driving the wheels, and keep
                    # the watchdog fed so we stay in the mode that honours it.
                    self.send_wheel(0.0, 0.0)
                elif elapsed < WHEEL_ZERO_SEC + STOP_BURST_SEC:
                    self.send_cmd("STOP")
                last_send = now
            self._spin_once(0.01)
        self.errored = False
        self.error_text = ""

    def wait_for_yaw_settled(self, leg_label: str, timeout_sec: float = 3.0,
                             stable_tol_deg: float = 1.0,
                             min_newer_than: float | None = None
                             ) -> tuple[Pose | None, str]:
        """Waits until vision yaw is STABLE (spread <= stable_tol_deg over a
        ~0.25s rolling window of consecutive samples) and returns
        (settled_pose, "settled") -- or (None, "still_moving") if fresh
        vision kept arriving but never stabilized within timeout_sec, or
        (None, "vision_stale") if no usable fresh sample arrived at all.

        ROOT-CAUSED 2026-08-25 via --stop-interrupt-test (9 real trials,
        Alvik1 -- full table in ROTATE_REL_HAZARD_2026-08-20.md): the
        firmware's ROTATE_REL COMPLETE was then a blind timer ending in a
        brake() that did NOT reliably halt a rotation already at speed (7/9 trials
        kept rotating 4-21deg after a mid-rotation brake/STOP, ~constant
        150-250ms takeover latency). Current firmware instead target-gates
        and brakes before COMPLETE. This independent vision settle remains
        as defense-in-depth because turn_to_heading_rotate_rel() once sampled
        vision for its camera-assist check IMMEDIATELY
        after the ack, no settle, unlike every bench tool (rotate_test()/
        yaw_stress_rotation(), which always settle ~0.5s before measuring
        and whose accuracy numbers were correspondingly clean). Measuring a
        still-rotating robot sizes the corrective ROTATE_REL from a moving
        target, which matches the observed erratic corrections (errors
        getting WORSE after a correction) far better than a broadly-bad
        rotate primitive does.

        A time-windowed spread (not consecutive-sample deltas) is required:
        at ~59Hz a robot still rotating 20deg/s only moves ~0.34deg between
        adjacent samples -- well under any sane tolerance -- so
        sample-to-sample comparison would false-positive "settled" on a
        moving robot. 0.25s of window at 20deg/s = 5deg spread: correctly
        not settled."""
        WINDOW_SEC = 0.25
        t0 = time.monotonic()
        window: list[tuple[float, float]] = []  # (pose.t, yaw)
        saw_fresh = False
        # DIAGNOSTICS (2026-08-25, added after the Alvik1 node-137 abort):
        # the old "still_moving" return logged NO numbers at its call sites,
        # so three completely different faults were indistinguishable in the
        # supervisor log -- (a) a genuine runaway rotation, (b) the motor
        # co-processor HUNTING a few tenths either side of its setpoint and
        # never converging, (c) plain vision yaw noise drifting just over
        # stable_tol_deg on a robot that is actually stopped. Each needs a
        # different fix (firmware cancel-rotate / rotate tuning / a looser
        # tolerance), and picking wrong costs a hardware session. Record
        # enough here to tell them apart from the log alone next time:
        #   net yaw ~= total yaw travel, both large -> (a) real rotation
        #   total yaw travel >> |net yaw|            -> (b) oscillation
        #   best window spread only just over tol    -> (c) noise/threshold
        n_samples = 0
        first_p: Pose | None = None
        last_p: Pose | None = None
        yaw_path_deg = 0.0        # summed |sample-to-sample| yaw change
        yaw_lo = yaw_hi = 0.0     # excursion envelope, relative to first
        best_spread = math.inf    # closest any window came to settling
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.02)
            p = self.fresh_pose()
            if p is None:
                continue
            if min_newer_than is not None and p.t <= min_newer_than:
                continue
            saw_fresh = True
            if window and p.t == window[-1][0]:
                continue  # same sample as last loop pass
            if first_p is None:
                first_p = p
            else:
                yaw_path_deg += abs(normalize_deg(p.yaw - last_p.yaw))
                rel = normalize_deg(p.yaw - first_p.yaw)
                yaw_lo = min(yaw_lo, rel)
                yaw_hi = max(yaw_hi, rel)
            last_p = p
            n_samples += 1
            window.append((p.t, p.yaw))
            cutoff = p.t - WINDOW_SEC
            window = [(t, y) for (t, y) in window if t >= cutoff]
            if (len(window) >= 3
                    and window[-1][0] - window[0][0] >= 0.8 * WINDOW_SEC):
                deltas = [normalize_deg(y - window[0][1]) for _t, y in window]
                spread = max(deltas) - min(deltas)
                best_spread = min(best_spread, spread)
                if spread <= stable_tol_deg:
                    return p, "settled"

        if not saw_fresh:
            self.get_logger().error(
                f"{leg_label}: NOT settled -- no fresh vision at all in "
                f"{timeout_sec:.1f}s, so whether the robot is moving is "
                "UNKNOWN (treat as moving)")
            return None, "vision_stale"

        net_yaw = (normalize_deg(last_p.yaw - first_p.yaw)
                   if first_p is not None and last_p is not None else 0.0)
        moved_in = (math.hypot(last_p.x - first_p.x, last_p.y - first_p.y)
                    if first_p is not None and last_p is not None else 0.0)
        if best_spread <= 2.0 * stable_tol_deg and yaw_path_deg < 5.0:
            verdict = "looks like vision NOISE near the tolerance, not motion"
        elif yaw_path_deg > 3.0 * max(abs(net_yaw), 1.0):
            verdict = "looks like the rotate controller HUNTING (oscillating)"
        else:
            verdict = "looks like REAL continued rotation"
        self.get_logger().error(
            f"{leg_label}: NOT settled in {timeout_sec:.1f}s -- "
            f"{n_samples} vision samples, yaw net {net_yaw:+.2f}deg "
            f"(envelope {yaw_lo:+.2f}..{yaw_hi:+.2f}), total yaw travel "
            f"{yaw_path_deg:.2f}deg, best {WINDOW_SEC:.2f}s window spread "
            f"{best_spread:.2f}deg vs tol {stable_tol_deg:.2f}deg; "
            f"position moved {moved_in:.2f}in "
            f"({first_p.x:.1f},{first_p.y:.1f})->({last_p.x:.1f},"
            f"{last_p.y:.1f}) -- {verdict}")
        return None, "still_moving"

    def stop_and_verify(self, leg_label: str, timeout_sec: float = 3.0,
                        resend_interval_sec: float = 0.4) -> bool:
        """STOP that does not trust a single send: re-sends STOP every
        resend_interval_sec until vision confirms the robot is actually
        stationary (yaw AND position stable over a ~0.25s window), or the
        timeout expires. Returns True only on verified stationarity.

        Motivated by the same 2026-08-25 --stop-interrupt-test data as
        wait_for_yaw_settled() (see its docstring): one STOP sent while a
        ROTATE_REL is at speed left 4-21deg of continued rotation in 7/9
        trials. A REPEATED stop is strictly stronger -- once the
        co-processor's rotate maneuver ends (or its ~150-250ms takeover
        latency elapses), the next 'V'(0,0) lands on a receptive state.
        NOTE on the old 2026-07-30 fear that STOP-mid-rotation corrupts the
        UART/ack state and hangs the firmware (see rotate_test()'s timeout
        comment): the 9 bench trials sent STOP mid-rotation deliberately,
        back to back, with ZERO hangs and status flowing throughout -- that
        fear is now empirically much weaker, at least for this firmware
        build. If vision is stale the whole time this still re-sends STOP
        blind for the full timeout (strictly better than the old single
        send) and returns False (unverified)."""
        WINDOW_SEC = 0.25
        POS_TOL_IN = 0.15
        YAW_TOL_DEG = 1.0
        self.send_cmd("STOP")
        sends = 1
        t0 = time.monotonic()
        last_send = t0
        window: list[tuple[float, float, float, float]] = []  # (t, x, y, yaw)
        # DIAGNOSTICS -- same rationale as wait_for_yaw_settled()'s (see
        # there). "STOP NOT verified" previously said nothing about HOW the
        # robot failed to stop, which is the single most important fact
        # after an incident: distance travelled answers "did it drive
        # forward or spin in place?" directly, and travelled-vs-net
        # separates a runaway from noise.
        n_samples = 0
        first_s: tuple[float, float, float] | None = None  # (x, y, yaw)
        last_s: tuple[float, float, float] | None = None
        pos_path_in = 0.0
        yaw_path_deg = 0.0
        best_pos_spread = math.inf
        best_yaw_spread = math.inf
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.02)
            now = time.monotonic()
            if now - last_send >= resend_interval_sec:
                self.send_cmd("STOP")
                sends += 1
                last_send = now
            p = self.fresh_pose()
            if p is None:
                continue
            if window and p.t == window[-1][0]:
                continue
            if first_s is None:
                first_s = (p.x, p.y, p.yaw)
            else:
                pos_path_in += math.hypot(p.x - last_s[0], p.y - last_s[1])
                yaw_path_deg += abs(normalize_deg(p.yaw - last_s[2]))
            last_s = (p.x, p.y, p.yaw)
            n_samples += 1
            window.append((p.t, p.x, p.y, p.yaw))
            cutoff = p.t - WINDOW_SEC
            window = [s for s in window if s[0] >= cutoff]
            if (len(window) >= 3
                    and window[-1][0] - window[0][0] >= 0.8 * WINDOW_SEC):
                _t0, x0, y0, yaw0 = window[0]
                pos_spread = max(math.hypot(x - x0, y - y0)
                                 for _t, x, y, _yaw in window)
                yaw_deltas = [normalize_deg(yaw - yaw0)
                              for _t, _x, _y, yaw in window]
                yaw_spread = max(yaw_deltas) - min(yaw_deltas)
                best_pos_spread = min(best_pos_spread, pos_spread)
                best_yaw_spread = min(best_yaw_spread, yaw_spread)
                if pos_spread <= POS_TOL_IN and yaw_spread <= YAW_TOL_DEG:
                    self.get_logger().info(
                        f"{leg_label}: STOP verified stationary after "
                        f"{now - t0:.2f}s ({sends} STOP send(s))")
                    return True
        if n_samples == 0:
            self.get_logger().error(
                f"{leg_label}: STOP NOT verified within {timeout_sec:.1f}s "
                f"({sends} STOP send(s)) -- NO vision samples at all, robot "
                "motion UNKNOWN (treat as moving)")
            return False
        net_in = math.hypot(last_s[0] - first_s[0], last_s[1] - first_s[1])
        net_yaw = normalize_deg(last_s[2] - first_s[2])
        self.get_logger().error(
            f"{leg_label}: STOP NOT verified stationary within "
            f"{timeout_sec:.1f}s ({sends} STOP send(s)) -- {n_samples} "
            f"vision samples, travelled {pos_path_in:.2f}in along path "
            f"(net {net_in:.2f}in, ({first_s[0]:.1f},{first_s[1]:.1f})->"
            f"({last_s[0]:.1f},{last_s[1]:.1f})), yaw travel "
            f"{yaw_path_deg:.2f}deg (net {net_yaw:+.2f}deg); best window "
            f"pos spread {best_pos_spread:.2f}in vs tol {POS_TOL_IN:.2f}in, "
            f"best yaw spread {best_yaw_spread:.2f}deg vs tol "
            f"{YAW_TOL_DEG:.2f}deg")
        return False

    # -- stopping-distance measurement (2026-07-28) -----------------------
    def stop_test(self, target_x: float, target_y: float, cruise_rpm: float,
                   brake_lead_in: float, leg_label: str) -> dict | None:
        """Drive at a CONSTANT base speed cruise_rpm (no proportional
        slowdown, unlike drive_leg()'s kp_dist law) until dist_to_target <=
        brake_lead_in, then fire one hard STOP. Reports target vs. actual
        final (x,y) and the resulting position error, so brake_lead_in can be
        tuned from real stopping-distance data instead of guessed -- this is
        the user's explicit request 2026-07-28: keep RPM at a steady cruise
        value (matching the old color-sensor firmware's 60-70 RPM constant-
        speed-then-hard-stop behavior) and find how much lead distance the
        brake needs, rather than slowing the commanded RPM down on approach.

        Applies the SAME live-target heading correction as drive_leg()
        (kp_yaw * heading error, clamped to max_turn_adjust, added/subtracted
        from the constant cruise_rpm base) -- added 2026-07-28 after chaining
        legs 1->8 open-loop drifted steadily off the taped line (y crept
        16.9->17.1->17.6->18.2->18.4in over 4 legs with no reset between
        them, since nothing was correcting the small real left/right wheel
        mismatch at 60RPM) and the robot eventually missed node8 by enough
        that it had to be picked up and placed by hand. Without correction
        this test only measured stopping distance; with it, it measures
        stopping distance AND straight-line tracking together, which is what
        the real route needs from a constant-cruise-then-brake law.

        Returns a result dict on success (does NOT re-arm WHEEL_FOLLOW_MODE --
        the caller, e.g. stop_test_route(), controls when/if the next leg
        starts) or None on abort (vision lost / errored)."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting")
            return None

        start_x, start_y = pose.x, pose.y
        total_dist = math.hypot(target_x - start_x, target_y - start_y)
        self.get_logger().info(
            f"{leg_label}: stop-test from ({start_x:.1f},{start_y:.1f}) to "
            f"({target_x:.1f},{target_y:.1f}) [{total_dist:.1f}in], "
            f"cruise={cruise_rpm:.0f}RPM, brake_lead={brake_lead_in:.2f}in")

        # Stall backstop -- confirmed 2026-07-28: a watchdog trip (ERROR
        # WHEEL_CMD_TIMEOUT -> IDLE) mid-leg can land in the gap between this
        # loop's non-blocking rclpy.spin_once(timeout_sec=0.0) calls, so
        # self.errored isn't observed as True on the very next tick if the
        # status callback hadn't been delivered/processed yet. The robot then
        # sits stopped (firmware dropped out of STATE_WHEEL_FOLLOW) while this
        # loop keeps waiting for dist_to_target to shrink, which it never
        # will -- an unbounded hang with no error and no route abort. Track
        # elapsed time with no forward progress and bail out rather than spin
        # forever; drive_leg() has an analogous backstop (closest_dist/
        # overshoot_margin_in) for the same class of "loop keeps running but
        # the robot isn't" failure.
        no_progress_timeout_sec = 5.0
        best_dist = math.inf
        t_progress0 = time.monotonic()

        period = 1.0 / CONTROL_HZ
        while rclpy.ok():
            loop_t0 = time.monotonic()
            self._spin_once(0.0)

            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting")
                return None

            pose = self.fresh_pose()
            if pose is None:
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: vision lost -- STOP sent, aborting")
                return None

            dx = target_x - pose.x
            dy = target_y - pose.y
            dist_to_target = math.hypot(dx, dy)

            if dist_to_target < best_dist - 0.05:
                best_dist = dist_to_target
                t_progress0 = loop_t0
            elif loop_t0 - t_progress0 > no_progress_timeout_sec:
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: no progress for {no_progress_timeout_sec:.0f}s "
                    f"(stuck at {dist_to_target:.2f}in from target, last "
                    f"status '{self.last_status}') -- STOP sent, aborting")
                return None

            if dist_to_target <= brake_lead_in:
                brake_pose = pose
                self.send_cmd("STOP")
                self.get_logger().info(
                    f"{leg_label}: brake fired at ({brake_pose.x:.2f},"
                    f"{brake_pose.y:.2f}), {dist_to_target:.2f}in short of "
                    "target")
                break

            target_heading = heading_to_target_deg(dx, dy)
            heading_err = yaw_error_deg(target_heading, pose.yaw)
            turn_adjust = max(-self.args.max_turn_adjust,
                               min(self.args.max_turn_adjust,
                                   self.args.kp_yaw * heading_err))
            left = cruise_rpm - turn_adjust
            right = cruise_rpm + turn_adjust
            self.send_wheel(left, right)
            if self.args.verbose:
                self.get_logger().info(
                    f"  x={pose.x:6.1f} y={pose.y:6.1f} yaw={pose.yaw:+6.1f}  "
                    f"hdg_err={heading_err:+5.1f}deg  "
                    f"dist_to_target={dist_to_target:5.2f}in  "
                    f"L={left:+5.1f} R={right:+5.1f}")
            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, period - elapsed))

        # Let the robot actually come to rest before reading the final pose --
        # this is exactly the number we're trying to measure (how far it
        # slides after brake()), so wait out a settle window rather than
        # sampling mid-slide.
        self.spin_for(1.0)
        final_pose = self.fresh_pose()
        if final_pose is None:
            self.get_logger().error(f"{leg_label}: lost vision after brake -- "
                                     "cannot report final position")
            return None

        final_dx = target_x - final_pose.x
        final_dy = target_y - final_pose.y
        final_err = math.hypot(final_dx, final_dy)
        slide_dist = math.hypot(final_pose.x - brake_pose.x,
                                 final_pose.y - brake_pose.y)
        # Sign of the along-path error: positive = past target (brake_pose ->
        # final_pose traveled FARTHER than brake_pose -> target did), negative
        # = short. Uses the dot product of the slide vector onto the
        # brake->target direction rather than a distance comparison, so it
        # stays correct even with the lateral/heading drift seen on longer
        # legs (2026-07-28 run: yaw drifted 91.6->92.7deg over one leg with
        # no heading correction, giving a final position offset that wasn't
        # purely along the brake->target line).
        bt_dx, bt_dy = target_x - brake_pose.x, target_y - brake_pose.y
        bt_len = math.hypot(bt_dx, bt_dy)
        if bt_len > 1e-6:
            along = ((final_pose.x - brake_pose.x) * bt_dx +
                     (final_pose.y - brake_pose.y) * bt_dy) / bt_len
        else:
            along = 0.0
        past_target_in = along - bt_len  # >0 = overshot past target, <0 = stopped short
        self.get_logger().info(
            f"{leg_label}: RESULT final=({final_pose.x:.2f},{final_pose.y:.2f}) "
            f"target=({target_x:.2f},{target_y:.2f}) "
            f"error={final_err:.2f}in  slide-after-brake={slide_dist:.2f}in "
            f"(brake fired {dist_to_target:.2f}in short of target)  "
            f"along-path={'+' if past_target_in >= 0 else ''}{past_target_in:.2f}in "
            f"({'past' if past_target_in >= 0 else 'short of'} target)")

        return {
            "leg_label": leg_label,
            "target": (target_x, target_y),
            "final": (final_pose.x, final_pose.y),
            "brake_point": (brake_pose.x, brake_pose.y),
            "brake_lead_in": brake_lead_in,
            "dist_at_brake": dist_to_target,
            "slide_after_brake": slide_dist,
            "final_error": final_err,
            "past_target_in": past_target_in,
        }

    def stop_test_route(self, route: list[tuple[int, float, float]],
                         cruise_rpm: float, brake_lead_in: float,
                         settle_sec: float = 0.5) -> None:
        """Chain stop_test() across every leg of a multi-node route (e.g.
        1,2,3,4,5,6,7,8), re-arming WHEEL_FOLLOW_MODE between legs, so one run
        gives multiple real stopping-distance samples instead of one per
        process invocation -- 2026-07-28, requested after 3 separate
        single-leg runs each needed a manual rerun/rearm and only gave one
        data point apiece. Prints a summary table across all completed legs
        at the end (or on abort, using whatever legs did complete).

        Confirmed 2026-07-28 (node2->node3 handoff): a blind spin_for(0.5)
        between legs let the firmware's former 300ms terminal watchdog
        trip (enter_wheel_follow_mode()'s post-ack send_wheel(0,0) is a
        ONE-SHOT hold, not a keepalive -- see that function's docstring),
        latching self.errored, which this function then never cleared before
        the next leg's stop_test() saw it and aborted instantly on a robot
        that was actually fine. Re-send the hold every tick during the
        settle wait (like a mini control loop) instead of one blind sleep,
        and clear a rearm-induced errored flag exactly like
        stop_and_rearm()."""
        results: list[dict] = []
        for i in range(1, len(route)):
            prev_n, _, _ = route[i - 1]
            n, tx, ty = route[i]
            leg_label = f"stop-test node{prev_n}->node{n}"
            result = self.stop_test(tx, ty, cruise_rpm, brake_lead_in, leg_label)
            if result is None:
                self.get_logger().error(
                    f"{leg_label}: aborting remainder of stop-test route")
                break
            results.append(result)
            if i < len(route) - 1:
                if not self.enter_wheel_follow_mode():
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE -- "
                        "aborting remainder of stop-test route")
                    break
                self.errored = False
                self.error_text = ""
                # Brief settle so the next leg starts from a fully-at-rest
                # pose -- keep re-sending the hold every tick (well under
                # WHEEL_CMD_BRAKE_MS=300ms) instead of one blind sleep, so
                # the watchdog never sees a gap.
                t_settle0 = time.monotonic()
                period = 1.0 / CONTROL_HZ
                while rclpy.ok() and time.monotonic() - t_settle0 < settle_sec:
                    loop_t0 = time.monotonic()
                    self._spin_once(0.0)
                    self.send_wheel(0.0, 0.0)
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, period - elapsed))
                self.errored = False
                self.error_text = ""

        if not results:
            return
        self.get_logger().info(
            f"stop-test route summary ({len(results)}/{len(route) - 1} legs, "
            f"cruise={cruise_rpm:.0f}RPM, brake_lead={brake_lead_in:.2f}in):")
        for r in results:
            self.get_logger().info(
                f"  {r['leg_label']:<28s} slide={r['slide_after_brake']:5.2f}in  "
                f"along-path={'+' if r['past_target_in'] >= 0 else ''}"
                f"{r['past_target_in']:5.2f}in  final-err={r['final_error']:4.2f}in")
        slides = [r["slide_after_brake"] for r in results]
        alongs = [r["past_target_in"] for r in results]
        n = len(results)
        self.get_logger().info(
            f"  slide-after-brake: mean={sum(slides)/n:.2f}in "
            f"min={min(slides):.2f}in max={max(slides):.2f}in")
        self.get_logger().info(
            f"  along-path (+=past target): mean={sum(alongs)/n:+.2f}in "
            f"min={min(alongs):+.2f}in max={max(alongs):+.2f}in")

    # -- turn stopping-angle measurement (2026-07-28) ----------------------
    def turn_test(self, target_heading_deg: float, turn_rpm: float,
                   brake_lead_deg: float, leg_label: str) -> dict | None:
        """Rotate in place at a CONSTANT turn_rpm (no decel-zone taper) until
        |yaw error| <= brake_lead_deg, then fire one hard STOP. Reports
        target vs. actual final heading, so brake_lead_deg can be tuned from
        real angular stopping-distance data -- same rationale as stop_test()
        for straight driving, applied to rotation 2026-07-28 after the user
        reported turn_to_heading()'s THEN-current tapered-decel-zone law
        (turn-min-speed/turn-decel-zone-deg/turn-creep-speed/turn-scale-deg/
        turn-max-speed, all tuned by hand earlier this session) was STILL
        overshooting the target yaw and correcting back, sometimes more than
        once, instead of completing in one smooth rotation -- exactly the
        guessed-not-measured failure mode drive_leg()'s old kp_dist law had.
        This function's measurements are what turn_to_heading() now
        actually uses (see its docstring) -- the tapered law described here
        no longer exists in that function, only in this historical note.

        Returns a result dict on success (does NOT re-arm WHEEL_FOLLOW_MODE --
        the caller, e.g. turn_test_route(), controls when/if the next turn
        starts) or None on abort (vision lost / errored)."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting")
            return None

        start_yaw = pose.yaw
        start_error = yaw_error_deg(target_heading_deg, start_yaw)
        self.get_logger().info(
            f"{leg_label}: turn-test from {start_yaw:+.1f} to "
            f"{target_heading_deg:.0f}deg [{start_error:+.1f}deg], "
            f"turn_rpm={turn_rpm:.0f}, brake_lead={brake_lead_deg:.2f}deg")

        no_progress_timeout_sec = 5.0
        best_abs_error = math.inf
        t_progress0 = time.monotonic()

        period = 1.0 / CONTROL_HZ
        while rclpy.ok():
            loop_t0 = time.monotonic()
            self._spin_once(0.0)

            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting")
                return None

            pose = self.fresh_pose()
            if pose is None:
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: vision lost -- STOP sent, aborting")
                return None
            pose_age_ms = (time.monotonic() - pose.t) * 1000.0

            error = yaw_error_deg(target_heading_deg, pose.yaw)
            abs_error = abs(error)

            if abs_error < best_abs_error - 0.1:
                best_abs_error = abs_error
                t_progress0 = loop_t0
            elif loop_t0 - t_progress0 > no_progress_timeout_sec:
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: no progress for {no_progress_timeout_sec:.0f}s "
                    f"(stuck at {abs_error:.1f}deg error, last status "
                    f"'{self.last_status}') -- STOP sent, aborting")
                return None

            if abs_error <= brake_lead_deg:
                brake_yaw = pose.yaw
                self.send_cmd("STOP")
                self.get_logger().info(
                    f"{leg_label}: brake fired at yaw={brake_yaw:+.1f}, "
                    f"{abs_error:.2f}deg short of target "
                    f"(pose was {pose_age_ms:.0f}ms old at brake decision)")
                break

            if error > 0.0:
                self.send_wheel(-turn_rpm, turn_rpm)
            else:
                self.send_wheel(turn_rpm, -turn_rpm)
            if self.args.verbose:
                # pose_age_ms flags a stale/frozen camera pose being reused
                # across ticks -- confirmed 2026-07-28 as a real suspect
                # after a turn-test run showed yaw reported perfectly flat
                # for ~1.6s of active 35RPM rotation, then jumping several
                # degrees per 20ms tick once fresh data arrived (a stale-
                # pose catch-up burst, not real robot motion). Anything
                # persistently above ~1-2 control periods (here, >40-60ms)
                # means the loop is steering/braking on old information.
                stale_flag = " STALE" if pose_age_ms > 60.0 else ""
                self.get_logger().info(
                    f"  yaw={pose.yaw:+6.1f} err={error:+6.1f}deg "
                    f"pose_age={pose_age_ms:5.0f}ms{stale_flag}")
            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, period - elapsed))
        else:
            # while rclpy.ok() went false without ever hitting the brake
            # break above (e.g. external shutdown/Ctrl+C mid-turn) -- REAL
            # BUG confirmed on hardware 2026-07-31: brake_yaw was only ever
            # assigned inside the brake branch, so falling through here
            # raised UnboundLocalError instead of a clean abort. STOP is
            # still safe to attempt (best-effort, no-op if the context is
            # already gone) and returning None matches every other abort
            # path in this function.
            self.send_cmd("STOP")
            self.get_logger().error(f"{leg_label}: rclpy shut down mid-turn -- aborting")
            return None

        # Let the robot actually come to rest before reading the final yaw --
        # but SAMPLE yaw throughout the settle window instead of a blind
        # spin_for(1.0), added 2026-07-28: the prior blind wait couldn't
        # distinguish "robot kept physically rotating for ~1s after brake"
        # from "camera pose was stale/frozen and only caught up once this
        # settle window gave it time," which turn-test data suggested might
        # be happening (yaw reported flat for ~1.6s of active rotation, then
        # jumped several degrees in a single tick once fresh data arrived).
        settle_t0 = time.monotonic()
        last_seen_yaw = brake_yaw
        while rclpy.ok() and time.monotonic() - settle_t0 < 1.0:
            self._spin_once(0.05)
            p = self.fresh_pose()
            if p is not None and p.yaw != last_seen_yaw:
                if self.args.verbose:
                    age_ms = (time.monotonic() - p.t) * 1000.0
                    self.get_logger().info(
                        f"  [settle] yaw={p.yaw:+6.1f} "
                        f"(+{time.monotonic() - settle_t0:.2f}s since brake, "
                        f"pose_age={age_ms:.0f}ms)")
                last_seen_yaw = p.yaw
        final_pose = self.fresh_pose()
        if final_pose is None:
            self.get_logger().error(f"{leg_label}: lost vision after brake -- "
                                     "cannot report final heading")
            return None

        final_error = yaw_error_deg(target_heading_deg, final_pose.yaw)
        # error (signed, from the loop's last iteration before break) and
        # final_error share the same sign convention (target - current) --
        # if the robot coasted PAST the target after brake, the sign flips
        # (was closing from one side, ends up past zero on the other side).
        overshot_past_target = (final_error > 0) != (error > 0)
        self.get_logger().info(
            f"{leg_label}: RESULT final_yaw={final_pose.yaw:+.1f} "
            f"target={target_heading_deg:.1f} final_error={final_error:+.2f}deg "
            f"(brake fired {abs_error:.2f}deg short)  "
            f"{'OVERSHOT' if overshot_past_target else 'stopped short'} "
            f"by {abs(final_error):.2f}deg")

        return {
            "leg_label": leg_label,
            "target_heading": target_heading_deg,
            "final_yaw": final_pose.yaw,
            "brake_yaw": brake_yaw,
            "brake_lead_deg": brake_lead_deg,
            "error_at_brake": abs_error,
            "final_error": final_error,
            "overshot": overshot_past_target,
        }

    def turn_test_route(self, headings: list[float], turn_rpm: float,
                         brake_lead_deg: float, settle_sec: float = 0.5) -> None:
        """Chain turn_test() across a list of target headings (e.g.
        90,180,270,0,90 to exercise repeated 90deg turns both directions),
        re-arming WHEEL_FOLLOW_MODE between turns, so one run gives multiple
        real angular-stopping-distance samples -- mirrors stop_test_route()'s
        structure exactly, including its fix for the inter-leg watchdog
        stall (send_wheel(0,0) every tick during the settle wait, not one
        blind sleep)."""
        results: list[dict] = []
        for i, target_heading in enumerate(headings):
            leg_label = f"turn-test #{i + 1}->{target_heading:.0f}deg"
            result = self.turn_test(target_heading, turn_rpm, brake_lead_deg, leg_label)
            if result is None:
                self.get_logger().error(
                    f"{leg_label}: aborting remainder of turn-test route")
                break
            results.append(result)
            if i < len(headings) - 1:
                if not self.enter_wheel_follow_mode():
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE -- "
                        "aborting remainder of turn-test route")
                    break
                self.errored = False
                self.error_text = ""
                t_settle0 = time.monotonic()
                period = 1.0 / CONTROL_HZ
                while rclpy.ok() and time.monotonic() - t_settle0 < settle_sec:
                    loop_t0 = time.monotonic()
                    self._spin_once(0.0)
                    self.send_wheel(0.0, 0.0)
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, period - elapsed))
                self.errored = False
                self.error_text = ""

        if not results:
            return
        self.get_logger().info(
            f"turn-test route summary ({len(results)}/{len(headings)} turns, "
            f"turn_rpm={turn_rpm:.0f}, brake_lead={brake_lead_deg:.2f}deg):")
        for r in results:
            self.get_logger().info(
                f"  {r['leg_label']:<24s} final_err={r['final_error']:+6.2f}deg  "
                f"{'OVERSHOT' if r['overshot'] else 'short   '}")
        final_errs = [abs(r["final_error"]) for r in results]
        n = len(results)
        overshoot_count = sum(1 for r in results if r["overshot"])
        self.get_logger().info(
            f"  |final_error|: mean={sum(final_errs)/n:.2f}deg "
            f"min={min(final_errs):.2f}deg max={max(final_errs):.2f}deg  "
            f"overshoots={overshoot_count}/{n}")

    def rotate_test(self, target_heading_deg: float, leg_label: str) -> dict | None:
        """Added 2026-07-30, mirrors turn_test() but exercises ROTATE_REL
        (alvik.rotate(), closed-loop on Alvik's motor-control MCU) instead
        of streaming WHEEL_FOLLOW_MODE wheel-speed setpoints -- this is the
        measurement mode used to validate the ROTATE_REL replacement for
        turn_to_heading() before trusting it inside a full route. Sends ONE
        relative-angle command computed from current vision yaw, waits for
        the firmware's ROTATE_REL COMPLETE ack, then (like turn_test())
        keeps sampling vision through a short settle window before reading
        the REAL final heading -- so this reports true accuracy, not just
        "did the firmware say complete."

        Returns a result dict on success (does NOT send another command --
        the caller, e.g. rotate_test_route(), controls what happens next)
        or None on abort (vision lost / errored)."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting")
            return None

        start_yaw = pose.yaw
        rel_deg = yaw_error_deg(target_heading_deg, start_yaw)
        self.get_logger().info(
            f"{leg_label}: rotate-test from {start_yaw:+.1f} to "
            f"{target_heading_deg:.0f}deg via ROTATE_REL {rel_deg:+.2f}deg")

        self.rotate_rel_done_seen = False
        self.rotate_rel_unconfirmed_seen = False
        self.errored = False
        self.error_text = ""
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")

        # RAISED 2026-07-30 after real hardware data showed the previous
        # formula (abs(deg)/15+2, e.g. 8.0s for a -90.3deg turn, 3.0s floor
        # for small angles) was WAY too tight: a real +179.00deg ROTATE_REL
        # took 10.23s to ack (vs. ~2.3s the firmware's own
        # ROTATE_DEG_PER_SEC=100 estimate predicts), and every "hang" this
        # session that required a hardware power-cycle turned out, on
        # closer look, to have timed out at almost EXACTLY its own
        # (too-short) timeout_sec value -- meaning the robot was very
        # likely still legitimately rotating, not actually stuck, and
        # sending STOP mid-rotation is what corrupted the UART/ack state
        # and caused the subsequent total silence, not a firmware bug.
        # 0.15s/deg (vs. the ~0.057s/deg the one 179deg sample implies)
        # bakes in a large safety factor since we only have ONE large-angle
        # timing sample so far; 5.0s floor covers whatever fixed overhead
        # (UART round-trip, alvik.rotate()'s own internal delay(200), ack
        # polling) makes even SMALL commanded angles take several seconds.
        # Re-tighten only after gathering more real timing data across a
        # range of angles -- do not shrink this blind.
        timeout_sec = max(5.0, abs(rel_deg) * 0.15 + 3.0)
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.rotate_rel_done_seen:
                break
            if self.rotate_rel_unconfirmed_seen:
                self.get_logger().warning(
                    f"{leg_label}: firmware braked without onboard target "
                    "confirmation; measuring the settled camera result")
                break
            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting")
                return None
        else:
            self.get_logger().error(
                f"{leg_label}: no ROTATE_REL COMPLETE within "
                f"{timeout_sec:.1f}s -- aborting")
            return None

        ack_elapsed = time.monotonic() - t0
        ack_yaw_pose = self.fresh_pose()
        ack_yaw = ack_yaw_pose.yaw if ack_yaw_pose is not None else None

        # Same rationale as turn_test()'s settle window: sample vision
        # through a short window after the ack instead of a blind sleep,
        # so a still-settling robot (or a stale/catching-up camera frame)
        # doesn't get misreported as already at rest.
        settle_t0 = time.monotonic()
        last_seen_yaw = ack_yaw
        while rclpy.ok() and time.monotonic() - settle_t0 < 0.5:
            self._spin_once(0.05)
            p = self.fresh_pose()
            if p is not None and p.yaw != last_seen_yaw:
                if self.args.verbose:
                    self.get_logger().info(
                        f"  [settle] yaw={p.yaw:+6.1f} "
                        f"(+{time.monotonic() - settle_t0:.2f}s since ack)")
                last_seen_yaw = p.yaw
        final_pose = self.fresh_pose()
        if final_pose is None:
            self.get_logger().error(
                f"{leg_label}: lost vision after ack -- cannot report final heading")
            return None

        final_error = yaw_error_deg(target_heading_deg, final_pose.yaw)
        self.get_logger().info(
            f"{leg_label}: RESULT final_yaw={final_pose.yaw:+.1f} "
            f"target={target_heading_deg:.1f} commanded={rel_deg:+.2f}deg "
            f"final_error={final_error:+.2f}deg  ack_after={ack_elapsed:.2f}s")

        return {
            "leg_label": leg_label,
            "target_heading": target_heading_deg,
            "start_yaw": start_yaw,
            "commanded_rel_deg": rel_deg,
            "final_yaw": final_pose.yaw,
            "final_error": final_error,
            "ack_elapsed": ack_elapsed,
        }

    def rotate_test_route(self, headings: list[float]) -> None:
        """Chain rotate_test() across a list of target headings -- mirrors
        turn_test_route(), but no re-arm step needed between turns since
        ROTATE_REL is a standalone command (not a WHEEL_FOLLOW_MODE
        streaming session), just a brief pause for full mechanical rest."""
        results: list[dict] = []
        for i, target_heading in enumerate(headings):
            leg_label = f"rotate-test #{i + 1}->{target_heading:.0f}deg"
            result = self.rotate_test(target_heading, leg_label)
            if result is None:
                self.get_logger().error(
                    f"{leg_label}: aborting remainder of rotate-test route")
                break
            results.append(result)
            if i < len(headings) - 1:
                self.spin_for(0.3)

        if not results:
            return
        self.get_logger().info(
            f"rotate-test route summary ({len(results)}/{len(headings)} turns):")
        for r in results:
            self.get_logger().info(
                f"  {r['leg_label']:<26s} cmd={r['commanded_rel_deg']:+7.2f}deg  "
                f"final_err={r['final_error']:+6.2f}deg  "
                f"ack_after={r['ack_elapsed']:.2f}s")
        final_errs = [abs(r["final_error"]) for r in results]
        ack_times = [r["ack_elapsed"] for r in results]
        n = len(results)
        self.get_logger().info(
            f"  |final_error|: mean={sum(final_errs)/n:.2f}deg "
            f"min={min(final_errs):.2f}deg max={max(final_errs):.2f}deg")
        self.get_logger().info(
            f"  ack_elapsed: mean={sum(ack_times)/n:.2f}s "
            f"min={min(ack_times):.2f}s max={max(ack_times):.2f}s")

    # -- STOP-mid-ROTATE_REL interrupt test (added 2026-08-24) -------------
    def stop_interrupt_test(self, rel_deg: float, delay_frac: float,
                            leg_label: str) -> dict | None:
        """Deliberately sends STOP partway through an in-flight ROTATE_REL
        and measures how far the robot keeps moving afterward -- the exact,
        previously-untested interaction behind incident #6 in
        ROTATE_REL_HAZARD_2026-08-20.md ("Third root cause").

        Every prior use of stop_only()/stop_and_rearm()/STOP in this
        codebase fires BETWEEN commands, when the robot is already
        stationary -- never as an interrupt of a command already in
        flight. The corrective-rotation-then-resync abort fix (same file,
        turn_to_heading_rotate_rel()) is the first code path that can send
        STOP while alvik.rotate() may still be executing, and a real
        hardware incident showed the robot did NOT actually stop when that
        happened -- it kept driving in a wide arc.

        Traced into the actual Arduino_Alvik library source
        (Documents/Arduino/libraries/Arduino_Alvik/src/Arduino_Alvik.cpp,
        not vendored in this repo): rotate() sends one 'R' UART packet to a
        separate motor co-processor and returns immediately (non-blocking);
        brake() (what STOP calls) sends a DIFFERENT packet type ('V',
        velocity) and never touches the co-processor's rotate-in-progress
        state at the library level. Whether the co-processor's own
        (unavailable) firmware actually preempts an in-flight rotate on
        receiving a velocity command is unknown -- this test measures the
        real-world answer directly.

        delay_frac: fraction (0.0-1.0) of the ROTATE_REL's own predicted
        duration (same timeout formula as rotate_test()) to wait before
        sending STOP -- e.g. 0.5 sends STOP roughly halfway through the
        commanded rotation. Samples vision continuously through the delay,
        at the moment STOP is sent, and for a settle window afterward, so
        the result shows the robot's actual yaw trajectory across the
        whole interrupt, not just a single before/after snapshot.

        Returns a result dict (always -- there is no "abort" case here,
        an unexpected outcome IS the measurement) or None only on a
        harder failure (vision lost entirely)."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting")
            return None

        start_yaw = pose.yaw
        start_x, start_y = pose.x, pose.y
        self.get_logger().info(
            f"{leg_label}: starting at yaw={start_yaw:+.1f} pos=({start_x:.1f},"
            f"{start_y:.1f}) -- sending ROTATE_REL {rel_deg:+.2f}deg, STOP "
            f"planned at {delay_frac*100:.0f}% of predicted duration")

        # delay_frac is relative to the SAME estimate the firmware's own
        # rotate_rel_done_ms deadline uses -- ROTATE_DEG_PER_SEC in
        # AGV_Factory_camera_correction.ino, currently 100.0 (verify this
        # still matches the DEPLOYED firmware on the robot before trusting
        # stop_at_sec; it has been retuned before, see that constant's own
        # comment for the 85-vs-100 history). Only affects WHEN this test
        # sends STOP relative to the rotation, not the pass/fail measurement
        # itself (which is real vision data, not a prediction).
        ROTATE_DEG_PER_SEC_ASSUMED = 100.0
        predicted_sec = abs(rel_deg) / ROTATE_DEG_PER_SEC_ASSUMED
        stop_at_sec = max(0.05, predicted_sec * max(0.0, min(1.0, delay_frac)))

        trajectory: list[tuple[float, float, float, float]] = []  # (t, x, y, yaw)

        def sample(tag: str = "") -> None:
            p = self.fresh_pose()
            if p is not None:
                t = time.monotonic() - t0
                trajectory.append((t, p.x, p.y, p.yaw))
                if self.args.verbose or tag:
                    self.get_logger().info(
                        f"  [{tag or 'sample'}] t={t:+.3f}s x={p.x:6.1f} "
                        f"y={p.y:6.1f} yaw={p.yaw:+6.1f}")

        self.rotate_rel_done_seen = False
        self.rotate_rel_unconfirmed_seen = False
        self.errored = False
        self.error_text = ""
        t0 = time.monotonic()
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")
        sample("start")

        # Sample continuously until stop_at_sec, then send STOP.
        stop_sent_at = None
        while rclpy.ok() and time.monotonic() - t0 < stop_at_sec:
            self._spin_once(0.02)
            sample()
        stop_sent_at = time.monotonic() - t0
        self.send_cmd("STOP")
        sample("stop-sent")
        self.get_logger().info(
            f"{leg_label}: STOP sent at t={stop_sent_at:.3f}s "
            f"(predicted rotate duration {predicted_sec:.2f}s)")

        # Keep watching for a full settle window AFTER stop -- this is the
        # actual measurement: does yaw/position keep changing post-STOP?
        settle_sec = 2.0
        settle_t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - settle_t0 < settle_sec:
            self._spin_once(0.02)
            sample()
            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} "
                    "during/after the interrupt")

        final_pose = self.fresh_pose()
        if final_pose is None:
            self.get_logger().error(
                f"{leg_label}: lost vision during interrupt test -- "
                "trajectory data below is still valid up to the last sample")

        # Movement AFTER the STOP was sent -- the real answer to "did STOP
        # actually stop it". Compares every sample after stop_sent_at
        # against the pose AT the moment STOP was sent (not against
        # start_yaw/start_x/start_y, which would also count the intended
        # pre-STOP rotation).
        post_stop = [s for s in trajectory if s[0] >= stop_sent_at]
        if len(post_stop) >= 2:
            ref_t, ref_x, ref_y, ref_yaw = post_stop[0]
            max_pos_drift_in = max(
                math.hypot(x - ref_x, y - ref_y) for _t, x, y, _yaw in post_stop)
            max_yaw_drift_deg = max(
                abs(yaw_error_deg(yaw, ref_yaw)) for _t, _x, _y, yaw in post_stop)
            final_t, final_x, final_y, final_yaw = post_stop[-1]
        else:
            max_pos_drift_in = max_yaw_drift_deg = 0.0
            final_t, final_x, final_y, final_yaw = trajectory[-1] if trajectory else \
                (stop_sent_at, start_x, start_y, start_yaw)

        # Position noise floor is a fixed, small physical distance (vision
        # jitter, not a tunable); yaw uses the SAME --turn-tol-deg the rest
        # of the codebase already treats as "close enough to call settled"
        # (default 1.0deg), so this test's pass/fail line matches what a
        # real route would already accept as a completed turn.
        stopped_cleanly = (max_pos_drift_in < 0.3
                          and max_yaw_drift_deg < self.args.turn_tol_deg)
        self.get_logger().info(
            f"{leg_label}: RESULT stopped_cleanly={stopped_cleanly}  "
            f"post_stop_max_pos_drift={max_pos_drift_in:.2f}in  "
            f"post_stop_max_yaw_drift={max_yaw_drift_deg:.2f}deg  "
            f"samples={len(trajectory)} (post-stop={len(post_stop)})")
        if not stopped_cleanly:
            self.get_logger().error(
                f"{leg_label}: STOP DID NOT CLEANLY HALT THE ROBOT -- "
                f"drifted {max_pos_drift_in:.2f}in / {max_yaw_drift_deg:.2f}deg "
                "after STOP was sent")

        return {
            "leg_label": leg_label,
            "rel_deg": rel_deg,
            "delay_frac": delay_frac,
            "predicted_sec": predicted_sec,
            "stop_sent_at": stop_sent_at,
            "start_yaw": start_yaw,
            "final_yaw": final_yaw,
            "stopped_cleanly": stopped_cleanly,
            "post_stop_max_pos_drift_in": max_pos_drift_in,
            "post_stop_max_yaw_drift_deg": max_yaw_drift_deg,
            "trajectory": trajectory,
        }

    def stop_interrupt_test_route(self, angles: list[float],
                                   delay_fracs: list[float]) -> None:
        """Runs stop_interrupt_test() across every (angle, delay_frac)
        combination -- e.g. angles=[90,-90,177] x delay_fracs=[0.25,0.5,0.75]
        covers small/large angles at early/mid/late interrupt points, since
        incidents #1/#2 flagged large angles as a possible correlated
        factor. Resyncs and pauses briefly between each to start every
        trial from a known-clean state. Robot should be in an open area
        with room to move unexpectedly -- that is the entire point of this
        test."""
        results: list[dict] = []
        trial = 0
        total = len(angles) * len(delay_fracs)
        for angle in angles:
            for frac in delay_fracs:
                trial += 1
                leg_label = f"stop-test #{trial}/{total} {angle:+.0f}deg@{frac:.2f}"
                result = self.stop_interrupt_test(angle, frac, leg_label)
                if result is not None:
                    results.append(result)
                self.spin_for(0.5)
                self.resync_yaw_offset()

        if not results:
            self.get_logger().error("stop-interrupt-test: no results collected")
            return
        clean = [r for r in results if r["stopped_cleanly"]]
        unclean = [r for r in results if not r["stopped_cleanly"]]
        self.get_logger().info(
            f"stop-interrupt-test summary: {len(clean)}/{len(results)} "
            f"stopped cleanly, {len(unclean)} did NOT")
        for r in results:
            mark = "OK  " if r["stopped_cleanly"] else "FAIL"
            self.get_logger().info(
                f"  [{mark}] {r['leg_label']:<32s} "
                f"stop_at={r['stop_sent_at']:.2f}s/{r['predicted_sec']:.2f}s "
                f"pos_drift={r['post_stop_max_pos_drift_in']:.2f}in "
                f"yaw_drift={r['post_stop_max_yaw_drift_deg']:.2f}deg")
        if unclean:
            self.get_logger().error(
                f"{len(unclean)} trial(s) show STOP NOT reliably halting "
                "an in-flight ROTATE_REL -- see ROTATE_REL_HAZARD_2026-08-"
                "20.md 'Third root cause' before trusting any abort path "
                "that sends STOP mid-rotation")

    # -- yaw-source stress test (encoder / camera_assist / camera_only) ---
    # Added 2026-08-13, ROS2/rosbridge equivalent of TurnSpeedBenchAlvik6.ino's
    # standalone-Arduino stress test -- exercises the REAL production code
    # path (turn_to_heading_rotate_rel()'s ROTATE_REL send + this file's own
    # _on_odom_pose/_on_vision_pose callbacks, over the actual wireless
    # link) rather than a bare alvik.rotate() call on the bench. Same
    # angle set and there-then-back pairing as the Arduino sketch (see its
    # STRESS_ANGLES_DEG/STRESS_REPEATS comment for the cable-safety
    # rationale) so results are directly comparable across both tiers.
    YAW_STRESS_ANGLES_DEG = [0.9, 2.6, 3.0, 1.5, 90.0, 90.3, 179.0, 177.5]
    YAW_STRESS_REPEATS = 8  # 8 * 8 * 2 (there+back) = 128 total rotations

    def yaw_stress_rotation(self, mode: str, target_heading_deg: float,
                             leg_label: str, resync_after: bool = True) -> dict | None:
        """One rotation in one of four yaw-source modes:

        "encoder": size ONE ROTATE_REL from corrected_odom_yaw() (local,
        fast) and trust it -- exactly what turn_to_heading_rotate_rel()
        does today, exercised directly here so a route doesn't have to be
        running to test it.

        "camera_assist": same odom-sized ROTATE_REL as "encoder", but after
        it settles, sample vision and send ONE corrective ROTATE_REL sized
        from the vision error if still outside --turn-tol-deg. Tests
        whether a cheap vision safety-net (paid only when needed, not every
        turn) catches cases where odom alone lands outside tolerance --
        per explicit user direction 2026-08-13, chosen over resyncing
        before every rotation (which would pay vision latency unconditionally).

        "camera_only": size the ROTATE_REL directly from fresh_pose()
        (vision yaw), ignoring odom entirely. This exercises the SAME
        firmware ROTATE_REL path as the other two modes -- the only
        difference from the deleted turn_to_heading_ROTATE_REL_EXPERIMENTAL()
        is that this now runs through the fixed firmware handler (see
        turn_to_heading_rotate_rel()'s docstring for the hang history and
        fix), not a re-litigation of that old, already-broken version.

        "encoder_drift": SAME sizing as "encoder" (corrected_odom_yaw()),
        but intended to be called with resync_after=False so error can
        accumulate freely across many rotations -- see resync_after's own
        docstring below. Treated identically to "encoder" for sizing
        purposes; the mode string only exists so results/logging/CSVs can
        tell this phase apart from the resync-every-turn "encoder" tier.

        resync_after: if True (default), calls resync_yaw_offset() at the
        end of this rotation, same as every mode has always done -- this
        means even "encoder" mode has NEVER tested raw uncorrected drift
        until this parameter existed (2026-08-20), since every prior tier
        resynced after every single rotation regardless of mode. Pass
        False (intended for "encoder_drift", chained across many rotations
        with no tier-start reset either -- see yaw_stress_test_route()'s
        resync_between_rotations param) to see real accumulated onboard-
        encoder drift over a long run instead of a corrected snapshot.

        Returns a result dict (mirrors rotate_test()'s shape, plus
        "mode"/"corrected" fields) or None on abort (vision/odom
        unavailable, errored, or ROTATE_REL timeout)."""
        if mode == "camera_only":
            start_pose = self.fresh_pose()
            if start_pose is None:
                self.get_logger().error(f"{leg_label}: lost vision -- aborting")
                return None
            start_yaw = start_pose.yaw
        else:
            start_yaw = self.corrected_odom_yaw()
            if start_yaw is None:
                self.get_logger().error(
                    f"{leg_label}: no corrected onboard yaw yet -- aborting "
                    "(try again once resync_yaw_offset() has succeeded once)")
                return None

        rel_deg = yaw_error_deg(target_heading_deg, start_yaw)
        # elapsed_sec measures TIME TO COMPLETION -- the real cost the user
        # explicitly flagged as missing 2026-08-13: accuracy alone doesn't
        # tell you whether a method is worth using if it takes much longer.
        # Starts here (just before the first send) and stops the instant the
        # last relevant ack lands (including a camera_assist correction, if
        # any) -- deliberately EXCLUDES the 0.5s vision settle-sample window
        # below, since that's measurement overhead this test adds for
        # reporting, not part of the maneuver itself, and would inflate
        # every mode's number equally rather than reflect a real difference.
        maneuver_t0 = time.monotonic()
        ok = self._send_rotate_rel_and_wait(rel_deg, leg_label)
        if not ok:
            return None

        corrected = False
        if mode == "camera_assist":
            self._spin_once(0.0)
            check_pose = self.fresh_pose()
            if check_pose is not None:
                remaining = yaw_error_deg(target_heading_deg, check_pose.yaw)
                if abs(remaining) > self.args.turn_tol_deg:
                    self.get_logger().info(
                        f"{leg_label}: camera_assist correction, vision "
                        f"yaw={check_pose.yaw:+.1f} still {remaining:+.2f}deg "
                        "off -- sending corrective ROTATE_REL")
                    ok = self._send_rotate_rel_and_wait(
                        remaining, f"{leg_label} (correction)")
                    if not ok:
                        return None
                    corrected = True
        elapsed_sec = time.monotonic() - maneuver_t0

        # Settle-sample vision (same rationale as rotate_test()) then report
        # the REAL final heading, regardless of mode -- vision is always
        # the ground-truth measurement for reporting, even in "encoder"
        # mode where it played no role in CONTROLLING the turn.
        settle_t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - settle_t0 < 0.5:
            self._spin_once(0.05)
        final_pose = self.fresh_pose()
        if final_pose is None:
            self.get_logger().error(
                f"{leg_label}: lost vision after ack -- cannot report final heading")
            return None

        final_error = yaw_error_deg(target_heading_deg, final_pose.yaw)
        self.get_logger().info(
            f"{leg_label}: [{mode}] RESULT final_yaw={final_pose.yaw:+.1f} "
            f"target={target_heading_deg:.1f} final_error={final_error:+.2f}deg "
            f"elapsed={elapsed_sec:.2f}s"
            f"{' (corrected)' if corrected else ''}")

        # Resync odom<->vision now, stationary, same as
        # turn_to_heading_rotate_rel() does -- keeps "encoder"/"camera_
        # assist" mode's next rotation from drifting further, and is
        # harmless for "camera_only" (it doesn't depend on odom, but
        # nothing else will resync otherwise since this test never calls
        # turn_to_heading_rotate_rel()). Skipped when resync_after=False
        # (see this method's own docstring) -- that's the whole point of
        # "encoder_drift": let real onboard-encoder error accumulate
        # instead of correcting it away after every single rotation.
        if resync_after:
            self.resync_yaw_offset()
        else:
            self.get_logger().info(
                f"{leg_label}: resync_after=False -- yaw_offset left as-is, "
                "drift accumulates into the next rotation")

        return {
            "leg_label": leg_label,
            "mode": mode,
            "target_heading": target_heading_deg,
            "start_yaw": start_yaw,
            "commanded_rel_deg": rel_deg,
            "final_yaw": final_pose.yaw,
            "final_error": final_error,
            "corrected": corrected,
            "elapsed_sec": elapsed_sec,
        }

    def _send_rotate_rel_and_wait(self, rel_deg: float, leg_label: str) -> bool:
        """Shared send+wait core of rotate_test()/yaw_stress_rotation() --
        same proven timeout formula, same ack/errored handling. Extracted
        2026-08-13 so yaw_stress_rotation()'s optional correction pass can
        reuse it without duplicating the timeout logic."""
        self.rotate_rel_done_seen = False
        self.rotate_rel_unconfirmed_seen = False
        self.errored = False
        self.error_text = ""
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")
        timeout_sec = max(5.0, abs(rel_deg) * 0.15 + 3.0)
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.rotate_rel_done_seen:
                return True
            if self.rotate_rel_unconfirmed_seen:
                self.get_logger().warning(
                    f"{leg_label}: firmware braked without onboard target "
                    "confirmation; continuing measurement from vision")
                return True
            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting")
                return False
        self.get_logger().error(
            f"{leg_label}: no ROTATE_REL COMPLETE within {timeout_sec:.1f}s -- aborting")
        return False

    def yaw_stress_test_route(self, mode: str, reset_first: bool = True,
                               resync_after_each: bool = True,
                               on_result: Callable[[dict], None] | None = None
                               ) -> list[dict]:
        """Runs the full 128-rotation there-and-back stress sequence (see
        YAW_STRESS_ANGLES_DEG/YAW_STRESS_REPEATS) in the given mode
        ("encoder"/"camera_assist"/"camera_only"/"encoder_drift"), against
        a FIXED absolute target derived from the robot's own starting
        heading -- mirrors the Arduino bench sketch's cable-safety design:
        every angle is rotated there then immediately back, so net
        rotation returns to ~0 every 2 iterations regardless of mode or
        outcome.

        on_result (added 2026-08-20, per explicit user direction after a
        real hardware run lost mid-tier data): if given, called with each
        rotation's result dict IMMEDIATELY after it completes, not just
        collected into the returned list -- lets a caller (e.g.
        fleet_yaw_stress_test.py) flush every row to disk as it happens,
        so a crash/hang partway through a 128-rotation tier doesn't lose
        the rotations that already succeeded. Never called for aborted
        rotations (yaw_stress_rotation() returning None) -- only real
        results.

        reset_first (default True): calls reset_onboard_pose() +
        resync_yaw_offset() FIRST -- per explicit user direction
        2026-08-13: running encoder -> camera_assist -> camera_only back-
        to-back with no reset between tiers let ~256 rotations of
        accumulated onboard-encoder drift from the first two tiers carry
        into later readings, making tier order itself a hidden confound.
        Pass False (intended for "encoder_drift", run LAST after the other
        three tiers, per explicit user direction 2026-08-20) to
        DELIBERATELY inherit whatever onboard yaw state the previous tier
        left, as the starting point for observing real drift accumulation.

        resync_after_each (default True): passed through to every
        yaw_stress_rotation() call as resync_after -- see that parameter's
        own docstring. Pass False (again, "encoder_drift") so error
        accumulates freely across all 128 rotations instead of being
        corrected away after each one, which is what EVERY tier has done
        until this parameter existed, including "encoder" -- see
        yaw_stress_rotation()'s docstring for why that means "encoder" has
        never actually shown raw uncorrected drift.

        Returns the list of per-rotation result dicts (see
        yaw_stress_rotation()'s docstring for the shape) -- empty list on
        an early abort (reset/resync/vision failure) or if every rotation
        aborted. Added 2026-08-20 (was previously -> None, summary-only)
        so a multi-robot caller (fleet_yaw_stress_test.py) can write its
        own per-robot files from the real data instead of scraping this
        method's get_logger() output, which is process-wide and unusable
        once 6 robots' messages are interleaved on one terminal."""
        if reset_first:
            if not self.reset_onboard_pose("stress-test-start reset"):
                self.get_logger().error(
                    "could not reset onboard pose -- aborting stress test")
                return []
            self.spin_for(0.3)  # let the robot fully settle post-reset
            resync_t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - resync_t0 < 2.0:
                self._spin_once(0.1)
                if self.resync_yaw_offset():
                    break
        else:
            self.get_logger().info(
                f"[{mode}]: reset_first=False -- starting from whatever "
                "onboard yaw state the previous tier left (deliberate, "
                "for observing drift accumulation)")
        if self.yaw_offset is None and mode != "camera_only":
            self.get_logger().error(
                "no odom-vision offset available -- aborting stress test "
                f"(mode={mode!r} requires odom; camera_only does not)")
            return []

        start_pose = self.fresh_pose()
        if start_pose is None:
            self.get_logger().error("no vision at stress-test start -- aborting")
            return []
        base_heading = start_pose.yaw

        total = self.YAW_STRESS_REPEATS * len(self.YAW_STRESS_ANGLES_DEG) * 2
        self.get_logger().info(
            f"### yaw-source stress test [{mode}]: {total} rotations "
            f"({self.YAW_STRESS_REPEATS} repeats x {len(self.YAW_STRESS_ANGLES_DEG)} "
            "angles x there+back) ###")

        results: list[dict] = []
        abort_count = 0
        iter_num = 0
        for rep in range(self.YAW_STRESS_REPEATS):
            first_sign = 1.0 if rep % 2 == 0 else -1.0
            for mag in self.YAW_STRESS_ANGLES_DEG:
                for sign in (first_sign, -first_sign):
                    iter_num += 1
                    target = normalize_deg(base_heading + sign * mag)
                    leg_label = f"[stress {iter_num}/{total}] angle={sign * mag:+.1f}"
                    result = self.yaw_stress_rotation(
                        mode, target, leg_label, resync_after=resync_after_each)
                    if result is None:
                        abort_count += 1
                        self.get_logger().error(
                            f"{leg_label}: aborted -- continuing with next rotation "
                            "(unlike rotate_test_route(), one bad rotation "
                            "shouldn't stop a 128-rotation stress run)")
                        self.spin_for(0.3)
                        continue
                    results.append(result)
                    if on_result is not None:
                        on_result(result)
                    self.spin_for(0.3)

        self.get_logger().info(
            f"### yaw-source stress test [{mode}] complete: "
            f"{len(results)}/{total} succeeded, {abort_count} aborted ###")
        if not results:
            return results
        final_errs = [abs(r["final_error"]) for r in results]
        elapsed_secs = [r["elapsed_sec"] for r in results]
        n = len(results)
        corrected_count = sum(1 for r in results if r["corrected"])
        self.get_logger().info(
            f"  |final_error|: mean={sum(final_errs)/n:.2f}deg "
            f"min={min(final_errs):.2f}deg max={max(final_errs):.2f}deg")
        self.get_logger().info(
            f"  elapsed_sec:   mean={sum(elapsed_secs)/n:.2f}s "
            f"min={min(elapsed_secs):.2f}s max={max(elapsed_secs):.2f}s "
            f"total={sum(elapsed_secs):.1f}s -- THE COST NUMBER: compare "
            "this against the other two tiers' mean before trusting an "
            "accuracy win alone (a slightly-more-accurate mode that takes "
            "much longer per rotation isn't necessarily worth it)")
        if mode == "camera_assist":
            self.get_logger().info(
                f"  corrective ROTATE_REL sent on {corrected_count}/{n} rotations")
        return results

    def dwell(self, dwell_ms: int | None, label: str) -> bool:
        """Send DWELL (or "DWELL <ms>"), wait for the firmware's own
        DWELL COMPLETE and leave the firmware IDLE. The next drive/streamed
        turn arms WHEEL_FOLLOW_MODE at its own start; arming it here would
        expose the 300 ms watchdog during a legitimate supervisor wait.
        Timeout is the requested dwell (or the firmware's default) plus a
        generous margin -- the firmware enforces the actual wait."""
        cmd = "DWELL" if dwell_ms is None else f"DWELL {dwell_ms}"
        wait_sec = (dwell_ms / 1000.0) if dwell_ms is not None else 2.0
        timeout_sec = wait_sec + 3.0
        self.dwell_done_seen = False
        self.send_cmd(cmd)
        self.get_logger().info(f"{label}: {cmd}")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.dwell_done_seen:
                break
            if self.errored:
                self.get_logger().error(
                    f"{label}: robot reported {self.error_text} during DWELL -- aborting route")
                return False
        else:
            self.get_logger().error(f"{label}: no DWELL COMPLETE within {timeout_sec:.1f}s -- aborting route")
            return False
        self.errored = False
        self.error_text = ""
        return True

    # -- one leg: drive from current position to (target_x, target_y) -----
    def drive_leg(self, target_x: float, target_y: float, leg_label: str) -> bool:
        """Returns False on error/abort (caller should stop the route).

        Steers toward the LIVE target every tick (distance/heading
        recomputed fresh from the current camera pose each iteration) rather
        than tracking a line fixed at leg start -- ported 2026-07-27 from a
        bench-verified onboard-odometry driveTo() (0.1in tolerance, high
        repeatability).

        REWRITTEN 2026-07-28: constant --cruise-rpm the whole leg (a hard
        brake via stop_and_rearm() at --brake-lead-in from target) replaces
        the old kp_dist proportional-slowdown law. User's explicit direction:
        "Instead of slowing down, we just need the alvik to brake" -- a
        gradually-decelerating approach doesn't reproduce the old color-
        sensor firmware's demonstrated 60-70RPM constant-speed-then-instant-
        hard-stop behavior, and repeated hardware tuning of drive_min_speed/
        kp_dist never converged on a reliably sharp stop.

        --brake-lead-in's default (1.9, RE-MEASURED 2026-07-28 later the
        same day) is taken from a 7-leg --stop-test route run at
        --cruise-rpm=60 with this SAME heading-correction law, AFTER a
        vision-pipeline fix (TcpFrameSource.read() in apriltag_localize.py,
        draining stale backlogged frames instead of serving them oldest-
        first) changed real measured stopping distance: mean slide-after-
        brake 1.61in (range 1.55-1.67in), mean along-path error only 0.23in
        short of target (range 0.11-0.30in), never overshooting -- see
        stop_test()/stop_test_route() for the measurement tool. An earlier
        3.7in value, measured before that vision fix, was landing every leg
        ~2in short once the fix was in (stale calibration, not a new
        problem) -- see --brake-lead-in's own CLI help text for the full
        before/after numbers. If --cruise-rpm changes, or the vision
        pipeline changes again, re-measure via --stop-test before trusting
        this default (stopping distance is not necessarily linear in RPM,
        and is sensitive to real pose latency)."""
        pose = self.control_pose()
        if pose is None:
            self.get_logger().error(
                f"{leg_label}: lost {self.localization_source_label()} -- "
                "aborting route")
            return False

        self.get_logger().info(
            f"{leg_label}: driving from ({pose.x:.1f},{pose.y:.1f}) to "
            f"({target_x:.1f},{target_y:.1f}), cruise={self.args.cruise_rpm:.0f}RPM, "
            f"brake_lead={self.args.brake_lead_in:.2f}in")

        # Re-arm WHEEL_FOLLOW_MODE HERE (2026-08-20, moved from the END of
        # the PREVIOUS drive_leg() call -- see stop_only()'s docstring for
        # the full root-cause/collision history this fixes). This is now
        # the ONE place that arms it before actually streaming wheel
        # setpoints below, so a chained drive-drive-drive route (no
        # rotation between legs) still re-arms every leg exactly as before,
        # while a drive-then-turn transition leaves the robot genuinely
        # IDLE (via stop_only()) for as long as it takes to decide whether
        # a ROTATE_REL follows, instead of sitting armed in
        # STATE_WHEEL_FOLLOW while that decision is made.
        if not self.enter_wheel_follow_mode():
            self.get_logger().error(
                f"{leg_label}: did not ack WHEEL_FOLLOW_MODE -- aborting route")
            return False

        # Stall backstop -- same rationale as stop_test()'s: a watchdog trip
        # (ERROR WHEEL_CMD_TIMEOUT -> IDLE) mid-leg can land in the gap
        # between this loop's non-blocking rclpy.spin_once(timeout_sec=0.0)
        # calls, leaving the robot stopped while this loop keeps waiting for
        # dist_to_target to shrink, which it never will without a bound.
        no_progress_timeout_sec = 5.0
        best_dist = math.inf
        t_progress0 = time.monotonic()

        period = 1.0 / CONTROL_HZ
        while rclpy.ok():
            loop_t0 = time.monotonic()
            self._spin_once(0.0)

            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting route")
                return False

            pose = self.control_pose()
            if pose is None:
                # Confirmed 2026-07-27: a bare send_wheel(0.0, 0.0) here did
                # NOT stop the robot -- it drove ~77in past its target and
                # nearly off the table edge, despite this branch firing
                # (the "vision stale" warning did print). Root cause
                # unconfirmed (STATE_WHEEL_FOLLOW's 0 RPM setpoint vs.
                # alvik.brake() on this hardware/library), but a same-mode
                # zero-speed setpoint is not a safe stop primitive here.
                # STOP unconditionally calls alvik.brake() and exits
                # STATE_WHEEL_FOLLOW entirely -- abort the route rather than
                # hold-and-hope vision returns. Verified/re-sent (2026-08-25,
                # see stop_and_verify()): with vision lost this re-sends
                # STOP blind for up to 1.5s (strictly stronger than one
                # send), and verifies stationarity if vision returns.
                self.stop_and_verify(leg_label, timeout_sec=1.5)
                self.get_logger().error(
                    f"{leg_label}: {self.localization_source_label()} lost -- "
                    "STOP sent, aborting route")
                return False

            dx = target_x - pose.x
            dy = target_y - pose.y
            dist_to_target = math.hypot(dx, dy)

            if dist_to_target < best_dist - 0.05:
                best_dist = dist_to_target
                t_progress0 = loop_t0
            elif loop_t0 - t_progress0 > no_progress_timeout_sec:
                self.get_logger().error(
                    f"{leg_label}: no progress for {no_progress_timeout_sec:.0f}s "
                    f"(stuck at {dist_to_target:.2f}in from target, last "
                    f"status '{self.last_status}') -- aborting route")
                # CHANGED 2026-08-25 from stop_and_rearm(): re-arming
                # WHEEL_FOLLOW_MODE on an ABORT path was always wrong --
                # nothing feeds the watchdog afterward (guaranteed
                # WHEEL_CMD_TIMEOUT ~300ms later, seen in the 2026-08-24
                # 4-robot run on Alvik2's exact stall), and it cleared
                # self.errored, masking whatever real error caused the
                # stall. A verified STOP is what an abort actually needs.
                self.stop_and_verify(leg_label)
                return False

            if dist_to_target <= self.args.brake_lead_in:
                # Log BEFORE stop_only() -- confirmed 2026-07-28: logging
                # after made every prior "arrived" timestamp actually mark the
                # END of the stop round trip, not the moment arrival was
                # detected. That misattributed two real ~3s stop_and_rearm()
                # stalls (repeatedly, at the same node10 transition) to a
                # "vision gap" that was never there -- occlusion and vision
                # throughput were both ruled out chasing the wrong timestamp
                # before this was caught.
                #
                # CHANGED 2026-08-20 from stop_and_rearm() to stop_only() --
                # this is THE call site of two real hardware collisions
                # (Alvik4 off-table, Alvik1 into the corner tag): every
                # arrival here is immediately followed, in run()'s loop, by
                # turn_to_heading_rotate_rel()'s ROTATE_REL, whose firmware
                # handler assumes the robot is already stationary/IDLE --
                # not armed in STATE_WHEEL_FOLLOW, which stop_and_rearm()
                # left it in. See stop_only()'s own docstring for the full
                # root-cause writeup. WHEEL_FOLLOW_MODE is now re-armed at
                # the START of the NEXT drive_leg() call instead (see the
                # top of this function), not here.
                self.get_logger().info(
                    f"{leg_label}: brake fired ({dist_to_target:.2f}in short of "
                    "target)")
                self.stop_only()
                return True

            target_heading = heading_to_target_deg(dx, dy)
            heading_err = yaw_error_deg(target_heading, pose.yaw)

            # Always drive nose-forward -- never reverse. Confirmed
            # 2026-07-27: the reference driveTo() this was ported from
            # reverses when the target is >90deg behind (free-roaming robot,
            # no fixed path), but this robot must stay on the taped grid
            # line between adjacent nodes at all times (same constraint that
            # bans a mid-grid ROTATE_180 in the color-sensor solver -- see
            # agv_grid_workstation_solver.html). On a real route the robot
            # starts each leg already facing roughly the right way (the
            # previous leg's own square-up turn), so heading_err should
            # rarely be large in practice; if it ever is, turn hard toward
            # the target rather than backing away from it off the tape.
            turn_adjust = max(-self.args.max_turn_adjust,
                               min(self.args.max_turn_adjust,
                                   self.args.kp_yaw * heading_err))
            left = self.args.cruise_rpm - turn_adjust
            right = self.args.cruise_rpm + turn_adjust
            self.send_wheel(left, right)

            if self.args.verbose:
                self.get_logger().info(
                    f"  x={pose.x:6.1f} y={pose.y:6.1f} yaw={pose.yaw:+6.1f}  "
                    f"hdg_target={target_heading:6.1f}  hdg_err={heading_err:+5.1f}deg  "
                    f"dist_to_target={dist_to_target:5.2f}in  "
                    f"L={left:+5.1f} R={right:+5.1f}")

            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, period - elapsed))
        return False

    # -- one turn-in-place: pivot to a new absolute heading ---------------
    def turn_to_heading_rotate_rel(self, target_heading_deg: float, leg_label: str) -> bool:
        """Added 2026-08-13, per explicit user direction after a bench
        comparison (TurnSpeedBenchAlvik6.ino, Alvik6) showed alvik.rotate()
        settling in ~1.0-1.7s at ~1-3deg accuracy, vs. turn_to_heading()'s
        wheel-streaming taper at 3.4-5.7s / ~2.8-2.9deg for the same 90deg
        turn -- a 2-4x speed win with comparable or better accuracy.

        Sizes its ROTATE_REL from control_pose(), so the fast maneuver honors
        --localization-mode: encoder and camera_assist use transformed onboard
        odometry, while camera uses the latest AprilTag yaw. The firmware does
        not publish ROTATE_REL COMPLETE until its onboard IMU has remained at
        the requested target and alvik.brake() has been issued. If that
        onboard check reaches its deadline, the firmware brakes and emits a
        provisional result which camera-capable modes must verify from fresh,
        settled vision; it is never accepted by encoder-only mode.

        HANG HISTORY -- read before ever changing this method's completion
        check: an EARLIER ROTATE_REL implementation (removed 2026-08-13,
        see git history if the old docstring is needed) hit multiple
        confirmed FULL FIRMWARE HANGS on 2026-07-30 (LED frozen, zero ROS
        traffic, power-cycle required). Root-caused at the FIRMWARE level
        (AGV_Factory_camera_correction.ino's ROTATE_REL handler comment) to
        a parse_message() ack-discard race, made likely by an alvik.brake()
        call immediately before alvik.rotate(). Fixed by removing that
        pre-rotate brake and abandoning is_target_reached() polling. The
        current firmware instead uses its continuously refreshed IMU to
        require a stable target, applies alvik.brake(), and only then emits
        ROTATE_REL COMPLETE. At its verification deadline it brakes and emits
        BRAKED_UNCONFIRMED, not success; camera modes must independently
        verify that result while encoder-only mode aborts.
        Independently re-stress-tested 2026-08-13 on Alvik6 (bench sketch,
        128 back-to-back rotate() calls incl. small angles specifically --
        the exact condition that hung before): 128/128 settled cleanly,
        zero hangs, zero is_on()-detected STM32 unresponsiveness. Do not
        reintroduce alvik.brake() before alvik.rotate() (in firmware) or
        is_target_reached()-style polling (here) without re-reading the
        firmware's ROTATE_REL handler comment first.

        If the selected localization source is unavailable, stop and abort
        rather than silently changing to the slow streamed turn method.

        CAMERA-ASSIST CORRECTION (added 2026-08-13, made the default after
        --yaw-stress-test results on Alvik6, 128 rotations/tier, all 3
        tiers self-resetting so comparable): after the odom-sized
        ROTATE_REL settles, sample vision once and send ONE corrective
        ROTATE_REL if still outside --turn-tol-deg. Real hardware numbers
        that motivated this: encoder-only mean |final_error| 2.46deg
        (max 7.10deg) at 1.30s mean/turn; camera_assist mean 0.91deg
        (max 3.10deg) at 2.01s mean/turn -- ~2.7x more accurate for +0.7s/
        turn. camera_only (vision sizes EVERY ROTATE_REL, odom unused) was
        statistically indistinguishable from encoder-only on both accuracy
        (2.33deg) and speed (1.28s), so it bought nothing over odom alone
        and was not adopted -- see yaw_stress_rotation()'s docstring for
        all three modes if this needs re-litigating."""
        start_pose = self.control_pose()
        if start_pose is None:
            self.get_logger().error(
                f"{leg_label}: no fresh {self.localization_source_label()} "
                "yaw -- STOP sent; refusing slow wheel-streaming fallback")
            self.stop_only()
            return False

        start_yaw = start_pose.yaw
        rel_deg = yaw_error_deg(target_heading_deg, start_yaw)
        if abs(rel_deg) <= self.args.turn_tol_deg:
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_tol_deg:.1f} "
                f"deg of target heading {target_heading_deg:.0f} "
                f"({self.localization_source_label()} yaw={start_yaw:+.1f}), "
                "skipping turn")
            return True

        self.get_logger().info(
            f"{leg_label}: turning from {self.localization_source_label()} "
            f"yaw {start_yaw:+.1f} to "
            f"{target_heading_deg:.0f} deg via ROTATE_REL {rel_deg:+.2f}deg")
        if not self._rotate_rel_and_wait_or_abort(
                rel_deg, leg_label, target_heading_deg=target_heading_deg):
            return False
        primary_unconfirmed = self.rotate_rel_unconfirmed_seen
        camera_confirmed = self.rotate_camera_confirmed_pose

        # Encoder-only experiments remain camera-independent. Firmware's
        # COMPLETE now comes only after the onboard IMU target is stable and
        # alvik.brake() has been sent, so there is no slow streamed cleanup.
        if self.localization_mode == "encoder":
            if primary_unconfirmed:
                self.get_logger().error(
                    f"{leg_label}: firmware braked but could not confirm the "
                    "onboard yaw target; encoder-only mode cannot use camera "
                    "verification -- aborting route")
                return False
            self.get_logger().info(
                f"{leg_label}: fast encoder-sized rotation complete; "
                "firmware target-gated brake confirmed")
            return True

        # Camera-capable modes still settle before measuring. Firmware now
        # brakes before COMPLETE using its onboard IMU target, while this
        # independent vision check prevents a stale/moving frame from sizing
        # the optional correction.
        if camera_confirmed is not None:
            # The early-exit already required vision within turn_tol_deg of
            # target AND stable over its window, so the robot has been
            # measured at rest at the target. Re-settling here would just
            # re-measure that same standstill and pay for it twice.
            check_pose, settle_state = camera_confirmed, "settled"
        else:
            check_pose, settle_state = self.wait_for_yaw_settled(
                f"{leg_label} (post-turn settle)")
        if settle_state == "still_moving":
            # Fresh vision kept arriving but yaw never stabilized: the
            # robot is genuinely still rotating long after the firmware's
            # terminal status.
            # Actively stop it (verified, re-sent) and abort the leg --
            # measuring or correcting a moving robot is how prior
            # incidents cascaded.
            self.get_logger().error(
                f"{leg_label}: yaw never settled after ROTATE_REL terminal "
                "status "
                "(see the measurements on the line above for which fault "
                "this is) -- sending verified STOP and aborting leg")
            self.stop_and_verify(leg_label)
            return False
        if settle_state == "vision_stale":
            # ROOT-CAUSED 2026-08-29 (Alvik4 off the table at DE4, run4_1316):
            # this branch did not exist, and the two aborts around it both
            # missed the case. settle_state was "vision_stale" so the
            # still_moving abort above did not fire; the firmware had sent an
            # IMU-confirmed COMPLETE so primary_unconfirmed was False and the
            # abort below did not fire either; check_pose was None so the
            # camera-verification block was skipped entirely. The turn then
            # fell through to the resync and returned True. The log shows the
            # result one line apart:
            #     ERROR ... NOT settled -- no fresh vision at all in 3.0s,
            #           so whether the robot is moving is UNKNOWN
            #     INFO  ... turn complete but yaw resync skipped
            #
            # A turn accepted with ZERO vision verification is exactly what
            # this whole settle-then-verify path exists to prevent. In a
            # camera-capable mode vision is the authority and the firmware's
            # terminal status is only a provisional gate in front of it, so
            # "the firmware says it turned" is not evidence of anything on its
            # own -- the 2026-08-19 Alvik4/DE4 incident is precisely a turn the
            # firmware reported while the robot drove forward instead.
            #
            # Aborting here would not have saved THIS run (the robot was
            # already gone, and the next leg failed 0.1s later on odometry
            # anyway). It matters for the case where vision goes dark but
            # odometry does not: today that returns True and drive_leg()
            # then drives a robot whose real heading nobody has checked.
            #
            # encoder mode never reaches here -- it returns earlier, above.
            self.get_logger().error(
                f"{leg_label}: no fresh vision at all during the post-turn "
                "settle, so this turn is UNVERIFIED -- the firmware's own "
                "terminal status is not sufficient in a camera-capable mode "
                "(see the measurements above) -- sending verified STOP and "
                "aborting leg")
            self.stop_and_verify(leg_label)
            return False
        if primary_unconfirmed and check_pose is None:
            self.get_logger().error(
                f"{leg_label}: firmware target was unconfirmed and no fresh, "
                "settled camera yaw is available to verify the braked turn "
                "-- aborting route")
            return False
        corrected = False
        if check_pose is not None:
            remaining = yaw_error_deg(target_heading_deg, check_pose.yaw)
            if abs(remaining) > self.args.turn_tol_deg:
                self.get_logger().info(
                    f"{leg_label}: camera correction, vision "
                    f"yaw={check_pose.yaw:+.1f} still {remaining:+.2f}deg "
                    "off -- sending corrective ROTATE_REL")
                if not self._rotate_rel_and_wait_or_abort(
                        remaining, f"{leg_label} (correction)",
                        target_heading_deg=target_heading_deg):
                    return False
                corrected = True
            elif primary_unconfirmed:
                self.get_logger().info(
                    f"{leg_label}: camera verified braked turn at "
                    f"yaw={check_pose.yaw:+.1f}; error {remaining:+.2f}deg is "
                    f"within +/-{self.args.turn_tol_deg:.1f}deg")

        # ROOT-CAUSED 2026-08-24 (Alvik1, single-robot run, real collision
        # with the corner reference tag): after the corrective ROTATE_REL
        # above, the old code did self._spin_once(0.0) -- ONE non-blocking
        # callback pump -- then called resync_yaw_offset() immediately,
        # trusting whatever self.last_pose already held. That is very
        # frequently a vision FRAME FROM BEFORE THE CORRECTION MOVED THE
        # ROBOT (the camera pipeline is 30-60Hz -- a single spin_once(0.0)
        # has no obligation to have received a new frame since the
        # correction was sent. In the affected firmware, ROTATE_REL completion
        # was only a timed deadline; current firmware is IMU-target-gated, but
        # a newer vision frame is still required before resync. resync_yaw_
        # offset() has no way to tell a stale-but-technically-fresh-enough
        # sample from a genuinely post-correction one, so it would anchor
        # yaw_offset to the PRE-correction heading as if it were ground
        # truth. Confirmed on hardware: three consecutive turns in one run
        # showed the post-correction vision error getting WORSE each time
        # (-3.7 -> +19.1 -> -90.7deg) while yaw_offset drifted further off
        # (+2.80 -> +23.20 -> +56.30) -- each resync was corrupting the
        # NEXT turn's odom baseline with an increasingly wrong reference,
        # until the robot's actual heading was nowhere near the commanded
        # angle and it drove off-track into the corner tag. This was the
        # exact gap flagged (but deliberately not fixed) in
        # ROTATE_REL_HAZARD_2026-08-20.md item 3.
        #
        # Fixed by actually waiting for a SETTLED vision reading newer
        # than the pre-correction sample (2026-08-25: upgraded from
        # "any newer sample" to "newer AND yaw-stable" per the
        # --stop-interrupt-test data -- a merely-newer frame can still
        # capture a mid-rotation robot, see wait_for_yaw_settled()), and
        # -- if a correction was sent but still isn't within tolerance
        # once the robot has demonstrably come to rest -- ABORTING the
        # leg instead of resyncing to a confirmed-bad heading. A silent
        # resync to a wrong reference would corrupt every subsequent turn
        # in the route; an abort here at least stops before that cascade
        # starts, and is a leg the advisor already knows how to
        # exclude/report cleanly.
        if corrected:
            settled_pose, post_state = self.wait_for_yaw_settled(
                f"{leg_label} (post-correction settle)",
                min_newer_than=check_pose.t if check_pose is not None else None)
            if post_state == "still_moving":
                self.get_logger().error(
                    f"{leg_label}: robot still rotating after the "
                    "corrective ROTATE_REL -- sending verified STOP and "
                    "aborting leg")
                self.stop_and_verify(leg_label)
                return False
            if settled_pose is None:
                self.get_logger().error(
                    f"{leg_label}: corrective ROTATE_REL sent but no fresh "
                    "post-correction vision arrived -- aborting rather "
                    "than resync to a stale/unknown heading")
                return False
            final_remaining = yaw_error_deg(
                target_heading_deg, settled_pose.yaw)
            if abs(final_remaining) > self.args.turn_tol_deg:
                self.get_logger().error(
                    f"{leg_label}: still {final_remaining:+.2f}deg off "
                    f"target after the corrective ROTATE_REL (vision "
                    f"yaw={settled_pose.yaw:+.1f}, settled) -- aborting "
                    "rather than resync to a confirmed-wrong heading")
                return False

        # Resync onboard yaw to vision now, while the robot is stationary
        # (right after a completed turn is exactly the safe window
        # resync_yaw_offset() calls for) -- keeps the NEXT turn's odom
        # reading from drifting further from ground truth. Not fatal if it
        # fails (e.g. vision briefly stale) -- the turn itself already
        # completed; this only affects the next one, which will retry the
        # resync when it starts.
        if self.localization_mode == "camera":
            self.get_logger().info(
                f"{leg_label}: camera-only fast rotation complete")
        elif not self.resync_yaw_offset():
            self.get_logger().info(
                f"{leg_label}: turn complete but yaw resync skipped "
                "(vision/odom not both fresh) -- next turn will retry")
        else:
            self.get_logger().info(
                f"{leg_label}: turn complete, yaw resynced "
                f"(offset={self.yaw_offset:+.2f})")
        return True

    def _rotate_rel_and_wait_or_abort(self, rel_deg: float, leg_label: str,
                                      target_heading_deg: float | None = None
                                      ) -> bool:
        """turn_to_heading_rotate_rel()'s send+wait core, with THIS
        method's route-abort logging (distinct from _send_rotate_rel_and_
        wait()'s measurement-mode logging, which callers like
        yaw_stress_rotation() rely on saying "aborting" not "aborting
        route"). Same proven timeout formula as rotate_test()/
        _send_rotate_rel_and_wait() -- do not shrink without new timing
        data across a range of angles.

        CAMERA EARLY-EXIT (2026-08-26): pass target_heading_deg in a
        camera-capable mode and this returns as soon as VISION shows the
        robot at the target heading and stationary, instead of waiting out
        the firmware status. Measured from run_1538.log (12 workstations,
        clean): all 72 rotations -- every one, corrections included -- fell
        through to the firmware hard deadline and reported
        BRAKED_UNCONFIRMED; the IMU-gated early COMPLETE never fired once.
        That deadline is 10*|deg| + ROTATE_REL_TARGET_TIMEOUT_MS, i.e.
        ~3.6s for a 90deg turn, of which the actual rotation is ~1.0-1.7s.
        The camera check that followed it took only 221ms. So ~2.4s per
        turn -- ~180s of a 503s run, 36% -- was the robot sitting still
        waiting for a timer to expire on a rotation it had already
        finished.

        Why vision rather than the firmware status: in the camera-assisted
        mode this fleet runs, vision is already the authority that verifies
        and corrects every turn; the firmware terminal status is only a
        provisional gate in front of it. Ending the turn on the same
        measurement that would have validated it a moment later removes the
        wait without removing a check.

        Stability is still required, and that is NOT the same as "one fresh
        frame". A single sample cannot distinguish a robot AT the target
        from one sweeping THROUGH it at speed -- exactly the confusion that
        drove a robot off the table on 2026-08-25. The window is 0.12s
        (matching the firmware ROTATE_REL_TARGET_STABLE_MS), which rejects
        anything still turning faster than ~8deg/s while costing ~7 frames
        at 60Hz.

        The min-wait guard exists because alvik.rotate() has ALREADY been
        sent when this loop starts: without it, a rotation whose target
        happens to be where the robot already is would be "confirmed"
        before the co-processor had begun to move (its takeover latency is
        ~150-250ms), and we would drive away while it turned."""
        self.rotate_rel_done_seen = False
        self.rotate_rel_unconfirmed_seen = False
        self.rotate_camera_confirmed_pose = None
        self.errored = False
        self.error_text = ""
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")
        timeout_sec = max(5.0, abs(rel_deg) * 0.15 + 3.0)

        cam_ok = (target_heading_deg is not None
                  and mode_requires_camera(self.localization_mode))
        CAM_STABLE_SEC = 0.12
        CAM_STABLE_TOL_DEG = 1.0
        cam_min_wait_sec = 0.25 + 0.010 * abs(rel_deg)
        cam_window: list[tuple[float, float]] = []  # (pose.t, yaw)

        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.02 if cam_ok else 0.1)

            if cam_ok and time.monotonic() - t0 >= cam_min_wait_sec:
                p = self.fresh_pose()
                if p is not None and not (cam_window
                                          and p.t == cam_window[-1][0]):
                    cam_window.append((p.t, p.yaw))
                    cutoff = p.t - CAM_STABLE_SEC
                    cam_window = [(t, y) for (t, y) in cam_window
                                  if t >= cutoff]
                    err = yaw_error_deg(target_heading_deg, p.yaw)
                    if (abs(err) <= self.args.turn_tol_deg
                            and len(cam_window) >= 3
                            and cam_window[-1][0] - cam_window[0][0]
                                >= 0.8 * CAM_STABLE_SEC):
                        deltas = [normalize_deg(y - cam_window[0][1])
                                  for _t, y in cam_window]
                        if max(deltas) - min(deltas) <= CAM_STABLE_TOL_DEG:
                            self.rotate_camera_confirmed_pose = p
                            self.get_logger().info(
                                f"{leg_label}: camera confirmed target early "
                                f"at yaw={p.yaw:+.1f} ({err:+.2f}deg off, "
                                f"settled) after {time.monotonic() - t0:.2f}s "
                                "-- not waiting for the firmware deadline")
                            return True

            if self.rotate_rel_done_seen:
                return True
            if self.rotate_rel_unconfirmed_seen:
                self.get_logger().warning(
                    f"{leg_label}: firmware hard deadline reached after it "
                    "braked; requiring settled camera verification")
                return True
            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} "
                    "during ROTATE_REL -- aborting route")
                # The rotation may still be physically executing on the
                # motor co-processor (2026-08-25 bench data, see
                # stop_and_verify()'s docstring) -- actively stop it,
                # verified, before handing an "aborted" robot back to the
                # caller. The old fear that STOP-mid-rotation hangs the
                # firmware was not reproduced in 9 deliberate bench trials.
                self.stop_and_verify(leg_label)
                return False
        self.get_logger().error(
            f"{leg_label}: no ROTATE_REL COMPLETE within "
            f"{timeout_sec:.1f}s -- aborting route")
        # Same reasoning as above: a timed-out rotation is EXACTLY the
        # case where the robot is most likely still moving (incident #1,
        # Alvik4 off-table, was this precise signature).
        self.stop_and_verify(leg_label)
        return False

    def turn_to_heading(self, target_heading_deg: float, leg_label: str) -> bool:
        """REVERTED 2026-07-31 to the tapered proportional-speed law -- the
        version actually committed to GitHub (commit 0634800) and the one
        that produced the accurate, no-overshoot, single-pass perimeter
        run. Session history: this file's turn law was later REWRITTEN
        2026-07-28 (same day, after that commit) to a constant-turn_rpm-
        then-hard-brake-then-discrete-re-approach law, chasing a different
        problem (drive_leg()'s straight-line brake behavior) -- that
        rewrite was never re-validated against the perimeter-run baseline
        and, confirmed on real 2-robot hardware testing 2026-07-31, showed
        real turn-to-turn coast-distance INCONSISTENCY at turn_rpm=35 (some
        turns settled within a few degrees on the first brake, others
        overshot 60+deg and needed a slow ~4s turn_creep_rpm re-approach to
        recover) -- exactly the kind of instability a hard-brake law is
        prone to and a smooth taper is not. Restored verbatim (adapted only
        to this file's self._spin_once() executor, not the module-level
        rclpy.spin_once() the original used) rather than re-tuned, since the
        original was proven accurate and this session's rewrite was not an
        improvement on it. If turn_then_drive_leg()'s no-brake fusion is
        ever revisited, note it was built against the LATER brake law and
        will need matching rework against this taper.

        No hard brake anywhere: speed decreases continuously as the robot
        approaches the target, so momentum is shed gradually instead of
        needing to be caught by a STOP. --turn-settle-count consecutive
        in-tolerance samples (not just one) are required before declaring
        the turn done and re-arming, because a single sample can land
        in-tolerance while the robot is still slowly rotating."""
        pose = self.control_pose()
        if pose is None:
            self.get_logger().error(
                f"{leg_label}: lost {self.localization_source_label()} -- "
                "aborting route")
            return False
        error = yaw_error_deg(target_heading_deg, pose.yaw)
        if abs(error) <= self.args.turn_tol_deg:
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_tol_deg:.1f} deg "
                f"of target heading {target_heading_deg:.0f}, skipping turn")
            return True

        # A worker can reach this method immediately after drive_leg(),
        # which deliberately leaves the firmware IDLE. Arm streaming only
        # when a real turn is needed; arming before the tolerance check would
        # start the 300 ms watchdog for a skipped turn and then leave it
        # unfed during the supervisor's next scheduling wait.
        if not self.enter_wheel_follow_mode():
            self.get_logger().error(
                f"{leg_label}: did not ack WHEEL_FOLLOW_MODE before turn")
            return False

        self.get_logger().info(
            f"{leg_label}: turning in place from {pose.yaw:+.1f} to "
            f"{target_heading_deg:.0f} deg")
        period = 1.0 / CONTROL_HZ
        settled_count = 0
        last_settle_pose_t: float | None = None
        while rclpy.ok():
            loop_t0 = time.monotonic()
            self._spin_once(0.0)

            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting route")
                return False

            pose = self.control_pose()
            if pose is None:
                # See the matching stale-vision handling in drive_leg() --
                # send_wheel(0.0, 0.0) is not a confirmed-safe stop here;
                # use a verified/re-sent STOP (blind while vision is out)
                # and abort.
                self.stop_and_verify(leg_label, timeout_sec=1.5)
                self.get_logger().error(
                    f"{leg_label}: {self.localization_source_label()} lost -- "
                    "STOP sent, aborting route")
                return False

            error = yaw_error_deg(target_heading_deg, pose.yaw)
            if abs(error) <= self.args.turn_tol_deg:
                # A hard stop_and_rearm() (alvik.brake()) fired the instant
                # the robot first sampled inside tolerance was itself the
                # disturbance -- braking hard mid-rotation has its own
                # recoil/settle, kicking the robot back OUT of tolerance
                # before a 2nd/3rd consecutive sample could land. Fixed at
                # the source by the creep-speed taper below (active once
                # |error| is inside --turn-decel-zone-deg): the robot is
                # already moving at near-crawl speed by the time it enters
                # tolerance, so a plain zero-speed hold here is enough; no
                # separate brake event, no recoil, no re-trigger.
                # Count independent localization observations, not 50 Hz
                # control-loop repeats of the same odometry sample. Without
                # this guard encoder mode could count one 300 ms pose three
                # times in 60 ms and declare a still-moving turn settled.
                if pose.t != last_settle_pose_t:
                    settled_count += 1
                    last_settle_pose_t = pose.t
                    if self.args.verbose:
                        self.get_logger().info(
                            f"  yaw={pose.yaw:+6.1f} err={error:+6.1f} in-tol "
                            f"({settled_count}/{self.args.turn_settle_count})")
                self.send_wheel(0.0, 0.0)
                if settled_count < self.args.turn_settle_count:
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, period - elapsed))
                    continue
                self.get_logger().info(f"{leg_label}: turn complete (yaw={pose.yaw:+.1f})")
                # Leave the robot genuinely IDLE between plan operations.
                # drive_leg()/the next real turn arms WHEEL_FOLLOW_MODE at
                # its own start. Re-arming here would expose the firmware's
                # 300 ms watchdog while the supervisor legitimately waits.
                self.stop_only()
                return True
            settled_count = 0
            last_settle_pose_t = None

            # In-place pivot: matches the firmware's own updateTurn() pattern
            # (proportional speed scaled by error, opposite wheel signs) --
            # positive error (matches yawError()'s sign: need to increase
            # yaw) turns left (right wheel +, left wheel -), same convention
            # confirmed for LEFT_UNTIL_COLOR earlier this session.
            #
            # Two-zone speed law: outside the decel zone the ramp targets
            # turn-max-speed as error grows (saturating at turn-scale-deg).
            # INSIDE the decel zone (|error| < turn-decel-zone-deg), speed
            # tapers (quadratically) toward turn-creep-speed as error
            # approaches turn-tol-deg, so the robot arrives already
            # near-stalled instead of decelerating only after crossing into
            # tolerance -- this is what lets the settle check above use a
            # plain zero-speed hold instead of a hard brake.
            abs_error = abs(error)
            if abs_error <= self.args.turn_decel_zone_deg:
                span = max(self.args.turn_decel_zone_deg - self.args.turn_tol_deg, 0.01)
                frac = max(abs_error - self.args.turn_tol_deg, 0.0) / span
                spd = self.args.turn_creep_speed + (
                    self.args.turn_min_speed - self.args.turn_creep_speed) * frac ** 2
            else:
                # Rescaled to start exactly at turn-min-speed at
                # turn-decel-zone-deg (continuous with the branch above --
                # no step change at the zone boundary) and saturate at
                # turn-max-speed by turn-scale-deg.
                span = max(self.args.turn_scale_deg - self.args.turn_decel_zone_deg, 0.01)
                scale = min((abs_error - self.args.turn_decel_zone_deg) / span, 1.0)
                spd = self.args.turn_min_speed + (
                    self.args.turn_max_speed - self.args.turn_min_speed) * scale
            if error > 0.0:
                self.send_wheel(-spd, spd)
            else:
                self.send_wheel(spd, -spd)

            if self.args.verbose:
                self.get_logger().info(f"  yaw={pose.yaw:+6.1f} err={error:+6.1f} spd={spd:.0f}")

            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, period - elapsed))
        return False

    def turn_heading_test_route(self, headings: list[float]) -> None:
        """Bench-measurement mode added 2026-07-31: chain the REAL
        turn_to_heading() (the reverted tapered-decel-zone law, see its
        docstring) across a list of absolute target headings, with no
        drive_leg() involved at all -- unlike --turn-test (constant-RPM/
        hard-brake law, a DIFFERENT method entirely) and --rotate-test
        (firmware ROTATE_REL, also a different method), this is the only
        mode that actually exercises turn_to_heading() in isolation. Needs
        the caller to have already called enter_wheel_follow_mode() once;
        turn_to_heading() re-arms itself via stop_and_rearm() after each
        successful turn, so no extra re-arm step is needed between turns
        here (unlike turn_test_route())."""
        results: list[tuple[str, float, float]] = []
        for i, target_heading in enumerate(headings):
            leg_label = f"turn-heading-test #{i + 1}->{target_heading:.0f}deg"
            pose_before = self.fresh_pose()
            start_yaw = pose_before.yaw if pose_before is not None else float("nan")
            ok = self.turn_to_heading(target_heading, leg_label)
            if not ok:
                self.get_logger().error(
                    f"{leg_label}: aborting remainder of turn-heading-test route")
                break
            pose_after = self.fresh_pose()
            final_yaw = pose_after.yaw if pose_after is not None else float("nan")
            final_error = yaw_error_deg(target_heading, final_yaw)
            self.get_logger().info(
                f"{leg_label}: RESULT start={start_yaw:+.1f} "
                f"final_yaw={final_yaw:+.1f} target={target_heading:.1f} "
                f"final_error={final_error:+.2f}deg")
            results.append((leg_label, target_heading, final_error))

        if not results:
            return
        self.get_logger().info(
            f"turn-heading-test route summary ({len(results)}/{len(headings)} turns):")
        for leg_label, target_heading, final_error in results:
            self.get_logger().info(
                f"  {leg_label:<28s} target={target_heading:6.1f}deg  "
                f"final_err={final_error:+6.2f}deg")
        final_errs = [abs(e) for _, _, e in results]
        n = len(final_errs)
        self.get_logger().info(
            f"  |final_error|: mean={sum(final_errs)/n:.2f}deg "
            f"min={min(final_errs):.2f}deg max={max(final_errs):.2f}deg")

    def turn_then_drive_leg(self, target_heading_deg: float, drive_x: float,
                            drive_y: float, leg_label: str) -> bool:
        """Turn toward target_heading_deg, then -- once confirmed stable
        within --turn-tol-deg -- hand off DIRECTLY into drive_leg()'s
        wheel-speed law toward (drive_x, drive_y), with no STOP, settle
        wait, or re-arm round trip between the turn and the drive: wheels
        go straight from (held-at-zero, confirmed-stopped) turning to
        driving speeds. The no-gap optimization is specifically the
        turn-to-drive transition; drive_leg() is still followed by a real,
        settled turn_to_heading() square-up at arrival (see below) before
        returning, same as every other leg -- this method has no way to
        know whether whatever comes after IT will itself fuse into that
        square-up, so it can't skip settling there.

        REAL BUG confirmed on hardware 2026-07-31 (second one, after the
        collision fix below): the very first working version of this method
        drove straight to drive_x/drive_y and returned WITHOUT ever squaring
        up at arrival. Fine for a route driven by run() (which always calls
        turn_to_heading() unconditionally right after drive_leg() -- see
        skip_drive there), but fleetSupervisor.py's per-plan-item dispatch
        never separately processes a move that was consumed as a fusion
        target (record_done_fused() advances next_index PAST it), so
        nothing else was ever going to square up at wherever this drive
        actually landed. Real result: Alvik1 arrived at node 0 still facing
        the previous leg's approach heading and drove toward node 1 ~90deg
        off. Fixed by squaring up here, using target_heading_deg itself --
        the leg just driven is a straight line toward that same heading, so
        the arrival square-up angle IS target_heading_deg, no need to
        recompute from origin/destination coordinates.

        Added 2026-07-31 per explicit user direction: turn_to_heading()
        always sends STOP the instant --turn-tol-deg is reached (to safely
        settle and re-check the REST position -- see that function's own
        history of bugs from skipping this for a STANDALONE turn), which is
        correct when a turn is the end of the story (a workstation/depot
        square-up with nothing following), but wastes real time -- a stop,
        ~1s settle poll, and a full re-arm round trip -- when the very next
        thing to do is drive_leg() anyway. Since wheel speed is streamed
        directly (WHEEL_FOLLOW_MODE), there's no reason the turn's last
        tick and the leg's first tick can't be the same tick.

        ONLY use this for a turn immediately followed by a drive on the SAME
        node (e.g. "turn at node9 toward node9->114" in run()/VisionLegWorker
        -- the turn toward the NEXT leg's heading). Never use this for a
        standalone rotate_at/workstation square-up with no following drive --
        those must still brake and settle via turn_to_heading(), or the
        robot would keep coasting with nothing to steer it back onto the
        taped line.

        FIXED 2026-07-31 (real hardware collision): the first version of
        this method drove at full --turn-rpm right up to the tick tolerance
        was crossed, then handed off to drive_leg() immediately -- with
        real, uncorrected angular momentum from the still-fast turn on
        handoff. drive_leg()'s own heading correction (kp_yaw, clamped to
        max_turn_adjust) is a WEAK proportional trim for small drift, not a
        real turn -- it assumes the robot arrives already pointed roughly
        right AND STATIONARY in yaw. On a real Alvik1 route this handed off
        5.2deg short of target while still actively rotating; drive_leg()
        could not correct the combined error and the robot drove off the
        taped line, colliding with a reference tag (node 20). Fixed by
        adding a real deceleration phase (mirrors turn_to_heading()'s own
        brake_lead/creep transition, just without ever fully stopping):
        --turn-rpm until within --turn-brake-lead-deg of target, THEN drop
        to slow --turn-creep-rpm, and only hand off to drive_leg() once (a)
        within --turn-tol-deg AND (b) yaw has been STABLE (not still
        actively rotating) for STABLE_TICKS_REQUIRED consecutive control
        ticks at creep speed -- crossing the tolerance threshold once is no
        longer sufficient by itself. If tolerance is never reached within
        --turn-tol-deg after max_attempts, falls back to a real
        stop_and_rearm() + drive_leg() (same as calling turn_to_heading()
        then drive_leg() separately).

        Returns False on error/abort (caller should stop the route), same
        as turn_to_heading()/drive_leg()."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting route")
            return False
        error = yaw_error_deg(target_heading_deg, pose.yaw)
        if abs(error) <= self.args.turn_tol_deg:
            # Already within tolerance at entry -- still goes through the
            # SAME stability-confirmation loop below (just starting with
            # zero turning to do) rather than handing off immediately. Real
            # bug fixed 2026-07-31: an earlier version handed off here with
            # NO check at all, which is unsafe if the robot arrives with any
            # residual rotation left over from whatever action preceded this
            # call (e.g. the tail of a drive_leg() brake).
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_tol_deg:.1f} "
                f"deg of target heading {target_heading_deg:.0f}, confirming "
                "stable before driving through")
        else:
            self.get_logger().info(
                f"{leg_label}: turning in place from {pose.yaw:+.1f} to "
                f"{target_heading_deg:.0f} deg (then driving, no brake), "
                f"turn_rpm={self.args.turn_rpm:.0f}, "
                f"brake_lead={self.args.turn_brake_lead_deg:.2f}deg")

        no_progress_timeout_sec = 5.0
        max_attempts = 5
        # Consecutive control ticks BOTH within turn_tol_deg AND with yaw
        # essentially unchanged tick-to-tick, required before handing off to
        # drive_leg() -- crossing the tolerance threshold once is not
        # sufficient (see the FIXED note above: that was the actual
        # collision cause). At CONTROL_HZ=50, 4 ticks is ~80ms -- long
        # enough to distinguish "still rotating" from "settled" without
        # meaningfully reintroducing the stop-and-settle delay this method
        # exists to avoid.
        STABLE_TICKS_REQUIRED = 4
        STABLE_YAW_EPS_DEG = 0.5
        for attempt in range(1, max_attempts + 1):
            best_abs_error = math.inf
            t_progress0 = time.monotonic()
            self.send_wheel(0.0, 0.0)  # re-stamp wheel_cmd_last_ms -- see turn_to_heading()
            stable_ticks = 0
            last_yaw_for_stability: float | None = None

            period = 1.0 / CONTROL_HZ
            while rclpy.ok():
                loop_t0 = time.monotonic()
                self._spin_once(0.0)

                if self.errored:
                    self.get_logger().error(
                        f"{leg_label}: robot reported {self.error_text} -- aborting route")
                    return False

                pose = self.fresh_pose()
                if pose is None:
                    # Verified/re-sent STOP (2026-08-25), blind while
                    # vision is out -- same as the other vision-lost aborts.
                    self.stop_and_verify(leg_label, timeout_sec=1.5)
                    self.get_logger().error(
                        f"{leg_label}: vision lost -- STOP sent, aborting route")
                    return False

                error = yaw_error_deg(target_heading_deg, pose.yaw)
                abs_error = abs(error)

                if abs_error < best_abs_error - 0.1:
                    best_abs_error = abs_error
                    t_progress0 = loop_t0
                elif loop_t0 - t_progress0 > no_progress_timeout_sec:
                    self.get_logger().error(
                        f"{leg_label}: no progress for {no_progress_timeout_sec:.0f}s "
                        f"(stuck at {abs_error:.1f}deg error, last status "
                        f"'{self.last_status}') -- aborting route")
                    # CHANGED 2026-08-25 from stop_and_rearm() -- same
                    # reasoning as drive_leg()'s no-progress abort: never
                    # re-arm (or clear errored) on an abort path; send a
                    # verified STOP instead.
                    self.stop_and_verify(leg_label)
                    return False

                if abs_error <= self.args.turn_tol_deg:
                    # Within tolerance: HOLD (zero speed), not creep --
                    # continuing to command creep-speed rotation here would
                    # let the robot drift back out of tolerance instead of
                    # actually coming to rest, defeating the whole point of
                    # the stability check below. Confirmed on hardware
                    # 2026-07-31: crossing this threshold once is NOT enough
                    # (see the FIXED note above).
                    self.send_wheel(0.0, 0.0)
                    yaw_delta = (0.0 if last_yaw_for_stability is None
                                 else abs(pose.yaw - last_yaw_for_stability))
                    if yaw_delta <= STABLE_YAW_EPS_DEG:
                        stable_ticks += 1
                    else:
                        stable_ticks = 0  # still actively rotating -- reset
                    last_yaw_for_stability = pose.yaw
                    if stable_ticks >= STABLE_TICKS_REQUIRED:
                        self.get_logger().info(
                            f"{leg_label}: stable within tolerance "
                            f"(yaw={pose.yaw:+.1f}, {abs_error:.2f}deg from "
                            "target) -- driving through, no brake")
                        # Square up at ARRIVAL too, same as the non-fused
                        # drive_leg()+turn_to_heading() pair every other leg
                        # uses. REAL BUG confirmed on hardware 2026-07-31: a
                        # move that fuses its OWN departure turn into the
                        # PRECEDING call (see run()/_execute() in
                        # fleetSupervisor.py) is never separately dispatched
                        # afterward, so nothing else ever squares up at
                        # WHERE this drive_leg() below actually lands --
                        # Alvik1 arrived at node 0 still facing whatever
                        # direction this leg happened to end pointing and
                        # immediately started driving toward node 1 ~90deg
                        # off. drive_leg() only targets position (see its
                        # own docstring) -- it can arrive facing an
                        # arbitrary angle just like any other leg.
                        # target_heading_deg IS this leg's own heading (the
                        # turn above pointed at drive_x/drive_y in a
                        # straight line), so re-use it directly -- no need
                        # to recompute from origin/destination coordinates.
                        if not self.drive_leg(drive_x, drive_y, leg_label):
                            return False
                        return self.turn_to_heading(
                            target_heading_deg,
                            f"square up at {leg_label} to "
                            f"{target_heading_deg:.0f}deg (post-fusion)")
                    if self.args.verbose:
                        self.get_logger().info(
                            f"  yaw={pose.yaw:+6.1f} err={error:+6.1f}deg "
                            f"HOLD stable_ticks={stable_ticks}")
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, period - elapsed))
                    continue

                stable_ticks = 0
                last_yaw_for_stability = None

                # Decelerate to creep speed BEFORE reaching tolerance (once
                # within turn_brake_lead_deg), same trigger turn_to_heading()
                # uses for its brake -- here it's a speed drop, not a stop.
                in_creep_zone = abs_error <= self.args.turn_brake_lead_deg
                spd = (self.args.turn_creep_rpm
                       if attempt > 1 or in_creep_zone else self.args.turn_rpm)
                if error > 0.0:
                    self.send_wheel(-spd, spd)
                else:
                    self.send_wheel(spd, -spd)

                if self.args.verbose:
                    self.get_logger().info(
                        f"  yaw={pose.yaw:+6.1f} err={error:+6.1f}deg spd={spd:.0f}")

                elapsed = time.monotonic() - loop_t0
                time.sleep(max(0.0, period - elapsed))
            else:
                return False  # rclpy.ok() went false mid-turn

        # Never reached tolerance within max_attempts (each attempt above
        # runs until abs_error <= turn_tol_deg or the no-progress timeout --
        # this is the fallback if turn_rpm/turn_creep_rpm genuinely can't
        # close the gap, mirroring turn_to_heading()'s own max_attempts
        # exhaustion). Fall back to a real brake + settle + drive, same as
        # calling turn_to_heading() then drive_leg() separately.
        self.get_logger().error(
            f"{leg_label}: did not reach {self.args.turn_tol_deg:.1f}deg tol "
            f"after {max_attempts} attempts -- braking and re-approaching "
            "via turn_to_heading() before driving")
        if not self.turn_to_heading(target_heading_deg, leg_label):
            return False
        if not self.drive_leg(drive_x, drive_y, leg_label):
            return False
        return self.turn_to_heading(
            target_heading_deg,
            f"square up at {leg_label} to {target_heading_deg:.0f}deg (post-fusion)")

    def run(self, route: list[tuple[int, float, float]],
            rotate_at: dict[int, float] | None = None,
            dwell_at: dict[int, int | None] | None = None) -> None:
        localization_err = self.wait_for_localization_ready()
        if localization_err is not None:
            self.get_logger().error(
                f"{localization_err} -- localization="
                f"{self.localization_mode} cannot start")
            return

        if mode_requires_odom(self.localization_mode):
            if not self.reset_onboard_pose(
                    f"localization={self.localization_mode} route start"):
                return
            if not self.wait_for_fresh_odom(timeout_sec=3.0):
                self.get_logger().error(
                    "no fresh onboard pose after route-start reset")
                return
            _n0, x0, y0 = route[0]
            _n1, x1, y1 = route[1]
            if not self.ensure_odom_anchor(
                    x0, y0, heading_between(x0, y0, x1, y1)):
                self.get_logger().error(
                    "could not anchor onboard odometry at route start")
                return

        self.get_logger().info(
            f"route localization={self.localization_mode}: "
            f"{' -> '.join(f'node{n}' for n, _, _ in route)}")

        rotate_at = rotate_at or {}
        dwell_at = dwell_at or {}
        try:
            # First node is assumed to be the robot's actual starting point
            # (route[0]'s coordinates are only used for logging/sanity --
            # driving starts toward route[1]). skip_drive: set when the
            # PREVIOUS iteration's turn_then_drive_leg() already drove this
            # leg as part of a fused turn+drive handoff (see below) -- the
            # next iteration must not drive_leg() the same leg again.
            skip_drive = False
            for i in range(1, len(route)):
                prev_n, px, py = route[i - 1]
                n, tx, ty = route[i]
                leg_label = f"leg {prev_n}->{n}"
                if not skip_drive:
                    if not self.drive_leg(tx, ty, leg_label):
                        return
                skip_drive = False
                # Square up to THIS leg's own intended heading before
                # anything else. Confirmed 2026-07-27: drive_leg() only
                # targets position -- it steers toward wherever currently
                # points at the target, which drifts as the robot
                # approaches, so it can arrive facing an arbitrary angle
                # (observed: ~106 deg instead of the intended 90). Mirrors
                # the reference driveTo()'s own structure (a separate
                # rotateTo() after its position loop, not folded into
                # steering). Use the fast onboard alvik.rotate() primitive;
                # camera/camera-assist modes retain one minor correction.
                leg_heading = heading_between(px, py, tx, ty)
                if not self.turn_to_heading_rotate_rel(
                        leg_heading, f"square up at node{n} to {leg_heading:.0f} deg"):
                    return
                # Explicit standalone rotation (--rotate-at), applied BEFORE
                # any automatic turn toward the next route leg, so a request
                # like "face 180 at the workstation" happens exactly once,
                # at arrival, regardless of whether more legs follow.
                if n in rotate_at:
                    if not self.turn_to_heading_rotate_rel(
                            rotate_at[n], f"rotate at node{n} (requested)"):
                        return
                # DWELL (service pause) AFTER any requested rotation, matching
                # the solver's own ordering (ROTATE_180 then DWELL when
                # leaving a workstation -- see generateCommandLines() in
                # agv_grid_workstation_solver.html).
                if n in dwell_at:
                    if not self.dwell(dwell_at[n], f"dwell at node{n} (requested)"):
                        return
                if i < len(route) - 1:
                    next_n, nx, ny = route[i + 1]
                    next_heading = heading_between(tx, ty, nx, ny)
                    # Preserve the selected localization source for sizing,
                    # but execute the efficient alvik.rotate() maneuver rather
                    # than the slow tapered wheel-speed controller.
                    if not self.turn_to_heading_rotate_rel(
                            next_heading, f"turn at node{n} toward node{next_n}"):
                        return
            self.get_logger().info("route complete.")
        except KeyboardInterrupt:
            pass
        finally:
            self.send_cmd("STOP")
            for _ in range(5):
                self._spin_once(0.05)
            self.get_logger().info("STOP sent, exiting.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Drive one Alvik through a sequence of grid waypoints "
                     "using selectable encoder, camera-assisted, or camera "
                     "localization without color-node detection.")
    ap.add_argument("--robot", default="Alvik3")
    ap.add_argument(
        "--localization-mode",
        choices=["encoder", "camera_assist", "camera"],
        default="camera_assist",
        help="pose source used for route steering/arrival and fast turn sizing: "
             "encoder=onboard odometry only; camera_assist=odometry with "
             "AprilTag drift correction; camera=AprilTag only")
    ap.add_argument("--route", required=True,
                     help="comma-separated grid node numbers, e.g. 1,8,16 "
                          "(first node = robot's actual starting position, "
                          "used for logging only -- driving starts toward "
                          "the second node)")
    ap.add_argument("--rotate-at", action="append", default=[],
                     metavar="NODE:DEGREES",
                     help="insert a standalone in-place rotation to an "
                          "absolute heading (this robot's yaw convention: "
                          "0=-y, 90=+x, 180=+y, 270=-x) once the route "
                          "arrives at NODE, before continuing to the next "
                          "leg's own turn. Repeatable. Example: "
                          "--rotate-at 65:180 rotates to face +y at node 65.")
    ap.add_argument("--dwell-at", action="append", default=[],
                     metavar="NODE[:MS]",
                     help="send DWELL (service pause) once the route arrives "
                          "at NODE, AFTER any --rotate-at for that node -- "
                          "matches agv_grid_workstation_solver.html's own "
                          "ROTATE_180-then-DWELL ordering when leaving a "
                          "workstation. Omit :MS to use the firmware's "
                          "WORKSTATION_WAIT_MS default (2000ms); otherwise "
                          "clamped to [200,30000]ms same as the firmware. "
                          "Repeatable. Example: --dwell-at 65 --dwell-at 91:3000")
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--cols", type=int, default=8)
    # turn_to_heading() tapered proportional-speed law -- RESTORED
    # 2026-07-31 to the version actually committed to GitHub (commit
    # 0634800), replacing a same-day-later (2026-07-28) constant-turn-RPM +
    # hard-brake rewrite that was never re-validated against the
    # perimeter-run baseline and showed real coast-distance inconsistency
    # on 2-robot hardware testing (some turns clean, others 60+deg
    # overshoot needing a slow re-approach). See turn_to_heading()'s own
    # docstring for the full history.
    ap.add_argument("--turn-tol-deg", type=float, default=1.0,
                     help="consider a turn complete within this many degrees "
                          "of the target heading (default 1.0 -- bench-"
                          "verified 2026-07-27 with turn-min/max-speed "
                          "10/70 on a rotateTo() using onboard odometry at "
                          "up to 200Hz; vision-based turning here is capped "
                          "by the camera pipeline's actual update rate, so "
                          "expect this tolerance to hold reliably but not "
                          "necessarily match that function's 0.1deg bench "
                          "repeatability)")
    ap.add_argument("--turn-settle-count", type=int, default=3,
                     help="require this many CONSECUTIVE fresh pose samples "
                          "within turn-tol-deg before declaring a turn "
                          "complete (default 3). Confirmed 2026-07-27: a "
                          "single in-tolerance sample can fire STOP while "
                          "the robot is still physically rotating (motor "
                          "still commanded at turn-min-speed, not "
                          "decelerated), and momentum/backlash then carries "
                          "the robot several more degrees past where the "
                          "log says it stopped (observed: logged yaw=+91.0, "
                          "camera overlay read +82 once actually at rest). "
                          "Waiting for consecutive samples both rejects a "
                          "one-off noisy reading and gives the robot more "
                          "ticks to actually decelerate before latching.")
    ap.add_argument("--turn-min-speed", type=float, default=10.0,
                     help="turn-in-place speed, RPM, at the OUTER edge of "
                          "the decel zone (--turn-decel-zone-deg) -- no "
                          "longer a hard floor for the whole approach, see "
                          "--turn-creep-speed for the taper inside that zone")
    ap.add_argument("--turn-decel-zone-deg", type=float, default=35.0,
                     help="once |error| drops below this many degrees, "
                          "speed tapers (quadratically -- see turn_to_heading) "
                          "from --turn-min-speed down to --turn-creep-speed "
                          "as error approaches --turn-tol-deg (default 35, "
                          "widened 2026-07-28 from an initial 8: at 8deg "
                          "there wasn't enough angular distance for the "
                          "robot's real momentum -- measured ~134deg/s "
                          "during the turn-max-speed phase -- to actually "
                          "bleed off before reaching the target, so it "
                          "coasted through the whole zone and overshot, "
                          "then had to reverse and re-approach, i.e. an "
                          "extra full pass instead of one smooth turn).")
    ap.add_argument("--turn-creep-speed", type=float, default=4.0,
                     help="turn-in-place speed, RPM, right at --turn-tol-deg "
                          "(the innermost edge of the decel zone) -- low "
                          "enough that one control tick (1/30s) can't carry "
                          "the robot past the target, so it settles without "
                          "needing a hard brake to catch it after the fact")
    ap.add_argument("--turn-scale-deg", type=float, default=60.0,
                     help="error magnitude (deg) at which turn speed "
                          "saturates to --turn-max-speed, ramping up from "
                          "--turn-min-speed at --turn-decel-zone-deg "
                          "(default 60; must be > --turn-decel-zone-deg). "
                          "Smaller values make errors just outside the "
                          "decel zone ramp to speed faster.")
    ap.add_argument("--turn-max-speed", type=float, default=35.0,
                     help="turn-in-place maximum wheel speed, RPM (default "
                          "35, lowered 2026-07-28 from 70 -- WHEEL_FOLLOW_MAX_RPM "
                          "is 70, but a 90deg turn (D1->DE1->node0's "
                          "node-2->node0 leg) held spd=65-70 for a long "
                          "stretch before the decel zone caught it and "
                          "overshot the 270deg target by ~58deg, out to "
                          "-147.9deg, then entered a sustained limit-cycle "
                          "oscillation for several seconds -- widening the "
                          "decel zone (see --turn-decel-zone-deg) wasn't "
                          "enough runway to shed that much rotational "
                          "momentum. Capping the top speed itself is the "
                          "more direct fix: less momentum ever builds up, "
                          "so there's less to decelerate regardless of zone "
                          "shape. Confirmed 2026-07-27 (an EARLIER, narrower "
                          "decel-zone version of this same code) that a "
                          "clean 90deg turn completes in well under 1s even "
                          "at reduced speeds -- this is not the bottleneck "
                          "for overall route time.")
    # turn_then_drive_leg()-ONLY params (the constant-RPM + hard-brake law
    # turn_to_heading() used before the 2026-07-31 revert above). This
    # method is currently DISABLED in fleetSupervisor.py (see
    # run_advised_vision()'s docstring -- caused two real hardware
    # collisions) but is still reachable from the standalone CLI/run(), so
    # these params still need to exist and validate even though nothing
    # exercises them in normal use right now.
    ap.add_argument("--turn-rpm", type=float, default=35.0,
                     help="[turn_then_drive_leg() only] constant turn-in-place "
                          "wheel speed, RPM, for the whole turn before handoff "
                          "(default 35).")
    ap.add_argument("--turn-brake-lead-deg", type=float, default=30.0,
                     help="[turn_then_drive_leg() only] drop to --turn-creep-rpm "
                          "once within this many degrees of the target heading "
                          "(default 30).")
    ap.add_argument("--turn-creep-rpm", type=float, default=10.0,
                     help="[turn_then_drive_leg() only] much slower turn speed, "
                          "RPM, used once within --turn-brake-lead-deg or on "
                          "re-approach attempts (default 10).")
    # drive_leg() constant-cruise + hard-brake law (REPLACED 2026-07-28 --
    # see drive_leg()'s docstring for the full history. The prior
    # distance-proportional kp_dist/drive-min-speed/drive-max-speed law is
    # gone: user's explicit direction was "Instead of slowing down, we just
    # need the alvik to brake" -- hold a constant cruise RPM the whole leg
    # (matching the old color-sensor firmware's demonstrated 60-70RPM
    # constant-speed-then-instant-hard-stop behavior) and brake hard at a
    # measured lead distance instead of tapering speed down on approach.)
    ap.add_argument("--cruise-rpm", type=float, default=60.0,
                     help="constant drive wheel speed, RPM, for the whole "
                          "leg (before heading-correction turn_adjust is "
                          "applied) -- default 60, matching the old "
                          "color-sensor firmware's demonstrated hard-stop-"
                          "capable cruise speed and the --stop-test-rpm "
                          "default used to measure --brake-lead-in below.")
    ap.add_argument("--brake-lead-in", type=float, default=1.9,
                     help="fire a hard STOP once within this many inches of "
                          "the target (default 1.9). RE-MEASURED 2026-07-28 "
                          "(same day, later) after a TcpFrameSource.read() "
                          "fix (apriltag_localize.py) eliminated a vision-"
                          "pipeline stale-frame backlog -- the earlier 3.7in "
                          "value (mean slide 3.21in) had been calibrated "
                          "against that backlog's extra lag and was braking "
                          "too early once fixed (a --stop-test re-run at "
                          "brake_lead_in=3.7 post-fix showed slide drop to "
                          "1.64in mean, landing ~2in SHORT every time). "
                          "1.9 is confirmed via a follow-up 7-leg --stop-test "
                          "at brake_lead_in=1.9: mean slide 1.61in (range "
                          "1.55-1.67in), mean along-path error only 0.23in "
                          "short (range 0.11-0.30in), never overshot. If "
                          "--cruise-rpm is changed from 60, or the vision "
                          "pipeline changes again, RE-MEASURE with "
                          "--stop-test before trusting this default.")
    ap.add_argument("--kp-yaw", type=float, default=0.6,
                     help="proportional gain: wheel-speed turn adjustment per "
                          "degree of heading error toward the live target")
    ap.add_argument("--max-turn-adjust", type=float, default=10.0,
                     help="clamp on the heading-correction wheel-speed "
                          "differential added on top of drive speed, RPM. "
                          "Lowered 15->10 2026-07-28: --cruise-rpm's new "
                          "default (60, up from the old --drive-max-speed "
                          "default of 55) left only 10 RPM of headroom "
                          "under the firmware's 70 RPM WHEEL_FOLLOW_MAX_RPM "
                          "cap, not 15.")
    ap.add_argument("--verbose", action="store_true",
                     help="print position/error/correction every control tick")
    ap.add_argument("--stop-test", action="store_true",
                     help="measurement mode (2026-07-28): drive --route leg by "
                          "leg (2 or more nodes, e.g. 1,2,3,4,5,6,7,8) at a "
                          "CONSTANT --stop-test-rpm (no proportional slowdown) "
                          "and fire one hard brake per leg once within "
                          "--stop-test-lead-in of that leg's target, re-arming "
                          "between legs, then report target vs. actual final "
                          "position per leg plus a summary across all legs. "
                          "Use this BEFORE trusting a --cruise-rpm/"
                          "--brake-lead-in value: it measures real stopping "
                          "distance at a steady cruise RPM directly, matching "
                          "the old color-sensor firmware's constant-speed-"
                          "then-hard-stop behavior. Uses the SAME live-target "
                          "heading correction as drive_leg() (--kp-yaw / "
                          "--max-turn-adjust, added on top of the constant "
                          "cruise RPM) so legs track the taped line instead "
                          "of drifting -- added 2026-07-28 after an "
                          "uncorrected multi-leg run drifted steadily off "
                          "the line over several legs. --cruise-rpm/"
                          "--brake-lead-in and all --turn-* args are still "
                          "ignored in this mode (use --stop-test-rpm/"
                          "--stop-test-lead-in instead).")
    ap.add_argument("--stop-test-rpm", type=float, default=60.0,
                     help="constant cruise wheel speed, RPM, for --stop-test "
                          "(default 60, matching the old firmware's "
                          "demonstrated 60-70RPM hard-stop-capable cruise)")
    ap.add_argument("--stop-test-lead-in", type=float, default=3.0,
                     help="fire the brake once within this many inches of "
                          "the target, for --stop-test (default 3.0 -- start "
                          "wide and generous, then narrow based on the "
                          "measured slide-after-brake distance from the "
                          "first run; the goal is to find the SMALLEST lead "
                          "that still lands within tolerance, not to guess "
                          "it up front)")
    ap.add_argument("--turn-test", metavar="HEADINGS",
                     help="measurement mode (2026-07-28): rotate in place "
                          "through a comma-separated list of absolute target "
                          "headings (this robot's yaw convention: 0=-y, "
                          "90=+x, 180=+y, 270=-x), e.g. --turn-test 90,180,270,0 "
                          "-- at a CONSTANT --turn-test-rpm (no decel-zone "
                          "taper, unlike turn_to_heading()) and fire one hard "
                          "brake per turn once within --turn-test-lead-deg of "
                          "that turn's target, re-arming between turns, then "
                          "report target vs. actual final heading per turn "
                          "plus a summary. Use this BEFORE trusting a "
                          "--turn-rpm/--turn-brake-lead-deg value: it "
                          "measures real angular stopping distance at a "
                          "steady turn RPM directly (this is exactly how "
                          "turn_to_heading()'s own --turn-rpm/"
                          "--turn-brake-lead-deg defaults were derived, "
                          "after its earlier hand-tuned tapered decel zone "
                          "produced multi-rotation overshoot-correct cycles "
                          "on hardware). --route is REQUIRED by argparse "
                          "(parse_route() needs >=2 nodes) but unused in "
                          "this mode -- pass any 2 valid nodes, e.g. "
                          "--route 1,2. --turn-rpm/--turn-brake-lead-deg are "
                          "ignored in this mode (use --turn-test-rpm/"
                          "--turn-test-lead-deg instead).")
    ap.add_argument("--turn-test-rpm", type=float, default=35.0,
                     help="constant turn-in-place wheel speed, RPM, for "
                          "--turn-test (default 35, matching "
                          "--turn-max-speed's bench-confirmed-safe value "
                          "from 2026-07-27/28 tuning)")
    ap.add_argument("--turn-test-lead-deg", type=float, default=15.0,
                     help="fire the brake once within this many degrees of "
                          "the target heading, for --turn-test (default 15 "
                          "-- start wide and generous, then narrow based on "
                          "the measured overshoot/undershoot from the first "
                          "run, same approach as --stop-test-lead-in)")
    ap.add_argument("--turn-heading-test", metavar="HEADINGS",
                     help="measurement mode (2026-07-31): rotate in place "
                          "through a comma-separated list of absolute target "
                          "headings (same yaw convention as --turn-test: "
                          "0=-y, 90=+x, 180=+y, 270=-x), e.g. "
                          "--turn-heading-test 0,180,90,270,0 -- via the "
                          "REAL turn_to_heading() (the reverted tapered-"
                          "decel-zone law), with no drive_leg() involved. "
                          "Unlike --turn-test (constant-RPM/hard-brake, a "
                          "different method) and --rotate-test (firmware "
                          "ROTATE_REL, also different), this is the mode "
                          "that actually benches turn_to_heading() in "
                          "isolation -- uses the --turn-tol-deg/--turn-"
                          "settle-count/--turn-min-speed/--turn-decel-zone-"
                          "deg/--turn-creep-speed/--turn-scale-deg/--turn-"
                          "max-speed flags already defined above, same as a "
                          "real route's turns would. --route is REQUIRED by "
                          "argparse (parse_route() needs >=2 nodes) but "
                          "unused in this mode -- pass any 2 valid nodes, "
                          "e.g. --route 1,2.")
    ap.add_argument("--rotate-test", metavar="HEADINGS",
                     help="measurement mode (2026-07-30): rotate in place "
                          "through a comma-separated list of absolute target "
                          "headings (same yaw convention as --turn-test: "
                          "0=-y, 90=+x, 180=+y, 270=-x), e.g. "
                          "--rotate-test 90,180,270,0 -- but via the "
                          "firmware's ROTATE_REL command (alvik.rotate(), "
                          "closed-loop on Alvik's own motor-control MCU) "
                          "instead of streaming WHEEL_FOLLOW_MODE wheel-speed "
                          "setpoints. Use this to validate ROTATE_REL's real "
                          "accuracy before trusting turn_to_heading() (which "
                          "now uses ROTATE_REL by default, see its "
                          "docstring) inside a full route. No RPM/brake-lead "
                          "flags needed -- alvik.rotate() has no equivalent "
                          "tunable, it either lands accurately or it "
                          "doesn't. --route is REQUIRED by argparse "
                          "(parse_route() needs >=2 nodes) but unused in "
                          "this mode -- pass any 2 valid nodes, e.g. "
                          "--route 1,2. Requires firmware with the "
                          "ROTATE_REL command (AGV_Factory_camera_correction.ino, "
                          "added 2026-07-30) -- older firmware will report "
                          "ERROR UNKNOWN_COMMAND and abort the first leg.")
    ap.add_argument("--stop-interrupt-test", metavar="ANGLES",
                     help="measurement mode (2026-08-24): deliberately sends "
                          "STOP partway through an in-flight ROTATE_REL and "
                          "measures how far the robot keeps moving afterward "
                          "-- see stop_interrupt_test()'s own docstring for "
                          "the exact real hardware incident and library-"
                          "source tracing behind this test "
                          "(ROTATE_REL_HAZARD_2026-08-20.md 'Third root "
                          "cause'). Comma-separated relative angles, e.g. "
                          "--stop-interrupt-test 90,-90,177 -- run at every "
                          "point in --stop-interrupt-delays. Every prior use "
                          "of STOP in this codebase fires BETWEEN commands; "
                          "this is the first deliberate test of STOP as an "
                          "INTERRUPT of one already in flight, so run this in "
                          "an open area with room for unexpected movement. "
                          "--route is REQUIRED by argparse but unused, same "
                          "as --rotate-test -- pass any 2 valid nodes.")
    ap.add_argument("--stop-interrupt-delays", metavar="FRACS",
                     default="0.25,0.5,0.75",
                     help="comma-separated fractions (0.0-1.0) of each "
                          "angle's predicted ROTATE_REL duration at which to "
                          "send STOP, for --stop-interrupt-test (default "
                          "0.25,0.5,0.75 -- early/mid/late in the rotation).")
    ap.add_argument("--yaw-stress-test",
                     choices=["encoder", "camera_assist", "camera_only",
                              "encoder_drift", "all"],
                     help="measurement mode (2026-08-13): ROS2/rosbridge "
                          "equivalent of TurnSpeedBenchAlvik6.ino's standalone "
                          "Arduino stress test -- 128 there-then-back ROTATE_REL "
                          "rotations (same angle set/cable-safety pairing as the "
                          "Arduino sketch) run through the REAL production code "
                          "path (turn_to_heading_rotate_rel()'s send + this "
                          "file's own odom/vision callbacks over the actual "
                          "wireless link), in one of four yaw-source modes: "
                          "'encoder' sizes every ROTATE_REL from corrected "
                          "onboard yaw only, RESYNCED to vision after every "
                          "rotation (no raw drift visible); 'camera_assist' "
                          "does the same odom sizing but samples vision after "
                          "and sends one corrective ROTATE_REL if still outside "
                          "--turn-tol-deg; 'camera_only' sizes every ROTATE_REL "
                          "from vision yaw directly, ignoring odom; "
                          "'encoder_drift' (added 2026-08-20, per explicit user "
                          "direction) is the ONE mode that shows real "
                          "accumulated onboard-encoder drift: same odom sizing "
                          "as 'encoder', but with NO tier-start reset and NO "
                          "per-rotation resync, so error compounds freely "
                          "across all 128 rotations -- only meaningful run LAST "
                          "via 'all' (inherits whatever onboard yaw state "
                          "'camera_only' left), not standalone from a fresh "
                          "reset. 'all' (added 2026-08-13, extended 2026-08-20 "
                          "to include 'encoder_drift' last) chains all four "
                          "back to back with a --yaw-stress-settle-sec pause "
                          "between, one process/one flash. Run 'encoder' (or "
                          "'all', which starts with it) first and confirm zero "
                          "aborts/hangs before trusting turn_to_heading_rotate_"
                          "rel() (which uses 'camera_assist' behavior by "
                          "default) inside a full route -- see that method's "
                          "docstring for the ROTATE_REL hang history. --route "
                          "is REQUIRED by argparse but unused, same as "
                          "--rotate-test -- pass any 2 valid nodes.")
    ap.add_argument("--yaw-stress-settle-sec", type=float, default=5.0,
                     help="pause between tiers when --yaw-stress-test all is "
                          "used, seconds (default 5.0) -- lets the robot fully "
                          "settle/come to rest and gives you a moment to watch "
                          "before the next tier's reset_onboard_pose() fires.")
    args = ap.parse_args()
    worst_case = args.cruise_rpm + args.max_turn_adjust
    if worst_case > 70.0:  # WHEEL_FOLLOW_MAX_RPM in AGV_Factory_camera_correction.ino
        ap.error(
            f"--cruise-rpm ({args.cruise_rpm:.1f}) + "
            f"--max-turn-adjust ({args.max_turn_adjust:.1f}) = "
            f"{worst_case:.1f} RPM exceeds the firmware's WHEEL_FOLLOW_MAX_RPM "
            "(70.0) -- a full-authority turn adjustment would clip "
            "asymmetrically. Lower one of them.")
    stop_test_worst_case = args.stop_test_rpm + args.max_turn_adjust
    if args.stop_test and stop_test_worst_case > 70.0:
        ap.error(
            f"--stop-test-rpm ({args.stop_test_rpm:.1f}) + "
            f"--max-turn-adjust ({args.max_turn_adjust:.1f}) = "
            f"{stop_test_worst_case:.1f} RPM exceeds the firmware's "
            "WHEEL_FOLLOW_MAX_RPM (70.0). Lower one of them.")
    if args.turn_test is not None and args.turn_test_rpm > 70.0:
        ap.error(
            f"--turn-test-rpm ({args.turn_test_rpm:.1f}) exceeds the "
            "firmware's WHEEL_FOLLOW_MAX_RPM (70.0).")
    if args.turn_rpm > 70.0:
        ap.error(
            f"--turn-rpm ({args.turn_rpm:.1f}) exceeds the firmware's "
            "WHEEL_FOLLOW_MAX_RPM (70.0).")
    if args.turn_creep_rpm > 70.0:
        ap.error(
            f"--turn-creep-rpm ({args.turn_creep_rpm:.1f}) exceeds the "
            "firmware's WHEEL_FOLLOW_MAX_RPM (70.0).")

    try:
        route = parse_route(args.route, args.rows, args.cols)
    except ValueError as exc:
        ap.error(str(exc))
        return

    if args.stop_test:
        if len(route) < 2:
            ap.error(f"--stop-test requires at least 2 --route nodes "
                      f"(start, target[, target2, ...]), got {len(route)}")
            return
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            # Two-stage match-then-data wait (2026-08-25) -- see
            # wait_for_vision_ready()'s docstring for the discovery-race
            # this replaces (the old flat 5s wait timed out on a real run
            # with apriltag_localize.py confirmed publishing the whole time).
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "running on the camera laptop and can it see this "
                    "robot's tag?")
                return
            if not node.enter_wheel_follow_mode():
                node.get_logger().error(
                    f"robot did not ack WHEEL_FOLLOW_MODE "
                    f"(last status: '{node.last_status}') -- aborting")
                return
            node.stop_test_route(route, args.stop_test_rpm, args.stop_test_lead_in)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    if args.turn_test is not None:
        try:
            headings = [float(tok) for tok in args.turn_test.split(",")]
        except ValueError:
            ap.error(f"--turn-test expects comma-separated numbers, got "
                      f"{args.turn_test!r}")
            return
        if not headings:
            ap.error("--turn-test requires at least one heading")
            return
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            # Two-stage match-then-data wait (2026-08-25) -- see
            # wait_for_vision_ready()'s docstring for the discovery-race
            # this replaces (the old flat 5s wait timed out on a real run
            # with apriltag_localize.py confirmed publishing the whole time).
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "running on the camera laptop and can it see this "
                    "robot's tag?")
                return
            if not node.enter_wheel_follow_mode():
                node.get_logger().error(
                    f"robot did not ack WHEEL_FOLLOW_MODE "
                    f"(last status: '{node.last_status}') -- aborting")
                return
            node.turn_test_route(headings, args.turn_test_rpm, args.turn_test_lead_deg)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    if args.turn_heading_test is not None:
        try:
            headings = [float(tok) for tok in args.turn_heading_test.split(",")]
        except ValueError:
            ap.error(f"--turn-heading-test expects comma-separated numbers, "
                      f"got {args.turn_heading_test!r}")
            return
        if not headings:
            ap.error("--turn-heading-test requires at least one heading")
            return
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            # Two-stage match-then-data wait (2026-08-25) -- see
            # wait_for_vision_ready()'s docstring for the discovery-race
            # this replaces (the old flat 5s wait timed out on a real run
            # with apriltag_localize.py confirmed publishing the whole time).
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "running on the camera laptop and can it see this "
                    "robot's tag?")
                return
            if not node.enter_wheel_follow_mode():
                node.get_logger().error(
                    f"robot did not ack WHEEL_FOLLOW_MODE "
                    f"(last status: '{node.last_status}') -- aborting")
                return
            node.turn_heading_test_route(headings)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    if args.rotate_test is not None:
        try:
            headings = [float(tok) for tok in args.rotate_test.split(",")]
        except ValueError:
            ap.error(f"--rotate-test expects comma-separated numbers, got "
                      f"{args.rotate_test!r}")
            return
        if not headings:
            ap.error("--rotate-test requires at least one heading")
            return
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            # Two-stage match-then-data wait (2026-08-25) -- see
            # wait_for_vision_ready()'s docstring for the discovery-race
            # this replaces (the old flat 5s wait timed out on a real run
            # with apriltag_localize.py confirmed publishing the whole time).
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "running on the camera laptop and can it see this "
                    "robot's tag?")
                return
            # No enter_wheel_follow_mode() here -- ROTATE_REL is a
            # standalone firmware command (like ROTATE_180/DWELL), not
            # something that only works inside WHEEL_FOLLOW_MODE.
            node.rotate_test_route(headings)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    if args.stop_interrupt_test is not None:
        try:
            angles = [float(tok) for tok in args.stop_interrupt_test.split(",")]
        except ValueError:
            ap.error(f"--stop-interrupt-test expects comma-separated "
                      f"numbers, got {args.stop_interrupt_test!r}")
            return
        if not angles:
            ap.error("--stop-interrupt-test requires at least one angle")
            return
        try:
            delay_fracs = [float(tok) for tok in
                          args.stop_interrupt_delays.split(",")]
        except ValueError:
            ap.error(f"--stop-interrupt-delays expects comma-separated "
                      f"numbers, got {args.stop_interrupt_delays!r}")
            return
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "actually running and can it see this robot's tag?")
                return
            node.get_logger().warning(
                "STOP-INTERRUPT TEST: this deliberately sends STOP while "
                "ROTATE_REL is still executing -- an untested interaction. "
                "Confirm the robot has open space to move unexpectedly "
                "before proceeding.")
            # No enter_wheel_follow_mode() here -- ROTATE_REL is a
            # standalone firmware command, same reasoning as --rotate-test.
            node.stop_interrupt_test_route(angles, delay_fracs)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    if args.yaw_stress_test is not None:
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            # Two-stage match-then-data wait (2026-08-25) -- see
            # wait_for_vision_ready()'s docstring for the discovery-race
            # this replaces (the old flat 5s wait timed out on a real run
            # with apriltag_localize.py confirmed publishing the whole time).
            vision_err = node.wait_for_vision_ready()
            if vision_err is not None:
                node.get_logger().error(
                    f"{vision_err} -- is apriltag_localize.py --rosbridge "
                    "running on the camera laptop and can it see this "
                    "robot's tag?")
                return
            # Initial resync so an "encoder"/"camera_assist" FIRST tier has
            # a corrected_odom_yaw() to start from -- same pattern as run()'s
            # own mission-start resync attempt. Each tier's own
            # yaw_stress_test_route() call resets/resyncs again itself at
            # its own start regardless, so this is only a fast-fail check
            # (odom simply never arriving at all) before committing to a
            # 128-rotation run, not load-bearing for correctness.
            resync_t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - resync_t0 < 2.0:
                node._spin_once(0.1)
                if node.resync_yaw_offset():
                    break
            first_mode = "encoder" if args.yaw_stress_test == "all" else args.yaw_stress_test
            if node.yaw_offset is None and first_mode != "camera_only":
                node.get_logger().error(
                    "no odom (<robot>_pose) received within 2s -- is the "
                    "firmware's publish_pose() running? aborting "
                    f"(mode={first_mode!r} requires odom; camera_only does not)")
                return
            # No enter_wheel_follow_mode() here -- same reasoning as
            # --rotate-test: ROTATE_REL is a standalone firmware command.
            if args.yaw_stress_test == "all":
                modes = ["encoder", "camera_assist", "camera_only", "encoder_drift"]
                for i, mode in enumerate(modes):
                    if mode == "encoder_drift":
                        # Deliberately NOT reset/resynced -- see
                        # yaw_stress_test_route()'s reset_first/
                        # resync_after_each docstrings. Only meaningful
                        # run here, right after camera_only, inheriting
                        # whatever onboard yaw state that tier left.
                        node.yaw_stress_test_route(
                            mode, reset_first=False, resync_after_each=False)
                    else:
                        node.yaw_stress_test_route(mode)
                    if i < len(modes) - 1:
                        node.get_logger().info(
                            f"### settling {args.yaw_stress_settle_sec:.1f}s "
                            f"before next tier ({modes[i + 1]}) ###")
                        node.spin_for(args.yaw_stress_settle_sec)
            elif args.yaw_stress_test == "encoder_drift":
                node.get_logger().info(
                    "encoder_drift run standalone -- this only shows real "
                    "drift if this robot has already accumulated some "
                    "(e.g. right after a fresh reset there will be little "
                    "to see; run --yaw-stress-test all for the intended "
                    "encoder -> camera_assist -> camera_only -> "
                    "encoder_drift sequence)")
                node.yaw_stress_test_route(
                    "encoder_drift", reset_first=False, resync_after_each=False)
            else:
                node.yaw_stress_test_route(args.yaw_stress_test)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.get_logger().info("STOP sent, exiting.")
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        return

    rotate_at: dict[int, float] = {}
    for spec in args.rotate_at:
        try:
            node_str, deg_str = spec.split(":", 1)
            rotate_at[int(node_str)] = float(deg_str)
        except ValueError:
            ap.error(f"--rotate-at expects NODE:DEGREES, got {spec!r}")
            return

    dwell_at: dict[int, int | None] = {}
    for spec in args.dwell_at:
        try:
            if ":" in spec:
                node_str, ms_str = spec.split(":", 1)
                dwell_at[int(node_str)] = int(ms_str)
            else:
                dwell_at[int(spec)] = None
        except ValueError:
            ap.error(f"--dwell-at expects NODE or NODE:MS, got {spec!r}")
            return

    rclpy.init()
    node = CameraGridNavigator(args.robot, args)
    try:
        node.run(route, rotate_at, dwell_at)
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
