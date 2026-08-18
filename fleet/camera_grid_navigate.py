#!/usr/bin/env python3
"""
camera_grid_navigate.py — drive ONE Alvik through a sequence of grid-lattice
waypoints (node numbers, e.g. "1,2,10,18") using camera (AprilTag) position +
yaw only. No onboard Alvik sensor is used anywhere in this loop, same as
camera_line_follow.py -- this is that script generalized from one captured
straight segment to a multi-leg route with stop-and-turn transitions.

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

Turning between legs: turn_to_heading() streams WHEEL_FOLLOW_MODE
wheel-speed setpoints computed from vision yaw every tick (bench-tuned
2026-07-27/28: turn_rpm=35, turn_brake_lead_deg=30 -- see that function's
own docstring); kept as the fallback when no onboard yaw is available yet
(see turn_to_heading_rotate_rel()). An EARLIER ROTATE_REL-based turn
(alvik.rotate(), closed-loop on Alvik's own motor-control MCU) was tried
2026-07-30 and reverted the same day after multiple firmware hangs on real
hardware, root cause unresolved at the time. That hang was later
root-caused and FIXED at the firmware level (see AGV_Factory_camera_
correction.ino's ROTATE_REL handler comment) and re-validated 2026-08-13
with a 128-rotation hardware stress test (zero hangs) -- turn_to_heading_
rotate_rel() is the current, actively-used ROTATE_REL turn path: sizes
from onboard odometry (corrected_odom_yaw()), then samples vision once
after settling and sends ONE corrective ROTATE_REL if still outside
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
DEPOT_SLOT_WORLD_IN = {
    "D1": (20.0, 10.5), "D2": (25.3, 10.7), "D3": (30.3, 10.8),
    "D4": (35.7, 10.7), "D5": (42.0, 10.6), "D6": (49.3, 10.6),
}
DEPOT_ENTRY_WORLD_IN = {
    "DE1": (19.6, 3.0), "DE2": (24.9, 3.0), "DE3": (30.2, 3.3),
    "DE4": (35.4, 3.2), "DE5": (41.9, 3.2), "DE6": (49.1, 3.2),
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

        self.cmd_pub = self.create_publisher(String, f"{robot}_cmd", qos_best_effort)
        self.wheel_pub = self.create_publisher(String, f"{robot}_wheel_cmd", qos_best_effort)
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
                    f"last send_wheel() call (firmware watchdog is 300ms)")
        elif text.startswith("BUSY WHEEL_FOLLOW_MODE"):
            self.mode_ack_seen = True
        elif text == "DWELL COMPLETE":
            self.dwell_done_seen = True
        elif text == "ROTATE_REL COMPLETE":
            self.rotate_rel_done_seen = True
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

    def _on_odom_pose(self, msg: String) -> None:
        """Firmware's publish_pose(): {"x":..,"y":..,"yaw":..,"battery":..,
        "ms":..} -- a DIFFERENT key set than _on_vision_pose's x_in/y_in/
        yaw_deg (this one is CM/DEG straight from alvik.get_pose(), no unit
        suffix). Only yaw is used (see corrected_odom_yaw()) -- x/y aren't
        trusted here since onboard odometry drifts with wheel slip over a
        route the way yaw does not (yaw is IMU-fused, not pure encoder
        integration -- see the Alvik library). No jump-rejection like
        _on_vision_pose's: this is a much higher-trust, lower-latency local
        link (no camera/rosbridge round-trip), and turn_to_heading_rotate_
        rel() only ever reads the single latest value right before a turn,
        never integrates it over time the way drive_leg() does with vision."""
        try:
            d = json.loads(msg.data)
            yaw = float(d["yaw"])
        except (ValueError, KeyError, TypeError):
            return
        self.last_odom_yaw = yaw
        self.last_odom_yaw_at = time.monotonic()

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
        the 300ms WHEEL_CMD_TIMEOUT_MS watchdog can already be most of the
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
        races the firmware's own 300ms WHEEL_CMD_TIMEOUT_MS watchdog if that
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
        between legs let the firmware's 300ms WHEEL_CMD_TIMEOUT_MS watchdog
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
                # WHEEL_CMD_TIMEOUT_MS=300ms) instead of one blind sleep, so
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
        self.errored = False
        self.error_text = ""
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")
        timeout_sec = max(5.0, abs(rel_deg) * 0.15 + 3.0)
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.rotate_rel_done_seen:
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
        DWELL COMPLETE, then re-enter WHEEL_FOLLOW_MODE -- same STATE_DWELL
        exits STATE_WHEEL_FOLLOW as STOP does (see stop_and_rearm), so this
        needs the identical re-arm-and-clear-stale-errored treatment. Timeout
        is the requested dwell (or the firmware's WORKSTATION_WAIT_MS default
        of 2s if none given) plus a generous margin -- the firmware enforces
        the actual wait, this is just bounding how long we poll for its ack."""
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
        rearmed = self.enter_wheel_follow_mode()
        if rearmed:
            self.errored = False
            self.error_text = ""
        else:
            self.get_logger().error(
                f"{label}: did not re-ack WHEEL_FOLLOW_MODE after DWELL -- aborting route")
        return rearmed

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
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting route")
            return False

        self.get_logger().info(
            f"{leg_label}: driving from ({pose.x:.1f},{pose.y:.1f}) to "
            f"({target_x:.1f},{target_y:.1f}), cruise={self.args.cruise_rpm:.0f}RPM, "
            f"brake_lead={self.args.brake_lead_in:.2f}in")

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

            pose = self.fresh_pose()
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
                # hold-and-hope vision returns.
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: vision lost -- STOP sent, aborting route")
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
                rearmed = self.stop_and_rearm()
                if not rearmed:
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE after "
                        "stopping -- aborting route")
                return False

            if dist_to_target <= self.args.brake_lead_in:
                # Log BEFORE stop_and_rearm() -- confirmed 2026-07-28: logging
                # after made every prior "arrived" timestamp actually mark the
                # END of the stop/re-arm round trip, not the moment arrival
                # was detected. That misattributed two real ~3s
                # stop_and_rearm() stalls (repeatedly, at the same node10
                # transition) to a "vision gap" that was never there --
                # occlusion and vision throughput were both ruled out chasing
                # the wrong timestamp before this was caught.
                self.get_logger().info(
                    f"{leg_label}: brake fired ({dist_to_target:.2f}in short of "
                    "target)")
                rearmed = self.stop_and_rearm()
                if not rearmed:
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE after "
                        "stopping -- aborting route")
                    return False
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

        Sizes its ROTATE_REL from corrected_odom_yaw() (the robot's own
        onboard get_pose() yaw, local UART link, no camera round-trip --
        see _on_odom_pose()/resync_yaw_offset()), NOT fresh_pose() (camera/
        AprilTag yaw). Camera latency (the "yaw swinging 85-95deg while
        driving" the user described) makes vision yaw a genuinely bad
        "current heading" input for sizing a single relative-angle command
        -- an onboard reading taken the instant before the turn is far
        closer to the robot's TRUE heading at send-time. Vision remains the
        ABSOLUTE ground truth for the overall route (drive_leg() still
        steers off it, and resync_yaw_offset() re-anchors onboard yaw to
        it), just not the input to this one relative-angle calculation.

        HANG HISTORY -- read before ever changing this method's completion
        check: an EARLIER ROTATE_REL implementation (removed 2026-08-13,
        see git history if the old docstring is needed) hit multiple
        confirmed FULL FIRMWARE HANGS on 2026-07-30 (LED frozen, zero ROS
        traffic, power-cycle required). Root-caused at the FIRMWARE level
        (AGV_Factory_camera_correction.ino's ROTATE_REL handler comment) to
        a parse_message() ack-discard race, made likely by an alvik.brake()
        call immediately before alvik.rotate(). Fixed by (1) removing that
        brake() (the robot is already stationary between commands) and (2)
        abandoning is_target_reached() polling entirely for a timed
        millis() deadline (ROTATE_DEG_PER_SEC). Both fixes are already
        deployed in the firmware's current ROTATE_REL handler -- this
        method just calls it, same as rotate_test() already does.
        Independently re-stress-tested 2026-08-13 on Alvik6 (bench sketch,
        128 back-to-back rotate() calls incl. small angles specifically --
        the exact condition that hung before): 128/128 settled cleanly,
        zero hangs, zero is_on()-detected STM32 unresponsiveness. Do not
        reintroduce alvik.brake() before alvik.rotate() (in firmware) or
        is_target_reached()-style polling (here) without re-reading the
        firmware's ROTATE_REL handler comment first.

        Falls back to turn_to_heading() (the proven wheel-streaming taper)
        if no odom yet -- see corrected_odom_yaw()'s None cases -- rather
        than guessing or blocking; that keeps this safe to call even on the
        very first leg of a route, before any odom/vision resync has had a
        chance to happen.

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
        odom_yaw = self.corrected_odom_yaw()
        if odom_yaw is None:
            self.get_logger().info(
                f"{leg_label}: no corrected onboard yaw yet -- falling back "
                "to turn_to_heading() (wheel-streaming)")
            return self.turn_to_heading(target_heading_deg, leg_label)

        rel_deg = yaw_error_deg(target_heading_deg, odom_yaw)
        if abs(rel_deg) <= self.args.turn_tol_deg:
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_tol_deg:.1f} "
                f"deg of target heading {target_heading_deg:.0f} (odom "
                f"yaw={odom_yaw:+.1f}), skipping turn")
            return True

        self.get_logger().info(
            f"{leg_label}: turning from odom yaw {odom_yaw:+.1f} to "
            f"{target_heading_deg:.0f} deg via ROTATE_REL {rel_deg:+.2f}deg")
        if not self._rotate_rel_and_wait_or_abort(rel_deg, leg_label):
            return False

        # Camera-assist correction: sample vision once, right after the
        # odom-sized turn settles, and send ONE corrective ROTATE_REL if
        # still outside tolerance -- see this method's own docstring for
        # the real hardware numbers behind making this the default.
        self._spin_once(0.0)
        check_pose = self.fresh_pose()
        if check_pose is not None:
            remaining = yaw_error_deg(target_heading_deg, check_pose.yaw)
            if abs(remaining) > self.args.turn_tol_deg:
                self.get_logger().info(
                    f"{leg_label}: camera-assist correction, vision "
                    f"yaw={check_pose.yaw:+.1f} still {remaining:+.2f}deg "
                    "off -- sending corrective ROTATE_REL")
                if not self._rotate_rel_and_wait_or_abort(
                        remaining, f"{leg_label} (correction)"):
                    return False

        # Resync onboard yaw to vision now, while the robot is stationary
        # (right after a completed turn is exactly the safe window
        # resync_yaw_offset() calls for) -- keeps the NEXT turn's odom
        # reading from drifting further from ground truth. Not fatal if it
        # fails (e.g. vision briefly stale) -- the turn itself already
        # completed; this only affects the next one, which will retry the
        # resync when it starts.
        self._spin_once(0.0)  # pump one callback pass so odom/vision are current
        if not self.resync_yaw_offset():
            self.get_logger().info(
                f"{leg_label}: turn complete but yaw resync skipped "
                "(vision/odom not both fresh) -- next turn will retry")
        else:
            self.get_logger().info(
                f"{leg_label}: turn complete, yaw resynced "
                f"(offset={self.yaw_offset:+.2f})")
        return True

    def _rotate_rel_and_wait_or_abort(self, rel_deg: float, leg_label: str) -> bool:
        """turn_to_heading_rotate_rel()'s send+wait core, with THIS
        method's route-abort logging (distinct from _send_rotate_rel_and_
        wait()'s measurement-mode logging, which callers like
        yaw_stress_rotation() rely on saying "aborting" not "aborting
        route"). Same proven timeout formula as rotate_test()/
        _send_rotate_rel_and_wait() -- do not shrink without new timing
        data across a range of angles."""
        self.rotate_rel_done_seen = False
        self.errored = False
        self.error_text = ""
        self.send_cmd(f"ROTATE_REL {rel_deg:.2f}")
        timeout_sec = max(5.0, abs(rel_deg) * 0.15 + 3.0)
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            self._spin_once(0.1)
            if self.rotate_rel_done_seen:
                return True
            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} "
                    "during ROTATE_REL -- aborting route")
                return False
        self.get_logger().error(
            f"{leg_label}: no ROTATE_REL COMPLETE within "
            f"{timeout_sec:.1f}s -- aborting route")
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
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting route")
            return False
        error = yaw_error_deg(target_heading_deg, pose.yaw)
        if abs(error) <= self.args.turn_tol_deg:
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_tol_deg:.1f} deg "
                f"of target heading {target_heading_deg:.0f}, skipping turn")
            return True

        self.get_logger().info(
            f"{leg_label}: turning in place from {pose.yaw:+.1f} to "
            f"{target_heading_deg:.0f} deg")
        period = 1.0 / CONTROL_HZ
        settled_count = 0
        while rclpy.ok():
            loop_t0 = time.monotonic()
            self._spin_once(0.0)

            if self.errored:
                self.get_logger().error(
                    f"{leg_label}: robot reported {self.error_text} -- aborting route")
                return False

            pose = self.fresh_pose()
            if pose is None:
                # See the matching stale-vision handling in drive_leg() --
                # send_wheel(0.0, 0.0) is not a confirmed-safe stop here;
                # use the real STOP command and abort.
                self.send_cmd("STOP")
                self.get_logger().error(
                    f"{leg_label}: vision lost -- STOP sent, aborting route")
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
                settled_count += 1
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
                rearmed = self.stop_and_rearm()
                if not rearmed:
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE after "
                        "stopping -- aborting route")
                    return False
                return True
            settled_count = 0

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
                    self.send_cmd("STOP")
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
                    rearmed = self.stop_and_rearm()
                    if not rearmed:
                        self.get_logger().error(
                            f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE after "
                            "stopping -- aborting route")
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
        if not self.wait_for_fresh_vision(timeout_sec=5.0):
            self.get_logger().error(
                f"no fresh {self.robot}_vision_pose received -- is "
                "apriltag_localize.py --rosbridge running on the camera "
                "laptop and can it see this robot's tag?")
            return

        # Attempt an initial yaw resync (see resync_yaw_offset()) while the
        # robot is presumed stationary at mission start, so the FIRST turn
        # of the route can already use turn_to_heading_rotate_rel()'s fast
        # path instead of always paying for one wheel-streaming fallback
        # turn per mission. Short retry loop, not a hard requirement:
        # <robot>_pose (odom) may not have arrived yet even though vision
        # has -- turn_to_heading_rotate_rel() already handles yaw_offset
        # still being None by falling back safely, so failing here is not
        # fatal to the route.
        resync_t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - resync_t0 < 2.0:
            self._spin_once(0.1)
            if self.resync_yaw_offset():
                break

        self.get_logger().info(
            f"route: {' -> '.join(f'node{n}' for n, _, _ in route)}")

        if not self.enter_wheel_follow_mode():
            self.get_logger().error(
                f"robot did not ack WHEEL_FOLLOW_MODE "
                f"(last status: '{self.last_status}') -- aborting")
            return

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
                # steering) -- reuses the already bench-verified
                # turn_to_heading() rather than changing drive_leg() itself.
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
                    # No rotate_at/dwell just happened at this node: the turn
                    # toward the next leg can hand off DIRECTLY into that
                    # leg's drive_leg() (turn_then_drive_leg(), added
                    # 2026-07-31) -- no brake/settle/re-arm between them.
                    # After a rotate_at or dwell, the robot must actually be
                    # stationary at the requested heading/for the requested
                    # wait, so those cases keep the separate brake-and-settle
                    # turn_to_heading() + the next iteration's own drive_leg().
                    if n not in rotate_at and n not in dwell_at:
                        if not self.turn_then_drive_leg(
                                next_heading, nx, ny,
                                f"turn at node{n} toward node{next_n}"):
                            return
                        skip_drive = True
                    elif not self.turn_to_heading_rotate_rel(
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
                     "using camera position + yaw only (no onboard Alvik sensors).")
    ap.add_argument("--robot", default="Alvik3")
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
            if not node.wait_for_fresh_vision(timeout_sec=5.0):
                node.get_logger().error(
                    f"no fresh {args.robot}_vision_pose received -- is "
                    "apriltag_localize.py --rosbridge running on the camera "
                    "laptop and can it see this robot's tag?")
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
            if not node.wait_for_fresh_vision(timeout_sec=5.0):
                node.get_logger().error(
                    f"no fresh {args.robot}_vision_pose received -- is "
                    "apriltag_localize.py --rosbridge running on the camera "
                    "laptop and can it see this robot's tag?")
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
            if not node.wait_for_fresh_vision(timeout_sec=5.0):
                node.get_logger().error(
                    f"no fresh {args.robot}_vision_pose received -- is "
                    "apriltag_localize.py --rosbridge running on the camera "
                    "laptop and can it see this robot's tag?")
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
            if not node.wait_for_fresh_vision(timeout_sec=5.0):
                node.get_logger().error(
                    f"no fresh {args.robot}_vision_pose received -- is "
                    "apriltag_localize.py --rosbridge running on the camera "
                    "laptop and can it see this robot's tag?")
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

    if args.yaw_stress_test is not None:
        rclpy.init()
        node = CameraGridNavigator(args.robot, args)
        try:
            if not node.wait_for_fresh_vision(timeout_sec=5.0):
                node.get_logger().error(
                    f"no fresh {args.robot}_vision_pose received -- is "
                    "apriltag_localize.py --rosbridge running on the camera "
                    "laptop and can it see this robot's tag?")
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
