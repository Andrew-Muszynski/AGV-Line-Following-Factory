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

Turning between legs is unchanged: turn_to_heading() pivots in place (one
wheel +speed, one wheel -speed) until camera yaw reaches the next leg's
heading -- see that function for its own tuning notes (bench-verified
2026-07-27, turn-min/max-speed 10/70, turn-tol-deg 1.0).

Grid <-> world conversion mirrors apriltag_localize.py's world_to_grid()
and agv_grid_workstation_solver.html's node numbering (nodeNumber(r,c) =
(rows-1-r)*cols + c + 1, node 1 = bottom-left / depot end). Verified against
two bench points: node 1 -> world (13.5, 16.75); node 8 -> world (83.5,
16.75), matching a real run to within 0.1in. Workstation/entry bay offsets
remeasured on the physical table 2026-07-27 (bay 1, node114/node65) -- see
node_to_world()'s comment for the corrected values vs. the dispatch model's
assumed ones.

Run on the LINUX laptop (ROS 2 sourced, native rclpy), with the micro-ROS
agent up, the robot powered/green, running AGV_Factory_camera_correction.ino,
and apriltag_localize.py --rosbridge running on the camera laptop:

    python3 camera_grid_navigate.py --robot Alvik3 --route 1,8,16

Ctrl+C sends STOP and exits early.
"""
from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
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
# get reserved node IDs rather than fitting the numeric scheme: D1=-1
# (depot/home, robot always starts here facing south/yaw~0), DE1=-2 (depot
# entry, straight south of D1), 0=node0 (west of DE1, turns to face north
# onto node1's column). Measured on the physical table 2026-07-28 (Alvik1):
# D1=(19.7,10.3,yaw~0 south), DE1=(19.6,3.0,yaw~0 south), node0=(13.3,3.2,
# yaw varies -- robot turns in place here from west-facing to north-facing).
# String tokens "D1"/"DE1"/"0" in --route map to these, matching the exact
# names already used in agv_grid_workstation_solver.html's own route output
# (e.g. "D1-DE1-0-1-9-114-...") so a route can be copied over with minimal
# translation.
DEPOT_WORLD_IN = {-1: (19.7, 10.3), -2: (19.6, 3.0), 0: (13.3, 3.2)}
DEPOT_NODE_TOKENS = {"D1": -1, "DE1": -2, "0": 0}


def node_to_world(n: int, rows: int, cols: int) -> tuple[float, float]:
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
    Negative/zero n -> DEPOT_WORLD_IN (see comment above)."""
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


def parse_route_token(tok: str) -> int:
    """"9" -> 9, "D1"/"DE1"/"0" -> the matching DEPOT_NODE_TOKENS ID."""
    tok = tok.strip()
    if tok in DEPOT_NODE_TOKENS:
        return DEPOT_NODE_TOKENS[tok]
    return int(tok)


def parse_route(route_str: str, rows: int, cols: int) -> list[tuple[int, float, float]]:
    """"1,8,16" -> [(1, x1, y1), (8, x8, y8), (16, x16, y16)]. Also accepts
    "D1"/"DE1"/"0" depot tokens, e.g. "D1,DE1,0,1,9,114" (see DEPOT_NODE_TOKENS)."""
    nodes = [parse_route_token(tok) for tok in route_str.split(",") if tok.strip()]
    if len(nodes) < 2:
        raise ValueError("--route needs at least 2 nodes (a start and a destination)")
    return [(n, *node_to_world(n, rows, cols)) for n in nodes]


class CameraGridNavigator(Node):
    def __init__(self, robot: str, args: argparse.Namespace):
        super().__init__("camera_grid_navigate")
        self.robot = robot
        self.args = args

        qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(String, f"{robot}_cmd", qos_best_effort)
        self.wheel_pub = self.create_publisher(String, f"{robot}_wheel_cmd", qos_best_effort)
        self.create_subscription(
            String, f"{robot}_status", self._on_status, qos_status)
        self.create_subscription(
            String, f"{robot}_vision_pose", self._on_vision_pose, qos_best_effort)

        self.last_status = ""
        self.mode_ack_seen = False
        self.dwell_done_seen = False
        self.errored = False
        self.error_text = ""

        self.last_pose: Pose | None = None
        self._last_stale_warn_at = 0.0

    # -- status / vision callbacks (identical to camera_line_follow.py) ---
    def _on_status(self, msg: String) -> None:
        text = msg.data.strip()
        self.last_status = text
        if self.args.verbose:
            self.get_logger().info(f"  [status] {text}")
        if text.startswith("ERROR"):
            self.errored = True
            self.error_text = text
        elif text.startswith("BUSY WHEEL_FOLLOW_MODE"):
            self.mode_ack_seen = True
        elif text == "DWELL COMPLETE":
            self.dwell_done_seen = True

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

    # -- lifecycle ----------------------------------------------------
    def send_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def send_wheel(self, left_rpm: float, right_rpm: float) -> None:
        msg = String()
        msg.data = f"{left_rpm:.1f} {right_rpm:.1f}"
        self.wheel_pub.publish(msg)

    def spin_for(self, duration_sec: float) -> None:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh_pose(self) -> Pose | None:
        pose = self.last_pose
        if pose is None or time.monotonic() - pose.t > VISION_STALE_SEC:
            return None
        return pose

    def wait_for_fresh_vision(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.fresh_pose() is not None:
                return True
        return False

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
        effect rather than whenever the caller gets around to it."""
        self.mode_ack_seen = False
        self.send_cmd("WHEEL_FOLLOW_MODE")
        if self.args.verbose:
            self.get_logger().info("  [rearm] WHEEL_FOLLOW_MODE sent, polling for ack...")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
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
            rclpy.spin_once(self, timeout_sec=0.0)

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
                    rclpy.spin_once(self, timeout_sec=0.0)
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
            rclpy.spin_once(self, timeout_sec=0.0)

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
            rclpy.spin_once(self, timeout_sec=0.05)
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
                    rclpy.spin_once(self, timeout_sec=0.0)
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
            rclpy.spin_once(self, timeout_sec=0.1)
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
        kp_dist never converged on a reliably sharp stop. --brake-lead-in's
        default (3.7) is taken directly from a 7-leg --stop-test route run
        at --cruise-rpm=60 with this SAME heading-correction law: mean
        slide-after-brake 3.21in (range 3.04-3.42in across all 7 legs),
        every leg landing short of target (never overshooting, mean 0.39in
        short) -- see stop_test()/stop_test_route() for the measurement
        tool. If --cruise-rpm is changed from the bench-tested 60, re-measure
        via --stop-test before trusting the default lead-in at the new
        speed (stopping distance is not necessarily linear in RPM)."""
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
            rclpy.spin_once(self, timeout_sec=0.0)

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
    def turn_to_heading(self, target_heading_deg: float, leg_label: str) -> bool:
        """REWRITTEN 2026-07-28: constant --turn-rpm the whole turn, then a
        hard brake (stop_and_rearm()) once within --turn-brake-lead-deg of
        target -- replaces the old tapered decel-zone speed law entirely,
        same fix already applied to drive_leg() (see that function's
        docstring). The old law's own history is preserved below for
        context, but no longer describes this function's actual behavior.

        This rewrite was blocked for a while by a real vision-pipeline bug,
        not a turn-law bug: --turn-test measurements were showing 46-58deg
        of "overshoot" that turned out to be almost entirely due to
        TcpFrameSource.read() (apriltag_localize.py) serving backlogged/
        stale frames during sustained rotation, not real robot momentum
        (confirmed by settle-window sampling: yaw jumping several degrees
        between camera reads only ~0.7ms apart in wall-clock time --
        physically impossible at any bench-tested turn RPM). Fixing read()
        to always drain to the newest buffered frame (see that function's
        docstring) dropped measured overshoot to a consistent, speed-
        proportional ~12-30deg across 30/35/50 RPM trials -- a real,
        repeatable stopping-distance signal instead of latency noise.
        --turn-brake-lead-deg's default (17) is taken directly from a 5-turn
        --turn-test run at --turn-rpm=35 (chosen as a speed/overshoot
        middle ground vs. 30 and 50 RPM alternatives, all bench-tested):
        overshoot 14.4-16.2deg across turns #2-5 (mean ~15.4deg), so 17
        covers the observed max with a small margin. If --turn-rpm changes,
        RE-MEASURE with --turn-test -- stopping distance scaled roughly
        linearly with RPM across the tested range (30/35/50), but that
        shouldn't be assumed to hold outside it."""
        pose = self.fresh_pose()
        if pose is None:
            self.get_logger().error(f"{leg_label}: lost vision -- aborting route")
            return False
        error = yaw_error_deg(target_heading_deg, pose.yaw)
        if abs(error) <= self.args.turn_brake_lead_deg:
            self.get_logger().info(
                f"{leg_label}: already within {self.args.turn_brake_lead_deg:.1f} "
                f"deg of target heading {target_heading_deg:.0f}, skipping turn")
            return True

        self.get_logger().info(
            f"{leg_label}: turning in place from {pose.yaw:+.1f} to "
            f"{target_heading_deg:.0f} deg, turn_rpm={self.args.turn_rpm:.0f}, "
            f"brake_lead={self.args.turn_brake_lead_deg:.2f}deg")

        no_progress_timeout_sec = 5.0
        best_abs_error = math.inf
        t_progress0 = time.monotonic()

        period = 1.0 / CONTROL_HZ
        while rclpy.ok():
            loop_t0 = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.0)

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

            if abs_error <= self.args.turn_brake_lead_deg:
                self.get_logger().info(
                    f"{leg_label}: brake fired (yaw={pose.yaw:+.1f}, "
                    f"{abs_error:.2f}deg short of target)")
                rearmed = self.stop_and_rearm()
                if not rearmed:
                    self.get_logger().error(
                        f"{leg_label}: did not re-ack WHEEL_FOLLOW_MODE after "
                        "stopping -- aborting route")
                    return False
                return True

            # In-place pivot: matches the firmware's own updateTurn() pattern
            # (opposite wheel signs) -- positive error (matches yawError()'s
            # sign: need to increase yaw) turns left (right wheel +, left
            # wheel -), same convention confirmed for LEFT_UNTIL_COLOR
            # earlier this session.
            if error > 0.0:
                self.send_wheel(-self.args.turn_rpm, self.args.turn_rpm)
            else:
                self.send_wheel(self.args.turn_rpm, -self.args.turn_rpm)

            if self.args.verbose:
                self.get_logger().info(f"  yaw={pose.yaw:+6.1f} err={error:+6.1f}deg")

            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, period - elapsed))
        return False

    def run(self, route: list[tuple[int, float, float]],
            rotate_at: dict[int, float] | None = None,
            dwell_at: dict[int, int | None] | None = None) -> None:
        if not self.wait_for_fresh_vision(timeout_sec=5.0):
            self.get_logger().error(
                f"no fresh {self.robot}_vision_pose received -- is "
                "apriltag_localize.py --rosbridge running on the camera "
                "laptop and can it see this robot's tag?")
            return

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
            # driving starts toward route[1]).
            for i in range(1, len(route)):
                prev_n, px, py = route[i - 1]
                n, tx, ty = route[i]
                leg_label = f"leg {prev_n}->{n}"
                if not self.drive_leg(tx, ty, leg_label):
                    return
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
                if not self.turn_to_heading(
                        leg_heading, f"square up at node{n} to {leg_heading:.0f} deg"):
                    return
                # Explicit standalone rotation (--rotate-at), applied BEFORE
                # any automatic turn toward the next route leg, so a request
                # like "face 180 at the workstation" happens exactly once,
                # at arrival, regardless of whether more legs follow.
                if n in rotate_at:
                    if not self.turn_to_heading(
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
                    if not self.turn_to_heading(
                            next_heading, f"turn at node{n} toward node{next_n}"):
                        return
            self.get_logger().info("route complete.")
        except KeyboardInterrupt:
            pass
        finally:
            self.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
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
    # turn_to_heading() constant-turn-RPM + hard-brake law (REPLACED
    # 2026-07-28 -- see that function's docstring for the full history,
    # including the vision-pipeline bug that blocked this rewrite for a
    # while by making measured overshoot look far larger/noisier than it
    # really was). The prior tapered decel-zone law (turn-min-speed/
    # turn-decel-zone-deg/turn-creep-speed/turn-scale-deg/turn-max-speed/
    # turn-tol-deg/turn-settle-count) is gone -- same fix already applied to
    # drive_leg()'s kp_dist law: hold a constant turn RPM the whole turn and
    # brake hard at a measured lead angle instead of tapering speed down.
    ap.add_argument("--turn-rpm", type=float, default=35.0,
                     help="constant turn-in-place wheel speed, RPM, for the "
                          "whole turn (default 35 -- bench-tested "
                          "2026-07-28 as a speed/overshoot middle ground "
                          "against 30 and 50 RPM alternatives; see "
                          "--turn-brake-lead-deg for the matching measured "
                          "stopping distance).")
    ap.add_argument("--turn-brake-lead-deg", type=float, default=17.0,
                     help="fire a hard STOP once within this many degrees "
                          "of the target heading (default 17). Measured "
                          "2026-07-28 via a 5-turn --turn-test run at "
                          "--turn-rpm=35 with the vision-pipeline stale-"
                          "frame bug already fixed (TcpFrameSource.read()): "
                          "overshoot 14.4-16.2deg across turns #2-5 (mean "
                          "~15.4deg). 17 covers the observed max (16.2deg) "
                          "with a small margin. If --turn-rpm is changed "
                          "from 35, RE-MEASURE with --turn-test before "
                          "trusting this default -- stopping distance "
                          "scaled roughly linearly with RPM across the "
                          "tested 30/35/50 range, but that shouldn't be "
                          "assumed to hold outside it.")
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
    ap.add_argument("--brake-lead-in", type=float, default=3.7,
                     help="fire a hard STOP once within this many inches of "
                          "the target (default 3.7). Measured 2026-07-28 via "
                          "a 7-leg --stop-test run at --cruise-rpm=60 with "
                          "this same heading-correction law: mean "
                          "slide-after-brake 3.21in (range 3.04-3.42in), "
                          "every leg landing short of target (mean 0.39in "
                          "short, max overshoot 0in -- never overshot). "
                          "3.7 covers the observed max slide (3.42in) with "
                          "margin. If --cruise-rpm is changed from 60, "
                          "RE-MEASURE with --stop-test before trusting this "
                          "default -- stopping distance is not necessarily "
                          "linear in RPM.")
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
