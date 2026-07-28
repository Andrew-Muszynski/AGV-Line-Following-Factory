#!/usr/bin/env python3
"""
camera_line_follow.py — hold ONE Alvik on a straight trajectory using camera
(AprilTag) position + yaw only. No onboard Alvik sensor (IR line sensors,
color sensor) is used anywhere in this loop -- lighting in this lab makes
those inconsistent, so both the correction and the stop condition come
entirely from the overhead camera.

Companion to AGV_Factory_camera_correction/AGV_Factory_camera_correction.ino
(a fork of AGV_Factory_color_pose.ino — that file is untouched). The firmware
adds WHEEL_FOLLOW_MODE: a no-ack streaming topic <robot>_wheel_cmd that this
script publishes "<left_rpm> <right_rpm>" to at CONTROL_HZ, plus a watchdog
that brakes the robot if fresh setpoints stop arriving.

Shared trajectory math (TargetLine, PID, TrajectoryController) lives in
camera_trajectory.py — see that file for the lateral/heading sign-convention
notes. For multi-waypoint routes with turns, see camera_grid_navigate.py,
which is built on the same primitives.

Run on the LINUX laptop (ROS 2 sourced, native rclpy — no roslibpy needed),
with the micro-ROS agent up, the robot powered/green, running
AGV_Factory_camera_correction.ino, and apriltag_localize.py --rosbridge
already running on the camera (Windows) laptop and forwarding
<robot>_vision_pose over the rosbridge both machines already share:

    python3 camera_line_follow.py --robot Alvik3 --stop-x-in 90.0

Ctrl+C sends STOP and exits early. Tune --kp-lateral/--kp-heading/etc. on the
bench; defaults are deliberately conservative as a starting point, not a
tuned result. Bench-validated 2026-07-22: node 1 -> node 8 (world x 13->91),
y drift ~0.3in over 78in, heading held within ~+-2 deg.
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

from camera_trajectory import (
    Pose, TargetLine, TrajectoryController, add_trajectory_args,
    check_speed_budget, normalize_deg,
)

CONTROL_HZ = 30.0            # wheel_cmd publish rate
VISION_STALE_SEC = 0.5       # no fresh vision pose within this -> brake, don't guess
CAPTURE_SETTLE_SEC = 1.0     # wait this long for vision to be flowing before capturing
POSE_JUMP_REJECT_IN = 6.0    # ignore a single-frame position reading this far from
                              # the last accepted one (misdetect/ID swap/reflection),
                              # not a real one-frame position change
YAW_JUMP_REJECT_DEG = 45.0   # same idea, for yaw


class CameraLineFollower(Node):
    def __init__(self, robot: str, args: argparse.Namespace):
        super().__init__("camera_line_follow")
        self.robot = robot
        self.args = args
        self.controller = TrajectoryController(args)

        qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(String, f"{robot}_cmd", qos_best_effort)
        self.wheel_pub = self.create_publisher(String, f"{robot}_wheel_cmd", qos_best_effort)
        self.create_subscription(
            String, f"{robot}_status", self._on_status, qos_status)
        self.create_subscription(
            String, f"{robot}_vision_pose", self._on_vision_pose, qos_best_effort)

        self.last_status = ""
        self.mode_ack_seen = False   # BUSY WHEEL_FOLLOW_MODE seen
        self.errored = False
        self.error_text = ""

        self.last_pose: Pose | None = None
        self.target_line: TargetLine | None = None
        self._last_stale_warn_at = 0.0

    # -- status / vision callbacks -------------------------------------
    def _on_status(self, msg: String) -> None:
        text = msg.data.strip()
        self.last_status = text
        if text.startswith("ERROR"):
            self.errored = True
            self.error_text = text
        elif text.startswith("BUSY WHEEL_FOLLOW_MODE"):
            self.mode_ack_seen = True

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
            pos_jump = math.hypot(x - prev.x, y - prev.y)
            yaw_jump = abs(normalize_deg(yaw - prev.yaw))
            if pos_jump > POSE_JUMP_REJECT_IN or yaw_jump > YAW_JUMP_REJECT_DEG:
                # Single-frame outlier (misdetect, wrong tag, reflection) --
                # a real change this fast between two ~30-60 Hz frames would
                # require unrealistic velocity. Drop it rather than feed a
                # spike into the controller.
                return
        self.last_pose = Pose(x, y, yaw, now)

    # -- lifecycle --------------------------------------------------------
    def send_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def send_wheel(self, left_rpm: float, right_rpm: float) -> None:
        msg = String()
        msg.data = f"{left_rpm:.1f} {right_rpm:.1f}"
        self.wheel_pub.publish(msg)

    def wait_for_fresh_vision(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_pose is not None and time.monotonic() - self.last_pose.t < VISION_STALE_SEC:
                return True
        return False

    def enter_wheel_follow_mode(self, timeout_sec: float = 3.0) -> bool:
        self.mode_ack_seen = False
        self.send_cmd("WHEEL_FOLLOW_MODE")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mode_ack_seen:
                return True
            if self.errored:
                return False
        return False

    def capture_target_line(self) -> TargetLine | None:
        """Settle briefly then take the current vision pose as the
        trajectory's origin + direction -- the robot is assumed already
        aligned by eye on the tape."""
        self.get_logger().info(
            f"capturing target trajectory in {CAPTURE_SETTLE_SEC:.0f}s "
            "(make sure the robot is aligned on the tape now)...")
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < CAPTURE_SETTLE_SEC:
            rclpy.spin_once(self, timeout_sec=0.05)
        pose = self.last_pose
        if pose is None or time.monotonic() - pose.t > VISION_STALE_SEC:
            return None
        return TargetLine(pose.x, pose.y, pose.yaw)

    def run(self) -> None:
        if not self.wait_for_fresh_vision(timeout_sec=5.0):
            self.get_logger().error(
                f"no fresh {self.robot}_vision_pose received -- is "
                "apriltag_localize.py --rosbridge running on the camera "
                "laptop and can it see this robot's tag?")
            return

        line = self.capture_target_line()
        if line is None:
            self.get_logger().error("lost vision during capture -- aborting")
            return
        self.target_line = line
        self.get_logger().info(
            f"target trajectory captured: origin=({line.origin_x:+.1f},"
            f"{line.origin_y:+.1f}) heading={line.heading_deg:+.1f} deg")

        if not self.enter_wheel_follow_mode():
            self.get_logger().error(
                f"robot did not ack WHEEL_FOLLOW_MODE "
                f"(last status: '{self.last_status}') -- aborting")
            return
        self.get_logger().info(
            f"{self.robot} in WHEEL_FOLLOW_MODE, base speed "
            f"{self.args.base_speed:.0f} rpm, stopping at x={self.args.stop_x_in:.1f}in "
            f"(Ctrl+C to stop early)")

        period = 1.0 / CONTROL_HZ
        try:
            while rclpy.ok():
                loop_t0 = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.0)

                if self.errored:
                    self.get_logger().error(
                        f"robot reported {self.error_text} -- stopping")
                    break

                pose = self.last_pose
                vision_age = (time.monotonic() - pose.t) if pose else math.inf
                if pose is None or vision_age > VISION_STALE_SEC:
                    # Stale vision: don't guess a correction from an old
                    # reading. The firmware's own WHEEL_CMD_TIMEOUT_MS
                    # watchdog also brakes if we stop publishing entirely,
                    # but braking here (rather than just skipping the send)
                    # reacts sooner, before that timeout elapses.
                    self.send_wheel(0.0, 0.0)
                    now_t = time.monotonic()
                    if now_t - self._last_stale_warn_at > 1.0:
                        self.get_logger().warning(
                            f"vision stale ({vision_age:.2f}s) -- holding still")
                        self._last_stale_warn_at = now_t
                    elapsed = time.monotonic() - loop_t0
                    time.sleep(max(0.0, period - elapsed))
                    continue

                lateral_err, heading_err, distance_along = line.errors(
                    pose.x, pose.y, pose.yaw)

                if pose.x >= self.args.stop_x_in:
                    self.get_logger().info(
                        f"reached stop position (x={pose.x:.1f}in >= "
                        f"{self.args.stop_x_in:.1f}in) -- stopping")
                    break

                correction = self.controller.update(
                    lateral_err, heading_err, time.monotonic())
                # Matches followLine()'s sign convention: a positive
                # correction (need to turn toward increasing yaw / back
                # toward the line) speeds the right wheel and slows the
                # left, same as the firmware's own error*KP term.
                left = self.args.base_speed - correction
                right = self.args.base_speed + correction
                self.send_wheel(left, right)

                if self.args.verbose:
                    self.get_logger().info(
                        f"x={pose.x:6.1f} y={pose.y:6.1f} yaw={pose.yaw:+6.1f}  "
                        f"lat_err={lateral_err:+5.2f}in  hdg_err={heading_err:+5.1f}deg  "
                        f"dist={distance_along:5.1f}in  corr={correction:+5.1f}")

                elapsed = time.monotonic() - loop_t0
                time.sleep(max(0.0, period - elapsed))
        except KeyboardInterrupt:
            pass
        finally:
            self.send_cmd("STOP")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info("STOP sent, exiting.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Hold one Alvik on a straight trajectory using camera "
                     "position + yaw only (no onboard Alvik sensors).")
    ap.add_argument("--robot", default="Alvik3")
    ap.add_argument("--stop-x-in", type=float, required=True,
                     help="stop once the robot's world-frame x (inches, from "
                          "apriltag_localize.py) reaches this value")
    add_trajectory_args(ap)
    ap.add_argument("--verbose", action="store_true",
                     help="print position/error/correction every control tick")
    args = ap.parse_args()
    check_speed_budget(ap, args)

    rclpy.init()
    node = CameraLineFollower(args.robot, args)
    try:
        node.run()
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
