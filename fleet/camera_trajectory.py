"""
camera_trajectory.py — shared vision-only trajectory-control primitives used
by camera_line_follow.py (single straight segment) and camera_grid_navigate.py
(multi-waypoint routes). Factored out so the two scripts can't drift apart on
something this fragile: the lateral/heading error sign convention was
independently verified against the firmware's followLine() wheel-speed
formula on 2026-07-22 (an initial version had the lateral sign backwards --
caught by manually tracing a drift case before running on hardware). Keeping
one copy of TargetLine/errors() means that verification only has to hold in
one place.

No ROS/rclpy dependencies here on purpose -- this module is pure math, usable
outside a Node for testing.
"""
from __future__ import annotations

import argparse
import math


def normalize_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def yaw_error_deg(target: float, current: float) -> float:
    """Shortest signed error target-current, wrapped to [-180, 180] --
    mirrors the firmware's yawError() so the two systems agree on sign."""
    return normalize_deg(target - current)


class Pose:
    __slots__ = ("x", "y", "yaw", "t")

    def __init__(self, x: float, y: float, yaw: float, t: float):
        self.x, self.y, self.yaw, self.t = x, y, yaw, t


class TargetLine:
    """A straight trajectory: origin point + unit direction vector.
    Everything is in the same world-frame inches apriltag_localize.py
    publishes (x_in, y_in, yaw_deg)."""

    def __init__(self, origin_x: float, origin_y: float, heading_deg: float):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.heading_deg = heading_deg
        rad = math.radians(heading_deg)
        # This direction convention is anchored to TWO independently verified
        # points, not extrapolated from one (an earlier version used
        # dx=sin,dy=cos from a single yaw=90 data point and got yaw=180
        # backwards -- caught before running camera_grid_navigate.py, which
        # is the first thing that ever exercised a heading other than 90):
        #   yaw=90  -> facing +x  (bench 2026-07-22, node1->node8 run, twice)
        #   yaw=180 -> facing +y  (bench 2026-07-22, confirmed: turning LEFT
        #              90 deg from yaw=90 -- right wheel +, left wheel -,
        #              yaw increases per the firmware's LEFT_UNTIL_COLOR
        #              convention -- faces the robot into the grid/+y)
        # dx=sin(yaw), dy=-cos(yaw) fits both exactly and traces a CCW
        # rotation as yaw increases (+x -> +y -> -x -> -y), consistent with
        # "positive yaw = CCW" (also independently confirmed this session).
        # This is NOT the raw camera atan2(dy,dx) convention in
        # apriltag_localize.py's tag_world_pose() -- it's this robot's own
        # reported yaw_deg semantics. If ported to a different table/robot
        # calibration, re-verify against a real run before trusting it.
        self.dx = math.sin(rad)
        self.dy = -math.cos(rad)

    def errors(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """(lateral_error_in, heading_error_deg, distance_along_line_in).
        Sign convention verified against the firmware's followLine(): its
        `error` there is NEGATIVE when the robot has drifted to its own LEFT
        of the tape. lateral_error here is defined the same way -- negative
        means drifted left, positive means drifted right -- specifically so
        it can combine with heading_err (which already matches yawError()'s
        sign) using the exact same left=BASE-correction / right=BASE+
        correction formula followLine() uses, with no relative sign flip
        between the two error terms."""
        rx = x - self.origin_x
        ry = y - self.origin_y
        distance_along = rx * self.dx + ry * self.dy
        lateral = rx * self.dy - ry * self.dx
        heading_err = yaw_error_deg(self.heading_deg, yaw)
        return lateral, heading_err, distance_along


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self._integral = 0.0
        self._prev_error = None
        self._prev_t = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None
        self._prev_t = None

    def update(self, error: float, now: float) -> float:
        if self._prev_t is None:
            self._prev_error = error
            self._prev_t = now
            return self.kp * error
        dt = max(now - self._prev_t, 1e-3)
        self._integral = max(-self.i_limit, min(self.i_limit,
                                                  self._integral + error * dt))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._prev_t = now
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class TrajectoryController:
    """Combines a lateral-offset PID and a heading-error PID into one
    wheel-speed correction -- similar in shape to how the firmware's
    followLine() turns a single centroid-offset error into a correction, just
    with two error terms (position AND heading) instead of one."""

    def __init__(self, args: argparse.Namespace):
        self.lateral_pid = PID(args.kp_lateral, args.ki_lateral,
                                args.kd_lateral, args.i_limit_lateral)
        self.heading_pid = PID(args.kp_heading, args.ki_heading,
                                args.kd_heading, args.i_limit_heading)
        self.max_correction = args.max_correction

    def reset(self) -> None:
        self.lateral_pid.reset()
        self.heading_pid.reset()

    def update(self, lateral_err: float, heading_err: float, now: float) -> float:
        lateral_term = self.lateral_pid.update(lateral_err, now)
        heading_term = self.heading_pid.update(heading_err, now)
        correction = lateral_term + heading_term
        return max(-self.max_correction, min(self.max_correction, correction))


def add_trajectory_args(ap: argparse.ArgumentParser) -> None:
    """Shared PID/speed flags for any script built on this module."""
    ap.add_argument("--base-speed", type=float, default=40.0,
                     help="wheel RPM when perfectly on-trajectory (default 40)")
    ap.add_argument("--kp-lateral", type=float, default=3.0,
                     help="proportional gain on lateral offset (in)")
    ap.add_argument("--ki-lateral", type=float, default=0.0)
    ap.add_argument("--kd-lateral", type=float, default=0.5)
    ap.add_argument("--i-limit-lateral", type=float, default=10.0)
    ap.add_argument("--kp-heading", type=float, default=0.8,
                     help="proportional gain on heading error (deg)")
    ap.add_argument("--ki-heading", type=float, default=0.0)
    ap.add_argument("--kd-heading", type=float, default=0.1)
    ap.add_argument("--i-limit-heading", type=float, default=20.0)
    ap.add_argument("--max-correction", type=float, default=25.0,
                     help="clamp on the combined +/- wheel-speed correction, RPM")


FIRMWARE_MAX_RPM = 70.0  # WHEEL_FOLLOW_MAX_RPM in AGV_Factory_camera_correction.ino


def check_speed_budget(ap: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """WHEEL_FOLLOW_MAX_RPM is a last-resort backstop, not meant to be hit in
    normal operation -- if base_speed + max_correction can reach it, a
    full-authority correction clips asymmetrically (one wheel capped, the
    other not), distorting the differential instead of just capping top
    speed. Fail loudly here rather than silently clip on the robot mid-run."""
    worst_case = args.base_speed + args.max_correction
    if worst_case > FIRMWARE_MAX_RPM:
        ap.error(
            f"--base-speed ({args.base_speed:.1f}) + --max-correction "
            f"({args.max_correction:.1f}) = {worst_case:.1f} RPM exceeds the "
            f"firmware's WHEEL_FOLLOW_MAX_RPM ({FIRMWARE_MAX_RPM:.1f}) -- a "
            "full-authority correction would clip asymmetrically. Lower one "
            "of them.")
