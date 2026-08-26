"""Pure helpers for mission-selectable robot localization.

The fleet UI and supervisor expose three control-localization modes:

``encoder``
    Transform the Alvik's onboard odometry into the table/world frame and
    never use camera poses for motion control.
``camera_assist``
    Use transformed odometry at control-loop rate, with bounded camera
    corrections applied to position and yaw as fresh AprilTag poses arrive.
``camera``
    Use AprilTag position and yaw directly; onboard odometry is not read by
    the controller.

This module deliberately has no ROS dependency so the frame math and mode
contract can be unit-tested on the Windows development machine.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


LOCALIZATION_MODES = ("encoder", "camera_assist", "camera")


def normalize_localization_mode(value: str | None) -> str:
    """Return the canonical localization mode or raise ``ValueError``."""
    text = (value or "camera_assist").strip().lower().replace("-", "_")
    aliases = {
        "encoder_only": "encoder",
        "encoders": "encoder",
        "fused": "camera_assist",
        "encoder_camera": "camera_assist",
        "encoder_camera_assist": "camera_assist",
        "camera_only": "camera",
        "vision": "camera",
        "vision_only": "camera",
    }
    text = aliases.get(text, text)
    if text not in LOCALIZATION_MODES:
        raise ValueError(
            f"unknown localization mode {value!r}; expected one of "
            f"{', '.join(LOCALIZATION_MODES)}")
    return text


def mode_requires_camera(mode: str) -> bool:
    return normalize_localization_mode(mode) in ("camera_assist", "camera")


def mode_requires_odom(mode: str) -> bool:
    return normalize_localization_mode(mode) in ("encoder", "camera_assist")


@dataclass(frozen=True)
class OdomAnchor:
    """Rigid transform tying one raw odometry pose to a table-world pose.

    Raw Alvik x is forward and raw y is left in the reset-pose frame. The
    table yaw convention is 0=south/-world-y, 90=east/+world-x. Therefore a
    raw odometry frame reset while the robot faces world yaw 0 is rotated
    -90 degrees relative to the table x/y axes.
    """

    raw_x_in: float
    raw_y_in: float
    raw_yaw_deg: float
    world_x_in: float
    world_y_in: float
    world_yaw_deg: float


def normalize_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def odom_to_world(raw_x_in: float, raw_y_in: float, raw_yaw_deg: float,
                  anchor: OdomAnchor) -> tuple[float, float, float]:
    """Transform an Alvik odometry pose into table-world x/y/yaw."""
    dx = raw_x_in - anchor.raw_x_in
    dy = raw_y_in - anchor.raw_y_in
    # Raw yaw is measured from raw +x. Table yaw is measured from -world-y.
    rotation_deg = (
        anchor.world_yaw_deg - anchor.raw_yaw_deg - 90.0)
    rotation = math.radians(rotation_deg)
    c = math.cos(rotation)
    s = math.sin(rotation)
    world_x = anchor.world_x_in + c * dx - s * dy
    world_y = anchor.world_y_in + s * dx + c * dy
    world_yaw = normalize_deg(
        raw_yaw_deg + anchor.world_yaw_deg - anchor.raw_yaw_deg)
    return world_x, world_y, world_yaw


def blended_correction(current: float, error: float, alpha: float) -> float:
    """One bounded complementary-filter correction update."""
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be within [0, 1]")
    return current + alpha * error

