#!/usr/bin/env python3
"""Second half of the isaac_ros_apriltag bridge (see memory:
isaac_ros_apriltag_gpu): subscribes to isaac_ros_apriltag's
AprilTagDetectionArray and republishes robot/table-frame poses to rosbridge
using apriltag_localize.py's EXISTING, UNCHANGED homography/world-coordinate
machinery (TableCalibration, tag_world_pose, RosbridgePublisher).

Key design point (confirmed by reading apriltag_localize.py directly, not
assumed): tag_world_pose(H, det) and TableCalibration.update(detections)
only ever touch det.tag_id, det.center (1 pixel point), and det.corners[:2]
(2 pixel points) -- pupil_apriltags' own 3D pose estimate is NEVER used, only
raw 2D pixel corners fed through a hand-fit homography. This means a small
adapter object exposing those three attributes, built from
AprilTagDetectionArray's per-tag family/id/center/corners fields (which ARE
pixel-space per the message's own field comments), is a drop-in replacement
for a pupil_apriltags detection -- none of the homography/calibration/
rosbridge-publish code needed to change at all.

Run this ALONGSIDE isaac_ros_image_publisher.py + the RectifyNode/
AprilTagNode composable-node container (see isaac_ros_apriltag_launch.py),
NOT instead of apriltag_localize.py running at the same time -- they'd both
try to hold the camera_bridge_windows.py TCP connection.

Usage:
    python3 fleet/isaac_ros_apriltag_bridge.py --rosbridge 192.168.0.212:9090
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# tools/isaac/<file> -> repo root is three levels up (was two when these
# lived in fleet/). Getting this wrong imports nothing and fails at the
# apriltag_localize import below, not here.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apriltag_localize import (  # noqa: E402
    DEFAULT_REF_TAG_SIZE_IN,
    REF_TAG_WORLD,
    ROBOT_NAMES,
    RosbridgePublisher,
    TableCalibration,
    tag_world_pose,
)


class _AdaptedDetection:
    """Mimics the subset of pupil_apriltags.Detection's interface that
    tag_world_pose()/TableCalibration.update() actually read: .tag_id,
    .center (1,2 or (2,)-shaped), .corners ((4,2))."""
    __slots__ = ("tag_id", "center", "corners")

    def __init__(self, tag_id: int, center: np.ndarray, corners: np.ndarray):
        self.tag_id = tag_id
        self.center = center
        self.corners = corners


def _to_adapted(msg: AprilTagDetectionArray) -> list[_AdaptedDetection]:
    out = []
    for det in msg.detections:
        try:
            tag_id = int(det.id)
        except (TypeError, ValueError):
            continue
        center = np.array([det.center.x, det.center.y], dtype=float)
        corners = np.array([[p.x, p.y] for p in det.corners], dtype=float)
        out.append(_AdaptedDetection(tag_id, center, corners))
    return out


class AprilTagBridgeNode(Node):
    def __init__(self, publisher: RosbridgePublisher, calib: TableCalibration,
                 publish_rate: float):
        super().__init__("isaac_ros_apriltag_bridge")
        self._publisher = publisher
        self._calib = calib
        self._min_period = 1.0 / publish_rate if publish_rate > 0 else 0.0
        self._last_publish = 0.0
        self._last_print = 0.0
        # detection-rate / frame-rate instrumentation
        self._stats_frames = 0
        self._stats_seen: dict = {}
        self._stats_since = time.monotonic()
        self._stats_period = 5.0
        # (x0, y0) to add back onto every detection's pixel coords once
        # isaac_ros_image_publisher.py's auto-crop takes effect -- see that
        # file's module docstring. Detections arrive relative to the
        # CROPPED frame's origin once cropping starts, but self._calib's
        # homography was (and stays) fit in FULL-frame pixel coordinates,
        # so every detection needs this offset added back before it's used
        # for anything, calibration fit included. (0, 0) until the crop
        # offset arrives = a no-op, correct for the pre-crop period too.
        self._crop_offset = (0.0, 0.0)

        # AprilTagNode's /tag_detections publisher is RELIABLE (confirmed via
        # `ros2 topic info --verbose` -- same QoS-incompatibility class of
        # bug as isaac_ros_image_publisher.py's image/camera_info topics had).
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(
            AprilTagDetectionArray, "tag_detections", self._on_detections, qos)
        self._crop_offset_sub = self.create_subscription(
            Point32, "crop_offset", self._on_crop_offset, qos)
        self.get_logger().info("Subscribed to /tag_detections, waiting for "
                                "isaac_ros_apriltag to publish...")

    def _report_rates(self) -> None:
        """Per-tag detection rate and frame rate, the two numbers that decide
        whether this pipeline is usable.

        The 2026-08-05 archive had to establish these by a separate hand-run
        "final decisive test" -- CPU detected every tag in 955/955 frames while
        cuAprilTags managed ~2% on two robot tags and 13-43% on two corner
        tags. Building the measurement in means the A/B is just "run both and
        read the line", instead of a bespoke experiment each time.

        Compare directly against apriltag_localize.py's own --stream output on
        the same physical scene, minutes apart, with nothing else changed.
        """
        now = time.monotonic()
        if now - self._stats_since < self._stats_period:
            return
        frames = self._stats_frames
        if frames:
            per = "  ".join(
                f"{ROBOT_NAMES.get(t, t)}:{100.0 * c / frames:.0f}%"
                for t, c in sorted(self._stats_seen.items()))
            self.get_logger().info(
                f"[rate] {frames / (now - self._stats_since):.1f} fps over "
                f"{frames} frames | per-tag detection: {per or '(none)'}")
        self._stats_frames = 0
        self._stats_seen = {}
        self._stats_since = now

    def _on_crop_offset(self, msg: Point32) -> None:
        self._crop_offset = (float(msg.x), float(msg.y))
        self.get_logger().info(
            f"crop offset received: ({msg.x:.0f}, {msg.y:.0f}) -- "
            "correcting all further detections back to full-frame coords")

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        detections = _to_adapted(msg)
        self._stats_frames += 1
        for _d in detections:
            self._stats_seen[_d.tag_id] = self._stats_seen.get(_d.tag_id, 0) + 1
        self._report_rates()
        if self._crop_offset != (0.0, 0.0):
            dx, dy = self._crop_offset
            offset = np.array([dx, dy], dtype=float)
            for det in detections:
                det.center = det.center + offset
                det.corners = det.corners + offset
        now = time.monotonic()

        self._calib.update(detections)

        if self._calib.H is None:
            if detections and now - self._last_print >= 2.0:
                self.get_logger().info(
                    f"seen tags {sorted(d.tag_id for d in detections)}; "
                    "waiting for >=2 corner tags (20-23) to calibrate")
                self._last_print = now
            return

        poses: dict[int, tuple[float, float, float]] = {}
        for det in detections:
            if det.tag_id not in REF_TAG_WORLD:
                poses[det.tag_id] = tag_world_pose(self._calib.H, det)

        if poses and now - self._last_publish >= self._min_period:
            self._publisher.publish(poses, self._calib, now)
            self._last_publish = now
            if now - self._last_print >= 2.0:
                self.get_logger().info("; ".join(
                    f"{ROBOT_NAMES.get(tid, f'id {tid}')}: "
                    f"x={x:.1f} y={y:.1f} yaw={yaw:+.1f}"
                    for tid, (x, y, yaw) in sorted(poses.items())))
                self._last_print = now


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--rosbridge", required=True, metavar="HOST[:PORT]",
                     help="rosbridge websocket host[:port] to publish poses "
                          "to, e.g. 192.168.0.212:9090 (required -- this "
                          "node's whole purpose is publishing poses)")
    ap.add_argument("--tag-size", type=float, default=None,
                     help=f"ref tag black-square edge length, inches "
                          f"(default {DEFAULT_REF_TAG_SIZE_IN}, the measured "
                          "value -- see apriltag_localize.py)")
    ap.add_argument("--freeze-calib", type=int, default=0, metavar="N",
                     help="average N ref-tag observations then freeze the "
                          "homography instead of re-fitting every frame "
                          "(default 0 = never freeze, matches "
                          "apriltag_localize.py's default)")
    ap.add_argument("--publish-batch", action="store_true",
                     help="publish one /vision_poses_batch message instead "
                          "of per-robot /<Name>_vision_pose topics")
    ap.add_argument("--publish-rate", type=float, default=30.0,
                     help="max rosbridge publish rate, Hz (default 30)")
    args = ap.parse_args()

    target = args.rosbridge.removeprefix("ws://").rstrip("/")
    host, _, port = target.partition(":")
    publisher = RosbridgePublisher(host, int(port) if port else 9090,
                                    batch=args.publish_batch)

    tag_size = args.tag_size if args.tag_size is not None else DEFAULT_REF_TAG_SIZE_IN
    calib = TableCalibration(tag_size, freeze_after_n=args.freeze_calib)

    rclpy.init()
    node = AprilTagBridgeNode(publisher, calib, args.publish_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
