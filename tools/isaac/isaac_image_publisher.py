#!/usr/bin/env python3
"""Publishes camera_bridge_windows.py's TCP JPEG frames as ROS2
sensor_msgs/Image + sensor_msgs/CameraInfo, so isaac_ros_image_proc's
RectifyNode -> isaac_ros_apriltag's AprilTagNode can consume them.

This is the first of two new pieces in the isaac_ros_apriltag bridge (see
memory: isaac_ros_apriltag_gpu) -- the second is
isaac_ros_apriltag_bridge.py, which subscribes to AprilTagDetectionArray
and republishes poses via the existing apriltag_localize.py machinery
(TableCalibration/tag_world_pose/RosbridgePublisher, all UNCHANGED).

Reuses camera_grid_navigate.py-style TCP framing (matches
apriltag_localize.py's TcpFrameSource exactly) so this can run alongside
or instead of apriltag_localize.py against the same camera_bridge_windows.py
process (only one TCP client at a time, though -- stop apriltag_localize.py
before running this, they'd otherwise fight over the same port).

Camera intrinsics come from fleet/camera_intrinsics.json (produced by
fleet/camera_calibrate.py's --solve mode, cv2.fisheye.calibrate() --
distortion_model is set to "equidistant", confirmed 2026-08 by reading
isaac_ros_image_proc's actual rectify_node.cpp source: it checks
distortion_model=="equidistant" and calls cv::fisheye::initUndistortRectifyMap
in that case, NOT the standard "plumb_bob" path -- using the wrong string
here would silently rectify with the wrong distortion model).

Fixed crop (added 2026-08-05, isaac_ros_apriltag_gpu throughput
investigation): after ruling out decode cost, socket buffer size, and
receiver scheduling latency as the ~28-30Hz ceiling's cause, found that
WSL2 mirrored-networking's 127.0.0.1 path routes through a real virtual NIC
(loopback0, MTU 1500) rather than a true in-kernel loopback -- every
~180KB full-frame JPEG was fragmenting into ~120 real packets, and that
per-packet cost was the actual bottleneck. Since only the table area
(bounded by the 4 corner AprilTags) is ever used for localization, cropping
to the table's bounding box cuts both encode work and bytes-on-the-wire.

An EARLIER version of this feature (--auto-crop, see below, kept only for
history) discovered the crop rect live via /tag_detections and switched
camera_bridge_windows.py's frame size mid-stream. That CRASHED AprilTagNode
on real hardware 2026-08-05: cuAprilTags' internal GPU buffer is
pitch-sized for whatever resolution it saw FIRST and does not support a
runtime resolution change (cudaErrorInvalidPitchValue, plus garbage tag
IDs before it errors -- actively dangerous, not just ineffective). The fix
is --crop-rect-file: load a crop rect that was MEASURED ONCE, OFFLINE,
BEFORE this node (and AprilTagNode) ever start, via
fleet/measure_crop_rect.py (CPU/pupil_apriltags, no Isaac ROS involved --
that tool doubles as re-validation that the crop math itself is correct,
already confirmed via apriltag_localize.py's --test-local-crop). With
--crop-rect-file set, this node sends ONE crop command to
camera_bridge_windows.py immediately on startup (before AprilTagNode ever
sees a frame) and CameraInfo is built pre-cropped from the start -- so
AprilTagNode only ever sees one consistent resolution for its whole
process lifetime, no runtime resize, no crash. Since the camera is
physically fixed, the saved crop rect is reusable across every future
pipeline launch until the camera moves (re-run measure_crop_rect.py if it
does).

Usage:
    python3 fleet/measure_crop_rect.py --port 8765 \\
        --out fleet/crop_rect.json   # one-time, do this FIRST
    python3 fleet/isaac_ros_image_publisher.py --port 8765 \\
        --intrinsics fleet/camera_intrinsics.json \\
        --crop-rect-file fleet/crop_rect.json
"""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# tools/isaac/<file> -> repo root is three levels up (was two when
# these lived in fleet/).
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apriltag_localize import (  # noqa: E402
    DEFAULT_REF_TAG_SIZE_IN,
    REF_TAG_WORLD,
    TableCalibration,
    TcpFrameSource,
    compute_crop_rect,
)


class _AdaptedDetection:
    """Mimics pupil_apriltags.Detection's .tag_id/.center/.corners -- same
    adapter shape as isaac_ros_apriltag_bridge.py's own, duplicated here
    (small/self-contained) rather than shared, since this node's use of it
    is one-shot calibration-only, not the main per-frame hot path."""
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


class RollingStats:
    """Minimal per-stage timing collector for --diag -- median/p95/max over
    samples since the last reset(), in milliseconds."""

    def __init__(self) -> None:
        self._samples: list[float] = []

    @property
    def count(self) -> int:
        return len(self._samples)

    def add(self, seconds: float) -> None:
        self._samples.append(seconds)

    def reset(self) -> None:
        self._samples.clear()

    def summary(self) -> str:
        n = len(self._samples)
        if n == 0:
            return "n=0"
        s = sorted(self._samples)
        median = statistics.median(s)
        p95 = s[min(int(0.95 * n), n - 1)]
        return f"median={median*1000:.1f}ms p95={p95*1000:.1f}ms max={s[-1]*1000:.1f}ms"


class ImagePublisherNode(Node):
    def __init__(self, port: int, intrinsics_path: Path, diag: bool = False,
                 auto_crop: bool = False, crop_rect_path: Path | None = None):
        super().__init__("isaac_ros_image_publisher")

        with open(intrinsics_path) as f:
            calib = json.load(f)
        if calib.get("model") != "fisheye":
            raise SystemExit(
                f"{intrinsics_path} was not produced by a fisheye "
                f"calibration (model={calib.get('model')!r}) -- this bridge "
                "assumes cv2.fisheye output, see camera_calibrate.py")

        self._width = calib["image_width"]
        self._height = calib["image_height"]
        K = calib["camera_matrix"]
        D = calib["distortion_coefficients"]

        # Fixed crop rect, measured OFFLINE before this node ever starts (see
        # fleet/measure_crop_rect.py and the module docstring above) -- if
        # set, the crop command is sent to camera_bridge_windows.py once at
        # startup (below), and CameraInfo is built pre-cropped from the
        # start, so AprilTagNode only ever sees ONE resolution for its whole
        # process lifetime. Mutually exclusive with --auto-crop (checked in
        # main()) -- that flag's live mid-stream discovery is what crashed
        # AprilTagNode on real hardware.
        self._fixed_crop: tuple[int, int, int, int] | None = None
        if crop_rect_path is not None:
            with open(crop_rect_path) as f:
                rect = json.load(f)
            if rect["full_width"] != self._width or rect["full_height"] != self._height:
                raise SystemExit(
                    f"{crop_rect_path} was measured against "
                    f"{rect['full_width']}x{rect['full_height']} frames but "
                    f"{intrinsics_path} is for {self._width}x{self._height} "
                    "-- re-run measure_crop_rect.py")
            self._fixed_crop = (rect["x0"], rect["y0"], rect["x1"], rect["y1"])
            self.get_logger().info(
                f"Loaded fixed crop rect from {crop_rect_path}: "
                f"({rect['x0']},{rect['y0']})-({rect['x1']},{rect['y1']}) "
                f"({rect['crop_width']}x{rect['crop_height']}, measured "
                f"{rect.get('measured_at', '?')})")

        crop_w = self._fixed_crop[2] - self._fixed_crop[0] if self._fixed_crop else self._width
        crop_h = self._fixed_crop[3] - self._fixed_crop[1] if self._fixed_crop else self._height
        crop_x0 = self._fixed_crop[0] if self._fixed_crop else 0
        crop_y0 = self._fixed_crop[1] if self._fixed_crop else 0

        self._camera_info = CameraInfo()
        self._camera_info.width = crop_w
        self._camera_info.height = crop_h
        self._camera_info.distortion_model = "equidistant"
        self._camera_info.d = [float(v) for v in D]
        # Principal point shifts by the crop offset (0,0 if uncropped); focal
        # length is unchanged (cropping doesn't change the lens).
        cx = K[0][2] - crop_x0
        cy = K[1][2] - crop_y0
        self._camera_info.k = [
            K[0][0], K[0][1], cx,
            K[1][0], K[1][1], cy,
            K[2][0], K[2][1], K[2][2],
        ]
        # R = identity (monocular, no stereo rectification rotation needed)
        self._camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P: left 3x3 = K again for a monocular camera (Tx=Ty=0), per
        # CameraInfo.msg's own documented convention.
        self._camera_info.p = [
            K[0][0], K[0][1], cx, 0.0,
            K[1][0], K[1][1], cy, 0.0,
            K[2][0], K[2][1], K[2][2], 0.0,
        ]
        self.get_logger().info(
            f"Loaded fisheye intrinsics from {intrinsics_path} "
            f"(RMS={calib.get('rms_reprojection_error_px', '?')}px, "
            f"{calib.get('num_frames_used', '?')} frames)")

        # RectifyNode's image/camera_info subscribers are RELIABLE (confirmed
        # via `ros2 topic info --verbose` after hitting a QoS-incompatible
        # warning with BEST_EFFORT here) -- DDS requires a RELIABLE publisher
        # to match a RELIABLE subscriber, BEST_EFFORT is not compatible.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._image_pub = self.create_publisher(Image, "image", qos)
        self._info_pub = self.create_publisher(CameraInfo, "camera_info", qos)

        self.get_logger().info(f"Connecting to camera_bridge_windows.py on port {port}...")
        self._src = TcpFrameSource(port, diag=diag)
        self.get_logger().info("Connected.")

        # crop_offset is published REGARDLESS of whether a crop is active
        # (0,0 when not) -- isaac_ros_apriltag_bridge.py subscribes
        # unconditionally and just adds a no-op (0,0) offset onto detections
        # until this fires, so there's no ordering hazard: a subscriber that
        # connects late still gets a correct answer once this message
        # arrives, and treats "no message yet" identically to "(0,0)".
        self._crop_offset_pub = self.create_publisher(Point32, "crop_offset", qos)
        if self._fixed_crop is not None:
            x0, y0, x1, y1 = self._fixed_crop
            self.get_logger().info(
                f"Sending fixed crop command to camera_bridge_windows.py: "
                f"({x0},{y0})-({x1},{y1}) -- BEFORE any frames are "
                "published, so AprilTagNode only ever sees the cropped "
                "resolution.")
            self._src.send_crop(x0, y0, x1, y1)
            self._crop_offset_pub.publish(Point32(x=float(x0), y=float(y0), z=0.0))

        # One-shot auto-crop calibration -- see module docstring. DISABLED
        # BY DEFAULT as of 2026-08-05: confirmed on real hardware that
        # AprilTagNode's internal GPU buffer (cuAprilTags) is sized/pitched
        # for whatever frame dimensions it saw FIRST and does NOT handle a
        # runtime resolution change -- switching from 1920-wide to a
        # 1872-wide cropped frame mid-stream produced repeated
        # cudaErrorInvalidPitchValue errors in AprilTagNode's own CUDA
        # memcpy and GARBAGE tag detections (IDs like 500, 257 that don't
        # exist in this project's tag set) rather than a clean failure --
        # actively dangerous to leave enabled, not just ineffective. See
        # memory: isaac_ros_apriltag_gpu for the full investigation and
        # what a real fix would need (crop BEFORE AprilTagNode ever starts,
        # so it only ever sees one consistent resolution for its whole
        # lifetime -- not attempted yet). Only construct the crop-discovery
        # subscription if explicitly requested with --auto-crop.
        self._auto_crop_enabled = auto_crop
        if self._auto_crop_enabled:
            self.get_logger().warning(
                "--auto-crop is ARMED. This crashed AprilTagNode's CUDA "
                "decoder on real hardware (cudaErrorInvalidPitchValue, "
                "garbage tag IDs) as of 2026-08-05 -- only use this if "
                "you've since fixed the runtime-resize issue, see memory: "
                "isaac_ros_apriltag_gpu")
            self._crop_calib = TableCalibration(DEFAULT_REF_TAG_SIZE_IN)
            self._cropped = False
            # self._crop_offset_pub already created above (published
            # unconditionally, (0,0) by default).
            self._detections_sub = self.create_subscription(
                AprilTagDetectionArray, "tag_detections", self._on_detections_for_crop, qos)
            self.get_logger().info(
                "Auto-crop: waiting for >=2 corner tags (20-23) in "
                "/tag_detections to compute the table's pixel bounding box...")

        self._seq = 0
        self._diag = diag
        if self._diag:
            self._read_stats = RollingStats()
            self._build_stats = RollingStats()
            self._publish_stats = RollingStats()
            self._total_stats = RollingStats()
            self._last_diag_print = time.monotonic()
        # Timer-driven rather than a tight loop so rclpy can still service
        # any callbacks/shutdown requests between frames. NOTE 2026-08-05:
        # deliberately NO rate-limiting gate here anymore -- an earlier
        # version skipped calling _src.read() whenever less than
        # 1/publish_rate had elapsed, which sounds like it should just slow
        # down publishing, but TcpFrameSource.read() drains-and-discards
        # every backlogged frame down to the newest on each call (see its
        # own docstring) -- so gating WHEN read() gets called doesn't reduce
        # work done, it just lets more frames pile up in the OS socket
        # buffer between calls, all of which then get received and thrown
        # away in one read() burst. That data can never be recovered as
        # separate messages once merged this way. Calling read() as fast as
        # possible (every tick) is actually the only way to get close to
        # camera_bridge_windows.py's real 60fps -- confirmed via --diag
        # timing (see memory: isaac_ros_apriltag_gpu) that a rate-limited
        # version measured ~9-10Hz despite a genuinely healthy 60fps source.
        self._timer = self.create_timer(0.0, self._tick)

    def _on_detections_for_crop(self, msg: AprilTagDetectionArray) -> None:
        if self._cropped:
            return  # shouldn't still be subscribed, but be defensive
        detections = _to_adapted(msg)
        self._crop_calib.update(detections)
        if self._crop_calib.H is None:
            return

        x0, y0, x1, y1 = compute_crop_rect(
            (self._height, self._width), self._crop_calib.H)
        crop_w, crop_h = x1 - x0, y1 - y0
        if crop_w <= 0 or crop_h <= 0:
            self.get_logger().warning(
                f"auto-crop: degenerate rect ({x0},{y0})-({x1},{y1}), "
                "skipping -- will retry as more corner tags are seen")
            return

        self.get_logger().info(
            f"auto-crop: table bounding box found at "
            f"({x0},{y0})-({x1},{y1}) ({crop_w}x{crop_h}, "
            f"{100*crop_w*crop_h/(self._width*self._height):.0f}% of full frame) "
            "-- sending crop command to camera_bridge_windows.py")
        self._src.send_crop(x0, y0, x1, y1)

        # Update CameraInfo for the new cropped frame size: principal point
        # shifts by the crop offset, focal length is unchanged (cropping
        # doesn't change the lens), width/height shrink to match.
        K = self._camera_info.k
        new_cx = K[2] - x0
        new_cy = K[5] - y0
        self._camera_info.width = crop_w
        self._camera_info.height = crop_h
        self._camera_info.k = [K[0], K[1], new_cx, K[3], K[4], new_cy, K[6], K[7], K[8]]
        self._camera_info.p = [
            K[0], K[1], new_cx, 0.0,
            K[3], K[4], new_cy, 0.0,
            K[6], K[7], K[8], 0.0,
        ]

        # Publish the offset once so isaac_ros_apriltag_bridge.py can add it
        # back onto AprilTagNode's now-cropped-frame-relative detections
        # before running them through the (unchanged, full-frame) homography.
        # depth=1 + this being sent only once means a subscriber that
        # connects even slightly late could miss it -- acceptable here since
        # both processes are started together as part of one pipeline launch
        # and this fires within the first few seconds once tags are visible,
        # not a live-updating value that needs guaranteed delivery.
        offset_msg = Point32(x=float(x0), y=float(y0), z=0.0)
        self._crop_offset_pub.publish(offset_msg)

        self._cropped = True
        self.destroy_subscription(self._detections_sub)
        self.get_logger().info("auto-crop: done, no longer watching /tag_detections")

    def _tick(self) -> None:
        t0 = time.perf_counter()
        ok, frame = self._src.read()
        t1 = time.perf_counter()
        if not ok or frame is None:
            return
        self._seq += 1

        stamp = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "camera"
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()
        t2 = time.perf_counter()

        self._image_pub.publish(img_msg)

        info_msg = self._camera_info
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = "camera"
        self._info_pub.publish(info_msg)
        t3 = time.perf_counter()

        if self._diag:
            self._read_stats.add(t1 - t0)
            self._build_stats.add(t2 - t1)
            self._publish_stats.add(t3 - t2)
            self._total_stats.add(t3 - t0)
            now = time.monotonic()
            if now - self._last_diag_print >= 2.0:
                n = self._total_stats.count
                elapsed = now - self._last_diag_print
                self.get_logger().info(
                    f"[diag] {n} frames/{elapsed:.1f}s = {n/elapsed:.1f}Hz  "
                    f"read={self._read_stats.summary()}  "
                    f"build={self._build_stats.summary()}  "
                    f"publish={self._publish_stats.summary()}  "
                    f"total={self._total_stats.summary()}")
                # Breaks read's single number down into TcpFrameSource's own
                # recv (network) vs. decode (cv2.imdecode, CPU) vs. how many
                # backlogged frames got drained per read() call -- added
                # 2026-08-05 to find out WHICH of those is actually
                # responsible when [diag]'s read= number goes up, instead of
                # guessing (see memory: isaac_ros_apriltag_gpu).
                src_diag = self._src.diag_summary()
                if src_diag:
                    self.get_logger().info(f"[diag] TcpFrameSource.read(): {src_diag}")
                self._read_stats.reset()
                self._build_stats.reset()
                self._publish_stats.reset()
                self._total_stats.reset()
                self._last_diag_print = now

    def destroy_node(self) -> None:
        try:
            self._src.release()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=8765,
                     help="camera_bridge_windows.py TCP port (default 8765)")
    ap.add_argument("--intrinsics", default="fleet/camera_intrinsics.json",
                     help="path to camera_calibrate.py --solve output "
                          "(default fleet/camera_intrinsics.json)")
    ap.add_argument("--diag", action="store_true",
                     help="print per-stage timing (read/build/publish) every "
                          "2s -- use this to find the real bottleneck instead "
                          "of guessing")
    ap.add_argument("--auto-crop", action="store_true",
                     help="DANGEROUS as of 2026-08-05, DO NOT USE until "
                          "fixed -- crashes AprilTagNode's CUDA decoder "
                          "(cudaErrorInvalidPitchValue) and produces garbage "
                          "tag IDs when the frame size changes at runtime. "
                          "See memory: isaac_ros_apriltag_gpu. Disabled by "
                          "default; this flag exists so the (broken) code "
                          "path is opt-in, not silently armed. Superseded "
                          "by --crop-rect-file, which is safe.")
    ap.add_argument("--crop-rect-file", default=None, metavar="PATH",
                     help="path to a crop rect measured OFFLINE by "
                          "fleet/measure_crop_rect.py (e.g. "
                          "fleet/crop_rect.json). If set, the crop is "
                          "applied from the very first frame -- "
                          "AprilTagNode never sees a resolution change, so "
                          "this is safe (unlike --auto-crop). Recommended "
                          "way to get the throughput win from cropping.")
    args = ap.parse_args()

    if args.auto_crop and args.crop_rect_file:
        print("--auto-crop and --crop-rect-file are mutually exclusive "
              "(pick one crop strategy)", file=sys.stderr)
        raise SystemExit(1)

    intrinsics_path = Path(args.intrinsics)
    if not intrinsics_path.is_file():
        print(f"intrinsics file not found: {intrinsics_path}", file=sys.stderr)
        raise SystemExit(1)

    crop_rect_path = None
    if args.crop_rect_file:
        crop_rect_path = Path(args.crop_rect_file)
        if not crop_rect_path.is_file():
            print(f"crop rect file not found: {crop_rect_path} -- run "
                  "fleet/measure_crop_rect.py first", file=sys.stderr)
            raise SystemExit(1)

    rclpy.init()
    node = ImagePublisherNode(args.port, intrinsics_path, diag=args.diag,
                               auto_crop=args.auto_crop,
                               crop_rect_path=crop_rect_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
