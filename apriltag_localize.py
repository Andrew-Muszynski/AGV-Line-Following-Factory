#!/usr/bin/env python3
"""
apriltag_localize.py — Metric localization of robot AprilTags from the
table-corner reference tags.

Builds on apriltag_detect.py (pixel-only viewer). The table-corner tags
(IDs 20-23) lie in the SAME plane as the robot-mounted tags, so a single 2D
homography image->table maps any detected tag center straight to metric table
coordinates. No camera intrinsics are needed for this — perspective is
absorbed by the homography. (Lens distortion is currently ignored; if accuracy
demands it later, calibrate the Nexigo N980P and undistort frames first.)

World frame (inches):
    origin  = table corner nearest tag 20 (bottom-left)
    +x      = along the bottom edge, tag 20 -> tag 21 (bottom-right)
    +y      = along the left edge,   tag 20 -> tag 22 (top-left)

Measured 2026-07-07: tag 20 center is 2.75 in from both table edges, and the
tag 20 -> tag 23 center offset is 91.5 in in both x and y.

Any >=2 visible corner tags are enough to fit (currently 20 and 23). With only
the two diagonal tags the fit is weakest perpendicular to the 20-23 diagonal,
so expect the largest errors near the empty 21/22 corners until those tags are
placed Thursday — then the same code picks them up automatically.

Robot tags: every detected tag whose ID is NOT in REF_TAG_WORLD is treated as
a robot and reported as (x, y, yaw). Yaw is the world-frame direction of the
tag's corner-0 -> corner-1 edge; it is consistent across tags but has a fixed
offset that depends on how the tag is mounted on the robot — calibrate that
offset once per robot (or mount all tags the same way).

Usage:
    python apriltag_localize.py --tag-size 4.0        # ref tag black square, inches
    python apriltag_localize.py --tag-size 4.0 --camera 1 --log run1.csv
    python apriltag_localize.py --tag-size 4.0 --no-preview

Keys in the preview window: q = quit, r = reset calibration (e.g. camera bumped).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import select
import socket
import statistics
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from pupil_apriltags import Detector

from apriltag_detect import open_camera


class TcpFrameSource:
    """Drop-in replacement for cv2.VideoCapture's .read()/.get()/.release(),
    but receives JPEG frames over a TCP socket instead of opening a local
    camera. Pairs with camera_bridge_windows.py, which does the actual
    cap.read() on Windows (fast MSMF backend) and streams frames here.

    Added 2026-07-28 after USB camera passthrough (usbipd) into WSL2 proved
    unusable for this: it added an ~80-113ms/frame latency floor (a USB-over-
    network transport limitation, not a tunable setting -- tried
    CAP_PROP_BUFFERSIZE=1, smaller resolutions, non-MJPG format, none
    helped, one attempt timed out outright), dropping throughput from ~40Hz
    to ~9Hz despite apriltag detection itself benchmarking identically fast
    on both sides (same CPU-only pupil_apriltags library either way -- GPU
    passthrough itself works fine, confirmed via nvidia-smi/nvcc/CUDA
    samples, it's specifically USB video passthrough that doesn't). This
    class instead receives ALREADY-CAPTURED frames over localhost TCP
    (sub-ms latency, confirmed 2026-07-28 -- WSL2 mirrored networking's
    localhost sharing works even though the mirrored LAN IP wasn't reachable
    from Windows) so capture stays on the fast native path and only the
    already-encoded bytes cross the Windows/WSL2 boundary.

    This is the CLIENT side -- camera_bridge_windows.py is the listener
    (matches how it's actually used: one long-lived Windows capture process
    can serve one WSL2 receiver, not the other way round). Confirmed
    2026-07-28: an earlier version had this backwards (WSL2 listening,
    Windows connecting out) and hit "Address already in use" inside WSL2,
    because mirrored networking's shared localhost means both sides see the
    same port namespace for 127.0.0.1 -- Windows' bridge was already bound
    to the port before WSL2 tried to bind it too.

    Wire format: 4-byte big-endian length prefix, then that many bytes of
    JPEG data. One direction, no ack -- matches camera_bridge_windows.py."""

    def __init__(self, port: int, host: str = "127.0.0.1", connect_timeout: float = 10.0):
        print(f"TcpFrameSource: connecting to camera_bridge_windows.py at "
              f"{host}:{port}...")
        self._conn = socket.create_connection((host, port), timeout=connect_timeout)
        self._conn.settimeout(None)
        print("TcpFrameSource: connected.")
        self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._last_frame: np.ndarray | None = None

    def _recv_exact(self, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._conn.recv(n - len(buf))
            if not chunk:
                return None  # sender disconnected
            buf.extend(chunk)
        return bytes(buf)

    def _recv_one_frame(self) -> bytes | None:
        """Read exactly one length-prefixed JPEG payload, blocking until it
        arrives. Does not consider whether a newer frame is already queued
        behind it -- see read() for that."""
        header = self._recv_exact(4)
        if header is None:
            return None
        length = int.from_bytes(header, "big")
        return self._recv_exact(length)

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Confirmed 2026-07-28 (turn-test hardware run): camera_bridge_
        windows.py's capture rate can exceed how fast this side actually
        calls read() (detection + pose math + publish takes real time per
        loop), so complete frames pile up in the OS TCP receive buffer. A
        naive single _recv_exact() per read() drains that backlog OLDEST-
        FIRST -- every read() for a while returns increasingly-stale
        backlogged frames (near-identical position, real capture time long
        past) until the backlog empties, at which point position appears to
        "jump" by several real camera frames' worth of motion in one read.
        Observed directly: settle-window samples ~0.7ms apart in wall-clock
        time reporting yaw deltas of several degrees -- physically
        impossible at the commanded turn RPM, i.e. definitively stale data,
        not real robot motion. Fixed by always draining every complete frame
        currently sitting in the socket buffer and keeping only the LAST
        (newest) one -- select() with a 0 timeout is non-blocking, so this
        never waits once the backlog is empty, only skips over frames that
        already fully arrived."""
        data = self._recv_one_frame()
        if data is None:
            return False, None
        while True:
            ready, _, _ = select.select([self._conn], [], [], 0.0)
            if not ready:
                break
            newer = self._recv_one_frame()
            if newer is None:
                break
            data = newer
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return False, None
        self._last_frame = frame
        return True, frame

    def isOpened(self) -> bool:  # noqa: N802 - matches cv2.VideoCapture's API
        return self._conn is not None

    def get(self, prop_id: int) -> float:
        # Only the two properties apriltag_localize.py actually queries after
        # setup (frame width/height, for the startup print) -- report from
        # the last received frame once one has arrived, else 0 (matches
        # cv2.VideoCapture's own behavior before the first successful read).
        if self._last_frame is None:
            return 0.0
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._last_frame.shape[1])
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._last_frame.shape[0])
        return 0.0

    def set(self, prop_id: int, value: float) -> bool:
        # No-op: resolution/FPS/FOURCC are controlled on the Windows sender
        # side (camera_bridge_windows.py), not here -- silently accept so
        # existing cap.set(...) calls in main() don't need special-casing.
        return True

    def release(self) -> None:
        try:
            self._conn.close()
        except Exception:
            pass


class RollingStats:
    """Collects a bounded window of samples (seconds) for one timing/rate
    metric and reports median/p95/max/count on demand. Used for the T0-T8
    benchmark matrix (2026-07-28) to separate camera capture, AprilTag
    detection, calibration, pose math, and rosbridge publish enqueue cost --
    the previous diag: line bundled detect()+calib.update() into one number
    and only ever reported a running average, which hides exactly the
    occasional-slow-frame behavior (the "38Hz vs 11Hz" alternation) this is
    meant to catch."""

    __slots__ = ("_samples",)

    def __init__(self) -> None:
        self._samples: list[float] = []

    def add(self, value: float) -> None:
        self._samples.append(value)

    def reset(self) -> None:
        self._samples.clear()

    def summary(self, window_sec: float) -> str:
        n = len(self._samples)
        if n == 0:
            return "n=0"
        s = sorted(self._samples)
        median = statistics.median(s)
        p95 = s[min(int(math.ceil(0.95 * n)) - 1, n - 1)]
        return (f"n={n} rate={n / window_sec:.1f}/s "
                f"median={median * 1000:.1f}ms p95={p95 * 1000:.1f}ms "
                f"max={s[-1] * 1000:.1f}ms")

# ---------------- table geometry (inches) ----------------
TAG20_INSET_IN = 2.75   # tag 20 center to table edge, both axes (measured)
SPAN_IN = 91.5          # tag 20 center -> tag 23 center, both axes (measured)

REF_TAG_WORLD: dict[int, tuple[float, float]] = {
    20: (TAG20_INSET_IN, TAG20_INSET_IN),                      # bottom-left  (placed)
    23: (TAG20_INSET_IN + SPAN_IN, TAG20_INSET_IN + SPAN_IN),  # top-right    (placed)
    # Arriving Thursday 2026-07-09 — these coordinates ASSUME the same 2.75 in
    # inset and 91.5 in spacing. Measure and correct when placed:
    21: (TAG20_INSET_IN + SPAN_IN, TAG20_INSET_IN),            # bottom-right (pending)
    22: (TAG20_INSET_IN, TAG20_INSET_IN + SPAN_IN),            # top-left     (pending)
}

# Table outline drawn on the preview; assumes tag 23 has the same 2.75 in inset.
TABLE_SIZE_IN = SPAN_IN + 2 * TAG20_INSET_IN  # 97.0

# Outer edge length of the ref tags' black square, remeasured 2026-07-24:
# 3.835 in. Override with --tag-size if the tags are ever reprinted.
DEFAULT_REF_TAG_SIZE_IN = 3.835

# Tag IDs 1-4 observed on the robots 2026-07-07; verify each tag is on the
# matching Alvik (i.e. tag 1 on the robot publishing Alvik1_pose).
ROBOT_NAMES: dict[int, str] = {1: "Alvik1", 2: "Alvik2", 3: "Alvik3", 4: "Alvik4"}

# ---- grid anchor (tape-measured 2026-07-09) ----
# Node 1 (first node of the 8x8 lattice) center in table inches: 13.5 in x,
# 16.75 in y from the table edges = +10.75/+14.0 from tag 20's center.
# Grid axes are assumed parallel to the table axes. Grid coordinates are
# reported in CELLS relative to node 1 (node 1 = (0,0), +grid_x along table
# +x, +grid_y along table +y); a robot centered on any lattice node should
# read near-integers — drift from integers on far nodes would reveal a pitch
# or rotation error.
GRID_NODE1_WORLD_IN = (13.5, 16.75)
GRID_PITCH_IN = 10.0


def world_to_grid(x_in: float, y_in: float) -> tuple[float, float]:
    return ((x_in - GRID_NODE1_WORLD_IN[0]) / GRID_PITCH_IN,
            (y_in - GRID_NODE1_WORLD_IN[1]) / GRID_PITCH_IN)


# ---------------- geometry helpers ----------------

def map_points(H: np.ndarray, pts) -> np.ndarray:
    """Apply homography H to an (N,2) array of points."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def _rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _fit_similarity(img_pts: np.ndarray, world_pts: np.ndarray, improper: bool):
    """Least-squares 2D similarity img->world (complex formulation); exact for
    2 points. `improper` includes a reflection. Returns an apply-function."""
    z = img_pts[:, 0] + 1j * img_pts[:, 1]
    if improper:
        z = z.conj()
    w = world_pts[:, 0] + 1j * world_pts[:, 1]
    zc, wc = z - z.mean(), w - w.mean()
    denom = float((zc * zc.conj()).real.sum())
    if denom < 1e-9:
        return None
    a = (wc * zc.conj()).sum() / denom
    b = w.mean() - a * z.mean()

    def apply(p: np.ndarray) -> np.ndarray:
        q = p[:, 0] + 1j * p[:, 1]
        if improper:
            q = q.conj()
        r = a * q + b
        return np.column_stack([r.real, r.imag])

    return apply


def _fit_rotation(template: np.ndarray, target: np.ndarray) -> float:
    """Angle of the rotation best aligning zero-mean `template` corners to
    `target` corners (2D Kabsch)."""
    q = target - target.mean(axis=0)
    num = float(np.sum(template[:, 0] * q[:, 1] - template[:, 1] * q[:, 0]))
    den = float(np.sum(template[:, 0] * q[:, 0] + template[:, 1] * q[:, 1]))
    return math.atan2(num, den)


def _jacobian_det_sign(H: np.ndarray, pt) -> float:
    p = np.asarray(pt, dtype=float)
    base, dx, dy = map_points(H, np.array([p, p + (1.0, 0.0), p + (0.0, 1.0)]))
    v1, v2 = dx - base, dy - base
    return math.copysign(1.0, v1[0] * v2[1] - v1[1] * v2[0])


def _shoelace_sign(pts: np.ndarray) -> float:
    x, y = pts[:, 0], pts[:, 1]
    return math.copysign(1.0, float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)))


def _dlt_homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray | None:
    """Least-squares homography src->dst via normalized DLT. Unlike
    cv2.findHomography's LM refinement this is exact (to float precision) on
    consistent data, which matters here because tiny corner-fit errors get
    amplified ~table/tag-size when extrapolated across the table."""

    def norm_transform(p: np.ndarray) -> np.ndarray:
        c = p.mean(axis=0)
        d = float(np.mean(np.linalg.norm(p - c, axis=1)))
        if d < 1e-12:
            return None
        s = math.sqrt(2.0) / d
        return np.array([[s, 0.0, -s * c[0]], [0.0, s, -s * c[1]], [0.0, 0.0, 1.0]])

    Ts, Td = norm_transform(src), norm_transform(dst)
    if Ts is None or Td is None:
        return None
    ones = np.ones((len(src), 1))
    sh = (Ts @ np.hstack([src, ones]).T).T
    dh = (Td @ np.hstack([dst, ones]).T).T
    rows = []
    for (x, y, _), (u, v, _) in zip(sh, dh):
        rows.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, -u])
        rows.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, -v])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    Hn = vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ Hn @ Ts
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    return H


def fit_table_homography(
    ref_corners_img: dict[int, np.ndarray],
    tag_size_in: float,
    world_centers: dict[int, tuple[float, float]] = REF_TAG_WORLD,
    init_thetas: dict[int, float] | None = None,
):
    """Fit the image->table homography from the corners of >=2 reference tags.

    ref_corners_img: {tag_id: (4,2) image-pixel corners as returned by the
    detector}. Uses all 4 corners of every visible ref tag (2 tags = 8 points,
    enough to constrain the 8-DOF homography), anchored to the known world
    centers. The in-plane rotation of each ref tag is estimated, so the tags
    do NOT need to be square with the table edges.

    The corner-order winding is derived, not assumed: the detector only ever
    sees tag fronts, image y points down and world y points up, so the corner
    sequence winds the OPPOSITE way in world coordinates than in image pixels.
    Getting this right matters — with two ref tags on the table diagonal, the
    reflected (wrong-winding) solution fits the corners exactly as well but
    mirrors every robot across the diagonal, so it cannot be told apart by
    residual alone.

    init_thetas: optional {tag_id: rotation} warm start from a previous fit;
    skips the cold-start search and converges in a couple of iterations.

    Returns (H, rms_in, used_ids, thetas) or None.
    """
    ids = sorted(i for i in ref_corners_img if i in world_centers)
    if len(ids) < 2:
        return None
    img_corners = [np.asarray(ref_corners_img[i], dtype=float) for i in ids]
    img_centers = np.array([c.mean(axis=0) for c in img_corners])
    wld_centers = np.array([world_centers[i] for i in ids], dtype=float)
    img_stack = np.vstack(img_corners)

    signs = {_shoelace_sign(c) for c in img_corners}
    if len(signs) != 1:
        return None  # detections disagree on winding: garbage frame
    sigma_img = signs.pop()

    half = tag_size_in / 2.0
    template = np.array([(-half, -half), (half, -half), (half, half), (-half, half)])
    if sigma_img > 0:  # world winding must be -sigma_img -> clockwise template
        template = template[::-1].copy()

    # With only two reference tags the per-tag rotation is nearly redundant
    # with the homography (H can locally mimic a small rotation of one tag),
    # which makes freely-estimated rotations ill-conditioned: noise walks the
    # fit along that flat valley and moves mid-table points by inches. So with
    # 2 tags we SNAP each rotation to the nearest 90 degrees (tags are placed
    # roughly square with the table; the square's symmetry makes 90-degree
    # steps safe) and keep it fixed. With >=3 tags the geometry pins the
    # rotations down and they are refined freely.
    free_theta = len(ids) >= 3

    def snap(thetas: list[float]) -> list[float]:
        return [round(t / (math.pi / 2)) * (math.pi / 2) for t in thetas]

    candidates: list[list[float]] = []
    if init_thetas is not None and all(i in init_thetas for i in ids):
        candidates.append([init_thetas[i] for i in ids])
    else:
        # Cold start: approximate image->world with a similarity fitted to the
        # tag centers and read each tag's rotation off it. The physical map
        # flips handedness (see winding note above) so the improper fit is the
        # right one, but try both — a wrong start just loses on residual.
        for improper in (True, False):
            sim = _fit_similarity(img_centers, wld_centers, improper)
            if sim is not None:
                thetas = [_fit_rotation(template, sim(c)) for c in img_corners]
                candidates.append(thetas if free_theta else snap(thetas))

    best = None
    for thetas in candidates:
        H = None
        rms = None
        for _ in range(150 if free_theta else 1):
            world_stack = np.vstack([
                wld_centers[k] + template @ _rot(th).T
                for k, th in enumerate(thetas)
            ])
            H = _dlt_homography(img_stack, world_stack)
            if H is None:
                break
            mapped = map_points(H, img_stack)
            rms = float(np.sqrt(np.mean(np.sum((mapped - world_stack) ** 2, axis=1))))
            if not free_theta:
                break
            new_thetas = [_fit_rotation(template, map_points(H, c)) for c in img_corners]
            dtheta = max(abs(n - t) for n, t in zip(new_thetas, thetas))
            thetas = new_thetas
            if dtheta < 1e-12:
                break
        if H is None or rms is None:
            continue
        # A physically valid map has one handedness everywhere on the table;
        # reject fits whose Jacobian flips sign between reference tags.
        det_signs = {_jacobian_det_sign(H, c) for c in img_centers}
        if len(det_signs) != 1:
            continue
        if best is None or rms < best[1]:
            best = (H, rms, ids, dict(zip(ids, thetas)))
    return best


def tag_world_pose(H: np.ndarray, det) -> tuple[float, float, float]:
    """(x_in, y_in, yaw_deg) of a detection. Yaw = world direction of the
    tag's corner0->corner1 edge, degrees, CCW from +x."""
    center = map_points(H, np.asarray(det.center, dtype=float).reshape(1, 2))[0]
    e0, e1 = map_points(H, np.asarray(det.corners[:2], dtype=float))
    yaw = math.degrees(math.atan2(e1[1] - e0[1], e1[0] - e0[0]))
    return float(center[0]), float(center[1]), yaw


class TableCalibration:
    """EMA-smoothed reference-tag corners + the fitted homography.

    Corner positions persist once seen, so the calibration survives the ref
    tags being occluded by robots or hands. Press 'r' after moving the camera.

    freeze_after_n (2026-07-28, T2 in the benchmark matrix): by default
    fit_table_homography() -- an SVD-based DLT plus up to 150 rotation-
    refinement iterations with 3+ ref tags -- reruns on EVERY frame where any
    ref tag corner moves at all, which under EMA smoothing of live pixel
    noise is essentially every frame; this was previously bundled into the
    same "avg detect" timer as detector.detect() itself, so its true cost was
    never isolated. Setting freeze_after_n > 0 accumulates that many ref-tag
    observations (averaging corners, following implementation review --
    NOT freezing on the first noisy single-frame fit), fits ONE homography,
    and stops re-fitting after that (only --tag-size/geometry are fixed
    inputs; recalibration still needs an explicit reset(), e.g. after the
    camera is bumped)."""

    def __init__(self, tag_size_in: float, ema_alpha: float = 0.15,
                 freeze_after_n: int = 0):
        self.tag_size_in = tag_size_in
        self.ema_alpha = ema_alpha
        self.freeze_after_n = freeze_after_n
        self.corners: dict[int, np.ndarray] = {}
        self.H: np.ndarray | None = None
        self.rms_in: float | None = None
        self.used_ids: list[int] = []
        self.thetas: dict[int, float] = {}
        self._frozen = False
        self._accum: dict[int, np.ndarray] = {}
        self._accum_count = 0

    def reset(self) -> None:
        self.corners.clear()
        self.H = None
        self.rms_in = None
        self.used_ids = []
        self.thetas = {}
        self._frozen = False
        self._accum.clear()
        self._accum_count = 0

    def update(self, detections) -> None:
        if self._frozen:
            return
        if self.freeze_after_n > 0:
            self._update_freezing(detections)
            return
        changed = False
        for det in detections:
            if det.tag_id not in REF_TAG_WORLD:
                continue
            c = np.asarray(det.corners, dtype=float)
            prev = self.corners.get(det.tag_id)
            self.corners[det.tag_id] = (
                c if prev is None else (1 - self.ema_alpha) * prev + self.ema_alpha * c
            )
            changed = True
        if changed:
            fit = fit_table_homography(self.corners, self.tag_size_in,
                                       init_thetas=self.thetas or None)
            if fit is not None:
                self.H, self.rms_in, self.used_ids, self.thetas = fit

    def _update_freezing(self, detections) -> None:
        """Accumulate up to freeze_after_n observations per ref tag, average
        them, fit ONCE, then stop touching self.corners/H/etc entirely."""
        seen_this_frame = False
        for det in detections:
            if det.tag_id not in REF_TAG_WORLD:
                continue
            seen_this_frame = True
            c = np.asarray(det.corners, dtype=float)
            prev = self._accum.get(det.tag_id)
            self._accum[det.tag_id] = c if prev is None else prev + c
        if seen_this_frame:
            self._accum_count += 1
        if self._accum_count < self.freeze_after_n:
            return
        if len(self._accum) < 2:
            return  # not enough distinct ref tags seen yet to fit
        averaged = {tid: c / self._accum_count for tid, c in self._accum.items()}
        fit = fit_table_homography(averaged, self.tag_size_in)
        if fit is not None:
            self.H, self.rms_in, self.used_ids, self.thetas = fit
            self.corners = averaged
            self._frozen = True


class RosbridgePublisher:
    """Publishes vision poses into the ROS2 graph via a rosbridge websocket
    (same route the solver HTML's Real mode uses), so nothing ROS needs to be
    installed on this machine.

    One std_msgs/String topic per robot, `/<Name>_vision_pose`, JSON payload
    {"x_in", "y_in", "yaw_deg", "tag_id", "ms"} — explicit units to avoid
    confusion with the cm-based odometry `_pose` topics. Calibration health
    goes to `/vision_calib` every couple of seconds.
    """

    def __init__(self, host: str, port: int, batch: bool = False):
        import roslibpy  # imported here so the script runs without it
        self._roslibpy = roslibpy
        self.client = roslibpy.Ros(host=host, port=port)
        self._topics: dict[str, object] = {}
        self._last_calib_pub = 0.0
        self._warned = False
        # batch (2026-07-28, T7 in the benchmark matrix): ADDITIVE, not a
        # replacement -- publishes a single /vision_poses_batch message
        # alongside the existing per-robot /<Name>_vision_pose topics, so
        # camera_grid_navigate.py's existing per-robot subscription (which
        # this session's control-loop tuning depends on) is never broken by
        # a benchmark flag. Measure batch vs. per-topic publish-enqueue cost
        # by comparing runs with/without --publish-batch.
        self.batch = batch
        self.frame_seq = 0
        # Diagnostic (2026-07-27): camera_grid_navigate.py measured ~2.5Hz
        # pose arrival despite --publish-rate 60 and Capture FPS: 60 -- this
        # counter proves/disproves whether the PUBLISH side is actually
        # hitting the requested rate, isolating publish-side vs.
        # rosbridge/network-side as the bottleneck.
        self._publish_count = 0
        self._publish_count_since = time.time()
        try:
            self.client.run(timeout=5)
            print(f"rosbridge: connected to ws://{host}:{port}")
        except Exception as exc:
            print(f"rosbridge: could not connect to ws://{host}:{port} ({exc}); "
                  "will keep localizing and retry in the background")

    def _topic(self, name: str):
        t = self._topics.get(name)
        if t is None:
            t = self._roslibpy.Topic(self.client, name, "std_msgs/String")
            t.advertise()
            self._topics[name] = t
        return t

    def publish(self, poses: dict[int, tuple[float, float, float]],
                calib: "TableCalibration", now: float) -> float:
        """Returns the publish-enqueue wall time (seconds) for this call, so
        the caller can feed it into the publish_enqueue_ms RollingStats."""
        t0 = time.perf_counter()
        if not self.client.is_connected:
            if not self._warned:
                print("rosbridge: not connected, poses not being published")
                self._warned = True
            return time.perf_counter() - t0
        self._warned = False
        ms = int(now * 1000)
        self.frame_seq += 1
        if self.batch:
            batch_poses = []
            for tid, (x, y, yaw) in sorted(poses.items()):
                gx, gy = world_to_grid(x, y)
                batch_poses.append({
                    "tag_id": tid, "name": ROBOT_NAMES.get(tid, f"tag{tid}"),
                    "x_in": round(x, 2), "y_in": round(y, 2),
                    "yaw_deg": round(yaw, 1),
                    "grid_x": round(gx, 3), "grid_y": round(gy, 3)})
            payload = {"seq": self.frame_seq, "ms": ms, "poses": batch_poses}
            self._topic("/vision_poses_batch").publish(
                self._roslibpy.Message({"data": json.dumps(payload)}))
            self._publish_count += len(batch_poses)
        else:
            for tid, (x, y, yaw) in sorted(poses.items()):
                name = ROBOT_NAMES.get(tid, f"tag{tid}")
                gx, gy = world_to_grid(x, y)
                payload = {"x_in": round(x, 2), "y_in": round(y, 2),
                           "yaw_deg": round(yaw, 1),
                           "grid_x": round(gx, 3), "grid_y": round(gy, 3),
                           "tag_id": tid, "seq": self.frame_seq, "ms": ms}
                self._topic(f"/{name}_vision_pose").publish(
                    self._roslibpy.Message({"data": json.dumps(payload)}))
                self._publish_count += 1
        if now - self._publish_count_since >= 2.0:
            print(f"rosbridge: {self._publish_count / (now - self._publish_count_since):.1f} "
                  "pose publishes/sec (diagnostic)")
            self._publish_count = 0
            self._publish_count_since = now
        if now - self._last_calib_pub >= 2.0 and calib.H is not None:
            status = {"refs": calib.used_ids, "rms_in": round(calib.rms_in, 3),
                      "ms": ms}
            self._topic("/vision_calib").publish(
                self._roslibpy.Message({"data": json.dumps(status)}))
            self._last_calib_pub = now
        return time.perf_counter() - t0

    def close(self) -> None:
        try:
            for t in self._topics.values():
                t.unadvertise()
            self.client.terminate()
        except Exception:
            pass


# ---------------- live viewer ----------------

def build_detector(family: str, decimate: float, nthreads: int = 16) -> Detector:
    return Detector(
        families=family,
        nthreads=nthreads,
        quad_decimate=decimate,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )


# distinct outline colors per robot (BGR): yellow, orange, magenta, cyan
ROBOT_PALETTE = [(0, 255, 255), (0, 140, 255), (255, 0, 255), (255, 255, 0)]


def _robot_color(tag_id: int) -> tuple[int, int, int]:
    return ROBOT_PALETTE[tag_id % len(ROBOT_PALETTE)]


def _panel_bg(frame, x0: int, y0: int, x1: int, y1: int, alpha: float = 0.55) -> None:
    """Darken a rectangle behind overlay text so readability doesn't depend
    on what happens to be in the live camera feed there (a bright window,
    whiteboard, or reflective tape all defeated the plain black-outline text
    at times). Blended, not opaque, so the underlying video is still visible
    through the panel."""
    h, w = frame.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return
    roi = frame[y0:y1, x0:x1]
    overlay = np.zeros_like(roi)
    cv2.addWeighted(overlay, alpha, roi, 1 - alpha, 0, dst=roi)


def _text(frame, s: str, org: tuple[int, int], color, scale: float = 0.55) -> None:
    """putText with a dark underlay so labels stay readable on bright video."""
    cv2.putText(frame, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4)
    cv2.putText(frame, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)


def draw_overlay(frame, calib: TableCalibration, detections, poses) -> None:
    if calib.H is not None:
        Hinv = np.linalg.inv(calib.H)
        border = np.array([
            [0, 0], [TABLE_SIZE_IN, 0], [TABLE_SIZE_IN, TABLE_SIZE_IN], [0, TABLE_SIZE_IN],
        ], dtype=float)
        cv2.polylines(frame, [map_points(Hinv, border).astype(np.int32)],
                      True, (255, 160, 0), 2)
        o, xt, yt = (tuple(int(v) for v in p) for p in
                     map_points(Hinv, np.array([[0.0, 0.0], [12.0, 0.0], [0.0, 12.0]])))
        cv2.arrowedLine(frame, o, xt, (0, 0, 255), 2, tipLength=0.25)
        cv2.arrowedLine(frame, o, yt, (0, 255, 0), 2, tipLength=0.25)
        cv2.putText(frame, "+x", (xt[0] + 6, xt[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, "+y", (yt[0] + 6, yt[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Tags get only a compact ID marker; full poses go to the side panel below
    # so side-by-side robots don't overprint each other.
    for det in detections:
        corners = det.corners.astype(np.int32)
        cx, cy = int(det.center[0]), int(det.center[1])
        if det.tag_id in REF_TAG_WORLD:
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            # label below the tag: corner tags sit at the frame edges, where a
            # label above would run off-screen or into the status line
            _text(frame, f"REF {det.tag_id}", (cx + 8, cy + 24), (0, 255, 0))
        else:
            color = _robot_color(det.tag_id)
            cv2.polylines(frame, [corners], True, color, 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            top = corners[corners[:, 1].argmin()]
            _text(frame, str(det.tag_id), (int(top[0]) - 8, int(top[1]) - 10),
                  color, scale=0.7)

    if calib.H is not None:
        status = f"calib OK  refs={calib.used_ids}  rms={calib.rms_in:.2f} in"
        scolor = (0, 255, 0) if calib.rms_in < 1.0 else (0, 165, 255)
    else:
        status = "waiting for >=2 corner tags (IDs 20-23)..."
        scolor = (0, 165, 255)

    pose_lines = []
    for tid, (x, y, yaw) in sorted(poses.items()):
        name = ROBOT_NAMES.get(tid, f"id {tid}")
        gx, gy = world_to_grid(x, y)
        pose_lines.append((
            f"{tid}  {name:<8} x={x:6.1f}  y={y:6.1f}  yaw={yaw:+5.0f}"
            f"  grid=({gx:+5.2f},{gy:+5.2f})",
            _robot_color(tid)))

    # One dark backing panel behind the whole status + pose block, sized to
    # the actual widest line (measured, not guessed) and however many robots
    # are currently detected, so the text stays readable regardless of what's
    # in the live scene behind it (a bright window or whiteboard previously
    # washed the plain outlined text out).
    widest = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)[0][0]
    for line, _ in pose_lines:
        w = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
        widest = max(widest, w)
    panel_bottom = 55 + 24 * len(pose_lines) - 8
    _panel_bg(frame, 4, 6, 18 + widest, panel_bottom)

    _text(frame, status, (10, 25), scolor, scale=0.65)
    panel_y = 55
    for line, color in pose_lines:
        _text(frame, line, (10, panel_y), color)
        panel_y += 24


class MjpegServer:
    """Tiny MJPEG server so the solver HTML's Live mode can embed the
    annotated camera view (<img src="http://localhost:PORT/stream">).
    Frames are pushed from the main loop; each connected client gets the
    latest frame as a multipart/x-mixed-replace stream."""

    def __init__(self, port: int):
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802 - stdlib naming
                if self.path not in ("/", "/stream"):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        with server._cond:
                            server._cond.wait_for(
                                lambda: server._seq != seq, timeout=1.0)
                            jpeg, seq = server._jpeg, server._seq
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\n"
                                         b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError,
                        ConnectionAbortedError):
                    pass  # viewer closed the tab / switched mode

            def log_message(self, *args):
                pass  # silence per-request console spam

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        print(f"MJPEG preview stream: http://localhost:{port}/stream "
              "(solver HTML Live mode)")

    def push(self, frame: np.ndarray) -> None:
        # Downscale + recompress: the stream is a monitor view, not the
        # detector input — ~960px wide at q70 keeps it near 0.5 MB/s.
        if frame.shape[1] > 960:
            scale = 960 / frame.shape[1]
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cond.notify_all()

    def close(self) -> None:
        self._httpd.shutdown()


def _run_capture_only(args: argparse.Namespace) -> None:
    """T8/camera-vs-CPU test (2026-07-28 benchmark matrix): cap.read() only,
    no detection/calibration/publish/preview, counting unique frames over
    --capture-only SECONDS. Note MJPG capture means cap.read() already
    includes CPU JPEG decoding, not pure camera/USB time -- see the
    decision-rule doc comment above the flag's help text."""
    cap = open_camera(args.camera, preferred_backend=args.backend)
    if cap is None:
        raise SystemExit(f"Could not open camera index {args.camera}; "
                         "try apriltag_detect.py --list-cameras")
    if args.width and args.height:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    print(f"Capture-only mode: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, running for "
          f"{args.capture_only:.0f}s...")
    stats = RollingStats()
    count = 0
    fail_count = 0
    t_start = time.perf_counter()
    while time.perf_counter() - t_start < args.capture_only:
        t0 = time.perf_counter()
        ok, _frame = cap.read()
        dt = time.perf_counter() - t0
        if not ok:
            fail_count += 1
            continue
        stats.add(dt)
        count += 1
    elapsed = time.perf_counter() - t_start
    cap.release()
    fps = count / elapsed
    print(f"capture-only: {count} frames in {elapsed:.1f}s = {fps:.1f} fps "
          f"({fail_count} failed reads)")
    print(f"capture-only read timing: {stats.summary(elapsed)}")
    if fps >= 55:
        print("  -> >=55fps: camera/USB/backend OK, CPU processing is the "
              "limiter for overall pipeline rate.")
    elif abs(fps - 30) < 2:
        print("  -> ~30fps: camera/backend likely negotiated 30fps despite "
              "reporting 60 -- check --backend, MJPG negotiation, USB port.")
    else:
        print("  -> variable/low fps: investigate USB port, MJPEG "
              "negotiation, MSMF, exposure, or webcam settings.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Metric AGV localization from table-corner AprilTags.")
    parser.add_argument("--camera", type=int, default=1,
                        help="webcam index (default 1 = overhead Nexigo; 0 is the built-in cam)")
    parser.add_argument("--backend", default=None, choices=["dshow", "msmf", "any"],
                        help="force a specific Windows capture backend instead "
                             "of the default DSHOW-first fallback order. "
                             "2026-07-27: cap.read() measured ~140ms/call on "
                             "DSHOW at 1920x1080 despite a 60fps-capable "
                             "camera -- try --backend msmf to compare.")
    parser.add_argument("--frame-source-port", type=int, default=None,
                        metavar="PORT",
                        help="run in TCP-frame-receiver mode instead of "
                             "opening a local camera: listens on this port "
                             "for camera_bridge_windows.py to connect and "
                             "stream JPEG frames (--camera/--backend/"
                             "--width/--height/--fps are then ignored -- "
                             "those are controlled on the Windows sender "
                             "side). Added 2026-07-28 as the working "
                             "alternative to USB camera passthrough into "
                             "WSL2, which measured ~80-113ms/frame and was "
                             "rejected; see TcpFrameSource's docstring.")
    parser.add_argument("--family", default="tag36h11", help="AprilTag family (default tag36h11)")
    parser.add_argument("--tag-size", type=float, default=None,
                        help="ref-tag black-square edge length in INCHES (measure it!)")
    parser.add_argument("--width", type=int, default=1920, help="capture width (default 1920)")
    parser.add_argument("--height", type=int, default=1080, help="capture height (default 1080)")
    parser.add_argument("--fps", type=int, default=None,
                        help="request this capture FPS from the camera (e.g. 60); "
                             "unset leaves the driver's default. Many USB webcams "
                             "silently clamp/ignore this -- the printed 'Capture "
                             "FPS' line after startup shows what was actually "
                             "granted, not just what was requested.")
    parser.add_argument("--decimate", type=float, default=1.0,
                        help="detector quad_decimate; raise to 1.5-2 if fps is low (default 1.0)")
    parser.add_argument("--print-interval", type=float, default=0.5,
                        help="seconds between console pose lines (default 0.5)")
    parser.add_argument("--log", default=None, help="append poses to this CSV file")
    parser.add_argument("--no-preview", action="store_true", help="headless: console output only")
    parser.add_argument("--rosbridge", default=None, metavar="HOST[:PORT]",
                        nargs="?", const="192.0.2.14:9090",
                        help="publish poses to a rosbridge websocket on the ROS2 "
                             "laptop; bare --rosbridge uses the lab default "
                             "192.0.2.14:9090")
    parser.add_argument("--publish-rate", type=float, default=30.0,
                        help="rosbridge publish rate in Hz (default 30 -- capture "
                             "FPS and publish rate are independent; raising --fps "
                             "alone does not reach the supervisor faster unless "
                             "this also goes up). Was 10; raised alongside --fps "
                             "so a faster camera actually lowers correction "
                             "latency instead of being throttled back down here.")
    parser.add_argument("--stream", type=int, nargs="?", const=8081,
                        default=None, metavar="PORT",
                        help="serve the annotated view as MJPEG for the "
                             "solver HTML's Live mode (bare --stream = port "
                             "8081)")
    parser.add_argument("--threads", type=int, default=16,
                        help="AprilTag detector worker threads (default 16 "
                             "-- T6 sweep, 2026-07-28, on this machine's "
                             "i7-14650HX: 1 thread was worst and most "
                             "volatile (apriltag p95 61-179ms, rate 5.8-"
                             "19.2Hz); 4 threads (the old default) still had "
                             "occasional 70ms+ spikes; 16 threads was both "
                             "fastest AND most stable -- p95 22-23ms with "
                             "ZERO bad windows across 8 consecutive 30s "
                             "samples, vs. sporadic outliers at every lower "
                             "count. Clock speed/utilization stayed low "
                             "(11-25 percent, 4.3-4.8GHz) throughout the whole "
                             "sweep -- ruled out CPU throttling -- so this "
                             "was scheduling/contention among too few "
                             "detector threads on a 24-logical-processor "
                             "CPU (8P+16E), not raw compute power. Re-test "
                             "if this ever runs on different hardware.)")
    parser.add_argument("--freeze-calib", type=int, default=0, metavar="N",
                        help="T2 in the 2026-07-28 benchmark matrix: "
                             "average N ref-tag observations, fit the "
                             "table homography ONCE, then stop re-fitting "
                             "every frame (default 0 = never freeze, "
                             "original continuous-refit behavior). "
                             "Recalibration still needs 'r' (or camera "
                             "move detection, not implemented) since this "
                             "is a hard freeze, not a slowdown.")
    parser.add_argument("--publish-batch", action="store_true",
                        help="T7 in the 2026-07-28 benchmark matrix: ALSO "
                             "publish one batched /vision_poses_batch "
                             "message per frame (all robots' poses + a "
                             "shared seq), in addition to the existing "
                             "per-robot /<Name>_vision_pose topics -- "
                             "additive, does not change or replace the "
                             "per-robot topics camera_grid_navigate.py "
                             "already subscribes to")
    parser.add_argument("--capture-only", type=float, default=None,
                        metavar="SECONDS",
                        help="T8/camera-vs-CPU test: run ONLY cap.read() "
                             "in a tight loop for this many seconds, "
                             "counting unique frames, then exit -- no "
                             "detection/calibration/publish/preview at "
                             "all. >=55fps means the camera/USB/backend "
                             "is fine and CPU processing is the limiter; "
                             "~30fps means the camera negotiated 30 "
                             "despite reporting 60.")
    parser.add_argument("--bench-interval", type=float, default=2.0,
                        help="seconds between RollingStats summary prints "
                             "for the T0-T8 benchmark matrix (default 2.0, "
                             "use a wider window e.g. 30-60 for the "
                             "3-5 minute test procedure so median/p95 "
                             "reflect the whole run, not one short slice)")
    args = parser.parse_args()

    if args.capture_only is not None:
        _run_capture_only(args)
        return

    publisher = None
    if args.rosbridge:
        target = args.rosbridge.removeprefix("ws://").rstrip("/")
        host, _, port = target.partition(":")
        publisher = RosbridgePublisher(host, int(port) if port else 9090,
                                        batch=args.publish_batch)

    streamer = MjpegServer(args.stream) if args.stream else None

    tag_size = args.tag_size
    if tag_size is None:
        tag_size = DEFAULT_REF_TAG_SIZE_IN
        print(f"Using measured ref-tag size {tag_size:.4f} in "
              f"black square; override with --tag-size.")

    if args.frame_source_port is not None:
        # camera_bridge_windows.py does the real cap.read() on Windows;
        # this process only receives already-captured JPEG frames over
        # localhost TCP -- see TcpFrameSource's docstring for why (USB
        # camera passthrough into WSL2 measured ~80-113ms/frame, unusable).
        cap = TcpFrameSource(args.frame_source_port)
        print("Frame source: TCP receiver (camera_bridge_windows.py) -- "
              "resolution/FPS are controlled on the Windows sender side.")
    else:
        cap = open_camera(args.camera, preferred_backend=args.backend)
        if cap is None:
            raise SystemExit(f"Could not open camera index {args.camera}; "
                             "try apriltag_detect.py --list-cameras")
        if args.width and args.height:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if args.fps:
            cap.set(cv2.CAP_PROP_FPS, args.fps)
        print(f"Capture resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        if args.fps:
            granted_fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"Capture FPS: requested {args.fps}, camera reports {granted_fps:.1f}"
                  + ("  <-- did not grant the request, check camera/driver support"
                     if granted_fps <= 0 or abs(granted_fps - args.fps) > 1.0 else ""))

    detector = build_detector(args.family, args.decimate, args.threads)
    calib = TableCalibration(tag_size, freeze_after_n=args.freeze_calib)
    if args.freeze_calib > 0:
        print(f"Calibration will FREEZE after averaging {args.freeze_calib} "
              "observations (T2 benchmark mode) -- press 'r' to reset and "
              "recollect if needed.")

    log_file = None
    log_writer = None
    if args.log:
        log_file = open(args.log, "a", newline="")
        log_writer = csv.writer(log_file)
        if log_file.tell() == 0:
            log_writer.writerow(["time_s", "tag_id", "x_in", "y_in", "yaw_deg",
                                 "grid_x", "grid_y"])

    print(f"World frame: origin at table corner by tag 20, +x toward tag 21 side, "
          f"+y toward tag 22 side, units inches. Ref tags: {sorted(REF_TAG_WORLD)}.")

    if not args.no_preview:
        # full-resolution 1:1 preview (user's screen is 1920x1200), but
        # WINDOW_NORMAL keeps it draggable/resizable if that ever changes
        cv2.namedWindow("AprilTag localization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag localization", args.width, args.height)

    last_print = 0.0
    last_publish = 0.0
    last_stream = 0.0
    publish_interval = 1.0 / max(args.publish_rate, 0.1)
    calib_announced = False
    frame_seq = 0

    # Instrumentation (2026-07-28 benchmark matrix, T0): split what used to
    # be one bundled "avg detect" number (detector.detect() + calib.update()
    # together) into every stage the implementation review flagged, using
    # perf_counter() (monotonic, immune to wall-clock adjustments -- time.time()
    # was used before only because it was already needed for the publish-rate
    # gate, not because it's the right clock for durations) and RollingStats
    # (median/p95/max/rate) instead of a running average, since the original
    # bug symptom -- alternating ~38Hz/~11Hz -- is exactly the kind of
    # occasional-slow-frame behavior an average hides.
    stats = {
        "read": RollingStats(), "cvt": RollingStats(),
        "apriltag": RollingStats(), "calibration": RollingStats(),
        "pose_math": RollingStats(), "publish_enqueue": RollingStats(),
        "total_loop": RollingStats(),
    }
    bench_since = time.perf_counter()
    try:
        while True:
            _loop_t0 = time.perf_counter()
            ok, frame = cap.read()
            _t_read = time.perf_counter()
            if not ok:
                print("Frame grab failed, retrying...")
                time.sleep(0.1)
                continue
            stats["read"].add(_t_read - _loop_t0)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _t_cvt = time.perf_counter()
            stats["cvt"].add(_t_cvt - _t_read)

            detections = detector.detect(gray)
            _t_apriltag = time.perf_counter()
            stats["apriltag"].add(_t_apriltag - _t_cvt)

            calib.update(detections)
            _t_calib = time.perf_counter()
            stats["calibration"].add(_t_calib - _t_apriltag)

            frame_seq += 1
            now = time.time()
            poses: dict[int, tuple[float, float, float]] = {}
            if calib.H is not None:
                if not calib_announced:
                    print(f"Calibration locked: refs {calib.used_ids}, "
                          f"rms {calib.rms_in:.2f} in"
                          + ("  <-- HIGH, check tag size / measurements!"
                             if calib.rms_in > 1.0 else ""))
                    calib_announced = True
                for det in detections:
                    if det.tag_id not in REF_TAG_WORLD:
                        poses[det.tag_id] = tag_world_pose(calib.H, det)
                if log_writer is not None:
                    for tid, (x, y, yaw) in sorted(poses.items()):
                        gx, gy = world_to_grid(x, y)
                        log_writer.writerow([f"{now:.3f}", tid, f"{x:.2f}", f"{y:.2f}",
                                             f"{yaw:.1f}", f"{gx:.3f}", f"{gy:.3f}"])
            _t_pose_math = time.perf_counter()
            stats["pose_math"].add(_t_pose_math - _t_calib)

            if (publisher is not None and calib.H is not None and poses
                    and now - last_publish >= publish_interval):
                enqueue_sec = publisher.publish(poses, calib, now)
                stats["publish_enqueue"].add(enqueue_sec)
                last_publish = now

            if now - last_print >= args.print_interval:
                if poses:
                    print("; ".join(
                        f"{ROBOT_NAMES.get(tid, f'id {tid}')}: "
                        f"x={x:.1f} y={y:.1f} yaw={yaw:+.1f} "
                        f"grid=({world_to_grid(x, y)[0]:+.2f},{world_to_grid(x, y)[1]:+.2f})"
                        for tid, (x, y, yaw) in sorted(poses.items())))
                    last_print = now
                elif calib.H is None and detections:
                    print(f"seen tags {sorted(d.tag_id for d in detections)}; "
                          "waiting for >=2 corner tags (20-23) to calibrate")
                    last_print = now

            if not args.no_preview or streamer is not None:
                draw_overlay(frame, calib, detections, poses)
            if streamer is not None and now - last_stream >= 0.1:  # ~10 fps
                streamer.push(frame)
                last_stream = now
            if not args.no_preview:
                cv2.imshow("AprilTag localization", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    calib.reset()
                    calib_announced = False
                    print("Calibration reset.")

            stats["total_loop"].add(time.perf_counter() - _loop_t0)
            elapsed = time.perf_counter() - bench_since
            if elapsed >= args.bench_interval:
                print(f"bench: seq={frame_seq}  "
                      f"read[{stats['read'].summary(elapsed)}]  "
                      f"cvt[{stats['cvt'].summary(elapsed)}]  "
                      f"apriltag[{stats['apriltag'].summary(elapsed)}]  "
                      f"calibration[{stats['calibration'].summary(elapsed)}]  "
                      f"pose_math[{stats['pose_math'].summary(elapsed)}]  "
                      f"publish_enqueue[{stats['publish_enqueue'].summary(elapsed)}]  "
                      f"total_loop[{stats['total_loop'].summary(elapsed)}]")
                for s in stats.values():
                    s.reset()
                bench_since = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if log_file is not None:
            log_file.close()
        if publisher is not None:
            publisher.close()
        if streamer is not None:
            streamer.close()


if __name__ == "__main__":
    main()
