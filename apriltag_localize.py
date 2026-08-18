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
import struct
import threading
import time
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from pupil_apriltags import Detector

from apriltag_detect import open_camera

# Must match camera_bridge_windows.py's CROP_MAGIC exactly -- this is the
# wire-format contract for the crop back-channel (see that file's module
# docstring, and TcpFrameSource.send_crop() below).
CROP_MAGIC = b"CROP"


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

    def __init__(self, port: int, host: str = "127.0.0.1", connect_timeout: float = 10.0,
                 diag: bool = False):
        print(f"TcpFrameSource: connecting to camera_bridge_windows.py at "
              f"{host}:{port}...")
        self._conn = socket.create_connection((host, port), timeout=connect_timeout)
        self._conn.settimeout(None)
        print("TcpFrameSource: connected.")
        self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # NOTE 2026-08-05: a larger SO_RCVBUF was tried here during the
        # isaac_ros_apriltag_gpu throughput investigation (recv() was
        # measured taking ~56 small calls per frame, theorized as
        # buffer-starvation) -- TESTED AND DISPROVEN with a genuine 8MB
        # buffer (confirmed actually granted, not kernel-capped): call count
        # barely changed (56->36) and total time didn't improve (stayed
        # ~34-35ms). Real bottleneck is still unexplained -- see memory:
        # isaac_ros_apriltag_gpu for the full investigation (rate-limiter
        # bug found+fixed elsewhere, decode/network/sender/AprilTagNode all
        # independently ruled out, buffer size also ruled out). Don't
        # re-try this fix without new evidence.
        self._last_frame: np.ndarray | None = None
        # Opt-in fine-grained internal timing (2026-08-05, isaac_ros_apriltag_gpu
        # throughput investigation continued): read()'s own --diag bucket in
        # isaac_ros_image_publisher.py wraps recv AND imdecode AND the
        # backlog-drain loop together as one number, which can't distinguish
        # "network/OS is slow to deliver bytes" from "this process is slow to
        # decode JPEGs" from "backlog piled up because this process fell
        # behind the sender's 60fps for some OTHER reason (e.g. Python
        # scheduling)". These are separated out here so the real cause can be
        # read off directly instead of guessed at.
        self._diag = diag
        if self._diag:
            self._diag_recv_ms: list[float] = []
            self._diag_decode_ms: list[float] = []
            self._diag_drained: list[int] = []

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
        t0 = time.perf_counter() if self._diag else 0.0
        data = self._recv_one_frame()
        if data is None:
            return False, None
        drained = 0
        while True:
            ready, _, _ = select.select([self._conn], [], [], 0.0)
            if not ready:
                break
            newer = self._recv_one_frame()
            if newer is None:
                break
            data = newer
            drained += 1
        t1 = time.perf_counter() if self._diag else 0.0
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return False, None
        if self._diag:
            t2 = time.perf_counter()
            self._diag_recv_ms.append((t1 - t0) * 1000.0)
            self._diag_decode_ms.append((t2 - t1) * 1000.0)
            self._diag_drained.append(drained)
        self._last_frame = frame
        return True, frame

    def diag_summary(self) -> str | None:
        """Opt-in (diag=True) breakdown of read()'s internal cost since the
        last call to this method -- see the NOTE in read() for why recv/
        decode/drain-count are tracked separately. Returns None (and resets
        nothing) if diag=False or no reads have happened yet."""
        if not self._diag or not self._diag_recv_ms:
            return None

        def _fmt(xs: list[float]) -> str:
            s = sorted(xs)
            n = len(s)
            return (f"median={s[n//2]:.1f}ms p95={s[min(int(0.95*n), n-1)]:.1f}ms "
                    f"max={s[-1]:.1f}ms")

        n = len(self._diag_recv_ms)
        drained_total = sum(self._diag_drained)
        summary = (f"n={n}  recv={_fmt(self._diag_recv_ms)}  "
                   f"decode={_fmt(self._diag_decode_ms)}  "
                   f"drained_total={drained_total} "
                   f"({drained_total/n:.1f}/read)")
        self._diag_recv_ms.clear()
        self._diag_decode_ms.clear()
        self._diag_drained.clear()
        return summary

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

    def send_crop(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Tell camera_bridge_windows.py to crop every subsequent frame to
        this pixel rect (in the FULL uncropped frame's coordinates) before
        encoding -- see that file's module docstring for the wire format
        and why (2026-08-05 throughput investigation). Call with all-zero
        args to reset back to full-frame uncropped mode. This is the only
        thing ever sent FROM this side TO the sender on this socket."""
        cmd = CROP_MAGIC + struct.pack(">HHHH", x0, y0, x1, y1)
        self._conn.sendall(cmd)

    def reset_crop(self) -> None:
        self.send_crop(0, 0, 0, 0)

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
# matching Alvik (i.e. tag 1 on the robot publishing Alvik1_pose). Tags 5/6
# added 2026-08-03 (unverified against the physical stickers -- confirm tag 5
# is really on Alvik5 and tag 6 on Alvik6 before trusting this mapping).
ROBOT_NAMES: dict[int, str] = {
    1: "Alvik1", 2: "Alvik2", 3: "Alvik3", 4: "Alvik4", 5: "Alvik5", 6: "Alvik6",
}

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

# ---- depot slots / entries / node 0 (measured 2026-07-30) ----
# Each robot's AprilTag read directly off apriltag_localize.py's own preview
# overlay while physically parked -- NOT extrapolated from a fixed pitch.
# CONFIRMED this matters: the dashboard HTML (agv_grid_workstation_solver.html)
# assumes a uniform DEPOT_SLOT_PITCH=0.6 grid cells (6.0in) between slots,
# but real measured spacing is ~5.0-5.4in for slots 1-4, widening to
# 6.3-7.3in for slots 5-6 -- extrapolating from the nominal pitch would have
# put D5/D6 measurably wrong. D1/DE1 here are close to (not identical to)
# camera_grid_navigate.py's own DEPOT_WORLD_IN[-1]/[-2] (19.7,10.3)/(19.6,3.0)
# -- small difference is expected parking precision, not a contradiction.
# Two separate passes: all 6 robots parked at D1-D6 for one frame, then all
# 6 moved to DE1-DE6 for a second frame (robots can't occupy both at once).
# node 0 measured the same way (Alvik1 driven onto it, facing west/-x).
DEPOT_SLOT_WORLD_IN: dict[str, tuple[float, float]] = {
    "D1": (20.0, 10.5), "D2": (25.3, 10.7), "D3": (30.3, 10.8),
    "D4": (35.7, 10.7), "D5": (42.0, 10.6), "D6": (49.3, 10.6),
}
DEPOT_ENTRY_WORLD_IN: dict[str, tuple[float, float]] = {
    "DE1": (19.6, 3.0), "DE2": (24.9, 3.0), "DE3": (30.2, 3.3),
    "DE4": (35.4, 3.2), "DE5": (41.9, 3.2), "DE6": (49.1, 3.2),
}
NODE0_WORLD_IN: tuple[float, float] = (13.3, 2.9)


def build_depot_overlay_points() -> list[tuple[str, float, float]]:
    """Depot slots, depot entries, and node 0 for the --show-nodes ('o')
    overlay -- added 2026-07-30 alongside the lattice/workstation/entry
    points from build_node_overlay_points(). Kept as a SEPARATE list with
    STRING ids (D1, DE1, 0) rather than folded into that function's
    integer-id list, since depot labels aren't plain grid-node numbers and
    nothing downstream (sticker detection, click diagnostics) needs to
    treat them the same way lattice/bay nodes are treated."""
    out: list[tuple[str, float, float]] = [("0", NODE0_WORLD_IN[0], NODE0_WORLD_IN[1])]
    for label, (x, y) in DEPOT_SLOT_WORLD_IN.items():
        out.append((label, x, y))
    for label, (x, y) in DEPOT_ENTRY_WORLD_IN.items():
        out.append((label, x, y))
    return out


def world_to_grid(x_in: float, y_in: float) -> tuple[float, float]:
    return ((x_in - GRID_NODE1_WORLD_IN[0]) / GRID_PITCH_IN,
            (y_in - GRID_NODE1_WORLD_IN[1]) / GRID_PITCH_IN)


def node_to_world(n: int, rows: int, cols: int) -> tuple[float, float]:
    """Grid/workstation node number -> world (x_in, y_in). Ported from
    fleet/camera_grid_navigate.py's node_to_world() 2026-07-28 for the
    --show-nodes overlay -- KEEP IN SYNC with that copy (same obligation as
    GRID_NODE1_WORLD_IN/GRID_PITCH_IN above, which already mirror it) if the
    node numbering or bay offsets ever change. See that function's own
    docstring for the full numbering scheme description:
      1..rows*cols                          lattice nodes (RED marker)
      rows*cols+1 .. rows*cols+bays          WORKSTATION nodes (dead-end
                                              spur past the entry, YELLOW)
      rows*cols+bays+1 .. rows*cols+2*bays   ENTRY nodes (bay mouth, YELLOW)
    where bays = (rows-1)*(cols-1). Node 1 = bottom-left lattice point."""
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


def build_node_overlay_points(
        rows: int, cols: int) -> list[tuple[int, float, float, bool]]:
    """All node ids/world positions for the --show-nodes overlay, tagged
    is_workstation_or_entry (True for both the workstation dead-end AND its
    entry -- both drawn yellow per the user's request; only plain lattice
    nodes are drawn red)."""
    nodes = rows * cols
    bays = (rows - 1) * (cols - 1)
    out = []
    for n in range(1, nodes + 2 * bays + 1):
        x, y = node_to_world(n, rows, cols)
        out.append((n, x, y, n > nodes))
    return out


# ---- floor-sticker color thresholds (2026-07-28, --show-stickers) --------
# Measured directly from the overhead camera view via apriltag_localize.py's
# own click-to-inspect HSV mode (added same day specifically for this,
# printing OpenCV-convention HSV -- H 0-179 -- for whatever pixel is
# clicked), NOT reused from color_sticker_test.py's thresholds, which are
# from the Alvik's onboard color sensor (different hardware, different HSV
# normalization -- not comparable to an overhead webcam's OpenCV HSV).
#
# Three earlier rounds of small-batch sampling (4, then +5, then +5 per
# class) each looked fine against their own sample but caused real
# regressions when tested against the full table -- lighting varies enough
# across an 8x8 grid (different rows, shadows, glare) that a handful of
# clicks from one area doesn't generalize. Final round: FULL sweep --
# every visible red/yellow/blue sticker on the table (73 red, 24 yellow,
# 18 blue total across all rounds) plus tape sampled densely across all 8
# rows (69 tape samples total). Thresholds below were found by a grid
# search over (s_floor, v_floor) requiring "s>=s_floor OR v>=v_floor" to
# hold for every real sample of that color and fail for every tape sample
# in the same hue band, then VERIFIED PROGRAMMATICALLY against the full
# combined dataset (not eyeballed) -- see the search/verification script
# used to derive these (not checked in; ad hoc), or re-derive from fresh
# click-to-inspect data if lighting changes enough to need retuning.
#   RED:    hue 0-17 / 171-179, needs s>=34 (v floor unused -- saturation
#           alone cleanly separates all 73 red samples, s>=61, from every
#           tape sample in this hue band, s<=33)
#   YELLOW: hue 21-36, needs s>=54 OR v>=214 (the one class that needed
#           the OR -- no single floor on either axis alone separates all
#           24 yellow samples from all tape samples in this hue band;
#           closest tape sample s=53,v=53, closest yellow sample s=57 or
#           v=165, only ~3-4 units of real margin on the tightest side)
#   BLUE:   hue 102-109, needs v>=158 (s floor unused -- value alone
#           cleanly separates all 18 blue samples, v>=170, from every tape
#           sample in this hue band, v<=152; closest tape sample
#           h=104,s=126,v=152 came within 18 of blue's v floor)
# If --show-stickers starts missing real stickers or picking up tape again
# after a lighting change, re-sample densely (not just a few clicks) --
# small samples have twice now looked fine locally and failed at scale.
STICKER_HSV_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    # red hue wraps around 0/179 in OpenCV's convention -- two ranges ORed.
    "RED": [((0, 34, 0), (17, 255, 255)), ((171, 34, 0), (179, 255, 255))],
    "YELLOW": [((21, 54, 0), (36, 255, 255)), ((21, 0, 214), (36, 255, 255))],
    "BLUE": [((102, 0, 158), (109, 255, 255))],
}
STICKER_DRAW_BGR = {
    "RED": (60, 60, 255), "YELLOW": (60, 255, 255), "BLUE": (255, 140, 60),
}
STICKER_MIN_AREA_PX = 40  # rejects single-pixel noise; a real sticker blob is much larger


def detect_stickers(frame, table_mask=None) -> list[tuple[str, int, int, int]]:
    """Find red/yellow/blue floor-sticker blobs in a single BGR frame.
    Returns (color_name, cx, cy, area_px) per detected blob, largest first
    within each color. Pure color/contour detection -- no homography
    involved in the detection itself, but table_mask (an optional single-
    channel uint8 mask, same HxW as frame, 255 inside the table / 0 outside)
    restricts the SEARCH AREA to the table interior.

    Added 2026-07-28 (same day as the first version): without a mask, chair
    frames, ceiling beams, a whiteboard, and clothing in the background all
    matched the same HSV ranges as real stickers -- confirmed on hardware,
    rings showed up scattered well outside the table entirely. The table
    boundary (the same corner homography already used for the blue border
    overlay) is a natural, already-available mask -- build it once via
    build_table_mask() and pass it in, rather than tightening HSV ranges
    trying to exclude backgrounds that just happen to share a hue.

    This is what --show-stickers freezes ONCE on the first real frame (see
    main()'s show_stickers handling) rather than re-running every frame,
    per the user's explicit request 2026-07-28."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    found: list[tuple[str, int, int, int]] = []
    for color, ranges in STICKER_HSV_RANGES.items():
        mask = None
        for lo, hi in ranges:
            part = cv2.inRange(hsv, np.array(lo, dtype=np.uint8),
                                np.array(hi, dtype=np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        if table_mask is not None:
            mask = cv2.bitwise_and(mask, table_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                 np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < STICKER_MIN_AREA_PX:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            blobs.append((color, cx, cy, int(area)))
        blobs.sort(key=lambda b: -b[3])
        found.extend(blobs)
    return found


STICKER_WINDOW_RADIUS_PX_DEFAULT = 32  # half-width of the per-node search box
# RAISED from 22 to 32 (2026-07-30, same day) after a real hardware miss: a
# clicked sticker's HSV (h=3-6,s=111-117,v=143-144) was confirmed well
# inside the RED range, yet its node's 22px window still found nothing --
# not a color-threshold problem, a window-too-tight/alignment problem. 32px
# gives more tolerance for small offsets between the projected node
# position and the real sticker's physical center without reopening the
# whole-table false-positive problem the windowed approach was built to fix.

# Separate, LOWER area threshold for windowed search than the whole-table
# STICKER_MIN_AREA_PX (40) -- CONFIRMED 2026-07-30 via --debug-sticker-nodes
# on real hardware that 40 was rejecting genuine stickers: node 155's real
# yellow sticker measured only 27px (mask had 78 matching pixels before
# MORPH_OPEN eroded it to 40, single contour then 27 -- MORPH_OPEN also
# removed, see the loop below, so this only needs to reject genuine noise,
# not compensate for erosion loss too). The small search window itself
# (64x64px = 4096px^2 total) already does most of the noise-rejection work
# a large area threshold exists for in a whole-frame scan -- it doesn't
# need to ALSO be large here. 15 gives real margin below the smallest
# confirmed-real blob (27) without going so low that single-pixel sensor
# noise could pass; re-derive from more --debug-sticker-nodes samples if
# real misses persist below this.
STICKER_WINDOW_MIN_AREA_PX = 15

# Which sticker color(s) to search for, per node type -- searching every
# color at every node (the first version of this function) wastes cycles
# and adds needless cross-color false-match risk. Matches the SAME
# red-lattice/yellow-bay convention build_node_overlay_points() already
# uses for the 'o' overlay's own circle coloring.
STICKER_COLORS_FOR_LATTICE = ["RED"]
STICKER_COLORS_FOR_BAY = ["YELLOW"]


def detect_stickers_at_nodes(
        frame, node_overlay_pixels, window_radius_px: int = STICKER_WINDOW_RADIUS_PX_DEFAULT,
        debug_node_ids: set[int] | None = None,
) -> list[tuple[int, str | None, int, int, int]]:
    """Added 2026-07-30, replaces whole-table detect_stickers() for the 'u'
    overlay: instead of scanning the ENTIRE table for color blobs and hoping
    the HSV thresholds alone separate real stickers from tape/glare/shadow
    noise, search only a small window around each node's already-known
    pixel position (node_overlay_pixels -- the exact same homography
    projection that makes the 'o' overlay accurate, see
    build_node_overlay_points()/main()'s Hinv_nodes block). This is a much
    easier detection problem: "which color, if any, is centered near this
    known point" instead of "find every color blob anywhere and guess which
    ones are real."

    WHY: on real hardware (2026-07-30, controlled/even lighting after a room
    change) the whole-table version was still producing false positives
    (color blobs matching in areas with no real sticker) AND false negatives
    (missing several real top-row stickers, e.g. nodes 57/59/61/63/64) at
    the same time -- both symptoms of the same root cause, a detector with
    no positional prior trying to do everything through color thresholds
    alone.

    SEARCHES EVERY NODE (lattice AND workstation/entry, CHANGED same day
    from an earlier lattice-only version) -- the first version excluded
    is_bay nodes assuming they had no floor stickers, but real hardware
    screenshots showed yellow stickers inside several workstation bay
    cutouts. Plain lattice nodes are searched for RED only, workstation/
    entry (is_bay) nodes for YELLOW only (STICKER_COLORS_FOR_LATTICE /
    STICKER_COLORS_FOR_BAY) -- matches the existing red-lattice/
    yellow-bay convention, and searching only the relevant color per node
    is both faster and less prone to a stray cross-color match than
    checking all three colors everywhere.

    Returns one entry per node: (node_id, color_or_None, cx, cy, area_px).
    color is None (cx/cy/area_px then 0) when NO sticker-colored blob was
    found inside that node's window at all -- every node is expected to
    have a real sticker, so a None here is a genuine missing/occluded/worn
    sticker worth flagging, not filtered out."""
    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    results: list[tuple[int, str | None, int, int, int]] = []
    for n, px, py, is_bay in node_overlay_pixels:
        colors_to_check = STICKER_COLORS_FOR_BAY if is_bay else STICKER_COLORS_FOR_LATTICE
        cx0, cy0 = int(round(px)), int(round(py))
        x0 = max(0, cx0 - window_radius_px)
        x1 = min(w, cx0 + window_radius_px)
        y0 = max(0, cy0 - window_radius_px)
        y1 = min(h, cy0 + window_radius_px)
        if x1 <= x0 or y1 <= y0:
            results.append((n, None, 0, 0, 0))
            continue
        window_hsv = hsv_full[y0:y1, x0:x1]

        debug = debug_node_ids is not None and n in debug_node_ids
        best_color, best_area, best_cx, best_cy = None, 0, 0, 0
        for color in colors_to_check:
            ranges = STICKER_HSV_RANGES[color]
            mask = None
            for lo, hi in ranges:
                part = cv2.inRange(window_hsv, np.array(lo, dtype=np.uint8),
                                    np.array(hi, dtype=np.uint8))
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask_pixels = int(cv2.countNonZero(mask))
            # NO MORPH_OPEN here (removed 2026-07-30) -- CONFIRMED via
            # --debug-sticker-nodes on real hardware to erode real sticker
            # blobs below STICKER_MIN_AREA_PX: node 155's real yellow
            # sticker had 78 matching pixels before opening, 40 after, and
            # its single surviving contour measured only 27px -- rejected
            # by the (old, whole-table-tuned) 40px threshold. Opening exists
            # to remove SCATTERED single-pixel noise across an entire frame
            # scan; inside an already-small, already-color-matched search
            # window that noise-rejection role is redundant (the window
            # itself is the noise filter) and the erosion cost is real.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            if debug:
                areas = sorted((cv2.contourArea(c) for c in contours), reverse=True)
                print(f"    [debug node {n}] color={color} window=({x0},{y0})-({x1},{y1}) "
                      f"raw_mask_px={mask_pixels} contour_areas={areas} "
                      f"(threshold={STICKER_WINDOW_MIN_AREA_PX})")
            for c in contours:
                area = cv2.contourArea(c)
                if area < STICKER_WINDOW_MIN_AREA_PX or area <= best_area:
                    continue
                M = cv2.moments(c)
                if M["m00"] <= 0:
                    continue
                best_color = color
                best_area = int(area)
                best_cx = x0 + int(M["m10"] / M["m00"])
                best_cy = y0 + int(M["m01"] / M["m00"])

        if best_color is None:
            results.append((n, None, 0, 0, 0))
        else:
            results.append((n, best_color, best_cx, best_cy, best_area))
    return results


def build_table_mask(frame_shape, H: np.ndarray) -> np.ndarray:
    """Single-channel uint8 mask (255 inside the table, 0 outside), built by
    projecting the table's 4 world corners through Hinv -- same corners
    already used for draw_overlay()'s blue border polygon, reused here so
    --show-stickers only searches inside the table instead of the whole
    frame (chairs/ceiling/whiteboard/etc)."""
    Hinv = np.linalg.inv(H)
    border = np.array([
        [0, 0], [TABLE_SIZE_IN, 0], [TABLE_SIZE_IN, TABLE_SIZE_IN], [0, TABLE_SIZE_IN],
    ], dtype=float)
    pixel_border = map_points(Hinv, border).astype(np.int32)
    mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [pixel_border], 255)
    return mask


def compute_crop_rect(frame_shape, H: np.ndarray,
                       margin_in: float = 6.0) -> tuple[int, int, int, int]:
    """Axis-aligned pixel bounding box around the table's 4 world corners
    (same corners build_table_mask() projects), padded by margin_in inches
    of real table-space margin on every side, clamped to the frame.

    Added 2026-08-05 (isaac_ros_apriltag_gpu throughput investigation): the
    crop back-channel sends this rect to camera_bridge_windows.py so it can
    encode/send only the table region instead of the full frame -- see that
    file's module docstring for why (WSL2 mirrored-loopback's real per-
    packet cost, not decode/buffer/scheduling, was the actual bottleneck).

    margin_in defaults to 6in, not 0 -- the fisheye lens means a tag exactly
    at the table's edge can still project outside a zero-margin bounding
    box if the homography's corner estimate is even slightly off, or if a
    robot's tag center is measured right at the boundary; 6in of real
    padding is cheap (small fraction of the ~97in table) and avoids
    silently clipping a valid detection."""
    h, w = frame_shape[:2]
    Hinv = np.linalg.inv(H)
    m = margin_in
    border = np.array([
        [-m, -m], [TABLE_SIZE_IN + m, -m],
        [TABLE_SIZE_IN + m, TABLE_SIZE_IN + m], [-m, TABLE_SIZE_IN + m],
    ], dtype=float)
    pixel_border = map_points(Hinv, border)
    x0 = int(np.clip(np.floor(pixel_border[:, 0].min()), 0, w))
    y0 = int(np.clip(np.floor(pixel_border[:, 1].min()), 0, h))
    x1 = int(np.clip(np.ceil(pixel_border[:, 0].max()), 0, w))
    y1 = int(np.clip(np.ceil(pixel_border[:, 1].max()), 0, h))
    return x0, y0, x1, y1


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


def draw_overlay(frame, calib: TableCalibration, detections, poses,
                  node_overlay_pixels=None, show_nodes: bool = False,
                  sticker_detections=None, show_stickers: bool = False,
                  depot_overlay_pixels=None) -> None:
    """node_overlay_pixels: optional list of (node_id, px, py, is_bay) in
    IMAGE pixel space, precomputed once (see build_node_overlay_points() +
    Hinv projection in main()) rather than every frame -- see --show-nodes'
    help text for why (user explicitly wants a one-time snapshot at first
    calibration lock, not live tracking of homography drift). Drawn as
    light (alpha-blended, not opaque) filled circles so the underlying
    video stays visible through them: red for plain lattice nodes, yellow
    for workstation + workstation-entry nodes. Only actually drawn when
    show_nodes is True -- toggled by the 'o' key in main()'s loop.

    sticker_detections: optional list of (node_id, color_or_None, cx, cy,
    area_px) from detect_stickers_at_nodes() (CHANGED 2026-07-30 from a
    free-floating color-blob list to a per-LATTICE-NODE result, searched in
    a small window around each node's already-known position -- see that
    function's docstring for why). ALSO a one-time freeze (first real frame
    after calibration locks, same as before), toggled by the 'u' key.
    color_or_None is None when that node's window found no matching sticker
    at all. For LATTICE nodes this is drawn as a thin gray dashed-look ring
    (every lattice node is expected to have a real sticker, so a miss is a
    genuine physical maintenance flag: missing, worn, or occluded). For BAY
    (workstation/entry) nodes a miss is NOT drawn at all and NOT a flag --
    confirmed 2026-07-30: only some workstations are "active" (have a real
    yellow sticker) at any given time, the rest are legitimately bare, so
    the gray-miss marker would otherwise flag ~74 correct non-detections on
    a 98-bay-node grid as if they were errors. A real match (active
    workstation) is drawn as a bright solid ring directly ON the detected
    position (not alpha-blended -- these mark real detections, not a
    coordinate projection), same for both node types."""
    if show_nodes and node_overlay_pixels:
        h, w = frame.shape[:2]
        node_layer = frame.copy()
        radius = 7
        for _n, px, py, is_bay in node_overlay_pixels:
            ipx, ipy = int(round(px)), int(round(py))
            if -radius <= ipx <= w + radius and -radius <= ipy <= h + radius:
                color = (0, 220, 220) if is_bay else (0, 0, 255)  # BGR: yellow / red
                cv2.circle(node_layer, (ipx, ipy), radius, color, -1)
        # Depot slots/entries/node 0 (added 2026-07-30, real measured
        # positions -- see DEPOT_SLOT_WORLD_IN/DEPOT_ENTRY_WORLD_IN/
        # NODE0_WORLD_IN) -- same alpha-blended-circle-then-opaque-label
        # treatment as the lattice/bay nodes above, drawn in the SAME pass
        # (same node_layer/blend) so they don't need their own toggle --
        # showing/hiding with show_nodes ('o') exactly like every other
        # node type. Blue, matching this project's established "depot ==
        # blue" convention (blue floor stickers mark the depot lane).
        if depot_overlay_pixels:
            for _label, px, py in depot_overlay_pixels:
                ipx, ipy = int(round(px)), int(round(py))
                if -radius <= ipx <= w + radius and -radius <= ipy <= h + radius:
                    cv2.circle(node_layer, (ipx, ipy), radius, (255, 140, 60), -1)  # BGR blue
        cv2.addWeighted(node_layer, 0.35, frame, 0.65, 0, dst=frame)
        # Labels drawn AFTER the blend (full opacity, not alpha-blended like
        # the circles) and offset below the circle rather than centered on
        # it, so digits don't sit on top of the (still fairly busy) tape
        # graphics -- confirmed too small/cramped at the original scale=0.35.
        for n, px, py, _is_bay in node_overlay_pixels:
            ipx, ipy = int(round(px)), int(round(py))
            if -radius <= ipx <= w + radius and -radius <= ipy <= h + radius:
                label = str(n)
                tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
                _text(frame, label, (ipx - tw // 2, ipy + radius + 14),
                      (255, 255, 255), scale=0.42)
        if depot_overlay_pixels:
            for label, px, py in depot_overlay_pixels:
                ipx, ipy = int(round(px)), int(round(py))
                if -radius <= ipx <= w + radius and -radius <= ipy <= h + radius:
                    tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
                    _text(frame, label, (ipx - tw // 2, ipy + radius + 14),
                          (255, 220, 150), scale=0.42)  # light blue-ish, matches the ring

    # MOVED 2026-07-30 to draw AFTER the 'o' node overlay above (was
    # before it) -- CONFIRMED on real hardware this was hiding real,
    # correctly-detected stickers: node 88's detection was genuine (same
    # frozen sticker_detections data the click-to-inspect diagnostic reads,
    # confirmed matching YELLOW every time) but its bright ring was being
    # visually buried under the 'o' overlay's own semi-transparent dot +
    # full-opacity "88" text label at the exact same pixel position, when
    # both overlays were toggled on together. A real sticker detection
    # should always be visible regardless of whether the node-position
    # overlay is also on, so it now draws last (on top).
    if show_stickers and sticker_detections:
        # MISSING markers (color is None) carry no real detected position
        # (detect_stickers_at_nodes() returns cx=cy=0 for those, since
        # nothing was found) -- look the node's own KNOWN position AND
        # is_bay back up from node_overlay_pixels instead, so the gray ring
        # lands on the node (not the frame's top-left corner) and bay
        # misses can be skipped entirely (see docstring above).
        node_info_by_id = {n: (px, py, is_bay)
                            for n, px, py, is_bay in (node_overlay_pixels or [])}
        for n, color, cx, cy, _area in sticker_detections:
            if color is None:
                info = node_info_by_id.get(n)
                if info is None:
                    continue
                pos_x, pos_y, is_bay = info
                if is_bay:
                    continue  # inactive workstation -- expected, not a flag
                gx, gy = int(round(pos_x)), int(round(pos_y))
                # Dashed-look ring: short arcs instead of a full circle,
                # thin and gray so it reads as "nothing found here" rather
                # than competing visually with a real solid detection ring.
                gray = (140, 140, 140)
                for start_deg in range(0, 360, 45):
                    cv2.ellipse(frame, (gx, gy), (10, 10), 0,
                                start_deg, start_deg + 25, gray, 1)
            else:
                ring_bgr = STICKER_DRAW_BGR.get(color, (255, 255, 255))
                cv2.circle(frame, (cx, cy), 12, ring_bgr, 2)
                cv2.circle(frame, (cx, cy), 2, ring_bgr, -1)

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
    # Default raised 1.0 -> 2.0 (2026-08-18) after a real 6-robot hardware
    # A/B: 1.0 gave total_loop rate ~20-26Hz (apriltag median ~20ms), 1.5 gave
    # ~29-34Hz (~9-10ms), 2.0 gave ~30-38Hz (~6-7ms) -- all three held
    # calibration rms at 0.06in and showed no pose jitter/dropout increase.
    # 2.0 does trigger pupil_apriltags' internal "WRN: Matrix is singular."
    # far more often (dozens/run vs 1-2 at 1.5) -- traced to refine_edges=1's
    # per-tag corner-refinement solve failing to converge more often from a
    # coarser initial estimate; confirmed cosmetic (falls back to the
    # unrefined corner for that one tag/frame, doesn't touch calib.H, never
    # crashed the loop, no corresponding pose degradation observed). If a
    # future accuracy regression shows up, try 1.5 (same speed tier, far
    # fewer refinement fallbacks) before going back to 1.0.
    parser.add_argument("--decimate", type=float, default=2.0,
                        help="detector quad_decimate; lower toward 1.0 if accuracy "
                             "matters more than fps (default 2.0)")
    parser.add_argument("--print-interval", type=float, default=0.5,
                        help="seconds between console pose lines (default 0.5)")
    parser.add_argument("--log", default=None, help="append poses to this CSV file")
    parser.add_argument("--no-preview", action="store_true", help="headless: console output only")
    parser.add_argument("--rows", type=int, default=8,
                        help="grid rows, for the --show-nodes overlay -- "
                             "must match fleet/camera_grid_navigate.py's "
                             "--rows for the same table (default 8)")
    parser.add_argument("--cols", type=int, default=8,
                        help="grid cols, for the --show-nodes overlay -- "
                             "must match fleet/camera_grid_navigate.py's "
                             "--cols for the same table (default 8)")
    parser.add_argument("--show-nodes", action="store_true",
                        help="start with the node-position overlay ON "
                             "(light red circles at lattice nodes, light "
                             "yellow at workstation/entry nodes). Toggle "
                             "any time with the 'o' key in the preview "
                             "window (ignored with --no-preview, since "
                             "there's no window to press a key in). "
                             "Positions are computed ONCE, frozen at "
                             "whatever calib.H is the first frame "
                             "calibration locks -- they do not track later "
                             "homography refinement/recalibration ('r') "
                             "until the script is restarted.")
    parser.add_argument("--show-stickers", action="store_true",
                        help="start with the floor-sticker overlay ON "
                             "(bright rings on detected red/yellow/blue "
                             "stickers). Toggle any time with the 'u' key "
                             "(ignored with --no-preview). Unlike "
                             "--show-nodes (homography-projected node "
                             "coordinates), this is REAL color-blob "
                             "detection run once on the first real frame "
                             "after calibration locks and then frozen -- "
                             "it visually marks whatever stickers are "
                             "physically on the table, not a coordinate "
                             "grid. Thresholds measured 2026-07-28 via "
                             "click-to-inspect HSV sampling of real "
                             "stickers/tape -- see STICKER_HSV_RANGES if "
                             "detections look wrong on different lighting.")
    parser.add_argument("--debug-sticker-nodes", type=int, nargs="*", default=None,
                        metavar="NODE_ID",
                        help="added 2026-07-30: for these specific node IDs "
                             "(space-separated), print every candidate "
                             "sticker-color contour found inside that "
                             "node's search window -- including ones "
                             "REJECTED by STICKER_MIN_AREA_PX -- so a real "
                             "miss (color detected but rejected/too small) "
                             "can be told apart from a genuine absence "
                             "(no matching pixels at all), instead of "
                             "guessing. e.g. --debug-sticker-nodes 63 155")
    parser.add_argument("--rosbridge", default=None, metavar="HOST[:PORT]",
                        nargs="?", const="192.168.0.212:9090",
                        help="publish poses to a rosbridge websocket on the ROS2 "
                             "laptop; bare --rosbridge uses the lab default "
                             "192.168.0.212:9090")
    # Default raised 30 -> 1000 (2026-08-18) after a real hardware finding:
    # publish_interval = 1/publish_rate gates EVERY publish call below, and
    # when that interval is even close to the loop's real period, ordinary
    # timer jitter causes it to silently skip publishes on iterations that
    # land slightly early -- confirmed via `ros2 topic hz /Alvik1_vision_pose`
    # on the actual subscriber (not just this process's own diagnostic
    # print): --publish-rate 60 with a real ~35-40Hz loop was topping out
    # around 210 total pose/sec (~35Hz/robot across 6 robots), matching the
    # OLDER 2026-07-27 comment on RosbridgePublisher's _publish_count (~2.5Hz
    # arrival despite --publish-rate 60) -- same underlying gate-vs-jitter
    # bug, just less visible at the time. Raising to 1000 (interval ~1ms)
    # makes the gate never bind at any loop rate we've measured (up to
    # ~60Hz), so every iteration's pose actually gets published -- confirmed
    # jumped straight to ros2 topic hz reporting ~59Hz/robot. There is no
    # real reason to cap this below the loop's own rate; rosbridge/network
    # was never the bottleneck here, the gate itself was.
    parser.add_argument("--publish-rate", type=float, default=1000.0,
                        help="rosbridge publish rate in Hz (default 1000, "
                             "i.e. effectively unthrottled -- capture FPS and "
                             "publish rate are independent; a publish-rate "
                             "gate close to the loop's real rate silently "
                             "drops publishes to timer jitter, so keep this "
                             "comfortably above whatever total_loop rate "
                             "the bench: line reports).")
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
    parser.add_argument("--test-local-crop", action="store_true",
                        help="Added 2026-08-05 (isaac_ros_apriltag_gpu "
                             "throughput investigation): once the table "
                             "homography first locks, crop every subsequent "
                             "frame to the table's pixel bounding box "
                             "(compute_crop_rect(), same margin/math as the "
                             "Isaac pipeline's auto-crop) PURELY LOCALLY --"
                             "no network protocol, no camera_bridge_windows.py "
                             "changes, just an in-process numpy slice after "
                             "cap.read(). Safe to test here since "
                             "pupil_apriltags has no persistent GPU buffer to "
                             "mismatch (unlike AprilTagNode's CUDA decoder, "
                             "which crashed on this exact kind of runtime "
                             "resize -- see memory). Validates the crop-rect "
                             "math and coordinate-offset correction in "
                             "isolation before reintroducing the sender-side "
                             "network crop.")
    # --remote-crop (a real sender-side crop via TcpFrameSource.send_crop())
    # was added 2026-08-21 and REVERTED the same day, per explicit user
    # direction, after two problems on real hardware: (1) a real bug --
    # the local frame[cy0:cy1,cx0:cx1] slice below ran a SECOND time on an
    # already-server-cropped (smaller) frame, double-shifting the origin
    # and leaving every detection under-corrected by one crop_offset --
    # every robot's reported yaw was off by ~20deg, and the on-screen
    # bounding boxes visibly did not line up with the actual robots; (2)
    # even before fully fixing that, live camera_bridge_windows.py --diag
    # output showed cropped frames coming out LARGER (179-180KB) than
    # uncropped ones (171-175KB) -- cropping ~5.5% of pixels (background
    # border only, low-entropy/cheap-to-compress) wasn't buying the
    # promised bandwidth win here, so the added complexity/risk wasn't
    # worth it. TcpFrameSource.send_crop()/reset_crop() themselves are
    # left in place (pre-existing, harmless, unused) in case this is
    # revisited later with a real controlled A/B benchmark -- if so, fix
    # the double-crop bug FIRST (skip the local slice entirely once the
    # sender is cropping, don't just note that it's "harmless") before
    # trusting any bandwidth measurement again.
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

    # Click-to-inspect HSV (2026-07-28, for --show-stickers threshold tuning):
    # OpenCV's own hover readout in the highgui titlebar is BGR-only, not
    # useful for picking HSV ranges. Mutable single-element list (not a bare
    # variable) so the mouse callback -- which OpenCV calls from outside this
    # function's normal control flow -- can see whatever the CURRENT frame is
    # without needing a class/global.
    _latest_frame_holder: list[np.ndarray | None] = [None]

    def _on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        f = _latest_frame_holder[0]
        if f is None or not (0 <= y < f.shape[0] and 0 <= x < f.shape[1]):
            return
        b, g, r = (int(v) for v in f[y, x])
        hsv_pixel = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = (int(c) for c in hsv_pixel)
        print(f"click (x={x},y={y}): BGR=({b},{g},{r})  "
              f"HSV=(h={h},s={s},v={v})  [OpenCV H range 0-179]")

        # Nearest-node + window diagnostic (added 2026-07-30): answers "was
        # this a real miss (nothing detected AND nothing in range near the
        # node), or a window-alignment miss (a real matching color exists
        # near here, but outside/at the edge of the node's search window)"
        # directly, instead of the user having to manually compare click
        # coordinates against node_overlay_pixels by hand.
        if node_overlay_pixels:
            nearest = min(
                node_overlay_pixels,
                key=lambda t: (t[1] - x) ** 2 + (t[2] - y) ** 2)
            n_id, npx, npy, is_bay = nearest
            dist_px = math.hypot(npx - x, npy - y)
            radius = STICKER_WINDOW_RADIUS_PX_DEFAULT
            inside = dist_px <= radius * math.sqrt(2)  # window is a square, not a circle
            expected_colors = STICKER_COLORS_FOR_BAY if is_bay else STICKER_COLORS_FOR_LATTICE
            det_result = None
            if sticker_detections:
                det_result = next((d for d in sticker_detections if d[0] == n_id), None)
            print(f"  nearest node: {n_id} ({'bay' if is_bay else 'lattice'}, "
                  f"expects {'/'.join(expected_colors)}) at ({npx:.0f},{npy:.0f}), "
                  f"click is {dist_px:.1f}px away "
                  f"({'INSIDE' if inside else 'OUTSIDE'} its "
                  f"{radius}px search window)")
            if det_result is not None:
                found_color = det_result[1]
                print(f"  detector result for node {n_id}: "
                      f"{found_color if found_color else 'NOTHING FOUND'}")

    if not args.no_preview:
        # full-resolution 1:1 preview (user's screen is 1920x1200), but
        # WINDOW_NORMAL keeps it draggable/resizable if that ever changes
        cv2.namedWindow("AprilTag localization", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag localization", args.width, args.height)
        cv2.setMouseCallback("AprilTag localization", _on_mouse)

    last_print = 0.0
    last_publish = 0.0
    last_stream = 0.0
    last_render = 0.0
    RENDER_INTERVAL_SEC = 1.0 / 15.0  # ~15fps preview refresh
    publish_interval = 1.0 / max(args.publish_rate, 0.1)
    calib_announced = False
    frame_seq = 0
    # --test-local-crop state: crop_rect stays None (no-op) until the
    # homography first locks, at which point it's computed once and never
    # changed again (camera is physically fixed -- see compute_crop_rect()'s
    # own docstring). crop_offset is added back onto every detection's
    # pixel coords AFTER cropping starts, so calib.H (fit in FULL-frame
    # coordinates, before cropping began) stays valid the whole time.
    crop_rect: tuple[int, int, int, int] | None = None
    crop_offset = np.array([0.0, 0.0])
    show_nodes = args.show_nodes
    node_overlay_pixels: list[tuple[int, float, float, bool]] | None = None
    depot_overlay_pixels: list[tuple[str, float, float]] | None = None
    show_stickers = args.show_stickers
    sticker_detections: list[tuple[int, str | None, int, int, int]] | None = None
    # Node-overlay pixel-position AVERAGING (added 2026-07-30): the overlay
    # used to project world node positions through calib.H from the SINGLE
    # frame where calibration first locked -- confirmed on real hardware
    # (node 88, via a paper AprilTag placed directly on the physical
    # sticker to get an exact ground-truth position) that this single-frame
    # snapshot has enough frame-to-frame jitter (from the underlying ref-tag
    # corner detections, which calib.H is itself derived from) that a
    # sticker sitting near a search window's edge sometimes falls inside it
    # and sometimes doesn't -- same physical sticker, same real position,
    # intermittent detection depending on which exact frame calibration
    # happened to lock on. Averaging the PROJECTED PIXEL positions over
    # NODE_OVERLAY_AVG_FRAMES frames after lock (not just using calib.H
    # once) smooths this out at the source instead of just widening the
    # search window further to tolerate more jitter.
    NODE_OVERLAY_AVG_FRAMES = 8
    _node_overlay_accum: list[np.ndarray] = []
    _node_overlay_world_nodes = None
    _depot_overlay_accum: list[np.ndarray] = []
    _depot_overlay_world_points = None

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
        "render": RollingStats(), "total_loop": RollingStats(),
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

            if crop_rect is not None:
                cx0, cy0, cx1, cy1 = crop_rect
                frame = frame[cy0:cy1, cx0:cx1]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _t_cvt = time.perf_counter()
            stats["cvt"].add(_t_cvt - _t_read)

            detections = detector.detect(gray)
            _t_apriltag = time.perf_counter()
            stats["apriltag"].add(_t_apriltag - _t_cvt)

            if crop_rect is not None:
                # Detections just came back relative to the CROPPED frame's
                # origin -- translate back to full-frame coordinates before
                # calib.update()/anything else touches them, so calib.H
                # (fit in full-frame coordinates, before cropping began)
                # stays valid. Confirmed by reading pupil_apriltags'
                # Detection class source directly: plain instance
                # attributes (self.center/self.corners set in __init__),
                # no __slots__/read-only properties -- safe to reassign.
                for det in detections:
                    det.center = det.center + crop_offset
                    det.corners = det.corners + crop_offset

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
                    if args.test_local_crop and crop_rect is None:
                        # frame.shape here is still the FULL frame -- cropping
                        # doesn't start until crop_rect is set below, this is
                        # the last frame processed at full size.
                        cx0, cy0, cx1, cy1 = compute_crop_rect(frame.shape, calib.H)
                        crop_w, crop_h = cx1 - cx0, cy1 - cy0
                        if crop_w > 0 and crop_h > 0:
                            print(f"--test-local-crop: table bounding box "
                                  f"found at ({cx0},{cy0})-({cx1},{cy1}) "
                                  f"({crop_w}x{crop_h}, "
                                  f"{100*crop_w*crop_h/(frame.shape[1]*frame.shape[0]):.0f}% "
                                  "of full frame) -- cropping starts next frame")
                            crop_rect = (cx0, cy0, cx1, cy1)
                            crop_offset = np.array([float(cx0), float(cy0)])
                        else:
                            print("--test-local-crop: degenerate crop rect, "
                                  "not cropping (will retry next calibration)")
                    _node_overlay_world_nodes = build_node_overlay_points(args.rows, args.cols)
                    _node_overlay_accum = []
                    _depot_overlay_world_points = build_depot_overlay_points()
                    _depot_overlay_accum = []
                    print(f"Averaging node overlay position over "
                          f"{NODE_OVERLAY_AVG_FRAMES} frames...")

                if node_overlay_pixels is None and _node_overlay_world_nodes is not None:
                    # --show-nodes / --show-stickers: project every grid/
                    # workstation/entry node's world position through THIS
                    # frame's Hinv and accumulate over several frames before
                    # finalizing (CHANGED 2026-07-30 from a single-frame
                    # snapshot -- see NODE_OVERLAY_AVG_FRAMES's comment
                    # above for the real-hardware jitter this fixes).
                    Hinv_nodes = np.linalg.inv(calib.H)
                    pixel_xy = map_points(
                        Hinv_nodes,
                        np.array([[x, y] for _n, x, y, _is_bay
                                   in _node_overlay_world_nodes]))
                    _node_overlay_accum.append(pixel_xy)
                    # Depot slots/entries/node 0 (added 2026-07-30): same
                    # Hinv, same accumulate-then-average treatment, just a
                    # separate world-point list (string ids, see
                    # build_depot_overlay_points()) and a separate output
                    # variable so lattice/bay code paths are untouched.
                    depot_pixel_xy = map_points(
                        Hinv_nodes,
                        np.array([[x, y] for _label, x, y
                                   in _depot_overlay_world_points]))
                    _depot_overlay_accum.append(depot_pixel_xy)

                    if len(_node_overlay_accum) >= NODE_OVERLAY_AVG_FRAMES:
                        avg_pixel_xy = np.mean(np.stack(_node_overlay_accum), axis=0)
                        node_overlay_pixels = [
                            (n, float(px), float(py), is_bay)
                            for (n, _x, _y, is_bay), (px, py)
                            in zip(_node_overlay_world_nodes, avg_pixel_xy)]
                        avg_depot_pixel_xy = np.mean(np.stack(_depot_overlay_accum), axis=0)
                        depot_overlay_pixels = [
                            (label, float(px), float(py))
                            for (label, _x, _y), (px, py)
                            in zip(_depot_overlay_world_points, avg_depot_pixel_xy)]
                        print(f"Node overlay ready: {len(node_overlay_pixels)} "
                              f"nodes ({args.rows}x{args.cols} grid) + "
                              f"{len(depot_overlay_pixels)} depot points, "
                              f"averaged over {NODE_OVERLAY_AVG_FRAMES} frames "
                              "-- press 'o' to toggle"
                              + (" (already ON)" if show_nodes else ""))
                        # --show-stickers: CHANGED 2026-07-30 from whole-table
                        # color-blob scanning to a small search window around
                        # each LATTICE node's already-known position (reuses
                        # node_overlay_pixels, just computed above) -- see
                        # detect_stickers_at_nodes()'s docstring for why (the
                        # whole-table version still had both false positives
                        # AND missed real stickers even with good lighting).
                        # Still a one-time freeze once the node overlay
                        # itself finalizes (now averaged, see above) --
                        # independent of the node overlay's own toggle,
                        # still bound to the 'u' key.
                        sticker_detections = detect_stickers_at_nodes(
                            frame, node_overlay_pixels,
                            debug_node_ids=args.debug_sticker_nodes)
                        counts = Counter(c for _n, c, *_ in sticker_detections)
                        is_bay_by_id = {n: is_bay for n, _px, _py, is_bay in node_overlay_pixels}
                        # Only LATTICE misses are a real flag -- every lattice
                        # node is expected to have a sticker. A bay-node miss
                        # just means that workstation isn't currently active
                        # (confirmed 2026-07-30: only some of the 98 bay nodes
                        # have a real sticker at any time), not an error, so
                        # it's excluded here the same way draw_overlay() skips
                        # drawing a gray marker for it.
                        lattice_missing = sum(
                            1 for n, c, *_ in sticker_detections
                            if c is None and not is_bay_by_id.get(n, False))
                        print(f"Sticker overlay ready: "
                              f"{counts.get('RED', 0)} red, "
                              f"{counts.get('YELLOW', 0)} yellow, "
                              f"{counts.get('BLUE', 0)} blue detected, "
                              f"{lattice_missing} lattice sticker(s) MISSING "
                              "(bay misses = inactive workstations, not shown) "
                              "-- press 'u' to toggle"
                              + (" (already ON)" if show_stickers else ""))
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

            # Render throttle (2026-08-18): draw_overlay()+imshow() used to
            # run every loop iteration, i.e. at the full ~30-38Hz detection
            # rate post-decimate -- pure display cost nothing downstream
            # (rosbridge publish, CSV log) needs, since no human perceives a
            # preview window redrawing faster than ~15fps. Gated on the SAME
            # wall-clock condition as the streamer's existing 10fps cap
            # (now - last_render, not a frame-count skip) so it stays
            # anchored to real time regardless of the current loop rate.
            # cv2.waitKey(1) still runs every iteration unthrottled below --
            # it pumps the OpenCV window's event loop and reads keyboard
            # input (q/r/o/u); throttling it too would make the window
            # appear frozen and hotkeys sluggish between render frames.
            do_render = now - last_render >= RENDER_INTERVAL_SEC
            if do_render and (not args.no_preview or streamer is not None):
                _t_render0 = time.perf_counter()
                draw_overlay(frame, calib, detections, poses,
                             node_overlay_pixels, show_nodes,
                             sticker_detections, show_stickers,
                             depot_overlay_pixels)
                if streamer is not None and now - last_stream >= 0.1:  # ~10 fps
                    streamer.push(frame)
                    last_stream = now
                if not args.no_preview:
                    _latest_frame_holder[0] = frame
                    cv2.imshow("AprilTag localization", frame)
                stats["render"].add(time.perf_counter() - _t_render0)
                last_render = now
            if not args.no_preview:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    calib.reset()
                    calib_announced = False
                    # Stale relative to whatever the NEXT calibration lock
                    # fits -- clear and let it rebuild on that lock rather
                    # than keep drawing circles from the old homography.
                    # (calib_announced=False also re-inits these on the next
                    # lock regardless -- cleared explicitly here too so no
                    # stale accumulator state lingers between resets.)
                    node_overlay_pixels = None
                    depot_overlay_pixels = None
                    sticker_detections = None
                    _node_overlay_world_nodes = None
                    _node_overlay_accum = []
                    _depot_overlay_world_points = None
                    _depot_overlay_accum = []
                    print("Calibration reset.")
                if key == ord("o"):
                    show_nodes = not show_nodes
                    print(f"Node overlay {'ON' if show_nodes else 'OFF'}"
                          + ("" if node_overlay_pixels is not None else
                             " (waiting for calibration to lock first)"))
                if key == ord("u"):
                    show_stickers = not show_stickers
                    print(f"Sticker overlay {'ON' if show_stickers else 'OFF'}"
                          + ("" if sticker_detections is not None else
                             " (waiting for calibration to lock first)"))

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
                      f"render[{stats['render'].summary(elapsed)}]  "
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
