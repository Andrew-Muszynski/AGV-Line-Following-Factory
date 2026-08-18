#!/usr/bin/env python3
"""
camera_bridge_windows.py — captures frames on Windows (fast MSMF backend,
confirmed 3-13ms/read) and streams them over a local TCP socket to
apriltag_localize.py running in WSL2 (--frame-source tcp:PORT), so WSL2 gets
GPU passthrough access without needing USB camera passthrough at all.

USB passthrough (usbipd) was tried and rejected 2026-07-28: it added an
~80-113ms/frame latency floor (fundamental to USB-over-network tunneling for
high-bandwidth devices, not tunable via OpenCV settings), dropping overall
throughput from ~40Hz to ~9Hz despite apriltag detection itself being
identically fast on both sides. This bridge instead keeps capture on the
fast native path and only ships already-captured frames across, which is a
localhost TCP transfer (sub-ms) instead of a USB protocol tunnel.

Wire format: for each frame, a 4-byte big-endian length prefix followed by
that many bytes of JPEG-encoded frame data. No handshake, no ack -- one
direction, best-effort (a broken connection just ends the loop; the
WSL2-side receiver in apriltag_localize.py handles reconnection).

Crop back-channel (added 2026-08-05, isaac_ros_apriltag_gpu throughput
investigation): WSL2's mirrored-networking 127.0.0.1 path was measured
routing through a real virtual NIC (loopback0, MTU 1500), NOT a true
zero-copy in-kernel loopback -- every ~180KB full-frame JPEG was
fragmenting into ~120 real packets, and that per-packet cost (not decode,
not buffer size, not receiver scheduling -- all independently ruled out)
was the actual throughput ceiling. Since only the table area (bounded by
the 4 corner AprilTags) is ever used for localization, cropping BEFORE
encoding on the Windows side cuts both encode work and bytes-on-the-wire
without touching resolution/quality. The receiver doesn't know the crop
region until it's calibrated from full frames, so: this bridge starts
uncropped, and the receiver may send back a crop-rectangle command once it
knows where the table is. Command wire format, sent FROM the receiver TO
this process (opposite direction from every other byte on this socket, so
it's unambiguous): 4-byte magic b"CROP", then 4x uint16 big-endian
(x0, y0, x1, y1) pixel coordinates in the FULL uncropped frame. Send
b"CR0P" (crop_x0=crop_y0=crop_x1=crop_y1=0) to reset back to full-frame
uncropped mode. Checked non-blockingly (select() with 0 timeout) once per
frame in the send loop, so it never adds latency to the hot path.

P-core/E-core CPU affinity (added 2026-08-05, isaac_ros_apriltag_gpu
throughput investigation): confirmed on real hardware that Windows
occasionally schedules this process's single hot thread (cap.read/
imencode/sendall all run on the main thread) onto an E-core instead of a
P-core -- package-level clock speed and utilization both looked fine
throughout (3.8-4.2GHz, ~13%), because they're aggregated across all 24
logical processors, but cv2.imencode()'s cost on THIS thread swung 19ms
<-> 4ms (~5x) in lockstep with nothing else explaining it. Confirmed via a
live test: manually restricting Task Manager's affinity for this process
to CPU 0-15 made the 19ms band disappear entirely (44+ consecutive 2s
samples at steady ~60Hz/~4ms, vs. constant reversion before). CPU 0-15 are
this machine's P-core hyperthread pairs (Intel Core i7-14650HX: 8 P-cores
x2 threads = 16, then 8 E-cores x1 thread = 8, for 24 total) -- Windows
consistently numbers P-cores first, but this isn't guaranteed on every
machine/BIOS, hence --cpu-affinity to override if this script is ever run
elsewhere. Applied automatically at startup via SetProcessAffinityMask (no
new dependency -- ctypes + kernel32, standard library only).

Fisheye undistortion (added 2026-08-05, isaac_ros_apriltag_gpu robustness
investigation): confirmed via a direct pixel measurement (pupil_apriltags'
own corner detections used as ground truth, since it reliably detects far
tags where cuAprilTags doesn't) that tags near the frame's periphery are
not just smaller but severely FORESHORTENED under this camera's ~120deg
fisheye lens -- edge-length ratios (longest/shortest of a tag's 4 sides)
measured 1.6-1.73x for far/edge tags vs 1.03-1.04x for near-center tags,
i.e. imaged as skewed parallelograms, not near-squares. This -- not raw
pixel size alone -- is believed to be why cuAprilTags (AprilTagNode, this
project's GPU detection path) fails to detect these tags at anywhere near
pupil_apriltags' (CPU path) success rate: cuAprilTags' faster GPU decode
pipeline appears to have a tighter tolerance for quad skew.

isaac_ros_image_proc's RectifyNode would normally fix this, but it CANNOT
be used in this pipeline -- confirmed by reading its actual C++ source
(rectify_node.cpp): it requires a genuine NITROS-producing publisher, and
this bridge's receiver (isaac_ros_image_publisher.py) is a plain rclpy
node with no NITROS support (would need a C++ bridge against NVIDIA's
NITROS SDK to fix that path -- out of scope for now). --undistort takes a
different, NITROS-independent route: since fisheye distortion is a known,
physically-modeled effect, this bridge can undistort the frame ITSELF
(cv2.fisheye.initUndistortRectifyMap + cv2.remap, using the SAME
fleet/camera_intrinsics.json fisheye calibration already used elsewhere)
BEFORE cropping/encoding -- so whatever receives frames from this bridge
(Isaac's image publisher, apriltag_localize.py, anything) gets an already-
rectified image with zero NITROS involvement. The undistort map is built
ONCE at startup (cheap; cv2.remap() itself is fast, confirmed via --diag),
not recomputed per frame.

Usage:
    python camera_bridge_windows.py --camera 1 --backend msmf --port 8765
    python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 \\
        --undistort --intrinsics fleet/camera_intrinsics.json
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
import select
import socket
import struct
import time
from pathlib import Path

import cv2
import numpy as np

from apriltag_detect import open_camera

CROP_MAGIC = b"CROP"
CROP_RESET_MAGIC = b"CR0P"
CROP_CMD_LEN = 4 + 8  # magic + 4x uint16

# This machine's confirmed P-core logical-processor range (CPU 0-15) -- see
# the module docstring for how this was measured. A bitmask, not a count:
# bit i set = logical processor i allowed.
DEFAULT_PCORE_AFFINITY_MASK = (1 << 16) - 1  # 0x0000FFFF = CPUs 0-15


def _set_process_affinity(mask: int) -> bool:
    """Best-effort: restrict this process to the given CPU bitmask via the
    Win32 API directly (ctypes, no pywin32/psutil dependency). Returns
    False (never raises) if this isn't Windows or the call fails, since a
    failed affinity pin should degrade to "no pin" rather than crash a
    process whose real job is streaming camera frames.

    NOTE: explicit argtypes/restype below are NOT optional decoration --
    confirmed by hitting the actual failure: GetCurrentProcess() returns a
    pseudo-handle of -1 (0xFFFFFFFFFFFFFFFF on 64-bit), and ctypes' default
    int inference (no restype declared) silently truncates/mishandles that
    64-bit value, making SetProcessAffinityMask fail every time with no
    exception raised -- it just returns 0/false. Declaring c_void_p/
    c_size_t explicitly, as done here, fixes this; verified working via a
    GetProcessAffinityMask round-trip read-back before this was accepted
    as correct."""
    if os.name != "nt":
        return False
    try:
        kernel32 = ctypes.windll.kernel32
        kernel32.GetCurrentProcess.restype = ctypes.c_void_p
        kernel32.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        kernel32.SetProcessAffinityMask.restype = ctypes.c_int
        handle = kernel32.GetCurrentProcess()
        ok = kernel32.SetProcessAffinityMask(handle, mask)
        return bool(ok)
    except (AttributeError, OSError):
        return False


def _build_undistort_maps(intrinsics_path: Path, width: int, height: int):
    """Load a cv2.fisheye.calibrate() intrinsics JSON (same format/file as
    fleet/camera_calibrate.py --solve produces and isaac_ros_image_publisher.py
    already consumes) and build a cv2.remap() map pair for undistorting
    frames at (width, height). balance=0.0 (default) keeps all pixels valid
    at the cost of some FOV; new_K is left as K itself (no extra scaling) --
    simplest correct choice, revisit only if the undistorted FOV needs
    tuning once this is validated on hardware."""
    with open(intrinsics_path) as f:
        calib = json.load(f)
    if calib.get("model") != "fisheye":
        raise SystemExit(
            f"{intrinsics_path} was not produced by a fisheye calibration "
            f"(model={calib.get('model')!r}) -- see camera_calibrate.py")
    K = np.array(calib["camera_matrix"], dtype=np.float64)
    D = np.array(calib["distortion_coefficients"], dtype=np.float64)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, (width, height), cv2.CV_16SC2)
    return map1, map2


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", type=int, default=1,
                     help="webcam index (default 1 = overhead Nexigo)")
    ap.add_argument("--backend", default="msmf", choices=["dshow", "msmf", "any"])
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--port", type=int, default=8765,
                     help="TCP port to listen on for the WSL2-side receiver")
    ap.add_argument("--jpeg-quality", type=int, default=90,
                     help="JPEG encode quality 1-100 (default 90 -- higher "
                          "preserves tag-corner precision better at the cost "
                          "of more bytes/frame over the socket)")
    ap.add_argument("--diag", action="store_true",
                     help="print per-stage timing (cap.read/imencode/sendall) "
                          "every 2s instead of just the fps summary")
    ap.add_argument("--cpu-affinity", type=lambda s: int(s, 0), default=DEFAULT_PCORE_AFFINITY_MASK,
                     metavar="MASK",
                     help="CPU affinity bitmask restricting this process to "
                          "specific logical processors, e.g. 0xFFFF for "
                          "CPUs 0-15 (default: this machine's confirmed "
                          "P-core range -- see module docstring for why "
                          "this exists). Pass 0 to disable pinning "
                          "entirely and let Windows schedule freely.")
    ap.add_argument("--undistort", action="store_true",
                     help="undistort each frame (cv2.fisheye, using "
                          "--intrinsics) BEFORE cropping/encoding -- fixes "
                          "the severe tag foreshortening measured at the "
                          "frame's periphery (1.6-1.73x edge-ratio skew) "
                          "that cuAprilTags fails to detect reliably, "
                          "without needing NITROS/RectifyNode. See module "
                          "docstring.")
    ap.add_argument("--intrinsics", default="fleet/camera_intrinsics.json",
                     help="path to camera_calibrate.py --solve output, "
                          "only used if --undistort is set (default "
                          "fleet/camera_intrinsics.json)")
    args = ap.parse_args()

    if args.cpu_affinity:
        if _set_process_affinity(args.cpu_affinity):
            print(f"CPU affinity pinned to mask 0x{args.cpu_affinity:X} "
                  "(avoids E-core scheduling slowdowns -- see module "
                  "docstring; pass --cpu-affinity 0 to disable)")
        else:
            print("CPU affinity pin failed or unsupported on this platform "
                  "-- continuing without it (encode timing may be less "
                  "consistent, see module docstring)")

    cap = open_camera(args.camera, preferred_backend=args.backend)
    if cap is None:
        raise SystemExit(f"Could not open camera index {args.camera}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    real_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Capture resolution: {real_width}x{real_height}")

    undistort_maps = None
    if args.undistort:
        intrinsics_path = Path(args.intrinsics)
        if not intrinsics_path.is_file():
            raise SystemExit(f"--undistort: intrinsics file not found: "
                              f"{intrinsics_path}")
        undistort_maps = _build_undistort_maps(
            intrinsics_path, real_width, real_height)
        print(f"Undistort ARMED using {intrinsics_path} -- every frame is "
              "rectified (cv2.fisheye) before crop/encode.")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)
    # 1s timeout on accept() -- added 2026-08-05 after confirming Ctrl+C is
    # silently swallowed while idle here: accept() is a blocking OS call, and
    # on Windows KeyboardInterrupt can only be delivered to the interpreter
    # between bytecode instructions, so it stays pending until accept()
    # actually returns (previously required connecting a second, throwaway
    # client just to unblock it). A short timeout with a caught
    # socket.timeout turns this into a poll loop instead, so Ctrl+C is
    # always responsive within ~1s even with no client connected yet.
    server.settimeout(1.0)
    print(f"Listening on 0.0.0.0:{args.port} -- waiting for WSL2 receiver to connect...")

    try:
        while True:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print(f"Receiver connected from {addr}")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sent = 0
            t_report = time.time()
            read_times: list[float] = []
            undistort_times: list[float] = []
            encode_times: list[float] = []
            send_times: list[float] = []
            crop: tuple[int, int, int, int] | None = None  # (x0, y0, x1, y1) or None = full frame
            try:
                while True:
                    # Non-blocking check for a crop command from the receiver
                    # -- see module docstring for wire format. select() with
                    # a 0 timeout costs effectively nothing when no data is
                    # waiting (the common case), so this never adds latency
                    # to the per-frame hot path below.
                    ready, _, _ = select.select([conn], [], [], 0.0)
                    if ready:
                        cmd = conn.recv(CROP_CMD_LEN)
                        if len(cmd) == CROP_CMD_LEN and cmd[:4] == CROP_MAGIC:
                            x0, y0, x1, y1 = struct.unpack(">HHHH", cmd[4:])
                            if (x0, y0, x1, y1) == (0, 0, 0, 0):
                                crop = None
                                print("crop: reset to full frame")
                            else:
                                crop = (x0, y0, x1, y1)
                                print(f"crop: set to ({x0},{y0})-({x1},{y1}) "
                                      f"({x1-x0}x{y1-y0})")
                        elif not cmd:
                            raise ConnectionResetError("receiver closed during crop-cmd read")

                    t0 = time.perf_counter()
                    ok, frame = cap.read()
                    t1 = time.perf_counter()
                    if not ok:
                        time.sleep(0.01)
                        continue
                    if undistort_maps is not None:
                        map1, map2 = undistort_maps
                        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
                    t1b = time.perf_counter()
                    if crop is not None:
                        x0, y0, x1, y1 = crop
                        frame = frame[y0:y1, x0:x1]
                    ok, buf = cv2.imencode(".jpg", frame, encode_params)
                    t2 = time.perf_counter()
                    if not ok:
                        continue
                    data = buf.tobytes()
                    header = len(data).to_bytes(4, "big")
                    conn.sendall(header)
                    conn.sendall(data)
                    t3 = time.perf_counter()
                    sent += 1
                    if args.diag:
                        read_times.append(t1 - t0)
                        undistort_times.append(t1b - t1)
                        encode_times.append(t2 - t1b)
                        send_times.append(t3 - t2)
                    now = time.time()
                    if now - t_report >= 2.0:
                        if args.diag and read_times:
                            def _fmt(xs: list[float]) -> str:
                                xs = sorted(xs)
                                n = len(xs)
                                return (f"median={xs[n//2]*1000:.1f}ms "
                                        f"p95={xs[min(int(0.95*n), n-1)]*1000:.1f}ms "
                                        f"max={xs[-1]*1000:.1f}ms")
                            print(f"[diag] {sent} frames/{now - t_report:.1f}s = "
                                  f"{sent / (now - t_report):.1f}fps  "
                                  f"crop={crop if crop else 'none (full frame)'}  "
                                  f"read={_fmt(read_times)}  "
                                  + (f"undistort={_fmt(undistort_times)}  "
                                     if undistort_maps is not None else "")
                                  + f"encode={_fmt(encode_times)}  "
                                  f"send={_fmt(send_times)}")
                            read_times.clear()
                            undistort_times.clear()
                            encode_times.clear()
                            send_times.clear()
                        else:
                            crop_note = f" crop={frame.shape[1]}x{frame.shape[0]}" if crop else ""
                            print(f"sent {sent / (now - t_report):.1f} frames/sec, "
                                  f"{len(data) / 1024:.0f}KB last frame{crop_note}")
                        sent = 0
                        t_report = now
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                print("Receiver disconnected, waiting for reconnect...")
                conn.close()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        server.close()


if __name__ == "__main__":
    main()
