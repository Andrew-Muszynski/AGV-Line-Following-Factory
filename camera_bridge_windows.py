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

Usage:
    python camera_bridge_windows.py --camera 1 --backend msmf --port 8765
"""
from __future__ import annotations

import argparse
import socket
import time

import cv2

from apriltag_detect import open_camera


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
    args = ap.parse_args()

    cap = open_camera(args.camera, preferred_backend=args.backend)
    if cap is None:
        raise SystemExit(f"Could not open camera index {args.camera}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    print(f"Capture resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)
    print(f"Listening on 0.0.0.0:{args.port} -- waiting for WSL2 receiver to connect...")

    try:
        while True:
            conn, addr = server.accept()
            print(f"Receiver connected from {addr}")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sent = 0
            t_report = time.time()
            try:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        time.sleep(0.01)
                        continue
                    ok, buf = cv2.imencode(".jpg", frame, encode_params)
                    if not ok:
                        continue
                    data = buf.tobytes()
                    header = len(data).to_bytes(4, "big")
                    conn.sendall(header)
                    conn.sendall(data)
                    sent += 1
                    now = time.time()
                    if now - t_report >= 2.0:
                        print(f"sent {sent / (now - t_report):.1f} frames/sec, "
                              f"{len(data) / 1024:.0f}KB last frame")
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
