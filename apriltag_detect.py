#!/usr/bin/env python3
"""
apriltag_detect.py  —  Live AprilTag ID + pixel-location viewer.

First step toward camera-based AGV localization (see thesis notes on
AprilTags on top of each Alvik + four corner tags on the table): this
script just opens the webcam, detects tag36h11 tags every frame, and
prints/draws each tag's ID and pixel location. No camera calibration or
metric (cm) pose yet — that needs the Nexigo N980P's focal length /
optical center, which isn't measured yet. Once that's done, feed
detector_at_size(fx, fy, cx, cy, tag_size_cm) instead of the pixel-only
detector to get real pose_R / pose_t per tag.

Usage:
    python3 apriltag_detect.py                  # webcam index 0, tag36h11
    python3 apriltag_detect.py --camera 1
    python3 apriltag_detect.py --family tag25h9
    python3 apriltag_detect.py --no-preview      # headless, ID+pixel to stdout only

Press 'q' in the preview window to quit.
"""

from __future__ import annotations

import argparse
import time

import cv2
from pupil_apriltags import Detector


def build_detector(family: str) -> Detector:
    return Detector(
        families=family,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
    )


def annotate_frame(frame, detections) -> None:
    for det in detections:
        corners = det.corners.astype(int)
        cx, cy = det.center
        cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        cv2.putText(
            frame, f"ID {det.tag_id}", (int(cx) + 8, int(cy) - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
        )


_BACKENDS = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}


def open_camera(index: int, preferred_backend: str | None = None) -> cv2.VideoCapture:
    """Try each Windows capture backend in turn — some webcams (or some
    Windows privacy-setting states) only succeed on one of these.

    MSMF tried first (2026-07-27, was DSHOW-first): bench-measured cap.read()
    taking ~140ms/call on DSHOW at 1920x1080 despite a 60fps-capable camera,
    vs. ~3ms/call on MSMF on the same hardware -- a ~5x publish-rate
    improvement (apriltag_localize.py's diag: output went from 5Hz to
    18-25Hz). DSHOW/ANY remain as fallback for a camera/machine where MSMF
    fails to open. preferred_backend lets a caller force a specific one
    (e.g. "dshow") to override this order, e.g. if MSMF misbehaves on a
    different setup later."""
    order = [(cv2.CAP_MSMF, "MSMF"), (cv2.CAP_DSHOW, "DSHOW"), (cv2.CAP_ANY, "ANY")]
    if preferred_backend is not None:
        key = preferred_backend.lower()
        if key not in _BACKENDS:
            raise ValueError(
                f"unknown backend {preferred_backend!r}, expected one of "
                f"{sorted(_BACKENDS)}")
        wanted = _BACKENDS[key]
        order = [item for item in order if item[0] == wanted] + \
                [item for item in order if item[0] != wanted]
    for backend, name in order:
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            print(f"Opened camera {index} via {name} backend.")
            return cap
        cap.release()
    return None


def list_open_cameras(max_index: int = 5) -> list[int]:
    found = []
    for idx in range(max_index):
        cap = open_camera(idx)
        if cap is not None:
            found.append(idx)
            cap.release()
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description="Live AprilTag ID + pixel-location viewer.")
    parser.add_argument("--camera", type=int, default=0, help="webcam device index (default 0)")
    parser.add_argument("--family", default="tag36h11", help="AprilTag family (default tag36h11)")
    parser.add_argument("--no-preview", action="store_true", help="headless: print detections, no window")
    parser.add_argument("--list-cameras", action="store_true", help="probe indices 0-4 and report which open, then exit")
    args = parser.parse_args()

    if args.list_cameras:
        found = list_open_cameras()
        print(f"Openable camera indices: {found}" if found else "No camera index opened.")
        return

    cap = open_camera(args.camera)
    if cap is None:
        raise SystemExit(
            f"Could not open camera index {args.camera} with any backend (DSHOW/MSMF/ANY).\n"
            "If the camera works in other apps (e.g. Windows Camera), this is usually:\n"
            "  Settings > Privacy & security > Camera > 'Let desktop apps access your camera' (must be ON)\n"
            "Run with --list-cameras to probe which indices actually open."
        )

    detector = build_detector(args.family)

    print(f"Detecting {args.family} tags on camera {args.camera}. "
          f"{'Headless mode, ' if args.no_preview else ''}Ctrl+C or 'q' to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame grab failed, retrying...")
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)

            if detections:
                summary = ", ".join(
                    f"id={det.tag_id} px=({det.center[0]:.0f},{det.center[1]:.0f})"
                    for det in detections
                )
                print(summary)

            if not args.no_preview:
                annotate_frame(frame, detections)
                cv2.imshow("AprilTag detections", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
