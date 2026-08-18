#!/usr/bin/env python3
"""One-time measurement tool: connects to camera_bridge_windows.py (CPU-side,
pupil_apriltags -- no Isaac ROS / AprilTagNode involved), waits for the 4
corner ref tags (20-23), fits the table homography, computes the crop
rectangle via apriltag_localize.compute_crop_rect(), and saves it to a JSON
file. This lets isaac_ros_image_publisher.py load a FIXED crop rect and crop
from frame 1 onward -- AprilTagNode's cuAprilTags decoder has a persistent
GPU buffer sized for whatever resolution it sees FIRST and crashes
(cudaErrorInvalidPitchValue) on a runtime resolution change, confirmed on
real hardware 2026-08-05 (see memory: isaac_ros_apriltag_gpu). The fix is to
never change resolution at runtime at all: measure the crop rect once here
(the camera is physically fixed, so this only needs re-running if the camera
moves), save it, then have the Isaac pipeline crop consistently from the
start.

The crop math itself (compute_crop_rect + offset correction) was separately
validated end-to-end on real hardware via apriltag_localize.py's
--test-local-crop (positions stayed continuous across the crop transition,
no jump) -- this tool reuses the exact same function, just to run once and
persist the result instead of applying it live.

Usage:
    python3 fleet/measure_crop_rect.py --port 8765 --out fleet/crop_rect.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from apriltag_localize import (  # noqa: E402
    DEFAULT_REF_TAG_SIZE_IN,
    TableCalibration,
    TcpFrameSource,
    build_detector,
    compute_crop_rect,
    cv2,
)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=8765,
                     help="camera_bridge_windows.py TCP port (default 8765)")
    ap.add_argument("--out", default="fleet/crop_rect.json",
                     help="output path (default fleet/crop_rect.json)")
    ap.add_argument("--tag-size", type=float, default=None,
                     help=f"ref tag black-square edge length, inches "
                          f"(default {DEFAULT_REF_TAG_SIZE_IN}, measured)")
    ap.add_argument("--margin-in", type=float, default=6.0,
                     help="padding around the table's fitted bounds, inches "
                          "(default 6.0, matches apriltag_localize.py's "
                          "compute_crop_rect default)")
    ap.add_argument("--observations", type=int, default=45,
                     help="ref-tag observations to average before fitting "
                          "the homography (default 45, matches the "
                          "--freeze-calib value used in prior hardware "
                          "tests tonight)")
    ap.add_argument("--threads", type=int, default=16,
                     help="pupil_apriltags detector threads (default 16, "
                          "the measured-optimal value on this machine's "
                          "16 performance cores)")
    args = ap.parse_args()

    tag_size = args.tag_size if args.tag_size is not None else DEFAULT_REF_TAG_SIZE_IN
    print(f"Connecting to camera_bridge_windows.py on port {args.port}...")
    cap = TcpFrameSource(args.port)
    print("Connected.")

    detector = build_detector("tag36h11", 1.0, args.threads)
    calib = TableCalibration(tag_size, freeze_after_n=args.observations)

    print(f"Averaging {args.observations} ref-tag observations, then fitting "
          "ONE homography (camera is physically fixed -- this only needs "
          "re-running if the camera moves)...")

    last_print = 0.0
    frame_w = frame_h = None
    while calib.H is None:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.01)
            continue
        frame_h, frame_w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)
        calib.update(detections)
        now = time.monotonic()
        if now - last_print >= 2.0:
            seen = sorted(d.tag_id for d in detections)
            print(f"seen tags {seen}; waiting for corner tags (20-23)...")
            last_print = now

    print(f"Calibration locked: refs {calib.used_ids}, rms {calib.rms_in:.2f} in")

    x0, y0, x1, y1 = compute_crop_rect((frame_h, frame_w), calib.H,
                                        margin_in=args.margin_in)
    crop_w, crop_h = x1 - x0, y1 - y0
    if crop_w <= 0 or crop_h <= 0:
        raise SystemExit(f"degenerate crop rect ({x0},{y0})-({x1},{y1}), "
                          "not saving -- check tag visibility and re-run")

    pct = 100 * crop_w * crop_h / (frame_w * frame_h)
    print(f"Crop rect: ({x0},{y0})-({x1},{y1}) ({crop_w}x{crop_h}, "
          f"{pct:.0f}% of full {frame_w}x{frame_h} frame)")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump({
            "full_width": frame_w,
            "full_height": frame_h,
            "x0": x0, "y0": y0, "x1": x1, "y1": y1,
            "crop_width": crop_w, "crop_height": crop_h,
            "margin_in": args.margin_in,
            "tag_size_in": tag_size,
            "rms_in": calib.rms_in,
            "ref_tag_ids": calib.used_ids,
            "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }, f, indent=2)
    print(f"Saved to {out_path} -- re-run this tool if the camera moves.")

    cap.release()


if __name__ == "__main__":
    main()
