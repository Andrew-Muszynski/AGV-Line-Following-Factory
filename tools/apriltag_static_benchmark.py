#!/usr/bin/env python3
"""Replay one image set through controlled AprilTag detector configurations."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from apriltag_localize import (  # noqa: E402
    AprilTagBackendDiagnostics,
    LensUndistorter,
    REF_TAG_WORLD,
    ROBOT_NAMES,
    SessionDiagnostics,
    TableCalibration,
    build_detector,
    map_points,
    tag_world_pose,
)


def timing(values: list[float]) -> dict:
    values_ms = np.asarray(values, dtype=np.float64) * 1000.0
    return {
        "median_ms": float(np.median(values_ms)),
        "p95_ms": float(np.percentile(values_ms, 95)),
        "max_ms": float(np.max(values_ms)),
    }


def run(frames, decimate: float, refine: bool, args) -> dict:
    detector = build_detector(args.family, decimate, args.threads, refine)
    backend = AprilTagBackendDiagnostics(detector)
    backend.reset()
    calibration = TableCalibration(
        args.tag_size, freeze_after_n=args.freeze_calib,
        freeze_min_refs=args.freeze_min_refs)
    undistorter = (LensUndistorter.from_file(args.camera_calibration)
                   if args.camera_calibration else None)
    diagnostics = SessionDiagnostics(sorted([*ROBOT_NAMES, *REF_TAG_WORLD]))
    detection_times: list[float] = []
    loop_times: list[float] = []
    undistort_times: list[float] = []
    legacy_yaws: dict[int, list[float]] = defaultdict(list)
    started = time.perf_counter()
    for frame in frames:
        loop_start = time.perf_counter()
        working_frame = frame
        if undistorter is not None:
            undistort_start = time.perf_counter()
            working_frame = undistorter.apply(frame)
            undistort_times.append(time.perf_counter() - undistort_start)
        gray = cv2.cvtColor(working_frame, cv2.COLOR_BGR2GRAY)
        detection_start = time.perf_counter()
        detections = detector.detect(gray)
        detection_times.append(time.perf_counter() - detection_start)
        diagnostics.observe(detections)
        calibration.update(detections)
        if calibration.H is not None:
            for det in detections:
                if det.tag_id in ROBOT_NAMES:
                    pose = tag_world_pose(calibration.H, det)
                    if pose is None:
                        diagnostics.invalid_geometry += 1
                    else:
                        diagnostics.record_raw_yaw(det.tag_id, pose[2])
                        corners = map_points(calibration.H, det.corners)
                        edge = corners[1] - corners[0]
                        legacy_yaws[det.tag_id].append(math.degrees(
                            math.atan2(edge[1], edge[0])))
        loop_times.append(time.perf_counter() - loop_start)
    elapsed = time.perf_counter() - started
    result = diagnostics.as_dict(backend)
    result.update({
        "quad_decimate": decimate,
        "refine_edges": refine,
        "elapsed_sec": elapsed,
        "processing_fps": len(frames) / elapsed,
        "apriltag_timing": timing(detection_times),
        "total_loop_timing": timing(loop_times),
        "calibration_refs": calibration.used_ids,
        "calibration_ref_counts": calibration.reference_counts,
        "calibration_rms_in": calibration.rms_in,
        "backend_counter_available": backend.available,
        "undistort_timing": (
            timing(undistort_times[1:] or undistort_times)
            if undistort_times else None),
        "legacy_one_edge_raw_yaw": {
            tag_id: SessionDiagnostics._yaw_summary(values, None)
            for tag_id, values in sorted(legacy_yaws.items())
        },
    })
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_dir", type=Path)
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--tag-size", type=float, default=3.835)
    parser.add_argument("--freeze-calib", type=int, default=45)
    parser.add_argument("--freeze-min-refs", type=int, default=4)
    parser.add_argument("--camera-calibration", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    paths = sorted(
        path for path in args.image_dir.iterdir()
        if path.suffix.lower() in {".png", ".jpg", ".jpeg"})
    frames = [cv2.imread(str(path), cv2.IMREAD_COLOR) for path in paths]
    if not paths or any(frame is None for frame in frames):
        raise SystemExit(f"no readable images in {args.image_dir}")
    print(f"Preloaded {len(frames)} frames; disk/JPEG read time is excluded.")
    results = [
        run(frames, 1.0, True, args),
        run(frames, 1.5, True, args),
        run(frames, 2.0, True, args),
        run(frames, 1.5, False, args),
    ]
    payload = {
        "image_dir": str(args.image_dir),
        "frame_count": len(frames),
        "results": results,
    }
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
