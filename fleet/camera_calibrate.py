#!/usr/bin/env python3
"""Fisheye camera calibration for the overhead NexiGo N980P webcam.

Two modes:
  --capture   Live preview from camera_bridge_windows.py (same TCP frame
              source apriltag_localize.py uses), press SPACE to save a
              checkerboard frame, 'q' to stop. Frames saved as PNGs to
              --out-dir.
  --solve     Run cv2.fisheye.calibrate() over all saved frames in --out-dir,
              print the resulting camera matrix + distortion coefficients,
              and save them as JSON.

Uses cv2.fisheye, NOT the standard cv2.calibrateCamera() -- confirmed
2026-08-03/04 (see memory: isaac_ros_apriltag_gpu) that this lens is
genuine fisheye-class: NexiGo's own datasheet states 120deg FOV at 2.43mm
focal length on a 1/2.7in sensor, which under a plain rectilinear/pinhole
model would only produce ~96deg FOV -- the stated 120deg is only reachable
through real fisheye (equidistant-family) projection, which the standard
model's polynomial distortion terms aren't designed to fit well.

Board: 9x6 interior corners, 19mm squares (see camera_calibration_checkerboard.png,
printed and print-scale-verified 2026-08-04 -- measured 10.00cm reference
line came out exactly correct, so 19mm squares are trustworthy as printed).

Usage:
    # capture (run from WSL2, same as apriltag_localize.py):
    python3 fleet/camera_calibrate.py --capture --port 8765 --out-dir calib_frames

    # after collecting 15-20+ good frames:
    python3 fleet/camera_calibrate.py --solve --out-dir calib_frames --out camera_intrinsics.json
"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

BOARD_COLS = 9  # interior corners
BOARD_ROWS = 6
SQUARE_MM = 19.0

# cv2.fisheye wants object points as float32, shape (1, N, 3)
_OBJP = np.zeros((1, BOARD_COLS * BOARD_ROWS, 3), np.float32)
_OBJP[0, :, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2) * SQUARE_MM

_FIND_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
               | cv2.CALIB_CB_NORMALIZE_IMAGE)
_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)


def find_board(gray: np.ndarray) -> np.ndarray | None:
    """Returns sub-pixel-refined corners (N,1,2) float32, or None if the
    board wasn't found in this frame."""
    ok, corners = cv2.findChessboardCorners(gray, (BOARD_COLS, BOARD_ROWS), _FIND_FLAGS)
    if not ok:
        return None
    cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
    return corners


def cmd_capture(args: argparse.Namespace) -> None:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from apriltag_localize import TcpFrameSource  # reuse the exact same frame source

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    existing = sorted(out_dir.glob("frame_*.png"))
    next_idx = (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0

    src = TcpFrameSource(args.port)
    print(f"Connected. {len(existing)} frame(s) already in {out_dir}.")
    if args.auto_save > 0:
        print(f"AUTO-SAVE armed: saves automatically whenever the board is "
              f"detected, at most once every {args.auto_save:.1f}s -- move "
              "to a new pose between saves. 'u' = undo last save, 'q' = quit "
              "still work.")
    else:
        print("SPACE = save frame (only if checkerboard detected), "
              "q/ESC = quit, u = undo last save")
    saved_this_run: list[Path] = []
    last_auto_save = 0.0

    window = "camera_calibrate -- SPACE=save, u=undo, q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    window_sized = False

    while True:
        ok, frame = src.read()
        if not ok or frame is None:
            print("frame read failed, retrying...")
            continue
        if not window_sized:
            # WSLg doesn't reliably auto-size WINDOW_NORMAL to the image --
            # force it explicitly to the real frame size once we know it,
            # rather than leaving it at HighGUI's small default.
            h, w = frame.shape[:2]
            cv2.resizeWindow(window, w, h)
            window_sized = True
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_board(gray)
        now = time.monotonic()

        display = frame.copy()
        auto_save_ready = (
            args.auto_save > 0 and corners is not None
            and now - last_auto_save >= args.auto_save)
        if corners is not None:
            cv2.drawChessboardCorners(display, (BOARD_COLS, BOARD_ROWS), corners, True)
            if args.auto_save > 0:
                remaining = max(0.0, args.auto_save - (now - last_auto_save))
                status = ("BOARD FOUND -- saving..." if auto_save_ready
                           else f"BOARD FOUND -- next auto-save in {remaining:.1f}s")
            else:
                status = "BOARD FOUND -- press SPACE to save"
            color = (0, 255, 0)
        else:
            status = "no board detected"
            color = (0, 0, 255)
        cv2.putText(display, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, color, 2, cv2.LINE_AA)
        cv2.putText(display, f"saved this run: {len(saved_this_run)}  "
                              f"total in {out_dir.name}: {next_idx}",
                    (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(window, display)

        def _save(frame_to_save) -> None:
            nonlocal next_idx
            path = out_dir / f"frame_{next_idx:03d}.png"
            cv2.imwrite(str(path), frame_to_save)
            saved_this_run.append(path)
            print(f"  saved {path.name} ({len(saved_this_run)} this run, "
                  f"{next_idx + 1} total)")
            next_idx += 1

        if auto_save_ready:
            _save(frame)
            last_auto_save = now

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord(' '):
            if args.auto_save > 0:
                continue  # SPACE is a no-op in auto-save mode -- avoid double-saves
            if corners is None:
                print("  no board detected -- not saved")
                continue
            _save(frame)
        elif key == ord('u'):
            if saved_this_run:
                last = saved_this_run.pop()
                last.unlink(missing_ok=True)
                next_idx -= 1
                print(f"  undid {last.name}")
            else:
                print("  nothing to undo this run")

    cv2.destroyAllWindows()
    src.release()
    print(f"\nDone. {next_idx} frame(s) total in {out_dir}.")
    if next_idx < 15:
        print(f"WARNING: only {next_idx} frames -- 15-20+ with varied angles/"
              "distances/positions is recommended for a stable fisheye fit.")


def cmd_solve(args: argparse.Namespace) -> None:
    out_dir = Path(args.out_dir)
    frame_paths = sorted(out_dir.glob("frame_*.png"))
    if not frame_paths:
        print(f"no frame_*.png files found in {out_dir}", file=sys.stderr)
        raise SystemExit(1)

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    used: list[str] = []
    skipped: list[str] = []

    for p in frame_paths:
        img = cv2.imread(str(p))
        if img is None:
            skipped.append(f"{p.name} (unreadable)")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != image_size:
            skipped.append(f"{p.name} (size mismatch: "
                            f"{gray.shape[1]}x{gray.shape[0]} vs {image_size})")
            continue
        corners = find_board(gray)
        if corners is None:
            skipped.append(f"{p.name} (board not found)")
            continue
        objpoints.append(_OBJP.copy())
        imgpoints.append(corners.reshape(1, -1, 2))
        used.append(p.name)

    print(f"{len(used)} frame(s) usable, {len(skipped)} skipped:")
    for s in skipped:
        print(f"  SKIP {s}")
    if len(used) < 10:
        print(f"\nERROR: only {len(used)} usable frames -- need at least "
              "10 (15-20+ recommended) for a stable fisheye calibration.",
              file=sys.stderr)
        raise SystemExit(1)

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    calib_flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                   | cv2.fisheye.CALIB_CHECK_COND
                   | cv2.fisheye.CALIB_FIX_SKEW)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    # CALIB_CHECK_COND rejects views whose extrinsics solve is numerically
    # ill-conditioned (e.g. the board was too near-parallel to the image
    # plane, or too small/flat in that one frame for stable pose recovery)
    # -- its error message names the OFFENDING FRAME's index directly
    # ("input array N"), which maps 1:1 onto `used[N]` since imgpoints/
    # objpoints were appended in the same order `used` was built above.
    # Added 2026-08-05 (camera raised to ~100in, checkerboard capture via
    # new --auto-save mode produced 100+ frames -- with that many, dropping
    # a handful of bad ones and retrying automatically is both safe (still
    # well over the 10-frame minimum) and much faster than a manual
    # frame-by-frame hunt): parse the index out of the exception, drop that
    # frame, retry, repeat up to a few times rather than failing outright
    # on the first bad view.
    dropped: list[str] = []
    max_retries = 10
    for attempt in range(max_retries + 1):
        n = len(used)
        rvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
        tvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                objpoints, imgpoints, image_size, K, D, rvecs, tvecs,
                calib_flags, criteria)
            break
        except cv2.error as exc:
            m = re.search(r"input array (\d+)", str(exc))
            if m is None or attempt == max_retries:
                print(f"\ncv2.fisheye.calibrate() failed: {exc}", file=sys.stderr)
                print("This usually means one or more frames have a bad/"
                      "ambiguous corner detection (CALIB_CHECK_COND rejects "
                      "ill-conditioned views) -- try removing extreme-angle "
                      "frames and re-running, or capture a few more frames "
                      "with gentler tilt.", file=sys.stderr)
                if dropped:
                    print(f"(already auto-dropped {len(dropped)} frame(s) "
                          f"this run: {', '.join(dropped)})", file=sys.stderr)
                raise SystemExit(1)
            bad_idx = int(m.group(1))
            bad_name = used[bad_idx]
            print(f"  ill-conditioned view at frame {bad_name} (array index "
                  f"{bad_idx}) -- dropping and retrying "
                  f"({n - 1} frame(s) remaining)")
            dropped.append(bad_name)
            del used[bad_idx]
            del objpoints[bad_idx]
            del imgpoints[bad_idx]
            if len(used) < 10:
                print(f"\nERROR: down to {len(used)} usable frames after "
                      "dropping ill-conditioned views -- need at least 10. "
                      "Capture more frames.", file=sys.stderr)
                raise SystemExit(1)

    if dropped:
        print(f"\nAuto-dropped {len(dropped)} ill-conditioned frame(s): "
              f"{', '.join(dropped)}")
        print(f"Final fit used {len(used)} frame(s).")

    print(f"\nRMS reprojection error: {rms:.4f} px "
          f"({'good' if rms < 1.0 else 'high -- consider more/better frames'})")
    print(f"Image size: {image_size[0]}x{image_size[1]}")
    print(f"Camera matrix K:\n{K}")
    print(f"Distortion coefficients D (k1,k2,k3,k4):\n{D.ravel()}")

    result = {
        "model": "fisheye",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.ravel().tolist(),
        "rms_reprojection_error_px": float(rms),
        "num_frames_used": len(used),
        "board_cols": BOARD_COLS,
        "board_rows": BOARD_ROWS,
        "square_size_mm": SQUARE_MM,
    }
    out_path = Path(args.out)
    out_path.write_text(json.dumps(result, indent=2))
    print(f"\nSaved to {out_path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--capture", action="store_true",
                       help="live capture mode: preview + save checkerboard frames")
    mode.add_argument("--solve", action="store_true",
                       help="run fisheye calibration over saved frames")
    ap.add_argument("--out-dir", default="calib_frames",
                     help="directory to save/read captured frames (default calib_frames)")
    ap.add_argument("--port", type=int, default=8765,
                     help="camera_bridge_windows.py TCP port, for --capture (default 8765)")
    ap.add_argument("--out", default="camera_intrinsics.json",
                     help="output JSON path, for --solve (default camera_intrinsics.json)")
    ap.add_argument("--auto-save", type=float, default=0.0, metavar="SECONDS",
                     help="for --capture: automatically save whenever the "
                          "board is detected, at most once every SECONDS "
                          "(default 0 = off, requires pressing SPACE as "
                          "usual). Added 2026-08-05 after the camera was "
                          "raised to ~100in -- holding the checkerboard up "
                          "AND reaching a keyboard at that height isn't "
                          "practical for one person. Recommended: 1.5-2.0 "
                          "(gives enough time to move to a new pose between "
                          "saves). 'u' (undo) and 'q' (quit) still work "
                          "normally alongside auto-save.")
    args = ap.parse_args()

    if args.capture:
        cmd_capture(args)
    else:
        cmd_solve(args)


if __name__ == "__main__":
    main()
