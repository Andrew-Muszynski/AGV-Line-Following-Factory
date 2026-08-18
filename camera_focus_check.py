#!/usr/bin/env python3
"""
camera_focus_check.py — live sharpness + exposure diagnostic for manually
tuning the overhead NexiGo camera's physical focus ring and room lighting.

Added 2026-07-30: the NexiGo exposes NO focus control via OpenCV on either
MSMF or DSHOW (CAP_PROP_FOCUS / CAP_PROP_AUTOFOCUS both read -1.0/unsupported,
every set() call fails, confirmed on hardware) -- this is a mechanical
twist-ring lens, not a software/motorized one. This tool can't turn the ring
for you; it just gives you a live number to watch WHILE you turn it by hand
(and while you adjust room lighting), same standard technique used for any
manual-focus lens.

Two live metrics, both computed on the same central crop every frame (the
table area, not background clutter -- a shaky/moving background would
otherwise dominate the sharpness score and mask real focus changes):

  SHARPNESS -- variance of the Laplacian (cv2.Laplacian(...).var()) of the
  grayscale crop. Higher = sharper edges = better focus. No fixed "good"
  number (depends on scene content/lighting) -- watch for a MAXIMUM as you
  slowly turn the focus ring, not a target value. Turn past the peak, then
  back to it, to confirm it's really the peak and not just still climbing.

  EXPOSURE -- mean brightness (0-255) of the same crop, plus % of pixels
  clipped at each end (0 = pure black, 255 = pure white). Aim for mean
  roughly 90-160 with LOW clip percentages on both ends: too low/dark loses
  shadow detail (risks the sticker-color thresholds in apriltag_localize.py
  missing real detections), too high/bright causes glare/blown highlights
  (risks false color-sticker matches or AprilTag corner-detection failure).
  Overexposed columns/rows in the crop are drawn on the histogram bar itself
  so you can see WHERE the clipping is, not just how much.

Usage:
    python camera_focus_check.py                    # camera index 1, MSMF
    python camera_focus_check.py --camera 0
    python camera_focus_check.py --backend dshow
    python camera_focus_check.py --crop 0.5          # use 50% central crop (default 0.6)
    python camera_focus_check.py --log lighting_pass.csv

--log PATH (added 2026-07-30 for walking-across-the-room lighting
adjustments): appends one row/second (timestamp, sharpness, exposure mean,
clip_low%%, clip_high%%) to a CSV -- so the tuning history is captured even
when you're not standing at the screen watching it happen live. Open it
afterward (Excel, or `python -c "import csv,sys; ..."`) to see exactly how
exposure moved as each adjustment was made, not just wherever it ended up.

Press 'q' to quit. Press 'r' to reset the running peak-sharpness tracker
(useful when you start a fresh focus pass after moving the ring a lot).
Press 'b' to toggle the large across-the-room readout between exposure
(for lighting passes) and sharpness (for focus-ring passes) without
restarting -- or start with the one you want via --big.
"""
from __future__ import annotations

import argparse
import csv
import time

import cv2
import numpy as np

from apriltag_detect import open_camera


def central_crop(frame, frac: float):
    h, w = frame.shape[:2]
    cw, ch = int(w * frac), int(h * frac)
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    return frame[y0:y0 + ch, x0:x0 + cw], (x0, y0, cw, ch)


def sharpness_score(gray_crop) -> float:
    return float(cv2.Laplacian(gray_crop, cv2.CV_64F).var())


def exposure_stats(gray_crop) -> tuple[float, float, float]:
    mean = float(gray_crop.mean())
    total = gray_crop.size
    clipped_low = float((gray_crop <= 2).sum()) / total * 100.0
    clipped_high = float((gray_crop >= 253).sum()) / total * 100.0
    return mean, clipped_low, clipped_high


def draw_histogram(frame, gray_crop, x0: int, y0: int, w: int = 260, h: int = 80) -> None:
    """Small histogram panel, top-left, so clipping at either end is visible
    directly rather than just as a percentage number."""
    hist = cv2.calcHist([gray_crop], [0], None, [256], [0, 256]).flatten()
    hist_norm = hist / (hist.max() + 1e-6)
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (30, 30, 30), -1)
    for i in range(256):
        bar_h = int(hist_norm[i] * (h - 4))
        if bar_h <= 0:
            continue
        bx = x0 + int(i * w / 256)
        color = (0, 0, 255) if i <= 2 else (255, 255, 255) if i >= 253 else (200, 200, 200)
        cv2.line(frame, (bx, y0 + h - 2), (bx, y0 + h - 2 - bar_h), color, 1)
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (120, 120, 120), 1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=1,
                     help="webcam index (default 1 = overhead NexiGo)")
    ap.add_argument("--backend", default="msmf", choices=["dshow", "msmf", "any"])
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--crop", type=float, default=0.6,
                     help="central crop fraction of width/height used for "
                          "both metrics (default 0.6 -- the table area, "
                          "avoids background clutter skewing sharpness)")
    ap.add_argument("--log", metavar="PATH", default=None,
                     help="append one CSV row/second (elapsed_sec, "
                          "sharpness, exposure_mean, clip_low_pct, "
                          "clip_high_pct) to PATH -- for passively "
                          "recording a lighting/focus adjustment pass "
                          "made from across the room, not just whatever "
                          "the metrics read when you get back to the "
                          "screen. Creates PATH with a header row if it "
                          "doesn't exist; appends if it does.")
    ap.add_argument("--big", choices=["exposure", "sharpness"], default="exposure",
                     help="which metric gets the large across-the-room "
                          "readout (default exposure, for tuning room "
                          "lighting from a dimmer/switch across the room; "
                          "use sharpness while turning the focus ring at "
                          "the camera itself). Toggle live with the 'b' "
                          "key instead of restarting.")
    args = ap.parse_args()

    cap = open_camera(args.camera, preferred_backend=args.backend)
    if cap is None:
        raise SystemExit(f"Could not open camera index {args.camera}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    print(f"Capture resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print("Turn the focus ring by hand and watch SHARPNESS for a peak.")
    print("Adjust room lighting and watch EXPOSURE mean (target ~90-160) "
          "and the clip%% on both ends (target low).")
    print("'r' resets the peak tracker, 'q' quits.")

    log_file = None
    log_writer = None
    last_log_t = 0.0
    t_start = time.monotonic()
    if args.log:
        import os
        write_header = not os.path.exists(args.log)
        log_file = open(args.log, "a", newline="")
        log_writer = csv.writer(log_file)
        if write_header:
            log_writer.writerow(
                ["elapsed_sec", "sharpness", "exposure_mean",
                 "clip_low_pct", "clip_high_pct"])
        print(f"Logging one row/sec to {args.log}")

    peak_sharpness = 0.0
    big_metric = args.big
    window = "Focus + Exposure Check"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"Big readout: {big_metric} (press 'b' to switch)")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        crop, (cx0, cy0, cw, ch) = central_crop(frame, args.crop)
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        sharp = sharpness_score(gray)
        mean, clip_lo, clip_hi = exposure_stats(gray)
        peak_sharpness = max(peak_sharpness, sharp)

        if log_writer is not None:
            now_t = time.monotonic()
            if now_t - last_log_t >= 1.0:
                elapsed = now_t - t_start
                log_writer.writerow(
                    [f"{elapsed:.1f}", f"{sharp:.1f}", f"{mean:.1f}",
                     f"{clip_lo:.2f}", f"{clip_hi:.2f}"])
                log_file.flush()  # so a tail -f (or reading mid-run) sees fresh rows
                last_log_t = now_t

        cv2.rectangle(frame, (cx0, cy0), (cx0 + cw, cy0 + ch), (0, 255, 0), 2)

        sharp_color = (0, 255, 0) if sharp >= peak_sharpness * 0.95 else (0, 200, 255)
        in_range = 90 <= mean <= 160 and clip_lo < 2 and clip_hi < 2
        exp_color = (0, 255, 0) if in_range else (0, 165, 255)

        # BIG readout (added 2026-07-30, readable from across the room --
        # exposure while walking a light dimmer/switch, sharpness while
        # turning the focus ring at the camera itself; 'b' toggles which).
        # Everything else stays at the normal small size for when you're
        # close enough to read detail.
        if big_metric == "exposure":
            big_text = f"{mean:.0f}"
            big_color = exp_color
            label = "IN RANGE (90-160)" if in_range else (
                "TOO DARK" if mean < 90 else "TOO BRIGHT")
        else:
            big_text = f"{sharp:.0f}"
            big_color = sharp_color
            label = "AT PEAK" if sharp >= peak_sharpness * 0.95 else \
                f"below peak ({peak_sharpness:.0f})"
        big_scale = 4.5
        (big_w, big_h), _ = cv2.getTextSize(
            big_text, cv2.FONT_HERSHEY_SIMPLEX, big_scale, 10)
        big_x = (frame.shape[1] - big_w) // 2
        big_y = big_h + 20
        cv2.putText(frame, big_text, (big_x, big_y), cv2.FONT_HERSHEY_SIMPLEX,
                    big_scale, (0, 0, 0), 16, cv2.LINE_AA)
        cv2.putText(frame, big_text, (big_x, big_y), cv2.FONT_HERSHEY_SIMPLEX,
                    big_scale, big_color, 10, cv2.LINE_AA)
        (lbl_w, _), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 1.4, 4)
        cv2.putText(frame, label, ((frame.shape[1] - lbl_w) // 2, big_y + 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 8, cv2.LINE_AA)
        cv2.putText(frame, label, ((frame.shape[1] - lbl_w) // 2, big_y + 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, big_color, 4, cv2.LINE_AA)

        y = big_y + 100
        for text, color in [
            (f"SHARPNESS: {sharp:7.1f}   (peak so far: {peak_sharpness:7.1f})", sharp_color),
            (f"EXPOSURE mean: {mean:5.1f}/255   clip_low: {clip_lo:4.1f}%%   clip_high: {clip_hi:4.1f}%%", exp_color),
        ]:
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        color, 2, cv2.LINE_AA)
            y += 32

        draw_histogram(frame, gray, x0=10, y0=y + 8)

        cv2.imshow(window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            peak_sharpness = 0.0
            print("Peak sharpness tracker reset.")
        if key == ord('b'):
            big_metric = "sharpness" if big_metric == "exposure" else "exposure"
            print(f"Big readout: {big_metric}")

    cap.release()
    cv2.destroyAllWindows()
    if log_file is not None:
        log_file.close()
        print(f"Log written to {args.log}")


if __name__ == "__main__":
    main()
