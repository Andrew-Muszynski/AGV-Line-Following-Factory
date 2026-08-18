# Vision Pipeline Throughput Session — 2026-08-18

## Starting point

After the fleet yaw-stress-test hardware run completed successfully, live
`apriltag_localize.py` runs against all 6 robots showed the vision pipeline
capped around **20-26Hz total loop rate** (`bench: total_loop[rate=...]`),
with `rosbridge: ~120 pose publishes/sec` across 6 robots working out to
~20Hz per robot. Goal for this session: push toward 30fps+ without
sacrificing pose accuracy, and without repeating the earlier `--remote-crop`
mistake (shipped a feature that turned out to have a real coordinate bug AND
didn't even deliver the bandwidth savings it was built for).

## What we found and changed

### 1. `--decimate` default raised 1.0 -> 2.0

`--decimate` (AprilTag's `quad_decimate` parameter) already existed as a CLI
flag but defaulted to 1.0 (full resolution) and was never actually being
passed on any real run. It downsamples the image before the expensive
quad-detection stage inside the detector.

Real 6-robot hardware A/B, same command each time:
```
python3 apriltag_localize.py --frame-source-port 8765 --publish-rate 60 \
  --rosbridge=192.168.0.212:9090 --threads 16 --freeze-calib 45 [--decimate N]
```

| decimate | apriltag stage (median) | total_loop rate | calib rms | pose stability |
|---|---|---|---|---|
| 1.0 (old default) | ~20ms | ~20-26Hz | 0.06in | clean |
| 1.5 | ~9-10ms | ~29-34Hz | 0.06in | clean |
| 2.0 (new default) | ~6-7ms | ~30-38Hz | 0.06in | clean |

2.0 triggers pupil_apriltags' internal `WRN: Matrix is singular.` message far
more often (dozens/run vs 1-2 at 1.5). Traced this (not guessed) to
`refine_edges=1`'s per-tag corner-refinement solve inside the C library
failing to converge more often from a coarser initial corner estimate at
higher decimation. Confirmed non-fatal: it falls back to the unrefined
corner for that one tag on that one frame, never touches `calib.H`, never
crashed the loop across ~2400+ frames of test runs, and no corresponding
pose degradation was observed in any printed pose line. Chose 2.0 as the new
default given the larger throughput win and confirmed-cosmetic warning; 1.5
is the documented fallback in the code comment if this ever needs revisiting.

**Changed in `apriltag_localize.py`**: `--decimate` default `1.0` -> `2.0`,
with an inline comment recording this A/B history and the 1.5 fallback path.

### 2. Preview render throttled to ~15Hz

`draw_overlay()` + `cv2.imshow()` were running on every single loop
iteration — i.e. at the full ~30-38Hz post-decimate detection rate — even
though no human perceives a preview window redrawing faster than ~15fps, and
nothing downstream (rosbridge publish, CSV log) depends on it. The
streamer's own push-to-web-view already had a 10fps self-throttle; the local
OpenCV preview window did not.

Added a `render` bucket to the bench stats first (so the real cost would be
measured, not guessed), then gated `draw_overlay()`/`streamer.push()`/
`cv2.imshow()` together behind a real wall-clock ~15Hz throttle
(`now - last_render >= 1/15`), same pattern as the existing streamer
throttle. `cv2.waitKey(1)` deliberately stays unthrottled every iteration so
the window doesn't appear frozen and hotkeys (q/r/o/u) stay responsive
between render frames.

Measured real cost once instrumented: `render[n=25 rate=12.4/s median=2.7ms]`
— confirms draw+imshow cost ~2.5-3ms per call, now paid ~12-13x/sec instead
of ~30-38x/sec.

**Changed in `apriltag_localize.py`**: added `render` stats bucket,
`last_render`/`RENDER_INTERVAL_SEC` state, gated the draw/stream/imshow
block behind the throttle, added `render[...]` to the printed bench line.

### Combined result (same 6-robot hardware, same command each time)

| Configuration | total_loop rate |
|---|---|
| Baseline: decimate 1.0, unthrottled render | ~20-26Hz |
| decimate 2.0, unthrottled render | ~30-38Hz |
| decimate 2.0, render throttled to 15Hz | **~44-48Hz** |
| decimate 2.0, `--no-preview` (no window at all) | ~60Hz |

More than doubled the starting rate with the live preview window still open,
via two real-hardware-verified, low-risk changes — no code path removed, no
accuracy regression observed in ~2400+ frames of test data, both changes are
plain flag/config-level (fully reversible: drop `--decimate` back to 1.0, or
raise `RENDER_INTERVAL_SEC` back toward 0 to disable throttling).

## What we deliberately did NOT do this session

- **Background TCP receive/decode thread** — the other real lever from the
  earlier ChatGPT code review (decouple JPEG decode from the detection loop
  via a small queue). Not attempted: decimate + render throttle already got
  well past the 30fps target, and this is a bigger, riskier change for a
  smaller expected win (`read()` was only ~4-9ms of a ~20-30ms loop even
  before these changes).
- **`select()` partial-frame blocking fix** in `TcpFrameSource._recv_one_frame()`/
  `read()` — identified as a real risk earlier in the week (can block
  synchronously waiting for the rest of an already-started newer frame with
  no timeout) but not yet fixed. Still outstanding if picked back up later.
- **The yaw offset seen in every run** (all 6 robots reading ~-9 to -19
  degrees) — confirmed by the user to be expected: robots were sitting
  wherever their last rotation test left them, not a live bug. Not touched.

## Where things stand

`apriltag_localize.py`'s current defaults (as of this session) now run at
~44-48Hz total loop rate with the live preview window open, ~60Hz with
`--no-preview`, on the full 6-robot testbed. Both changes are committed
in-code (not just flags you have to remember) — `--decimate 2.0` and the
15Hz render cap are now the defaults, so the original baseline command still
gets the speedup with no extra flags needed.
