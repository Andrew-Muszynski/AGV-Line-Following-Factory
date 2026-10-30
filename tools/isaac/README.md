# Isaac ROS AprilTag (cuAprilTags) — controlled retest, 2026-09-29

Restored from `archive/isaac_ros_apriltag/`, which was archived 2026-08-05.
The pipeline itself worked end to end and was **faster**; it was shelved for
one reason only — detection rate.

> **Answered 2026-09-29. The archive's untested hypothesis was correct:
> cuAprilTags needs an undistorted image.** Feeding it one took five of six
> robot tags from ~2% to a sustained 100%, at ~45 fps. Two things did not
> come back: Alvik6 (92.8%) and corner tag 23 (~2%).
>
> **DECISION, same day: NOT adopted. `apriltag_localize.py` (pupil_apriltags,
> CPU) stays the production detector** — because the CPU pipeline is now
> measured at **52.0 fps and 98.8–100% on every tag**, so the GPU path is
> slower *and* less accurate, with nothing left on its side of the ledger.
> See [Decision](#decision-2026-09-29--noted-not-adopted).
>
> Jump to [Result](#result-of-the-retest-2026-09-29) for the numbers. The
> sections between here and there are the setup, kept because they explain
> *why* these were the two variables to control.

## What the previous attempt established

Measured, same physical conditions, minutes apart:

| | CPU (`pupil_apriltags`) | GPU (`cuAprilTags`) |
|---|---|---|
| detection | **every tag in 955/955 frames** | ~2% on two robot tags, 13–43% on two corner tags |
| throughput | ~28–30 Hz | **~60 Hz** |
| worst stall | ~1.3–1.4 s, all robots at once | **~400 ms** |

So the GPU path wins on the two things that are hurting the fleet right now
(throughput and stall severity) and lost decisively on the one thing that
matters more (does it see the tags).

Everything environmental was already ruled out: camera raised ~50in→~100in
and re-angled, tag orientation corrected, lighting re-tuned with a glare
detector, fresh fisheye calibration at 0.68px RMS. None closed the gap.

## The two variables this retest controls

**1. Rectification — the archive's own leading hypothesis, never tested.**
`cuAprilTags.h` documents that its detection step expects *the undistorted
image*. `isaac_ros_image_proc`'s `RectifyNode` was unusable here: it requires
a genuine NITROS-producing publisher, and the Python image publisher is plain
`rclpy`. So **every frame cuAprilTags ever saw still had fisheye distortion.**

But `camera_bridge_windows.py --undistort` removes distortion *before* the
JPEG is encoded — which sidesteps `RectifyNode` entirely and delivers exactly
what cuAprilTags asks for. The archive notes `--undistort` was tested against
*foreshortening* (where it correctly made no difference, being a different
phenomenon) — not against this requirement.

**2. Full frame.** The old publisher had `--auto-crop` / `--crop-rect-file`
as a throughput optimisation, so Isaac was fed a cropped image. Both default
to OFF here; pass nothing and the full 1920x1080 goes through.

## Running it

Four terminals. `--undistort` and the absence of any crop flag are the
experiment; everything else matches the CPU pipeline.

**Windows** — note `--undistort`, which the normal launcher does NOT use:
```
python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 --jpeg-quality 95 --undistort
```

**WSL2**, ROS sourced, one per terminal:
```
ros2 launch tools/isaac/isaac_apriltag_launch.py
```
```
python3 tools/isaac/isaac_image_publisher.py --port 8765
```
```
python3 tools/isaac/isaac_apriltag_bridge.py --rosbridge 127.0.0.1:9090
```

Stop `apriltag_localize.py` first — both publish the same `<robot>_vision_pose`
topics and two publishers would interleave poses.

## Result of the retest (2026-09-29)

Both controlled variables mattered, and the rectification one mattered
enormously. With `camera_bridge_windows.py --undistort` and no crop:

|                | 2026-08-05 (distorted, cropped) | 2026-09-29 (undistorted, full frame) |
|---|---|---|
| Alvik1–5       | ~2% on two robot tags | **100% in every window** |
| Alvik6         | — | 92.8% |
| corner 20/21/22| 13–43% | **98.3–99.1%** |
| corner 23      | — | **1.6%** |
| throughput     | ~60 Hz (cropped) | ~45 fps (full 1920x1080) |

**Do not compare that 45 fps against the "~28–30 Hz" figure above.** That
number is from 2026-08-05 and is long stale — the E-core affinity pinning,
the `quad_decimate` raise and the preview-render throttle have roughly
doubled the CPU pipeline since. Measured the same day as this retest, over
98,608 frames / 31.6 min, it runs at **52.0 fps**. See the decision section
for the head-to-head.

### tile_size

Swept after the undistort fix, ~30s windows each. `tile_size` is the
adaptive-thresholding window; the full table is in
`isaac_apriltag_launch.py`'s module docstring. **The default is now 8**: it
takes the whole Alvik6 gain (70.1% → 92.8%) and is also the best setting for
tags 21 and 22, for 0.7 fps. 16 adds +2 more on Alvik6 but drops tag 22 to
93.7% and widens the fps spread.

### Positional accuracy

Checked against the six measured depot stickers
(`camera_grid_navigate.py`'s `DEPOT_SLOT_WORLD_IN`) with all robots parked:
**mean 0.26 in, worst 0.38 in.** That is with mismatched intrinsics — the
image is undistorted but the `CameraInfo` still carries the distortion
coefficients — so it should only improve once that is fixed.

Note what this does and does not cover: the depots sit at y≈11 in, x 19–51 in,
which is the bottom-middle of the table, well inside the 20–21 edge. It does
not sample the top-right, which is exactly the region tag 23 anchors.

### The two that did not come back

**Tag 23 (~2%) is not a tuning problem.** It is flat at 1.6–2.5% across every
`tile_size`, and the CPU pipeline sees it fine — the live overlay reads
`calib OK refs=[20, 21, 22, 23] rms=0.06 in` on the same physical scene. So
this is a cuAprilTags weakness on that specific tag, not something physical
about the tag.

Three refs still fit a well-conditioned homography — at ≥3 tags
`fit_table_homography()` refines the per-tag rotations freely instead of
snapping them to 90°, which is the better-conditioned branch. The cost is
coverage, not conditioning: with 20 (BL), 21 (BR) and 22 (TL), the top-right
corner is extrapolated rather than anchored, so error there will exceed what
the depot-row measurement above implies. **Measure a robot parked near the
top-right before trusting this pipeline for the whole table.**

**Alvik6 at 92.8%** means roughly one frame in fourteen has no Alvik6 pose.
Alvik1–5 are at 100%, so this is specific to that tag — worth checking its
print quality, flatness and glare before treating it as a detector limit.

## Decision 2026-09-29 — noted, not adopted

**The CPU pipeline (`apriltag_localize.py`, pupil_apriltags) remains the
production detector.** The gap that sent this to the archive is mostly
closed, and the pipeline is worth keeping runnable — but its only real
advantage is throughput, and throughput is not the binding constraint.

**Poses per robot per second is the number that matters, not frames per
second — and the CPU pipeline wins it on every tag.** Measured the same day
from `apriltag_localize.py`'s own session summary (98,608 frames over
31.6 min, `total_loop rate=52.0/s`), cross-checked against the vision
terminal's `rosbridge: ~320 pose publishes/sec` over six robots (~53/robot/s):

| tag | CPU det% | CPU poses/s | Isaac det% | Isaac poses/s |
|---|---|---|---|---|
| Alvik1 | 99.19% | 51.6 | 100.0% | 45.0 |
| Alvik2 | 98.93% | 51.5 | 100.0% | 45.0 |
| Alvik3 | 98.78% | 51.4 | 100.0% | 45.0 |
| Alvik4 | 99.13% | 51.6 | 100.0% | 45.0 |
| Alvik5 | 99.37% | 51.7 | 100.0% | 45.0 |
| Alvik6 | **99.72%** | **51.9** | 92.8% | 41.8 |
| tag 20 | 99.35% | 51.7 | 98.7% | 44.4 |
| tag 21 | 99.96% | 52.0 | 99.1% | 44.6 |
| tag 22 | 100.00% | 52.0 | 98.3% | 44.2 |
| tag 23 | **100.00%** | **52.0** | 1.6% | 0.7 |

Isaac does not win a single row. Its higher per-tag detection on Alvik1–5 is
more than cancelled by running 7 fps slower, and on Alvik6 and tag 23 it is
not close. **The GPU path has no remaining advantage** — which is a different
and much simpler conclusion than "the trade isn't worth it".

Tag 23 at 100% on CPU vs 1.6% on GPU, same camera and same scene, also
settles what that failure is: a cuAprilTags weakness on that tag, not
anything physical.

**Secondary: `--undistort` costs ~34% more relay bandwidth.** Measured at the
same `--jpeg-quality 95` on both sides, so it is the undistortion and not a
quality difference (this matters — a 2026-09-10 VLM pilot was invalidated by
exactly that confound):

| | per frame | at ~52 fps |
|---|---|---|
| no `--undistort` | ~320 KB | ~16.3 MB/s |
| `--undistort`, full frame | ~429 KB | ~21.8 MB/s |

Probable mechanism: `_build_undistort_maps()` passes `new_K = K` with no
rescaling, so rectifying the fisheye at the original focal length magnifies
radially outward and stretches peripheral edges across many more 8x8 JPEG
blocks. Directionally sound; not independently confirmed, and nothing
depends on it.

Revisiting is hard to justify: a frame-rate-bound workload is the usual
reason, and the CPU pipeline is the faster of the two.

### If it is revisited, three things are open

1. **Tag 23** — needs a fix, or an accepted 3-corner calibration with
   top-right error actually measured. Nothing validated so far samples that
   region.
2. **Alvik6 at 92.8%** — inspect the physical tag; the other five are at 100%.
3. **`CameraInfo` publishes distorted intrinsics alongside an undistorted
   image.** Affects pose, not detection. The 0.26 in accuracy above was
   measured *despite* this.

The one argument that survives is CPU contention, not throughput:
`apriltag_localize.py --threads 8` is the largest CPU consumer on the machine
that now also hosts the micro-ROS agent, and the `WHEEL_CMD_TIMEOUT` stalls
are partly that. Offloading detection to the GPU would relieve it. But that
is an argument for freeing cores, and it would have to be worth giving up
7 fps, tag 23 and part of Alvik6 to get them.

## Before you spend long on it

**The RTX 5060 is Blackwell (sm_120).** cuAprilTags ships as a prebuilt
binary, and an Isaac ROS release predating Blackwell may have no kernels for
it — the failure could be a hard CUDA error, or a silent fallback. Check the
`isaac_apriltag_launch.py` terminal for CUDA/arch errors before trusting any
detection number. If it will not initialise, that is an Isaac ROS version
question, not something to tune.

Prerequisite (installed as of archiving, apt name — the docs' shorthand
`isaac-ros-apriltag` does not exist):
```
ros-jazzy-isaac-ros-apriltag
```

## If detection reaches parity

The throughput and stall numbers above make it worth adopting, and the
current `WHEEL_CMD_TIMEOUT` stalls are partly a CPU-contention problem that
moving AprilTag detection onto the GPU would directly relieve —
`apriltag_localize.py --threads 8` is the largest CPU consumer on the
machine that now also hosts the micro-ROS agent.

If detection does not reach parity, re-archive and note which of the two
variables was tested, so the next person does not repeat it a third time.
