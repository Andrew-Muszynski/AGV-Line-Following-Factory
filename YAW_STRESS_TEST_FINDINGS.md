# Yaw-Source Stress Test Findings

Two independent 128-rotation hardware runs on Alvik6, comparing three
methods for sizing the `ROTATE_REL` command that turns the robot toward a
target heading. Test tool: `camera_grid_navigate.py --yaw-stress-test all`
(`fleet/camera_grid_navigate.py`). Each tier rotates through the same 8
angles (0.9°, 2.6°, 3.0°, 1.5°, 90.0°, 90.3°, 179.0°, 177.5°) × 8 repeats ×
there-and-back = 128 rotations, sends `RESET_POSE` and resyncs onboard yaw
to vision at the start of every tier (so no tier inherits drift from the
one before it), and pauses 5s between tiers.

## The three modes

- **`encoder`** — sizes the `ROTATE_REL` from the robot's own onboard
  `get_pose()` yaw only (local UART link, no camera round-trip). No vision
  check after.
- **`camera_assist`** — same odom-sized `ROTATE_REL`, but after it settles,
  samples vision once and sends one corrective `ROTATE_REL` if still
  outside tolerance (`--turn-tol-deg`, default 3.0°).
- **`camera_only`** — sizes the `ROTATE_REL` directly from vision yaw every
  time, ignoring onboard odometry entirely.

All three go through the same firmware `ROTATE_REL` handler
(`AGV_Factory_camera_correction.ino`), the one that had a documented hang
history (2026-07-30, root-caused and fixed at the firmware level — removed
a `brake()` call that raced with `rotate()`'s internal ack arming, switched
completion detection from `is_target_reached()` polling to a timed
`millis()` deadline). Both runs below: zero hangs.

## Results

### Run 1 — 2026-08-13 (one physical fault, unrelated to yaw-source logic)

| Tier | mean \|error\| | min | max | mean elapsed/turn | total time | aborts |
|---|---|---|---|---|---|---|
| encoder | 2.46° | 0.10° | 7.10° | 1.30s | 166.2s | 0/128 |
| camera_assist | 0.91° | 0.00° | 3.10° | 2.01s | 255.6s | 1/128 |
| camera_only | 2.33° | 0.10° | 7.30° | 1.28s | 162.1s | 1/128 |

One rotation in each of `camera_assist` (tier 2) and `camera_only` (tier 3)
aborted on `ERROR EMERGENCY_STOP` — the Alvik's physical touch-cancel
sensor, tripped by something contacting the robot (most likely the cable)
mid-rotation, not a control or accuracy failure. Confirmed harmless to the
rest of the run: the firmware's `cmdCallback()` unconditionally overwrites
`current_state` on the next recognized command, so the following
`ROTATE_REL` proceeded normally — the LED just stayed red for the
remainder (cosmetic; nothing ever explicitly clears it back to green).

### Run 2 — 2026-08-20 (clean, zero aborts)

| Tier | mean \|error\| | min | max | mean elapsed/turn | total time | aborts |
|---|---|---|---|---|---|---|
| encoder | 2.29° | 0.10° | 6.40° | 1.30s | 166.7s | 0/128 |
| camera_assist | 0.76° | 0.10° | 2.20° | 2.01s | 257.0s | 0/128 |
| camera_only | 2.37° | 0.20° | 6.30° | 1.30s | 166.6s | 0/128 |

## Conclusions

1. **`camera_assist` is a real, repeatable accuracy win.** Both runs land
   within a tight band of each other (mean 0.91° / 0.76°, max 3.10° /
   2.20°) — roughly **2.7–3x more accurate** than `encoder` alone, with a
   meaningfully tighter worst case (max error under 3.5° vs. up to 7.3° for
   the other two modes).
2. **The cost is real but bounded**: +0.7s per turn (1.30s → 2.01s mean),
   a ~55% time increase, from the one extra vision-check + conditional
   corrective `ROTATE_REL`. Correction fired on ~92% of rotations in run 1
   (117/127) and ~92% in run 2 (118/128) — i.e. the odom-only estimate
   needed correcting on nearly every large-angle turn, which is consistent
   with the accuracy gap being real rather than noise.
3. **`camera_only` buys nothing over `encoder` alone.** Across both runs
   the two are statistically indistinguishable on both accuracy (2.3–2.5°
   mean either way) and speed (~1.3s either way). Sizing every `ROTATE_REL`
   from vision instead of odometry does not improve on a freshly-reset
   encoder reading in this setup — plausibly because both runs reset
   onboard yaw at the start of every tier, so `encoder` never accumulates
   the long-run drift `camera_only` would in principle be immune to. This
   result does NOT rule out `camera_only` being better over a long,
   never-resynced route; it only shows no advantage over a single
   128-rotation tier starting from a fresh reset.
4. **Decision, made 2026-08-13**: `camera_assist` is now the default
   behavior in `turn_to_heading_rotate_rel()` (`fleet/camera_grid_navigate.py`),
   the method actually used by `run()` and `fleetSupervisor.py`'s live
   dispatch path. The accuracy/consistency win was judged worth the ~0.7s/
   turn cost for a research testbed prioritizing correct positioning over
   raw mission time.

## Known limitation of both runs

Both runs used a single robot (Alvik6) and reset onboard yaw at the start
of every tier, so neither run tests long-route drift behavior (many turns
between resyncs, as a real multi-leg mission would produce) or
robot-to-robot variation. `turn_to_heading_rotate_rel()`'s own resync
happens once per turn in production (not just once per 128-rotation tier),
which is a more favorable condition than either stress-test run exercised
directly — worth keeping in mind if a future full-route hardware test
shows different numbers than this isolated-rotation benchmark.
