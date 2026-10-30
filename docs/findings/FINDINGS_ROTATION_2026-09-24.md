# Rotation and turn control — data collected 2026-09-24

Written for: the thesis record and anyone picking this testbed up later.

Every number here is measured on the six-robot fleet against the overhead
camera. Where a claim is inference rather than measurement it says so.

---

## 1. Why this day happened

Six-robot missions were failing roughly once per run with `vision leg failed
(turn ... failed)`. The robot would arrive at a node, be commanded to rotate,
and not move — wheels dead, no error from the firmware, telemetry flowing
normally. The supervisor then excluded it, and the exclusion cascaded.

Three separate faults turned out to be stacked on top of each other. They are
separated below because each was measured independently.

---

## 2. `alvik.rotate()` undershoots — a two-parameter, mechanical error

### Model

```
achieved_deg = k * commanded_deg - q
```

Fitted per robot on uncorrected camera-measured rotations (2,762 from the
23 Sept yaw-stress run, plus a 102-rotation 17-angle sweep on Alvik3).
Residual sd 0.24–0.56°, worst single sample 1.86°, across 5–179°.

| robot | k | q (°) | command = f(desired θ) |
|---|---|---|---|
| Alvik1 | 0.9637 | 0.80 | `1.0377·θ + 0.83` |
| Alvik2 | 0.9799 | 0.80 | `1.0205·θ + 0.81` |
| Alvik3 | 0.9504 | 0.80 | `1.0522·θ + 0.84` |
| Alvik4 | 0.9684 | 0.80 | `1.0326·θ + 0.82` |
| Alvik5 | 0.9794 | 0.80 | `1.0211·θ + 0.81` |
| Alvik6 | 0.9737 | 0.80 | `1.0271·θ + 0.82` |

`q` came out 0.64 / 0.79 / 0.85 / 0.80 / 0.75 / 0.85 when fitted independently
per robot — a shared constant, so it is pinned at 0.80.

### Both terms have a mechanism in `Arduino_AlvikCarrier`

- **`k` is a scale error.** The carrier closes its rotate PID on
  `kinematics->getTheta()`, integrated from wheel encoders, which over-reports
  true rotation by 2–5%. Per-robot, as wheel diameter and track are.
- **`q` is a fixed early stop.** `updateKinematics()` drives the turn as
  `drive(0, round(rotate_pid->getControlOutput()/10.0)*10)` with
  `ROTATE_KP_DEFAULT 4.0`, so any error below **1.25°** commands *exactly zero*
  angular velocity, and `ROTATE_THRESHOLD` is 1.0°. The maneuver dies in a dead
  band a fixed distance short of target, independent of the angle asked for.

Two independent derivations landing on the same 0.8° is the strongest evidence
this is a calibration rather than a curve fit.

### The 1.25° dead band, measured directly

Zero-motion rate by commanded angle, n = 2,762, `peakgz < 2 dps`:

| commanded | n | never turned |
|---|---|---|
| 0 – 1.25° | 149 | **144 (96.6%)** |
| 1.25 – 2° | 113 | 1 (0.9%) |
| 2 – 5° | 1,146 | 0 |
| 5 – 20° | 412 | 0 |
| 20 – 120° | 322 | 0 |
| 120 – 200° | 620 | 0 |

A step function at exactly the value the arithmetic predicts. All 52 failed
firmware retries in that run were commanded between 1.0 and 2.2° — they were
arithmetically incapable of moving the robot.

### Result after deploying the calibration

`AGV_Factory_camera_correction_v2.ino`, `carrier_rotate_command()`, applied
only to the value handed to `alvik.rotate()` — `rotate_rel_target_yaw` stays on
the unscaled angle, or the IMU completion gate would target 3–5% past where the
caller asked to stop.

| | before | after |
|---|---|---|
| Alvik3 gain, \|angle\| ≥ 45° | 0.9416 | **0.9985** |
| absolute error, 3–179° | — | **mean 0.17°, worst 0.48°** (n=102) |
| `BRAKED_UNCONFIRMED` (solo sweep) | — | 0 / 79 |
| `BRAKED_UNCONFIRMED` (six-robot mission) | 66.4% | **2.2%** |

---

## 3. A second scale error, in the completion gate

`get_orientation()` under-reports rotation by ~7.5%, measured against the
camera on 23 turns that ended `BRAKED_UNCONFIRMED`:

```
Alvik1  fw 164.56  vs camera 179.66   0.916
Alvik2  fw 166.96  vs camera 179.42   0.931
Alvik3  fw 165.84  vs camera 179.56   0.924
Alvik5  fw 164.41  vs camera 177.73   0.925
Alvik6  fw 166.44  vs camera 179.15   0.929
```

This is a **different sensor** from the carrier's wheel kinematics, and it is
the one the sketch's IMU completion gate reads. The gate needs
`|target − robot_heading_deg|` under `constrain(0.25·|rel|, 1.0, 8.0)`, capped
at 8°, so:

- 179°: 0.075 × 179 = **13.4° > 8.0** → gate never fires → `BRAKED_UNCONFIRMED`
- 90°: 6.8° < 8.0 → fires
- 45°: 3.4° < 8.0 → fires

Which reproduces the observed pattern exactly, including which angles pass.
**Not fixed.** After the §2 calibration this is a false alarm — the turn lands
within half a degree and the firmware simply can't confirm it — so the cost is
time, not correctness. Superseded in practice by §6.

---

## 4. Two supervisor faults, fixed

Both in the deadlock/exclusion path, both found in `base_0924_1336.log`.

**Excluded robots kept their reservations.** `fleetSupervisor.py` released them
only when the failed job was a *fused* turn-then-drive
(`if fused_index[name] is not None`). Alvik5 failed on a plain `drive_leg`, so
nothing was released; Alvik3 waited on "shared depot entrance reserved by
Alvik5" for **38 seconds** after Alvik5 was already dead, Alvik6 queued behind
Alvik3, and the run was aborted by hand. One robot's failure took out three.

Fixed with `DispatchAdvisor.strand()`, called unconditionally on both exclusion
paths. It drops edge and corridor claims but **keeps** the node reservation,
because the dead robot is still physically sitting on that square.

**The yield resolver livelocked.** The advisor had no record of which robots
were mid-yield, and `end_yield()`'s own docstring notes the plan is never
rewritten — so `_simulate_fleet()` kept feeding a yielded robot's original
planned window into `detect_conflicts()`. Alvik1 sidestepped 18→17 to clear
Alvik3; at t=81.1s, with Alvik1 parked at 17, the simulation still showed it on
edge 10↔18, so the advisor told Alvik3 to wait for it. Alvik1 held 42.7 s, gave
up, returned to 18, and the identical deadlock and sidestep fired again.

Fixed with `self.yielding_robots`, maintained by `begin_yield`/`end_yield` and
skipped in `_simulate_fleet()`. Safety is unaffected — that path feeds the
advisory dwell only; the reservation layer still arbitrates movement.

Both changes pass the 69-test advisor suite.

---

## 5. The two movement methods are mutually destructive

`Arduino_AlvikCarrier`'s firmware handles a wheel-speed packet as:

```cpp
case 'J':
  alvik.disableKinematicsMovement();   // kills MOVEMENT_ROTATE
  alvik.disablePositionControl();
  alvik.setRpm(left, right);
```

Drive legs stream wheel speeds (`STATE_WHEEL_FOLLOW`, which the firmware
describes as *"busy/never-idle by design — there is no 'done' for a streamed
mode"*). Turns used `alvik.rotate()`. **Every** observed no-motion failure —
Alvik1 on 22 Sept, Alvik1 and Alvik4 on 24 Sept — was the first rotation after
a drive leg, i.e. the seam between the two.

Supporting eliminations, each with evidence:

- the chassis never twitches (visually confirmed by the operator)
- the ESP32↔carrier link is alive throughout (`qstall` 12–33 ms, `istall` 4–7 ms)
- the same command succeeds by hand, seconds later, on the same robot in the
  same failed state
- the fault has **never** appeared in 1,354 isolated rotations above 5°
  (23 Sept) or 96 more in the 24 Sept gain sweep — neither of which drives
- battery alert cannot latch (`behaviours = 0`, `set_behaviour` never called)
- `RESET_POSE` mid-rotation is excluded (its handler sets `current_state = IDLE`)
- late `_wheel_cmd` is excluded (`wheelCmdCallback` returns unless in
  `STATE_WHEEL_FOLLOW`, and only sets variables)

**Not proven**: the exact ordering by which a `'J'` lands after the `'R'`.
`cmdCallback` sets `current_state` before the state machine runs, which should
close the window. So this is the last candidate standing rather than a
demonstrated mechanism.

Acting on it: `AGV_TURN_METHOD=wheel` makes the whole route use one control
paradigm. **The first six-robot mission under it completed with no collision
and no no-motion failure** — but final headings landed 6–9° off, which §6
addresses.

---

## 6. Characterising the wheel-speed actuator

`tools/turn_rpm_sweep.py`, 22 RPM values × 4 bursts × 6 robots, open loop,
pure counter-rotation (right wheel always `-left`).

### Speed law is linear above 3 RPM

**1.80 deg/s per commanded RPM**, flat from 3 to 35, easing to ~1.75 at 70.
Sub-linear below: 1.61 at 2 RPM, 1.10 at 1.

### There is no meaningful stiction floor above 2 RPM

4 RPM moved on 4/4 bursts for all six robots. The hypothesis that
`--turn-creep-speed 4.0` sat below the floor was **wrong**.

### Coast grows ~0.31° per RPM

| RPM | median coast | sd |
|---|---|---|
| 4 | 1.05–1.42° | 0.09–0.30 |
| 10 | 3.3–4.6° | 0.52–1.90 |
| 25 | 7.0–8.5° | 0.42–0.95 |
| 45 | 13.4–14.6° | 0.38–1.55 |
| 70 | 21–24° | 0.90–7.54 |

### Which explains the 6–9°

`--turn-tol-deg 4.0` stopped the taper the moment it was inside 4°, **at
whatever speed it was doing**, and then it coasted. Exit around 10 RPM and you
get 4° of tolerance plus ~4° of coast. Neither stiction nor the creep.

### Sign convention, measured

A **positive** left-wheel RPM produces a **negative** yaw change on every
robot. Closing the loop therefore needs `left = -sign(remaining)`. Getting this
backwards drives the robot away from target until the timeout — observed on all
six during a first attempt.

### Hardware note

The light blockers were removed from Alvik1–4 partway through the day. Alvik5
and Alvik6 had been consistently ~7% faster at every RPM; they were the two
already without blockers. All constants measured before that change are void,
and the fleet now behaves uniformly.

---

## 7. The staged turn law

`tools/turn_two_stage_test.py`, closed loop on camera yaw, measuring final
error and total time directly rather than inferring them from constants.

```
while |remaining| > lead:
    rpm = fast   if |remaining| > handoff
          final  if |remaining| <= handoff2
          slow   otherwise
    drive (-rpm, +rpm)
stop
```

### Stage-1 handoff: 40°

87–90 turns per value, lead corrected to 9.3:

| handoff | mean \|err\| | p90 | worst | 180° time | overshoot |
|---|---|---|---|---|---|
| **40** | 1.13 | 2.34 | 3.28 | **1.79 s** | +0.51 |
| 50 | 1.11 | 2.25 | 3.24 | 1.92 s | −0.52 |
| 60 | 1.19 | 2.21 | 3.46 | 2.00 s | −0.78 |

Accuracy flat; 40 is fastest, and the overshoot sign flips from + to − between
40 and 50, so the optimum is right there. Handoff must be a **minimum angle,
not a fraction** — coast from 70 RPM is 21–24°, so a "last 25%" rule gives only
11° on a 45° turn and the robot blows through the target.

### Stage-2 handoff: 12°

Overshoot at each H2 is almost pure bias (the robot still carrying 25 RPM speed
into a 1.7° lead), which the lead corrects. What remains is the scatter:

| H2 | bias | sd | residual worst | crawl | meets 2°? |
|---|---|---|---|---|---|
| 4 | +5.27 | 1.08 | 2.55 | 0.04 s | no |
| 6 | +4.05 | 1.21 | 2.29 | 0.07 s | no |
| 8 | +2.63 | 1.01 | 2.06 | 0.11 s | just no |
| **12** | **+1.01** | **0.55** | **1.35** | **0.25 s** | **yes** |
| 16 | +0.62 | 0.70 | 3.76 | 0.43 s | no (one Alvik6 outlier) |

H2 = 8 passes on all six with *per-robot* leads (worst 1.39–1.65) but saves only
0.14 s per turn — not worth six constants.

### Control rate

`apriltag_localize.py --no-preview --stream` publishes ~300 poses/s across six
robots, ~50 Hz each. The control loop was raised from 20 Hz to **50 Hz** to
match: at 25 RPM the robot travels 2.25° between 20 Hz decisions but only 0.90°
between camera frames, so the loop, not the camera, was setting how far past
the stop threshold it could get.

### Final constants

```
fast     70 RPM   while remaining > 40
slow     25 RPM   while 12 < remaining <= 40
final     6 RPM   while remaining <= 12
stop at  remaining <= 2.71
control loop 50 Hz
```

| | original taper | staged |
|---|---|---|
| worst error | 6–9° | **1.35°** |
| 90° turn | 3.4–5.7 s | ~1.3 s |
| 180° turn | — | 2.02 s |

Faster than `alvik.rotate()` was (1.0–1.7 s per 90° on the 2026-08-13 bench) at
better accuracy, without the no-motion failure mode.

The tight law costs only **0.23 s more per turn** than the two-stage version, so
no node-class distinction is warranted — running it everywhere gives ±1.35° at
intersections as well as at workstations and depots, and removes a mechanism
instead of adding one. This comfortably meets the ±178° docking requirement and
is roughly five times better than the ±5° misalignment that was causing
workstation collisions.

---

## 8. Still open

- **§3**, the `get_orientation()` 7.5% under-report. Harmless under the staged
  law, which never consults it, but it still mis-reports on any `ROTATE_REL`.
- **No collision detector.** Nothing compares two robots' vision poses for
  overlap; the 24 Sept Alvik5→Alvik2 impact left no trace but a
  `drive_leg failed`. `VisionMonitor` already has every pose at ~50 Hz.
- **`current_node` lags reality.** It advances only in `record_done()`, so a
  robot that has arrived but not had its leg recorded still reserves the node it
  left — and leaves the one it occupies free. Two robots reported `DE1`
  simultaneously. This is upstream of the collision above.
- **Unbounded advisor wait.** `"no finite safe delay; waiting for fleet state
  change"` has no timeout and no escalation, unlike the deadlock path's 20 s
  abort. It froze Alvik2 in another robot's path for 7+ seconds.
- **Startup discovery race.** 4 of 6 robots missed `RESET_POSE` at stage start
  (`_cmd` is BEST_EFFORT and the publisher had not matched). Confirmed from a
  rosbag: the robots never answered, and by stage 2 it was 12/12.
- **Alvik1 intermittent no-motion**, 37/72 turns in one run, recovered by
  itself. Mechanical check warranted.
