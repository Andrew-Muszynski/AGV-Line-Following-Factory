# Staged-turn findings, 2026-09-25

Source: `runs_linux/supervisor_live.log` (2,272,331 bytes), covering the fleet
runs of 2026-09-25. **1,156 staged turns** parsed — every turn that logged the
current `[loop NN.NHz ... worst gap NNms]` format, so the numbers below are
scoped to the code as it runs today, not to older runs in the same file.

Every claim here is marked **measured** (counted from the log) or **inferred**
(a mechanism consistent with the log but not proven by it). Nothing was run on
hardware to produce this document; it is log analysis only.

---

## 1. The headline

Two separate problems, one root cause.

| | |
|---|---|
| **Visible** | 10 of 12 `vision leg failed` events are one line of code: the no-progress guard in `turn_to_heading_staged()`, in the final 6 RPM stage. |
| **Silent, and worse** | **159 turns (14%) settled more than 5 deg off target and were logged `staged turn complete`.** +/-5 deg is the stated bay-collision threshold. |

The leg failures are the visible tail of the silent problem. Both come from the
same place: once a turn uses 70 RPM, nothing downstream of it is under control.

---

## 2. The staged law does not execute (measured)

Achieved angular rate per stage, against what that stage commands:

| stage | commanded | **achieved, median** | range |
|---|---|---|---|
| 25 RPM | 45.0 deg/s | **141.4 deg/s** | 51.6 - 406.8 |
| 6 RPM | 10.8 deg/s | **38.7 deg/s** | - |
| 6 RPM **from rest** | 10.8 deg/s | **3.5 deg/s** | 1.1 - 8.6 |

Read those three rows together:

* The 25 RPM stage runs 3x faster than commanded, up to 9x. With `h1=65, h2=45`
  it is a 20 deg window, crossed in one or two control ticks. **It is a no-op.**
* The 6 RPM stage runs 3.6x faster than commanded — it is the robot coasting
  down from 70 RPM, not crawling.
* But 6 RPM *from a standstill* runs 3x **slower** than nominal (3.5 deg/s, not
  10.8), across 311 turns, **with zero stalls**.

So the final stage has two completely different behaviours depending on how it
is entered, and neither is the 10.8 deg/s the tuning assumed. The `lead = 2.71`
constant was fitted against a rate that never occurs.

**Consequence:** on any turn over ~65 deg, the robot goes to 70 RPM and then
coasts. Whether that coast happens to bottom out inside +/-2.71 deg when a
vision sample lands decides "complete" versus "stall." It is a coin flip.

## 3. It is not a tuning problem (measured)

Stalls and successes are statistically indistinguishable where the tuning acts:

| | completed | stalled |
|---|---|---|
| angle when the 6 RPM stage began | 38.4 deg | 39.4 deg |
| rate when the 6 RPM stage began | 38.7 deg/s | 43.8 deg/s |

Moving `h1`/`h2` cannot separate two populations that overlap this completely.
They are the same event landing in different places.

## 4. It is not comms, firmware, or pose rate (measured)

| | |
|---|---|
| control loop rate | median **40.7 Hz**, range 25.7 - 44.6 |
| worst tick gap, any turn | median 23 ms, **maximum 35 ms** |
| firmware brake watchdog (`WHEEL_CMD_BRAKE_MS`) | 300 ms |

The worst gap across 1,156 turns is 35 ms — an order of magnitude under the
watchdog. No turn in this log was braked by the firmware. The 50 Hz pose feed,
the micro-ROS link, and `RESET_POSE` are all exonerated for this failure.

Firmware was also checked directly: `STATE_WHEEL_FOLLOW` calls
`alvik.set_wheels_speed(l, r, RPM)` with no minimum clamp, so the 6 RPM
setpoint does reach the carrier.

## 5. Where the error actually is (measured)

Settled error on turns logged `complete`, n = 1,145.

**By turn size — this is the whole story:**

| turn size | n | median | >2.71 deg | **>5 deg** | worst |
|---|---|---|---|---|---|
| under 10 deg (never leaves 6 RPM) | 227 | 1.5 | 5% | **0%** | 5.9 |
| 10-45 deg (never leaves 6 RPM) | 84 | 1.8 | 1% | **0%** | 2.7 |
| over 65 deg (uses 70 RPM) | 834 | 2.6 | 46% | **19%** | 19.1 |

Turns that never touch 70 RPM are excellent — one turn out of 311 exceeded
5 deg. Turns that do are wrong 19% of the time. **The 70 RPM stage is the
defect.**

**By purpose:**

| | n | median | >2.71 deg | **>5 deg** | worst |
|---|---|---|---|---|---|
| square up (workstation / depot dock) | 137 | 3.2 | 55% | **25%** | 19.1 |
| turn at intersection | 965 | 2.1 | 33% | **13%** | 18.3 |
| turn after dwell | 43 | 1.8 | 9% | 2% | 5.9 |

Docking square-ups are the worst bucket, and they are the one place where
+/-5 deg causes a physical collision.

**By robot, turns over 65 deg only:**

| robot | n | median | >2.71 deg | **>5 deg** | worst |
|---|---|---|---|---|---|
| Alvik1 | 144 | 2.0 | 23% | 3% | 17.6 |
| Alvik2 | 143 | 2.4 | 35% | 10% | 9.3 |
| Alvik3 | 174 | 1.9 | 21% | 3% | 10.8 |
| Alvik4 | 146 | 2.4 | 42% | 21% | 19.1 |
| Alvik5 | 126 | 4.6 | 91% | **41%** | 12.2 |
| Alvik6 | 101 | 5.1 | 88% | **50%** | 18.3 |

Alvik5 and Alvik6 are an order of magnitude worse than Alvik1 and Alvik3 on the
same code and the same law. **Inferred:** a per-robot difference in achievable
deceleration (drivetrain friction, battery sag, wheel condition). Consistent
with the stall list below, which Alvik6 dominates — but not proven here.

## 6. The eleven stalls (measured)

| robot | turn | magnitude | stalled at | 6 RPM stage began at |
|---|---|---|---|---|
| Alvik6 | square up at D6 to 0 deg | 178.4 | 3.6 | 39.4 |
| Alvik5 | at 123 toward 123->74 | 100.3 | 13.3 | 35.0 |
| Alvik6 | square up at 94 to 180 deg | 165.7 | 17.4 | 41.8 |
| Alvik4 | at 55 toward 55->47 | 100.2 | 8.2 | 35.9 |
| Alvik4 | at 0 toward 0->1 | 89.7 | 10.6 | 40.4 |
| Alvik5 | at 1 toward 1->2 | 88.1 | 7.5 | 37.9 |
| Alvik6 | at 143 toward 143->43 | 95.9 | 9.0 | 40.4 |
| Alvik3 | at 6 toward 6->14 | 90.0 | 7.7 | 32.8 |
| Alvik6 | square up at 94 to 180 deg | 162.1 | 8.3 | 38.6 |
| Alvik6 | at 144 toward 144->95 | 107.4 | 10.9 | 43.0 |
| Alvik6 | square up at 95 to 180 deg | 180.0 | 6.4 | 43.4 |

Every stall is a turn of 88 deg or more — i.e. every one used 70 RPM. No turn
under 88 deg stalled, in 1,156 turns.

The guard requires no 0.2 deg improvement for 1.22 s. Even the slowest observed
from-rest crawl (1.1 deg/s) clears 0.2 deg in 180 ms, so the robot was
genuinely stopped — moving under 0.16 deg/s — while 6 RPM was being commanded
at ~40 Hz.

**Inferred, not proven:** 6 RPM commanded *from rest* moves the robot (311/311
turns), but 6 RPM commanded *after decelerating from 70 RPM* can leave it dead.
The most likely mechanism is the carrier's wheel PID parking below the motor
deadband after braking, needing its integrator to unwind before it can push
again. Confirming this requires the raw `/<robot>_vision_pose` stream through a
stall, which this log does not contain.

## 7. What this explains beyond the leg failures

A robot that begins a drive leg up to 19 deg mis-aimed will leave the tape.
`drive_leg()` steers on bearing to the target node with **no cross-track term**
(see `tools/drive_straightness.py`), so it cuts a diagonal rather than
rejoining the line. **Inferred:** some of the off-route exclusions, bay
collisions and short-stops previously attributed to `drive_leg()` tuning
originate one step earlier, in the turn that preceded them.

## 8. Fix direction

1. **Stall escape** — when the no-progress guard trips while still outside
   `lead`, ramp the commanded speed and re-arm instead of failing the leg.
   Purely defensive: it can only convert a route abort into a recovery. This is
   what removes the `vision leg failed` events.
2. **Continuous taper on the approach** — replace the fixed 6 RPM final stage
   with speed proportional to remaining angle, floored above the deadband and
   capped at `fast`, so the robot is never carrying more speed than it can shed
   in the angle it has left. This is what addresses the 19%.

Item 1 is safe to deploy on reasoning alone. **Item 2 changes the motion
profile and must be measured on hardware before it is trusted** — its constants
are derived from this log, not validated on a robot.

## 9. What would confirm the inferences

* Record `/<robot>_vision_pose` through a deliberate stall (repeat 180 deg
  square-ups on Alvik6 until one trips) to settle section 6's mechanism.
* Re-run this analysis against the next fleet log. The three numbers that must
  move: achieved rate in the final stage (should approach commanded),
  percentage over 5 deg on turns above 65 deg (currently 19%), and stall count
  (currently 11).
