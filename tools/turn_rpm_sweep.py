#!/usr/bin/env python3
"""Open-loop pure-rotation sweep: what does each wheel RPM actually do?

WHY
---
Switching the fleet to AGV_TURN_METHOD=wheel on 2026-09-24 removed the
no-motion failures -- a six-robot run completed with no collision for the
first time -- but final headings landed 6-9deg off, against the 0.1deg this
user's earlier robots held with the same control law.

turn_to_heading()'s taper is currently --turn-max-speed 35, --turn-min-speed
10, --turn-creep-speed 4, --turn-decel-zone-deg 35. Nothing in that set has
been measured on this hardware. In particular 4 RPM is low enough to be
plausibly BELOW the drive train's stiction floor, in which case the final
approach commands a creep the wheels never execute and the turn ends wherever
the taper handed off -- which would look exactly like a 6-9deg miss.

This measures the three things needed to fix the taper honestly, rather than
tuning it by guess:

  1. deg/s achieved per commanded RPM -- the speed law itself, and whether it
     is linear. The taper assumes it is.
  2. The stiction floor: the lowest RPM that reliably moves the chassis at
     all. --turn-creep-speed must sit above it.
  3. Coast after the stop command, per RPM. That is what --turn-brake-lead-deg
     has to cancel, and it is the other half of a 6-9deg miss.

PURE ROTATION
-------------
Every command is (-rpm, +rpm). Equal magnitude, opposite sign, so the contact
patches counter-rotate about the chassis centre. Any inequality turns this
into an arc and the measurement means nothing -- the script therefore derives
the right wheel from the left and never accepts them separately.

Open loop on purpose: no camera correction, no target, no closed-loop taper.
The point is to characterise the actuator, not to hit a heading.

    python3 tools/turn_rpm_sweep.py --robot Alvik3

Prerequisites: micro-ROS agent + rosbridge up, apriltag_localize.py
publishing, robot on its tag with room to spin in place.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import statistics as st
import sys
import time
from collections import deque
from datetime import datetime

try:
    import roslibpy
except ImportError:                                            # pragma: no cover
    sys.exit("roslibpy is required:  python3 -m pip install roslibpy")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# WHEEL_FOLLOW_MAX_RPM in the firmware is 70.0. constrain(70,-70,70) is 70,
# so 70 itself is NOT clamped -- it is the real ceiling of the control law and
# has to be characterised like any other point.
FIRMWARE_MAX_RPM = 70.0
# 1-10 by 1, then 15-70 by 5. Dense at the bottom because that is where the
# stiction floor lives and a single missing value there is the difference
# between knowing the floor and bracketing it; even steps above 10 because the
# speed law is expected to be linear there and the question is only whether it
# actually is.
SWEEP_RPM = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
             15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70]

# Rough deg/s per commanded RPM for a pure counter-rotation, from the chassis
# geometry (wheel dia ~34mm, track ~89mm): omega = 2*v/track. Used ONLY to
# pick a burst duration -- the measurement itself never assumes it.
DEG_PER_SEC_PER_RPM = 2.3
# Each burst aims to sweep about this far. At a fixed 1.5s, 1 RPM would move
# ~3.4deg, barely above the 0.4deg settle tolerance, and the deg/s estimate
# would be mostly noise. Scaling the duration keeps every RPM measured to
# comparable precision and gives the low end long enough to break stiction.
TARGET_SWEEP_DEG = 40.0
BURST_SEC_MIN, BURST_SEC_MAX = 1.0, 6.0

# The firmware brakes if it has not seen a wheel_cmd in WHEEL_CMD_BRAKE_MS
# (300ms) and aborts at 1000ms, so the stream has to stay well inside that.
WHEEL_STREAM_HZ = 20.0

SETTLE_WINDOW_SEC = 0.60
SETTLE_TOL_DEG = 0.40
SETTLE_MIN_SAMPLES = 6
SETTLE_TIMEOUT_SEC = 8.0
VISION_STALE_SEC = 0.5


def norm(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


# Set once in main(). Every line this script emits carries the robot name so
# six instances can share one terminal and still be readable -- running the
# fleet in parallel is the normal case, not the exception.
LOG_PREFIX = ""


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {LOG_PREFIX}{msg}", flush=True)


def emit(msg: str = "") -> None:
    """Summary lines: prefixed too, for the same reason."""
    print(f"{LOG_PREFIX}{msg}", flush=True)


class Robot:
    def __init__(self, ros, name: str):
        self.name = name
        self.cmd = roslibpy.Topic(ros, f"/{name}_cmd", "std_msgs/String")
        self.wheel = roslibpy.Topic(ros, f"/{name}_wheel_cmd", "std_msgs/String")
        self.pose = roslibpy.Topic(ros, f"/{name}_vision_pose", "std_msgs/String")
        self.poses: deque = deque(maxlen=400)
        self.pose.subscribe(self._on_pose)

    def _on_pose(self, msg):
        try:
            d = json.loads(msg["data"])
            self.poses.append((time.monotonic(), float(d["yaw_deg"])))
        except (ValueError, KeyError, TypeError):
            pass

    def send(self, text: str) -> None:
        self.cmd.publish(roslibpy.Message({"data": text}))

    def spin_wheels(self, left_rpm: float) -> None:
        """Pure rotation only: the right wheel is ALWAYS -left."""
        self.wheel.publish(roslibpy.Message(
            {"data": f"{left_rpm:.1f} {-left_rpm:.1f}"}))

    def yaw_now(self):
        if not self.poses:
            return None
        t, y = self.poses[-1]
        return y if time.monotonic() - t < VISION_STALE_SEC else None

    def settled_yaw(self, timeout=SETTLE_TIMEOUT_SEC):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            win = [(t, y) for t, y in self.poses if now - t <= SETTLE_WINDOW_SEC]
            if len(win) >= SETTLE_MIN_SAMPLES and now - win[-1][0] < VISION_STALE_SEC:
                base = win[0][1]
                rel = [norm(y - base) for _t, y in win]
                if (max(rel) - min(rel) <= SETTLE_TOL_DEG
                        and win[-1][0] - win[0][0] >= 0.8 * SETTLE_WINDOW_SEC):
                    return norm(base + st.median(rel))
            time.sleep(0.03)
        return None

    def close(self):
        for t in (self.cmd, self.wheel, self.pose):
            try:
                t.unsubscribe()
            except Exception:
                pass
            try:
                t.unadvertise()
            except Exception:
                pass


def one_burst(rb: Robot, rpm: float, duration: float) -> dict | None:
    """One open-loop rotation burst at `rpm`, measured end to end."""
    before = rb.settled_yaw()
    if before is None:
        log("  no settled vision before the burst -- skipping")
        return None

    rb.send("WHEEL_FOLLOW_MODE")
    time.sleep(0.15)

    t0 = time.monotonic()
    period = 1.0 / WHEEL_STREAM_HZ
    while time.monotonic() - t0 < duration:
        rb.spin_wheels(rpm)
        time.sleep(period)
    moving_sec = time.monotonic() - t0

    # Yaw at the instant of the stop command, so coast can be separated from
    # the commanded motion. Taken from the live stream, not settled.
    at_stop = rb.yaw_now()
    rb.spin_wheels(0.0)
    time.sleep(0.10)
    rb.send("STOP")

    after = rb.settled_yaw()
    if after is None:
        log(f"  {rpm:+.0f} RPM: no settled vision after the burst")
        return None

    swept = norm(after - before)
    coast = norm(after - at_stop) if at_stop is not None else None
    return {
        "rpm": rpm,
        "duration_sec": round(moving_sec, 3),
        "yaw_before": round(before, 3),
        "yaw_after": round(after, 3),
        "swept_deg": round(swept, 3),
        "deg_per_sec": round(swept / moving_sec, 3) if moving_sec else "",
        "coast_deg": round(coast, 3) if coast is not None else "",
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="Alvik3")
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--rpm", default=None,
                    help="comma-separated override for the RPM sweep")
    ap.add_argument("--repeats", type=int, default=4,
                    help="bursts per RPM; issued alternating in sign so the "
                         "robot stays near its starting heading")
    ap.add_argument("--duration", type=float, default=None,
                    help="fixed seconds of rotation per burst. Default is "
                         "adaptive: each RPM runs long enough to sweep about "
                         f"{TARGET_SWEEP_DEG:.0f}deg, clamped to "
                         f"{BURST_SEC_MIN:.0f}-{BURST_SEC_MAX:.0f}s, so 1 RPM "
                         "is measured to the same precision as 70.")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    global LOG_PREFIX
    LOG_PREFIX = f"{args.robot:<7} "

    rpms = ([float(r) for r in args.rpm.split(",")]
            if args.rpm else [float(r) for r in SWEEP_RPM])
    over = [r for r in rpms if abs(r) > FIRMWARE_MAX_RPM]
    if over:
        log(f"note: {over} are ABOVE WHEEL_FOLLOW_MAX_RPM={FIRMWARE_MAX_RPM:.0f} "
            "and the firmware will clamp them -- they will measure as 70")

    def burst_sec(rpm: float) -> float:
        if args.duration is not None:
            return args.duration
        est = DEG_PER_SEC_PER_RPM * max(abs(rpm), 0.5)
        return max(BURST_SEC_MIN, min(BURST_SEC_MAX, TARGET_SWEEP_DEG / est))

    out = args.out or os.path.join(
        ROOT, "runs_linux",
        f"turn_rpm_{args.robot}_{datetime.now():%m%d_%H%M}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    if not ros.is_connected:
        sys.exit(f"could not reach rosbridge at {args.host}:{args.port}")
    rb = Robot(ros, args.robot)
    log(f"connected; listening for {args.robot} vision pose")

    warm = time.monotonic()
    while time.monotonic() - warm < 10.0 and len(rb.poses) <= 10:
        time.sleep(0.2)
    if len(rb.poses) <= 10:
        rb.close(); ros.terminate()
        sys.exit(f"no vision on /{args.robot}_vision_pose -- is "
                 "apriltag_localize.py running and the robot on camera?")

    total = len(rpms) * args.repeats
    secs = [burst_sec(r) for r in rpms]
    how = (f"{args.duration:.1f}s each" if args.duration is not None
           else f"{min(secs):.1f}-{max(secs):.1f}s each (adaptive)")
    log(f"{len(rpms)} RPM values x {args.repeats} bursts of {how} "
        f"= {total} bursts, ~{sum(s + 3.0 for s in secs) * args.repeats / 60.0:.0f} min; "
        f"writing {out}")
    log("pure rotation throughout: right wheel is always -left")

    rows: list[dict] = []
    fields = ["rpm", "duration_sec", "yaw_before", "yaw_after", "swept_deg",
              "deg_per_sec", "coast_deg"]
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        try:
            n = 0
            for rpm in rpms:
                got = []
                for i in range(args.repeats):
                    signed = rpm if i % 2 == 0 else -rpm
                    n += 1
                    r = one_burst(rb, signed, burst_sec(rpm))
                    if r:
                        rows.append(r)
                        w.writerow(r)
                        fh.flush()
                        got.append(r)
                        log(f"  [{n}/{total}] {signed:+6.1f} RPM -> "
                            f"swept {r['swept_deg']:+7.2f}deg  "
                            f"{r['deg_per_sec']:+7.2f} deg/s  "
                            f"coast {r['coast_deg']}")
                    time.sleep(0.3)
                if got:
                    dps = [abs(float(x["deg_per_sec"])) for x in got
                           if x["deg_per_sec"] != ""]
                    log(f"  {rpm:>5.1f} RPM  n={len(got)}  "
                        f"median {st.median(dps):.2f} deg/s")
        except KeyboardInterrupt:
            log("interrupted -- rows already written are kept")
        finally:
            try:
                rb.spin_wheels(0.0)
                time.sleep(0.2)
                rb.send("STOP")
                time.sleep(0.3)
            except Exception:
                pass
            rb.close()
            ros.terminate()

    if not rows:
        log("no usable samples")
        return

    emit()
    emit(f"  {'RPM':>6} {'n':>3} {'deg/s':>9} {'deg/s sd':>9} "
          f"{'coast deg':>10} {'moved?':>7}")
    by: dict[float, list[dict]] = {}
    for r in rows:
        by.setdefault(abs(r["rpm"]), []).append(r)
    floor = None
    for rpm in sorted(by):
        v = by[rpm]
        dps = [abs(float(x["deg_per_sec"])) for x in v if x["deg_per_sec"] != ""]
        co = [abs(float(x["coast_deg"])) for x in v if x["coast_deg"] != ""]
        moved = sum(1 for d in dps if d > 1.0)
        if moved == len(dps) and floor is None:
            floor = rpm
        emit(f"  {rpm:>6.1f} {len(v):>3} {st.median(dps):>9.2f} "
              f"{(st.pstdev(dps) if len(dps) > 1 else 0.0):>9.2f} "
              f"{(st.median(co) if co else float('nan')):>10.2f} "
              f"{moved}/{len(dps):>2}")
    emit()
    if floor is not None:
        emit(f"  lowest RPM that moved on every burst: {floor:.1f}")
        emit(f"  --turn-creep-speed is currently 4.0 -- it must sit above "
              f"{floor:.1f}")
    emit(f"  wrote {out}")


if __name__ == "__main__":
    main()
