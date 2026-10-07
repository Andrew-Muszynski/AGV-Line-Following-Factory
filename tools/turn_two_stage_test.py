#!/usr/bin/env python3
"""Closed-loop two-stage turn: fast bulk, slow finish. Find the constants.

WHY THIS AND NOT MORE OPEN-LOOP SWEEPS
--------------------------------------
turn_rpm_sweep.py characterised the actuator: the speed law is linear at
~1.80 deg/s per commanded RPM above 3 RPM, and coast-to-zero grows ~0.31deg
per RPM (1.2deg at 4 RPM, 7-8.5deg at 25, 21-24deg at 70).

What that does NOT give is the handoff. Dropping 70 -> 25 is not a stop: the
robot keeps driving, so the angle consumed is smaller than coast-to-zero and
is a different quantity nobody has measured. Rather than measure it and then
infer a handoff from it, this drives the ACTUAL two-stage law at real target
angles and measures what we care about directly -- final error and total time.
One fewer inferred quantity between the data and the decision.

THE LAW UNDER TEST
------------------
    while |remaining| > lead:
        rpm = fast if |remaining| > handoff else slow
        drive (-rpm, +rpm)
    stop

No taper, no decel zone, no creep. Four numbers: fast, slow, handoff, lead.

HANDOFF IS A MINIMUM ANGLE, NOT A FRACTION
------------------------------------------
Coast from 70 RPM is 21-24deg. A "last 25%" rule gives 22.5deg on a 90deg
turn -- right at the edge -- and 11deg on a 45deg turn, which is less than the
coast, so the robot would blow through the target before the slow stage could
take over. A turn shorter than the handoff therefore runs entirely at `slow`,
which is why --angles sweeps well below it.

CALIBRATING lead IN ONE PASS
----------------------------
Signed error is reported, not just magnitude. A consistent bias IS the lead
correction: lead_new = lead_old + mean_signed_error. So a first run with a
rough lead measures the right one rather than needing a search over it.

NOTE (2026-09-24): the light blockers were removed from Alvik1-4, which is
what made Alvik5/6 ~7% faster in the earlier sweep. Every per-robot constant
measured before that change is stale; this is a fresh baseline for all six.

    python3 tools/turn_two_stage_test.py --robot Alvik3

Fleet, live output, all six sharing one terminal:

    for r in Alvik1 Alvik2 Alvik3 Alvik4 Alvik5 Alvik6; do \
      python3 tools/turn_two_stage_test.py --robot $r & done; wait
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics as st
import sys
import time
from collections import deque, defaultdict
from datetime import datetime

try:
    import roslibpy
except ImportError:                                            # pragma: no cover
    sys.exit("roslibpy is required:  python3 -m pip install roslibpy")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Small ones matter most: they are where a 70 RPM first stage stops being
# usable, and the point of the run is to find that boundary rather than assume
# it. 180 is the largest turn the planner ever issues.
ANGLES_DEG = [10, 20, 30, 45, 60, 90, 120, 180]
# Handoff candidates, in degrees remaining. Must exceed the 70 RPM coast
# (21-24deg measured) or stage 1 overshoots before stage 2 can take over.
HANDOFF_DEG = [25, 30, 40]

FAST_RPM = 70.0        # firmware WHEEL_FOLLOW_MAX_RPM; the real ceiling
SLOW_RPM = 25.0        # coast 7-8.5deg, sd <1deg -- the repeatable regime
LEAD_DEG = 7.5         # starting guess at 25 RPM; refined from signed error

# Matched to the vision rate, not to the watchdog. apriltag_localize.py with
# --no-preview --stream publishes ~300 poses/sec across six robots, i.e. ~50Hz
# each, so a 20Hz control loop was the coarser of the two: at 25 RPM the robot
# travels 2.25deg between decisions but only 0.90deg between camera frames, so
# the loop -- not the camera -- set how far past the stop threshold it could
# get. 50Hz matches them. Still well inside the firmware's 300ms
# WHEEL_CMD_BRAKE_MS, and the sleep is unadjusted for processing time so the
# achieved rate lands a little under nominal.
CONTROL_HZ = 50.0
SETTLE_WINDOW_SEC = 0.60
SETTLE_TOL_DEG = 0.40
SETTLE_MIN_SAMPLES = 6
SETTLE_TIMEOUT_SEC = 8.0
VISION_STALE_SEC = 0.5
TURN_TIMEOUT_SEC = 25.0
# MEASURED, not assumed: in turn_rpm_sweep's open-loop data a POSITIVE left
# wheel RPM produced a NEGATIVE yaw change on every robot (Alvik1 +RPM ->
# -9.9, -6.2, -15.7 ...). So closing the loop needs left = -sign(remaining).
# Getting this backwards drives the robot away from target, |remaining| grows
# past the handoff, and it runs at `fast` until the timeout -- which is
# exactly what the first hardware attempt did on all six robots.
LEFT_RPM_SIGN = -1.0
# If |remaining| grows for this many consecutive control ticks the loop is
# diverging. Abort in ~0.6s rather than spin for the full TURN_TIMEOUT_SEC.
DIVERGE_TICKS = 12

LOG_PREFIX = ""


def norm(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {LOG_PREFIX}{msg}", flush=True)


def emit(msg: str = "") -> None:
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

    def spin(self, left_rpm: float) -> None:
        """Pure rotation: right wheel is ALWAYS -left, never accepted apart."""
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
            for fn in ("unsubscribe", "unadvertise"):
                try:
                    getattr(t, fn)()
                except Exception:
                    pass


def two_stage_turn(rb: Robot, target_rel_deg: float, fast: float, slow: float,
                   handoff: float, lead: float, final: float = 0.0,
                   handoff2: float = 0.0,
                   control_hz: float = CONTROL_HZ) -> dict | None:
    """Drive the law once and measure it. Returns None if vision was unusable."""
    start = rb.settled_yaw()
    if start is None:
        log("  no settled vision before the turn -- skipping")
        return None
    target_abs = norm(start + target_rel_deg)

    rb.send("WHEEL_FOLLOW_MODE")
    time.sleep(0.15)

    t0 = time.monotonic()
    period = 1.0 / control_hz
    fast_sec = 0.0
    slow_sec = 0.0
    final_sec = 0.0
    handoff_yaw = None
    last = t0
    timed_out = False
    diverged = False
    worst = None
    growing = 0
    # Accumulated UNWRAPPED rotation. At a 180deg target, start+180 and
    # start-180 are the same heading, so the commanded sign cannot tell you
    # which way the robot actually went -- and 180 is the commonest turn on
    # this grid. Summing per-tick deltas gives an unambiguous travel
    # direction and a true swept angle regardless of wrapping.
    swept = 0.0
    y_prev = None

    while True:
        now = time.monotonic()
        if now - t0 > TURN_TIMEOUT_SEC:
            timed_out = True
            break
        y = rb.yaw_now()
        if y is None:                      # vision gap: hold, do not guess
            rb.spin(0.0)
            time.sleep(period)
            last = time.monotonic()
            continue
        if y_prev is not None:
            swept += norm(y - y_prev)
        y_prev = y
        remaining = norm(target_abs - y)
        if abs(remaining) <= lead:
            break
        # Divergence guard: a wrong sign or a stuck wheel shows up as
        # |remaining| climbing, and without this the loop happily spins the
        # robot for the whole timeout.
        if worst is None or abs(remaining) < worst - 0.2:
            worst, growing = abs(remaining), 0
        else:
            growing += 1
            if growing >= DIVERGE_TICKS:
                diverged = True
                break
        # Stage selection. handoff2 <= lead disables stage 3, so the same
        # code runs the two-stage law unchanged.
        if abs(remaining) > handoff:
            stage, speed = 1, fast
        elif handoff2 > lead and abs(remaining) <= handoff2:
            stage, speed = 3, final
        else:
            stage, speed = 2, slow
        if stage != 1 and handoff_yaw is None:
            handoff_yaw = y                # where stage 2 actually began
        rpm = (speed * LEFT_RPM_SIGN
               * (1.0 if remaining > 0 else -1.0))
        rb.spin(rpm)
        time.sleep(period)
        now2 = time.monotonic()
        if stage == 1:
            fast_sec += now2 - last
        elif stage == 2:
            slow_sec += now2 - last
        else:
            final_sec += now2 - last
        last = now2

    rb.spin(0.0)
    time.sleep(0.10)
    rb.send("STOP")
    moving_sec = time.monotonic() - t0

    after = rb.settled_yaw()
    if after is None:
        log(f"  {target_rel_deg:+.0f}deg: no settled vision after the turn")
        return None

    if y_prev is not None:
        swept += norm(after - y_prev)
    achieved = swept
    err = norm(target_abs - after)
    # Overshoot measured IN THE DIRECTION OF TRAVEL. Turns alternate +/- so
    # the robot stays near its start, which means raw signed error flips sign
    # with them and averages to nothing -- +20 and -20 are the SAME fault
    # twice, not a cancelling pair. Positive here always means "went too far",
    # whichever way it turned, so the mean is the lead correction.
    # From what the robot DID, not what it was asked for: unambiguous at 180.
    travel = 1.0 if swept >= 0 else -1.0
    overshoot = -err * travel
    return {
        "target_deg": target_rel_deg,
        "handoff_deg": handoff,
        "lead_deg": lead,
        "achieved_deg": round(achieved, 3),
        "signed_err_deg": round(err, 3),
        "overshoot_deg": round(overshoot, 3),
        "abs_err_deg": round(abs(err), 3),
        "total_sec": round(moving_sec, 2),
        "fast_sec": round(fast_sec, 2),
        "slow_sec": round(slow_sec, 2),
        "final_sec": round(final_sec, 2),
        "handoff2_deg": handoff2,
        "used_fast": int(fast_sec > 0.05),
        "handoff_yaw": round(handoff_yaw, 2) if handoff_yaw is not None else "",
        "timed_out": int(timed_out),
        "diverged": int(diverged),
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="Alvik3")
    ap.add_argument("--host", default="192.168.0.212")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--angles", default=None,
                    help="comma-separated target angles in degrees")
    ap.add_argument("--handoffs", default=None,
                    help="comma-separated handoff angles (deg remaining)")
    ap.add_argument("--fast", type=float, default=FAST_RPM)
    ap.add_argument("--slow", type=float, default=SLOW_RPM)
    ap.add_argument("--lead", type=float, default=LEAD_DEG,
                    help="deg remaining at which to stop. A consistent signed "
                         "error in the report is the correction to apply.")
    ap.add_argument("--final", type=float, default=0.0,
                    help="stage-3 RPM for the last few degrees. 6 RPM is the "
                         "tightest regime measured: 0.27deg of travel per "
                         "camera frame and coast sd 0.12-0.34deg, against "
                         "1.13deg/frame and sd 0.42-0.95 at 25 RPM. 0 "
                         "disables stage 3.")
    ap.add_argument("--handoff2", default=None,
                    help="comma-separated deg-remaining values at which to "
                         "drop from --slow to --final. This is the thing to "
                         "minimise: stage 3 is accurate but crawls at "
                         "~10.8 deg/s, so the smallest value that still holds "
                         "tolerance is the right one.")
    ap.add_argument("--control-hz", type=float, default=CONTROL_HZ,
                    help="wheel-command rate. Must stay well inside the "
                         "firmware's 300ms brake watchdog; 50 matches the "
                         "measured ~50Hz per-robot vision rate.")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    global LOG_PREFIX
    LOG_PREFIX = f"{args.robot:<7} "

    angles = ([float(a) for a in args.angles.split(",")]
              if args.angles else [float(a) for a in ANGLES_DEG])
    handoffs = ([float(h) for h in args.handoffs.split(",")]
                if args.handoffs else [float(h) for h in HANDOFF_DEG])
    handoff2s = ([float(h) for h in args.handoff2.split(",")]
                 if args.handoff2 else [0.0])

    out_path = args.out or os.path.join(
        ROOT, "runs_linux",
        f"two_stage_{args.robot}_{datetime.now():%m%d_%H%M}.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

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

    total = len(angles) * len(handoffs) * len(handoff2s) * args.repeats
    log(f"{len(angles)} angles x {len(handoffs)} handoffs x {args.repeats} "
        f"= {total} turns  (fast={args.fast:.0f} slow={args.slow:.0f} "
        f"lead={args.lead:.1f})")
    log("pure rotation throughout: right wheel is always -left")

    rows: list[dict] = []
    fields = ["target_deg", "handoff_deg", "lead_deg", "achieved_deg",
              "signed_err_deg", "overshoot_deg", "abs_err_deg",
              "total_sec", "fast_sec",
              "slow_sec", "final_sec", "handoff2_deg",
              "used_fast", "handoff_yaw", "timed_out",
              "diverged"]
    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        n = 0
        try:
            for h2 in handoff2s:
              for handoff in handoffs:
                for ang in angles:
                    for i in range(args.repeats):
                        signed = ang if i % 2 == 0 else -ang
                        n += 1
                        r = two_stage_turn(rb, signed, args.fast, args.slow,
                                           handoff, args.lead,
                                           final=args.final, handoff2=h2,
                                           control_hz=args.control_hz)
                        if r:
                            rows.append(r)
                            w.writerow(r)
                            fh.flush()
                            log(f"  [{n}/{total}] h={handoff:>3.0f}"
                                + (f"/{h2:>4.1f}" if h2 > args.lead else "")
                                + f" "
                                f"{signed:+6.1f}deg -> err {r['signed_err_deg']:+6.2f} "
                                f"in {r['total_sec']:>5.2f}s "
                                f"({r['fast_sec']:.2f} fast / {r['slow_sec']:.2f} slow)"
                                + ("  TIMEOUT" if r["timed_out"] else "")
                                + ("  DIVERGED" if r["diverged"] else ""))
                        time.sleep(0.3)
        except KeyboardInterrupt:
            log("interrupted -- rows already written are kept")
        finally:
            try:
                rb.spin(0.0)
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
    emit(f"{'h2':>8} {'angle':>7} {'n':>3} {'overshoot':>10} {'|err|':>7} "
         f"{'worst':>7} {'sec':>6} {'crawl s':>8}")
    grp = defaultdict(list)
    for r in rows:
        grp[(r.get("handoff2_deg", 0.0), abs(r["target_deg"]))].append(r)
    for (h, a) in sorted(grp):
        v = grp[(h, a)]
        se = [x["overshoot_deg"] for x in v]
        ae = [x["abs_err_deg"] for x in v]
        emit(f"{h:>8.0f} {a:>7.0f} {len(v):>3} {st.fmean(se):>+10.2f} "
             f"{st.fmean(ae):>7.2f} {max(ae):>7.2f} "
             f"{st.fmean([x['total_sec'] for x in v]):>6.2f} "
             f"{st.fmean([x.get('final_sec', 0.0) for x in v]):>8.2f}")

    emit()
    bias = st.fmean([r["overshoot_deg"] for r in rows])
    emit(f"  mean OVERSHOOT (direction of travel) {bias:+.2f}deg "
         f"over {len(rows)} turns")
    emit(f"  -> lead should be {args.lead:.1f} {'+' if bias >= 0 else '-'} "
         f"{abs(bias):.2f} = {args.lead + bias:.2f} deg")
    # Smallest angle at which stage 1 actually ran AND nothing overshot badly.
    ok = defaultdict(list)
    for r in rows:
        ok[(r["handoff_deg"], abs(r["target_deg"]))].append(r)
    emit()
    for h in sorted({r["handoff_deg"] for r in rows}):
        usable = [a for (hh, a) in sorted(ok)
                  if hh == h and any(x["used_fast"] for x in ok[(hh, a)])
                  and max(x["abs_err_deg"] for x in ok[(hh, a)]) <= 3.0]
        emit(f"  handoff {h:>3.0f}deg: stage 1 usable and accurate from "
             + (f"{min(usable):.0f}deg up" if usable else "no angle tested"))
    emit()
    emit(f"  wrote {out_path}")


if __name__ == "__main__":
    main()
