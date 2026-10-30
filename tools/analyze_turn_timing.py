"""Dead time AFTER a rotation, before the robot drives again.

The 2026-09-24 findings note the old method's IMU completion gate could not
confirm large turns (BRAKED_UNCONFIRMED) and said "the cost is time, not
correctness". If that stall is real it lands here, not in the rotation
itself -- so the rotation-only comparison would understate the old method's
cost and the accuracy/speed trade might still hold.

Measures, per rotation episode: the interval from the last rotating sample
to the first subsequent sample with real translation.
"""
import collections
import math
import os
import sqlite3
import statistics as st

YAW_RATE_MIN = 15.0
TRANS_MAX = 0.03
TRANS_MOVING = 0.05      # m/s that counts as driving again
GAP_MAX_MS = 400
MIN_SWEEP = 20.0
MIN_SAMPLES = 3
SETTLE_CAP_S = 20.0      # beyond this it is a traffic hold, not a settle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
conn = sqlite3.connect(os.path.join(ROOT, "data", "agv_metrics.sqlite3"))
conn.row_factory = sqlite3.Row


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def era(d):
    if d < "2026-09-24":
        return "pre-staged"
    if d < "2026-09-26":
        return "taper-trial"
    return "current"


runs = [dict(r) for r in conn.execute("""
    SELECT run_id, substr(created_at_utc,1,10) AS d
      FROM runs WHERE robot_count = 6 AND drive_mode = 'vision'
       AND status IN ('completed','aborted','failed')""")]

gaps = collections.defaultdict(list)
gaps_big = collections.defaultdict(list)

for r in runs:
    e = era(r["d"])
    bots = collections.defaultdict(list)
    for t in conn.execute("""SELECT robot_id, elapsed_ms, x_m, y_m, yaw_deg
                               FROM telemetry_samples
                              WHERE run_id = ? AND valid = 1
                              ORDER BY robot_id, elapsed_ms""", (r["run_id"],)):
        if t["yaw_deg"] is None:
            continue
        bots[t["robot_id"]].append(
            (t["elapsed_ms"], t["x_m"], t["y_m"], t["yaw_deg"]))

    for robot, pts in bots.items():
        # mark rotating transitions
        rot = [False] * len(pts)
        for i in range(1, len(pts)):
            t0, x0, y0, a0 = pts[i - 1]
            t1, x1, y1, a1 = pts[i]
            dt = (t1 - t0) / 1000.0
            if dt <= 0 or (t1 - t0) > GAP_MAX_MS:
                continue
            if (abs(wrap(a1 - a0)) / dt > YAW_RATE_MIN
                    and math.hypot(x1 - x0, y1 - y0) / dt < TRANS_MAX):
                rot[i] = True

        i = 1
        while i < len(pts):
            if not rot[i]:
                i += 1
                continue
            start = i
            sweep = 0.0
            while i < len(pts) and rot[i]:
                sweep += wrap(pts[i][3] - pts[i - 1][3])
                i += 1
            if (i - start) < MIN_SAMPLES or abs(sweep) < MIN_SWEEP:
                continue
            t_end = pts[i - 1][0]
            # walk forward to the first real translation
            j = i
            while j < len(pts):
                t0, x0, y0, _ = pts[j - 1]
                t1, x1, y1, _ = pts[j]
                dt = (t1 - t0) / 1000.0
                if dt > 0 and math.hypot(x1 - x0, y1 - y0) / dt > TRANS_MOVING:
                    break
                j += 1
            if j >= len(pts):
                continue
            gap = (pts[j][0] - t_end) / 1000.0
            if 0 <= gap <= SETTLE_CAP_S:
                gaps[e].append(gap)
                if abs(sweep) >= 120:
                    gaps_big[e].append(gap)

print("DEAD TIME BETWEEN FINISHING A ROTATION AND DRIVING AGAIN")
print("(capped at %.0f s; longer is a traffic hold, not a settle)\n" % SETTLE_CAP_S)
print(f"{'era':14s} {'n':>6s} {'mean s':>7s} {'median':>7s} {'p90':>7s} {'p99':>7s}")
for e in ("pre-staged", "taper-trial", "current"):
    v = sorted(gaps.get(e) or [])
    if not v:
        continue
    print(f"{e:14s} {len(v):>6d} {st.mean(v):7.2f} {st.median(v):7.2f} "
          f"{v[int(0.90*len(v))]:7.2f} {v[int(0.99*len(v))]:7.2f}")

print("\nAFTER LARGE TURNS (>=120 deg) — where BRAKED_UNCONFIRMED was reported")
print(f"{'era':14s} {'n':>6s} {'mean s':>7s} {'median':>7s} {'p90':>7s}")
for e in ("pre-staged", "taper-trial", "current"):
    v = sorted(gaps_big.get(e) or [])
    if not v:
        continue
    print(f"{e:14s} {len(v):>6d} {st.mean(v):7.2f} {st.median(v):7.2f} "
          f"{v[int(0.90*len(v))]:7.2f}")

print("\nTOTAL turn + settle, seconds per 90 deg equivalent")
print("  (rotation time from the previous analysis, plus the median gap)")
for e, rot_per90 in (("pre-staged", 0.99), ("taper-trial", 1.10),
                     ("current", 1.08)):
    v = gaps.get(e) or []
    if not v:
        continue
    print(f"  {e:14s} rotation {rot_per90:4.2f}s + settle "
          f"{st.median(v):4.2f}s = {rot_per90 + st.median(v):4.2f}s")
