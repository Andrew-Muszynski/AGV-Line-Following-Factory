"""Signed terminal-yaw error: is there a systematic offset, or just spread?

terminal_heading_error_deg is |error|, so it cannot answer this. Recompute
the SIGNED error from run_robots.terminal_yaw_deg against the same target the
engine uses, and split by tuning era.
"""
import collections
import math
import os
import sqlite3
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from metrics_engine import REQUIRED_DEPOT_HEADING_DEG, wrap_deg  # noqa: E402

conn = sqlite3.connect(os.path.join(ROOT, "data", "agv_metrics.sqlite3"))
conn.row_factory = sqlite3.Row

print(f"target depot heading: {REQUIRED_DEPOT_HEADING_DEG} deg")

rows = [dict(r) for r in conn.execute("""
    SELECT rb.robot_id, rb.terminal_yaw_deg AS yaw,
           substr(r.created_at_utc, 1, 10) AS d, r.status
      FROM run_robots rb JOIN runs r ON r.run_id = rb.run_id
     WHERE rb.terminal_yaw_deg IS NOT NULL
     ORDER BY r.created_at_utc""")]

for x in rows:
    x["err"] = wrap_deg(x["yaw"] - REQUIRED_DEPOT_HEADING_DEG)


def era(d):
    if d < "2026-09-24":
        return "pre-staged"
    if d < "2026-09-26":
        return "taper-trial"
    return "current"


print(f"\n{len(rows)} signed observations\n")
print(f"{'robot':8s} {'n':>4s} {'signed mean':>12s} {'sd':>6s} {'se':>5s} "
      f"{'bias?':>8s} {'|err|>2':>8s}")
by = collections.defaultdict(list)
for x in rows:
    by[x["robot_id"]].append(x["err"])
for rb in sorted(by):
    v = by[rb]
    m = st.mean(v)
    sd = st.pstdev(v) if len(v) > 1 else 0.0
    se = sd / math.sqrt(len(v)) if len(v) > 1 else 0.0
    sig = "YES" if se > 0 and abs(m) > 2 * se else "no"
    over = 100 * sum(1 for y in v if abs(y) > 2.0) / len(v)
    print(f"{rb:8s} {len(v):>4d} {m:+12.2f} {sd:6.2f} {se:5.2f} "
          f"{sig:>8s} {over:7.0f}%")

allv = [x["err"] for x in rows]
m = st.mean(allv)
sd = st.pstdev(allv)
se = sd / math.sqrt(len(allv))
print(f"\nFLEET  n={len(allv)}  signed mean {m:+.2f}deg  sd {sd:.2f}  "
      f"se {se:.2f}")
print(f"       systematic offset? "
      f"{'YES — mean is ' + format(abs(m)/se, '.1f') + ' SE from zero' if abs(m) > 2*se else 'no'}")
print(f"       |error| > 2deg: {100*sum(1 for y in allv if abs(y)>2)/len(allv):.0f}%")

print("\nBY TUNING ERA")
print(f"{'era':14s} {'n':>4s} {'signed mean':>12s} {'sd':>6s} "
      f"{'|err|>2':>8s} {'p95|err|':>9s}")
eras = collections.defaultdict(list)
for x in rows:
    eras[era(x["d"])].append(x["err"])
for e in ("pre-staged", "taper-trial", "current"):
    v = eras.get(e) or []
    if not v:
        continue
    a = sorted(abs(y) for y in v)
    print(f"{e:14s} {len(v):>4d} {st.mean(v):+12.2f} {st.pstdev(v):6.2f} "
          f"{100*sum(1 for y in v if abs(y)>2)/len(v):7.0f}% "
          f"{a[min(len(a)-1,int(0.95*len(a)))]:8.2f}")

print("\nPER ROBOT, CURRENT ERA ONLY (what a recalibration would target)")
cur = collections.defaultdict(list)
for x in rows:
    if era(x["d"]) == "current":
        cur[x["robot_id"]].append(x["err"])
print(f"{'robot':8s} {'n':>4s} {'signed mean':>12s} {'sd':>6s} {'se':>5s} {'bias?':>7s}")
for rb in sorted(cur):
    v = cur[rb]
    if not v:
        continue
    m = st.mean(v)
    sd = st.pstdev(v) if len(v) > 1 else 0.0
    se = sd / math.sqrt(len(v)) if len(v) > 1 else 0.0
    sig = "YES" if se > 0 and abs(m) > 2 * se else "no"
    print(f"{rb:8s} {len(v):>4d} {m:+12.2f} {sd:6.2f} {se:5.2f} {sig:>7s}")
