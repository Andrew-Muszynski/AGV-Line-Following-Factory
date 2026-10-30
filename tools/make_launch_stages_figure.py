#!/usr/bin/env python3
"""figures/fig_launch_stages.png -- the depot launch split into its two stages.

WHY THIS FIGURE REPLACES THE OLD ONE
------------------------------------
The earlier chart plotted one bar per robot, "seconds to reach node 0", and
invited the reading that fixed offsets are slow to DRIVE. They are not. Both
schemes drive at the same speed. What changed is when a robot is allowed to
start, so a single bar hides the whole mechanism.

    stage 1   parked in its depot slot, not yet moving
    stage 2   moving: depot slot -> DE_i -> down the lane -> node 0

Under fixed offsets stage 1 dominates: Alvik6 sits still for 106.5 s and then
drives for 13.5 s. Under the cascade every robot is released at once, so
stage 1 collapses to ~1.7 s for all six and the queueing moves into stage 2,
where the robots are at least making progress down the lane.

HOW THE NUMBERS ARE MEASURED
----------------------------
arrival at node 0
    The LAUNCH_COMPLETED event, which the supervisor raises from the live
    vision pose the first time a robot comes within LaunchTimer.ARRIVE_RADIUS
    (0.45 grid units = 4.5 in) of node 0 at (13.3, 2.9) in.

departure from the depot slot
    Recovered here from telemetry_samples: the first time the robot is more
    than 3.0 in from its resting position, requiring 3 qualifying samples
    inside a 2 s forward window rather than 3 consecutive ones.

    The window matters. In run 7c349511 the vision stream interleaves
    Alvik1's true track with two stale fixes pinned at D1, so a
    consecutive-sample rule reports departure at 8.04 s when the robot in
    fact left at 1.73 s and was at node 0 by 5.89 s. The density rule reads
    through that; a consecutive rule does not.

TWO INDEPENDENT CHECKS, both run by --verify
    1. Telemetry-derived arrival vs the logged LAUNCH_COMPLETED agree to
       within 0.20 s on all 12 robot-runs.
    2. Measured departures under fixed offsets sit 1.5-2.0 s after the
       launch_offset_sec values stored in r4's own plan
       (0, 23.19, 37.89, 60.18, 80.37, 104.96) -- that gap is dispatch
       latency, and it is consistent across robots.

    python3 tools/make_launch_stages_figure.py [--verify]
"""
from __future__ import annotations

import argparse
import os
import sqlite3
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.patches import Patch                  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DB = os.path.join(ROOT, "data", "agv_metrics.sqlite3")
FIGS = os.path.join(ROOT, "figures")
OUT = os.path.join(FIGS, "fig_launch_stages.png")

M2IN = 39.3701
NODE0_IN = (13.3, 2.9)          # camera_grid_navigate.node_to_world("0", 8, 8)

#: the run already on the slide: 09-15 r4, fixed launch offsets, completed.
OFFSETS_RUN = "b7ebe53f"
#: 10-06 cascade, completed in 212.0 s = 3:32, the fastest on record.
CASCADE_RUN = "7c349511"
#: launch_offset_sec as stored in r4's own plan, for the cross-check.
R4_PLAN_OFFSETS = [0.0, 23.19, 37.89, 60.18, 80.37, 104.96]

INK, MUTE = "#1b2733", "#5b6b7a"
RUST, RUST_PALE = "#b3471f", "#eccfc4"
GREEN, GREEN_PALE = "#2f6f52", "#cfe3d8"

MOVE_IN, ARRIVE_IN, WIN, NEED = 3.0, 4.5, 2.0, 3


def _run_id(con, prefix):
    row = con.execute("select run_id from runs where run_id like ?",
                      (prefix + "%",)).fetchone()
    if row is None:
        sys.exit(f"no run matching {prefix!r} in {DB}")
    return row[0]


def stages(con, run_id):
    """-> {robot: (depart_s, arrive_telemetry_s, arrive_event_s)}."""
    events = dict(con.execute(
        "select robot_id, elapsed_ms/1000.0 from events "
        "where run_id=? and event_type='LAUNCH_COMPLETED'", (run_id,)))
    out = {}
    for (robot,) in con.execute(
            "select distinct robot_id from telemetry_samples "
            "where run_id=? order by 1", (run_id,)):
        rows = [(ms / 1000.0, x * M2IN, y * M2IN) for ms, x, y in con.execute(
            "select elapsed_ms, x_m, y_m from telemetry_samples "
            "where run_id=? and robot_id=? and valid=1 order by elapsed_ms",
            (run_id, robot))]
        if len(rows) < 20:
            continue
        base = [p for p in rows if p[0] <= 1.0] or rows[:5]
        bx = statistics.median(p[1] for p in base)
        by = statistics.median(p[2] for p in base)

        def first(pred):
            for i, p in enumerate(rows):
                if not pred(p):
                    continue
                j, n = i, 0
                while j < len(rows) and rows[j][0] <= p[0] + WIN:
                    n += bool(pred(rows[j]))
                    j += 1
                if n >= NEED:
                    return p[0]
            return None

        depart = first(lambda p: ((p[1] - bx) ** 2 + (p[2] - by) ** 2) ** 0.5 > MOVE_IN)
        arrive = first(lambda p: ((p[1] - NODE0_IN[0]) ** 2
                                  + (p[2] - NODE0_IN[1]) ** 2) ** 0.5 < ARRIVE_IN)
        out[robot] = (depart, arrive, events.get(robot))
    return out


def verify(con):
    ok = True
    print("check 1 -- telemetry arrival vs logged LAUNCH_COMPLETED")
    worst = 0.0
    for tag in (OFFSETS_RUN, CASCADE_RUN):
        for robot, (_d, a, e) in sorted(stages(con, _run_id(con, tag)).items()):
            if a is None or e is None:
                print(f"  {tag} {robot}: MISSING"); ok = False; continue
            worst = max(worst, abs(a - e))
    print(f"  worst disagreement across 12 robot-runs: {worst:.2f} s"
          f"  -> {'OK' if worst <= 0.25 else 'FAIL'}")
    ok &= worst <= 0.25

    print("check 2 -- measured departure vs r4's stored launch_offset_sec")
    st = stages(con, _run_id(con, OFFSETS_RUN))
    measured = sorted(d for d, _a, _e in st.values() if d is not None)
    lags = [m - p for m, p in zip(measured, sorted(R4_PLAN_OFFSETS))]
    for p, m, lag in zip(sorted(R4_PLAN_OFFSETS), measured, lags):
        print(f"  plan {p:6.2f} s -> measured {m:6.2f} s   lag {lag:4.2f} s")
    spread = max(lags) - min(lags)
    print(f"  dispatch lag {min(lags):.2f}-{max(lags):.2f} s, spread {spread:.2f} s"
          f"  -> {'OK' if spread < 1.5 else 'FAIL'}")
    return ok and spread < 1.5


def arm(ax, data, shift, pale, solid, height):
    """One scheme's bars, placed by POSITION in the launch order.

    Row i is the i-th robot out of the depot lane under that scheme, which is
    the only like-for-like comparison: the two schemes release in different
    orders, so pairing on robot identity compares a robot that went first
    against one that went fourth.
    """
    order = sorted(data, key=lambda r: data[r][2])
    for i, robot in enumerate(order):
        depart, _tel, arrive = data[robot]
        y = i + shift
        ax.barh(y, depart, color=pale, edgecolor=solid, linewidth=0.7, height=height)
        ax.barh(y, arrive - depart, left=depart, color=solid, height=height)
        if depart > 9:
            ax.text(depart / 2, y, f"{depart:.0f}s", ha="center", va="center",
                    fontsize=8.5, color=solid, fontweight="bold")
        ax.text(arrive + 1.6, y, f"{arrive:.1f}  {robot}", va="center",
                fontsize=9.5, color=solid, fontweight="bold")
    return order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true",
                    help="run both measurement checks and exit")
    args = ap.parse_args()

    if not os.path.exists(DB):
        sys.exit(f"metrics database not found: {DB}")
    con = sqlite3.connect(DB)

    if args.verify:
        sys.exit(0 if verify(con) else 1)

    off = stages(con, _run_id(con, OFFSETS_RUN))
    cas = stages(con, _run_id(con, CASCADE_RUN))
    for name, d in (("offsets", off), ("cascade", cas)):
        missing = [r for r, v in d.items() if None in v]
        if missing:
            sys.exit(f"{name}: incomplete stage data for {missing}")

    idle_off = sum(v[0] for v in off.values())
    idle_cas = sum(v[0] for v in cas.values())
    last_off = max(v[2] for v in off.values())
    last_cas = max(v[2] for v in cas.values())
    os.makedirs(FIGS, exist_ok=True)
    fig, ax = plt.subplots(figsize=(11.2, 6.0), dpi=150)
    fig.patch.set_facecolor("white")

    H, OFFSET = 0.33, 0.185
    order_off = arm(ax, off, -OFFSET, RUST_PALE, RUST, H)
    order_cas = arm(ax, cas, +OFFSET, GREEN_PALE, GREEN, H)

    for x, color, label in ((last_off, RUST, f"sequential {last_off:.0f} s"),
                            (last_cas, GREEN, f"cascade {last_cas:.0f} s")):
        ax.axvline(x, color=color, ls="--", lw=1.2, alpha=0.7, zorder=0)
        ax.text(x, -0.92, label, ha="center", va="bottom", fontsize=9.5,
                color=color, fontweight="bold")

    ax.set_yticks(range(len(order_off)))
    ax.set_yticklabels(["1st out", "2nd out", "3rd out", "4th out",
                        "5th out", "6th out"][:len(order_off)],
                       fontsize=11, color=INK)
    ax.invert_yaxis()
    ax.set_xlim(0, 150)
    ax.set_ylim(len(order_off) - 0.45, -1.05)
    ax.set_xlabel("seconds from run start", fontsize=11.5, color=INK)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color("#c8d0d8")
    ax.tick_params(axis="x", colors=MUTE, labelsize=10)
    ax.grid(axis="x", color="#e6eaee", lw=0.8, zorder=0)
    ax.set_axisbelow(True)

    fig.suptitle("Comparing Sequential vs Cascade Launch",
                 fontsize=17, fontweight="bold", color=INK, x=0.058, ha="left", y=0.975)
    fig.text(0.058, 0.892,
             "Time for the Nth robot out of the depot lane to reach node 0.      "
             "Sequential: r4, 09-15.      Cascade: the 3:32 run, 10-06.",
             fontsize=10.4, color=MUTE)

    legend = [Patch(facecolor=RUST_PALE, edgecolor=RUST, label="Sequential  parked"),
              Patch(facecolor=RUST, label="Sequential  moving"),
              Patch(facecolor=GREEN_PALE, edgecolor=GREEN, label="Cascade  parked"),
              Patch(facecolor=GREEN, label="Cascade  moving")]
    fig.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.055, 0.878),
               frameon=False, fontsize=10, ncol=4, handlelength=1.3,
               columnspacing=1.5, handletextpad=0.55)

    fig.subplots_adjust(top=0.805, bottom=0.115, left=0.075, right=0.975)
    fig.savefig(OUT, dpi=150, facecolor="white")
    print(f"wrote {OUT}")
    print(f"  sequential: last {last_off:.2f}s, fleet idle {idle_off:.1f} robot-s")
    print(f"  cascade:    last {last_cas:.2f}s, fleet idle {idle_cas:.1f} robot-s")
    print("  by launch position (sequential -> cascade):")
    for i, (ro, rc) in enumerate(zip(order_off, order_cas), start=1):
        a, b = off[ro][2], cas[rc][2]
        print(f"    {i} out   {a:6.1f} ({ro})  ->  {b:6.1f} ({rc})   {b-a:+6.1f}s")


if __name__ == "__main__":
    main()
