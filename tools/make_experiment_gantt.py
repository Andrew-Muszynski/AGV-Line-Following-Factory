#!/usr/bin/env python3
"""figures/fig_experiment_schedule.png -- the core thesis schedule.

REVISED 2026-10-14: 25 items -> 11. The previous chart scheduled everything
the testbed could produce. This one schedules only what the APPROVED PROPOSAL
asks for, which is a much smaller thing:

    "does jointly optimizing scheduling and routing outperform sequential
     optimization in constrained multi-AGV pickup-and-delivery on
     fixed-topology networks?"  ... and does it survive physical execution?

Everything on this chart is on the path to answering that. Everything that is
not on that path was removed rather than shortened. What came off:

  * THE VLM DISPATCH-ADVISOR STUDY -> future work. It was Appendix A and a
    drafting slot. It is a finished, self-contained result that answers a
    different question, and the proposal does not mention it.
  * THE MAPF METHOD COMPARISON (WAIT vs PP, DOE_MAPF_COMPARISON_2026-10-13)
    -> held as the CONTINGENCY, not a parallel workstream. If the 13 Nov gate
    fails, that study is what the physical window runs instead. The design
    document stands; it is simply not on the critical path.
  * K = 1 AS A COMPARISON CELL. One robot has no inter-robot conflicts, so
    joint and sequential are the same algorithm there. K = 1 stays as
    travel-time calibration, which existing data already covers.
  * SEPARATE BARS for work that is one sitting: the rig fixes, the sim pilot
    and campaign, the metrics and the figures were each one task split three
    ways on the old chart.

OPERATOR CONSTRAINTS, unchanged and asserted at the bottom of this file:
    * all physical work finished before 1 December 2026
    * thesis >= 80% drafted by 1 December 2026
    * nothing continues into 2027

    python3 tools/make_experiment_gantt.py
"""
from __future__ import annotations

import datetime as dt
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                        # noqa: E402
import matplotlib.dates as mdates                      # noqa: E402
from matplotlib.patches import Patch                   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "figures", "fig_experiment_schedule.png")

INK, MUTE = "#1b2733", "#5b6b7a"
RIG = "#b3471f"
BUILD = "#6b5ca5"
PHYS = "#2f6f52"
SIM = "#1f6f7a"
ANAL = "#1b2733"
WRITE = "#9c4f82"

TODAY = dt.date(2026, 10, 14)
PHYS_DEADLINE = dt.date(2026, 12, 1)
D = dt.date

#: Eleven items. Each one is a prerequisite for answering the proposal's
#: primary question or for writing it up. Nothing here is optional.
TASKS = [
    ("R1  Rig: guard-stop resume + brake lead", D(2026, 10, 14), D(2026, 10, 23), RIG),
    ("R2  Pickup leg end to end - a visit is drop AND pick", D(2026, 10, 14), D(2026, 10, 30), RIG),

    ("B1  Joint pipeline: decoder + congestion fitness", D(2026, 10, 14), D(2026, 11, 6), BUILD),
    ("B2  Validation fixtures + smoke campaign", D(2026, 11, 2), D(2026, 11, 13), BUILD),

    ("H1  Physical: SEQUENTIAL arm, K2-3 x J3,6", D(2026, 11, 2), D(2026, 11, 13), PHYS),
    ("H2  Physical: JOINT arm - contingent on 13 Nov", D(2026, 11, 16), D(2026, 11, 27), PHYS),

    ("S1  Paired simulation campaign - primary evidence", D(2026, 11, 16), D(2026, 12, 4), SIM),

    ("A1  Makespan tests + result figures", D(2026, 11, 30), D(2026, 12, 9), ANAL),

    ("W1  Ch3-5  Formulation, testbed, reliability", D(2026, 10, 14), D(2026, 11, 13), WRITE),
    ("W2  Ch1,2,6,7  Intro, related work, methods, design", D(2026, 11, 2), D(2026, 12, 1), WRITE),
    ("W3  Ch8-11  Results, sim-to-real, discussion", D(2026, 12, 1), D(2026, 12, 11), WRITE),
]

#: row index after which a hairline separates one workstream from the next
GROUP_BREAKS = (1.5, 3.5, 5.5, 6.5, 7.5)

GATES = [
    ("Rig ready", D(2026, 10, 30), RIG),
    ("GO / NO-GO on the joint arm", D(2026, 11, 13), BUILD),
    ("PHYSICAL WORK COMPLETE", D(2026, 11, 27), PHYS),
    ("Thesis 80% drafted", D(2026, 12, 1), WRITE),
    ("DRAFT COMPLETE", D(2026, 12, 11), ANAL),
]

#: the defense is shown as a window, not a date -- it is scheduled with the
#: committee, not planned here. All work finishes before it opens.
DEFENSE = (D(2026, 12, 14), D(2026, 12, 31))


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig, ax = plt.subplots(figsize=(13.4, 6.4), dpi=150)
    fig.patch.set_facecolor("white")

    for i, (label, start, end, colour) in enumerate(TASKS):
        ax.barh(i, (end - start).days, left=start, height=0.52,
                color=colour, alpha=0.92, zorder=3)
        days = (end - start).days
        span = f"{days}d" if days < 5 else f"{days/7.0:.0f}w"
        ax.text(end + dt.timedelta(days=1.5), i, span,
                va="center", fontsize=8.6, color=MUTE)

    for b in GROUP_BREAKS:
        ax.axhline(b, color="#dfe5ea", lw=1.0, zorder=1)

    ax.axvspan(PHYS_DEADLINE, D(2026, 12, 31), color="#f3e6e4", alpha=0.55, zorder=0)
    ax.axvspan(DEFENSE[0], DEFENSE[1], color="#dfd7ec", alpha=0.75, zorder=0)
    ax.text(PHYS_DEADLINE + dt.timedelta(days=1.5), 0.55,
            "no physical work past 1 Dec", fontsize=9.3, color="#b3471f",
            fontweight="bold", va="center")
    mid = DEFENSE[0] + (DEFENSE[1] - DEFENSE[0]) / 2
    NL = chr(10)
    ax.text(mid, 3.1, "DEFENSE" + NL + "WINDOW", ha="center", va="center",
            fontsize=10.5, color="#5a4a8a", fontweight="bold", linespacing=1.35)
    ax.text(mid, 4.9, NL.join(("mid-to-late Dec", "date set with", "the committee")),
            ha="center", va="center", fontsize=8.6, color="#6b5ca5",
            linespacing=1.35)

    for i, (name, when, colour) in enumerate(GATES):
        y = (-1.55, -2.05, -2.55)[i % 3]
        ax.plot([when], [-1.0], marker="D", markersize=8.0, color=colour,
                zorder=5, clip_on=False)
        ax.plot([when, when], [-1.0, y + 0.10], color=colour, lw=0.9,
                alpha=0.55, zorder=4, clip_on=False)
        ax.text(when, y, name, ha="center", va="bottom", fontsize=8.7,
                color=colour, fontweight="bold")

    ax.axvline(TODAY, color=INK, lw=1.6, zorder=4)
    ax.text(TODAY - dt.timedelta(days=1.5), len(TASKS) - 0.3, "today",
            ha="right", va="center", fontsize=9.5, color=INK, fontweight="bold")

    ax.set_yticks(range(len(TASKS)))
    ax.set_yticklabels([t[0] for t in TASKS], fontsize=9.0, color=INK)
    ax.invert_yaxis()
    ax.set_ylim(len(TASKS) - 0.4, -3.1)
    ax.set_xlim(D(2026, 10, 5), D(2026, 12, 31))
    ax.xaxis.set_major_locator(mdates.WeekdayLocator(byweekday=mdates.MO, interval=1))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%d " + "%b"))
    ax.tick_params(axis="x", colors=MUTE, labelsize=9)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color("#c8d0d8")
    ax.grid(axis="x", color="#e6eaee", lw=0.8, zorder=0)
    ax.set_axisbelow(True)

    fig.suptitle("What Is Left: Joint vs Sequential, to 31 December 2026",
                 fontsize=16.4, fontweight="bold", color=INK, x=0.012, ha="left", y=0.972)
    fig.text(0.012, 0.912,
             "Eleven items, all on the path to the proposal's primary question. The VLM study and the "
             "MAPF method comparison are off the critical path; K=1 is calibration, not a comparison cell.",
             fontsize=10.1, color=MUTE)

    legend = [Patch(facecolor=RIG, label="Rig"), Patch(facecolor=BUILD, label="Joint arm"),
              Patch(facecolor=PHYS, label="Physical"), Patch(facecolor=SIM, label="Simulation"),
              Patch(facecolor=ANAL, label="Analysis"), Patch(facecolor=WRITE, label="Drafting")]
    fig.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.009, 0.892),
               frameon=False, fontsize=9.5, ncol=6, handlelength=1.2,
               columnspacing=1.2, handletextpad=0.5)

    fig.subplots_adjust(top=0.805, bottom=0.068, left=0.300, right=0.968)
    fig.savefig(OUT, dpi=150, facecolor="white")

    print(f"wrote {OUT}   ({len(TASKS)} items)")
    for name, when, _c in GATES:
        print(f"  {when:%Y-%m-%d}  ({when:%a})  {name}")
    last = max(t[2] for t in TASKS)
    phys = max(t[2] for t in TASKS if t[3] == PHYS)
    print(f"\n  last physical day : {phys:%Y-%m-%d}  (deadline {PHYS_DEADLINE:%Y-%m-%d})")
    print(f"  last task ends    : {last:%Y-%m-%d}")
    assert len(TASKS) < 13, "the point of this revision is fewer than half of 25 items"
    assert phys < PHYS_DEADLINE, "physical work must finish before 1 Dec"
    assert last.year == 2026, "nothing may run into 2027"
    assert last < DEFENSE[0], "every task must finish before the defense window opens"
    print("  constraints: OK")

    # Physical cost, priced off the proposal's own subset rather than the
    # six-robot runs. 12.6 min/attempt is measured on SIX robots; small
    # fleets are quicker to stage and quicker to recover, so 8 min is used
    # for K <= 3 and the six-robot rate is shown beside it as the pessimist.
    cells = 2 * 2          # K in {2,3} x |J| in {3,6}
    arms = 2               # sequential, joint
    print(f"\n  physical subset: {cells} cells x {arms} arms")
    for reps in (10, 5):
        done = cells * arms * reps
        print(f"    {reps:2d} reps -> {done:3d} completed runs")
        for rate, yld in ((8.0, 0.65), (12.6, 0.46)):
            att = done / yld
            print(f"        at {yld:.0%} yield, {rate:.1f} min/attempt"
                  f" -> {att:4.0f} attempts, {att*rate/60:5.1f} lab hours")


if __name__ == "__main__":
    main()
