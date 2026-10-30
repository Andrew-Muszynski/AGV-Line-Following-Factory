#!/usr/bin/env python3
"""SUPERSEDED 2026-10-12 -- see tools/make_experiment_gantt.py.

This chart was built against option B at full scope and landed the defense in
April 2027. The operator then fixed three constraints: all physical work done
before 1 December 2026, the thesis 80% drafted by that date, and nothing
running into 2027.

That is a scope decision, not a scheduling one, and the compressed plan lives
in tools/make_experiment_gantt.py. This script is kept only as the record of
what full-scope option B costs, which is the number the scope cuts are
measured against.

    python3 tools/make_thesis_gantt.py      # still runs; writes to _superseded
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
OUT = os.path.join(ROOT, "figures", "fig_thesis_schedule_superseded.png")

INK, MUTE = "#1b2733", "#5b6b7a"
BUILD = "#b3471f"      # engineering that blocks the experiment
EXPT = "#1f6f7a"       # running the experiment
WRITE = "#2f6f52"      # drafting
REVIEW = "#6b5ca5"     # advisor / committee

TODAY = dt.date(2026, 10, 12)
TARGET = dt.date(2026, 12, 15)       # the carried December defense target

D = dt.date
#  (lane, label, start, end, colour)
TASKS = [
    ("BUILD", "Pickup leg enabled - completes the RPD cycle", D(2026, 10, 12), D(2026, 10, 23), BUILD),
    ("BUILD", "Reliability for unattended runs (brake lead, guard resume,\nper-robot rotation, Alvik1 drivetrain)", D(2026, 10, 12), D(2026, 10, 30), BUILD),
    ("BUILD", "Joint instance representation + decoder extension", D(2026, 10, 19), D(2026, 11, 6), BUILD),
    ("BUILD", "Joint fitness: congestion surrogate + exact MAPF for elite", D(2026, 11, 2), D(2026, 11, 27), BUILD),
    ("BUILD", "State extractor: live supervisor state -> solver instance", D(2026, 11, 16), D(2026, 11, 27), BUILD),
    ("BUILD", "Pipeline validation + smoke campaign", D(2026, 11, 30), D(2026, 12, 11), BUILD),

    ("EXPT", "Simulation campaign - paired seeded instances", D(2026, 12, 14), D(2027, 1, 15), EXPT),
    ("EXPT", "Physical fidelity check - subset of conditions", D(2027, 1, 18), D(2027, 2, 5), EXPT),
    ("EXPT", "Analysis, statistics, result figures", D(2027, 2, 1), D(2027, 2, 12), EXPT),

    ("WRITE", "Ch4 Testbed + Ch5 Execution reliability  (data complete)", D(2026, 10, 12), D(2026, 11, 6), WRITE),
    ("WRITE", "Ch2 Related work  (July deck lit review -> prose)", D(2026, 11, 2), D(2026, 11, 20), WRITE),
    ("WRITE", "Ch3 Problem formulation", D(2026, 11, 16), D(2026, 12, 4), WRITE),
    ("WRITE", "Ch6 Methods: the two planning pipelines", D(2026, 11, 30), D(2026, 12, 18), WRITE),
    ("WRITE", "Ch7 Experimental design", D(2026, 12, 7), D(2026, 12, 18), WRITE),
    ("WRITE", "Appendix: VLM dispatch-advisor study", D(2026, 12, 14), D(2026, 12, 23), WRITE),
    ("WRITE", "Ch8 Results + Ch9 Sim-to-real gap", D(2027, 2, 8), D(2027, 2, 26), WRITE),
    ("WRITE", "Ch1 Introduction, Ch10 Discussion, Ch11 Conclusion", D(2027, 2, 22), D(2027, 3, 12), WRITE),
    ("WRITE", "Full revision pass", D(2027, 3, 8), D(2027, 3, 26), WRITE),

    ("REVIEW", "Advisor review - chapters as they land", D(2026, 11, 9), D(2027, 3, 26), REVIEW),
    ("REVIEW", "Committee draft circulated (3-4 week rule)", D(2027, 3, 29), D(2027, 4, 23), REVIEW),
]

MILESTONES = [
    ("Joint pipeline runs end to end", D(2026, 12, 11), BUILD),
    ("Simulation result in hand", D(2027, 1, 15), EXPT),
    ("Full draft to advisor", D(2027, 3, 26), WRITE),
    ("Defense", D(2027, 4, 23), REVIEW),
]


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig, ax = plt.subplots(figsize=(13.6, 8.6), dpi=150)
    fig.patch.set_facecolor("white")

    rows = list(range(len(TASKS)))
    for i, (_lane, label, start, end, colour) in enumerate(TASKS):
        ax.barh(i, (end - start).days, left=start, height=0.58,
                color=colour, alpha=0.92, zorder=3)
        weeks = (end - start).days / 7.0
        ax.text(end + dt.timedelta(days=2), i, f"{weeks:.0f}w",
                va="center", fontsize=8.5, color=MUTE)

    # Stagger the labels: the first two milestones are five weeks apart and
    # their captions are wider than that, so on one row they overlap.
    for i, (name, when, colour) in enumerate(MILESTONES):
        label_y = -1.80 if i % 2 == 0 else -2.35
        ax.plot([when], [-1.1], marker="D", markersize=9, color=colour,
                zorder=5, clip_on=False)
        ax.plot([when, when], [-1.1, label_y + 0.12], color=colour, lw=0.9,
                alpha=0.55, zorder=4, clip_on=False)
        ax.text(when, label_y, name, ha="center", va="bottom", fontsize=9,
                color=colour, fontweight="bold")

    ax.axvline(TODAY, color=INK, lw=1.6, zorder=4)
    ax.text(TODAY - dt.timedelta(days=2), len(TASKS) - 0.2, "today",
            ha="right", va="center", fontsize=9.5, color=INK, fontweight="bold")
    ax.axvline(TARGET, color="#c0392b", lw=1.6, ls="--", zorder=4)
    ax.text(TARGET + dt.timedelta(days=3), len(TASKS) - 0.2,
            "December target", ha="left", va="center", fontsize=9.5,
            color="#c0392b", fontweight="bold")

    ax.set_yticks(rows)
    ax.set_yticklabels([t[1] for t in TASKS], fontsize=9, color=INK)
    ax.invert_yaxis()
    ax.set_ylim(len(TASKS) - 0.4, -3.0)
    ax.set_xlim(D(2026, 10, 5), D(2027, 5, 10))
    ax.xaxis.set_major_locator(mdates.MonthLocator())
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%b %Y"))
    ax.tick_params(axis="x", colors=MUTE, labelsize=9.5)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color("#c8d0d8")
    ax.grid(axis="x", color="#e6eaee", lw=0.8, zorder=0)
    ax.set_axisbelow(True)

    fig.suptitle("SUPERSEDED - Full-Scope Option B (kept for comparison)",
                 fontsize=16.5, fontweight="bold", color=INK, x=0.012, ha="left", y=0.978)
    fig.text(0.012, 0.930,
             "Durations set from the work, not the target date. Critical path runs through the "
             "joint fitness evaluation, then the campaign it makes possible.",
             fontsize=10.3, color=MUTE)

    legend = [Patch(facecolor=BUILD, label="Engineering that blocks the experiment"),
              Patch(facecolor=EXPT, label="Running the experiment"),
              Patch(facecolor=WRITE, label="Drafting"),
              Patch(facecolor=REVIEW, label="Advisor and committee")]
    fig.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.009, 0.912),
               frameon=False, fontsize=9.8, ncol=4, handlelength=1.3,
               columnspacing=1.6, handletextpad=0.55)

    fig.subplots_adjust(top=0.845, bottom=0.055, left=0.300, right=0.962)
    fig.savefig(OUT, dpi=150, facecolor="white")
    print(f"wrote {OUT}")
    for name, when, _c in MILESTONES:
        print(f"  {when:%Y-%m-%d}  {name}")
    slip = (MILESTONES[-1][1] - TARGET).days
    print(f"\n  defense lands {slip} days ({slip/7:.0f} weeks) past the December target")


if __name__ == "__main__":
    main()
