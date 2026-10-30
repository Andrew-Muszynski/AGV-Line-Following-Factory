#!/usr/bin/env python3
"""Two focused schedules, split out of fig_experiment_schedule.png.

    figures/fig_drafting_schedule.png   -- writing the thesis, chapter by chapter
    figures/fig_physical_schedule.png   -- rig, campaign, simulation, analysis

The combined chart (tools/make_experiment_gantt.py) stays as the one-page
overview and is still what PROGRESS_REPORT_2026-10 embeds. These two are for
working from: each shows only one kind of day, so neither is read past.

WHY EACH CHART ENCODES WHAT IT DOES

  Drafting -- bars are coloured by WHAT GATES THE CHAPTER, not by chapter
      number. Six chapters can be written today against recorded data; one
      waits on the joint pipeline; four wait on results. That distinction is
      the only thing a drafting schedule is really for, so it is the colour.

  Physical -- bars are annotated with ESTIMATED LAB HOURS, not just duration.
      A four-week bar that costs three hours at the table and a two-week bar
      that costs eight are not the same object, and a plain Gantt hides it.
      Desk and unattended work is marked as such and costs no lab time.

Both share an x-axis (5 Oct - 31 Dec) so they can sit side by side.

    python3 tools/make_schedule_figures.py
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
FIGS = os.path.join(ROOT, "figures")

INK, MUTE = "#1b2733", "#5b6b7a"
D = dt.date
TODAY = D(2026, 10, 14)
PHYS_DEADLINE = D(2026, 12, 1)
DEFENSE = (D(2026, 12, 14), D(2026, 12, 31))
XLIM = (D(2026, 10, 5), D(2026, 12, 31))
NL = chr(10)

# ---------------------------------------------------------------- drafting

#: what each chapter is waiting on -- this is the colour axis
READY = "#2f6f52"       # write it today, the material exists
NEEDBUILD = "#8a6d1f"   # waits on the joint pipeline (B1, done 6 Nov)
NEEDDATA = "#5b6b7a"    # waits on the analysis (A1, done 9 Dec)

#: Ch3 is deliberately first. The formulation fixes the instance
#: representation, and the decoder in B1 is built against it -- writing it
#: after the solver means writing down whatever the solver happened to do.
DRAFT_TASKS = [
    ("Ch3   Problem formulation", D(2026, 10, 14), D(2026, 10, 23), READY),
    ("Ch4   The testbed as built", D(2026, 10, 19), D(2026, 11, 2), READY),
    ("Ch5   Execution reliability", D(2026, 10, 26), D(2026, 11, 13), READY),
    ("Ch2   Background and related work", D(2026, 11, 2), D(2026, 11, 16), READY),
    ("Ch7   Experimental design", D(2026, 11, 9), D(2026, 11, 20), READY),
    ("Ch6   Methods: the two pipelines", D(2026, 11, 9), D(2026, 11, 27), NEEDBUILD),
    ("App B  Measured constants", D(2026, 11, 16), D(2026, 11, 20), READY),
    ("Ch1   Introduction", D(2026, 11, 16), D(2026, 12, 1), READY),
    ("Ch8   Results: simulation", D(2026, 12, 1), D(2026, 12, 9), NEEDDATA),
    ("Ch9   Results: physical and sim-to-real", D(2026, 12, 1), D(2026, 12, 9), NEEDDATA),
    ("Ch10  Discussion", D(2026, 12, 4), D(2026, 12, 11), NEEDDATA),
    ("Ch11  Conclusion and future work", D(2026, 12, 7), D(2026, 12, 11), NEEDDATA),
]
DRAFT_BREAKS = (7.5,)
DRAFT_GATES = [
    ("Thesis 80% drafted", D(2026, 12, 1), READY),
    ("DRAFT COMPLETE", D(2026, 12, 11), INK),
]

# ---------------------------------------------------------------- physical

RIG = "#b3471f"
BUILD = "#6b5ca5"
PHYS = "#2f6f52"
SIM = "#1f6f7a"
ANAL = "#1b2733"

#: (label, start, end, colour, lab hours or None for desk/unattended)
PHYS_TASKS = [
    ("R1  Guard-stopped robots resume", D(2026, 10, 14), D(2026, 10, 21), RIG, 2.0),
    ("R2  Emergency brake lead", D(2026, 10, 19), D(2026, 10, 23), RIG, 3.0),
    ("R3  Pickup leg end to end", D(2026, 10, 14), D(2026, 10, 30), RIG, 5.0),
    ("V1  Shakedown + yield check, 10 runs", D(2026, 10, 26), D(2026, 10, 30), RIG, 3.0),

    ("B1  Joint pipeline: decoder + fitness", D(2026, 10, 14), D(2026, 11, 6), BUILD, None),
    ("B2  Validation fixtures + smoke", D(2026, 11, 2), D(2026, 11, 13), BUILD, None),

    ("H1  SEQUENTIAL arm, K2-3 x J3,6, 10 reps", D(2026, 11, 2), D(2026, 11, 13), PHYS, 8.2),
    ("H2  JOINT arm - contingent on 13 Nov", D(2026, 11, 16), D(2026, 11, 27), PHYS, 8.2),

    ("S1  Paired simulation campaign", D(2026, 11, 16), D(2026, 12, 4), SIM, None),

    ("A1  Makespan tests + result figures", D(2026, 11, 30), D(2026, 12, 9), ANAL, None),
]
PHYS_BREAKS = (3.5, 5.5, 7.5, 8.5)
PHYS_GATES = [
    ("Rig ready", D(2026, 10, 30), RIG),
    ("GO / NO-GO on the joint arm", D(2026, 11, 13), BUILD),
    ("PHYSICAL WORK COMPLETE", D(2026, 11, 27), PHYS),
]


def _frame(ax, n_rows, gate_floor):
    """Shared axis treatment, so the two figures read as a pair."""
    ax.axvspan(DEFENSE[0], DEFENSE[1], color="#dfd7ec", alpha=0.75, zorder=0)
    ax.axvline(TODAY, color=INK, lw=1.6, zorder=4)
    ax.text(TODAY - dt.timedelta(days=1.5), n_rows - 0.3, "today",
            ha="right", va="center", fontsize=9.5, color=INK, fontweight="bold")

    ax.invert_yaxis()
    ax.set_ylim(n_rows - 0.4, gate_floor)
    ax.set_xlim(*XLIM)
    ax.xaxis.set_major_locator(mdates.WeekdayLocator(byweekday=mdates.MO, interval=1))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%d " + "%b"))
    ax.tick_params(axis="x", colors=MUTE, labelsize=9)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color("#c8d0d8")
    ax.grid(axis="x", color="#e6eaee", lw=0.8, zorder=0)
    ax.set_axisbelow(True)


def _gates(ax, gates, levels):
    for i, (name, when, colour) in enumerate(gates):
        y = levels[i % len(levels)]
        ax.plot([when], [-1.0], marker="D", markersize=8.0, color=colour,
                zorder=5, clip_on=False)
        ax.plot([when, when], [-1.0, y + 0.10], color=colour, lw=0.9,
                alpha=0.55, zorder=4, clip_on=False)
        ax.text(when, y, name, ha="center", va="bottom", fontsize=8.7,
                color=colour, fontweight="bold")


def drafting():
    out = os.path.join(FIGS, "fig_drafting_schedule.png")
    fig, ax = plt.subplots(figsize=(13.4, 6.6), dpi=150)
    fig.patch.set_facecolor("white")

    for i, (label, start, end, colour) in enumerate(DRAFT_TASKS):
        ax.barh(i, (end - start).days, left=start, height=0.52,
                color=colour, alpha=0.92, zorder=3)
        days = (end - start).days
        span = f"{days}d" if days < 5 else f"{days/7.0:.0f}w"
        ax.text(end + dt.timedelta(days=1.5), i, span,
                va="center", fontsize=8.6, color=MUTE)

    for b in DRAFT_BREAKS:
        ax.axhline(b, color="#dfe5ea", lw=1.0, zorder=1)

    ax.axvspan(PHYS_DEADLINE, D(2026, 12, 31), color="#f6f1f4", alpha=0.75, zorder=0)
    mid = DEFENSE[0] + (DEFENSE[1] - DEFENSE[0]) / 2
    ax.text(mid, 3.2, "DEFENSE" + NL + "WINDOW", ha="center", va="center",
            fontsize=10.5, color="#5a4a8a", fontweight="bold", linespacing=1.35)
    ax.text(mid, 5.1, NL.join(("mid-to-late Dec", "date set with", "the committee")),
            ha="center", va="center", fontsize=8.6, color="#6b5ca5", linespacing=1.35)

    _gates(ax, DRAFT_GATES, (-1.55, -2.05))
    ax.set_yticks(range(len(DRAFT_TASKS)))
    ax.set_yticklabels([t[0] for t in DRAFT_TASKS], fontsize=9.0, color=INK)
    _frame(ax, len(DRAFT_TASKS), -2.7)

    ready = sum(1 for t in DRAFT_TASKS if t[3] == READY)
    fig.suptitle("Drafting the Thesis, Chapter by Chapter",
                 fontsize=16.4, fontweight="bold", color=INK, x=0.012, ha="left", y=0.972)
    fig.text(0.012, 0.912,
             f"{ready} of {len(DRAFT_TASKS)} chapters can be written today against recorded data. "
             "Ch3 comes first because the formulation fixes the instance representation the decoder is built against.",
             fontsize=10.1, color=MUTE)
    fig.legend(handles=[Patch(facecolor=READY, label="Ready now - material exists"),
                        Patch(facecolor=NEEDBUILD, label="Waits on the joint pipeline"),
                        Patch(facecolor=NEEDDATA, label="Waits on the analysis")],
               loc="upper left", bbox_to_anchor=(0.009, 0.892), frameon=False,
               fontsize=9.5, ncol=3, handlelength=1.2, columnspacing=1.2,
               handletextpad=0.5)
    fig.subplots_adjust(top=0.812, bottom=0.066, left=0.300, right=0.968)
    fig.savefig(out, dpi=150, facecolor="white")
    return out, ready


def physical():
    out = os.path.join(FIGS, "fig_physical_schedule.png")
    fig, ax = plt.subplots(figsize=(13.4, 6.0), dpi=150)
    fig.patch.set_facecolor("white")

    for i, (label, start, end, colour, hours) in enumerate(PHYS_TASKS):
        ax.barh(i, (end - start).days, left=start, height=0.52,
                color=colour, alpha=0.92, zorder=3)
        days = (end - start).days
        span = f"{days}d" if days < 5 else f"{days/7.0:.0f}w"
        if hours is None:
            tail = "unattended" if colour == SIM else "desk"
            note, weight = f"{span}  -  {tail}", "normal"
        else:
            note, weight = f"{span}  -  ~{hours:.0f} lab h", "bold"
        ax.text(end + dt.timedelta(days=1.5), i, note, va="center",
                fontsize=8.6, color=MUTE, fontweight=weight)

    for b in PHYS_BREAKS:
        ax.axhline(b, color="#dfe5ea", lw=1.0, zorder=1)

    ax.axvspan(PHYS_DEADLINE, D(2026, 12, 31), color="#f3e6e4", alpha=0.55, zorder=0)
    ax.text(PHYS_DEADLINE + dt.timedelta(days=1.5), 0.45,
            "no physical work past 1 Dec", fontsize=9.3, color="#b3471f",
            fontweight="bold", va="center")
    mid = DEFENSE[0] + (DEFENSE[1] - DEFENSE[0]) / 2
    ax.text(mid, 3.0, "DEFENSE" + NL + "WINDOW", ha="center", va="center",
            fontsize=10.5, color="#5a4a8a", fontweight="bold", linespacing=1.35)

    _gates(ax, PHYS_GATES, (-1.55, -2.05, -2.55))
    ax.set_yticks(range(len(PHYS_TASKS)))
    ax.set_yticklabels([t[0] for t in PHYS_TASKS], fontsize=9.0, color=INK)
    _frame(ax, len(PHYS_TASKS), -3.1)

    lab = sum(t[4] for t in PHYS_TASKS if t[4] is not None)
    weeks = (D(2026, 11, 27) - TODAY).days / 7.0
    fig.suptitle("Experimentation and Physical Work, to 27 November",
                 fontsize=16.4, fontweight="bold", color=INK, x=0.012, ha="left", y=0.972)
    fig.text(0.012, 0.912,
             f"~{lab:.0f} hours at the table across {weeks:.0f} weeks, about {lab/weeks:.0f} per week. "
             "Bars are calendar windows; the figure beside each one is what it costs in lab time.",
             fontsize=10.1, color=MUTE)
    fig.legend(handles=[Patch(facecolor=RIG, label="Rig"), Patch(facecolor=BUILD, label="Joint arm"),
                        Patch(facecolor=PHYS, label="Physical campaign"),
                        Patch(facecolor=SIM, label="Simulation"), Patch(facecolor=ANAL, label="Analysis")],
               loc="upper left", bbox_to_anchor=(0.009, 0.892), frameon=False,
               fontsize=9.5, ncol=5, handlelength=1.2, columnspacing=1.2,
               handletextpad=0.5)
    fig.subplots_adjust(top=0.800, bottom=0.072, left=0.300, right=0.968)
    fig.savefig(out, dpi=150, facecolor="white")
    return out, lab


def main():
    os.makedirs(FIGS, exist_ok=True)

    d_out, ready = drafting()
    print(f"wrote {d_out}   ({len(DRAFT_TASKS)} chapters, {ready} draftable today)")
    p_out, lab = physical()
    print(f"wrote {p_out}   ({len(PHYS_TASKS)} items, {lab:.1f} lab hours)")

    d_last = max(t[2] for t in DRAFT_TASKS)
    p_last = max(t[2] for t in PHYS_TASKS if t[3] == PHYS)
    by_dec1 = [t[0] for t in DRAFT_TASKS if t[2] <= PHYS_DEADLINE]
    print(f"\n  chapters drafted by 1 Dec : {len(by_dec1)} of {len(DRAFT_TASKS)}"
          f"  ({100*len(by_dec1)/len(DRAFT_TASKS):.0f}%)")
    print(f"  last physical day         : {p_last:%Y-%m-%d}")
    print(f"  last drafting day         : {d_last:%Y-%m-%d}")

    assert p_last < PHYS_DEADLINE, "physical work must finish before 1 Dec"
    assert d_last.year == 2026 and d_last < DEFENSE[0], "drafting must finish in 2026, before the defense"
    assert len(by_dec1) / len(DRAFT_TASKS) >= 0.66, "1 Dec must leave only the results tail"
    # the decoder in B1 is written against Ch3's formulation, not the reverse
    ch3_end = next(t[2] for t in DRAFT_TASKS if t[0].startswith("Ch3"))
    b1_end = next(t[2] for t in PHYS_TASKS if t[0].startswith("B1"))
    assert ch3_end < b1_end, "Ch3 must land before the decoder is finished"
    print("  constraints: OK")


if __name__ == "__main__":
    main()
