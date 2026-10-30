#!/usr/bin/env python3
"""Figures for the traffic-planning DOE.

    python3 tools/plot_doe.py        (after tools/build_doe_dataset.py)

Writes data/doe/figures/:
    fig1_makespan_by_method.png    every run, cell mean and 95% CI, planned
    fig2_interaction.png           method x robot count, means and 95% CI
    fig3_robot_finish_6r.png       per-robot finish time, 6 robots
"""
import csv
import math
import statistics as st
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy import stats  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DOE = ROOT / "data" / "doe"
FIG = DOE / "figures"
METHODS = ["astar", "wait", "pp", "cbs"]
LABEL = {"astar": "A*", "wait": "WAIT", "pp": "PP", "cbs": "CBS"}
# Categorical slots 1-4, fixed order; marker shape is the second channel.
COLOR = {"astar": "#2a78d6", "wait": "#eb6834", "pp": "#1baf7a",
         "cbs": "#eda100"}
MARKER = {"astar": "o", "wait": "s", "pp": "^", "cbs": "D"}
INK, INK2, GRID = "#1f1f1e", "#5f5e58", "#e4e3de"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": INK2,
    "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID,
    "grid.linewidth": 0.8, "axes.axisbelow": True,
})


def ci95(v):
    se = st.stdev(v) / math.sqrt(len(v))
    return st.mean(v), stats.t.ppf(0.975, len(v) - 1) * se


def load():
    runs = list(csv.DictReader(open(DOE / "doe_runs.csv", encoding="utf-8")))
    cells = defaultdict(list)
    planned = {}
    for r in runs:
        k = (int(r["robots"]), r["method"])
        cells[k].append((int(r["round"]), float(r["actual_makespan_sec"]),
                         r["video_title"]))
        planned[k] = float(r["planned_makespan_sec"])
    return cells, planned


def fig1(cells, planned):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.6))
    for ax, R in zip(axes, (4, 6)):
        for i, m in enumerate(METHODS):
            pts = sorted(cells[(R, m)])
            ys = [p[1] for p in pts]
            xs = [i - 0.16 + 0.08 * (p[0] - 1) for p in pts]   # round order
            ax.scatter(xs, ys, s=46, marker=MARKER[m], color=COLOR[m],
                       edgecolor="white", linewidth=1.2, zorder=3)
            mu, h = ci95(ys)
            ax.errorbar(i + 0.30, mu, yerr=h, fmt="_", ms=16, mew=2,
                        color=INK, elinewidth=1.6, capsize=4, zorder=4)
            ax.text(i + 0.40, mu, f"{mu:.1f}", va="center", ha="left",
                    fontsize=8.5, color=INK)
            ax.plot([i - 0.25, i + 0.22], [planned[(R, m)]] * 2, ls="--",
                    lw=1.2, color=INK2, zorder=2)
            top = max(pts, key=lambda p: p[1])
            if top[1] - mu > 25:
                ax.annotate(top[2], (xs[pts.index(top)], top[1]),
                            xytext=(8, -2), textcoords="offset points",
                            fontsize=8, color=INK2)
        ax.set_xticks(range(4), [LABEL[m] for m in METHODS])
        ax.set_xlim(-0.5, 3.75)
        ax.set_title(f"{R} robots", loc="left", fontsize=11, color=INK,
                     fontweight="bold")
        ax.set_ylabel("Makespan (s)")
    fig.text(0.01, 0.01, "Dots: individual runs, rounds 1-5 left to right."
             "   Bar: mean with 95% CI.   Dashed line: planned makespan."
             "   Each panel has its own y scale.", fontsize=8, color=INK2)
    fig.suptitle("Actual makespan by traffic-planning method",
                 x=0.01, ha="left", fontsize=13, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(FIG / "fig1_makespan_by_method.png", dpi=200)
    plt.close(fig)


def fig2(cells):
    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    ends = {}
    for j, m in enumerate(METHODS):
        xs, mus, hs = [], [], []
        for R in (4, 6):
            mu, h = ci95([p[1] for p in cells[(R, m)]])
            xs.append(R + (j - 1.5) * 0.06)
            mus.append(mu)
            hs.append(h)
        ax.errorbar(xs, mus, yerr=hs, color=COLOR[m], marker=MARKER[m],
                    ms=8, mec="white", mew=1.2, lw=2, capsize=3, zorder=3,
                    label=LABEL[m])
        ends[m] = mus[1]
    placed = []                      # end labels, at least 2.8 s apart
    for m, y in sorted(ends.items(), key=lambda t: t[1]):
        ly = max([y] + [q + 2.8 for q in placed])
        placed.append(ly)
        ax.text(6.22, ly, f"{LABEL[m]} {y:.1f}", va="center", fontsize=9,
                color=INK)
    ax.set_xticks([4, 6], ["4 robots", "6 robots"])
    ax.set_xlim(3.6, 6.9)
    ax.set_ylabel("Mean makespan (s), 95% CI")
    ax.legend(frameon=False, loc="upper right", fontsize=9)
    ax.set_title("Method x robot count", loc="left", fontsize=12, color=INK,
                 fontweight="bold")
    fig.tight_layout()
    fig.savefig(FIG / "fig2_interaction.png", dpi=200)
    plt.close(fig)


def fig3():
    rob = defaultdict(list)
    for r in csv.DictReader(open(DOE / "doe_robots.csv", encoding="utf-8")):
        if r["robots"] == "6" and r["completion_sec"]:
            rob[(r["robot_id"], r["method"])].append(float(r["completion_sec"]))
    fig, ax = plt.subplots(figsize=(9, 4.4))
    names = [f"Alvik{i}" for i in range(1, 7)]
    for j, m in enumerate(METHODS):
        xs, mus, hs = [], [], []
        for i, a in enumerate(names):
            mu, h = ci95(rob[(a, m)])
            xs.append(i + (j - 1.5) * 0.17)
            mus.append(mu)
            hs.append(h)
        ax.errorbar(xs, mus, yerr=hs, fmt=MARKER[m], color=COLOR[m], ms=7,
                    mec="white", mew=1, elinewidth=1.4, capsize=2,
                    label=LABEL[m], zorder=3)
    ax.set_xticks(range(6), names)
    ax.set_ylabel("Finish time (s), mean and 95% CI")
    ax.legend(frameon=False, ncol=4, loc="upper left", fontsize=9)
    ax.set_title("6 robots: when each robot finishes", loc="left",
                 fontsize=12, color=INK, fontweight="bold")
    fig.tight_layout()
    fig.savefig(FIG / "fig3_robot_finish_6r.png", dpi=200)
    plt.close(fig)


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    cells, planned = load()
    fig1(cells, planned)
    fig2(cells)
    fig3()
    for p in sorted(FIG.glob("*.png")):
        print(" ", p)


if __name__ == "__main__":
    main()
