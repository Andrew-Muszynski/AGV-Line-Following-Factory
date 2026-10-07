#!/usr/bin/env python3
"""Split the old three-panel fig_repeatability.png into three standalone
figures, and regenerate panel C from live data.

WHY A AND B ARE CROPS
---------------------
Panels A (stopping error per leg) and B (per-robot sensor rates) were plotted
on 2026-09-18 from a parse of the launch-rig run that is not preserved:
runs_linux/launch_cascade_*.json records each of the 44 legs but carries no
stopping error and no sample rates. Rather than re-derive approximate values
and pass them off as measurements, the original panels are cropped out of the
archived PNG. The pixels are the real 2026-09-18 plot.

Panel C is different: it is a formula over the completed-run makespans, all of
which are in the metrics database, so it is redrawn from source. The 18 Sept
version said "three completed runs ... CV 4.4%", which is now out of date -
there are five, and the two most recent ran under identical code 0.1 s apart.
It also quoted a single MDD curve from a CV point estimate computed on n=3,
whose own 95% interval spans 2.3% to 27.8%. That interval is now drawn.

Re-run after locating the leg-level source data to replace the crops.
"""
from __future__ import annotations

import os
import sqlite3
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                    # noqa: E402
import numpy as np                                                 # noqa: E402
from PIL import Image                                              # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIGS = os.path.join(ROOT, "figures")
SOURCE = os.path.join(FIGS, "fig_repeatability.png")
ARCHIVE = os.path.join(FIGS, "fig_repeatability_3panel_20260918.png")

INK = "#1b2733"
MUTE = "#5b6b7a"
TEAL = "#1f6f7a"
RUST = "#b3471f"

# 80% power, 5% two-sided, paired two-arm: MDD = (z.975 + z.80) * sd * sqrt(2/n)
Z = 1.959964 + 0.841621


def split_panels() -> None:
    if not os.path.exists(ARCHIVE):
        Image.open(SOURCE).save(ARCHIVE)
        print("archived the three-panel original ->",
              os.path.basename(ARCHIVE))
    im = Image.open(ARCHIVE)
    w, h = im.size
    # The panel grid is 2 columns over 1 row, then a full-width row beneath.
    im.crop((0, 0, int(w * 0.485), int(h * 0.508))).save(
        os.path.join(FIGS, "fig_repeat_a_precision.png"))
    im.crop((int(w * 0.478), 0, w, int(h * 0.508))).save(
        os.path.join(FIGS, "fig_repeat_b_sensors.png"))
    print("wrote fig_repeat_a_precision.png, fig_repeat_b_sensors.png")


def makespans() -> list[float]:
    con = sqlite3.connect(os.path.join(ROOT, "data", "agv_metrics.sqlite3"))
    rows = con.execute(
        """SELECT MAX(rr.completion_elapsed_ms) AS m
             FROM runs r JOIN run_robots rr ON rr.run_id = r.run_id
            WHERE r.robot_count = 6 AND r.status = 'completed'
            GROUP BY r.run_id HAVING m IS NOT NULL
            ORDER BY r.started_at_utc""").fetchall()
    con.close()
    return [r[0] / 1000.0 for r in rows]


def panel_c() -> None:
    mk = makespans()
    old, new = mk[:3], mk[3:]
    sd = st.stdev(old)
    mean = st.fmean(old)
    cv = 100.0 * sd / mean
    # 95% interval on a CV from n=3 -- chi-square on the variance.
    lo_cv, hi_cv = 2.3, 27.8
    n = np.linspace(2, 30, 400)

    def mdd(c):
        return Z * (c / 100.0 * mean) * np.sqrt(2.0 / n)

    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=150)
    ax.fill_between(n, mdd(lo_cv), mdd(hi_cv), color=TEAL, alpha=0.13,
                    linewidth=0,
                    label=f"95% interval on the CV ({lo_cv}% - {hi_cv}%)")
    ax.plot(n, mdd(cv), color=TEAL, lw=2.4,
            label=f"CV point estimate ({cv:.1f}%, n=3)")

    for k in (3, 10, 30):
        y = Z * sd * (2.0 / k) ** 0.5
        ax.plot([k], [y], "o", color=RUST, ms=8, zorder=5)
        # the n=30 marker sits on the right spine, so its label goes left
        off = (-78, 14) if k == 30 else (10, 12)
        ax.annotate(f"n={k}: {y:.0f} s", (k, y), textcoords="offset points",
                    xytext=off, fontsize=11, color=INK, fontweight="bold")

    ax.set_xlabel("replications per arm", fontsize=12)
    ax.set_ylabel("smallest detectable difference (s)", fontsize=12)
    ax.set_title("C · What the noise floor costs", fontsize=15,
                 fontweight="bold", color=INK, loc="left", pad=12)
    ax.set_xlim(2, 30)
    ax.set_ylim(0, 180)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=10, loc="upper right", framealpha=0.95)

    txt = (f"five completed six-robot runs, same exported plan\n"
           + " / ".join(f"{m:.1f}" for m in mk) + " s\n"
           f"first three (n=3): sd {sd:.1f} s, CV {cv:.1f}%\n"
           f"most recent two, identical code: {abs(new[0] - new[1]):.1f} s apart")
    ax.text(0.035, 0.955, txt, transform=ax.transAxes, fontsize=10,
            color=MUTE, va="top", linespacing=1.5)

    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    fig.tight_layout()
    out = os.path.join(FIGS, "fig_repeat_c_power.png")
    fig.savefig(out, dpi=150, facecolor="white")
    print("wrote", os.path.basename(out))
    print(f"  makespans {[round(m, 1) for m in mk]}")
    print(f"  MDD at CV {cv:.1f}%: n=3 {Z * sd * (2 / 3) ** 0.5:.0f} s | "
          f"at CV {hi_cv}%: n=3 {Z * (hi_cv / 100 * mean) * (2 / 3) ** 0.5:.0f} s")


if __name__ == "__main__":
    split_panels()
    panel_c()
