#!/usr/bin/env python3
"""Figures for the 2026-09-22 VLM 2x2x2. Every number comes from
analysis/findings_<date>.json -- nothing here is retyped.

    python3 tools/vlm_aggregate.py       # master table
    python3 tools/vlm_findings.py        # the numbers
    python3 tools/make_vlm_figures.py    # these figures

Writes to figures/. House style matches the existing deck figures (same ink,
paper, rule and family colours) so these compose with fig_arch / fig_launch
rather than looking like a different document.
"""
from __future__ import annotations

import json
import os
import sys
from datetime import date

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "figures")

INK, PAPER, GRID, RULE = "#191c1e", "#ffffff", "#d9d6cf", "#5a6469"
GOOD, BAD, WARN = "#2f6a4a", "#a8322a", "#8c6d21"
MODEL_COLOR = {
    "gemma4:e4b-it-qat": "#2f6a4a", "gemma4:e2b-it-qat": "#4d8f6a",
    "minicpm-v4.6:1b": "#8c6d21", "qwen3.5:0.8b": "#125668",
}
MODELS = list(MODEL_COLOR)
CEILING_SEC = 5.0

#: (dx, dy, ha, va) for the model label in the accuracy/latency figure.
LABEL_NUDGE = {
    "gemma4:e4b-it-qat": (0.14, -2.2, "left", "top"),
    "gemma4:e2b-it-qat": (-0.12, 1.6, "right", "bottom"),
    "minicpm-v4.6:1b":   (-0.12, 1.6, "right", "bottom"),
    "qwen3.5:0.8b":      (-0.12, 1.6, "right", "bottom"),
}

FAMILY_LABEL = {
    "counting": "counting", "control": "control probes",
    "yaw_readback": "yaw readback", "relation": "relation",
    "orientation": "orientation", "spatial": "spatial",
    "position_readback": "position readback",
}


def style(ax, xgrid=True):
    ax.set_facecolor(PAPER)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK, labelsize=10.5)
    ax.grid(axis="x" if xgrid else "y", color=GRID, lw=.7, alpha=.7)
    ax.set_axisbelow(True)


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, bbox_inches="tight", facecolor=PAPER, dpi=200)
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))


#: Display names. Stripping the vendor prefix from minicpm left "v4.6:1b",
#: which names nothing a reader would recognise.
SHORT = {"gemma4:e4b-it-qat": "gemma4 e4b", "gemma4:e2b-it-qat": "gemma4 e2b",
         "minicpm-v4.6:1b": "minicpm 1b", "qwen3.5:0.8b": "qwen3.5 0.8b"}


def short(m):
    return SHORT.get(m, m)


# ---------------------------------------------------------------- figure 1
def fig_factors(F):
    """The headline: three factors on one axis, with intervals.

    An effect plot rather than three bar charts, because the comparison IS the
    finding -- the factor that was held constant is the only one that moved.
    """
    order = ["blank_image", "overlay", "jpeg"]
    label = {"blank_image": "blank reference image\n(present vs absent)",
             "overlay": "pose overlay\n(on vs off)",
             "jpeg": "JPEG quality\n(95 vs 75)"}
    fig, ax = plt.subplots(figsize=(9.6, 2.9))
    fig.patch.set_facecolor(PAPER)
    ys = list(range(len(order)))[::-1]
    for y, k in zip(ys, order):
        e = F["effects"][k]
        lo, hi = e["ci95"]
        sig = isinstance(e["wilcoxon_p"], float) and e["wilcoxon_p"] < 0.05
        col = BAD if (sig and e["block_mean_delta_pp"] < 0) else (
            GOOD if sig else RULE)
        ax.plot([lo, hi], [y, y], color=col, lw=3.2, solid_capstyle="round",
                zorder=3)
        ax.plot([e["block_mean_delta_pp"]], [y], "o", color=col, ms=11,
                zorder=4, markeredgecolor="white", markeredgewidth=1.4)
        p = e["wilcoxon_p"]
        ptxt = ("p < 0.0001" if isinstance(p, float) and p < 1e-4
                else f"p = {p:.2f}")
        ax.text(hi + 0.35, y, f"{e['block_mean_delta_pp']:+.2f} pp    {ptxt}",
                va="center", fontsize=11, color=col,
                fontweight="700" if sig else "500")
    ax.axvline(0, color=INK, lw=1.2, zorder=2)
    ax.set_yticks(ys)
    ax.set_yticklabels([label[k] for k in order], fontsize=11)
    ax.set_xlim(-7.2, 5.6)
    ax.set_xlabel("change in accuracy, percentage points "
                  "(95% CI over 24 blocks)", fontsize=11)
    ax.text(0, 1.10, "Effect of Three Configuration Factors on VLM Accuracy",
            transform=ax.transAxes, fontsize=12.5, fontweight="700",
            color=INK, va="bottom")
    ax.text(0, 1.015,
            f"2x2x2 design: {F['design']['rows']:,} question-instances, "
            f"{len(F['design']['cells'])} cells, "
            f"{len(F['design']['models'])} models, "
            f"{F['design']['blocks']} blocks; identical frames in every cell",
            transform=ax.transAxes, fontsize=10, color=RULE, va="bottom")
    style(ax)
    save(fig, "fig_vlm_factors.png")


# ---------------------------------------------------------------- figure 2
def fig_by_model(F):
    """Which model is sensitive to what. The deployment argument.

    Grouped by model rather than by factor: the question a reader has is
    "which one do I run", and that is answered by reading one row.
    """
    factors = ["blank_image", "overlay", "jpeg"]
    names = {"blank_image": "blank ref. image", "overlay": "pose overlay",
             "jpeg": "JPEG 95 vs 75"}
    fig, ax = plt.subplots(figsize=(10.0, 4.6))
    fig.patch.set_facecolor(PAPER)
    h, gap = 0.24, 0.30
    yticks, ylabels = [], []
    for i, m in enumerate(MODELS):
        base = -i * (len(factors) * h + gap)
        yticks.append(base - h)
        ylabels.append(short(m))
        for j, fac in enumerate(factors):
            e = F["by_model"][fac][m]
            if not e:
                continue
            y = base - j * h
            v = e["block_mean_delta_pp"]
            p = e["wilcoxon_p"]
            sig = isinstance(p, float) and p < 0.05
            col = (BAD if v < 0 else GOOD) if sig else GRID
            ax.barh(y, v, height=h * 0.82, color=col,
                    edgecolor=RULE if not sig else "none", linewidth=.8)
            ax.text(v + (0.35 if v >= 0 else -0.35), y,
                    f"{v:+.1f}" + ("*" if sig else ""),
                    va="center", ha="left" if v >= 0 else "right",
                    fontsize=9.5, color=INK,
                    fontweight="700" if sig else "400")
            ax.text(-21.4, y, names[fac], va="center", ha="left",
                    fontsize=8.5, color=RULE)
    ax.axvline(0, color=INK, lw=1.1)
    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels, fontsize=11, fontweight="600")
    ax.set_xlim(-21.6, 6.4)
    ax.set_xlabel("change in accuracy, percentage points   "
                  "(* = Wilcoxon p < 0.05 over 24 blocks)", fontsize=10.5)
    ax.text(0, 1.085, "Configuration Factor Effects on VLM Accuracy by Model",
            transform=ax.transAxes, fontsize=12.5, fontweight="700",
            color=INK, va="bottom")
    ax.text(0, 1.02, "paired block differences, 24 blocks per model",
            transform=ax.transAxes, fontsize=10, color=RULE, va="bottom")
    style(ax)
    save(fig, "fig_vlm_by_model.png")


# ---------------------------------------------------------------- figure 3
def fig_blank_families(F):
    """Where the blank-image harm lands, with the counting caveat drawn in.

    Counting is shown twice -- as graded, and under the stricter reading --
    because most of the as-graded gap is the grader taking the first number
    out of an enumerated answer, and a figure that showed only the larger
    bar would be quoting an artefact.
    """
    fam = F["by_family"]["blank_image"]
    rows = [(k, v) for k, v in fam.items() if v and v["hi_n"] >= 200]
    rows.sort(key=lambda kv: kv[1]["delta_pp"])
    fig, ax = plt.subplots(figsize=(9.8, 5.0))
    fig.patch.set_facecolor(PAPER)
    ys = list(range(len(rows)))
    for y, (k, e) in zip(ys, rows):
        v = e["delta_pp"]
        p = e.get("holm_p", e.get("p"))
        sig = isinstance(p, float) and p < 0.05
        ax.barh(y, v, height=.62, color=BAD if sig else GRID,
                edgecolor="none" if sig else RULE, linewidth=.8)
        ax.text(v - 0.6, y, f"{v:+.1f}", va="center", ha="right",
                fontsize=10.5, color=INK, fontweight="700" if sig else "400")
    # the corrected counting bar, drawn as an outline over the as-graded one
    corrected = F["counting_sensitivity"]["last-number"]["counting"]
    if corrected and any(k == "counting" for k, _ in rows):
        yc = [y for y, (k, _) in zip(ys, rows) if k == "counting"][0]
        cv = corrected["delta_pp"]
        ax.barh(yc, cv, height=.62, facecolor="none", edgecolor=INK,
                linewidth=2.0, linestyle="--", zorder=5)
        # Above the bar, not inside it: at counting's scale the dashed
        # outline is only ~11 pp wide and the caption overprinted it.
        ax.text(cv, yc - 0.52,
                f"{cv:+.1f} once the grader's first-number rule is corrected",
                va="top", ha="center", fontsize=9, color=INK, zorder=6)
    ax.axvline(0, color=INK, lw=1.1)
    ax.set_yticks(ys)
    ax.set_yticklabels([FAMILY_LABEL.get(k, k) for k, _ in rows], fontsize=11)
    ax.set_ylim(-1.25, len(rows) - 0.4)
    ax.set_xlim(-32, 7)
    ax.set_xlabel("change in accuracy from adding the blank reference image, "
                  "percentage points", fontsize=10.5)
    ax.text(0, 1.075, "Effect of Reference Image on VLM Accuracy by Question "
                      "Category", transform=ax.transAxes, fontsize=12.5,
            fontweight="700", color=INK, va="bottom")
    ax.text(0, 1.012, "dashed outline: counting under the corrected "
                      "number-extraction rule",
            transform=ax.transAxes, fontsize=10, color=RULE, va="bottom")
    style(ax)
    save(fig, "fig_vlm_blank_families.png")


# ---------------------------------------------------------------- figure 4
def fig_accuracy_latency(F):
    """Model selection: accuracy against the tail latency a robot waits out.

    Each model is one arrow, from its no-reference-image state to its
    with-reference state. Plotting BOTH coordinates per arm matters -- an
    earlier draft put the same accuracy on both ends and drew only the
    latency cost, hiding half of what the factor does.
    """
    fig, ax = plt.subplots(figsize=(9.4, 5.0))
    fig.patch.set_facecolor(PAPER)
    for m in MODELS:
        a0 = F["accuracy"][m]["blank_False_pct"]
        a1 = F["accuracy"][m]["blank_True_pct"]
        k0, k1 = f"{m}|blank=False", f"{m}|blank=True"
        if k0 not in F["latency"] or k1 not in F["latency"]:
            continue
        x0, x1 = F["latency"][k0]["p95"], F["latency"][k1]["p95"]
        c = MODEL_COLOR[m]
        ax.annotate("", xy=(x1, a1), xytext=(x0, a0),
                    arrowprops=dict(arrowstyle="-|>", color=c, lw=1.8,
                                    alpha=.65, shrinkA=9, shrinkB=9), zorder=3)
        ax.plot([x0], [a0], "o", color=c, ms=13, zorder=5,
                markeredgecolor="white", markeredgewidth=1.4)
        ax.plot([x1], [a1], "s", color=c, ms=10, zorder=5, alpha=.55,
                markeredgecolor="white", markeredgewidth=1.2)
        # Hand-placed: the two gemma points sit ~0.7s apart on x and one
        # label above each overprints the other.
        dx, dy, ha, va = LABEL_NUDGE.get(m, (-0.10, 1.5, "right", "bottom"))
        ax.text(x0 + dx, a0 + dy, short(m), ha=ha, va=va,
                fontsize=10.5, color=c, fontweight="600")
        if abs(a0 - a1) >= 1.0:
            ax.text(x1 + 0.10, a1 - 0.6, f"{a1 - a0:+.1f} pp",
                    ha="left", va="top", fontsize=9.5, color=c)
    ax.axvline(CEILING_SEC, color=BAD, ls="--", lw=1.5)
    ax.text(CEILING_SEC - 0.12, 88.4, "5 s ceiling", color=BAD, fontsize=10,
            va="top", ha="right")
    ax.set_xlim(0, 5.5)
    # qwen3.5 0.8b lands at 55.9% with the reference image; a floor of 56
    # clipped the very point the figure exists to show.
    ax.set_ylim(52, 90)
    ax.set_xlabel("p95 seconds per answer", fontsize=11)
    ax.set_ylabel("accuracy", fontsize=11)
    ax.text(0, 1.075, "VLM Accuracy and p95 Latency With and Without "
                      "Reference Image", transform=ax.transAxes,
            fontsize=12.5, fontweight="700", color=INK, va="bottom")
    ax.text(0, 1.012, "circle = without reference image, square = with; "
                      f"all runs on {F.get('machine', 'one machine')}",
            transform=ax.transAxes, fontsize=10, color=RULE, va="bottom")
    style(ax, xgrid=False)
    ax.grid(axis="both", color=GRID, lw=.7, alpha=.6)
    save(fig, "fig_vlm_accuracy_latency.png")


# ---------------------------------------------------------------- figure 5
def fig_orientation(F):
    """Orientation by TRUE facing. An overall score hides a dead axis."""
    axes_order = ["north", "south", "east", "west"]
    fig, ax = plt.subplots(figsize=(9.4, 4.4))
    fig.patch.set_facecolor(PAPER)
    w = 0.2
    for i, m in enumerate(MODELS):
        vals = [(F["orientation"][m].get(a) or {}).get("pct") or 0.0
                for a in axes_order]
        xs = [j + (i - 1.5) * w for j in range(len(axes_order))]
        ax.bar(xs, vals, width=w * 0.9, color=MODEL_COLOR[m], label=short(m))
        for x, v in zip(xs, vals):
            if v < 15:
                ax.text(x, v + 1.8, f"{v:.0f}", ha="center", fontsize=8.5,
                        color=BAD, fontweight="700")
    ax.axhline(25, color=RULE, ls=":", lw=1.2)
    ax.set_xticks(range(len(axes_order)))
    ax.set_xticklabels([a.upper() for a in axes_order], fontsize=11,
                       fontweight="600")
    ax.set_xlim(-0.5, 3.5)
    ax.set_ylim(0, 108)
    ax.set_ylabel("accuracy on that true facing", fontsize=11)
    ax.set_xlabel("true facing of the robot asked about", fontsize=11)
    ax.text(0, 1.075, "VLM Orientation Accuracy by True Robot Facing",
            transform=ax.transAxes, fontsize=12.5, fontweight="700",
            color=INK, va="bottom")
    ax.text(0, 1.012, "pooled over all 8 cells of the design; "
                      "dotted line = 25%, the chance rate on four options",
            transform=ax.transAxes, fontsize=10, color=RULE, va="bottom")
    ax.legend(frameon=False, fontsize=10, ncol=4, loc="upper center",
              bbox_to_anchor=(0.5, -0.17))
    style(ax, xgrid=False)
    ax.grid(axis="y", color=GRID, lw=.7, alpha=.7)
    save(fig, "fig_vlm_orientation.png")


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else str(date.today())
    path = os.path.join(ROOT, "analysis", f"findings_{stamp}.json")
    if not os.path.isfile(path):
        raise SystemExit(f"no findings file at {path} -- run vlm_findings.py")
    F = json.load(open(path, encoding="utf-8"))
    for f in (fig_factors, fig_by_model, fig_blank_families,
              fig_accuracy_latency, fig_orientation):
        f(F)


if __name__ == "__main__":
    main()
