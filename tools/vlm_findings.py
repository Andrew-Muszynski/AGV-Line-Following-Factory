#!/usr/bin/env python3
"""Compute the 2x2x2 findings into one JSON, so figures never retype a number.

Reads data/vlm_analysis/vlm_master.json (produced by vlm_aggregate.py) and writes
data/vlm_analysis/findings_<date>.json. Everything the figures and the write-up quote
comes from here, so a number on a slide can be traced to a block-level
difference and from there to individual answers.

THE DESIGN THIS SUMMARISES
--------------------------
    24 blocks x {overlay on, off} x {JPEG 95, 75} x {blank ref image, none}
    x 2 frames x 4 models = 32,192 question-instances, one sample pool,
    identical robot placement in every cell.

Effects are paired ON THE BLOCK, which is the randomisation unit: each block
was posed once and photographed under every condition, so a block-level
difference compares like with like. The reported interval is a 95% t
interval on those 24 block differences; the p-value is Wilcoxon signed-rank
on the same 24 numbers, because the differences are not assumed normal.

THE COUNTING CAVEAT, CARRIED IN THE DATA
----------------------------------------
vlm_bench grades a count by the FIRST number in the reply. A model that
answers "Box 1 is at ... Box 6 is at ... so there are 6" is scored as having
said 1. The blank-reference arm made models narrate both images, so this rare
grader weakness became the dominant failure there and inflated the measured
harm. Every counting figure is therefore computed under three reading rules
-- as-graded, last number, concluding sentence -- and the figures show all
three rather than silently picking one. `correct` in the master table is
never modified.
"""
from __future__ import annotations

import collections
import json
import os
import re
import sys
from datetime import date

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vlm_bench as VB  # noqa: E402
from vlm_aggregate import contrast  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = ["gemma4:e4b-it-qat", "gemma4:e2b-it-qat",
          "minicpm-v4.6:1b", "qwen3.5:0.8b"]
FACTORS = {
    # name: (column, high arm, low arm, how it is spoken)
    "blank_image": ("blank_image", True, False, "blank reference image"),
    "overlay":     ("overlay", "on", "off", "pose overlay"),
    "jpeg":        ("jpeg_quality", 95, 75, "JPEG quality 95 vs 75"),
}
RULES = ("as-graded", "last-number", "concluding-sentence")


def counting_score(row: dict, rule: str) -> bool:
    """Was the count right, under one reading rule. Non-counting rows are
    returned as graded, so a rule only ever changes the counting family."""
    if rule == "as-graded" or row["family"] != "counting":
        return bool(row["correct"])
    try:
        want = float(row["expected"])
    except (TypeError, ValueError):
        return bool(row["correct"])
    if rule == "last-number":
        nums = VB._numbers_in(VB._final_answer(row["answer"] or ""))
        return bool(nums) and nums[-1] == want
    tail = " ".join((row["answer"] or "").split())[-120:]
    return bool(re.search(r"\b(%d|six)\b" % int(want), tail, re.I))


def block_effect(rows, factor, rule="as-graded", model=None):
    """Paired high-minus-low, via vlm_aggregate.contrast.

    Delegating rather than re-deriving the pairing is deliberate: an earlier
    version paired on the block alone and disagreed with the contrast output
    by up to 1 pp, because the two overlay arms do NOT ask an identical
    question set -- build_questions rotates which questions each sample gets
    via _spread, so the arms differ slightly in composition. contrast() pairs
    on (model, question_id, block) and only then aggregates to the block, so
    it compares like with like. One implementation, one answer.
    """
    if rule != "as-graded":
        rows = [dict(r, correct=counting_score(r, rule)) for r in rows]
    res = contrast(rows, factor, model, "static")
    if not res:
        return None
    o = res["overall"]
    return {
        "hi_pct": o["hi_pct"], "lo_pct": o["lo_pct"],
        "pooled_delta_pp": o["delta_pp"],
        "block_mean_delta_pp": round(
            sum(o["block_diffs"]) / len(o["block_diffs"]), 3)
        if o.get("block_diffs") else None,
        "ci95": o.get("ci95", ""), "wilcoxon_p": o["wilcoxon_p"],
        "blocks": o["blocks"], "hi_n": o["hi_n"], "lo_n": o["lo_n"],
        "by_family": {f: {"delta_pp": v["delta_pp"], "p": v["wilcoxon_p"],
                          "holm_p": v.get("holm_p", ""),
                          "hi_pct": v["hi_pct"], "lo_pct": v["lo_pct"],
                          "hi_n": v["hi_n"], "lo_n": v["lo_n"]}
                      for f, v in res["families"].items()},
    }


def _unused_block_effect(rows, column, hi, lo, rule="as-graded"):
    """Superseded by the delegating version above; kept out of the results."""
    per = collections.defaultdict(lambda: [0, 0, 0, 0])
    for r in rows:
        v = r[column]
        if v == hi:
            i = 0
        elif v == lo:
            i = 2
        else:
            continue
        per[r["block"]][i] += counting_score(r, rule)
        per[r["block"]][i + 1] += 1
    diffs, hi_ok, hi_n, lo_ok, lo_n = [], 0, 0, 0, 0
    for a, b, c, d in per.values():
        if not (b and d):
            continue
        hi_ok += a; hi_n += b; lo_ok += c; lo_n += d
        diffs.append(100.0 * a / b - 100.0 * c / d)
    if not diffs:
        return None
    import statistics
    mean = statistics.fmean(diffs)
    ci = ""
    p = ""
    if len(diffs) > 2:
        try:
            from scipy import stats
            sem = statistics.stdev(diffs) / (len(diffs) ** 0.5)
            t = stats.t.ppf(0.975, len(diffs) - 1)
            ci = [round(mean - t * sem, 3), round(mean + t * sem, 3)]
            if any(abs(x) > 1e-9 for x in diffs):
                p = float(stats.wilcoxon(diffs).pvalue)
        except Exception:
            pass
    return {
        "hi_pct": round(100.0 * hi_ok / hi_n, 3),
        "lo_pct": round(100.0 * lo_ok / lo_n, 3),
        "pooled_delta_pp": round(100.0 * hi_ok / hi_n - 100.0 * lo_ok / lo_n, 3),
        "block_mean_delta_pp": round(mean, 3),
        "ci95": ci, "wilcoxon_p": p,
        "blocks": len(diffs), "hi_n": hi_n, "lo_n": lo_n,
    }


def main() -> None:
    master = json.load(open(os.path.join(ROOT, "analysis", "vlm_master.json"),
                            encoding="utf-8"))
    d = [r for r in master if "rcbd_2x2" in r["source_file"]]
    if not d:
        raise SystemExit("no 2x2x2 rows found -- run vlm_aggregate.py first")

    out = {
        "generated": str(date.today()),
        "design": {
            "rows": len(d),
            "models": sorted({r["model"] for r in d}),
            "blocks": len(sorted({r["block"] for r in d})),
            "cells": sorted({f"overlay={r['overlay']},jpeg={r['jpeg_quality']},"
                             f"blank={r['blank_image']}" for r in d}),
            "source_files": sorted({r["source_file"] for r in d}),
        },
        "effects": {}, "by_family": {}, "by_model": {},
        "counting_sensitivity": {}, "latency": {}, "orientation": {},
        "accuracy": {},
    }

    for name in FACTORS:
        e = block_effect(d, name)
        out["effects"][name] = e
        # contrast() already decomposes by family, with Holm across them.
        out["by_family"][name] = (e or {}).get("by_family", {})
        out["by_model"][name] = {m: block_effect(d, name, model=m)
                                 for m in MODELS}

    # The counting caveat, quantified rather than asserted.
    for rule in RULES:
        allr = block_effect(d, "blank_image", rule)
        out["counting_sensitivity"][rule] = {
            "overall": allr,
            "counting": (allr or {}).get("by_family", {}).get("counting"),
        }

    # Latency is kept apart from accuracy on purpose: it is a property of the
    # machine that served it, and both 2x2x2 runs were the same machine.
    lat = collections.defaultdict(list)
    for r in d:
        try:
            lat[(r["model"], bool(r["blank_image"]))].append(float(r["seconds"]))
        except (TypeError, ValueError):
            pass
    machines = sorted({str(r["machine"]) for r in d if r.get("machine")})
    out["machine"] = machines[0] if len(machines) == 1 else "|".join(machines)
    for (m, blank), s in lat.items():
        s.sort()
        q = lambda p: round(s[min(len(s) - 1, int(p * len(s)))], 3)
        out["latency"][f"{m}|blank={blank}"] = {
            "n": len(s), "p50": q(.50), "p90": q(.90), "p95": q(.95),
            "machine": out["machine"],
        }

    # Accuracy per model in the cheapest cell (no blank, no overlay, q75) and
    # overall, so the deployment claim is checkable.
    for m in MODELS:
        rows = [r for r in d if r["model"] == m]
        cheap = [r for r in rows if r["blank_image"] is False
                 and r["overlay"] == "off" and r["jpeg_quality"] == 75]
        f = lambda x: (round(100.0 * sum(bool(r["correct"]) for r in x) / len(x), 2)
                       if x else None)
        core = [r for r in rows if r["core"]]
        out["accuracy"][m] = {
            "all_cells_pct": f(rows), "core_pct": f(core),
            "cheapest_cell_pct": f(cheap),
            "blank_False_pct": f([r for r in rows if r["blank_image"] is False]),
            "blank_True_pct": f([r for r in rows if r["blank_image"] is True]),
            "n": len(rows), "n_cheapest": len(cheap)}

    # Orientation by TRUE facing: an overall score hides an axis a model
    # cannot resolve at all, and e4b has one.
    for m in MODELS:
        per = collections.defaultdict(lambda: {"ok": 0, "n": 0,
                                               "said": collections.Counter()})
        for r in d:
            if r["model"] != m or r["family"] != "orientation":
                continue
            a = per[str(r["expected"]).strip().lower()]
            a["n"] += 1
            a["ok"] += bool(r["correct"])
            a["said"][r["given"] or "(none)"] += 1
        out["orientation"][m] = {
            k: {"ok": v["ok"], "n": v["n"],
                "pct": round(100.0 * v["ok"] / v["n"], 2) if v["n"] else None,
                "top_wrong": [kk for kk, _ in v["said"].most_common(3) if kk != k][:1]}
            for k, v in sorted(per.items())
        }

    dest = os.path.join(ROOT, "analysis", f"findings_{out['generated']}.json")
    with open(dest, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
        fh.write("\n")
    print(f"wrote {dest}")
    print(f"  design: {out['design']['rows']} rows, "
          f"{len(out['design']['cells'])} cells, "
          f"{len(out['design']['models'])} models, "
          f"{out['design']['blocks']} blocks")
    for name, e in out["effects"].items():
        print(f"  {name:<12} {e['block_mean_delta_pp']:+6.2f} pp  "
              f"95% CI {e['ci95']}  p={e['wilcoxon_p']}")


if __name__ == "__main__":
    main()
