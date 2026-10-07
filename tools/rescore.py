#!/usr/bin/env python3
"""Re-grade a saved results JSON with the CURRENT checkers. No inference.

WHY THIS EXISTS
---------------
2026-09-10. A 1h20m run reported orientation at 33% against a 42% base-rate
floor, which reads as a model that cannot do orientation at all. The model
was answering correctly; _single_cardinal() was reading the whole transcript
instead of the conclusion, and a correct "Therefore, Alvik2 is facing
**east**" scored wrong because the model had restated the four-way legend on
its way there. Every answer was already on disk. Re-running 665 questions to
find that out would have cost another 80 minutes of GPU time for information
the file already contained.

So: grader fixes are free from here on. Every row stores the raw answer, and
the questions rebuild deterministically from truth.json -- _spread() is
rotated by _sample_offset(truth), not by call order -- so the same sample
yields the same question ids on every rebuild.

    python3 tools/rescore.py tools/vlm_results_yawfix_image.json
    python3 tools/rescore.py results.json --write results_rescored.json

Without --write nothing is modified; it prints what would change. Rescoring
does NOT alter the saved answers, only the correct flags, so a file can be
re-graded repeatedly as the checkers improve.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vlm_bench as vb  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("results", help="a results JSON written by vlm_bench.py")
    ap.add_argument("--samples-dir", default=None)
    ap.add_argument("--max-per-category", type=int, default=3,
                    help="must match the run being rescored (default 3). A "
                         "mismatch shows up as unmatched question ids.")
    ap.add_argument("--write", default=None,
                    help="write the re-graded rows here (default: dry run)")
    args = ap.parse_args()

    with open(args.results, encoding="utf-8") as fh:
        doc = json.load(fh)
    rows = doc["rows"]

    samples_dir = args.samples_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vlm_samples")

    # One build per sample, not per row: build_questions() is the expensive
    # part and every row of a sample shares it.
    checks: dict[str, dict] = {}
    for sid in sorted({r["sample"] for r in rows}):
        path = os.path.join(samples_dir, sid, "truth.json")
        if not os.path.exists(path):
            print(f"  !! no truth.json for {sid}, its rows keep their scores")
            continue
        with open(path, encoding="utf-8") as fh:
            truth = json.load(fh)
        checks[sid] = {q["id"]: q for q in
                       vb.build_questions(truth, args.max_per_category)}

    changed, unmatched, by_t = [], 0, {}
    for r in rows:
        q = checks.get(r["sample"], {}).get(r["question_id"])
        if q is None:
            unmatched += 1
            continue
        try:
            new = bool(q["check"](r["answer"] or ""))
        except Exception:
            new = False
        t = r.get("template", r.get("category", "?"))
        st = by_t.setdefault(t, [0, 0, 0])
        st[0] += 1
        st[1] += bool(r["correct"])
        st[2] += new
        if new != bool(r["correct"]):
            changed.append((r, new))
        r["correct"] = new

    n = len(rows)
    old = sum(v[1] for v in by_t.values())
    now = sum(v[2] for v in by_t.values())
    print(f"{args.results}\n{n} rows, {len(checks)} samples"
          + (f", {unmatched} UNMATCHED question ids" if unmatched else ""))
    if unmatched:
        print("  (unmatched rows keep their original score -- check that "
              "--max-per-category matches the run)")

    print(f"\n{'template':16}{'n':>5}{'before':>9}{'after':>9}{'delta':>8}")
    for t, (cnt, o, w) in sorted(by_t.items(), key=lambda kv: -(kv[1][2] - kv[1][1])):
        d = (w - o) / cnt * 100
        print(f"{t:16}{cnt:>5}{o/cnt:>8.0%}{w/cnt:>9.0%}"
              + (f"{d:>+7.0f}pt" if o != w else f"{'--':>9}"))
    print(f"{'OVERALL':16}{n:>5}{old/n:>8.0%}{now/n:>9.0%}"
          f"{(now-old)/n*100:>+7.0f}pt")

    flips = sum(1 for _, new in changed if new)
    print(f"\n{len(changed)} rows changed: {flips} wrong->right, "
          f"{len(changed)-flips} right->wrong")
    for r, new in changed[:5]:
        if new:
            print(f"  now CORRECT  [{r.get('template')}] expected "
                  f"{r['expected']!r}: {(r['answer'] or '')[-90:].strip()!r}")

    if args.write:
        doc["rescored_utc"] = vb.datetime.now(vb.timezone.utc).isoformat(
            timespec="seconds")
        with open(args.write, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=1)
        print(f"\nwrote {args.write}")
        print(f"report it with: python3 tools/vlm_bench.py --report {args.write}")
    else:
        print("\ndry run -- pass --write PATH to save the re-graded rows")


if __name__ == "__main__":
    main()
