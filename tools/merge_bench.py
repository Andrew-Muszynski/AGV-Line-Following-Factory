#!/usr/bin/env python3
"""Merge vlm_bench result files so models run separately can be compared.

WHY THIS EXISTS
---------------
vlm_bench --models a b c runs every model in one invocation and writes one
JSON, and --report reads exactly one JSON. But a six-model RCBD pass over 96
samples is a multi-hour job, and running it as one command means a crash,
a reboot or a model that hangs costs the whole thing. Running one model per
invocation is the practical way to do it; this puts the pieces back together.

Separate invocations really are comparable. Question selection is
deterministic: _spread() is rotated by _sample_offset(), which hashes the
sample's OWN id, so there is no RNG and no run-level seed. Verified
empirically 2026-09-15 -- four separate probe runs over the same 3 samples
produced byte-identical (sample, question_id) sets.

WHAT IT REFUSES TO DO
---------------------
Comparability is exactly what a merge can silently destroy, so the settings
that change what a model is being asked are checked and a mismatch is an
error, not a warning:

    num_predict   an output cap binds verbose models and not terse ones. On
                  r4 it never fired for gemma4 and fired on 92% of
                  qwen3-vl's answers; merging capped with uncapped would
                  have read as a capability gap.
    num_ctx       changes how much of the prompt the model actually sees.
    condition     state+image vs state-only vs image-only are different
                  experiments wearing the same schema.
    repeats       changes the consistency denominator.

Rows are also intersected down to the (sample, question_id) pairs every
model actually answered, because a model that ran 96 samples and one that
ran 48 are not comparable on raw totals. What gets dropped is reported.

    python3 tools/merge_bench.py -o merged.json a.json b.json c.json
    python3 tools/vlm_bench.py --report merged.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

#: Settings that change what the model was asked. A mismatch is fatal.
MUST_MATCH = ("num_predict", "num_ctx", "condition", "repeats")


def load(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("files", nargs="+", help="vlm_bench result JSONs")
    ap.add_argument("-o", "--out", required=True, help="merged JSON path")
    ap.add_argument("--keep-all", action="store_true",
                    help="skip the intersection and keep every row. Totals "
                         "then come from different question sets per model "
                         "and are NOT comparable -- for inspection only.")
    args = ap.parse_args()

    docs = [(p, load(p)) for p in args.files]

    # -- settings must agree ------------------------------------------------
    ref_path, ref = docs[0]
    for path, d in docs[1:]:
        for k in MUST_MATCH:
            a, b = ref.get(k), d.get(k)
            if a != b:
                raise SystemExit(
                    "refusing to merge: %s is %r in %s but %r in %s.\n"
                    "These are different experiments; compare them "
                    "separately." % (k, a, os.path.basename(ref_path), b,
                                     os.path.basename(path)))

    # -- collect rows, newest file wins a duplicated model ------------------
    rows_by_model: dict[str, list] = {}
    order: list[str] = []
    for path, d in docs:
        for r in d.get("rows", []):
            m = r.get("model")
            if m not in rows_by_model:
                rows_by_model[m] = []
                order.append(m)
            rows_by_model[m].append(r)
    if not rows_by_model:
        raise SystemExit("no rows in any input file")

    # -- intersect to what every model answered -----------------------------
    keysets = {m: {(r["sample"], r["question_id"]) for r in rs}
               for m, rs in rows_by_model.items()}
    common = set.intersection(*keysets.values())
    print("models: %d" % len(order))
    for m in order:
        n = len(keysets[m])
        note = "" if n == len(common) else "  (%d not shared)" % (n - len(common))
        print("  %-24s %5d question(s)%s" % (m, n, note))
    print("common to all: %d" % len(common))
    if not common and not args.keep_all:
        raise SystemExit("no (sample, question) pair is shared by every "
                         "model -- nothing comparable to merge")

    rows = []
    for m in order:
        for r in rows_by_model[m]:
            if args.keep_all or (r["sample"], r["question_id"]) in common:
                rows.append(r)

    samples = {r["sample"] for r in rows}
    merged = dict(ref)
    merged.update({
        "models": order,
        "endpoints": {k: v for _p, d in docs
                      for k, v in (d.get("endpoints") or {}).items()},
        "rows": rows,
        "sample_count": len(samples),
        "question_count": len(rows),
        "merged_from": [os.path.basename(p) for p, _d in docs],
        "merged_intersected": not args.keep_all,
    })
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(merged, fh, indent=1)
    print("\nwrote %s  (%d samples, %d rows)"
          % (args.out, len(samples), len(rows)))
    print("report it with:\n  python3 tools/vlm_bench.py --report %s" % args.out)


if __name__ == "__main__":
    main()
