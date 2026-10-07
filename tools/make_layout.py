#!/usr/bin/env python3
"""Generate tools/testbed_layout.txt -- the static layout the VLM is told.

Positions come from apriltag_localize.py's measured tables (49 bays measured
individually 2026-08-26, plus depot slots and entries). Nothing is retyped
here; a second copy of those numbers is a second thing to drift.

WHY FORMULAS AND NOT THE TABLE
------------------------------
The first version listed all 49 bays explicitly. Measured 2026-09-09 against
qwen3.5:4b's own tokenizer, that block cost 1086 tokens -- because coordinate
data tokenizes at ~1.0 characters per token, not the ~5.8 of English prose.
Every digit, comma and decimal point is its own token. The prompt went from
2087 to 3353 tokens and per-question latency went from 2.1s to 8.6s.

The lattice is regular enough that formulas replace it almost exactly:

    bays          grid node + (4.83, 4.97)   worst error 0.61in
    depot slots   x=19.6+6.28*(k-1), y=11.1  worst error 0.25in
    depot entries x=19.7+6.30*(k-1), y=3.2   worst error 0.30in

Against a 4in "at a workstation" threshold and a 3in "in the depot" one, with
the closest two bays 9.4in apart, sub-inch error cannot change an answer.

--explicit restores the full measured table if you ever need the precision
and can afford the tokens.

    python3 tools/make_layout.py
    python3 tools/make_layout.py --explicit
"""
from __future__ import annotations

import argparse
import os
import sys

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "testbed_layout.txt")


def _fit_row(table: dict, prefix: str) -> tuple[float, float, float, float]:
    """(x0, dx, y, worst_error) for a evenly-spaced labelled row."""
    keys = sorted(table, key=lambda k: int(k[len(prefix):]))
    xs = [table[k][0] for k in keys]
    ys = [table[k][1] for k in keys]
    dx = (xs[-1] - xs[0]) / (len(xs) - 1)
    y = sum(ys) / len(ys)
    worst = max(((xs[i] - (xs[0] + dx * i)) ** 2 + (ys[i] - y) ** 2) ** 0.5
                for i in range(len(keys)))
    return xs[0], dx, y, worst


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--explicit", action="store_true",
                    help="list all 49 measured bays instead of the formula "
                         "(about 25x the tokens, for 0.2in median accuracy)")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, root)
    try:
        import apriltag_localize as al
    except Exception as exc:
        raise SystemExit(f"cannot import apriltag_localize: {exc}")

    gx, gy = al.GRID_NODE1_WORLD_IN
    pitch = al.GRID_PITCH_IN
    bays = al.WORKSTATION_WORLD_IN
    cols_of_bays = 7

    # Best-fit offset of a bay from its grid node, then the residual.
    offs = []
    for b, (bx, by) in sorted(bays.items()):
        i = b - min(bays)
        offs.append((bx - (gx + pitch * (i % cols_of_bays)),
                     by - (gy + pitch * (i // cols_of_bays))))
    ox = sum(o[0] for o in offs) / len(offs)
    oy = sum(o[1] for o in offs) / len(offs)
    bx0, by0 = round(gx + ox, 1), round(gy + oy, 1)
    bay_err = max(
        ((bays[b][0] - (bx0 + pitch * ((b - min(bays)) % cols_of_bays))) ** 2
         + (bays[b][1] - (by0 + pitch * ((b - min(bays)) // cols_of_bays)))
         ** 2) ** 0.5
        for b in bays)

    sx, sdx, sy, s_err = _fit_row(al.DEPOT_SLOT_WORLD_IN, "D")
    ex, edx, ey, e_err = _fit_row(al.DEPOT_ENTRY_WORLD_IN, "DE")

    def _rows(first: int, per_row: int, n_rows: int, y0: float) -> str:
        """Row bands as literal ranges: "1-8 y=16.75 | 9-16 y=26.75 | ..."."""
        out = []
        for k in range(n_rows):
            lo = first + k * per_row
            out.append(f"{lo}-{lo + per_row - 1} y={y0 + pitch * k:.2f}")
        return " | ".join(out)

    def _cols(x0: float, n: int) -> str:
        """Column x values by position, no index arithmetic."""
        return ", ".join(f"{x0 + pitch * k:.1f}" for k in range(n))

    # NO SUBTRACTION ANYWHERE IN THIS BLOCK. The previous version wrote the
    # positions as formulas -- "x=13.5+10*((n-1) mod 8)" and "x=19.6+6.3*(k-1)"
    # -- which put "(n-1)" in the prompt twice and "(k-1)" twice. Measured
    # 2026-09-10 on qwen3.5:4b with no image, the counting question then
    # answered N-1: truth 2 gave "1", truth 6 gave "5", 22 of 39 answers were
    # exactly one short, and both a 3080 Ti and a 5060 reproduced it. Deleting
    # the layout block restored 100%. The state text says "robots tracked: 6"
    # outright, so this was retrieval being derailed by nearby decrements.
    # Row bands and column lists cost a few more tokens and ask the model to
    # look something up instead of computing it.
    lines = [
        "Testbed layout (inches, same frame as the robot poses above):",
        f"- Grid nodes 1-{8 * 8}: an 8 by 8 lattice spaced {pitch:.0f}in "
        "apart. Node 1 is the south-west corner. Numbering runs east along "
        "a row of 8, then continues on the next row to the north.",
        f"    row y, south to north: {_rows(1, 8, 8, gy)}",
        f"    x by position in the row, west to east: {_cols(gx, 8)}",
    ]

    if args.explicit:
        lines += [
            f"- Workstation bays {min(bays)}-{max(bays)}:",
            "    " + " ".join(f"{b}:({x:.1f},{y:.1f})"
                              for b, (x, y) in sorted(bays.items())),
            "  A robot is AT a workstation when within 4in of a bay.",
        ]
    else:
        n_bay_rows = -(-len(bays) // cols_of_bays)
        lines += [
            f"- Workstation bays {min(bays)}-{max(bays)}: each sits at the "
            f"centre of a grid cell, {cols_of_bays} per row. A robot is AT "
            "a workstation when within 4in of a bay.",
            f"    row y, south to north: "
            f"{_rows(min(bays), cols_of_bays, n_bay_rows, by0)}",
            f"    x by position in the row, west to east: "
            f"{_cols(bx0, cols_of_bays)}",
        ]

    slots = al.DEPOT_SLOT_WORLD_IN
    entries = al.DEPOT_ENTRY_WORLD_IN
    lines += [
        "- Workstation entry nodes: entry = bay + 49, sitting on the grid "
        "edge just outside its bay. A robot on an entry node is in the "
        "corridor, NOT at the workstation.",
        "- Depot slots, where robots park. A robot is IN THE DEPOT when "
        "within 3in of a slot.",
        "    " + " ".join(f"{k}:({v[0]:.1f},{v[1]:.1f})" for k, v in
                          sorted(slots.items(),
                                 key=lambda kv: int(kv[0][1:]))),
        "- Depot entry nodes, the lane in front of the depot. A robot here "
        "is NOT in the depot.",
        "    " + " ".join(f"{k}:({v[0]:.1f},{v[1]:.1f})" for k, v in
                          sorted(entries.items(),
                                 key=lambda kv: int(kv[0][2:]))),
    ]

    text = "\n".join(lines) + "\n"
    with open(OUT, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)

    print(f"wrote {OUT}  ({len(text)} chars)")
    if not args.explicit:
        print(f"  approximation error vs the measured tables: "
              f"bays {bay_err:.2f}in, depot {s_err:.2f}in, "
              f"entries {e_err:.2f}in")
        print("  thresholds are 4in (workstation) and 3in (depot); the two "
              "closest bays are 9.4in apart")


if __name__ == "__main__":
    main()
