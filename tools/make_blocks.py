#!/usr/bin/env python3
"""Generate validated robot arrangements to use as RCBD blocks.

WHY THIS EXISTS
---------------
CLAUDE_CODE_VLM_RCBD_IMPLEMENTATION_BRIEF.md section 9 defines one block as
one physical robot configuration, and section 10 expects roughly 24-25 of
them. The 2026-09-10 session produced THREE configurations photographed four
times each -- which is 3 blocks, not 12, because section 20 is explicit that
deterministic repeats must not inflate the inferential sample size. Three
blocks cannot support the six-permutation counterbalancing of section 16, let
alone a block effect.

So the scarce resource is distinct arrangements, and placing six robots by
hand is the slow step. This generates arrangements that are known-good BEFORE
anyone walks to the table, so no lab time is spent on a layout that turns out
to produce unscoreable questions.

WHAT "VALIDATED" MEANS
----------------------
Each arrangement is run through the real truth_for() and build_questions()
and rejected unless:

  * all four headings appear, and every robot's yaw is unambiguous
    (truth_for drops facing within 30deg of two cardinals)
  * each of the four spatial extremes clears the 4in margin, so
    build_questions actually generates the question
  * the closest pair beats the runner-up by 4in, so closest_ok is not a
    coin flip -- measured 2026-09-10, 15 of 36 closest questions in the
    old set were ties under 2in
  * no robot is within 14in of another, so nothing occludes or collides
  * backfill_truth flags nothing as depot/workstation "unclear"

Two geometric traps the generator avoids, both found by hand on 2026-09-10:

  * AT MOST ONE robot in the depot. Slots are 6.3in apart and all at
    y~11, so two parked robots are permanently tied for southernmost AND
    permanently the closest pair, killing both questions.
  * NO robots on row-1 grid nodes that sit above the depot band. Node 3
    is 5.8in from slot D3, inside DEPOT_OUT_IN, so it is flagged
    "unclear in depot". Node 8 is fine -- it is east of D7.

Each robot only ever parks in ITS OWN slot: Alvik<N> -> D<N>, matching
the solver's depot-slot-${route.agent} assignment.

    python3 tools/make_blocks.py                 # 24 blocks, default seed
    python3 tools/make_blocks.py --blocks 12 --seed 7
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import apriltag_localize as al          # noqa: E402
import backfill_truth as bt             # noqa: E402
import vlm_bench as vb                  # noqa: E402
from vlm_capture import truth_for       # noqa: E402

ROBOTS = [f"Alvik{i}" for i in range(1, 7)]
YAWS = [0.0, 90.0, 180.0, -90.0]        # as the HUD reports them, -180..180
MIN_SEPARATION_IN = 14.0
EXTREME_MARGIN_IN = 4.0
CLOSEST_MARGIN_IN = 4.0


def _geom():
    gx, gy = al.GRID_NODE1_WORLD_IN
    p = al.GRID_PITCH_IN
    return (lambda n: (round(gx + p * ((n - 1) % 8), 2),
                       round(gy + p * ((n - 1) // 8), 2)),
            lambda b: al.WORKSTATION_WORLD_IN[b],
            lambda k: al.DEPOT_SLOT_WORLD_IN[f"D{k}"])


def validate(plan: dict) -> tuple[dict, int] | None:
    """(truth, question_count) if the arrangement is scoreable, else None."""
    poses = {r: {"x_in": c[0], "y_in": c[1], "yaw_deg": y}
             for r, (_w, c, y) in plan.items()}
    t = truth_for(poses)
    t.update(bt.occupancy(t["robots"], al.WORKSTATION_WORLD_IN,
                          al.DEPOT_SLOT_WORLD_IN))
    t["sample_id"] = "generated"
    if len(t["facing"]) < len(plan):
        return None                      # some yaw was too close to a diagonal to call
    if len(set(t["facing"].values())) < 4:
        return None
    for d, key, sign in (("east", "x_in", 1), ("west", "x_in", -1),
                         ("north", "y_in", 1), ("south", "y_in", -1)):
        v = sorted((poses[r][key] * sign, r) for r in poses)[::-1]
        if v[0][0] - v[1][0] <= EXTREME_MARGIN_IN:
            return None
    ds = sorted(((poses[a]["x_in"] - poses[b]["x_in"]) ** 2
                 + (poses[a]["y_in"] - poses[b]["y_in"]) ** 2) ** 0.5
                for i, a in enumerate(sorted(poses))
                for b in sorted(poses)[i + 1:])
    if ds[0] < MIN_SEPARATION_IN or (ds[1] - ds[0]) < CLOSEST_MARGIN_IN:
        return None
    if t.get("in_depot_unclear") or t.get("at_workstation_unclear"):
        return None
    return t, len(vb.build_questions(t, 3))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--blocks", type=int, default=24,
                    help="how many arrangements to generate (default 24: "
                         "the brief's target, and divisible by the six "
                         "treatment-order permutations)")
    ap.add_argument("--seed", type=int, default=20260911)
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "rcbd_blocks"))
    ap.add_argument("--tries", type=int, default=400000)
    args = ap.parse_args()

    node, bay, slot = _geom()
    # Row-1 nodes sitting above the depot band land in the "unclear" ring.
    safe_nodes = [n for n in range(1, 65)
                  if not (n <= 8 and 19.0 <= node(n)[0] <= 58.0)]
    bays = sorted(al.WORKSTATION_WORLD_IN)
    rng = random.Random(args.seed)

    found, seen = [], set()
    for _ in range(args.tries):
        if len(found) >= args.blocks:
            break
        idx = rng.sample(range(6), 6)
        n_dep = rng.choice([0, 1, 1, 1])          # at most one, see docstring
        n_bay = rng.choice([0, 1, 1, 2, 2])
        dep, at_bay = set(idx[:n_dep]), set(idx[n_dep:n_dep + n_bay])
        plan, used, ok = {}, [], True
        for i, r in enumerate(ROBOTS):
            if i in dep:
                where, xy = f"depot D{i + 1}", slot(i + 1)
            elif i in at_bay:
                b = rng.choice(bays)
                where, xy = f"bay {b}", bay(b)
            else:
                n = rng.choice(safe_nodes)
                where, xy = f"node {n}", node(n)
            if xy in used:
                ok = False
                break
            used.append(xy)
            plan[r] = (where, xy, rng.choice(YAWS))
        if not ok:
            continue
        key = tuple(sorted((r, v[0]) for r, v in plan.items()))
        if key in seen:
            continue
        res = validate(plan)
        if res:
            seen.add(key)
            found.append((plan, res[0], res[1]))

    os.makedirs(args.out, exist_ok=True)
    blocks = []
    for i, (plan, truth, nq) in enumerate(found, 1):
        blocks.append({
            "block_id": f"B{i:02d}",
            "questions": nq,
            "in_depot": truth.get("in_depot", []),
            "at_workstation": truth.get("at_workstation", []),
            "robots": {r: {"where": w, "x_in": c[0], "y_in": c[1],
                           "yaw_deg": y, "faces": truth["facing"][r]}
                       for r, (w, c, y) in plan.items()},
        })
    with open(os.path.join(args.out, "blocks.json"), "w",
              encoding="utf-8") as fh:
        json.dump({"seed": args.seed, "count": len(blocks),
                   "blocks": blocks}, fh, indent=2)
        fh.write("\n")

    sheet = os.path.join(args.out, "placement_sheet.txt")
    with open(sheet, "w", encoding="utf-8", newline="") as fh:
        fh.write(f"RCBD placement sheet -- {len(blocks)} blocks, "
                 f"seed {args.seed}\n")
        fh.write("Yaw is as the HUD reports it (-180..180). "
                 "0=south 90=east 180=north -90=west\n\n")
        for b in blocks:
            fh.write(f"{'=' * 58}\n{b['block_id']}   "
                     f"{b['questions']} questions   "
                     f"depot={b['in_depot'] or '-'}  "
                     f"workstation={b['at_workstation'] or '-'}\n")
            fh.write(f"  {'robot':8}{'position':12}{'x':>7}{'y':>7}"
                     f"{'yaw':>6}  faces\n")
            for r in ROBOTS:
                d = b["robots"][r]
                fh.write(f"  {r:8}{d['where']:12}{d['x_in']:>7.1f}"
                         f"{d['y_in']:>7.1f}{int(d['yaw_deg']):>6}  "
                         f"{d['faces']}\n")
    print(f"{len(blocks)} validated blocks "
          f"({sum(b['questions'] for b in blocks)} questions total)")
    print(f"  {os.path.join(args.out, 'blocks.json')}")
    print(f"  {sheet}")
    if len(blocks) < args.blocks:
        print(f"  !! only found {len(blocks)} of {args.blocks} in "
              f"{args.tries} tries -- loosen a constraint or raise --tries")


if __name__ == "__main__":
    main()
