#!/usr/bin/env python3
"""Add depot / workstation occupancy to captured samples' truth.json.

Existing samples were captured before these fields existed, but they already
store every robot's x_in and y_in, and the measured bay and depot tables live
in apriltag_localize.py -- so occupancy is derivable after the fact and the
32 samples already on disk can answer the new questions without recapturing.

Positions come from apriltag_localize.py rather than being copied here. That
file and camera_grid_navigate.py already hold identical copies of these
tables; a third copy is a third thing to drift.

    python tools/backfill_truth.py            # report only, writes nothing
    python tools/backfill_truth.py --write
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

# Measured spacing (checked 2026-09-08): the two closest bays are 9.40in
# apart and the two closest depot slots 6.14in. IN has to sit well inside
# half of each, and anything in the band between IN and OUT is refused
# rather than guessed -- the same rule the facing questions already use for
# headings within 30deg of a cardinal.
DEPOT_IN_IN = 3.0
DEPOT_OUT_IN = 6.0
BAY_IN_IN = 4.0
BAY_OUT_IN = 6.0


def _tables():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, root)
    try:
        import apriltag_localize as al
    except Exception as exc:                       # pragma: no cover
        raise SystemExit(
            f"cannot import apriltag_localize from {root}: {exc}")
    return al.WORKSTATION_WORLD_IN, al.DEPOT_SLOT_WORLD_IN


def classify(point: tuple[float, float], table: dict,
             near_in: float, far_out: float) -> tuple[str, object, float]:
    """(state, label, distance) where state is 'in', 'out' or 'unclear'."""
    label, pos = min(table.items(), key=lambda kv: math.dist(point, kv[1]))
    d = math.dist(point, pos)
    if d <= near_in:
        return "in", label, d
    if d >= far_out:
        return "out", None, d
    return "unclear", label, d


def occupancy(robots: dict, bays: dict, depots: dict) -> dict:
    """Occupancy facts for one sample.

    `*_unclear` lists exist so the question generator can refuse to ask about
    a sample where any robot sits in the ambiguous band, instead of inventing
    a definite answer for a picture that does not show one.
    """
    in_depot, at_bay, depot_unclear, bay_unclear = [], [], [], []
    bay_of: dict[str, int] = {}
    for name, r in sorted(robots.items()):
        p = (r["x_in"], r["y_in"])
        ds, _dl, _dd = classify(p, depots, DEPOT_IN_IN, DEPOT_OUT_IN)
        if ds == "in":
            in_depot.append(name)
        elif ds == "unclear":
            depot_unclear.append(name)
        bs, bl, _bd = classify(p, bays, BAY_IN_IN, BAY_OUT_IN)
        if bs == "in":
            at_bay.append(name)
            bay_of[name] = bl
        elif bs == "unclear":
            bay_unclear.append(name)
    return {
        "in_depot": in_depot,
        "in_depot_unclear": depot_unclear,
        "at_workstation": at_bay,
        "at_workstation_bay": bay_of,
        "at_workstation_unclear": bay_unclear,
    }


def _flag_dropouts(root: str, sids: list, write: bool) -> None:
    """Mark samples whose tracked robot set is a strict subset of its
    session's own -- a vision dropout, not a real scene.

    Found 2026-09-09: four consecutive samples across 15 seconds of a
    six-robot run reported 4, 1, 5 and 3 robots. The IMAGE still showed six.
    Truth and picture disagreeing is worse than a missing sample, because
    every question keys off the robot set -- counting, extremes,
    closest-pair and occupancy would all be scored against a fleet that is
    visibly there.

    The frames are kept. build_questions() simply generates nothing for a
    flagged sample, so the evidence survives and the scoring does not use it.
    """
    from collections import Counter
    groups: dict = {}
    for sid in sids:
        with open(os.path.join(root, sid, "truth.json"), encoding="utf-8") as fh:
            t = json.load(fh)
        groups.setdefault(t.get("label") or "(none)", []).append(
            (sid, frozenset(t.get("robots") or {})))

    flagged = []
    for label, members in groups.items():
        if len(members) < 3:
            continue
        modal, n_modal = Counter(s for _sid, s in members).most_common(1)[0]
        if n_modal < 2:
            continue
        for sid, robots in members:
            if robots < modal:            # strict subset: someone vanished
                flagged.append((sid, label, sorted(modal - robots)))

    for sid, label, missing in flagged:
        path = os.path.join(root, sid, "truth.json")
        with open(path, encoding="utf-8") as fh:
            t = json.load(fh)
        reason = ("vision dropout: "
                  + ", ".join(missing) + " not tracked but on the table")
        if t.get("suspect") != reason and write:
            t["suspect"] = reason
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(t, fh, indent=2)
                fh.write(chr(10))
    if flagged:
        print(f"{len(flagged)} sample(s) flagged as vision dropouts"
              + (" and excluded from scoring:" if write else " (use --write):"))
        for sid, label, missing in flagged:
            print(f"    {sid}  {label}  missing {', '.join(missing)}")
        print()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--samples-dir", default=None)
    ap.add_argument("--write", action="store_true",
                    help="without this, nothing is modified")
    args = ap.parse_args()

    bays, depots = _tables()
    d = args.samples_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vlm_samples")
    sids = sorted(s for s in os.listdir(d)
                  if os.path.isfile(os.path.join(d, s, "truth.json")))
    if not sids:
        raise SystemExit(f"no samples in {d}")

    from collections import Counter
    depot_sets: Counter = Counter()
    bay_sets: Counter = Counter()
    changed = 0
    for sid in sids:
        path = os.path.join(d, sid, "truth.json")
        with open(path, encoding="utf-8") as fh:
            truth = json.load(fh)
        occ = occupancy(truth["robots"], bays, depots)
        occ["suspect"] = None
        depot_sets[tuple(occ["in_depot"])] += 1
        bay_sets[tuple(occ["at_workstation"])] += 1
        if any(truth.get(k) != v for k, v in occ.items()):
            changed += 1
            if args.write:
                truth.update(occ)
                with open(path, "w", encoding="utf-8") as fh:
                    json.dump(truth, fh, indent=2)
                    fh.write(chr(10))

    _flag_dropouts(d, sids, args.write)

    print(f"{len(sids)} sample(s), {changed} would change"
          if not args.write else
          f"{len(sids)} sample(s), {changed} updated")
    print()
    # The point of printing these: a field with one distinct value across the
    # whole set cannot measure anything, and it is far cheaper to see that
    # here than after a 25-minute benchmark reports 100%.
    for label, counter in (("in_depot", depot_sets),
                           ("at_workstation", bay_sets)):
        print(f"{label}: {len(counter)} distinct value(s) across {len(sids)}"
              + ("   <-- INERT, every sample identical"
                 if len(counter) == 1 else ""))
        for value, n in counter.most_common():
            print(f"    {n:>3}x  {value if value else '(none)'}")
        print()

    _report_variance(d, sids)


def _report_variance(root: str, sids: list) -> None:
    """Does the captured set actually vary, field by field?

    Run this between capture batches, not after. Every inert question in the
    first 32-sample set -- count, closest, depot:which, extreme:south -- was
    only discovered after a 25-minute benchmark scored 100% on it and meant
    nothing. Placing robots by hand is slow enough that finding out mid-session
    is worth a lot more than finding out afterwards.
    """
    from collections import Counter
    counts: Counter = Counter()
    facing: Counter = Counter()
    closest: Counter = Counter()
    labels: Counter = Counter()
    extremes: dict = {}
    for sid in sids:
        with open(os.path.join(root, sid, "truth.json"), encoding="utf-8") as fh:
            t = json.load(fh)
        counts[t.get("robot_count")] += 1
        labels[t.get("label") or "(unlabelled)"] += 1
        for card in (t.get("facing") or {}).values():
            facing[card] += 1
        pair = (t.get("closest_pair") or {}).get("robots")
        closest[tuple(sorted(pair)) if pair else None] += 1
        for k, v in (t.get("extremes") or {}).items():
            extremes.setdefault(k, Counter())[v] += 1

    def verdict(c: Counter) -> str:
        if not c:
            return ""
        n = sum(c.values())
        top = c.most_common(1)[0][1]
        if len(c) == 1:
            return "   <-- INERT, one answer only"
        if top / n >= 0.8:
            return f"   <-- near-constant ({100 * top / n:.0f}%)"
        return ""

    print("=" * 66)
    print("VARIANCE CHECK -- a field with one value cannot measure anything")
    print("=" * 66)
    print(f"session labels : {dict(labels)}")
    print(f"robot_count    : {dict(sorted(counts.items()))}{verdict(counts)}")
    print(f"closest_pair   : {len(closest)} distinct{verdict(closest)}")
    for k in sorted(extremes):
        print(f"extreme:{k:<7}: {dict(extremes[k].most_common())}"
              f"{verdict(extremes[k])}")
    print()
    total = sum(facing.values()) or 1
    print("facing (the most skewed field in the first capture set --"
          " 142 south / 21 north / 15 east / 8 west):")
    for card in ("north", "south", "east", "west"):
        n = facing.get(card, 0)
        bar = "#" * int(40 * n / total)
        flag = "   <-- none captured" if n == 0 else ""
        print(f"    {card:<6}{n:>4}  {100 * n / total:>3.0f}%  {bar}{flag}")


if __name__ == "__main__":
    main()
