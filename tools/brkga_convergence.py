#!/usr/bin/env python3
"""Find where the BRKGA actually converges, then size the stopping rule.

WHY THIS EXISTS
---------------
`VRPRPDSolver` runs every generation it is given and stops because the counter
ran out, not because the search finished. `restart_stagnation` is accepted,
assigned to `self`, and never read anywhere -- a dead parameter. The operator's
observation that "there is often a clear winner and the makespan rarely changes
from one generation to the next" is therefore exactly what you would expect.

The cost is not academic. The solver's own estimate is ~173 s per 100
generations, so a 500-generation solve is ~14 min, and a paired campaign of
30 instances x 2 arms x 5 seeds is ~72 hours of compute. If the incumbent
settles at generation 80, most of that is spent re-confirming a result already
in hand.

WHAT IT DOES
------------
Runs the solver with fine-grained history (`history_every`, default 10) and
early stopping disabled, on several instances, and records the incumbent
against generation. Then reports, per instance:

    gen_best      the generation that first reached the final makespan
    gen_99        the generation that first came within 1% of it
    wasted        generations run after gen_best, i.e. the saving available

A stopping rule is only safe if it would not have changed the answer on any
instance tested, so the recommended `stagnation_limit` is taken from the
WORST case observed, with a margin -- not the mean.

    python3 tools/brkga_convergence.py --instances 3 --generations 500
    python3 tools/brkga_convergence.py --replay runs/brkga_convergence.csv
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VRP = ROOT / "external" / "vrp_rpd"
OUT_CSV = ROOT / "runs" / "brkga_convergence.csv"
OUT_PNG = ROOT / "figures" / "fig_brkga_convergence.png"


def _solver_imports():
    """Import the raw solver, not the `run_brkga` wrapper.

    `run_brkga` returns a `SolverResult`, which carries the plan but not the
    per-generation trace -- `solve()`'s dict is where the histories live.
    """
    sys.path.insert(0, str(VRP))
    from agv_testbed.grid_env import load_physical_grid                     # noqa: E402
    from vrp_rpd.agv_testbed.instance_builder import build_vrp_instance     # noqa: E402
    from vrp_rpd.solver import VRPRPDSolver                                 # noqa: E402
    return load_physical_grid, build_vrp_instance, VRPRPDSolver


def collect(n_instances: int, generations: int, agents: int, resources: int,
            history_every: int, seed0: int) -> list[dict]:
    import numpy as np
    load_physical_grid, build_vrp_instance, VRPRPDSolver = _solver_imports()
    rows = []
    for k in range(n_instances):
        seed = seed0 + k
        rng = np.random.default_rng(seed)
        grid = load_physical_grid(rng=rng)
        instance = build_vrp_instance(grid, agents, resources)
        print(f"\n=== instance {k + 1}/{n_instances} (seed {seed}) ===", flush=True)
        t0 = time.time()
        out = VRPRPDSolver(
            instance=instance,
            total_generations=generations,
            gens_per_cycle=min(100, generations),
            use_gp=False,                 # keep the run reproducible and simple
            stagnation_limit=0,           # measure the FULL curve, never cut it
            history_every=history_every,
        ).solve()
        wall = time.time() - t0

        # Merge the per-worker traces into one best-so-far curve: at each
        # sampled generation the fleet's incumbent is the best any island held.
        merged: dict[int, float] = {}
        for _worker, hist in (out.get("histories") or {}).items():
            for gen, fit in hist:
                g = int(gen)
                if fit is not None and (g not in merged or fit < merged[g]):
                    merged[g] = float(fit)
        if not merged:
            print("  no history returned -- check that solve() carries 'histories'")
            continue
        best = float("inf")
        for g in sorted(merged):                      # enforce monotonicity
            best = min(best, merged[g])
            rows.append({"seed": seed, "gen": g, "fitness": best,
                         "wall_s": round(wall, 1)})
        print(f"  {len(merged)} samples, final {best:.2f}, {wall:.1f}s wall")
    return rows


def analyse(rows: list[dict]) -> None:
    by_seed: dict[int, list[tuple[int, float]]] = {}
    for r in rows:
        by_seed.setdefault(int(r["seed"]), []).append((int(r["gen"]), float(r["fitness"])))

    print(f"\n{'seed':>6}{'final':>10}{'gen_best':>10}{'gen_99':>9}{'last gen':>10}{'wasted':>9}")
    gen_bests, gaps = [], []
    for seed, pts in sorted(by_seed.items()):
        pts.sort()
        final = min(f for _g, f in pts)
        last = max(g for g, _f in pts)
        gen_best = next(g for g, f in pts if f <= final * 1.0000001)
        thresh = final * 1.01
        gen_99 = next(g for g, f in pts if f <= thresh)
        gen_bests.append(gen_best)
        gaps.append(last - gen_best)
        print(f"{seed:>6}{final:>10.2f}{gen_best:>10}{gen_99:>9}{last:>10}{last - gen_best:>9}")

    if not gen_bests:
        return
    worst = max(gen_bests)
    print(f"\n  last improvement, worst case : generation {worst}")
    print(f"  generations run after it     : {statistics.mean(gaps):.0f} mean, {min(gaps)} min")

    # The rule must not change the answer on the worst instance seen. Round the
    # worst observed last-improvement up and double it as margin, because this
    # is a stochastic search and the sample is small.
    rec = int(round(worst * 2 / 50.0)) * 50 or 50
    print(f"\n  RECOMMENDED stagnation_limit : {rec}"
          f"   (2x the worst last-improvement, rounded to 50)")
    print(f"  expected saving at 500 gens  : ~{max(0, 500 - (worst + rec)) / 500:.0%}")
    print("\n  Apply identically to BOTH arms, and fix it BEFORE the campaign")
    print("  starts -- changing it mid-campaign confounds the comparison.")


def plot(rows: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    by_seed: dict[int, list[tuple[int, float]]] = {}
    for r in rows:
        by_seed.setdefault(int(r["seed"]), []).append((int(r["gen"]), float(r["fitness"])))

    fig, ax = plt.subplots(figsize=(10.5, 5.6), dpi=150)
    fig.patch.set_facecolor("white")
    for seed, pts in sorted(by_seed.items()):
        pts.sort()
        final = min(f for _g, f in pts)
        ax.plot([g for g, _ in pts], [f / final for _, f in pts],
                lw=1.6, label=f"seed {seed}")
    ax.axhline(1.01, color="#b3471f", ls="--", lw=1.1)
    ax.text(ax.get_xlim()[1], 1.0105, " within 1% of final", color="#b3471f",
            fontsize=9, va="bottom", ha="right")
    ax.set_xlabel("generation", fontsize=11)
    ax.set_ylabel("incumbent makespan / final", fontsize=11)
    ax.set_title("BRKGA convergence", fontsize=14, fontweight="bold",
                 color="#1b2733", loc="left")
    ax.legend(frameon=False, fontsize=9)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(color="#e6eaee", lw=0.8)
    ax.set_axisbelow(True)
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(OUT_PNG, facecolor="white")
    print(f"  wrote {OUT_PNG}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instances", type=int, default=3)
    ap.add_argument("--generations", type=int, default=500)
    ap.add_argument("--agents", type=int, default=6)
    ap.add_argument("--resources", type=int, default=2)
    ap.add_argument("--history-every", type=int, default=10)
    ap.add_argument("--seed0", type=int, default=101)
    ap.add_argument("--replay", type=Path, help="analyse an existing CSV instead of solving")
    ap.add_argument("--validate", type=int, metavar="LIMIT",
                    help="A/B one instance with and without early stopping at LIMIT, "
                         "and report whether the answer changed")
    args = ap.parse_args()

    if args.validate:
        # Must run from a file, not stdin: Windows multiprocessing uses spawn,
        # which re-imports __main__ and silently falls back to the heuristic
        # if it cannot.
        import numpy as np
        load_physical_grid, build_vrp_instance, VRPRPDSolver = _solver_imports()
        res = []
        for limit in (0, args.validate):
            rng = np.random.default_rng(args.seed0)
            inst = build_vrp_instance(load_physical_grid(rng=rng), args.agents, args.resources)
            t0 = time.time()
            out = VRPRPDSolver(instance=inst, total_generations=args.generations,
                               gens_per_cycle=min(100, args.generations), use_gp=False,
                               stagnation_limit=limit,
                               history_every=args.history_every).solve()
            res.append((limit, out["makespan"], out.get("source"), time.time() - t0))
        print("\n=== early-stopping validation ===")
        for limit, mk, src, wall in res:
            tag = "full run" if limit == 0 else f"stagnation_limit={limit}"
            print(f"  {tag:24} makespan {mk:8.2f}  [{src}]  wall {wall:6.1f}s")
        if any(r[2] == "heuristic" for r in res):
            print("  WARNING: a run fell back to the heuristic -- the GA workers did not"
                  " start, so this comparison is meaningless.")
        else:
            print(f"  identical makespan : {abs(res[0][1] - res[1][1]) < 1e-6}")
            print(f"  wall-clock saved   : {1 - res[1][3] / res[0][3]:.0%}")
        return

    if args.replay:
        with open(args.replay, newline="") as f:
            rows = list(csv.DictReader(f))
    else:
        rows = collect(args.instances, args.generations, args.agents,
                       args.resources, args.history_every, args.seed0)
        if not rows:
            sys.exit("no history collected")
        OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"\n  wrote {OUT_CSV}  ({len(rows)} samples)")

    analyse(rows)
    try:
        plot(rows)
    except Exception as e:                      # plotting is a convenience
        print(f"  (plot skipped: {e})")


if __name__ == "__main__":
    main()
