"""
Generate AGV testbed datasets in agv_testbed/datasets/.

Processing times are sampled from the grid's own travel-time range [1, 18]
(BFS hops on a 10x10 grid), independent of the original TSPlib edge weights.
This matches the warehouse physical model: work duration depends on the job,
not on how far the robot traveled.

Variants:
  base  : U[1, 18]            — uniform base workload
  2x    : U[2, 36]            — 2x multiplier
  5x    : U[5, 90]            — 5x multiplier
  1R10  : U[1,18] * randint(1,10)  — heterogeneous, 10 instances
  1R20  : U[1,18] * randint(1,20)  — heterogeneous, 10 instances

Usage:
  python3 agv_testbed/generate_datasets.py
  python3 agv_testbed/generate_datasets.py --seed 42 --output agv_testbed/datasets
"""

from __future__ import annotations
import json
import argparse
import numpy as np
from pathlib import Path

# Grid constants
D_MIN = 1    # minimum BFS hop distance on 10x10 grid
D_MAX = 18   # maximum BFS hop distance on 10x10 grid (corner to corner)

DATASETS = {
    "gr17":     16,
    "gr21":     20,
    "gr24":     23,
    "gr48":     47,
    "bays29":   28,
    "berlin52": 51,
    "eil51":    50,
}

N_STOCHASTIC = 10   # number of instances for 1R10 and 1R20


def sample_base(n: int, rng: np.random.Generator) -> list[int]:
    return rng.integers(D_MIN, D_MAX + 1, size=n).tolist()


def generate_dataset(name: str, n: int, out_dir: Path, base_seed: int):
    ds_dir = out_dir / name
    rng = np.random.default_rng(base_seed)

    # ── base ─────────────────────────────────────────────────────────────────
    base_times = sample_base(n, rng)
    d = ds_dir / "base"
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "job_times.json", "w") as f:
        json.dump({
            "name": name, "n_customers": n,
            "variant": "base", "d_min": D_MIN, "d_max": D_MAX,
            "seed": base_seed,
            "processing_times": base_times,
        }, f, indent=2)

    # ── 2x ───────────────────────────────────────────────────────────────────
    d = ds_dir / "2x"
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "job_times.json", "w") as f:
        json.dump({
            "name": name, "n_customers": n,
            "variant": "2x", "multiplier": 2,
            "seed": base_seed,
            "job_times": [t * 2 for t in base_times],
        }, f, indent=2)

    # ── 5x ───────────────────────────────────────────────────────────────────
    d = ds_dir / "5x"
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "job_times.json", "w") as f:
        json.dump({
            "name": name, "n_customers": n,
            "variant": "5x", "multiplier": 5,
            "seed": base_seed,
            "job_times": [t * 5 for t in base_times],
        }, f, indent=2)

    # ── 1R10 — 10 instances ──────────────────────────────────────────────────
    d = ds_dir / "1R10"
    d.mkdir(parents=True, exist_ok=True)
    for i in range(1, N_STOCHASTIC + 1):
        inst_rng = np.random.default_rng(base_seed + i)
        base_i = sample_base(n, inst_rng)
        mult   = inst_rng.integers(1, 11, size=n).tolist()
        times  = [b * m for b, m in zip(base_i, mult)]
        with open(d / f"job_times_{i}.json", "w") as f:
            json.dump({
                "name": name, "n_customers": n,
                "variant": "1R10", "instance": i,
                "seed": base_seed + i,
                "multipliers": mult,
                "job_times": times,
            }, f, indent=2)

    # ── 1R20 — 10 instances ──────────────────────────────────────────────────
    d = ds_dir / "1R20"
    d.mkdir(parents=True, exist_ok=True)
    for i in range(1, N_STOCHASTIC + 1):
        inst_rng = np.random.default_rng(base_seed + 100 + i)
        base_i = sample_base(n, inst_rng)
        mult   = inst_rng.integers(1, 21, size=n).tolist()
        times  = [b * m for b, m in zip(base_i, mult)]
        with open(d / f"job_times_{i}.json", "w") as f:
            json.dump({
                "name": name, "n_customers": n,
                "variant": "1R20", "instance": i,
                "seed": base_seed + 100 + i,
                "multipliers": mult,
                "job_times": times,
            }, f, indent=2)

    print(f"  {name:10s}  n={n:2d}  "
          f"base=[{min(base_times)},{max(base_times)}]  "
          f"2x=[{min(base_times)*2},{max(base_times)*2}]  "
          f"5x=[{min(base_times)*5},{max(base_times)*5}]")


def main():
    parser = argparse.ArgumentParser(description="Generate AGV testbed datasets")
    parser.add_argument("--seed", type=int, default=42,
                        help="Base RNG seed (default: 42)")
    parser.add_argument("--output", default=None,
                        help="Output directory (default: agv_testbed/datasets next to this script)")
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    out_dir = Path(args.output) if args.output else script_dir / "datasets"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Output directory : {out_dir.resolve()}")
    print(f"Grid range       : d_min={D_MIN}, d_max={D_MAX}")
    print(f"Base seed        : {args.seed}")
    print(f"Stochastic inst  : {N_STOCHASTIC} each for 1R10 and 1R20\n")
    print(f"{'Dataset':12s}  {'n':>4}  Processing time ranges")
    print("-" * 60)

    for name, n in DATASETS.items():
        generate_dataset(name, n, out_dir, args.seed)

    total = len(DATASETS) * (3 + 2 * N_STOCHASTIC)
    print(f"\nDone. {total} job_times files written to {out_dir.resolve()}")


if __name__ == "__main__":
    main()
