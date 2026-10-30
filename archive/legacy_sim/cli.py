"""
Command-line entry point for the VRP-RPD simulator.

Typical usage:
  # Generate a random instance, solve it, and visualize
  python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42

  # Run on a specific instance file
  python -m vrp_rpd_sim.main --instance my_instance.json --solve

  # Skip solving and use a saved schedule
  python -m vrp_rpd_sim.main --instance inst.json --schedule sched.json

  # Just show the static plan (no animation)
  python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --static-only

  # Save the animation as a GIF instead of showing it
  python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --save-gif run.gif --no-show
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from instance import Instance, load_positions_xlsx, load_positions_csv
from schedule import Schedule
from solver import solve_alns, ALNSConfig, evaluate_makespan, build_initial
from simulator import GridSimulator, SimConfig
from visualizer import Visualizer


def main():
    p = argparse.ArgumentParser(
        description="VRP-RPD grid simulator for the Alvik-Factory demo"
    )
    # Instance configuration
    p.add_argument("--instance", type=str, help="Path to instance JSON file")
    p.add_argument("--rows", type=int, default=8)
    p.add_argument("--cols", type=int, default=8)
    p.add_argument("--agvs", type=int, default=3)
    p.add_argument("--capacity", type=int, default=4)
    p.add_argument("--cell-time", type=float, default=2.0,
                   help="Seconds to traverse one grid cell")
    p.add_argument("--turn-penalty", type=float, default=0.5)
    p.add_argument("--load-time", type=float, default=0.0,
                   help="Seconds per AGV in the staging queue (staggered launch)")
    p.add_argument("--proc-min", type=float, default=5.0)
    p.add_argument("--proc-max", type=float, default=30.0)
    p.add_argument("--proc-multiplier", type=float, default=1.0,
                   help="Multiplier for processing times (1x, 2x, 5x, ...)")
    p.add_argument("--depot-row", type=int, default=7)
    p.add_argument("--depot-col", type=int, default=0)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--dist-matrix", type=str, default=None,
                   help="Path to physical position matrix (.xlsx or .csv). "
                        "Each cell holds 'y_coord, x_coord' relative to the depot. "
                        "Row 0 in the file = depot row (y=0). "
                        "Overrides uniform cell_traversal_time with measured distances.")

    # Solver
    p.add_argument("--schedule", type=str, help="Path to pre-computed schedule JSON")
    p.add_argument("--solve", action="store_true",
                   help="Force running the solver even if a schedule is provided")
    p.add_argument("--iters", type=int, default=1500, help="ALNS max iterations")
    p.add_argument("--time-limit", type=float, default=60.0,
                   help="ALNS wall-clock time limit in seconds")
    p.add_argument("--greedy-only", action="store_true",
                   help="Skip ALNS; just use the greedy initial solution")
    p.add_argument("--save-instance", type=str,
                   help="Save the generated instance to this path")
    p.add_argument("--save-schedule", type=str,
                   help="Save the solved schedule to this path")

    # Simulation
    p.add_argument("--no-contention", action="store_true",
                   help="Disable cell contention (AGVs pass through each other)")
    p.add_argument("--stagger", type=float, default=0.0,
                   help="Seconds between successive AGVs leaving depot queue")

    # Visualization
    p.add_argument("--static-only", action="store_true",
                   help="Show static route overview; skip animation")
    p.add_argument("--no-show", action="store_true",
                   help="Don't display the figure (useful with --save-gif)")
    p.add_argument("--save-gif", type=str, help="Save animation to this GIF path")
    p.add_argument("--save-plan", type=str, help="Save static plan to this PNG path")
    p.add_argument("--speed", type=float, default=4.0,
                   help="Playback speed multiplier")
    p.add_argument("--fps", type=int, default=30)

    args = p.parse_args()

    # -------- Load or build instance --------
    if args.instance:
        inst = Instance.load(args.instance)
        print(f"[load] {args.instance}")
    else:
        inst = Instance.random_instance(
            grid_rows=args.rows,
            grid_cols=args.cols,
            num_agvs=args.agvs,
            capacity=args.capacity,
            cell_traversal_time=args.cell_time,
            turn_penalty=args.turn_penalty,
            load_time=args.load_time,
            processing_time_range=(args.proc_min, args.proc_max),
            processing_time_multiplier=args.proc_multiplier,
            depot_row=args.depot_row,
            depot_col=args.depot_col,
            seed=args.seed,
        )
    # -------- Load physical distance matrix (optional) --------
    if args.dist_matrix:
        dist_path = Path(args.dist_matrix)
        suffix = dist_path.suffix.lower()
        if suffix == ".xlsx":
            positions = load_positions_xlsx(dist_path)
        elif suffix in (".csv", ".tsv", ".txt"):
            positions = load_positions_csv(dist_path)
        else:
            print(f"[dist] Unknown extension '{suffix}', trying CSV parser")
            positions = load_positions_csv(dist_path)
        inst.set_node_positions(positions)
        depot_pos = positions[inst.depot_node]
        print(f"[dist] Loaded position matrix from {args.dist_matrix} "
              f"({len(positions)} nodes, depot at {depot_pos})")

    print(inst.summary())

    if args.save_instance:
        inst.save(args.save_instance)
        print(f"[save] instance -> {args.save_instance}")

    # -------- Load or solve schedule --------
    schedule = None
    if args.schedule and not args.solve:
        schedule = Schedule.load(args.schedule)
        print(f"[load] schedule {args.schedule}")
    else:
        if args.greedy_only:
            import random as _r
            rng = _r.Random(args.seed)
            schedule = build_initial(inst, rng)
            mk = evaluate_makespan(schedule, inst)
            print(f"[greedy] initial makespan = {mk:.2f}s")
        else:
            print(f"[alns] solving ({args.iters} iters, {args.time_limit}s limit)...")
            cfg = ALNSConfig(
                max_iter=args.iters,
                time_limit_sec=args.time_limit,
                verbose=True,
            )
            result = solve_alns(inst, cfg=cfg)
            schedule = result.schedule
            print(f"[alns] done. makespan = {result.makespan:.2f}s "
                  f"in {result.elapsed_sec:.1f}s ({result.iterations} iters)")

    if args.save_schedule:
        schedule.save(args.save_schedule)
        print(f"[save] schedule -> {args.save_schedule}")

    # Validate
    errors = schedule.validate(inst)
    if errors:
        print("[ERROR] Schedule is invalid:")
        for e in errors:
            print(f"  - {e}")
        sys.exit(1)

    print("\n[schedule]")
    print(schedule.pretty(inst))
    print(f"\ncross-agent %: {schedule.cross_agent_pct(inst):.1f}")

    # -------- Simulate --------
    sim_cfg = SimConfig(
        enable_contention=not args.no_contention,
        agv_launch_stagger=args.stagger,
    )
    sim = GridSimulator(inst, schedule, cfg=sim_cfg)
    result = sim.run()
    print(f"\n[simulation]")
    print(f"  simulated makespan: {result.makespan:.2f}s")
    for v in range(inst.num_agvs):
        wait = result.waits.get(v, 0.0)
        print(f"  AGV {v}: return={result.agv_return_times[v]:7.2f}s, "
              f"total wait={wait:6.2f}s")

    # -------- Visualize --------
    viz = Visualizer(
        inst, schedule, result,
        fps=args.fps, playback_speed=args.speed
    )
    show = not args.no_show

    if args.save_plan or args.static_only:
        viz.static_overview(save_path=args.save_plan, show=(show and args.static_only))

    if not args.static_only:
        viz.animate(save_path=args.save_gif, show=show)


if __name__ == "__main__":
    main()
