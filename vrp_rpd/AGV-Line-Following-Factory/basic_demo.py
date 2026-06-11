"""
Programmatic example: build an instance, solve it, simulate, visualize.

Run with:
  python examples/basic_demo.py
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from vrp_rpd_sim.instance import Instance
from vrp_rpd_sim.solver import solve_alns, ALNSConfig
from vrp_rpd_sim.simulator import GridSimulator, SimConfig
from vrp_rpd_sim.visualizer import Visualizer


def main():
    # Build an instance: 8x8 grid, 3 AGVs with capacity 4 each
    inst = Instance.random_instance(
        grid_rows=8,
        grid_cols=8,
        num_agvs=3,
        capacity=4,
        cell_traversal_time=2.0,     # 2s per grid cell
        turn_penalty=0.5,             # 0.5s per turn
        processing_time_range=(5.0, 30.0),
        processing_time_multiplier=2.0,  # 2x variant for visible coordination
        depot_row=7,
        depot_col=0,
        seed=42,
    )
    print(inst.summary())
    print(f"Demand: {inst.demand}")

    # Solve with ALNS
    cfg = ALNSConfig(max_iter=1000, time_limit_sec=30.0, verbose=True)
    result = solve_alns(inst, cfg=cfg)
    schedule = result.schedule

    print(f"\nALNS makespan: {result.makespan:.2f}s")
    print(f"Cross-agent coordination: {schedule.cross_agent_pct(inst):.1f}%")
    print("\nSchedule:")
    print(schedule.pretty(inst))

    # Validate
    errors = schedule.validate(inst)
    if errors:
        print(f"INVALID: {errors}")
        return
    print("\nSchedule is valid.")

    # Simulate with stop-and-wait contention
    sim = GridSimulator(inst, schedule, cfg=SimConfig(enable_contention=True))
    sim_result = sim.run()
    print(f"\nSimulated makespan: {sim_result.makespan:.2f}s")
    print("(Difference from planned = time lost to cell contention)")
    for v in range(inst.num_agvs):
        print(f"  AGV {v}: return={sim_result.agv_return_times[v]:.2f}s, "
              f"wait={sim_result.waits[v]:.2f}s")

    # Visualize
    viz = Visualizer(inst, schedule, sim_result, playback_speed=4.0)
    viz.animate(show=True)


if __name__ == "__main__":
    main()
