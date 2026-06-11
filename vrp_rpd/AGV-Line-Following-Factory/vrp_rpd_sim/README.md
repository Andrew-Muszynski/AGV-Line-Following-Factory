# VRP-RPD Simulator for Alvik-Factory

SimPy + matplotlib simulator for the **Vehicle Routing Problem with Resource-Constrained Pickup and Delivery** (Sodhi, Prasad & Saseendran, 2026), tailored to an 8×8 duct-tape grid with Arduino Alvik AGVs.

Use this to plan and preview routes **before** running the live demo.

## Install

```bash
pip install simpy matplotlib numpy pillow
```

## Quick start

```bash
# Generate a random 8x8 instance, solve with ALNS, animate
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42

# Save the instance + schedule for reproducibility
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42 \
    --save-instance instance.json --save-schedule schedule.json

# Replay a saved schedule
python -m vrp_rpd_sim.main --instance instance.json --schedule schedule.json

# Just the static plan (no animation)
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --static-only

# Show coordination effect: same instance at 1x, 2x, 5x processing times
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42 --proc-multiplier 1
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42 --proc-multiplier 2
python -m vrp_rpd_sim.main --agvs 3 --capacity 4 --seed 42 --proc-multiplier 5
```

## Architecture

| Module | Responsibility |
|---|---|
| `instance.py` | Grid layout, depot, demand, distance matrix |
| `schedule.py` | Per-AGV operation sequences (D/P) with validator |
| `solver.py` | ALNS-lite: greedy init + random/worst/critical-path destroy + greedy/regret-2 repair + SA acceptance |
| `simulator.py` | SimPy discrete-event engine with per-cell movement and stop-and-wait contention |
| `visualizer.py` | Matplotlib animation with processing status rings and AGV load indicators |
| `main.py` | CLI |

## Physical mapping

- **Node 0 = depot** (default: bottom-left corner at row 7, col 0 on an 8×8). All AGVs start here, queued on a tape branch.
- **Nodes 1..n = machine stickers** on grid intersections. `num_agvs × capacity` of these are chosen as customers (demand).
- **Travel time** between two nodes = Manhattan cells × `cell_traversal_time` + turns × `turn_penalty`. Measure `cell_traversal_time` empirically from one Alvik on a straight tape segment.
- **Processing time** per node sampled from `[proc_min, proc_max]`, multiplied by `proc_multiplier` to reproduce the paper's 1×/2×/5× variants.
- **Stop-and-wait contention**: if an AGV needs to enter a cell owned by another AGV, it waits until the cell is released. On the physical robots you'll replace this with the TOF-based passing maneuver.

## What the visualization shows

- **Grid**: 8×8 tape lattice with labeled depot and customer nodes
- **AGV markers**: colored circles with load indicator `[k]` showing carried resources
- **Ghost paths**: light-colored outline of each AGV's planned route
- **Processing rings**:
  - gold = resource currently processing
  - green = resource ready for pickup
  - gray = completed (picked up)
- **Title bar**: current sim time, final makespan, cross-agent coordination %

## Solver notes

The solver is a CPU-only ALNS adaptation of the paper's algorithm. On a 64-customer instance (8×8 grid with all non-depot cells as demand) it typically converges in 20–60 seconds. The paper's BRKGA second stage is omitted because, per Table 5 of the paper, its marginal contribution is small on instances <100 customers and negligible absent the GPU-accelerated ALNS it builds on.

If you need the full pipeline later, the `Schedule` class has JSON I/O so you can plug in an external solver.

## Integrating with Alvik-Factory

The saved `schedule.json` is designed to be consumed by your superfactory node. Each AGV's route is a list of `{node, op}` where `op ∈ {"D", "P"}`. Your existing line-following + red-sticker-stop logic handles the mechanics; the superfactory just needs to:

1. Issue the next `node` as a waypoint to each Alvik.
2. Track processing completion times and gate pickups on `T_drop[c] + p_c`.
3. Publish updates to `/agv_factory/queue_map` as each operation completes.
