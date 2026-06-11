# VRP-RPD → Physical AGV Bridge

Converts the `agv_testbed` simulator output into dispatch commands for the physical Alvik AGV fleet.

## What it does

The simulator (`vrp_rpd/agv_testbed`) solves VRP-RPD on an 8×8 grid and produces a MAPF-validated schedule: for each robot, an ordered list of workstations to visit (dropoff `D` or pickup `P`) with per-workstation processing times.

This bridge translates that schedule into the JSON format the factory node (`BaseAGV_v4/agv_factory_node.py`) understands, and enforces VRP-RPD pickup gating on the physical robots: a pickup at a workstation is held until the resource has finished processing.

```
agv_testbed pipeline  →  [bridge]  →  factory node  →  Alvik robots
  (gr21, 8x8, MAPF)        (this)       (ROS2)          (F/L/R moves)
```

---

## Prerequisites

**Simulator side (for generating the schedule):**
```bash
pip install simpy numpy
```
The `vrp_rpd` directory must be accessible. The bridge adds it to `sys.path` automatically based on its own location — no install needed.

**Robot side (for dispatching):**
- ROS2 (Humble or later) with `rclpy`
- Factory node running: `ros2 run agv_factory agv_factory_node`
- micro-ROS agent running for the Alvik boards
- Physical 8×8 tape grid with workstation stickers at the positions listed in the dry-run output

---

## Step 1 — Run the simulator and save the schedule

From the `vrp_rpd/` directory:

```bash
PYTHONPATH=/home/romesh/vrp_rpd_github/vrp_rpd python3 agv_testbed/web_viewer.py \
    --dataset gr21 --variant base --seed 42 \
    --save gr21_base_42.json
```

This produces `gr21_base_42.json` containing the VRP-RPD solution, MAPF-validated paths, and per-workstation processing times.

To skip the web viewer and just save the JSON:
```bash
PYTHONPATH=/home/romesh/vrp_rpd_github/vrp_rpd python3 agv_testbed/pipeline.py \
    --dataset gr21 --variant base --seed 42 \
    --save gr21_base_42.json
```

---

## Step 2 — Inspect the dispatch payload (dry run)

From the `bridge/` directory:

```bash
python3 testbed_to_factory.py --input /path/to/gr21_base_42.json --dry-run
```

Example output:
```
=== Factory Dispatch Payload ===
  Depot         : [7, 0]
  node_rcs size : 104 entries
  processing_times: 20 workstations

  AGV 1: 14 stops
    Dropoffs: [87, 90, 91, 94, 93, 95, 99]
    Pickups : [93, 95, 99, 91, 84, 87, 97]
  ...

  Workstation → physical grid intersection (factory row, col):
    node  84  →  [6, 3]  proc=2.0s
    node  87  →  [5, 1]  proc=8.0s
    ...
```

**Use this output to place your workstation stickers on the physical grid.** The `(factory row, col)` column tells you which tape intersection each workstation occupies. Factory row 0 = top row, row 7 = bottom row (where the depot is).

---

## Step 3 — Save the payload to a file

```bash
python3 testbed_to_factory.py \
    --input /path/to/gr21_base_42.json \
    --output dispatch.json
```

This saves the translated payload. You can inspect it, commit it alongside your experiment, or load it into another tool.

---

## Step 4 — Dispatch to the physical robots

With the factory node and micro-ROS agent already running:

```bash
python3 testbed_to_factory.py \
    --input /path/to/gr21_base_42.json \
    --dispatch
```

You can also combine `--output` and `--dispatch` in one call:
```bash
python3 testbed_to_factory.py \
    --input /path/to/gr21_base_42.json \
    --output dispatch.json \
    --dispatch
```

---

## How the physical grid maps to the simulation

The simulator uses spur geometry: each workstation lives inside a cell, reached via a short tape branch off the main grid. In the physical 8×8 grid, workstations are placed directly at grid intersections (the spur transit node — the intersection one row above the simulated cell). No spur tape is needed on the physical floor.

**Grid coordinates:**

| System | Row 0 | Row 7 | Depot |
|---|---|---|---|
| agv_testbed | bottom-left | top | node 0 = `(0,0)` |
| factory node | top-left | bottom | `[7, 0]` |

The bridge handles this flip automatically.

---

## VRP-RPD pickup gating

The factory node extension (`agv_factory_node.py`) enforces the VRP-RPD constraint on the physical floor:

- **Dropoff (`D`)**: robot arrives at workstation, factory node records `ready_at = wall_time + processing_time`. Robot leaves immediately.
- **Pickup (`P`)**: robot arrives at workstation, factory node waits until `wall_time >= ready_at`. If the resource is already done, the robot picks up immediately. If another robot dropped it off and it's still processing, the pickup robot waits at the workstation.

This mirrors the simulation semantics exactly: any robot can pick up from any workstation, but only after processing completes.

---

## Files

| File | Purpose |
|---|---|
| `path_translator.py` | Core translation logic — builds `node_rcs` table and routes |
| `testbed_to_factory.py` | CLI — dry-run, save, or dispatch |
| `../BaseAGV_v4/agv_factory_node.py` | Extended with `processing_times` and per-node D/P gating |

---

## Changing the dataset or variant

The bridge is dataset-agnostic — it reads whatever JSON the simulator saved. To run a different experiment:

```bash
# 2x processing times
python3 testbed_to_factory.py --input gr21_2x_42.json --dry-run

# Different seed
python3 testbed_to_factory.py --input gr21_base_99.json --dispatch
```

The `--seed` only affects workstation placement on the grid. Different seeds give different physical layouts — re-run `--dry-run` to get the updated sticker positions.
