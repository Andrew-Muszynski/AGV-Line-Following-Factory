# AGV Warehouse Testbed

Physical cobot warehouse extension of the VRP-RPD solver. A fleet of AGVs services workstations on a 10×10 grid warehouse using a two-layer pipeline: VRP-RPD heuristic routing followed by prioritized MAPF collision avoidance.

---

## What We Built

### Two-layer architecture

```
Layer 1 — VRP-RPD Search
  Construction heuristics (nearest-neighbor, max-regret, greedy-defer)
  → per-agent task assignments + completion times + priority order

Layer 2 — Prioritized MAPF
  A* path planning per agent, highest-priority agent first
  Priority = VRP completion time (most critical agent gets priority)
  → collision-free timed paths on the grid

Feedback loop
  If conflicts remain → increment seed → re-run Layer 1 → repeat
```

### Spur geometry

Each workstation lives **inside** a grid cell — off the transit grid. A vertical spur connects it to the transit corridor above:

```
  [transit node]  ← robots travel freely here
       |
  [spur entry]    ← robot turns off the main corridor
       |
  [workstation]   ← robot delivers or picks up resource
       |
  [spur entry]    ← robot exits back to transit corridor
       |
  [transit node]
```

This cleanly separates transit traffic from service traffic. Only the robot with a job at that workstation ever enters the spur. All other robots pass by on the transit grid.

### Virtual node ID scheme

To avoid ID collisions (especially with dense workstation placement), workstations and spur entries use virtual IDs outside the transit grid:

- Transit nodes: `0–99`
- Spur entry nodes: `100–(100+n-1)`
- Workstation nodes: `(100+n)–(100+2n-1)`

This means any workstation placement density works correctly regardless of adjacency between workstations.

---

## Grid Environment

- **10×10 transit grid** — 100 nodes, all freely navigable
- **Depot** — node 0, bottom-left corner
- **Workstations** — randomly placed in grid cells (excluding top row and right column to ensure valid cell geometry)
- **Travel times** — BFS shortest-path hop count + spur cost (6 hops per workstation visit each way)
- **Processing times** — from existing TSPlib dataset files

---

## Dataset Configuration

| Dataset  | Customers | Agents (m) | Capacity (k) |
|----------|-----------|------------|--------------|
| gr17     | 16        | 3          | 5            |
| gr21     | 20        | 3          | 5            |
| gr24     | 23        | 4          | 6            |
| gr48     | 47        | 4          | 6            |
| bays29   | 28        | 4          | 6            |
| berlin52 | 51        | 4          | 6            |
| eil51    | 50        | 4          | 6            |

Processing-time variants: `base`, `2x`, `5x`, `1R10`, `1R20`

---

## File Structure

```
agv_testbed/
├── grid_env.py          # Grid, virtual node IDs, spur geometry, distance matrix
├── instance_builder.py  # WarehouseGrid → VRPRPDInstance; MAPF waypoint expansion
├── vrp_solver.py        # Heuristic search, returns SolverResult
├── mapf_solver.py       # Prioritized A* MAPF with spur-aware graph
├── pipeline.py          # Iterative VRP→MAPF feedback loop
├── analyze.py           # Batch analysis across datasets and variants
├── visualize.py         # Pygame renderer — GIF export (headless server)
├── web_viewer.py        # Flask/MJPEG web viewer — stream to browser
├── datasets/            # Processing time files per dataset and variant
└── results/             # Saved pipeline JSON results
```

---

## How to Run

### Run the pipeline (single dataset)

```bash
cd agv_testbed
python3 pipeline.py
```

### Batch analysis across all datasets

```bash
python3 analyze.py --datasets gr17 gr21 gr24 gr48 bays29 berlin52 eil51 \
                   --variants base 2x 5x \
                   --output results_spur.json
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `--datasets` | all 7 | One or more dataset names |
| `--variants` | base | Processing-time variants |
| `--seed` | 42 | RNG seed for workstation placement |
| `--output` | — | Save results to JSON file |
| `--verbose` | — | Print per-agent breakdown |

### Export animation as GIF (headless)

```bash
python3 visualize.py --export --dataset bays29 --variant base --output bays29.gif

# Control speed and file size
python3 visualize.py --export --dataset bays29 --variant base \
                     --fps 10 --steps-per-frame 2 --output bays29_fast.gif
```

Copy to local machine:

```bash
scp <server>:/home/romesh/vrp_rpd_github/agv_testbed/bays29.gif .
```

---

## Web Viewer

Streams a live animation to your browser via MJPEG. Works on headless servers — no display required. Must be on VPN to access.

### Run and solve fresh

```bash
python3 web_viewer.py --dataset bays29 --variant base --port 5001
```

### Run, solve, and save result

```bash
python3 web_viewer.py --dataset bays29 --variant base --port 5001 \
                      --save results/bays29_base.json
```

### Load a previously saved result (skips re-solving)

```bash
python3 web_viewer.py --load results/bays29_base.json --port 5001
```

Then open in your browser (VPN required):

```
http://<server-ip>:5001
```

Browser controls: **Pause / Resume** — **Restart** — **Speed +** — **Speed −**

### Web viewer options

| Flag | Default | Description |
|------|---------|-------------|
| `--dataset` | bays29 | Dataset to solve |
| `--variant` | base | Processing-time variant |
| `--seed` | 42 | RNG seed |
| `--port` | 5000 | HTTP port |
| `--speed` | 4.0 | Initial playback speed (sim steps/sec) |
| `--save FILE` | — | Save pipeline result to JSON after solving |
| `--load FILE` | — | Load saved result, skip solving |

---

## Visualization Legend

| Colour | Meaning |
|--------|---------|
| Yellow circle | Workstation — no resource yet |
| Red circle | Workstation — resource dropped, processing |
| Green circle | Workstation — processing done, awaiting pickup |
| Dark circle | Workstation — complete |
| Black square (D) | Depot |
| Coloured dot | AGV robot |
| Vertical line | Spur connecting transit grid to workstation |

---

## Results (base variant, seed=42)

- 100% MAPF convergence across all 7 datasets
- Average ~3 MAPF wait steps inserted per run
- Total wall time ~2–3 seconds for all 7 datasets

---

## Dependencies

```bash
pip install pygame pillow flask numpy
```

The `vrp_rpd` package must be accessible (handled automatically via `sys.path.insert` in each script).
