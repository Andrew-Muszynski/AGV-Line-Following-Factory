# VRP-RPD: Vehicle Routing Problem with Resource-Constrained Pickup and Delivery

Research codebase for the paper *"VRP-RPD"* accepted at GECCO '26 (Genetic and Evolutionary Computation Conference, July 2026, San Jose, Costa Rica).

**Authors:** Romesh Prasad & Prof. Manbir Sodhi — University of Rhode Island, Department of Mechanical, Industrial and Systems Engineering.

---

## The Problem

A fleet of **m vehicles** (each carrying **k identical resources**) services **n customer locations**. Each customer requires a resource to be dropped off, which then processes autonomously for some time, after which **any vehicle** (not necessarily the one that delivered it) can pick it up. The objective is to minimize **makespan** — the time until all pickups are complete.

The key differentiator from classical pickup-and-delivery problems: **cross-vehicle resource handoff**. The vehicle that drops off a resource is not required to retrieve it. This creates a joint optimization across dropoffs and pickups that is NP-hard and intractable for exact solvers beyond ~16 customers.

---

## Repository Structure

```
vrp_rpd_github/
│
├── vrp_rpd/                  # Core VRP-RPD solver (GECCO '26)
│   ├── vrp_rpd/              # Python package
│   │   ├── solver.py         # BRKGA+ALNS main solver
│   │   ├── decoder.py        # Simulation decoder (discrete-event, feasibility-guaranteed)
│   │   ├── heuristics.py     # Construction heuristics (NN, max-regret, greedy-defer)
│   │   ├── alns.py           # ALNS destroy-repair operators
│   │   ├── islands.py        # Parallel island model (GPU, L40S)
│   │   ├── models.py         # Problem instance and chromosome representation
│   │   └── utils.py          # Simulation utilities
│   ├── datasets/             # TSPlib benchmark instances with processing times
│   ├── main.py               # Run BRKGA+ALNS solver
│   ├── main_paper.tex        # GECCO '26 paper source
│   └── statistical_analysis.py
│
├── agv_testbed/              # AGV warehouse physical testbed extension
│   ├── grid_env.py           # 10×10 grid, virtual node IDs, spur geometry
│   ├── instance_builder.py   # Grid → VRPRPDInstance adapter
│   ├── vrp_solver.py         # Heuristic routing layer
│   ├── mapf_solver.py        # Prioritized A* MAPF collision avoidance
│   ├── pipeline.py           # Two-layer VRP→MAPF iterative pipeline
│   ├── analyze.py            # Batch analysis script
│   ├── visualize.py          # GIF export (headless server)
│   ├── web_viewer.py         # Flask/MJPEG browser-based live viewer
│   ├── datasets/             # Processing time files per dataset/variant
│   ├── results/              # Saved pipeline results (JSON)
│   └── README.md             # Detailed AGV testbed documentation
│
├── main_paper.tex            # Paper source (top-level copy)
├── datasets/                 # Shared benchmark datasets
├── results/                  # Solver output results
└── paper_results/            # Results used in the paper
```

---

## Part 1 — VRP-RPD Solver

### Algorithm

- **Chromosome encoding:** 4 genes per customer — (dropoff agent, pickup agent, dropoff priority, pickup priority)
- **Decoder:** Discrete-event simulation guaranteeing feasibility; rescue mechanism handles stuck states
- **Warm start:** Construction heuristics (nearest-neighbor, greedy-defer, max-regret) seed the top 20% of the population
- **BRKGA:** Biased Random-Key Genetic Algorithm on parallel island model (GPU)
- **ALNS:** Adaptive Large Neighborhood Search destroy-repair operators
- **GP hyper-heuristic:** Gene injection from elite building blocks

### Key Results

Warm-start BRKGA achieves **13–67% makespan reduction** over construction heuristics. Friedman tests p < 0.01 across all 5 variants (base, 2x, 5x, 1R10, 1R20) and all 7 benchmark datasets.

### Run the solver

```bash
cd vrp_rpd
python3 main.py --dataset bays29 --variant base
```

---

## Part 2 — AGV Warehouse Testbed

An extension of the VRP-RPD problem to a physical cobot warehouse environment. The testbed maps benchmark instances onto a 10×10 grid warehouse and adds a MAPF (Multi-Agent Path Finding) collision avoidance layer, creating a full two-layer optimization pipeline.

### What's novel here

| Feature | Prior Work | This Work |
|---------|-----------|-----------|
| Cross-agent handoff at service node | ✗ | ✓ |
| Spur geometry (off-grid workstations) | ✗ | ✓ |
| Complete MAPF collision avoidance | Partial | ✓ |
| Task ↔ path feedback loop | ✗ | ✓ |
| Critical-path derived priority | ✗ | ✓ |

### Spur geometry

Workstations are physical installations **inside** grid cells — not on the transit grid. Each has a vertical spur connecting it to the main corridor:

```
  [transit node]  ← robots travel freely along the grid
       |
  [spur entry]    ← robot turns off the main corridor here
       |
  [workstation]   ← robot drops off / picks up resource
       |
  [spur entry]    ← robot exits back to transit corridor
```

Only the robot with a job at a workstation ever enters the spur. All other robots pass by unaffected.

### Run the AGV testbed

```bash
cd agv_testbed

# Run pipeline on a single dataset
python3 pipeline.py

# Batch analysis across all datasets
python3 analyze.py --datasets gr17 gr21 gr24 gr48 bays29 berlin52 eil51 \
                   --variants base 2x 5x --output results_spur.json

# Export animation as GIF (headless server)
python3 visualize.py --export --dataset bays29 --variant base --output bays29.gif

# Web viewer — stream live animation to browser (VPN required)
python3 web_viewer.py --dataset bays29 --variant base --port 5001

# Load a previously saved result into the web viewer (faster — skips re-solving)
python3 web_viewer.py --load results/bays29_base.json --port 5001
```

See [agv_testbed/README.md](agv_testbed/README.md) for full documentation.

---

## Datasets

All experiments use TSPlib benchmark instances mapped onto the warehouse grid:

| Dataset  | Customers | Agents | Resources/Agent |
|----------|-----------|--------|-----------------|
| gr17     | 16        | 3      | 5               |
| gr21     | 20        | 3      | 5               |
| gr24     | 23        | 4      | 6               |
| gr48     | 47        | 4      | 6               |
| bays29   | 28        | 4      | 6               |
| berlin52 | 51        | 4      | 6               |
| eil51    | 50        | 4      | 6               |

Processing-time variants: `base`, `2x`, `5x` (deterministic) and `1R10`, `1R20` (stochastic instances).

---

## Dependencies

```bash
pip install numpy pygame pillow flask
```

GPU solver additionally requires CUDA and the Gurobi Python API for exact MILP baselines.

---

## Patent & IP

A technical disclosure covering the cross-agent handoff mechanism, two-layer conflict interface, and critical-path priority system has been filed with the URI Office of the Vice President for Research (April 2026, Sodhi & Prasad).
