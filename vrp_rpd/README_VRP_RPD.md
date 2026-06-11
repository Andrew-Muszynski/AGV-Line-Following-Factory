# VRP-RPD Solver

A high-performance solver for the **Vehicle Routing Problem with Resource-Constrained Pickup and Delivery (VRP-RPD)** using Biased Random-Key Genetic Algorithm (BRKGA) with Genetic Programming-driven gene injection.

## Problem Description

VRP-RPD introduces a novel vehicle routing variant where agents deploy finite identical resources at customer locations for processing before retrieval and redeployment. Key applications include:

- Portable medical equipment delivery and retrieval
- Tool rental logistics
- Disaster relief resource deployment

### Key Distinguishing Features

Unlike classical pickup-and-delivery variants, VRP-RPD permits **different agents to perform dropoff and pickup** for the same customer, creating inter-route dependencies absent from standard formulations. This introduces significant computational complexity as routes become interdependent rather than independent.

## Paper Abstract

> We introduce the Vehicle Routing Problem with Resource-Constrained Pickup and Delivery (VRP-RPD), where agents deploy finite identical resources at customer locations for processing before retrieval and redeployment. Applications include portable medical equipment, tool rental, and disaster relief. Unlike classical pickup-and-delivery variants, VRP-RPD permits different agents to perform dropoff and pickup for the same customer—creating inter-route dependencies absent from standard formulations. We provide a complete mixed-integer linear programming formulation and demonstrate that exact methods are intractable even for small instances. Problems with 16 customers cannot be solved to optimality within two hours of computational time. We develop a Biased Random-Key Genetic Algorithm (BRKGA) with a four-gene-per-customer encoding. Two genes assign dropoff and pickup agents independently, while two priority keys determine sequencing. A simulation decoder guarantees feasibility by deferring operations until resources become available. Experiments on 14 TSPlib-derived benchmarks across five variants of processing time (base, 2X, 5X, 1R10, 1R20) compare four configurations. Warm-start BRKGA achieves 13–67% makespan reduction over the heuristics, with larger gains on higher-resource instances. Ablation tests show warm-start initialization is the primary driver of performance. Friedman tests (p < 0.01) confirm warm-start BRKGA superiority across all instance variants.

## Algorithm Configurations

The solver implements five configurations to isolate the contribution of each component:

1. **Heur**: Best of nearest neighbor, greedy-defer, and max-regret heuristics
2. **BRKGA**: Random initialization, no gene injection
3. **CSGI** (Cold Start with Gene Injection): Random initialization with GP-driven gene injection during evolution
4. **WS** (Warm Start): Heuristic-seeded initialization, no gene injection
5. **WSGI** (Warm Start with Gene Injection): Heuristic-seeded initialization with GP-driven gene injection

All BRKGA variants share the same encoding, decoder, operators, and island model. They differ only in initialization and use of gene injection.

## Key Features

### Algorithm Components
- **Four-gene-per-customer encoding**: Two genes assign dropoff/pickup agents independently, two priority keys determine sequencing
- **Simulation decoder**: Guarantees feasibility by deferring operations until resources become available
- **Multiple heuristics**: Nearest Neighbor, Max Regret, Greedy-Defer
- **Genetic Programming analysis**: Building block detection, FFT frequency analysis, similarity clustering
- **Gene injection**: GP-driven candidate generation from elite solutions
- **Warm start**: Heuristic-seeded population initialization
- **Island model**: Distributed evolution across GPU and CPU workers

### Implementation Features
- Multi-GPU + CPU parallel processing
- Numba JIT acceleration for decoder
- CUDA kernel evaluation support
- Checkpointing and solution tracking
- HTML Gantt chart visualization
- JSON solution export with convergence history

## Repository Structure

```
vrp_rpd/
├── vrp_rpd/                    # Core solver package
│   ├── models.py               # VRPRPDInstance problem definition
│   ├── solver.py               # VRPRPDSolver main class
│   ├── decoder.py              # Chromosome decoder and simulation
│   ├── heuristics.py           # Construction heuristics (NN, Max Regret, etc.)
│   ├── genetic_analyzer.py     # GP-based gene analysis
│   ├── islands.py              # GPU/CPU island implementations
│   ├── workers.py              # Parallel worker management
│   ├── visualization.py        # Gantt chart generation
│   ├── heuristic_runner.py     # Heuristic-only mode
│   ├── alns.py                 # ALNS implementation
│   └── utils.py                # I/O utilities (TSPLIB, CSV, JSON)
├── main.py                     # CLI entry point
├── datasets/                   # TSPlib-derived benchmarks
│   ├── berlin52/
│   ├── eil51/
│   ├── eil101/
│   └── ...                     # 14 benchmark instances
├── results/                    # Experimental results
│   ├── heuristics_only/
│   ├── pure_brkga/
│   ├── cold_start/
│   ├── warm_start_no_gene/
│   └── warm_start/
├── run_all_*.sh                # Batch experiment scripts
├── statistical_analysis.py     # Friedman test and analysis
└── extract_makespan_comparison.py  # Results aggregation
```

## Installation

### Requirements

- Python 3.8+
- PyTorch (for GPU support)
- NumPy
- Numba (optional, for JIT acceleration)
- CUDA toolkit (optional, for GPU acceleration)

### Setup

```bash
# Clone the repository
git clone https://github.com/yourusername/vrp-rpd-solver.git
cd vrp-rpd-solver/vrp_rpd

# Install dependencies
pip install torch numpy numba

# Verify installation
python main.py --help
```

## Usage

### Basic Usage

```bash
# Run with nearest neighbor heuristic
python main.py --tsp datasets/berlin52/berlin52.tsp \
               --jobs datasets/berlin52/berlin52_jobs_2X.txt \
               --Heuristic 1

# Run warm-start BRKGA with gene injection (WSGI configuration)
python main.py --tsp datasets/berlin52/berlin52.tsp \
               --jobs datasets/berlin52/berlin52_jobs_2X.txt \
               --Agents 6 --Resources 4 \
               --Heuristic 3 --GP yes --Warm yes --GeneInjection yes
```

### Algorithm Configuration Options

```bash
# BRKGA (random initialization)
python main.py --tsp <file> --jobs <file> --Warm no --GeneInjection no

# CSGI (random init + gene injection)
python main.py --tsp <file> --jobs <file> --Warm no --GeneInjection yes --GP yes

# WS (warm start)
python main.py --tsp <file> --jobs <file> --Warm yes --GeneInjection no

# WSGI (warm start + gene injection)
python main.py --tsp <file> --jobs <file> --Warm yes --GeneInjection yes --GP yes

# Heuristics only
python main.py --tsp <file> --jobs <file> --heuristic-only
```

### Advanced Options

```bash
# Custom population and generations
python main.py --tsp <file> --jobs <file> \
               --pop-gpu 1024 --pop-cpu 512 \
               --gens 10000 --interval 200

# GPU/CPU worker configuration
python main.py --tsp <file> --jobs <file> \
               --gpus 4 --cpu-workers 8

# Diversity and exploration parameters
python main.py --tsp <file> --jobs <file> \
               --mutation-rate 0.15 \
               --mutation-strength 0.03 \
               --elite-size 5 \
               --crossover-rate 0.85

# Enable checkpointing
python main.py --tsp <file> --jobs <file> \
               --checkpoint-interval 500 \
               --output berlin52_solution.json

# Load from existing solution
python main.py --tsp <file> --jobs <file> \
               --from-json previous_solution.json

# Reproducible results
python main.py --tsp <file> --jobs <file> --seed 42
```

### GP Analysis Components

```bash
# Enable specific analysis components
python main.py --tsp <file> --jobs <file> \
               --GP yes \
               --use-blocks yes \      # Building block analysis (default)
               --use-fft no \          # FFT frequency analysis
               --use-similarity no     # Similarity clustering
```

## Input File Formats

### TSPLIB Format
Standard TSPLIB `.tsp` files with NODE_COORD_SECTION or EDGE_WEIGHT_SECTION.

### Jobs File Format
```
depot_index
num_agents num_resources
processing_time_1
processing_time_2
...
processing_time_n
```

Example:
```
0
6 4
10.5
15.2
8.0
...
```

### CSV Distance Matrix
Symmetric distance matrix in CSV format (n x n).

## Output Files

### Solution JSON
Contains:
- Problem parameters (customers, agents, resources)
- Solution routes and timing
- Job schedules (dropoff, processing, pickup)
- Algorithm configuration
- Convergence history
- Per-worker results

### HTML Gantt Chart
Interactive timeline visualization showing:
- Agent routes with travel and operations
- Resource utilization over time
- Customer processing windows
- Cross-agent dependencies (diagonal lines for mixed operations)

## Benchmarks

The repository includes 14 TSPlib-derived benchmark instances with 5 processing time variants each:

**Instances**: bays29, berlin52, dsj1000, eil51, eil101, gr17, gr21, gr24, gr48, gr202, gr431, gr666, kroA100, rat783

**Processing Time Variants**:
- **base**: Standard processing times
- **2X**: Double processing times
- **5X**: Quintuple processing times
- **1R10**: Random processing times [1, 10]
- **1R20**: Random processing times [1, 20]

Total: **70 benchmark problem instances**

## Running Experiments

### Batch Experiments

```bash
# Run all configurations on all benchmarks
./run_all_warm_start.sh          # WSGI configuration
./run_all_warm_start_no_gene.sh  # WS configuration
./run_all_pure_brkga.sh          # BRKGA configuration
./run_all_cold_start.sh          # CSGI configuration

# Heuristics baseline
./run_all_heuristics.sh
```

### Statistical Analysis

```bash
# Perform Friedman test and generate comparison tables
python statistical_analysis.py

# Extract makespan comparisons
python extract_makespan_comparison.py
```

## Results

Experimental results are organized by configuration in the [results/](results/) directory:

- `heuristics_only/`: Baseline heuristic results
- `pure_brkga/`: Random initialization (BRKGA)
- `cold_start/`: Random init + gene injection (CSGI)
- `warm_start_no_gene/`: Heuristic warm start (WS)
- `warm_start/`: Warm start + gene injection (WSGI)

Each directory contains JSON solution files with convergence histories and timing data.

### Key Findings

- Warm-start BRKGA achieves **13-67% makespan reduction** over heuristics
- Larger gains on **higher-resource instances** (1R10, 1R20 variants)
- **Warm-start initialization is the primary driver** of performance
- Friedman tests confirm **WSGI superiority** across all variants (p < 0.01)

## Computational Complexity

The paper demonstrates that exact MILP methods are intractable:
- Problems with **16 customers cannot be solved to optimality within 2 hours**
- VRP-RPD complexity stems from inter-route dependencies created by mixed agent operations

## Performance Notes

### GPU Acceleration
- Requires PyTorch with CUDA support
- Recommended: NVIDIA GPU with compute capability 6.0+
- Scales efficiently across multiple GPUs

### CPU Parallelization
- Numba JIT compilation provides 10-50x speedup over pure Python
- Multi-core CPU workers complement GPU processing

### Memory Requirements
- Scales with population size and number of customers
- Typical usage: 2-8 GB for instances with 50-100 customers

## Citation

If you use this solver in your research, please cite:

## Acknowledgments

- TSPlib benchmark instances: Gerhard Reinelt, University of Heidelberg
- BRKGA framework inspiration from Gonçalves & Resende (2011)
