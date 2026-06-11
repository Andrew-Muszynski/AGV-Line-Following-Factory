#!/usr/bin/env python3
"""
VRP-RPD Solver Package

A Vehicle Routing Problem solver with Resource constraints, Pickups, and Dropoffs.
Uses BRKGA (Biased Random Key Genetic Algorithm) combined with Genetic Programming.

Features:
  - Multiple heuristic options (Nearest Neighbor, Max Regret)
  - Genetic Programming with gene analysis (FFT, similarity, building blocks)
  - Warm start from heuristics or database
  - Gene injection from best solutions
  - Multi-GPU + CPU worker pools
  - Numba JIT acceleration
  - CUDA kernel evaluation
"""

from .models import VRPRPDInstance
from .solver import VRPRPDSolver
from .decoder import (
    decode_chromosome,
    compute_makespan_fast,
    compute_makespan_from_tours,
    HAS_NUMBA,
)
from .heuristics import (
    generate_nearest_neighbor_solution,
    generate_max_regret_solution,
    generate_greedy_defer_solution,
    generate_savings_solution,
    apply_2opt_improvement,
    apply_relocate_improvement,
    apply_swap_improvement,
    generate_2opt_improved_solution,
)
from .genetic_analyzer import GeneticAnalyzer
from .islands import GPUIsland, CPUIsland
from .visualization import generate_html_gantt
from .heuristic_runner import run_heuristics_only
from .utils import (
    load_tsplib,
    load_csv_distances,
    load_jobs,
    load_solution_from_json,
    simulate_solution,
    verify_makespan,
    build_chromosome_from_tours,
    save_heuristic_solution_json,
    load_heuristic_solution_json,
)

__version__ = "1.0.0"
__author__ = "VRP-RPD Team"

__all__ = [
    # Core classes
    "VRPRPDInstance",
    "VRPRPDSolver",
    "GeneticAnalyzer",
    "GPUIsland",
    "CPUIsland",
    # Decoder functions
    "decode_chromosome",
    "compute_makespan_fast",
    "compute_makespan_from_tours",
    "HAS_NUMBA",
    # Heuristics
    "generate_nearest_neighbor_solution",
    "generate_max_regret_solution",
    "generate_greedy_defer_solution",
    "generate_savings_solution",
    "apply_2opt_improvement",
    "apply_relocate_improvement",
    "apply_swap_improvement",
    "generate_2opt_improved_solution",
    # Heuristic Runner
    "run_heuristics_only",
    # Visualization
    "generate_html_gantt",
    # Utilities
    "load_tsplib",
    "load_csv_distances",
    "load_jobs",
    "load_solution_from_json",
    "simulate_solution",
    "verify_makespan",
    "build_chromosome_from_tours",
    "save_heuristic_solution_json",
    "load_heuristic_solution_json",
]
