#!/usr/bin/env python3
"""
Run ALNS with warm-start from heuristic solutions or from scratch.

Usage:
    # Run from scratch (cold start)
    python run_alns.py --tsp data/berlin52.tsp --jobs data/berlin52_jobs.txt --output solution_alns.json

    # Run with warm-start from heuristic JSON
    python run_alns.py --tsp data/berlin52.tsp --jobs data/berlin52_jobs.txt \
        --warmstart results/heuristics_only/berlin52/base/berlin52_heuristics.json \
        --output solution_alns.json

    # Run with custom ALNS parameters
    python run_alns.py --tsp data/berlin52.tsp --jobs data/berlin52_jobs.txt \
        --warmstart results/heuristics_only/berlin52/base/berlin52_heuristics.json \
        --alns_iter 20000 --alns_time 600 --parallel 4 \
        --output solution_alns.json

    # Run without interleaving (no cross-vehicle pickups)
    python run_alns.py --tsp data/berlin52.tsp --jobs data/berlin52_jobs.txt \
        --no_interleave --output solution_alns.json
"""

import argparse
import json
import numpy as np
import sys
import os
from pathlib import Path

# Add vrp_rpd to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vrp_rpd.models import VRPRPDInstance
from vrp_rpd.alns import run_alns, ALNSConfig
from vrp_rpd.decoder import decode_chromosome
from vrp_rpd.utils import simulate_solution, load_tsplib, load_jobs
from vrp_rpd.visualization import generate_html_gantt


def load_heuristic_solution(json_path: str, heuristic_name: str = None):
    """
    Load a heuristic solution from JSON file.

    Args:
        json_path: Path to heuristics JSON file
        heuristic_name: Optional specific heuristic to load (e.g., "Greedy Defer 5.0x")
                       If None, loads the best solution

    Returns:
        chromosome: numpy array [n x 4]
        makespan: float
        name: str
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    if heuristic_name is not None:
        # Find specific heuristic
        solution = None
        for sol in data.get('all_heuristics', []):
            if sol['heuristic'] == heuristic_name:
                solution = sol
                break
        if solution is None:
            raise ValueError(f"Heuristic '{heuristic_name}' not found in {json_path}")
    else:
        # Use best solution
        if 'all_heuristics' in data and len(data['all_heuristics']) > 0:
            solution = data['all_heuristics'][0]  # Already sorted by makespan
        else:
            raise ValueError(f"No heuristic solutions found in {json_path}")

    chromosome = np.array(solution['chromosome'], dtype=np.float32)
    makespan = solution['makespan']
    name = solution['heuristic']

    return chromosome, makespan, name


def main():
    parser = argparse.ArgumentParser(
        description='Run ALNS for VRP-RPD with optional warm-start from heuristics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    # Problem files
    parser.add_argument('--tsp', '--dist', required=True,
                       help='TSP distance file (TSPLIB format)')
    parser.add_argument('--jobs', required=True,
                       help='Jobs file with processing times')
    parser.add_argument('--output', '-o', default='solution_alns.json',
                       help='Output JSON file (default: solution_alns.json)')
    parser.add_argument('--html', default=None,
                       help='Optional output HTML Gantt chart')

    # Warm-start options
    parser.add_argument('--warmstart', default=None,
                       help='Path to heuristics JSON file for warm-start')
    parser.add_argument('--heuristic', default=None,
                       help='Specific heuristic to use from JSON (default: best)')

    # ALNS parameters
    parser.add_argument('--alns_iter', type=int, default=10000,
                       help='Maximum ALNS iterations (default: 10000)')
    parser.add_argument('--alns_time', type=int, default=300,
                       help='Maximum ALNS time in seconds (default: 300)')
    parser.add_argument('--no_improve_limit', type=int, default=500,
                       help='Stop after N iterations without improvement (default: 500)')
    parser.add_argument('--parallel', '-p', type=int, default=1,
                       help='Number of parallel ALNS workers (default: 1)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')

    # ALNS tuning
    parser.add_argument('--min_destroy', type=float, default=0.1,
                       help='Minimum destroy percentage (default: 0.1)')
    parser.add_argument('--max_destroy', type=float, default=0.3,
                       help='Maximum destroy percentage (default: 0.3)')
    parser.add_argument('--init_temp', type=float, default=100.0,
                       help='Initial temperature for simulated annealing (default: 100.0)')
    parser.add_argument('--cooling_rate', type=float, default=0.999,
                       help='Temperature cooling rate (default: 0.999)')

    # Problem settings
    parser.add_argument('--no_interleave', action='store_true',
                       help='Disable interleaving (same agent for drop and pick)')

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("ALNS Solver for VRP-RPD")
    print("=" * 70)

    # Load instance
    print(f"\nLoading instance...")
    print(f"  TSP: {args.tsp}")
    print(f"  Jobs: {args.jobs}")

    # Load distance matrix and coordinates
    dist, depot, coords = load_tsplib(args.tsp)
    n = len(dist)

    # Load jobs (processing times and problem parameters)
    proc, depot_jobs, m, k = load_jobs(args.jobs, n)

    # Depot should match between TSP and jobs file
    if depot != depot_jobs:
        print(f"  WARNING: Depot mismatch (TSP: {depot}, Jobs: {depot_jobs}). Using jobs file depot: {depot_jobs}")
        depot = depot_jobs

    # Create instance
    instance = VRPRPDInstance(
        distance_matrix=dist,
        processing_times=proc,
        num_agents=m,
        resources_per_agent=k,
        depot=depot,
        coordinates=coords
    )

    print(f"\nProblem:")
    print(f"  Customers: {instance.num_customers}")
    print(f"  Agents: {instance.m}")
    print(f"  Capacity: {instance.k}")
    print(f"  Depot: {instance.depot}")
    print(f"  Interleaving: {'disabled' if args.no_interleave else 'enabled'}")

    # Load warm-start if provided
    initial_chromosome = None
    warmstart_name = None
    warmstart_makespan = None

    if args.warmstart:
        print(f"\nLoading warm-start from: {args.warmstart}")
        try:
            initial_chromosome, warmstart_makespan, warmstart_name = load_heuristic_solution(
                args.warmstart, args.heuristic
            )
            print(f"  Heuristic: {warmstart_name}")
            print(f"  Makespan: {warmstart_makespan:.2f}")
        except Exception as e:
            print(f"  WARNING: Failed to load warm-start: {e}")
            print(f"  Continuing with cold start...")
            initial_chromosome = None
    else:
        print(f"\nNo warm-start provided - starting from scratch (cold start)")

    # Configure ALNS
    config = ALNSConfig(
        max_iterations=args.alns_iter,
        max_iterations_no_improve=args.no_improve_limit,
        max_time_seconds=args.alns_time,
        min_destroy_pct=args.min_destroy,
        max_destroy_pct=args.max_destroy,
        initial_temperature=args.init_temp,
        cooling_rate=args.cooling_rate,
        num_parallel=args.parallel,
        allow_cross_vehicle=not args.no_interleave
    )

    print(f"\nALNS Configuration:")
    print(f"  Max iterations: {config.max_iterations}")
    print(f"  Max time: {config.max_time_seconds}s")
    print(f"  No-improve limit: {config.max_iterations_no_improve}")
    print(f"  Destroy range: {config.min_destroy_pct:.1%} - {config.max_destroy_pct:.1%}")
    print(f"  Temperature: {config.initial_temperature} (cooling: {config.cooling_rate})")
    print(f"  Parallel workers: {config.num_parallel}")
    print(f"  Cross-vehicle pickups: {config.allow_cross_vehicle}")

    # Run ALNS
    print("\n" + "=" * 70)
    print("Running ALNS...")
    print("=" * 70)

    best_chromosome, best_makespan, metadata = run_alns(
        instance, config, initial_chromosome, args.seed
    )

    # Decode solution for output
    tours = decode_chromosome(best_chromosome, instance, allow_mixed=not args.no_interleave)
    job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
        tours, instance
    )

    # Build output JSON
    routes_json = []
    for agent in range(instance.m):
        tour = agent_tours.get(agent, [])
        route_stops = []
        for cust_loc, op in tour:
            route_stops.append({
                'node': int(cust_loc),
                'operation': 'dropoff' if op == 'D' else 'pickup'
            })
        routes_json.append({
            'agent': agent,
            'stops': route_stops,
            'completion_time': float(agent_completion_times.get(agent, 0))
        })

    jobs_json = {}
    for cust_loc, jt in job_times.items():
        jobs_json[int(cust_loc)] = {
            'dropoff_time': float(jt.get('dropoff', 0)),
            'processing_start': float(jt.get('start', 0)),
            'processing_end': float(jt.get('end', 0)),
            'pickup_time': float(jt.get('pickup', jt.get('end', 0))),
            'wait_time': float(jt.get('wait', 0)),
            'assigned_agent': int(customer_assignment.get(cust_loc, -1))
        }

    output_data = {
        'problem': {
            'num_customers': instance.num_customers,
            'num_agents': instance.m,
            'resources_per_agent': instance.k,
            'depot': instance.depot,
            'tsp_file': args.tsp,
            'jobs_file': args.jobs
        },
        'alns_config': {
            'max_iterations': config.max_iterations,
            'max_time_seconds': config.max_time_seconds,
            'parallel_workers': config.num_parallel,
            'allow_cross_vehicle': config.allow_cross_vehicle
        },
        'warm_start': {
            'used': args.warmstart is not None,
            'file': args.warmstart,
            'heuristic': warmstart_name,
            'initial_makespan': warmstart_makespan
        } if args.warmstart else None,
        'solution': {
            'makespan': float(best_makespan),
            'solve_time_seconds': metadata['solve_time'],
            'cross_vehicle_pickups': metadata['cross_vehicle_pickups'],
            'improvement': float(warmstart_makespan - best_makespan) if warmstart_makespan else None,
            'improvement_pct': float((warmstart_makespan - best_makespan) / warmstart_makespan * 100) if warmstart_makespan else None,
            'chromosome': best_chromosome.tolist(),
            'routes': routes_json,
            'jobs': jobs_json
        }
    }

    # Save solution
    with open(args.output, 'w') as f:
        json.dump(output_data, f, indent=2)
    print(f"\nSolution saved to: {args.output}")

    # Generate HTML Gantt if requested
    if args.html:
        result = {
            'best_chromosome': best_chromosome,
            'makespan': best_makespan,
            'solve_time': metadata['solve_time']
        }
        generate_html_gantt(
            result, instance, args.html,
            title=f"ALNS Solution (warm-start: {warmstart_name if warmstart_name else 'None'})",
            allow_mixed=not args.no_interleave
        )
        print(f"Gantt chart saved to: {args.html}")

    # Print summary
    print("\n" + "=" * 70)
    print("ALNS COMPLETE")
    print("=" * 70)
    if warmstart_makespan:
        improvement = warmstart_makespan - best_makespan
        improvement_pct = improvement / warmstart_makespan * 100
        print(f"Warm-start makespan:  {warmstart_makespan:.2f} ({warmstart_name})")
        print(f"ALNS makespan:        {best_makespan:.2f}")
        print(f"Improvement:          {improvement:.2f} ({improvement_pct:.2f}%)")
    else:
        print(f"ALNS makespan:        {best_makespan:.2f} (cold start)")
    print(f"Solve time:           {metadata['solve_time']:.2f}s")
    print(f"Cross-vehicle:        {metadata['cross_vehicle_pickups']}")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
