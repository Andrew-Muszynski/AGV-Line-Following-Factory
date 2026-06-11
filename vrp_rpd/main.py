#!/usr/bin/env python3
"""
VRP-RPD Solver CLI Entry Point

Usage:
  python main.py --tsp berlin52.tsp --jobs berlin52_jobs.txt \\
      --Agents 6 --Resources 4 --Heuristic 1 --GP yes --Warm yes --GeneInjection yes
"""

import numpy as np
import random
import os
import json
import argparse
import torch.multiprocessing as mp


def set_seed(seed: int):
    """Set seeds for Python, NumPy, and optionally PyTorch."""
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    except ImportError:
        pass

from vrp_rpd import (
    VRPRPDInstance,
    VRPRPDSolver,
    decode_chromosome,
    generate_html_gantt,
    load_tsplib,
    load_csv_distances,
    load_jobs,
    load_solution_from_json,
    simulate_solution,
    run_heuristics_only,
)


def main():
    parser = argparse.ArgumentParser(
        description='VRP-RPD Solver with Heuristics, GP, and Gene Injection (PATCHED: Diagonal Cross-Agent Lines)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic run with nearest neighbor heuristic
  python %(prog)s --tsp berlin52.tsp --jobs berlin52_jobs.txt --Heuristic 1

  # Max regret heuristic with GP and gene injection
  python %(prog)s --tsp berlin52.tsp --jobs berlin52_jobs.txt --Heuristic 3 --GP yes --GeneInjection yes

  # Custom agents and resources
  python %(prog)s --tsp berlin52.tsp --jobs berlin52_jobs.txt --Agents 4 --Resources 2

  # Disable warm start
  python %(prog)s --tsp berlin52.tsp --jobs berlin52_jobs.txt --Warm no

  # Use CSV distance matrix
  python %(prog)s --csv distances.csv --jobs jobs.txt --Agents 6 --Resources 4

  # Start from existing JSON solution
  python %(prog)s --tsp berlin52.tsp --jobs berlin52_jobs.txt --from-json previous_soln.json
        """
    )



    # Input files
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--tsp', help='TSPLIB file (looks in current directory)')
    input_group.add_argument('--csv', help='CSV distance matrix file')

    parser.add_argument('--jobs', required=True, help='Jobs file path')
    parser.add_argument('--from-json', type=str, help='Load initial solution from JSON file')

    # Problem parameters
    parser.add_argument('--Agents', '-m', type=int, help='Number of agents (overrides jobs file)')
    parser.add_argument('--Resources', '-k', type=int, help='Resources per agent (overrides jobs file)')

    # Heuristic selection
    parser.add_argument('--Heuristic', type=int, default=0, choices=[0, 1, 3],
                        help='Heuristic: 0=None, 1=Nearest Neighbor, 3=Max Regret')

    # Algorithm options
    parser.add_argument('--GP', type=str, default='yes', choices=['yes', 'no'],
                        help='Enable Genetic Programming analysis (yes/no)')
    parser.add_argument('--Warm', type=str, default='yes', choices=['yes', 'no'],
                        help='Enable warm start from heuristics (yes/no)')
    parser.add_argument('--GeneInjection', type=str, default='yes', choices=['yes', 'no'],
                        help='Enable gene injection from analysis (yes/no)')

    # GP Analysis Components (only used if --GP yes)
    parser.add_argument('--use-blocks', type=str, default='yes', choices=['yes', 'no'],
                        help='Use building block analysis in candidate generation (default: yes)')
    parser.add_argument('--use-fft', type=str, default='no', choices=['yes', 'no'],
                        help='Use FFT frequency analysis in candidate generation (default: no)')
    parser.add_argument('--use-similarity', type=str, default='no', choices=['yes', 'no'],
                        help='Use similarity clustering in candidate generation (default: no)')

    # Worker configuration
    parser.add_argument('--gpus', type=int, default=None, help='Number of GPUs (default: all available)')
    parser.add_argument('--cpu-workers', type=int, default=0, help='Number of CPU workers')
    parser.add_argument('--pop-gpu', type=int, default=512, help='Population per GPU')
    parser.add_argument('--pop-cpu', type=int, default=256, help='Population per CPU')

    # Evolution parameters
    parser.add_argument('--gens', type=int, default=5000, help='Total generations')
    parser.add_argument('--interval', type=int, default=200, help='Generations per GP cycle (default: 200)')
    parser.add_argument('--candidates', type=int, default=20, help='Candidates per injection')

    # Diversity & Exploration parameters
    parser.add_argument('--mutation-rate', type=float, default=None,
                        help='Base mutation rate (default: 0.1 for GPU, 0.15 for CPU). Higher = more exploration')
    parser.add_argument('--mutation-range', type=float, default=0.18,
                        help='Mutation rate range across workers (default: 0.18)')
    parser.add_argument('--mutation-strength', type=float, default=0.02,
                        help='Mutation perturbation strength (default: 0.02). Higher = larger changes')
    parser.add_argument('--elite-size', type=int, default=10,
                        help='Number of elite solutions preserved (default: 10). Lower = more diversity')
    parser.add_argument('--crossover-rate', type=float, default=0.8,
                        help='Crossover probability (default: 0.8)')
    parser.add_argument('--restart-stagnation', type=int, default=0,
                        help='Restart population after N stagnant generations (default: 0=disabled)')

    # Output
    parser.add_argument('--output', type=str, help='Output JSON file (overrides default)')
    parser.add_argument('--output_html', type=str, default='gantt.html', help='Output HTML Gantt chart')
    parser.add_argument('--checkpoint-interval', type=int, default=0, help='Save checkpoint every N generations (0=disabled, saves as A.json, B.json, etc.)')

    # Heuristic-only mode
    parser.add_argument('--heuristic-only', action='store_true',
                        help='Run heuristics only (no GA evolution). Saves result to {basename}H.json')

    parser.add_argument('--seed', type=int, default=None)

    parser.add_argument("--allow_mixed", action="store_false", default=True,
    help="Disables interleaving if this is used")

    args = parser.parse_args()

    # Apply seed if provided
    if args.seed is not None:
        print(f"Setting random seed to {args.seed}")
        set_seed(args.seed)

    # Load instance
    if args.tsp:
        dist, n, coords = load_tsplib(args.tsp)
    else:
        dist, n = load_csv_distances(args.csv)
        coords = None

    proc, depot, agents, resources = load_jobs(args.jobs, n, args.Agents, args.Resources)

    print(f"DEBUG: proc shape: {proc.shape if hasattr(proc, 'shape') else proc}, "
      f"agents: {agents}, resources: {resources}, n: {n}")

    instance = VRPRPDInstance(
        distance_matrix=dist,
        processing_times=proc,
        num_agents=agents,
        resources_per_agent=resources,
        depot=depot,
        coordinates=coords
    )

    # Handle heuristic-only mode
    if args.heuristic_only:
        # Generate output filename: {basename}H.json
        jobs_basename = os.path.splitext(os.path.basename(args.jobs))[0]
        heuristic_output = args.output if args.output else f"{jobs_basename}H.json"
        html_output = args.output_html if args.output_html != 'gantt.html' else f"{jobs_basename}H.html"

        run_heuristics_only(instance, heuristic_output, html_output, allow_mixed=args.allow_mixed)
        return

    # Load JSON solution if provided
    json_chromosome = None
    if args.from_json:
        print(f"\n{'='*70}")
        print(f"LOADING INITIAL SOLUTION FROM JSON")
        print(f"{'='*70}")
        try:
            json_chromosome, json_makespan = load_solution_from_json(args.from_json, instance)
            print(f"File: {args.from_json}")
            print(f"Makespan: {json_makespan:.2f}")
            print(f"Status: SUCCESS - Will use as warm start")
            print(f"{'='*70}\n")
        except Exception as e:
            import traceback
            print(f"Status: FAILED")
            print(f"Error: {e}")
            print(f"\nTraceback:")
            traceback.print_exc()
            print(f"\nContinuing without JSON warm start...")
            print(f"{'='*70}\n")
            json_chromosome = None

    # Determine checkpoint prefix from output filename
    checkpoint_prefix = ""
    if args.checkpoint_interval > 0:
        if args.output:
            # Remove .json extension and use as prefix
            checkpoint_prefix = args.output.replace('.json', '_')
        else:
            jobs_basename = os.path.splitext(os.path.basename(args.jobs))[0]
            checkpoint_prefix = f"{jobs_basename}_"

    # Create solver
    solver = VRPRPDSolver(
        instance,
        num_gpus=args.gpus,
        num_cpu_workers=args.cpu_workers,
        population_per_gpu=args.pop_gpu,
        population_per_cpu=args.pop_cpu,
        total_generations=args.gens,
        gens_per_cycle=args.interval,
        candidates_per_injection=args.candidates,
        heuristic=args.Heuristic,
        use_gp=(args.GP == 'yes'),
        use_warm=(args.Warm == 'yes'),
        use_gene_injection=(args.GeneInjection == 'yes'),
        use_blocks=(args.use_blocks == 'yes'),
        use_fft=(args.use_fft == 'yes'),
        use_similarity=(args.use_similarity == 'yes'),
        allow_mixed=args.allow_mixed,
        json_solution_chromosome=json_chromosome,
        checkpoint_interval=args.checkpoint_interval,
        checkpoint_prefix=checkpoint_prefix,
        mutation_rate_base=args.mutation_rate,
        mutation_rate_range=args.mutation_range,
        mutation_strength=args.mutation_strength,
        elite_size=args.elite_size,
        crossover_rate=args.crossover_rate,
        restart_stagnation=args.restart_stagnation
    )

    # Solve
    mp.set_start_method('spawn', force=True)
    result = solver.solve()

    # Generate default solution JSON filename from jobs file
    jobs_basename = os.path.splitext(os.path.basename(args.jobs))[0]
    default_json_output = f"{jobs_basename}_soln.json"
    json_output_path = args.output if args.output else default_json_output

    # Always save solution JSON with useful information
    if result.get('makespan', float('inf')) < float('inf') and result.get('best_chromosome') is not None:
        chrom = result['best_chromosome']
        tours = decode_chromosome(chrom, instance, allow_mixed=args.allow_mixed)
        job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
            tours, instance
        )

        # Compute CORRECT makespan from simulation (not from GA fitness!)
        correct_makespan = max(agent_completion_times.values()) if agent_completion_times else float('inf')

        routes_json = []
        for a in range(instance.m):
            tour = agent_tours.get(a, [])
            route_stops = []
            for cust_loc, op in tour:
                route_stops.append({
                    'node': int(cust_loc),
                    'operation': 'dropoff' if op == 'D' else 'pickup'
                })
            routes_json.append({
                'agent': a,
                'stops': route_stops,
                'num_customers': len([s for s in route_stops if s['operation'] == 'dropoff']),
                'completion_time': float(agent_completion_times.get(a, 0))
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

        # Build convergence history - list of (generation, fitness) for best worker
        convergence_history = result.get('convergence_history', {})

        # Merge all worker histories into a single best-so-far curve
        merged_history = []
        all_points = []
        for worker_id, history in convergence_history.items():
            for gen, fitness in history:
                all_points.append((gen, fitness, worker_id))

        # Sort by generation and track best-so-far
        all_points.sort(key=lambda x: x[0])
        best_so_far = float('inf')
        for gen, fitness, worker_id in all_points:
            if fitness < best_so_far:
                best_so_far = fitness
                merged_history.append({'generation': gen, 'fitness': fitness, 'worker': worker_id})

        solution_json = {
            'problem': {
                'jobs_file': args.jobs,
                'tsp_file': args.tsp if args.tsp else None,
                'csv_file': args.csv if args.csv else None,
                'num_customers': instance.num_customers,
                'num_agents': instance.m,
                'resources_per_agent': instance.k,
                'depot': instance.depot
            },
            'solution': {
                'makespan': float(correct_makespan),
                'solve_time_seconds': float(result.get('solve_time', 0)),
                'source': result.get('source', 'unknown'),
                'heuristic_makespan': float(result.get('heuristic_makespan', 0)) if result.get('heuristic_makespan') else None,
                'routes': routes_json,
                'jobs': jobs_json
            },
            'algorithm': {
                'heuristic': args.Heuristic,
                'gp_enabled': args.GP == 'yes',
                'warm_start_enabled': args.Warm == 'yes',
                'gene_injection_enabled': args.GeneInjection == 'yes',
                'population_size': args.pop_gpu,
                'total_generations': args.gens,
                'mutation_rate': args.mutation_rate,
                'elite_size': args.elite_size,
                'crossover_rate': args.crossover_rate
            },
            'convergence': {
                'history': merged_history,
                'per_worker': {wid: [{'generation': g, 'fitness': f} for g, f in hist]
                               for wid, hist in convergence_history.items()}
            },
            'worker_results': result.get('all_results', [])
        }

        with open(json_output_path, 'w') as f:
            json.dump(solution_json, f, indent=2)
        print(f"\nSolution JSON saved to {json_output_path}")
    else:
        error_json = {
            'problem': {
                'jobs_file': args.jobs,
                'num_customers': instance.num_customers,
                'num_agents': instance.m,
                'resources_per_agent': instance.k
            },
            'solution': {
                'makespan': float(result.get('makespan', float('inf'))),
                'error': result.get('error', 'No valid solution found')
            }
        }
        with open(json_output_path, 'w') as f:
            json.dump(error_json, f, indent=2)
        print(f"\nError JSON saved to {json_output_path}")

    # Generate HTML Gantt chart with DIAGONAL cross-agent lines
    if args.output_html and result.get('makespan', float('inf')) < float('inf'):
        generate_html_gantt(
            result=result,
            instance=instance,
            output_path=args.output_html,
            title=f"BRKGA-GP Solution (Makespan: {result['makespan']:.1f})",
            allow_mixed=args.allow_mixed
        )

if __name__ == '__main__':
    main()
