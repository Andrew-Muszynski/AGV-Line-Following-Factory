#!/usr/bin/env python3
"""
Heuristic-Only Mode

Run all heuristics without GA evolution and compare results.
"""

import json
import numpy as np
import signal
import time
import sys
import os
from typing import Dict
from contextlib import contextmanager

from .models import VRPRPDInstance
from .heuristics import (
    generate_nearest_neighbor_solution,
    generate_max_regret_solution,
    generate_savings_solution,
    generate_greedy_defer_solution,
    generate_2opt_improved_solution
)
from .decoder import compute_makespan_fast, decode_chromosome
from .utils import simulate_solution
from .visualization import generate_html_gantt

# Import validator's simulation for accurate makespan calculation
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from eval_soln import evaluate_solution_physical_model


class TimeoutException(Exception):
    """Exception raised when a heuristic exceeds time limit"""
    pass


@contextmanager
def timeout(seconds: int):
    """
    Context manager to enforce timeout on heuristic execution.

    Args:
        seconds: Maximum execution time in seconds

    Raises:
        TimeoutException: If execution exceeds time limit
    """
    def timeout_handler(signum, frame):
        raise TimeoutException(f"Execution exceeded {seconds} seconds")

    # Set the signal handler and alarm
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(seconds)

    try:
        yield
    finally:
        # Restore original handler and cancel alarm
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)


def run_heuristics_only(instance: VRPRPDInstance, output_path: str, html_path: str = None, allow_mixed: bool = True):
    """
    Run all heuristics and save ALL results to JSON for comparison.

    Args:
        instance: VRPRPDInstance
        output_path: Path for output JSON file
        html_path: Optional path for HTML Gantt chart
        allow_mixed: Whether to allow interleaving (default: True)

    Returns:
        Best makespan found
    """
    # Timeout limit per heuristic (in seconds)
    HEURISTIC_TIMEOUT = 5 * 60  # 5 minutes

    print("\n" + "=" * 70)
    print("HEURISTIC-ONLY MODE")
    print("=" * 70)
    print(f"Problem: {instance.num_customers} customers, {instance.m} agents, k={instance.k}")
    print(f"Timeout per heuristic: {HEURISTIC_TIMEOUT // 60} minutes")
    print("=" * 70 + "\n")

    best_makespan = float('inf')
    best_tours = None
    best_chrom = None
    best_heuristic = "None"

    # Store ALL results with full solution data
    all_solutions = []

    def add_solution(name: str, chrom: np.ndarray, tours: Dict = None, exec_time: float = 0.0):
        """Helper to add a solution with full details"""
        nonlocal best_makespan, best_tours, best_chrom, best_heuristic

        # Decode to get tours if not provided
        if tours is None:
            tours = decode_chromosome(chrom, instance, allow_mixed=allow_mixed)

        # First get job times using internal simulate_solution for route details
        job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
            tours, instance
        )

        # Prepare customers list
        if instance.depot == 0:
            customers = list(range(1, len(instance.dist)))
        else:
            customers = [i for i in range(len(instance.dist)) if i != instance.depot]

        # Use validator's simulation for ACCURATE makespan (matches validation)
        simulated_makespan, _, _ = evaluate_solution_physical_model(
            agent_tours, instance.dist, instance.proc, instance.depot,
            instance.m, instance.k, customers
        )

        # Validate customer coverage
        served_customers = set()
        for a in range(instance.m):
            tour = agent_tours.get(a, [])
            for cust_loc, op in tour:
                if op == 'D':  # Count dropoffs
                    served_customers.add(cust_loc)

        is_valid = len(served_customers) == instance.num_customers

        # Build routes JSON
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

        # Build jobs JSON
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

        solution_entry = {
            'heuristic': name,
            'makespan': float(simulated_makespan),
            'execution_time_seconds': float(exec_time),
            'chromosome': chrom.tolist(),
            'routes': routes_json,
            'jobs': jobs_json,
            'valid': is_valid,
            'customers_served': len(served_customers)
        }
        all_solutions.append(solution_entry)

        # Print with validation status
        status = "✓" if is_valid else f"✗ INVALID ({len(served_customers)}/{instance.num_customers} customers)"
        print(f"  {name}: {simulated_makespan:.2f} (time: {exec_time:.2f}s) {status}")

        # Only consider valid solutions for "best"
        if is_valid and simulated_makespan < best_makespan:
            best_makespan = simulated_makespan
            best_tours = tours
            best_chrom = chrom
            best_heuristic = name

        return simulated_makespan

    # 1. Nearest Neighbor
    print("Running Nearest Neighbor heuristic...")
    start_time = time.time()
    try:
        with timeout(HEURISTIC_TIMEOUT):
            chrom, makespan, tours = generate_nearest_neighbor_solution(
                instance.dist, instance.proc, instance.depot,
                instance.m, instance.k, instance.num_customers,
                allow_mixed=allow_mixed
            )
            exec_time = time.time() - start_time
            add_solution('Nearest Neighbor', chrom, tours, exec_time)
    except TimeoutException as e:
        print(f"  Nearest Neighbor TIMEOUT: {e}")
    except Exception as e:
        print(f"  Nearest Neighbor failed: {e}")

    # 2. Max Regret
    print("Running Max Regret heuristic...")
    start_time = time.time()
    try:
        with timeout(HEURISTIC_TIMEOUT):
            chrom, makespan, tours = generate_max_regret_solution(
                instance.dist, instance.proc, instance.depot,
                instance.m, instance.k, instance.num_customers,
                allow_mixed=allow_mixed
            )
            exec_time = time.time() - start_time
            add_solution('Max Regret', chrom, tours, exec_time)
    except TimeoutException as e:
        print(f"  Max Regret TIMEOUT: {e}")
    except Exception as e:
        print(f"  Max Regret failed: {e}")

    # 3. Savings (Clarke-Wright)
    print("Running Savings (Clarke-Wright) heuristic...")
    start_time = time.time()
    try:
        with timeout(HEURISTIC_TIMEOUT):
            chrom, makespan, tours = generate_savings_solution(
                instance.dist, instance.proc, instance.depot,
                instance.m, instance.k, instance.num_customers,
                allow_mixed=allow_mixed
            )
            exec_time = time.time() - start_time
            add_solution('Savings', chrom, tours, exec_time)
    except TimeoutException as e:
        print(f"  Savings TIMEOUT: {e}")
    except Exception as e:
        print(f"  Savings failed: {e}")

    # 4. Greedy Defer variants
    print("Running Greedy Defer heuristics...")
    for mult in [5.0, 8.0, 10.0, 12.0, 15.0, 20.0]:
        start_time = time.time()
        try:
            with timeout(HEURISTIC_TIMEOUT):
                chrom, makespan = generate_greedy_defer_solution(
                    instance.dist, instance.proc, instance.depot,
                    instance.m, instance.k, instance.num_customers,
                    defer_multiplier=mult,
                    allow_mixed=allow_mixed
                )
                exec_time = time.time() - start_time
                add_solution(f'Greedy Defer {mult}x', chrom, exec_time=exec_time)
        except TimeoutException as e:
            print(f"  Greedy Defer {mult}x TIMEOUT: {e}")
        except Exception as e:
            print(f"  Greedy Defer {mult}x failed: {e}")

    # 5. 2-opt improved solutions
    # DISABLED: 2-opt is too slow for large datasets
    # Uncomment the section below to re-enable 2-opt heuristics
    """
    print("Running 2-opt improved heuristics...")
    for base in ['nearest_neighbor', 'max_regret', 'savings']:
        start_time = time.time()
        try:
            with timeout(HEURISTIC_TIMEOUT):
                chrom, makespan, tours = generate_2opt_improved_solution(
                    instance.dist, instance.proc, instance.depot,
                    instance.m, instance.k, instance.num_customers,
                    base_heuristic=base,
                    allow_mixed=allow_mixed
                )
                exec_time = time.time() - start_time
                name = f"2-opt ({base.replace('_', ' ').title()})"
                add_solution(name, chrom, tours, exec_time)
        except TimeoutException as e:
            print(f"  2-opt ({base}) TIMEOUT: {e}")
        except Exception as e:
            print(f"  2-opt ({base}) failed: {e}")
    """
    print("Skipping 2-opt heuristics (disabled for performance)")

    # Print summary table
    print("\n" + "=" * 70)
    print("HEURISTIC RESULTS SUMMARY")
    print("=" * 70)
    print(f"{'Heuristic':<35} {'Makespan':>12} {'Time (s)':>12}")
    print("-" * 70)

    # Sort by makespan
    all_solutions_sorted = sorted(all_solutions, key=lambda x: x['makespan'])
    for sol in all_solutions_sorted:
        marker = " <-- BEST" if sol['heuristic'] == best_heuristic else ""
        print(f"{sol['heuristic']:<35} {sol['makespan']:>12.2f} {sol['execution_time_seconds']:>12.2f}{marker}")

    print("-" * 70)
    print(f"{'BEST: ' + best_heuristic:<35} {best_makespan:>12.2f}")
    print("=" * 70 + "\n")

    if best_chrom is None:
        print("ERROR: All heuristics failed!")
        return float('inf')

    # Build final JSON with ALL solutions
    solution_json = {
        'problem': {
            'num_customers': instance.num_customers,
            'num_agents': instance.m,
            'resources_per_agent': instance.k,
            'depot': instance.depot
        },
        'best_solution': {
            'heuristic': best_heuristic,
            'makespan': float(best_makespan)
        },
        'all_heuristics': all_solutions_sorted  # All solutions sorted by makespan
    }

    # Save JSON
    with open(output_path, 'w') as f:
        json.dump(solution_json, f, indent=2)
    print(f"Saved {len(all_solutions)} heuristic solutions to: {output_path}")

    # Generate HTML Gantt for best solution
    if html_path:
        result_for_gantt = {
            'best_chromosome': best_chrom,
            'makespan': best_makespan,
            'solve_time': 0
        }
        generate_html_gantt(result_for_gantt, instance, html_path, title=f"Heuristic Solution ({best_heuristic})", allow_mixed=allow_mixed)
        print(f"Saved Gantt chart to: {html_path}")

    print("\n" + "=" * 70)
    print(f"HEURISTIC-ONLY COMPLETE: Best makespan = {best_makespan:.2f}")
    print("=" * 70 + "\n")

    return best_makespan
