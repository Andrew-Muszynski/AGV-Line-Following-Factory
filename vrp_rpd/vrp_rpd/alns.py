#!/usr/bin/env python3
"""
ALNS (Adaptive Large Neighborhood Search) for VRP-RPD

Metaheuristic that iteratively improves solutions using destroy-and-repair operations.
Can be used with warm-start from heuristic solutions or from scratch.
"""

import numpy as np
import random
import math
import time
from typing import Dict, Tuple, Optional, List
from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor, as_completed

from .decoder import decode_chromosome, compute_makespan_from_tours, compute_makespan_fast
from .utils import simulate_solution

# Optional Numba for performance
try:
    from numba import njit
    NUMBA_AVAILABLE = True
except ImportError:
    def njit(*args, **kwargs):
        def decorator(func):
            return func
        return decorator
    NUMBA_AVAILABLE = False


@dataclass
class ALNSConfig:
    """Configuration for ALNS algorithm"""
    max_iterations: int = 10000
    max_iterations_no_improve: int = 500
    max_time_seconds: int = 300
    min_destroy_pct: float = 0.1
    max_destroy_pct: float = 0.3
    initial_temperature: float = 100.0
    cooling_rate: float = 0.999
    num_parallel: int = 1
    allow_cross_vehicle: bool = True  # Allow different agents for drop/pick


def compute_makespan_for_validation(chrom: np.ndarray, dist: np.ndarray, proc: np.ndarray,
                                     depot: int, m: int, k: int, n: int) -> float:
    """
    Compute makespan using validated decoder.
    This ensures consistency with the rest of the codebase.
    """
    # Create minimal instance for decoder
    from .models import VRPRPDInstance
    instance = VRPRPDInstance(
        distance_matrix=dist,
        processing_times=proc,
        num_agents=m,
        resources_per_agent=k,
        depot=depot
    )

    # Use validated decoder and simulator
    tours = decode_chromosome(chrom, instance, allow_mixed=True)
    job_times, agent_tours, agent_completion_times, _ = simulate_solution(tours, instance)

    return max(agent_completion_times.values()) if agent_completion_times else float('inf')


def alns_worker(
    worker_id: int,
    dist: np.ndarray,
    proc: np.ndarray,
    customers: np.ndarray,
    depot: int,
    m: int,
    k: int,
    n: int,
    config_dict: dict,
    seed: int,
    initial_events: Optional[np.ndarray] = None,
    initial_drop: Optional[np.ndarray] = None,
    initial_pick: Optional[np.ndarray] = None
) -> Tuple[float, Dict]:
    """ALNS worker with cross-vehicle pickup support."""
    random.seed(seed + worker_id)
    np.random.seed(seed + worker_id)

    max_iter = config_dict.get('max_iterations', 10000)
    max_no_improve = config_dict.get('max_iterations_no_improve', 500)
    max_time = config_dict.get('max_time_seconds', 300)
    min_destroy = config_dict.get('min_destroy_pct', 0.1)
    max_destroy = config_dict.get('max_destroy_pct', 0.3)
    init_temp = config_dict.get('initial_temperature', 100.0)
    cooling = config_dict.get('cooling_rate', 0.999)
    allow_cross = config_dict.get('allow_cross_vehicle', True)

    customers_arr = np.array(customers, dtype=np.int32)
    dist_arr = np.array(dist, dtype=np.float64)
    proc_arr = np.array(proc, dtype=np.float64)

    max_events_per_vehicle = 2 * n

    # Initialize solution
    if initial_events is not None and initial_drop is not None and initial_pick is not None:
        events = initial_events.copy()
        event_counts = np.array([np.sum(events[v] >= 0) for v in range(m)], dtype=np.int32)
        drop_assign = initial_drop.copy()
        pick_assign = initial_pick.copy()
    else:
        # Generate initial with cross-vehicle pickups
        drop_assign = np.arange(n, dtype=np.int32) % m
        pick_assign = np.zeros(n, dtype=np.int32)

        events = np.full((m, max_events_per_vehicle), -1, dtype=np.int32)
        event_counts = np.zeros(m, dtype=np.int32)

        # Add dropoffs
        for idx in range(n):
            v = drop_assign[idx]
            events[v, event_counts[v]] = idx
            event_counts[v] += 1

        # Add pickups with 30% cross-vehicle if allowed
        for idx in range(n):
            drop_v = drop_assign[idx]
            if allow_cross and random.random() < 0.3 and m > 1:
                other_vehicles = [v for v in range(m) if v != drop_v]
                pick_v = random.choice(other_vehicles) if other_vehicles else drop_v
            else:
                pick_v = drop_v
            pick_assign[idx] = pick_v
            events[pick_v, event_counts[pick_v]] = idx + n
            event_counts[pick_v] += 1

    # Compute initial makespan - convert to chromosome and use validated decoder
    best_chrom = alns_format_to_chromosome(events, drop_assign, pick_assign, m, n)
    best_makespan = compute_makespan_for_validation(best_chrom, dist_arr, proc_arr, depot, m, k, n)
    best_events = events.copy()
    best_drop = drop_assign.copy()
    best_pick = pick_assign.copy()
    best_counts = event_counts.copy()

    current_events = events.copy()
    current_counts = event_counts.copy()
    current_drop = drop_assign.copy()
    current_pick = pick_assign.copy()
    current_makespan = best_makespan

    temperature = init_temp
    iterations_no_improve = 0
    start_time = time.time()

    for iteration in range(max_iter):
        if time.time() - start_time > max_time:
            break

        # Destroy
        n_remove = random.randint(
            max(1, int(n * min_destroy)),
            max(2, int(n * max_destroy))
        )

        assigned = [i for i in range(n) if current_drop[i] >= 0]
        if len(assigned) <= n_remove:
            continue

        removed = set(random.sample(assigned, n_remove))

        # Create candidate
        cand_events = current_events.copy()
        cand_counts = current_counts.copy()
        cand_drop = current_drop.copy()
        cand_pick = current_pick.copy()

        # Remove events
        for idx in removed:
            drop_v = cand_drop[idx]
            if drop_v >= 0:
                new_ev = [cand_events[drop_v, i] for i in range(cand_counts[drop_v])
                         if cand_events[drop_v, i] != idx]
                for i, e in enumerate(new_ev):
                    cand_events[drop_v, i] = e
                for i in range(len(new_ev), cand_counts[drop_v]):
                    cand_events[drop_v, i] = -1
                cand_counts[drop_v] = len(new_ev)

            pick_v = cand_pick[idx]
            if pick_v >= 0:
                new_ev = [cand_events[pick_v, i] for i in range(cand_counts[pick_v])
                         if cand_events[pick_v, i] != idx + n]
                for i, e in enumerate(new_ev):
                    cand_events[pick_v, i] = e
                for i in range(len(new_ev), cand_counts[pick_v]):
                    cand_events[pick_v, i] = -1
                cand_counts[pick_v] = len(new_ev)

            cand_drop[idx] = -1
            cand_pick[idx] = -1

        # Repair with cross-vehicle support
        removed_list = list(removed)
        random.shuffle(removed_list)

        for idx in removed_list:
            loc = customers_arr[idx]

            # Find best dropoff
            best_drop_cost = 1e18
            best_drop_v = 0
            best_drop_pos = 0

            for v in range(m):
                nc = cand_counts[v]
                for pos in range(nc + 1):
                    balance = 0
                    for i in range(pos):
                        e = cand_events[v, i]
                        if e >= 0:
                            balance += 1 if e < n else -1
                    if balance >= k:
                        continue

                    if pos == 0:
                        prev_loc = depot
                    else:
                        pe = cand_events[v, pos - 1]
                        prev_loc = customers_arr[pe if pe < n else pe - n]

                    if pos >= nc:
                        next_loc = depot
                    else:
                        ne = cand_events[v, pos]
                        next_loc = customers_arr[ne if ne < n else ne - n]

                    cost = dist_arr[prev_loc, loc] + dist_arr[loc, next_loc] - dist_arr[prev_loc, next_loc]

                    if cost < best_drop_cost:
                        best_drop_cost = cost
                        best_drop_v = v
                        best_drop_pos = pos

            # Insert dropoff
            drop_v = best_drop_v
            drop_pos = best_drop_pos
            for i in range(cand_counts[drop_v], drop_pos, -1):
                cand_events[drop_v, i] = cand_events[drop_v, i - 1]
            cand_events[drop_v, drop_pos] = idx
            cand_counts[drop_v] += 1
            cand_drop[idx] = drop_v

            # Find best pickup (ANY vehicle if allowed)
            best_pick_cost = 1e18
            best_pick_v = 0
            best_pick_pos = 0

            vehicles_to_try = range(m) if allow_cross else [drop_v]

            for v in vehicles_to_try:
                nc = cand_counts[v]

                if v == drop_v:
                    drop_idx = -1
                    for i in range(nc):
                        if cand_events[v, i] == idx:
                            drop_idx = i
                            break
                    start_pos = drop_idx + 1 if drop_idx >= 0 else 0
                else:
                    start_pos = 0

                for pos in range(start_pos, nc + 1):
                    if pos == 0:
                        prev_loc = depot
                    else:
                        pe = cand_events[v, pos - 1]
                        prev_loc = customers_arr[pe if pe < n else pe - n]

                    if pos >= nc:
                        next_loc = depot
                    else:
                        ne = cand_events[v, pos]
                        next_loc = customers_arr[ne if ne < n else ne - n]

                    cost = dist_arr[prev_loc, loc] + dist_arr[loc, next_loc] - dist_arr[prev_loc, next_loc]

                    # Bonus for cross-vehicle if allowed
                    if allow_cross and v != drop_v:
                        cost *= 0.85

                    if cost < best_pick_cost:
                        best_pick_cost = cost
                        best_pick_v = v
                        best_pick_pos = pos

            # Insert pickup
            pick_v = best_pick_v
            pick_pos = best_pick_pos
            for i in range(cand_counts[pick_v], pick_pos, -1):
                cand_events[pick_v, i] = cand_events[pick_v, i - 1]
            cand_events[pick_v, pick_pos] = idx + n
            cand_counts[pick_v] += 1
            cand_pick[idx] = pick_v

        # Evaluate - convert to chromosome and use validated makespan
        cand_chrom = alns_format_to_chromosome(cand_events, cand_drop, cand_pick, m, n)
        cand_makespan = compute_makespan_for_validation(cand_chrom, dist_arr, proc_arr, depot, m, k, n)

        if cand_makespan <= 0 or cand_makespan > 1e17:
            iterations_no_improve += 1
            continue

        # Accept/reject
        accept = False
        if cand_makespan < best_makespan:
            best_makespan = cand_makespan
            best_events = cand_events.copy()
            best_drop = cand_drop.copy()
            best_pick = cand_pick.copy()
            best_counts = cand_counts.copy()
            accept = True
            iterations_no_improve = 0

            cross = sum(1 for i in range(n) if cand_drop[i] != cand_pick[i])
            elapsed = time.time() - start_time
            print(f"  W{worker_id} iter {iteration}: best={best_makespan:.2f}, cross={cross}, t={elapsed:.1f}s")

        if cand_makespan < current_makespan:
            accept = True
            iterations_no_improve = 0
        elif random.random() < math.exp(-(cand_makespan - current_makespan) / max(temperature, 1e-10)):
            accept = True
            iterations_no_improve += 1
        else:
            iterations_no_improve += 1

        if accept:
            current_events = cand_events
            current_counts = cand_counts
            current_drop = cand_drop
            current_pick = cand_pick
            current_makespan = cand_makespan

        temperature *= cooling

        if iterations_no_improve >= max_no_improve:
            break

    cross_vehicle = sum(1 for i in range(n) if best_drop[i] != best_pick[i])
    elapsed = time.time() - start_time
    print(f"  W{worker_id} done: makespan={best_makespan:.2f}, cross={cross_vehicle}, t={elapsed:.1f}s")

    return best_makespan, {
        'events': best_events.tolist(),
        'drop_assign': best_drop.tolist(),
        'pick_assign': best_pick.tolist(),
        'cross_vehicle': cross_vehicle
    }


def chromosome_to_alns_format(
    chromosome: np.ndarray,
    instance,
    allow_mixed: bool = True
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Convert chromosome to ALNS format (events, drop_assign, pick_assign).

    Args:
        chromosome: BRKGA chromosome [n x 4] - [drop_agent, pick_agent, drop_key, pick_key]
        instance: VRPRPDInstance
        allow_mixed: Whether to allow cross-vehicle pickups

    Returns:
        (events, drop_assign, pick_assign) as numpy arrays
    """
    n = len(chromosome)
    m = instance.m
    k = instance.k

    # Decode chromosome to tours
    tours = decode_chromosome(chromosome, instance, allow_mixed=allow_mixed)

    # Build event arrays
    max_events = 2 * n
    events = np.full((m, max_events), -1, dtype=np.int32)
    event_counts = np.zeros(m, dtype=np.int32)
    drop_assign = np.full(n, -1, dtype=np.int32)
    pick_assign = np.full(n, -1, dtype=np.int32)

    # Map customer location to index
    if instance.depot == 0:
        customers = list(range(1, len(instance.dist)))
    else:
        customers = [i for i in range(len(instance.dist)) if i != instance.depot]

    loc_to_idx = {loc: idx for idx, loc in enumerate(customers)}

    # Extract from tours
    for agent in range(m):
        tour = tours.get(agent, [])
        for loc, op, c_idx in tour:
            if loc in loc_to_idx:
                idx = loc_to_idx[loc]
                if op == 'D':
                    events[agent, event_counts[agent]] = idx
                    drop_assign[idx] = agent
                else:  # op == 'P'
                    events[agent, event_counts[agent]] = idx + n
                    pick_assign[idx] = agent
                event_counts[agent] += 1

    return events, drop_assign, pick_assign


def alns_format_to_chromosome(
    events: np.ndarray,
    drop_assign: np.ndarray,
    pick_assign: np.ndarray,
    m: int,
    n: int
) -> np.ndarray:
    """
    Convert ALNS format back to chromosome.

    Args:
        events: Event arrays [m x max_events]
        drop_assign: Dropoff agent for each customer [n]
        pick_assign: Pickup agent for each customer [n]
        m: Number of agents
        n: Number of customers

    Returns:
        chromosome: [n x 4] array
    """
    chromosome = np.zeros((n, 4), dtype=np.float32)

    # Count events per agent
    agent_tour_lengths = {a: 0 for a in range(m)}
    for v in range(m):
        for e in events[v]:
            if e >= 0:
                agent_tour_lengths[v] += 1

    # Build position maps
    drop_position = {}
    pick_position = {}

    for v in range(m):
        pos = 0
        for e in events[v]:
            if e < 0:
                break
            if e < n:  # Dropoff
                drop_position[e] = pos
            else:  # Pickup
                pick_position[e - n] = pos
            pos += 1

    # Build chromosome
    for c_idx in range(n):
        da = drop_assign[c_idx]
        pa = pick_assign[c_idx]

        if da >= 0 and pa >= 0:
            da_tour_len = max(agent_tour_lengths.get(da, 1), 1)
            pa_tour_len = max(agent_tour_lengths.get(pa, 1), 1)

            chromosome[c_idx, 0] = da
            chromosome[c_idx, 1] = pa
            chromosome[c_idx, 2] = drop_position.get(c_idx, 0) / da_tour_len
            chromosome[c_idx, 3] = pick_position.get(c_idx, 0) / pa_tour_len

    return chromosome


def run_alns(
    instance,
    config: ALNSConfig,
    initial_chromosome: Optional[np.ndarray] = None,
    seed: int = 42
) -> Tuple[np.ndarray, float, Dict]:
    """
    Run ALNS optimization.

    Args:
        instance: VRPRPDInstance
        config: ALNSConfig
        initial_chromosome: Optional warm-start chromosome [n x 4]
        seed: Random seed

    Returns:
        (best_chromosome, best_makespan, metadata)
    """
    print("\n" + "=" * 60)
    print("ALNS for VRP-RPD")
    print("=" * 60)
    print(f"Customers: {instance.num_customers}, Vehicles: {instance.m}, Capacity: {instance.k}")
    print(f"Parallel workers: {config.num_parallel}")
    print(f"Numba JIT: {NUMBA_AVAILABLE}")
    print(f"Allow cross-vehicle: {config.allow_cross_vehicle}")

    # Prepare data
    if instance.depot == 0:
        customers = np.array(list(range(1, len(instance.dist))), dtype=np.int32)
    else:
        customers = np.array([i for i in range(len(instance.dist)) if i != instance.depot], dtype=np.int32)

    n = len(customers)

    # Convert initial solution if provided
    init_events, init_drop, init_pick = None, None, None
    if initial_chromosome is not None:
        print(f"Using warm-start from provided chromosome")
        init_events, init_drop, init_pick = chromosome_to_alns_format(
            initial_chromosome, instance, allow_mixed=config.allow_cross_vehicle
        )

    # Prepare config dict
    config_dict = {
        'max_iterations': config.max_iterations,
        'max_iterations_no_improve': config.max_iterations_no_improve,
        'max_time_seconds': config.max_time_seconds,
        'min_destroy_pct': config.min_destroy_pct,
        'max_destroy_pct': config.max_destroy_pct,
        'initial_temperature': config.initial_temperature,
        'cooling_rate': config.cooling_rate,
        'allow_cross_vehicle': config.allow_cross_vehicle,
    }

    start_time = time.time()

    if config.num_parallel <= 1:
        best_makespan, best_dict = alns_worker(
            0, instance.dist, instance.proc, customers,
            instance.depot, instance.m, instance.k, n,
            config_dict, seed,
            init_events, init_drop, init_pick
        )
    else:
        best_makespan = float('inf')
        best_dict = None

        with ProcessPoolExecutor(max_workers=config.num_parallel) as executor:
            futures = [
                executor.submit(
                    alns_worker, i, instance.dist, instance.proc, customers,
                    instance.depot, instance.m, instance.k, n,
                    config_dict, seed + i * 1000,
                    init_events, init_drop, init_pick
                )
                for i in range(config.num_parallel)
            ]

            for f in as_completed(futures):
                makespan, sol_dict = f.result()
                if makespan < best_makespan:
                    best_makespan = makespan
                    best_dict = sol_dict

    elapsed = time.time() - start_time

    print("\n" + "-" * 60)
    print(f"ALNS Final: {best_makespan:.2f}")
    print(f"Cross-vehicle: {best_dict.get('cross_vehicle', 0)}")
    print(f"Time: {elapsed:.1f}s")
    print("=" * 60 + "\n")

    # Convert back to chromosome
    events_array = np.array(best_dict['events'], dtype=np.int32)
    drop_array = np.array(best_dict['drop_assign'], dtype=np.int32)
    pick_array = np.array(best_dict['pick_assign'], dtype=np.int32)

    best_chromosome = alns_format_to_chromosome(
        events_array, drop_array, pick_array, instance.m, n
    )

    metadata = {
        'solver': 'ALNS',
        'solve_time': elapsed,
        'cross_vehicle_pickups': best_dict.get('cross_vehicle', 0),
        'warm_start': initial_chromosome is not None
    }

    return best_chromosome, best_makespan, metadata
