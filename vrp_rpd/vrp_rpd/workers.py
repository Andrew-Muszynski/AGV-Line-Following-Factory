#!/usr/bin/env python3
"""
VRP-RPD Worker Functions for Multiprocessing

Contains GPU and CPU worker functions for parallel island model execution.
"""

import time
import numpy as np
import torch
import torch.multiprocessing as mp
from typing import List
import json


from .models import VRPRPDInstance
from .islands import GPUIsland, CPUIsland


def gpu_worker(
    gpu_id: int,
    instance_dict: dict,
    population_size: int,
    gens_per_cycle: int,
    total_cycles: int,
    allow_mixed: bool,
    result_queue: mp.Queue,
    solution_queue: mp.Queue,
    candidate_pipe,
    use_gp: bool,
    use_gene_injection: bool,
    warm_start: List[np.ndarray] = None,
    mutation_rate: float = 0.1,
    elite_size: int = 10,
    crossover_rate: float = 0.8,
    mutation_strength: float = 0.02,
    global_best_fitness: mp.Value = None,
    global_best_lock: mp.Lock = None,
    output_json_path: str = None,
    checkpoint_interval: int = 0,
    checkpoint_prefix: str = ""
):
    """GPU worker process"""
    try:
        device = torch.device(f'cuda:{gpu_id}')
        torch.cuda.set_device(device)
        
        instance = VRPRPDInstance(
            distance_matrix=instance_dict['dist'],
            processing_times=instance_dict['proc'],
            num_agents=instance_dict['m'],
            resources_per_agent=instance_dict['k'],
            depot=instance_dict['depot']
        )
        
        island = GPUIsland(
            instance=instance,
            device=device,
            population_size=population_size,
            elite_size=elite_size,
            mutation_rate=mutation_rate,
            crossover_rate=crossover_rate,
            allow_mixed=allow_mixed,
            mutation_strength=mutation_strength
        )
        
        if warm_start:
            island.inject_warm_start(warm_start)
            print(f"[GPU {gpu_id}] Warm start: {len(warm_start)} chromosomes", flush=True)
        
        start_time = time.time()
        total_injected = 0
        gen = 0
        history = []
        my_best_fitness = float('inf')  # Track this worker's best

        for cycle in range(total_cycles):
            for g in range(gens_per_cycle):
                island.evolve_generation()

                # Check every 100 generations for new best and save immediately
                if (gen + g) % 100 == 0:
                    elapsed = time.time() - start_time
                    current_gen = gen + g

                    # Check if we found a new global best
                    if island.best_fitness < my_best_fitness:
                        my_best_fitness = island.best_fitness

                        # Check and update global best if we have shared state
                        if global_best_fitness is not None and global_best_lock is not None:
                            with global_best_lock:
                                if island.best_fitness < global_best_fitness.value:
                                    global_best_fitness.value = island.best_fitness
                                    print(f"\n*** [GPU {gpu_id}] NEW GLOBAL BEST: {island.best_fitness:.2f} at gen {current_gen} ***", flush=True)

                                    # Save to JSON immediately
                                    if output_json_path and island.best_chromosome is not None:
                                        try:
                                            solution_data = {
                                                'makespan': float(island.best_fitness),
                                                'chromosome': island.best_chromosome.tolist(),
                                                'source': f'gpu_{gpu_id}',
                                                'generation': current_gen,
                                                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
                                            }
                                            with open(output_json_path, 'w') as f:
                                                json.dump(solution_data, f, indent=2)
                                            print(f"    Saved to {output_json_path}\n", flush=True)
                                        except Exception as e:
                                            print(f"    Warning: Could not save JSON: {e}\n", flush=True)

                    print(f"[GPU {gpu_id}] Gen {current_gen:5d} | Best: {island.best_fitness:8.2f} | {elapsed:.1f}s", flush=True)
                    history.append((current_gen, island.best_fitness))

                    # Save checkpoint if enabled and at checkpoint interval
                    if checkpoint_interval > 0 and current_gen > 0 and current_gen % checkpoint_interval == 0:
                        if checkpoint_prefix and island.best_chromosome is not None:
                            try:
                                from .decoder import decode_chromosome
                                from .utils import simulate_solution

                                checkpoint_filename = f"{checkpoint_prefix}{current_gen}.json"

                                # Decode and simulate solution for full details
                                tours = decode_chromosome(island.best_chromosome, instance, allow_mixed=allow_mixed)
                                job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
                                    tours, instance
                                )
                                correct_makespan = max(agent_completion_times.values()) if agent_completion_times else float('inf')

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

                                checkpoint_data = {
                                    'problem': {
                                        'num_customers': instance.num_customers,
                                        'num_agents': instance.m,
                                        'resources_per_agent': instance.k,
                                        'depot': instance.depot
                                    },
                                    'solution': {
                                        'makespan': float(correct_makespan),
                                        'generation': current_gen,
                                        'source': f'gpu_{gpu_id}',
                                        'routes': routes_json,
                                        'jobs': jobs_json
                                    },
                                    'checkpoint': {
                                        'generation': current_gen,
                                        'elapsed_time': float(elapsed)
                                    }
                                }

                                with open(checkpoint_filename, 'w') as f:
                                    json.dump(checkpoint_data, f, indent=2)
                                print(f"    [Checkpoint] Saved to {checkpoint_filename}", flush=True)

                            except Exception as e:
                                print(f"    [Checkpoint] Warning: Could not save: {e}", flush=True)

                    # Send intermediate result every 100 gens (not just every cycle)
                    try:
                        result_queue.put({
                            'worker_id': f'gpu_{gpu_id}',
                            'best_fitness': island.best_fitness,
                            'best_chromosome': island.best_chromosome.copy() if island.best_chromosome is not None else None,
                            'generation': current_gen,
                            'intermediate': True
                        }, timeout=0.1)
                    except:
                        pass

            gen += gens_per_cycle

            # CRITICAL: Send intermediate best solution via result_queue EVERY cycle
            # This ensures we don't lose the best solution if worker is terminated early
            try:
                result_queue.put({
                    'worker_id': f'gpu_{gpu_id}',
                    'best_fitness': island.best_fitness,
                    'best_chromosome': island.best_chromosome.copy() if island.best_chromosome is not None else None,
                    'generation': gen,
                    'intermediate': True  # Mark as intermediate
                }, timeout=1.0)  # Wait up to 1 second instead of silently dropping
            except:
                pass  # Queue full after timeout, skip this update

            if use_gp:
                island._evaluate_population()
                all_sols = island.get_all_solutions()

                # For large instances, send fewer solutions to avoid pipe blocking
                num_sols_to_send = 20 if instance.num_customers > 500 else 100
                all_sols_sorted = sorted(all_sols, key=lambda x: x['fitness'])[:num_sols_to_send]

                # Send solutions to main process for GP analysis (queue never blocks!)
                try:
                    solution_queue.put({'solutions': all_sols_sorted}, block=False)
                except:
                    pass  # Queue full (very unlikely), skip this cycle

                if use_gene_injection:
                    try:
                        if candidate_pipe.poll(0.1):
                            response = candidate_pipe.recv()
                            if response and 'candidates' in response:
                                n = island.inject_candidates(response['candidates'])
                                total_injected += n
                                print(f"[GPU {gpu_id}] Gene injection: {n} candidates injected at gen {gen} (total so far: {total_injected})", flush=True)
                    except Exception as e:
                        pass

        elapsed = time.time() - start_time
        island._evaluate_population()

        print(f"[GPU {gpu_id}] Evolution complete. Sending final results...", flush=True)
        result_queue.put({
            'worker_id': f'gpu_{gpu_id}',
            'best_fitness': island.best_fitness,
            'best_chromosome': island.best_chromosome,
            'history': history,
            'total_injected': total_injected
        })
        
    except Exception as e:
        import traceback
        print(f"[GPU {gpu_id}] ERROR: {e}", flush=True)
        traceback.print_exc()
        result_queue.put({'worker_id': f'gpu_{gpu_id}', 'best_fitness': float('inf'), 'error': str(e)})


def cpu_worker(
    cpu_id: int,
    instance_dict: dict,
    population_size: int,
    gens_per_cycle: int,
    total_cycles: int,
    allow_mixed: bool,
    result_queue: mp.Queue,
    solution_queue: mp.Queue,
    candidate_pipe,
    use_gp: bool,
    use_gene_injection: bool,
    warm_start: List[np.ndarray] = None,
    mutation_rate: float = 0.15,
    elite_size: int = 10,
    crossover_rate: float = 0.8,
    mutation_strength: float = 0.02,
    global_best_fitness: mp.Value = None,
    global_best_lock: mp.Lock = None,
    output_json_path: str = None
):
    """CPU worker process"""
    try:
        instance = VRPRPDInstance(
            distance_matrix=instance_dict['dist'],
            processing_times=instance_dict['proc'],
            num_agents=instance_dict['m'],
            resources_per_agent=instance_dict['k'],
            depot=instance_dict['depot']
        )

        island = CPUIsland(
            instance=instance,
            population_size=population_size,
            elite_size=elite_size,
            mutation_rate=mutation_rate,
            crossover_rate=crossover_rate,
            allow_mixed=allow_mixed,
            mutation_strength=mutation_strength
        )

        if warm_start:
            island.inject_warm_start(warm_start)
            print(f"[CPU {cpu_id}] Warm start: {len(warm_start)} chromosomes", flush=True)

        start_time = time.time()
        total_injected = 0
        gen = 0
        history = []
        my_best_fitness = float('inf')  # Track this worker's best

        for cycle in range(total_cycles):
            for g in range(gens_per_cycle):
                island.evolve_generation()

                # Check every 500 generations for new best and save immediately
                if (gen + g) % 500 == 0:
                    elapsed = time.time() - start_time
                    current_gen = gen + g

                    # Check if we found a new global best
                    if island.best_fitness < my_best_fitness:
                        my_best_fitness = island.best_fitness

                        # Check and update global best if we have shared state
                        if global_best_fitness is not None and global_best_lock is not None:
                            with global_best_lock:
                                if island.best_fitness < global_best_fitness.value:
                                    global_best_fitness.value = island.best_fitness
                                    print(f"\n*** [CPU {cpu_id}] NEW GLOBAL BEST: {island.best_fitness:.2f} at gen {current_gen} ***", flush=True)

                                    # Save to JSON immediately
                                    if output_json_path and island.best_chromosome is not None:
                                        try:
                                            solution_data = {
                                                'makespan': float(island.best_fitness),
                                                'chromosome': island.best_chromosome.tolist(),
                                                'source': f'cpu_{cpu_id}',
                                                'generation': current_gen,
                                                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
                                            }
                                            with open(output_json_path, 'w') as f:
                                                json.dump(solution_data, f, indent=2)
                                            print(f"    Saved to {output_json_path}\n", flush=True)
                                        except Exception as e:
                                            print(f"    Warning: Could not save JSON: {e}\n", flush=True)

                    print(f"[CPU {cpu_id}] Gen {current_gen:5d} | Best: {island.best_fitness:8.2f} | {elapsed:.1f}s", flush=True)
                    history.append((current_gen, island.best_fitness))

                    # Send intermediate result
                    try:
                        result_queue.put({
                            'worker_id': f'cpu_{cpu_id}',
                            'best_fitness': island.best_fitness,
                            'best_chromosome': island.best_chromosome.copy() if island.best_chromosome is not None else None,
                            'generation': current_gen,
                            'intermediate': True
                        }, timeout=0.1)
                    except:
                        pass

            gen += gens_per_cycle

            # CRITICAL: Send intermediate best solution via result_queue EVERY cycle
            # This ensures we don't lose the best solution if worker is terminated early
            try:
                result_queue.put({
                    'worker_id': f'cpu_{cpu_id}',
                    'best_fitness': island.best_fitness,
                    'best_chromosome': island.best_chromosome.copy() if island.best_chromosome is not None else None,
                    'generation': gen,
                    'intermediate': True  # Mark as intermediate
                }, timeout=1.0)
            except:
                pass  # Queue full after timeout, skip this update

            if use_gp:
                island._evaluate_population()
                all_sols = island.get_all_solutions()

                # For large instances, send fewer solutions to avoid pipe blocking
                num_sols_to_send = 20 if instance.num_customers > 500 else 100
                all_sols_sorted = sorted(all_sols, key=lambda x: x['fitness'])[:num_sols_to_send]

                # Send solutions to main process for GP analysis (queue never blocks!)
                try:
                    solution_queue.put({'solutions': all_sols_sorted}, block=False)
                except:
                    pass  # Queue full (very unlikely), skip this cycle

                if use_gene_injection:
                    try:
                        if candidate_pipe.poll(0.1):
                            response = candidate_pipe.recv()
                            if response and 'candidates' in response:
                                n = island.inject_candidates(response['candidates'])
                                total_injected += n
                                print(f"[CPU {cpu_id}] Gene injection: {n} candidates injected at gen {gen} (total so far: {total_injected})", flush=True)
                    except Exception as e:
                        pass

        elapsed = time.time() - start_time
        island._evaluate_population()

        print(f"[CPU {cpu_id}] Evolution complete. Sending final results...", flush=True)
        result_queue.put({
            'worker_id': f'cpu_{cpu_id}',
            'best_fitness': island.best_fitness,
            'best_chromosome': island.best_chromosome,
            'history': history,
            'total_injected': total_injected
        })
        
    except Exception as e:
        import traceback
        print(f"[CPU {cpu_id}] ERROR: {e}", flush=True)
        traceback.print_exc()
        result_queue.put({'worker_id': f'cpu_{cpu_id}', 'best_fitness': float('inf'), 'error': str(e)})
