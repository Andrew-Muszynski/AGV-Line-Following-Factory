#!/usr/bin/env python3
"""
VRP-RPD Main Solver

Contains the VRPRPDSolver class that orchestrates the optimization process.
"""

import time
import json
import numpy as np
import torch
import torch.multiprocessing as mp
from typing import Dict, List, Optional, Tuple

from .models import VRPRPDInstance
from .heuristics import (
    generate_nearest_neighbor_solution,
    generate_max_regret_solution,
    generate_greedy_defer_solution,
    generate_savings_solution,
    generate_2opt_improved_solution
)
from .decoder import compute_makespan_fast, decode_chromosome
from .genetic_analyzer import GeneticAnalyzer
from .workers import gpu_worker, cpu_worker
from .utils import simulate_solution

# Optional CuPy import
try:
    import cupy as cp
    HAS_CUPY = True
except ImportError:
    HAS_CUPY = False


class VRPRPDSolver:
    """Main solver with all options"""
    
    def __init__(
        self,
        instance: VRPRPDInstance,
        num_gpus: Optional[int] = None,
        num_cpu_workers: int = 0,
        population_per_gpu: int = 512,
        population_per_cpu: int = 256,
        total_generations: int = 5000,
        gens_per_cycle: int = 200,
        candidates_per_injection: int = 20,
        heuristic: int = 0,
        use_gp: bool = True,
        use_warm: bool = True,
        use_gene_injection: bool = True,
        use_blocks: bool = True,
        use_fft: bool = False,
        use_similarity: bool = False,
        allow_mixed: bool = True,
        json_solution_chromosome: Optional[np.ndarray] = None,
        checkpoint_interval: int = 0,
        checkpoint_prefix: str = "",
        # Diversity parameters
        mutation_rate_base: Optional[float] = None,
        mutation_rate_range: float = 0.18,
        mutation_strength: float = 0.02,
        elite_size: int = 10,
        crossover_rate: float = 0.8,
        restart_stagnation: int = 0
    ):
        self.instance = instance
        self.num_gpus = num_gpus if num_gpus is not None else torch.cuda.device_count()
        self.num_cpu_workers = num_cpu_workers
        self.pop_per_gpu = population_per_gpu
        self.pop_per_cpu = population_per_cpu
        self.total_generations = total_generations
        self.gens_per_cycle = gens_per_cycle
        self.candidates_per_injection = candidates_per_injection
        self.heuristic = heuristic
        self.use_gp = use_gp
        self.use_warm = use_warm
        self.use_gene_injection = use_gene_injection
        self.use_blocks = use_blocks
        self.use_fft = use_fft
        self.use_similarity = use_similarity
        self.allow_mixed = allow_mixed
        self.json_solution_chromosome = json_solution_chromosome
        self.checkpoint_interval = checkpoint_interval
        self.checkpoint_prefix = checkpoint_prefix

        # Diversity parameters
        self.mutation_rate_base = mutation_rate_base
        self.mutation_rate_range = mutation_rate_range
        self.mutation_strength = mutation_strength
        self.elite_size = elite_size
        self.crossover_rate = crossover_rate
        self.restart_stagnation = restart_stagnation

        self.total_cycles = total_generations // gens_per_cycle
        self.total_workers = self.num_gpus + self.num_cpu_workers
    
    def _generate_warm_start(self) -> Tuple[List[np.ndarray], float, np.ndarray, Dict]:
        """
        Generate warm start chromosomes from heuristics and JSON solution.

        IMPORTANT: Uses build_chromosome_from_tours() to create chromosomes that
        the decoder will interpret correctly (matching the heuristic's execution order).

        Returns:
            - List of chromosomes for warm start
            - Best heuristic makespan
            - Best heuristic chromosome
            - Best heuristic tours (for HTML if needed)
        """
        warm_chroms = []
        best_heuristic_makespan = float('inf')
        best_heuristic_chrom = None
        best_heuristic_tours = None
        best_heuristic_name = "None"

        results = []  # Track all results for summary

        # Add JSON solution if provided (highest priority)
        if self.json_solution_chromosome is not None:
            print("Using solution from JSON file as warm start...")
            warm_chroms.append(self.json_solution_chromosome.copy())
            makespan = compute_makespan_fast(self.json_solution_chromosome, self.instance, self.allow_mixed)
            print(f"  JSON solution: {makespan:.2f}")
            results.append(('JSON Solution', makespan))
            if makespan < best_heuristic_makespan:
                best_heuristic_makespan = makespan
                best_heuristic_chrom = self.json_solution_chromosome.copy()
                best_heuristic_tours = None
                best_heuristic_name = "JSON Solution"

            for _ in range(5):
                perturbed = self.json_solution_chromosome.copy()
                perturbed += np.random.randn(*perturbed.shape).astype(np.float32) * 0.03
                perturbed[:, 0] = np.clip(np.round(perturbed[:, 0]), 0, self.instance.m - 1)
                perturbed[:, 1] = np.clip(np.round(perturbed[:, 1]), 0, self.instance.m - 1)
                perturbed[:, 3] = np.maximum(perturbed[:, 3], perturbed[:, 2] + 0.01)
                warm_chroms.append(perturbed)

            # Skip expensive heuristic re-computation when JSON is provided
            print("Skipping heuristic re-computation (using JSON solution + perturbations)")
            print(f"\nGenerated {len(warm_chroms)} warm start chromosomes from JSON")
            return warm_chroms, best_heuristic_makespan, best_heuristic_chrom, best_heuristic_tours

        print("Running heuristics for warm start...")

        # 1. Nearest Neighbor
        try:
            chrom, makespan, tours = generate_nearest_neighbor_solution(
                self.instance.dist, self.instance.proc, self.instance.depot,
                self.instance.m, self.instance.k, self.instance.num_customers,
                allow_mixed=self.allow_mixed
            )
            decoder_makespan = compute_makespan_fast(chrom, self.instance, self.allow_mixed)
            print(f"  Nearest Neighbor: {decoder_makespan:.2f}")
            results.append(('Nearest Neighbor', decoder_makespan))
            warm_chroms.append(chrom)
            if decoder_makespan < best_heuristic_makespan:
                best_heuristic_makespan = decoder_makespan
                best_heuristic_chrom = chrom.copy()
                best_heuristic_tours = tours
                best_heuristic_name = "Nearest Neighbor"
        except Exception as e:
            print(f"  Nearest Neighbor failed: {e}")

        # 2. Max Regret
        try:
            chrom, makespan, tours = generate_max_regret_solution(
                self.instance.dist, self.instance.proc, self.instance.depot,
                self.instance.m, self.instance.k, self.instance.num_customers,
                allow_mixed=self.allow_mixed
            )
            decoder_makespan = compute_makespan_fast(chrom, self.instance, self.allow_mixed)
            print(f"  Max Regret: {decoder_makespan:.2f}")
            results.append(('Max Regret', decoder_makespan))
            warm_chroms.append(chrom)
            if decoder_makespan < best_heuristic_makespan:
                best_heuristic_makespan = decoder_makespan
                best_heuristic_chrom = chrom.copy()
                best_heuristic_tours = tours
                best_heuristic_name = "Max Regret"
        except Exception as e:
            print(f"  Max Regret failed: {e}")

        # 3. Savings
        try:
            chrom, makespan, tours = generate_savings_solution(
                self.instance.dist, self.instance.proc, self.instance.depot,
                self.instance.m, self.instance.k, self.instance.num_customers,
                allow_mixed=self.allow_mixed
            )
            decoder_makespan = compute_makespan_fast(chrom, self.instance, self.allow_mixed)
            print(f"  Savings: {decoder_makespan:.2f}")
            results.append(('Savings', decoder_makespan))
            warm_chroms.append(chrom)
            if decoder_makespan < best_heuristic_makespan:
                best_heuristic_makespan = decoder_makespan
                best_heuristic_chrom = chrom.copy()
                best_heuristic_tours = tours
                best_heuristic_name = "Savings"
        except Exception as e:
            print(f"  Savings failed: {e}")

        # 4. Greedy Defer variants
        if self.use_warm:
            for mult in [5.0, 8.0, 10.0, 12.0, 15.0]:
                try:
                    chrom, makespan = generate_greedy_defer_solution(
                        self.instance.dist, self.instance.proc, self.instance.depot,
                        self.instance.m, self.instance.k, self.instance.num_customers,
                        defer_multiplier=mult,
                        allow_mixed=self.allow_mixed
                    )
                    name = f"Greedy Defer {mult}x"
                    print(f"  {name}: {makespan:.2f}")
                    results.append((name, makespan))
                    warm_chroms.append(chrom)
                    if makespan < best_heuristic_makespan:
                        best_heuristic_makespan = makespan
                        best_heuristic_chrom = chrom.copy()
                        best_heuristic_tours = None
                        best_heuristic_name = name

                    for _ in range(2):
                        perturbed = chrom.copy()
                        perturbed += np.random.randn(*perturbed.shape).astype(np.float32) * 0.05
                        perturbed[:, 0] = np.clip(np.round(perturbed[:, 0]), 0, self.instance.m - 1)
                        perturbed[:, 1] = np.clip(np.round(perturbed[:, 1]), 0, self.instance.m - 1)
                        perturbed[:, 3] = np.maximum(perturbed[:, 3], perturbed[:, 2] + 0.01)
                        warm_chroms.append(perturbed)
                except Exception as e:
                    print(f"  Greedy Defer {mult}x failed: {e}")

        # 5. 2-opt improved (only run if we have time - skip for now in warm start)
        # Can be enabled for more thorough search
        try:
            chrom, makespan, tours = generate_2opt_improved_solution(
                self.instance.dist, self.instance.proc, self.instance.depot,
                self.instance.m, self.instance.k, self.instance.num_customers,
                base_heuristic='nearest_neighbor',
                allow_mixed=self.allow_mixed
            )
            name = "2-opt (NN)"
            decoder_makespan = compute_makespan_fast(chrom, self.instance, self.allow_mixed)
            print(f"  {name}: {decoder_makespan:.2f}")
            results.append((name, decoder_makespan))
            warm_chroms.append(chrom)
            if decoder_makespan < best_heuristic_makespan:
                best_heuristic_makespan = decoder_makespan
                best_heuristic_chrom = chrom.copy()
                best_heuristic_tours = tours
                best_heuristic_name = name
        except Exception as e:
            print(f"  2-opt (NN) failed: {e}")

        # Print summary table
        if results:
            print("\n" + "-" * 50)
            print(f"{'Heuristic':<25} {'Makespan':>12}")
            print("-" * 50)
            results_sorted = sorted(results, key=lambda x: x[1])
            for name, ms in results_sorted:
                marker = " <-- BEST" if name == best_heuristic_name else ""
                print(f"{name:<25} {ms:>12.2f}{marker}")
            print("-" * 50)

        if best_heuristic_makespan < float('inf'):
            print(f"\n  *** Best heuristic: {best_heuristic_name} = {best_heuristic_makespan:.2f} ***\n")

            # CRITICAL: Put best chromosome FIRST and add multiple copies
            final_warm = [best_heuristic_chrom.copy()]

            for _ in range(min(10, self.num_gpus * 2)):
                perturbed = best_heuristic_chrom.copy()
                perturbed[:, 2] += np.random.randn(self.instance.num_customers).astype(np.float32) * 0.001
                perturbed[:, 3] += np.random.randn(self.instance.num_customers).astype(np.float32) * 0.001
                perturbed[:, 3] = np.maximum(perturbed[:, 3], perturbed[:, 2] + 0.001)
                final_warm.append(perturbed)

            for c in warm_chroms:
                if c is not best_heuristic_chrom:
                    final_warm.append(c)

            warm_chroms = final_warm

        return warm_chroms, best_heuristic_makespan, best_heuristic_chrom, best_heuristic_tours

    def _save_checkpoint(self, results: List[Dict], current_gen: int):
        """Save intermediate checkpoint solution"""
        if not results:
            return

        # Find best result so far
        best_result = min(results, key=lambda r: r.get('best_fitness', float('inf')))
        best_fitness = best_result.get('best_fitness', float('inf'))
        best_chrom = best_result.get('best_chromosome')

        if best_chrom is None or best_fitness >= float('inf'):
            return

        # Generate checkpoint letter (A, B, C, ...)
        checkpoint_number = current_gen // self.checkpoint_interval
        checkpoint_letter = chr(ord('A') + checkpoint_number - 1)  # -1 because first checkpoint is at interval, not 0

        checkpoint_filename = f"{self.checkpoint_prefix}{checkpoint_letter}.json"

        # Decode and simulate solution
        tours = decode_chromosome(best_chrom, self.instance, allow_mixed=self.allow_mixed)
        job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
            tours, self.instance
        )
        correct_makespan = max(agent_completion_times.values()) if agent_completion_times else float('inf')

        # Build JSON
        routes_json = []
        for a in range(self.instance.m):
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

        checkpoint_json = {
            'checkpoint': {
                'generation': current_gen,
                'letter': checkpoint_letter,
                'total_generations': self.total_generations
            },
            'solution': {
                'makespan': float(correct_makespan),
                'routes': routes_json
            },
            'problem': {
                'num_customers': self.instance.num_customers,
                'num_agents': self.instance.m,
                'resources_per_agent': self.instance.k,
                'depot': self.instance.depot
            }
        }

        # Save
        with open(checkpoint_filename, 'w') as f:
            json.dump(checkpoint_json, f, indent=2)

        print(f"\n[CHECKPOINT] Saved {checkpoint_letter}.json at gen {current_gen} (makespan: {correct_makespan:.2f})", flush=True)

    def solve(self) -> Dict:
        heur_name = {0: "None", 1: "Nearest Neighbor", 3: "Max Regret"}.get(self.heuristic, "Unknown")
        
        print("\n" + "=" * 70)
        print("VRP-RPD SOLVER")
        print("=" * 70)
        print(f"Problem: {self.instance.num_customers} customers, {self.instance.m} agents, k={self.instance.k}")
        print(f"Heuristic: {heur_name}")
        print(f"Genetic Programming: {'Yes' if self.use_gp else 'No'}")
        print(f"Warm Start: {'Yes' if self.use_warm else 'No'}")
        print(f"Gene Injection: {'Yes' if self.use_gene_injection else 'No'}")
        print(f"Workers: {self.num_gpus} GPU + {self.num_cpu_workers} CPU")
        print(f"Generations: {self.total_generations}")
        print("=" * 70 + "\n")
        
        # Generate warm start and track best heuristic solution
        if self.use_warm or self.heuristic > 0:
            warm_start, best_heuristic_makespan, best_heuristic_chrom, best_heuristic_tours = \
                self._generate_warm_start()
        else:
            warm_start = []
            best_heuristic_makespan = float('inf')
            best_heuristic_chrom = None
            best_heuristic_tours = None
        
        start_time = time.time()
        
        instance_dict = {
            'dist': self.instance.dist,
            'proc': self.instance.proc,
            'm': self.instance.m,
            'k': self.instance.k,
            'depot': self.instance.depot
        }
        
        # Use Queue for solutions (unbounded, won't block) and Pipes for candidates (small, bidirectional)
        solution_queue = mp.Queue()  # All workers share one queue (no blocking!)
        candidate_pipes = [mp.Pipe() for _ in range(self.total_workers)]
        result_queue = mp.Queue()

        # Shared state for tracking global best across all workers
        global_best_fitness = mp.Value('d', best_heuristic_makespan)  # Start with heuristic
        global_best_lock = mp.Lock()
        output_json_path = 'best_solution.json'  # Workers save best solutions here

        processes = []

        # Calculate mutation rate distribution
        gpu_base_rate = self.mutation_rate_base if self.mutation_rate_base is not None else 0.1
        cpu_base_rate = self.mutation_rate_base if self.mutation_rate_base is not None else 0.15

        for gpu_id in range(self.num_gpus):
            # Distribute mutation rates across workers for diversity
            if self.num_gpus > 1:
                mutation_rate = gpu_base_rate + (gpu_id % 10) * (self.mutation_rate_range / 9)
            else:
                mutation_rate = gpu_base_rate

            # Give warm start to all workers when we have few workers (<=8),
            # otherwise give to ~half of them for diversity
            if warm_start and (self.num_gpus <= 8 or gpu_id < self.num_gpus // 2):
                ws = list(warm_start)
            else:
                ws = None

            p = mp.Process(
                target=gpu_worker,
                args=(gpu_id, instance_dict, self.pop_per_gpu, self.gens_per_cycle,
                      self.total_cycles, self.allow_mixed, result_queue,
                      solution_queue, candidate_pipes[gpu_id][0],
                      self.use_gp, self.use_gene_injection, ws, mutation_rate,
                      self.elite_size, self.crossover_rate, self.mutation_strength,
                      global_best_fitness, global_best_lock, output_json_path,
                      self.checkpoint_interval, self.checkpoint_prefix)
            )
            p.start()
            processes.append(p)

        for cpu_id in range(self.num_cpu_workers):
            worker_idx = self.num_gpus + cpu_id
            # Distribute mutation rates across CPU workers
            if self.num_cpu_workers > 1:
                mutation_rate = cpu_base_rate + (cpu_id % 8) * (self.mutation_rate_range / 7)
            else:
                mutation_rate = cpu_base_rate

            # Give warm start to all workers when we have few workers (<=8),
            # otherwise give to ~half of them for diversity
            if warm_start and (self.num_cpu_workers <= 8 or cpu_id < self.num_cpu_workers // 2):
                ws = list(warm_start)
            else:
                ws = None

            p = mp.Process(
                target=cpu_worker,
                args=(cpu_id, instance_dict, self.pop_per_cpu, self.gens_per_cycle,
                      self.total_cycles, self.allow_mixed, result_queue,
                      solution_queue, candidate_pipes[worker_idx][0],
                      self.use_gp, self.use_gene_injection, ws, mutation_rate,
                      self.elite_size, self.crossover_rate, self.mutation_strength,
                      global_best_fitness, global_best_lock, output_json_path)
            )
            p.start()
            processes.append(p)
        
        results = []
        
        if self.use_gp:
            analyzer = GeneticAnalyzer(
                use_gpu=HAS_CUPY,
                use_blocks=self.use_blocks,
                use_fft=self.use_fft,
                use_similarity=self.use_similarity
            )
            gp_cycle = 0
            all_solutions = []
            last_analysis_time = time.time()
            analysis_interval = 5.0  # Reduced from 10s for more frequent GP analysis
            workers_finished = 0

            # Track best solution for checkpoints
            best_checkpoint_solution = None
            best_checkpoint_fitness = float('inf')

            # Calculate expected runtime with timeout buffer
            # Updated estimate based on actual performance: ~173s per 100 generations for 10k population
            expected_runtime = (self.total_generations / 100) * 173  # Realistic estimate
            max_runtime = max(1800, expected_runtime * 1.5)  # At least 30 minutes, or 1.5x expected
            gp_start_time = time.time()

            print(f"\n[GP] Starting GP collection loop (timeout: {max_runtime:.0f}s)...", flush=True)

            while workers_finished < self.total_workers:
                # Check for timeout to prevent infinite loop
                elapsed = time.time() - gp_start_time
                if elapsed > max_runtime:
                    print(f"\n[GP] WARNING: Timeout after {elapsed:.1f}s. Stopping GP loop.", flush=True)
                    print(f"[GP] Collected results from {workers_finished}/{self.total_workers} workers", flush=True)

                    # CRITICAL: Drain all pipes to unblock workers before they can send final results
                    print(f"[GP] Draining pipes to unblock workers...", flush=True)
                    for worker_idx in range(self.total_workers):
                        try:
                            while solution_pipes[worker_idx][0].poll(0.001):
                                solution_pipes[worker_idx][0].recv()  # Discard
                        except:
                            pass
                    print(f"[GP] Pipes drained. Waiting for final results...", flush=True)
                    break

                # Check if all processes are still alive
                alive_count = sum(1 for p in processes if p.is_alive())
                if alive_count == 0 and workers_finished < self.total_workers:
                    print(f"\n[GP] WARNING: All processes terminated but only {workers_finished}/{self.total_workers} reported results", flush=True)
                    break

                try:
                    while not result_queue.empty():
                        r = result_queue.get_nowait()

                        # Handle both intermediate and final results
                        worker_id = r.get('worker_id')
                        is_intermediate = r.get('intermediate', False)

                        if is_intermediate:
                            # Update existing result or add new one
                            existing_idx = next((i for i, res in enumerate(results) if res.get('worker_id') == worker_id), None)
                            if existing_idx is not None:
                                # Keep best solution
                                if r.get('best_fitness', float('inf')) < results[existing_idx].get('best_fitness', float('inf')):
                                    results[existing_idx] = r
                                    print(f"[GP] {worker_id} intermediate update: {r.get('best_fitness'):.2f} at gen {r.get('generation', '?')}", flush=True)
                            else:
                                results.append(r)
                                print(f"[GP] {worker_id} first intermediate: {r.get('best_fitness'):.2f} at gen {r.get('generation', '?')}", flush=True)
                        else:
                            # Final result
                            existing_idx = next((i for i, res in enumerate(results) if res.get('worker_id') == worker_id), None)
                            if existing_idx is not None:
                                results[existing_idx] = r  # Replace with final
                            else:
                                results.append(r)
                            workers_finished += 1
                            print(f"[GP] Worker finished: {worker_id} (best={r.get('best_fitness', 'N/A'):.2f})", flush=True)
                except:
                    pass

                # Drain solution queue (non-blocking) - workers can send anytime without blocking!
                while not solution_queue.empty():
                    try:
                        data = solution_queue.get_nowait()
                        solutions = data.get('solutions', [])
                        all_solutions.extend(solutions)

                        # Track best solution for checkpoints
                        for sol in solutions:
                            if sol['fitness'] < best_checkpoint_fitness:
                                best_checkpoint_fitness = sol['fitness']
                                best_checkpoint_solution = sol.copy()
                    except:
                        break  # Queue empty

                current_time = time.time()
                # For large instances, trigger with fewer solutions (workers send less to avoid pipe blocking)
                min_solutions = 30 if self.instance.num_customers > 500 else 50
                if (len(all_solutions) >= min_solutions and
                    current_time - last_analysis_time >= analysis_interval):

                    print(f"\n{'='*50}")
                    print(f"GP ANALYSIS - Cycle {gp_cycle + 1}")
                    print(f"  Collected {len(all_solutions)} solutions")
                    print(f"{'='*50}")

                    analysis = analyzer.analyze(all_solutions, gp_cycle)

                    if self.use_gene_injection:
                        candidates = analyzer.generate_candidates(
                            analysis, self.candidates_per_injection, self.instance.m
                        )

                        sent_count = 0
                        for worker_idx in range(self.total_workers):
                            try:
                                candidate_pipes[worker_idx][1].send({'candidates': candidates})
                                sent_count += 1
                            except:
                                pass

                        print(f"  -> {len(candidates)} candidates sent to {sent_count}/{self.total_workers} workers")

                    all_solutions = []
                    last_analysis_time = current_time
                    gp_cycle += 1

                    # Save checkpoint if needed
                    if self.checkpoint_interval > 0 and best_checkpoint_solution is not None:
                        current_gen = gp_cycle * self.gens_per_cycle
                        if current_gen % self.checkpoint_interval == 0:
                            # Create a fake result entry for checkpoint
                            checkpoint_result = [{
                                'best_fitness': best_checkpoint_fitness,
                                'best_chromosome': best_checkpoint_solution['chromosome']
                            }]
                            self._save_checkpoint(checkpoint_result, current_gen)

                time.sleep(0.1)

            print(f"\n[GP] All workers finished. Total GP cycles: {gp_cycle}", flush=True)
        else:
            # Calculate expected runtime with timeout buffer
            # Updated estimate based on actual performance: ~173s per 100 generations for 10k population
            expected_runtime = (self.total_generations / 100) * 173  # Realistic estimate
            max_runtime = max(1800, expected_runtime * 1.5)  # At least 30 minutes, or 1.5x expected
            no_gp_start_time = time.time()
            workers_finished = 0
            last_status_time = 0

            print(f"\n[No GP] Waiting for workers to complete evolution (timeout: {max_runtime:.0f}s)...", flush=True)
            while workers_finished < self.total_workers:
                # Check for timeout
                elapsed = time.time() - no_gp_start_time
                if elapsed > max_runtime:
                    print(f"\n[No GP] Timeout after {elapsed:.1f}s. Collected {len(results)} intermediate results.", flush=True)
                    break

                try:
                    # Non-blocking check for all available results
                    while not result_queue.empty():
                        r = result_queue.get_nowait()
                        worker_id = r.get('worker_id')
                        is_intermediate = r.get('intermediate', False)

                        # Deduplicate: update existing worker entry or add new one
                        existing_idx = next((i for i, res in enumerate(results) if res.get('worker_id') == worker_id), None)
                        if existing_idx is not None:
                            # Keep the better result (lower fitness)
                            if r.get('best_fitness', float('inf')) < results[existing_idx].get('best_fitness', float('inf')):
                                results[existing_idx] = r
                        else:
                            results.append(r)

                        if not is_intermediate:
                            workers_finished += 1
                            print(f"[No GP] Worker finished: {worker_id} (best={r.get('best_fitness', 'N/A'):.2f})", flush=True)
                except:
                    pass

                alive = sum(1 for p in processes if p.is_alive())
                if alive == 0 and workers_finished < self.total_workers:
                    print(f"[No GP] All processes terminated.", flush=True)
                    break

                # Only print status every 60 seconds to reduce noise
                if workers_finished < self.total_workers:
                    if elapsed - last_status_time >= 60:
                        print(f"[No GP] {elapsed:.0f}s elapsed, {alive} workers running, {len(results)} intermediate results", flush=True)
                        last_status_time = elapsed
                    time.sleep(5.0)  # Check every 5 seconds instead of 1
        
        # Extended final collection with pipe draining
        remaining_timeout = 120  # Increased from 60 to 120 seconds
        deadline = time.time() + remaining_timeout

        # Count unique workers that have reported results (not duplicates)
        unique_workers = set(r.get('worker_id') for r in results if r.get('worker_id'))
        print(f"\n[Final Collection] Have {len(unique_workers)}/{self.total_workers} workers. Waiting up to {remaining_timeout}s for remaining...", flush=True)

        while len(unique_workers) < self.total_workers and time.time() < deadline:
            # Drain pipes to unblock workers
            for worker_idx in range(self.total_workers):
                try:
                    while solution_pipes[worker_idx][0].poll(0.001):
                        solution_pipes[worker_idx][0].recv()  # Discard to unblock
                except:
                    pass

            # Check result queue
            try:
                r = result_queue.get(timeout=1.0)
                worker_id = r.get('worker_id')
                is_intermediate = r.get('intermediate', False)

                # Deduplicate: update existing worker entry or add new one
                existing_idx = next((i for i, res in enumerate(results) if res.get('worker_id') == worker_id), None)
                if existing_idx is not None:
                    # Keep the better result (lower fitness)
                    if r.get('best_fitness', float('inf')) < results[existing_idx].get('best_fitness', float('inf')):
                        results[existing_idx] = r
                        print(f"[Final Collection] Updated {worker_id}: {r.get('best_fitness', 'N/A'):.2f} (intermediate={is_intermediate})", flush=True)
                else:
                    results.append(r)
                    unique_workers.add(worker_id)
                    print(f"[Final Collection] Got result from {worker_id}: {r.get('best_fitness', 'N/A'):.2f} (intermediate={is_intermediate})", flush=True)
            except:
                pass

        # CRITICAL: Before terminating workers, try one more time to get ANY results
        print(f"\n[Critical] Attempting final result collection before terminating workers...", flush=True)
        final_deadline = time.time() + 30
        while len(unique_workers) < self.total_workers and time.time() < final_deadline:
            try:
                r = result_queue.get(timeout=1.0)
                worker_id = r.get('worker_id')
                is_intermediate = r.get('intermediate', False)

                # Deduplicate: update existing worker entry or add new one
                existing_idx = next((i for i, res in enumerate(results) if res.get('worker_id') == worker_id), None)
                if existing_idx is not None:
                    # Keep the better result (lower fitness)
                    if r.get('best_fitness', float('inf')) < results[existing_idx].get('best_fitness', float('inf')):
                        results[existing_idx] = r
                        print(f"[Critical] Updated {worker_id}: {r.get('best_fitness', 'N/A'):.2f}", flush=True)
                else:
                    results.append(r)
                    unique_workers.add(worker_id)
                    print(f"[Critical] Rescued result from {worker_id}: {r.get('best_fitness', 'N/A'):.2f}", flush=True)
            except:
                pass

        for p in processes:
            p.join(timeout=5)  # Reduced to 5s since we already waited above
            if p.is_alive():
                print(f"[Warning] Terminating worker that didn't finish", flush=True)
                p.terminate()

        # Close all pipes and queues to prevent hanging
        for parent_conn, child_conn in candidate_pipes:
            try:
                parent_conn.close()
            except:
                pass
            try:
                child_conn.close()
            except:
                pass

        # Close the queues
        try:
            result_queue.close()
            result_queue.join_thread()
        except:
            pass

        try:
            solution_queue.close()
            solution_queue.join_thread()
        except:
            pass

        solve_time = time.time() - start_time

        # Debug: Show all collected results
        print(f"\n[Results Summary] Collected {len(results)} results:", flush=True)
        for r in results:
            wid = r.get('worker_id', '?')
            fit = r.get('best_fitness', float('inf'))
            is_int = r.get('intermediate', False)
            gen = r.get('generation', 'final')
            print(f"  {wid}: fitness={fit:.2f}, intermediate={is_int}, gen={gen}", flush=True)

        valid = [r for r in results if r.get('best_fitness', float('inf')) < float('inf')]
        if not valid:
            # No valid GA solution - check if we have heuristic
            if best_heuristic_makespan < float('inf'):
                print("\n" + "=" * 70)
                print("COMPLETE (using heuristic - GA found no valid solution)")
                print("=" * 70)
                print(f"Best: {best_heuristic_makespan:.2f} from heuristic")
                print(f"Time: {solve_time:.1f}s")
                return {
                    'makespan': best_heuristic_makespan,
                    'best_chromosome': best_heuristic_chrom,
                    'solve_time': solve_time,
                    'source': 'heuristic',
                    'all_results': []
                }
            return {'makespan': float('inf'), 'error': 'No valid solutions'}
        
        best_ga = min(valid, key=lambda x: x['best_fitness'])
        ga_makespan = best_ga['best_fitness']
        
        # ============================================================
        # CRITICAL: Ensure we never return worse than heuristic!
        # ============================================================
        if best_heuristic_makespan < ga_makespan:
            print("\n" + "=" * 70)
            print("COMPLETE")
            print("=" * 70)
            print(f"??  GA solution ({ga_makespan:.2f}) is WORSE than heuristic ({best_heuristic_makespan:.2f})")
            print(f"    Returning heuristic solution instead!")
            print(f"Best: {best_heuristic_makespan:.2f} from heuristic")
            print(f"Time: {solve_time:.1f}s")
            
            if self.use_gene_injection:
                total_inj = sum(r.get('total_injected', 0) for r in results)
                print(f"Total candidates injected: {total_inj}")
            
            return {
                'makespan': best_heuristic_makespan,
                'best_chromosome': best_heuristic_chrom,
                'solve_time': solve_time,
                'source': 'heuristic',
                'ga_makespan': ga_makespan,
                'all_results': [(r['worker_id'], r.get('best_fitness')) for r in results]
            }
        else:
            # GA improved on heuristic (or heuristic wasn't used)
            improvement = ((best_heuristic_makespan - ga_makespan) / best_heuristic_makespan * 100) \
                          if best_heuristic_makespan < float('inf') else 0
            
            print("\n" + "=" * 70)
            print("COMPLETE")
            print("=" * 70)
            print(f"Best: {ga_makespan:.2f} from {best_ga['worker_id']}")
            if best_heuristic_makespan < float('inf'):
                print(f"  (Improved {improvement:.1f}% over heuristic {best_heuristic_makespan:.2f})")
            print(f"Time: {solve_time:.1f}s")
            
            if self.use_gene_injection:
                total_inj = sum(r.get('total_injected', 0) for r in results)
                print(f"Total candidates injected: {total_inj}")
            
            return {
                'makespan': ga_makespan,
                'best_chromosome': best_ga.get('best_chromosome'),
                'solve_time': solve_time,
                'source': 'ga',
                'heuristic_makespan': best_heuristic_makespan,
                'all_results': [(r['worker_id'], r.get('best_fitness')) for r in results]
            }