#!/usr/bin/env python3
"""
VRP-RPD Island Classes for Genetic Algorithm

Contains GPU and CPU island implementations for the island model GA.
"""

import numpy as np
import torch
from typing import Dict, List

from .models import VRPRPDInstance
from .decoder import (
    compute_makespan_fast,
    HAS_NUMBA,
)

# Conditional import for batch evaluation
if HAS_NUMBA:
    from .decoder import evaluate_batch_numba


class GPUIsland:
    """GPU-based genetic algorithm island"""

    def __init__(
        self,
        instance: VRPRPDInstance,
        device: torch.device,
        population_size: int = 512,
        elite_size: int = 10,
        mutation_rate: float = 0.1,
        crossover_rate: float = 0.8,
        allow_mixed: bool = True,
        mutation_strength: float = 0.02
    ):
        self.instance = instance
        self.device = device
        self.pop_size = population_size
        self.elite_size = elite_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.allow_mixed = allow_mixed
        self.mutation_strength = mutation_strength

        self.num_cust = instance.num_customers
        self.m = instance.m
        self.k = instance.k

        self.population = self._init_population()
        self.fitness = torch.full((population_size,), 1e18, device=device)

        self.best_fitness = float('inf')
        self.best_chromosome = None

    def _init_population(self) -> torch.Tensor:
        """Initialize population"""
        pop = torch.zeros(self.pop_size, self.num_cust, 4, device=self.device)

        for i in range(self.pop_size):
            agents = torch.randint(0, self.m, (self.num_cust,), device=self.device)
            pop[i, :, 0] = agents.float()
            pop[i, :, 1] = agents.float()

            keys = torch.rand(self.num_cust, device=self.device)
            pop[i, :, 2] = keys
            pop[i, :, 3] = keys + 0.1 + torch.rand(self.num_cust, device=self.device) * 0.1

        return pop

    def inject_warm_start(self, chromosomes: List[np.ndarray]):
        """Inject warm start chromosomes"""
        n_inject = min(len(chromosomes), self.pop_size - self.elite_size)
        for i, chrom in enumerate(chromosomes[:n_inject]):
            self.population[i] = torch.tensor(chrom, device=self.device, dtype=torch.float32)

    def inject_candidates(self, candidates: List[np.ndarray]) -> int:
        """Inject external candidates"""
        if not candidates:
            return 0

        n_inject = min(len(candidates), self.pop_size // 5)
        worst_indices = self.fitness.argsort()[-n_inject:]

        for i, idx in enumerate(worst_indices):
            if i < len(candidates):
                self.population[idx] = torch.tensor(candidates[i], device=self.device, dtype=torch.float32)
                self.fitness[idx] = 1e18

        return n_inject

    def _evaluate_population(self):
        """Evaluate population using Numba"""
        needs_eval = (self.fitness > 1e15).nonzero().squeeze(-1)

        if len(needs_eval) == 0:
            return

        if HAS_NUMBA:
            indices = needs_eval.cpu().numpy()
            pop_subset = self.population[needs_eval].cpu().numpy().astype(np.float64)
            fit_subset = np.full(len(indices), 1e18, dtype=np.float64)

            customers = np.array(self.instance.customers, dtype=np.int32)
            dist = self.instance.dist.astype(np.float64)
            proc = self.instance.proc.astype(np.float64)

            evaluate_batch_numba(
                pop_subset, fit_subset, customers, dist, proc,
                self.instance.depot, self.m, self.k, self.allow_mixed
            )

            for i, idx in enumerate(indices):
                self.fitness[idx] = fit_subset[i]
                if fit_subset[i] < self.best_fitness:
                    self.best_fitness = fit_subset[i]
                    self.best_chromosome = self.population[idx].cpu().numpy().copy()
        else:
            for idx in needs_eval:
                idx = idx.item()
                chrom_np = self.population[idx].cpu().numpy()
                makespan = compute_makespan_fast(chrom_np, self.instance, self.allow_mixed)
                self.fitness[idx] = makespan

                if makespan < self.best_fitness:
                    self.best_fitness = makespan
                    self.best_chromosome = chrom_np.copy()

    def evolve_generation(self):
        """Run one generation"""
        self._evaluate_population()

        new_pop = torch.zeros_like(self.population)
        elite_indices = self.fitness.argsort()[:self.elite_size]
        new_pop[:self.elite_size] = self.population[elite_indices]
        elite_fitness = self.fitness[elite_indices].clone()

        for i in range(self.elite_size, self.pop_size):
            t1 = torch.randint(0, self.pop_size, (3,), device=self.device)
            p1 = self.population[t1[self.fitness[t1].argmin()]].clone()

            t2 = torch.randint(0, self.pop_size, (3,), device=self.device)
            p2 = self.population[t2[self.fitness[t2].argmin()]].clone()

            if torch.rand(1).item() < self.crossover_rate:
                mask = torch.rand(self.num_cust, device=self.device) < 0.5
                child = torch.where(mask.unsqueeze(1).expand(-1, 4), p1, p2)
            else:
                child = p1.clone()

            if torch.rand(1).item() < self.mutation_rate:
                idx = torch.randint(0, self.num_cust, (1,)).item()
                child[idx, 0] = torch.randint(0, self.m, (1,), device=self.device).float()
                child[idx, 1] = child[idx, 0].clone()

            if torch.rand(1).item() < self.mutation_rate:
                idx = torch.randint(0, self.num_cust, (1,)).item()
                old_val = child[idx, 2].item()
                new_val = old_val + self.mutation_strength * torch.randn(1).item()
                child[idx, 2] = new_val
                child[idx, 3] = max(new_val + self.mutation_strength * 0.5, child[idx, 3].item())

            new_pop[i] = child

        self.population = new_pop
        self.fitness[:self.elite_size] = elite_fitness
        self.fitness[self.elite_size:] = 1e18

    def get_all_solutions(self) -> List[Dict]:
        self._evaluate_population()
        return [{'id': i, 'chromosome': self.population[i].cpu().numpy(), 'fitness': self.fitness[i].item()}
                for i in range(self.pop_size) if self.fitness[i] < 1e15]


class CPUIsland:
    """CPU-based genetic algorithm island"""

    def __init__(
        self,
        instance: VRPRPDInstance,
        population_size: int = 256,
        elite_size: int = 10,
        mutation_rate: float = 0.15,
        crossover_rate: float = 0.8,
        allow_mixed: bool = True,
        mutation_strength: float = 0.02
    ):
        self.instance = instance
        self.pop_size = population_size
        self.elite_size = elite_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.allow_mixed = allow_mixed
        self.mutation_strength = mutation_strength

        self.num_cust = instance.num_customers
        self.m = instance.m
        self.k = instance.k

        self.population = self._init_population()
        self.fitness = np.full(population_size, 1e18)

        self.best_fitness = float('inf')
        self.best_chromosome = None

    def _init_population(self) -> np.ndarray:
        """Initialize population"""
        pop = np.zeros((self.pop_size, self.num_cust, 4), dtype=np.float32)

        for i in range(self.pop_size):
            agents = np.random.randint(0, self.m, self.num_cust)
            pop[i, :, 0] = agents
            pop[i, :, 1] = agents

            keys = np.random.rand(self.num_cust)
            pop[i, :, 2] = keys
            pop[i, :, 3] = keys + 0.1 + np.random.rand(self.num_cust) * 0.1

        return pop

    def inject_warm_start(self, chromosomes: List[np.ndarray]):
        """Inject warm start chromosomes"""
        n_inject = min(len(chromosomes), self.pop_size - self.elite_size)
        for i, chrom in enumerate(chromosomes[:n_inject]):
            self.population[i] = chrom.astype(np.float32)

    def inject_candidates(self, candidates: List[np.ndarray]) -> int:
        if not candidates:
            return 0

        n_inject = min(len(candidates), self.pop_size // 5)
        worst_indices = np.argsort(self.fitness)[-n_inject:]

        for i, idx in enumerate(worst_indices):
            if i < len(candidates):
                self.population[idx] = candidates[i].astype(np.float32)
                self.fitness[idx] = 1e18

        return n_inject

    def _evaluate_population(self):
        needs_eval = np.where(self.fitness > 1e15)[0]

        if len(needs_eval) == 0:
            return

        if HAS_NUMBA and len(needs_eval) > 10:
            pop_subset = self.population[needs_eval].astype(np.float64)
            fit_subset = np.full(len(needs_eval), 1e18, dtype=np.float64)

            customers = np.array(self.instance.customers, dtype=np.int32)
            dist = self.instance.dist.astype(np.float64)
            proc = self.instance.proc.astype(np.float64)

            evaluate_batch_numba(
                pop_subset, fit_subset, customers, dist, proc,
                self.instance.depot, self.m, self.k, self.allow_mixed
            )

            for i, idx in enumerate(needs_eval):
                self.fitness[idx] = fit_subset[i]
                if fit_subset[i] < self.best_fitness:
                    self.best_fitness = fit_subset[i]
                    self.best_chromosome = self.population[idx].copy()
        else:
            for idx in needs_eval:
                makespan = compute_makespan_fast(self.population[idx], self.instance, self.allow_mixed)
                self.fitness[idx] = makespan

                if makespan < self.best_fitness:
                    self.best_fitness = makespan
                    self.best_chromosome = self.population[idx].copy()

    def evolve_generation(self):
        self._evaluate_population()

        new_pop = np.zeros_like(self.population)
        elite_indices = np.argsort(self.fitness)[:self.elite_size]
        new_pop[:self.elite_size] = self.population[elite_indices]
        elite_fitness = self.fitness[elite_indices].copy()

        for i in range(self.elite_size, self.pop_size):
            t1 = np.random.choice(self.pop_size, 3, replace=False)
            p1 = self.population[t1[np.argmin(self.fitness[t1])]].copy()

            t2 = np.random.choice(self.pop_size, 3, replace=False)
            p2 = self.population[t2[np.argmin(self.fitness[t2])]].copy()

            if np.random.rand() < self.crossover_rate:
                mask = np.random.rand(self.num_cust) < 0.5
                child = np.where(mask[:, None], p1, p2)
            else:
                child = p1.copy()

            if np.random.rand() < self.mutation_rate:
                idx = np.random.randint(0, self.num_cust)
                child[idx, 0] = np.random.randint(0, self.m)
                child[idx, 1] = child[idx, 0]

            if np.random.rand() < self.mutation_rate:
                idx = np.random.randint(0, self.num_cust)
                child[idx, 2] += self.mutation_strength * np.random.randn()
                child[idx, 3] = max(child[idx, 2] + self.mutation_strength * 0.5, child[idx, 3])

            new_pop[i] = child

        self.population = new_pop
        self.fitness[:self.elite_size] = elite_fitness
        self.fitness[self.elite_size:] = 1e18

    def get_all_solutions(self) -> List[Dict]:
        self._evaluate_population()
        return [{'id': i, 'chromosome': self.population[i], 'fitness': self.fitness[i]}
                for i in range(self.pop_size) if self.fitness[i] < 1e15]
