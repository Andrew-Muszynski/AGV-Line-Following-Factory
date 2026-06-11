#!/usr/bin/env python3
"""
VRP-RPD Chromosome Decoder and Makespan Computation

Contains:
- Chromosome decoding to tours
- Makespan computation (Python and Numba JIT versions)
- Batch evaluation for populations
"""

import numpy as np
from typing import Dict, List


from .models import VRPRPDInstance

# Optional Numba import
try:
    import numba
    from numba import jit, prange
    from numba import cuda as numba_cuda
    HAS_NUMBA = True
    HAS_NUMBA_CUDA = numba_cuda.is_available()
except ImportError:
    HAS_NUMBA = False
    HAS_NUMBA_CUDA = False
    print("Warning: Numba not found. Install with 'pip install numba' for speedup.")


def compute_makespan_from_tours(tours, dist, proc, depot, m, k, customers):
    """
    Compute makespan from tour dictionary using FEASIBILITY-PRESERVING simulation.

    When an agent is at capacity, we skip dropoff events and look for the next
    pickup. This ensures any tour configuration produces a valid makespan.
    """
    agent_time = [0.0] * m
    agent_loc = [depot] * m
    agent_deployed = [k] * m  # Start with k tokens
    dropoff_complete = {}
    dropped_by = {}
    picked_up = set()

    # Build event lists for each agent: [(key, cust_loc, op, c_idx), ...]
    # key is position in original tour (used for ordering preference)
    agent_events = {a: [] for a in range(m)}
    for a in range(m):
        for pos, (cust_loc, op, c_idx) in enumerate(tours.get(a, [])):
            agent_events[a].append({'pos': pos, 'cust_loc': cust_loc, 'op': op, 'c_idx': c_idx, 'done': False})

    total_events = sum(len(events) for events in agent_events.values())
    completed = 0
    max_iterations = total_events * 3 + 100

    for iteration in range(max_iterations):
        if completed >= total_events:
            break

        # Find best feasible event across all agents
        best_agent = -1
        best_time = float('inf')
        best_event_idx = -1
        best_event = None

        for a in range(m):
            # Find first undone, feasible event for this agent
            for ev_idx, event in enumerate(agent_events[a]):
                if event['done']:
                    continue

                cust_loc = event['cust_loc']
                op = event['op']
                c_idx = event['c_idx']

                # Check feasibility
                if op == 'D':
                    if agent_deployed[a] <= 0:
                        continue  # Skip - no tokens to drop
                else:  # Pickup
                    if c_idx not in dropoff_complete:
                        continue  # Skip - not dropped yet
                    if c_idx in picked_up:
                        continue  # Skip - already picked
                    if agent_deployed[a] >= k:
                        continue  # Skip - at capacity, cannot pick up more

                # Compute arrival time
                travel = dist[agent_loc[a]][cust_loc]
                arrival = agent_time[a] + travel

                if op == 'P':
                    arrival = max(arrival, dropoff_complete[c_idx])

                if arrival < best_time:
                    best_time = arrival
                    best_agent = a
                    best_event_idx = ev_idx
                    best_event = event

                # Only consider first feasible event per agent
                break

        # ===== FIX: Handle stranded pickups when no feasible event found =====
        if best_agent < 0:
            # Check for stranded pickups (dropped but not picked up)
            stranded_pickups = []
            for c_idx in dropoff_complete:
                if c_idx not in picked_up:
                    stranded_pickups.append(c_idx)

            if not stranded_pickups:
                break  # All pickups done, safe to exit

            # Find ANY agent with capacity to handle a stranded pickup
            for c_idx in stranded_pickups:
                cust_loc = customers[c_idx]

                best_rescue_time = float('inf')
                best_rescue_agent = -1

                for a in range(m):
                    if agent_deployed[a] >= k:
                        continue  # This agent at capacity

                    travel = dist[agent_loc[a]][cust_loc]
                    arrival = max(agent_time[a] + travel, dropoff_complete[c_idx])

                    if arrival < best_rescue_time:
                        best_rescue_time = arrival
                        best_rescue_agent = a

                if best_rescue_agent >= 0:
                    # Execute rescue pickup
                    agent_time[best_rescue_agent] = best_rescue_time
                    agent_loc[best_rescue_agent] = cust_loc
                    picked_up.add(c_idx)
                    agent_deployed[best_rescue_agent] += 1

                    # Mark the original pickup event as done
                    for a in range(m):
                        for ev_idx, event in enumerate(agent_events[a]):
                            if event['c_idx'] == c_idx and event['op'] == 'P' and not event['done']:
                                event['done'] = True
                                break

                    completed += 1
                    best_agent = best_rescue_agent  # Signal we did something
                    break  # Process one stranded pickup per iteration

            if best_agent < 0:
                break  # No agent can rescue - truly stuck
            continue  # Skip normal execution, already handled rescue
        # ===== END FIX =====

        # Execute event
        cust_loc = best_event['cust_loc']
        op = best_event['op']
        c_idx = best_event['c_idx']

        if op == 'D':
            agent_deployed[best_agent] -= 1
            dropped_by[c_idx] = best_agent
            completion = best_time + proc[cust_loc]
            dropoff_complete[c_idx] = completion
            agent_time[best_agent] = best_time
        else:
            agent_time[best_agent] = best_time
            picked_up.add(c_idx)
            agent_deployed[best_agent] += 1

        agent_loc[best_agent] = cust_loc
        agent_events[best_agent][best_event_idx]['done'] = True
        completed += 1

    # Return to depot
    for a in range(m):
        if agent_loc[a] != depot:
            agent_time[a] += dist[agent_loc[a]][depot]

    return max(agent_time) if agent_time else 0.0


# =============================================================================
# NUMBA-ACCELERATED EVALUATION
# =============================================================================

if HAS_NUMBA:
    @jit(nopython=True, cache=True)
    def compute_makespan_numba(
        drop_agents: np.ndarray,
        pick_agents: np.ndarray,
        drop_keys: np.ndarray,
        pick_keys: np.ndarray,
        customers: np.ndarray,
        dist: np.ndarray,
        proc: np.ndarray,
        depot: int,
        m: int,
        k: int
    ) -> float:
        """
        Numba JIT-compiled makespan computation with FEASIBILITY-PRESERVING decoder.
        """
        n_cust = len(customers)

        max_events_per_agent = n_cust * 2 + 1
        agent_events = np.zeros((m, max_events_per_agent, 4), dtype=np.float64)
        agent_event_count = np.zeros(m, dtype=np.int32)

        for i in range(n_cust):
            da = drop_agents[i]
            pa = pick_agents[i]

            idx = agent_event_count[da]
            agent_events[da, idx, 0] = drop_keys[i]
            agent_events[da, idx, 1] = i
            agent_events[da, idx, 2] = 0
            agent_events[da, idx, 3] = 0
            agent_event_count[da] += 1

            idx = agent_event_count[pa]
            agent_events[pa, idx, 0] = pick_keys[i]
            agent_events[pa, idx, 1] = i
            agent_events[pa, idx, 2] = 1
            agent_events[pa, idx, 3] = 0
            agent_event_count[pa] += 1

        for a in range(m):
            n_ev = agent_event_count[a]
            for i in range(n_ev):
                for j in range(i + 1, n_ev):
                    ki, ti = agent_events[a, i, 0], agent_events[a, i, 2]
                    kj, tj = agent_events[a, j, 0], agent_events[a, j, 2]
                    if (ki > kj) or (ki == kj and ti > tj):
                        for c in range(4):
                            tmp = agent_events[a, i, c]
                            agent_events[a, i, c] = agent_events[a, j, c]
                            agent_events[a, j, c] = tmp

        agent_time = np.zeros(m, dtype=np.float64)
        agent_loc = np.full(m, depot, dtype=np.int32)
        agent_deployed = np.full(m, k, dtype=np.int32)  # Start with k tokens

        dropoff_complete = np.full(n_cust, -1.0, dtype=np.float64)
        dropped_by = np.full(n_cust, -1, dtype=np.int32)
        picked_up = np.zeros(n_cust, dtype=np.int32)

        n_events = n_cust * 2
        total_completed = 0
        max_iterations = n_events * 3 + 100

        for iteration in range(max_iterations):
            if total_completed >= n_events:
                break

            best_agent = -1
            best_time = 1e18
            best_event_idx = -1
            best_cust_idx = -1
            best_event_type = -1
            best_cust_loc = -1

            for a in range(m):
                n_ev = agent_event_count[a]

                for ev_idx in range(n_ev):
                    if agent_events[a, ev_idx, 3] > 0.5:
                        continue

                    cust_idx = int(agent_events[a, ev_idx, 1])
                    event_type = int(agent_events[a, ev_idx, 2])
                    cust_loc = customers[cust_idx]

                    if event_type == 0:
                        if agent_deployed[a] <= 0:
                            continue
                    else:
                        if dropoff_complete[cust_idx] < 0:
                            continue
                        if picked_up[cust_idx] > 0:
                            continue
                        if agent_deployed[a] >= k:
                            continue

                    travel = dist[agent_loc[a], cust_loc]
                    arrival = agent_time[a] + travel

                    if event_type == 1:
                        ready_time = dropoff_complete[cust_idx]
                        arrival = max(arrival, ready_time)

                    if arrival < best_time:
                        best_time = arrival
                        best_agent = a
                        best_event_idx = ev_idx
                        best_cust_idx = cust_idx
                        best_event_type = event_type
                        best_cust_loc = cust_loc

                    break

            # ===== FIX: Handle stranded pickups when no feasible event found =====
            if best_agent < 0:
                # Look for stranded pickups (dropped but not picked up)
                found_rescue = False
                for c_idx_check in range(n_cust):
                    if dropoff_complete[c_idx_check] < 0:
                        continue  # Not dropped yet
                    if picked_up[c_idx_check] > 0:
                        continue  # Already picked up

                    # This customer is stranded - find any agent with capacity
                    stranded_loc = customers[c_idx_check]
                    rescue_best_time = 1e18
                    rescue_best_agent = -1

                    for a in range(m):
                        if agent_deployed[a] >= k:
                            continue

                        rescue_travel = dist[agent_loc[a], stranded_loc]
                        rescue_arrival = max(agent_time[a] + rescue_travel, dropoff_complete[c_idx_check])

                        if rescue_arrival < rescue_best_time:
                            rescue_best_time = rescue_arrival
                            rescue_best_agent = a

                    if rescue_best_agent >= 0:
                        # Execute rescue pickup
                        agent_time[rescue_best_agent] = rescue_best_time
                        agent_loc[rescue_best_agent] = stranded_loc
                        picked_up[c_idx_check] = 1
                        agent_deployed[rescue_best_agent] += 1

                        # Mark original pickup event as done
                        for a in range(m):
                            n_ev = agent_event_count[a]
                            for ev_idx in range(n_ev):
                                if agent_events[a, ev_idx, 3] > 0.5:
                                    continue
                                ev_cust_idx = int(agent_events[a, ev_idx, 1])
                                ev_type = int(agent_events[a, ev_idx, 2])
                                if ev_cust_idx == c_idx_check and ev_type == 1:
                                    agent_events[a, ev_idx, 3] = 1.0
                                    break

                        total_completed += 1
                        found_rescue = True
                        break  # Handle one stranded pickup per iteration

                if not found_rescue:
                    break  # Truly stuck - no agent can rescue
                continue  # Skip normal execution, handled rescue
            # ===== END FIX =====

            if best_event_type == 0:
                agent_deployed[best_agent] -= 1
                dropped_by[best_cust_idx] = best_agent
                completion_time = best_time + proc[best_cust_loc]
                dropoff_complete[best_cust_idx] = completion_time
                agent_time[best_agent] = best_time
            else:
                agent_time[best_agent] = best_time
                picked_up[best_cust_idx] = 1
                agent_deployed[best_agent] += 1

            agent_loc[best_agent] = best_cust_loc
            agent_events[best_agent, best_event_idx, 3] = 1.0
            total_completed += 1

        makespan = 0.0
        for a in range(m):
            final_time = agent_time[a]
            if agent_loc[a] != depot:
                final_time += dist[agent_loc[a], depot]
            if final_time > makespan:
                makespan = final_time

        return makespan

    @jit(nopython=True, parallel=True, cache=True)
    def evaluate_batch_numba(
        population: np.ndarray,
        fitness: np.ndarray,
        customers: np.ndarray,
        dist: np.ndarray,
        proc: np.ndarray,
        depot: int,
        m: int,
        k: int,
        allow_mixed: bool
    ):
        """Parallel batch evaluation using Numba prange"""
        pop_size = population.shape[0]

        for i in prange(pop_size):
            if fitness[i] > 1e15 or fitness[i] != fitness[i]:
                chrom = population[i]
                drop_agents = chrom[:, 0].astype(np.int32)
                if allow_mixed:
                    pick_agents = chrom[:, 1].astype(np.int32)
                else:
                    pick_agents = drop_agents.copy()
                drop_keys = chrom[:, 2]
                pick_keys = chrom[:, 3]

                for j in range(len(drop_agents)):
                    if drop_agents[j] < 0:
                        drop_agents[j] = 0
                    elif drop_agents[j] >= m:
                        drop_agents[j] = m - 1
                    if pick_agents[j] < 0:
                        pick_agents[j] = 0
                    elif pick_agents[j] >= m:
                        pick_agents[j] = m - 1

                fitness[i] = compute_makespan_numba(
                    drop_agents, pick_agents, drop_keys, pick_keys,
                    customers, dist, proc, depot, m, k
                )


def compute_makespan_fast(chrom: np.ndarray, instance: VRPRPDInstance, allow_mixed: bool = True) -> float:
    """Fast makespan computation - uses Numba if available"""
    if HAS_NUMBA:
        drop_agents = chrom[:, 0].astype(np.int32)
        pick_agents = chrom[:, 1].astype(np.int32) if allow_mixed else drop_agents
        drop_keys = chrom[:, 2].astype(np.float64)
        pick_keys = chrom[:, 3].astype(np.float64)
        customers = np.array(instance.customers, dtype=np.int32)

        drop_agents = np.clip(drop_agents, 0, instance.m - 1)
        pick_agents = np.clip(pick_agents, 0, instance.m - 1)

        return compute_makespan_numba(
            drop_agents, pick_agents, drop_keys, pick_keys,
            customers, instance.dist.astype(np.float64),
            instance.proc.astype(np.float64),
            instance.depot, instance.m, instance.k
        )
    else:
        return compute_makespan_python(chrom, instance, allow_mixed)


def compute_makespan_python(chrom: np.ndarray, instance: VRPRPDInstance, allow_mixed: bool = True) -> float:
    """Pure Python makespan computation"""
    tours = decode_chromosome(chrom, instance, allow_mixed)
    return compute_makespan_from_tours(
        tours, instance.dist, instance.proc, instance.depot,
        instance.m, instance.k, instance.customers
    )


def decode_chromosome(chrom: np.ndarray, instance: VRPRPDInstance, allow_mixed: bool = True) -> Dict:
    """Decode chromosome to tours"""
    drop_agents = chrom[:, 0].astype(int)
    pick_agents = chrom[:, 1].astype(int) if allow_mixed else drop_agents
    drop_keys = chrom[:, 2]
    pick_keys = chrom[:, 3]

    m = instance.m
    agent_events = {a: [] for a in range(m)}

    for cust_idx in range(instance.num_customers):
        cust_loc = instance.customers[cust_idx]
        da = min(max(0, drop_agents[cust_idx]), m - 1)
        pa = min(max(0, pick_agents[cust_idx]), m - 1)

        agent_events[da].append((drop_keys[cust_idx], cust_loc, 'D', cust_idx))
        agent_events[pa].append((pick_keys[cust_idx], cust_loc, 'P', cust_idx))

    tours = {}
    for a in range(m):
        events = sorted(agent_events[a], key=lambda x: (x[0], 0 if x[2] == 'D' else 1))
        tours[a] = [(e[1], e[2], e[3]) for e in events]

    return tours
