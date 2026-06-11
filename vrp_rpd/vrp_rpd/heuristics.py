#!/usr/bin/env python3
"""
VRP-RPD Heuristic Generators

Provides construction heuristics for generating initial solutions:
- Nearest Neighbor
- Max Regret
- Greedy Defer
"""

import numpy as np
from typing import Dict, Tuple

from .decoder import compute_makespan_from_tours


def generate_nearest_neighbor_solution(
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    num_customers: int,
    allow_mixed: bool = True
) -> Tuple[np.ndarray, float, Dict]:
    """
    Heuristic 1: Nearest Neighbor

    Greedy construction prioritizing shortest travel distance.
    Each step picks the nearest unvisited customer that can be served.

    Args:
        allow_mixed: If False, forces same agent for pickup and dropoff (no interleaving)

    Returns: (chromosome, makespan, tours)
    """
    n = len(dist)

    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]

    # Agent state
    agent_loc = [depot] * m
    agent_time = [0.0] * m
    agent_deployed = [k] * m  # Start with k tokens

    # Customer state
    dropped = {}  # cust_idx -> (agent, ready_time)
    picked = set()
    tours = {a: [] for a in range(m)}

    # Track order for chromosome generation - PER AGENT positions
    drop_order = {}  # cust_idx -> position within dropper's tour
    pick_order = {}  # cust_idx -> position within picker's tour
    drop_agent_map = {}  # cust_idx -> drop agent
    pick_agent_map = {}  # cust_idx -> pick agent
    agent_event_count = {a: 0 for a in range(m)}  # Per-agent event counter

    iteration_count = 0
    max_iterations = num_customers * 10  # Prevent infinite loops

    while len(picked) < num_customers and iteration_count < max_iterations:
        iteration_count += 1
        best_score = float('inf')
        best_action = None

        for a in range(m):
            loc = agent_loc[a]

            # Nearest neighbor for dropoffs (only if agent has tokens)
            if agent_deployed[a] > 0:
                for c_idx in range(num_customers):
                    if c_idx in dropped:
                        continue
                    cust_loc = customers[c_idx]
                    travel = dist[loc][cust_loc]

                    if travel < best_score:
                        best_score = travel
                        best_action = ('D', a, c_idx)

            # Nearest neighbor for pickups (only if agent has room for tokens)
            if agent_deployed[a] < k:
                for c_idx in range(num_customers):
                    if c_idx not in dropped or c_idx in picked:
                        continue

                    # If allow_mixed=False, only the dropoff agent can do pickup
                    if not allow_mixed:
                        dropper, _ = dropped[c_idx]
                        if a != dropper:
                            continue

                    cust_loc = customers[c_idx]
                    travel = dist[loc][cust_loc]

                    # Prioritize pickups when empty (must get tokens)
                    if agent_deployed[a] == 0:
                        travel *= 0.5

                    if travel < best_score:
                        best_score = travel
                        best_action = ('P', a, c_idx)

        if best_action is None:
            # Fallback: Force assignment to break deadlock
            # First, try to pickup any ready customers
            for c_idx in range(num_customers):
                if c_idx in dropped and c_idx not in picked:
                    dropper, _ = dropped[c_idx]
                    if not allow_mixed:
                        # Must use same agent for pickup
                        if agent_deployed[dropper] < k:
                            best_action = ('P', dropper, c_idx)
                            break
                    else:
                        # Can use any agent
                        for a in range(m):
                            if agent_deployed[a] < k:
                                best_action = ('P', a, c_idx)
                                break
                        if best_action:
                            break

            # If still none, force a dropoff
            if best_action is None:
                for c_idx in range(num_customers):
                    if c_idx not in dropped:
                        for a in range(m):
                            if agent_deployed[a] > 0:
                                best_action = ('D', a, c_idx)
                                break
                        if best_action:
                            break

            if best_action is None:
                break

        op, a, c_idx = best_action
        cust_loc = customers[c_idx]

        if op == 'D':
            travel = dist[agent_loc[a]][cust_loc]
            arrival = agent_time[a] + travel
            completion = arrival + proc[cust_loc]

            agent_time[a] = completion
            agent_loc[a] = cust_loc
            agent_deployed[a] -= 1  # FIXED: Dropoff consumes a resource
            dropped[c_idx] = (a, completion)
            tours[a].append((cust_loc, 'D', c_idx))
            drop_order[c_idx] = agent_event_count[a]
            drop_agent_map[c_idx] = a
            agent_event_count[a] += 1
        else:
            dropper, ready_time = dropped[c_idx]
            travel = dist[agent_loc[a]][cust_loc]
            arrival = agent_time[a] + travel
            actual_pickup = max(arrival, ready_time)

            agent_time[a] = actual_pickup
            agent_loc[a] = cust_loc
            agent_deployed[a] += 1  # FIXED: Pickup returns a resource to the picker (agent a)
            picked.add(c_idx)
            tours[a].append((cust_loc, 'P', c_idx))
            pick_order[c_idx] = agent_event_count[a]
            pick_agent_map[c_idx] = a  # Track actual picker, not dropper!
            agent_event_count[a] += 1

    # Return to depot
    for a in range(m):
        if agent_loc[a] != depot:
            agent_time[a] += dist[agent_loc[a]][depot]

    makespan = max(agent_time)

    # Build chromosome [drop_agent, pick_agent, drop_key, pick_key]
    # Keys are normalized by per-agent tour length for portability
    chrom = np.zeros((num_customers, 4), dtype=np.float32)

    for c_idx in range(num_customers):
        if c_idx in dropped:
            da = drop_agent_map[c_idx]
            pa = pick_agent_map.get(c_idx, da)  # Actual picker, fallback to dropper

            # Normalize by the respective agent's tour length
            da_tour_len = max(agent_event_count[da], 1)
            pa_tour_len = max(agent_event_count[pa], 1)

            chrom[c_idx, 0] = da
            chrom[c_idx, 1] = pa
            chrom[c_idx, 2] = drop_order.get(c_idx, 0) / da_tour_len
            chrom[c_idx, 3] = pick_order.get(c_idx, 0) / pa_tour_len

    return chrom, makespan, tours


def generate_max_regret_solution(
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    num_customers: int,
    allow_mixed: bool = True
) -> Tuple[np.ndarray, float, Dict]:
    """
    Heuristic 3: Max Regret

    Insertion heuristic that prioritizes customers with highest "regret" -
    the difference between best and second-best insertion positions.
    This avoids committing to customers that have many good options.

    Args:
        allow_mixed: If False, forces same agent for pickup and dropoff (no interleaving)

    Returns: (chromosome, makespan, tours)
    """
    n = len(dist)

    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]

    # Initialize agent tours as empty
    agent_tours = {a: [] for a in range(m)}  # List of (cust_idx, 'D'/'P')
    unassigned = set(range(num_customers))
    dropped_set = set()

    def evaluate_tour_makespan(agent_tours_local):
        """Compute makespan for current tour assignment"""
        agent_time = [0.0] * m
        agent_loc = [depot] * m
        agent_deployed = [k] * m  # Start with k tokens
        dropoff_complete = {}

        # Build event sequence
        events = []
        for a in range(m):
            for pos, (c_idx, op) in enumerate(agent_tours_local[a]):
                events.append((a, pos, c_idx, op))

        # Sort by position (approximate)
        events.sort(key=lambda x: x[1])

        for a, pos, c_idx, op in events:
            cust_loc = customers[c_idx]
            travel = dist[agent_loc[a]][cust_loc]
            arrival = agent_time[a] + travel

            if op == 'D':
                if agent_deployed[a] <= 0:
                    return float('inf')
                completion = arrival + proc[cust_loc]
                agent_time[a] = completion
                agent_loc[a] = cust_loc
                agent_deployed[a] -= 1
                dropoff_complete[c_idx] = completion
            else:
                if c_idx not in dropoff_complete:
                    return float('inf')
                if agent_deployed[a] >= k:
                    return float('inf')
                actual = max(arrival, dropoff_complete[c_idx])
                agent_time[a] = actual
                agent_loc[a] = cust_loc
                agent_deployed[a] += 1

        # Return to depot
        for a in range(m):
            if agent_loc[a] != depot:
                agent_time[a] += dist[agent_loc[a]][depot]

        return max(agent_time)

    def compute_insertion_cost(c_idx, agent, pos, op, current_tours):
        """Compute cost of inserting customer at position"""
        test_tours = {a: list(current_tours[a]) for a in range(m)}
        test_tours[agent].insert(pos, (c_idx, op))
        return evaluate_tour_makespan(test_tours)

    # Greedy insertion with regret
    iteration_count = 0
    max_iterations = num_customers * 10  # Prevent infinite loops

    while unassigned and iteration_count < max_iterations:
        iteration_count += 1
        best_regret = -float('inf')
        best_insertion = None
        fallback_insertion = None  # Track ANY valid insertion as fallback

        for c_idx in unassigned:
            # Find best and second-best dropoff insertion
            costs = []
            for a in range(m):
                for pos in range(len(agent_tours[a]) + 1):
                    cost = compute_insertion_cost(c_idx, a, pos, 'D', agent_tours)
                    if cost < float('inf'):
                        costs.append((cost, a, pos))

            if len(costs) < 1:
                continue

            costs.sort(key=lambda x: x[0])
            best_cost = costs[0][0]
            second_cost = costs[1][0] if len(costs) > 1 else best_cost * 1.5

            regret = second_cost - best_cost

            # Track first valid insertion as fallback
            if fallback_insertion is None:
                fallback_insertion = (c_idx, costs[0][1], costs[0][2])

            if regret > best_regret:
                best_regret = regret
                best_insertion = (c_idx, costs[0][1], costs[0][2])

        # Use fallback if no regret-based insertion found
        if best_insertion is None and fallback_insertion is not None:
            best_insertion = fallback_insertion

        if best_insertion is None:
            # No valid insertion found using cost evaluation
            # Force assign remaining customers to balance load across agents
            if unassigned:
                c_idx = next(iter(unassigned))
                # Find agent with fewest customers and most available resources
                agent_loads = [(len(agent_tours[a]), a) for a in range(m)]
                agent_loads.sort()
                agent = agent_loads[0][1]
                # Append at end (simplest position)
                agent_tours[agent].append((c_idx, 'D'))
                dropped_set.add(c_idx)
                unassigned.discard(c_idx)
                continue  # Don't break - keep trying to assign remaining customers
            else:
                break

        c_idx, agent, pos = best_insertion
        agent_tours[agent].insert(pos, (c_idx, 'D'))
        dropped_set.add(c_idx)
        unassigned.discard(c_idx)

    # Insert pickups after all dropoffs
    # First, build a map of which agent dropped each customer
    customer_dropper = {}
    for a in range(m):
        for c_idx, op in agent_tours[a]:
            if op == 'D':
                customer_dropper[c_idx] = a

    for c_idx in dropped_set:
        best_cost = float('inf')
        best_agent = None
        best_pos = 0

        # Determine which agents to consider for pickup
        if allow_mixed:
            # Can use any agent
            agents_to_try = range(m)
        else:
            # Must use same agent that did dropoff
            dropper = customer_dropper.get(c_idx, 0)
            agents_to_try = [dropper]

        for a in agents_to_try:
            for pos in range(len(agent_tours[a]) + 1):
                cost = compute_insertion_cost(c_idx, a, pos, 'P', agent_tours)
                if cost < best_cost:
                    best_cost = cost
                    best_agent = a
                    best_pos = pos

        # If no valid position found, force insert at end of dropper's tour (non-interleaved case)
        if best_agent is None:
            if allow_mixed:
                # Try any agent with capacity
                for a in range(m):
                    if len(agent_tours[a]) > 0:
                        best_agent = a
                        best_pos = len(agent_tours[a])
                        break
            else:
                # Must use dropper agent
                dropper = customer_dropper.get(c_idx, 0)
                best_agent = dropper
                best_pos = len(agent_tours[dropper])

        if best_agent is not None:
            agent_tours[best_agent].insert(best_pos, (c_idx, 'P'))

    # Convert to simulation format and compute final makespan
    # Use PER-AGENT position-based keys for proper GA compatibility
    # IMPORTANT: Use single counter per agent to preserve interleaving order
    tours = {a: [] for a in range(m)}
    drop_agent = {}
    pick_agent = {}
    drop_position = {}  # Position within drop agent's FULL tour
    pick_position = {}  # Position within pick agent's FULL tour
    agent_tour_lengths = {a: len(agent_tours[a]) for a in range(m)}

    for a in range(m):
        pos = 0  # Single position counter for this agent's full tour
        for c_idx, op in agent_tours[a]:
            cust_loc = customers[c_idx]
            tours[a].append((cust_loc, op, c_idx))
            if op == 'D':
                drop_agent[c_idx] = a
                drop_position[c_idx] = pos
            else:
                pick_agent[c_idx] = a
                pick_position[c_idx] = pos
            pos += 1

    # Compute actual makespan
    makespan = compute_makespan_from_tours(tours, dist, proc, depot, m, k, customers)

    # Build chromosome with per-agent normalized keys
    chrom = np.zeros((num_customers, 4), dtype=np.float32)
    for c_idx in range(num_customers):
        da = drop_agent.get(c_idx, 0)
        pa = pick_agent.get(c_idx, da)

        # Normalize by agent's FULL tour length
        da_tour_len = max(agent_tour_lengths.get(da, 1), 1)
        pa_tour_len = max(agent_tour_lengths.get(pa, 1), 1)

        chrom[c_idx, 0] = da
        chrom[c_idx, 1] = pa
        chrom[c_idx, 2] = drop_position.get(c_idx, 0) / da_tour_len
        chrom[c_idx, 3] = pick_position.get(c_idx, 0) / pa_tour_len

    return chrom, makespan, tours


def generate_greedy_defer_solution(
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    num_customers: int,
    defer_multiplier: float = 10.0,
    allow_mixed: bool = True
) -> Tuple[np.ndarray, float]:
    """
    Greedy solution with defer penalty (from BRKGA).
    Prioritizes dropoffs by adding penalty to pickups.

    Args:
        allow_mixed: If False, forces same agent for pickup and dropoff (no interleaving)

    Returns: (chromosome, makespan)
    """
    n = len(dist)

    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]

    avg_proc = np.mean(proc[proc > 0]) if np.any(proc > 0) else 50.0
    defer_penalty = avg_proc * defer_multiplier

    agent_loc = [depot] * m
    agent_time = [0.0] * m
    agent_deployed = [k] * m  # Start with k tokens

    dropped = {}
    picked = set()
    drop_agent = {}
    pick_agent = {}
    # Track ALL events per agent in order (both drops and pickups)
    # Each entry is (c_idx, op_type) to track position in full tour
    agent_events = {a: [] for a in range(m)}

    while len(picked) < num_customers:
        remaining_drops = num_customers - len(dropped)
        best_score = float('inf')
        best_action = None

        for a in range(m):
            loc = agent_loc[a]
            time = agent_time[a]
            has_tokens = agent_deployed[a] > 0
            at_capacity = agent_deployed[a] >= k

            if has_tokens:
                for c_idx in range(num_customers):
                    if c_idx in dropped:
                        continue
                    cust_loc = customers[c_idx]
                    travel = dist[loc][cust_loc]
                    arrival = time + travel

                    if arrival < best_score:
                        best_score = arrival
                        best_action = ('D', a, c_idx, arrival)

            if not at_capacity:
                for c_idx in range(num_customers):
                    if c_idx not in dropped or c_idx in picked:
                        continue

                    dropper, ready_time = dropped[c_idx]

                    # If allow_mixed=False, only the dropoff agent can do pickup
                    if not allow_mixed and a != dropper:
                        continue

                    cust_loc = customers[c_idx]
                    travel = dist[loc][cust_loc]
                    arrival = time + travel
                    actual_pickup = max(arrival, ready_time)

                    score = actual_pickup
                    if has_tokens and remaining_drops > 0:
                        score += defer_penalty

                    if score < best_score:
                        best_score = score
                        best_action = ('P', a, c_idx, actual_pickup, dropper)

        if best_action is None:
            break

        if best_action[0] == 'D':
            _, a, c_idx, arrival = best_action
            cust_loc = customers[c_idx]
            completion = arrival + proc[cust_loc]

            agent_time[a] = completion
            agent_loc[a] = cust_loc
            agent_deployed[a] -= 1
            dropped[c_idx] = (a, completion)
            drop_agent[c_idx] = a
            agent_events[a].append((c_idx, 'D'))
        else:
            _, a, c_idx, actual_pickup, dropper = best_action
            cust_loc = customers[c_idx]

            agent_time[a] = actual_pickup
            agent_loc[a] = cust_loc
            agent_deployed[a] += 1
            picked.add(c_idx)
            pick_agent[c_idx] = a
            agent_events[a].append((c_idx, 'P'))

    for a in range(m):
        if agent_loc[a] != depot:
            agent_time[a] += dist[agent_loc[a]][depot]

    makespan = max(agent_time)

    # Build drop_position and pick_position from agent_events
    # Position is the index in the agent's FULL tour (preserves interleaving)
    drop_position = {}
    pick_position = {}
    for a in range(m):
        for pos, (c_idx, op) in enumerate(agent_events[a]):
            if op == 'D':
                drop_position[c_idx] = pos
            else:
                pick_position[c_idx] = pos

    # Build chromosome with per-agent normalized position keys
    chrom = np.zeros((num_customers, 4), dtype=np.float32)
    for c_idx in range(num_customers):
        da = drop_agent.get(c_idx, 0)
        pa = pick_agent.get(c_idx, da)

        # Normalize by agent's FULL tour length
        da_tour_len = max(len(agent_events[da]), 1)
        pa_tour_len = max(len(agent_events[pa]), 1)

        chrom[c_idx, 0] = da
        chrom[c_idx, 1] = pa
        chrom[c_idx, 2] = drop_position.get(c_idx, 0) / da_tour_len
        chrom[c_idx, 3] = pick_position.get(c_idx, 0) / pa_tour_len

    return chrom, makespan

def generate_savings_solution(
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    num_customers: int,
    allow_mixed: bool = True
) -> Tuple[np.ndarray, float, Dict]:
    """
    Heuristic: Clarke-Wright Savings adapted for VRP-RPD

    Computes savings from merging routes and greedily assigns customers.

    Args:
        allow_mixed: If False, forces same agent for pickup and dropoff (no interleaving)

    Returns: (chromosome, makespan, tours)
    """
    from .decoder import compute_makespan_from_tours
    from .utils import build_chromosome_from_tours

    n = len(dist)

    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]

    # Compute savings: s(i,j) = d(depot,i) + d(depot,j) - d(i,j)
    savings = []
    for i in range(num_customers):
        for j in range(i + 1, num_customers):
            ci = customers[i]
            cj = customers[j]
            save = dist[depot][ci] + dist[depot][cj] - dist[ci][cj]
            savings.append((save, i, j))

    # Sort by savings (descending)
    savings.sort(reverse=True, key=lambda x: x[0])

    # Initialize: each customer in its own route
    customer_route = {c_idx: c_idx for c_idx in range(num_customers)}
    route_customers = {c_idx: [c_idx] for c_idx in range(num_customers)}

    # Merge routes based on savings
    for save, i, j in savings:
        ri = customer_route[i]
        rj = customer_route[j]

        if ri == rj:
            continue  # Already in same route

        # Check if merge is beneficial (combined route not too long)
        combined = route_customers[ri] + route_customers[rj]
        if len(combined) > (num_customers + m - 1) // m * 2:  # Rough balance check
            continue

        # Merge routes
        for c in route_customers[rj]:
            customer_route[c] = ri
        route_customers[ri] = combined
        del route_customers[rj]

        if len(route_customers) <= m:
            break

    # Assign routes to agents
    routes_list = list(route_customers.values())
    while len(routes_list) < m:
        routes_list.append([])

    # Build tours with interleaved D/P
    tours = {a: [] for a in range(m)}
    agent_available = [k] * m
    dropped = {}
    picked = set()

    # Simple assignment: do all dropoffs for a route, then pickups
    for a, route in enumerate(routes_list[:m]):
        # First pass: dropoffs (respecting capacity)
        for c_idx in route:
            if agent_available[a] > 0:
                cust_loc = customers[c_idx]
                tours[a].append((cust_loc, 'D', c_idx))
                agent_available[a] -= 1
                dropped[c_idx] = a

        # Second pass: pickups to free capacity, then more dropoffs
        remaining = [c_idx for c_idx in route if c_idx not in dropped]
        max_iter = len(route) * 3
        for _ in range(max_iter):
            if not remaining and all(c_idx in picked for c_idx in route if c_idx in dropped):
                break

            # Do pickups
            for c_idx in list(dropped.keys()):
                if c_idx in picked:
                    continue
                dropper = dropped[c_idx]

                # If allow_mixed=False, only dropper can do pickup
                if not allow_mixed:
                    if a == dropper and agent_available[a] < k:
                        cust_loc = customers[c_idx]
                        tours[a].append((cust_loc, 'P', c_idx))
                        agent_available[a] += 1
                        picked.add(c_idx)
                        break
                else:
                    # Allow any agent to pick up (check if picker has capacity)
                    if agent_available[a] < k:
                        cust_loc = customers[c_idx]
                        tours[a].append((cust_loc, 'P', c_idx))
                        agent_available[a] += 1
                        picked.add(c_idx)
                        break

            # Do more dropoffs if we have capacity
            for c_idx in remaining[:]:
                if agent_available[a] > 0:
                    cust_loc = customers[c_idx]
                    tours[a].append((cust_loc, 'D', c_idx))
                    agent_available[a] -= 1
                    dropped[c_idx] = a
                    remaining.remove(c_idx)

    # Handle any unassigned customers
    unassigned_d = [c_idx for c_idx in range(num_customers) if c_idx not in dropped]
    unassigned_p = [c_idx for c_idx in dropped if c_idx not in picked]

    for c_idx in unassigned_d:
        for a in range(m):
            if agent_available[a] > 0:
                cust_loc = customers[c_idx]
                tours[a].append((cust_loc, 'D', c_idx))
                agent_available[a] -= 1
                dropped[c_idx] = a
                break

    for c_idx in unassigned_p:
        dropper = dropped[c_idx]

        # If allow_mixed=False, only dropper can do pickup
        if not allow_mixed:
            cust_loc = customers[c_idx]
            tours[dropper].append((cust_loc, 'P', c_idx))
            agent_available[dropper] += 1
            picked.add(c_idx)
        else:
            # Allow any agent to pick up
            for a in range(m):
                cust_loc = customers[c_idx]
                tours[a].append((cust_loc, 'P', c_idx))
                agent_available[a] += 1
                picked.add(c_idx)
                break

    # Compute makespan
    makespan = compute_makespan_from_tours(tours, dist, proc, depot, m, k, customers)
    chrom = build_chromosome_from_tours(tours, num_customers, m)

    if len(picked) != num_customers:
        print(f"  Warning: Savings only assigned {len(picked)} of {num_customers} customers")

    return chrom, makespan, tours


def apply_2opt_improvement(
    tours: Dict,
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    customers,
    max_iterations: int = 100
) -> Tuple[Dict, float]:
    """
    Apply 2-opt local search to improve tours.

    For VRP-RPD, we do intra-route 2-opt that preserves D before P constraint.

    Returns: (improved_tours, improved_makespan)
    """
    from .decoder import compute_makespan_from_tours

    best_tours = {a: list(tours.get(a, [])) for a in range(m)}
    best_makespan = compute_makespan_from_tours(best_tours, dist, proc, depot, m, k, customers)

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for a in range(m):
            tour = best_tours[a]
            n_events = len(tour)

            if n_events < 4:
                continue

            for i in range(n_events - 2):
                for j in range(i + 2, n_events):
                    # Try reversing segment [i+1, j]
                    new_tour = tour[:i+1] + tour[i+1:j+1][::-1] + tour[j+1:]

                    # Check if reversal maintains D-before-P for each customer
                    valid = True
                    drop_pos = {}
                    pick_pos = {}
                    for pos, (loc, op, c_idx) in enumerate(new_tour):
                        if op == 'D':
                            drop_pos[c_idx] = pos
                        else:
                            pick_pos[c_idx] = pos

                    for c_idx in drop_pos:
                        if c_idx in pick_pos:
                            # Check if both are in this agent's tour
                            if drop_pos[c_idx] > pick_pos[c_idx]:
                                valid = False
                                break

                    if not valid:
                        continue

                    # Evaluate new solution
                    test_tours = {agent: list(best_tours[agent]) for agent in range(m)}
                    test_tours[a] = new_tour
                    new_makespan = compute_makespan_from_tours(test_tours, dist, proc, depot, m, k, customers)

                    if new_makespan < best_makespan - 0.01:
                        best_tours[a] = new_tour
                        best_makespan = new_makespan
                        improved = True

    return best_tours, best_makespan


def apply_relocate_improvement(
    tours: Dict,
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    customers,
    max_iterations: int = 50
) -> Tuple[Dict, float]:
    """
    Apply relocate (Or-opt) local search: move a customer's D or P to a different position.

    Returns: (improved_tours, improved_makespan)
    """
    from .decoder import compute_makespan_from_tours

    best_tours = {a: list(tours.get(a, [])) for a in range(m)}
    best_makespan = compute_makespan_from_tours(best_tours, dist, proc, depot, m, k, customers)

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for a in range(m):
            tour = best_tours[a]
            n_events = len(tour)

            for i in range(n_events):
                event = tour[i]
                loc, op, c_idx = event

                # Try moving this event to other positions in same tour
                for j in range(n_events + 1):
                    if j == i or j == i + 1:
                        continue

                    # Create new tour with event moved
                    new_tour = tour[:i] + tour[i+1:]
                    if j > i:
                        new_tour = new_tour[:j-1] + [event] + new_tour[j-1:]
                    else:
                        new_tour = new_tour[:j] + [event] + new_tour[j:]

                    # Check D-before-P constraint
                    valid = True
                    drop_pos = {}
                    pick_pos = {}
                    for pos, (l, o, c) in enumerate(new_tour):
                        if o == 'D':
                            drop_pos[c] = pos
                        else:
                            pick_pos[c] = pos

                    for c in drop_pos:
                        if c in pick_pos and drop_pos[c] > pick_pos[c]:
                            valid = False
                            break

                    if not valid:
                        continue

                    test_tours = {agent: list(best_tours[agent]) for agent in range(m)}
                    test_tours[a] = new_tour
                    new_makespan = compute_makespan_from_tours(test_tours, dist, proc, depot, m, k, customers)

                    if new_makespan < best_makespan - 0.01:
                        best_tours[a] = new_tour
                        best_makespan = new_makespan
                        improved = True
                        break

                if improved:
                    break

            if improved:
                break

    return best_tours, best_makespan


def apply_swap_improvement(
    tours: Dict,
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    customers,
    max_iterations: int = 50
) -> Tuple[Dict, float]:
    """
    Apply swap local search: swap customers between agents.

    Returns: (improved_tours, improved_makespan)
    """
    from .decoder import compute_makespan_from_tours

    best_tours = {a: list(tours.get(a, [])) for a in range(m)}
    best_makespan = compute_makespan_from_tours(best_tours, dist, proc, depot, m, k, customers)

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        # Try swapping dropoff assignments between agents
        for a1 in range(m):
            for a2 in range(a1 + 1, m):
                tour1 = best_tours[a1]
                tour2 = best_tours[a2]

                # Find dropoffs in each tour
                drops1 = [(i, e) for i, e in enumerate(tour1) if e[1] == 'D']
                drops2 = [(i, e) for i, e in enumerate(tour2) if e[1] == 'D']

                for i1, (idx1, (loc1, _, c1)) in enumerate(drops1):
                    for i2, (idx2, (loc2, _, c2)) in enumerate(drops2):
                        # Try swapping these two dropoffs
                        new_tour1 = list(tour1)
                        new_tour2 = list(tour2)

                        new_tour1[idx1] = (loc2, 'D', c2)
                        new_tour2[idx2] = (loc1, 'D', c1)

                        # Also need to update pickups if they're in same tour
                        for i, (l, o, c) in enumerate(new_tour1):
                            if o == 'P' and c == c1:
                                new_tour1[i] = (loc2, 'P', c2)
                            elif o == 'P' and c == c2:
                                new_tour1[i] = (loc1, 'P', c1)

                        for i, (l, o, c) in enumerate(new_tour2):
                            if o == 'P' and c == c1:
                                new_tour2[i] = (loc2, 'P', c2)
                            elif o == 'P' and c == c2:
                                new_tour2[i] = (loc1, 'P', c1)

                        test_tours = {agent: list(best_tours[agent]) for agent in range(m)}
                        test_tours[a1] = new_tour1
                        test_tours[a2] = new_tour2

                        new_makespan = compute_makespan_from_tours(test_tours, dist, proc, depot, m, k, customers)

                        if new_makespan < best_makespan - 0.01:
                            best_tours[a1] = new_tour1
                            best_tours[a2] = new_tour2
                            best_makespan = new_makespan
                            improved = True
                            break

                    if improved:
                        break

                if improved:
                    break

            if improved:
                break

    return best_tours, best_makespan


def generate_2opt_improved_solution(
    dist: np.ndarray,
    proc: np.ndarray,
    depot: int,
    m: int,
    k: int,
    num_customers: int,
    base_heuristic: str = 'nearest_neighbor',
    allow_mixed: bool = True
) -> Tuple[np.ndarray, float, Dict]:
    """
    Generate a solution using a base heuristic, then improve with 2-opt and other local search.

    Args:
        allow_mixed: If False, forces same agent for pickup and dropoff (no interleaving)

    Returns: (chromosome, makespan, tours)
    """
    from .utils import build_chromosome_from_tours

    n = len(dist)
    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]

    # Get base solution
    if base_heuristic == 'max_regret':
        _, base_makespan, tours = generate_max_regret_solution(dist, proc, depot, m, k, num_customers, allow_mixed=allow_mixed)
    elif base_heuristic == 'savings':
        _, base_makespan, tours = generate_savings_solution(dist, proc, depot, m, k, num_customers, allow_mixed=allow_mixed)
    else:  # nearest_neighbor
        _, base_makespan, tours = generate_nearest_neighbor_solution(dist, proc, depot, m, k, num_customers, allow_mixed=allow_mixed)

    # Apply local search improvements
    tours, makespan = apply_2opt_improvement(tours, dist, proc, depot, m, k, customers)
    tours, makespan = apply_relocate_improvement(tours, dist, proc, depot, m, k, customers)
    tours, makespan = apply_swap_improvement(tours, dist, proc, depot, m, k, customers)

    # Second round
    tours, makespan = apply_2opt_improvement(tours, dist, proc, depot, m, k, customers, max_iterations=50)

    chrom = build_chromosome_from_tours(tours, num_customers, m)

    return chrom, makespan, tours
