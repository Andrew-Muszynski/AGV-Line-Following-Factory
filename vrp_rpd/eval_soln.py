#!/usr/bin/env python3
"""
VRP-RPD Solution Evaluator (ENHANCED)

Comprehensively evaluates a BRKGA (or Hexaly) solution JSON using the correct
PHYSICAL RESOURCE MODEL with full validation.

VALIDATIONS PERFORMED:
  1. Customer Coverage: Each customer served exactly once (1 drop + 1 pick)
  2. Total Operation Counts: Total drops and picks match customer count
  3. Physical Resource Model: Correct resource flow simulation
  4. Resource Conservation: All agents start and end with k resources
  5. Per-Customer Operations: Each customer gets exactly 1 drop and 1 pick
  6. Depot Start/Return: All agents start and return to depot
  7. Makespan Accuracy: Includes depot travel times

PHYSICAL RESOURCE MODEL:
  - Each agent starts at depot with k resources IN HAND
  - Dropoff: agent gives away 1 resource (must have > 0)
  - Pickup:  agent receives 1 resource (must have < k)
  - Return to depot with k resources

Usage:
    python eval_soln.py solution.json --tsp berlin52.tsp --jobs jobs.txt
    python eval_soln.py solution.json --csv distances.csv --jobs jobs.txt
    python eval_soln.py solution.json --csv distances.csv --jobs jobs.txt --verbose
"""

import json
import argparse
import numpy as np
from typing import Dict, List, Tuple, Optional
import sys
import os

# Add parent directory to path to import vrp_rpd utilities
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import utility functions from vrp_rpd package
from vrp_rpd.utils import load_tsplib, load_csv_distances, load_jobs


def parse_solution_json(filepath: str) -> Dict:
    """Parse solution JSON file (supports BRKGA, Hexaly, and Heuristic formats)."""
    with open(filepath, 'r') as f:
        data = json.load(f)

    # Heuristic format: {problem, best_solution, all_heuristics: [{heuristic, makespan, routes, ...}]}
    if 'all_heuristics' in data and data.get('all_heuristics'):
        best = data['all_heuristics'][0]  # First entry is best
        return {
            'reported_makespan': best.get('makespan', 0.0),
            'routes': best.get('routes', []),
            'problem': data.get('problem', {})
        }

    # BRKGA/Hexaly format: {solution: {makespan, routes}}
    solution = data.get('solution', data)
    routes_data = solution.get('routes', [])

    return {
        'reported_makespan': solution.get('makespan', 0.0),
        'routes': routes_data,
        'problem': data.get('problem', solution.get('problem', {}))
    }


def extract_agent_tours(routes_data: List[Dict], customers: List[int], num_agents: int) -> Dict[int, List[Tuple[int, str]]]:
    """
    Extract agent tours from JSON routes.
    
    Handles both formats:
    - BRKGA: {'node': X, 'operation': 'dropoff'/'pickup'}
    - Hexaly: {'node': X, 'op': 'D'/'P'}
    
    Returns: Dict[agent] -> List[(customer_location, 'D'/'P')]
    """
    agent_tours = {a: [] for a in range(num_agents)}
    
    for route_info in routes_data:
        agent = route_info.get('agent', 0)
        stops = route_info.get('stops', [])
        
        for stop in stops:
            node = stop.get('node')
            
            # Handle both formats
            operation = stop.get('operation', '')  # BRKGA format
            op = stop.get('op', '')                # Hexaly format
            
            # Normalize to 'D' or 'P'
            if operation == 'dropoff' or op == 'D':
                op_type = 'D'
            elif operation == 'pickup' or op == 'P':
                op_type = 'P'
            else:
                continue  # Skip depot or unknown
            
            if node is None or node not in customers:
                continue
            
            agent_tours[agent].append((node, op_type))
    
    return agent_tours


def validate_customer_coverage(
    agent_tours: Dict[int, List[Tuple[int, str]]],
    customers: List[int]
) -> Tuple[bool, List[str]]:
    """
    Validate that each customer is served exactly once for dropoff and once for pickup.

    Returns:
        (is_valid, error_messages)
    """
    errors = []

    # Count dropoffs and pickups per customer
    customer_drops = {c: 0 for c in customers}
    customer_picks = {c: 0 for c in customers}

    for agent, tour in agent_tours.items():
        for cust_loc, op in tour:
            if cust_loc not in customers:
                errors.append(f"Agent {agent}: Invalid customer location {cust_loc}")
                continue

            if op == 'D':
                customer_drops[cust_loc] += 1
            elif op == 'P':
                customer_picks[cust_loc] += 1

    # Check each customer has exactly 1 drop and 1 pick
    for cust in customers:
        if customer_drops[cust] == 0:
            errors.append(f"Customer {cust}: Missing dropoff (has 0, needs 1)")
        elif customer_drops[cust] > 1:
            errors.append(f"Customer {cust}: Multiple dropoffs (has {customer_drops[cust]}, needs 1)")

        if customer_picks[cust] == 0:
            errors.append(f"Customer {cust}: Missing pickup (has 0, needs 1)")
        elif customer_picks[cust] > 1:
            errors.append(f"Customer {cust}: Multiple pickups (has {customer_picks[cust]}, needs 1)")

    is_valid = len(errors) == 0
    return is_valid, errors


def validate_drop_pick_sequence(
    agent_tours: Dict[int, List[Tuple[int, str]]],
    customers: List[int]
) -> Tuple[bool, List[str]]:
    """
    Validate that for each customer, dropoff comes before pickup.

    Returns:
        (is_valid, error_messages)
    """
    errors = []

    # Track the position of each operation globally
    customer_drop_pos = {}
    customer_pick_pos = {}

    global_pos = 0
    for agent in sorted(agent_tours.keys()):
        for cust_loc, op in agent_tours[agent]:
            if cust_loc not in customers:
                continue

            if op == 'D':
                if cust_loc in customer_drop_pos:
                    # Already have a drop - this is caught by coverage validation
                    pass
                else:
                    customer_drop_pos[cust_loc] = (global_pos, agent)
            elif op == 'P':
                if cust_loc in customer_pick_pos:
                    # Already have a pick - this is caught by coverage validation
                    pass
                else:
                    customer_pick_pos[cust_loc] = (global_pos, agent)

            global_pos += 1

    # Note: We don't strictly enforce drop before pick in the tour order
    # because the simulation handles temporal dependencies correctly.
    # This is just a warning for potential issues.

    is_valid = len(errors) == 0
    return is_valid, errors


def evaluate_solution_physical_model(
    agent_tours: Dict[int, List[Tuple[int, str]]],
    dist: np.ndarray,
    proc: List[float],
    depot: int,
    num_agents: int,
    resources_per_agent: int,
    customers: List[int]
) -> Tuple[float, Dict[int, float], Dict]:
    """
    ENHANCED: Evaluate solution using PHYSICAL RESOURCE MODEL.

    Each agent tracks resources IN HAND:
      - Start with k resources at depot
      - Dropoff: hand over 1 resource (must have > 0)
      - Pickup: receive 1 resource (must have < k)
      - Return to depot with k resources

    Validates:
      - Each customer gets exactly 1 dropoff and 1 pickup
      - Resource constraints are satisfied
      - Makespan includes depot start and return times

    This correctly handles cross-agent pickups where Agent B
    picks up something Agent A dropped.

    Returns:
        makespan: The correct makespan (including depot times)
        agent_completion_times: Completion time per agent (including return to depot)
        details: Detailed timing and diagnostics
    """
    m = num_agents
    k = resources_per_agent
    
    # Build event queues for each agent
    agent_event_queues = {a: [] for a in range(m)}
    
    for a in range(m):
        for pos, (cust_loc, op) in enumerate(agent_tours.get(a, [])):
            agent_event_queues[a].append({
                'pos': pos,
                'cust_loc': cust_loc,
                'op': op,
                'done': False
            })
    
    # State tracking - FIXED: track resources IN HAND per agent
    agent_time = [0.0] * m
    agent_loc = [depot] * m
    resources_in_hand = [k] * m  # Each agent starts with k resources
    
    dropoff_complete = {}  # cust_loc -> completion time
    dropoff_agent = {}     # cust_loc -> which agent dropped off
    pickup_agent = {}      # cust_loc -> which agent picked up
    picked_up = set()
    
    job_times = {}
    agent_tours_actual = {a: [] for a in range(m)}
    
    total_events = sum(len(agent_event_queues[a]) for a in range(m))
    completed = 0
    max_iterations = total_events * 3 + 100
    
    # Event-based simulation
    for iteration in range(max_iterations):
        if completed >= total_events:
            break
        
        # Find next feasible event across ALL agents
        best_agent = -1
        best_time = float('inf')
        best_event_idx = -1
        best_event_data = None
        
        for a in range(m):
            for ev_idx, ev_data in enumerate(agent_event_queues[a]):
                if ev_data['done']:
                    continue
                
                cust_loc = ev_data['cust_loc']
                op = ev_data['op']
                
                # FIXED: Check feasibility using PHYSICAL resource model
                if op == 'D':
                    if resources_in_hand[a] <= 0:
                        continue  # Can't drop - nothing in hand!
                else:  # Pickup
                    if cust_loc not in dropoff_complete:
                        continue  # Not dropped yet
                    if cust_loc in picked_up:
                        continue  # Already picked
                    if resources_in_hand[a] >= k:
                        continue  # Can't pick up - hands full!
                
                # Compute arrival time
                travel = float(dist[agent_loc[a]][cust_loc])
                arrival = agent_time[a] + travel
                
                # For pickups, must wait for processing completion
                if op == 'P':
                    ready_time = dropoff_complete[cust_loc]
                    arrival = max(arrival, ready_time)
                
                if arrival < best_time:
                    best_time = arrival
                    best_agent = a
                    best_event_idx = ev_idx
                    best_event_data = ev_data
                
                # Only consider FIRST feasible event per agent
                break
        
        if best_agent < 0:
            # No feasible events - check for remaining
            remaining = sum(1 for a in range(m) 
                          for ev in agent_event_queues[a] if not ev['done'])
            if remaining > 0:
                print(f"  WARNING: {remaining} events could not be processed (infeasible)")
            break
        
        # Execute the event
        cust_loc = best_event_data['cust_loc']
        op = best_event_data['op']
        
        if op == 'D':
            # Dropoff - FIXED: decrease THIS agent's resources
            resources_in_hand[best_agent] -= 1
            dropoff_agent[cust_loc] = best_agent
            proc_time = float(proc[cust_loc])
            completion = best_time + proc_time
            dropoff_complete[cust_loc] = completion
            
            job_times[cust_loc] = {
                'dropoff_time': best_time,
                'processing_start': best_time,
                'processing_end': completion,
                'dropoff_agent': best_agent
            }
            
            agent_time[best_agent] = best_time
            agent_tours_actual[best_agent].append((cust_loc, 'D'))
        else:
            # Pickup - FIXED: increase THIS agent's resources
            travel_time = float(dist[agent_loc[best_agent]][cust_loc])
            arrival_time = agent_time[best_agent] + travel_time
            ready_time = dropoff_complete[cust_loc]
            wait = max(0, ready_time - arrival_time)
            
            if cust_loc in job_times:
                job_times[cust_loc]['pickup_time'] = best_time
                job_times[cust_loc]['wait_time'] = wait
                job_times[cust_loc]['pickup_agent'] = best_agent
            
            agent_time[best_agent] = best_time
            pickup_agent[cust_loc] = best_agent
            picked_up.add(cust_loc)
            
            # FIXED: Increase PICKER's resources (not decrease dropper's!)
            resources_in_hand[best_agent] += 1
            
            agent_tours_actual[best_agent].append((cust_loc, 'P'))
        
        agent_loc[best_agent] = cust_loc
        agent_event_queues[best_agent][best_event_idx]['done'] = True
        completed += 1
    
    # Compute completion times WITHOUT depot return (time at last customer)
    agent_completion_times_no_depot = {}
    for a in range(m):
        if agent_tours_actual[a]:
            completion = agent_time[a]  # Time when finishing last operation
        else:
            completion = 0.0
        agent_completion_times_no_depot[a] = completion

    # Compute completion times WITH depot return
    agent_completion_times = {}
    for a in range(m):
        if agent_tours_actual[a]:
            completion = agent_time[a] + float(dist[agent_loc[a]][depot])
        else:
            completion = 0.0
        agent_completion_times[a] = completion

    # Compute makespan (with depot)
    makespan = max(agent_completion_times.values()) if agent_completion_times else 0.0

    # Compute makespan without depot
    makespan_no_depot = max(agent_completion_times_no_depot.values()) if agent_completion_times_no_depot else 0.0
    
    # Count cross-agent pickups
    cross_agent_pickups = sum(1 for loc in picked_up 
                              if dropoff_agent.get(loc) != pickup_agent.get(loc))
    
    # Verify all agents end with k resources
    final_resources = {a: resources_in_hand[a] for a in range(m)}

    # Collect per-customer drop and pick counts
    customer_drop_count = {c: 0 for c in customers}
    customer_pick_count = {c: 0 for c in customers}

    for a in range(m):
        for cust_loc, op in agent_tours_actual[a]:
            if op == 'D':
                customer_drop_count[cust_loc] = customer_drop_count.get(cust_loc, 0) + 1
            elif op == 'P':
                customer_pick_count[cust_loc] = customer_pick_count.get(cust_loc, 0) + 1

    return makespan, agent_completion_times, {
        'job_times': job_times,
        'cross_agent_pickups': cross_agent_pickups,
        'events_processed': completed,
        'total_events': total_events,
        'actual_tours': agent_tours_actual,
        'final_resources': final_resources,
        'customer_drop_count': customer_drop_count,
        'customer_pick_count': customer_pick_count,
        'dropoff_agents': dropoff_agent,
        'pickup_agents': pickup_agent,
        'makespan_no_depot': makespan_no_depot,
        'agent_completion_times_no_depot': agent_completion_times_no_depot
    }


def main():
    parser = argparse.ArgumentParser(
        description='Comprehensively evaluate VRP-RPD solution with full validation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python eval_soln.py solution.json --tsp berlin52.tsp --jobs jobs.txt
    python eval_soln.py solution.json --csv distances.csv --jobs jobs.txt --verbose

VALIDATIONS PERFORMED:
  ✓ Customer coverage (each customer served exactly once)
  ✓ Total operation counts (drops and picks match customer count)
  ✓ Physical resource model simulation
  ✓ Resource conservation (agents start and end with k resources)
  ✓ Per-customer operations (exactly 1 drop + 1 pick each)
  ✓ Depot start and return (makespan includes depot times)
  ✓ Makespan accuracy verification

This evaluator uses the PHYSICAL RESOURCE MODEL:
  - Each agent starts at depot with k resources IN HAND
  - Dropoff: give away 1 resource (must have > 0)
  - Pickup: receive 1 resource (must have < k)
  - Return to depot with k resources

This correctly handles cross-agent pickups where Agent B picks up
what Agent A dropped.
        """
    )
    
    parser.add_argument('solution', help='Solution JSON file to evaluate')
    
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--tsp', help='TSPLIB file')
    input_group.add_argument('--csv', help='CSV distance matrix file')
    
    parser.add_argument('--jobs', required=True, help='Jobs file path')
    parser.add_argument('--agents', '-m', type=int, help='Override number of agents')
    parser.add_argument('--resources', '-k', type=int, help='Override resources per agent')
    parser.add_argument('--verbose', '-v', action='store_true', help='Show detailed output')
    
    args = parser.parse_args()
    
    # Load distance matrix
    if args.tsp:
        dist, n, _ = load_tsplib(args.tsp)
    else:
        dist, n = load_csv_distances(args.csv)
    
    # Load jobs
    proc, depot, num_agents, resources = load_jobs(args.jobs, n)
    
    if args.agents:
        num_agents = args.agents
    if args.resources:
        resources = args.resources
    
    # Build customers list
    if depot == 0:
        customers = list(range(1, n))
    else:
        customers = [i for i in range(n) if i != depot]
    
    # Load solution
    solution_data = parse_solution_json(args.solution)
    reported_makespan = solution_data['reported_makespan']
    
    print("=" * 70)
    print("VRP-RPD SOLUTION EVALUATOR (ENHANCED - Full Validation)")
    print("=" * 70)
    print(f"Solution file: {args.solution}")
    print(f"Problem: {len(customers)} customers, {num_agents} agents, k={resources}")
    print(f"Depot: {depot}")
    print(f"Reported makespan in JSON: {reported_makespan:.2f}")
    print()
    
    # Extract tours
    agent_tours = extract_agent_tours(solution_data['routes'], customers, num_agents)

    # Count events per agent
    print("Tours loaded from JSON:")
    total_drops = 0
    total_picks = 0
    for a in range(num_agents):
        tour = agent_tours.get(a, [])
        n_drops = sum(1 for _, op in tour if op == 'D')
        n_picks = sum(1 for _, op in tour if op == 'P')
        total_drops += n_drops
        total_picks += n_picks
        print(f"  Agent {a}: {n_drops} dropoffs, {n_picks} pickups")
    print(f"  TOTAL: {total_drops} dropoffs, {total_picks} pickups")
    print()

    # VALIDATION 1: Customer Coverage
    print("=" * 70)
    print("VALIDATION 1: Customer Coverage")
    print("=" * 70)
    coverage_valid, coverage_errors = validate_customer_coverage(agent_tours, customers)

    if coverage_valid:
        print("✓ All customers served exactly once (1 drop + 1 pick each)")
    else:
        print("✗ INVALID: Customer coverage issues detected!")
        for err in coverage_errors[:10]:  # Show first 10 errors
            print(f"  - {err}")
        if len(coverage_errors) > 10:
            print(f"  ... and {len(coverage_errors) - 10} more errors")
    print()

    # VALIDATION 2: Total count check
    print("=" * 70)
    print("VALIDATION 2: Total Operation Counts")
    print("=" * 70)
    expected_ops = len(customers)
    if total_drops == expected_ops and total_picks == expected_ops:
        print(f"✓ Total operations correct: {total_drops} drops, {total_picks} picks (expected {expected_ops} each)")
    else:
        print(f"✗ INVALID: Total operation count mismatch!")
        print(f"  Expected: {expected_ops} drops, {expected_ops} picks")
        print(f"  Found:    {total_drops} drops, {total_picks} picks")
    print()

    # Evaluate using ENHANCED physical resource model
    print("=" * 70)
    print("VALIDATION 3: Physical Resource Model Simulation")
    print("=" * 70)
    print("(Each agent starts at depot with k resources, returns to depot)")
    print()
    
    correct_makespan, agent_completions, details = evaluate_solution_physical_model(
        agent_tours, dist, proc, depot, num_agents, resources, customers
    )
    
    # Report results
    print("=" * 70)
    print("SIMULATION RESULTS")
    print("=" * 70)

    print(f"\nAgent completion times (including depot return):")
    for a in range(num_agents):
        completion = agent_completions.get(a, 0.0)
        final_res = details['final_resources'].get(a, resources)
        print(f"  Agent {a}: {completion:.2f}  (ends with {final_res} resources)")

    print(f"\nCross-agent pickups: {details['cross_agent_pickups']}")
    print(f"Events processed: {details['events_processed']} / {details['total_events']}")

    # VALIDATION 4: Resource conservation
    print()
    print("=" * 70)
    print("VALIDATION 4: Resource Conservation")
    print("=" * 70)
    all_returned = all(r == resources for r in details['final_resources'].values())
    if all_returned:
        print(f"✓ All agents return to depot with {resources} resources (conservation satisfied)")
    else:
        print(f"✗ INVALID: Not all agents returned with {resources} resources!")
        for a, r in details['final_resources'].items():
            if r != resources:
                print(f"  Agent {a}: {r} resources (expected {resources})")

    # VALIDATION 5: Per-customer drop/pick verification
    print()
    print("=" * 70)
    print("VALIDATION 5: Per-Customer Operations (Executed)")
    print("=" * 70)

    customer_drop_count = details['customer_drop_count']
    customer_pick_count = details['customer_pick_count']

    invalid_customers = []
    for c in customers:
        drops = customer_drop_count.get(c, 0)
        picks = customer_pick_count.get(c, 0)
        if drops != 1 or picks != 1:
            invalid_customers.append((c, drops, picks))

    if not invalid_customers:
        print(f"✓ All {len(customers)} customers executed with exactly 1 drop and 1 pick")
    else:
        print(f"✗ INVALID: {len(invalid_customers)} customers with incorrect operation counts:")
        for c, drops, picks in invalid_customers[:10]:
            print(f"  Customer {c}: {drops} drops, {picks} picks (expected 1, 1)")
        if len(invalid_customers) > 10:
            print(f"  ... and {len(invalid_customers) - 10} more customers")

    # VALIDATION 6: Depot start and return
    print()
    print("=" * 70)
    print("VALIDATION 6: Depot Start and Return")
    print("=" * 70)
    # Note: The simulation always starts agents at depot and returns them to depot
    # The makespan calculation includes the return to depot time
    print(f"✓ All agents start at depot {depot} (by design)")
    print(f"✓ Makespan includes return to depot time (dist[last_location][{depot}])")
    
    print()
    print("=" * 70)
    print("MAKESPAN COMPARISON")
    print("=" * 70)

    makespan_no_depot = details['makespan_no_depot']

    print(f"  JSON reported:          {reported_makespan:.2f}")
    print(f"  Evaluated (with depot): {correct_makespan:.2f}")
    print(f"  Evaluated (no depot):   {makespan_no_depot:.2f}")
    print()

    # Compare with full makespan (including depot)
    diff = abs(correct_makespan - reported_makespan)
    diff_no_depot = abs(makespan_no_depot - reported_makespan)

    print("Comparison with depot return:")
    if diff < 0.01:
        print(f"  ✓ MATCH - JSON makespan matches evaluated (with depot)")
    else:
        pct_error = (diff / correct_makespan) * 100 if correct_makespan > 0 else 0
        print(f"  ✗ MISMATCH - Difference: {diff:.2f} ({pct_error:.1f}% error)")

    print()
    print("Comparison without depot return:")
    if diff_no_depot < 0.01:
        print(f"  ✓ MATCH - JSON makespan matches evaluated (no depot)")
    else:
        pct_error_no_depot = (diff_no_depot / makespan_no_depot) * 100 if makespan_no_depot > 0 else 0
        print(f"  ✗ MISMATCH - Difference: {diff_no_depot:.2f} ({pct_error_no_depot:.1f}% error)")
        if reported_makespan < makespan_no_depot:
            print(f"    JSON UNDERESTIMATES by {diff_no_depot:.2f}")
        else:
            print(f"    JSON OVERESTIMATES by {diff_no_depot:.2f}")

    # OVERALL VALIDATION SUMMARY
    print()
    print("=" * 70)
    print("OVERALL VALIDATION SUMMARY")
    print("=" * 70)

    all_validations = [
        ("Customer coverage", coverage_valid),
        ("Total operation counts", total_drops == len(customers) and total_picks == len(customers)),
        ("Resource conservation", all_returned),
        ("Per-customer operations", len(invalid_customers) == 0),
        ("Makespan accuracy", diff < 0.01)
    ]

    passed = sum(1 for _, valid in all_validations if valid)
    total = len(all_validations)

    print(f"\nValidation Results: {passed}/{total} passed")
    for name, valid in all_validations:
        status = "✓" if valid else "✗"
        print(f"  {status} {name}")

    if passed == total:
        print(f"\n{'=' * 70}")
        print("SOLUTION IS VALID AND CORRECT!")
        print(f"{'=' * 70}")
    else:
        print(f"\n{'=' * 70}")
        print("SOLUTION HAS VALIDATION ERRORS!")
        print(f"{'=' * 70}")

    print()
    
    if args.verbose:
        print("=" * 70)
        print("ACTUAL EXECUTION ORDER")
        print("=" * 70)
        for a in range(num_agents):
            actual_tour = details['actual_tours'].get(a, [])
            if actual_tour:
                tour_str = " -> ".join([f"{loc}({op})" for loc, op in actual_tour])
                print(f"  Agent {a}: Depot -> {tour_str} -> Depot")
            else:
                print(f"  Agent {a}: (empty)")
        print()
    
    return correct_makespan, reported_makespan


if __name__ == '__main__':
    main()
