#!/usr/bin/env python3
"""
VRP-RPD Utility Functions

Contains:
- File loading (TSPLIB, CSV, Jobs)
- Solution simulation
- JSON solution loading
"""

import math
import json
import csv
import numpy as np
from typing import Dict, Tuple

from .models import VRPRPDInstance


import numpy as np
import math
from typing import Tuple

import numpy as np
import math
from typing import Tuple

def load_tsplib(filepath: str) -> Tuple[np.ndarray, int, np.ndarray | None]:
    edge_weight_type = None
    edge_weight_format = None
    dimension = None

    coords = []
    dist_values = []

    reading_coords = False
    reading_weights = False

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()

            if not line or line == "EOF":
                reading_coords = False
                reading_weights = False
                continue

            # Header parsing
            if line.startswith("DIMENSION"):
                dimension = int(line.split(":")[1])
            elif line.startswith("EDGE_WEIGHT_TYPE"):
                edge_weight_type = line.split(":")[1].strip()
            elif line.startswith("EDGE_WEIGHT_FORMAT"):
                edge_weight_format = line.split(":")[1].strip()

            # Section switches
            elif line == "NODE_COORD_SECTION":
                reading_coords = True
                reading_weights = False
                continue
            elif line == "EDGE_WEIGHT_SECTION":
                reading_weights = True
                reading_coords = False
                continue
            elif line.endswith("_SECTION"):
                # Any other section → stop reading
                reading_coords = False
                reading_weights = False
                continue

            # Read coordinate data
            if reading_coords:
                parts = line.split()
                if len(parts) >= 3:
                    coords.append((float(parts[1]), float(parts[2])))

            # Read distance data
            elif reading_weights:
                dist_values.extend(float(x) for x in line.split())

    # Case 1: Coordinate-based
    if edge_weight_type != "EXPLICIT":
        n = len(coords)
        coords = np.array(coords)
        dist = np.zeros((n, n), dtype=np.float32)

        # Use appropriate distance calculation based on edge weight type
        if edge_weight_type == "GEO":
            # Geographic distance (lat/lon coordinates in degrees)
            # Returns integer distance in kilometers
            for i in range(n):
                for j in range(n):
                    if i == j:
                        dist[i, j] = 0.0
                    else:
                        dist[i, j] = _geo_distance(coords[i, 0], coords[i, 1],
                                                   coords[j, 0], coords[j, 1])
        elif edge_weight_type == "CEIL_2D":
            # Euclidean distance rounded up (ceiling)
            for i in range(n):
                for j in range(n):
                    dx = coords[i, 0] - coords[j, 0]
                    dy = coords[i, 1] - coords[j, 1]
                    dist[i, j] = math.ceil(math.sqrt(dx * dx + dy * dy))
        elif edge_weight_type == "ATT":
            # Pseudo-Euclidean distance (ATT - special rounding for att48)
            for i in range(n):
                for j in range(n):
                    dx = coords[i, 0] - coords[j, 0]
                    dy = coords[i, 1] - coords[j, 1]
                    rij = math.sqrt((dx * dx + dy * dy) / 10.0)
                    tij = int(round(rij))
                    if tij < rij:
                        dist[i, j] = tij + 1
                    else:
                        dist[i, j] = tij
        else:
            # EUC_2D or default: Euclidean distance rounded to nearest integer
            for i in range(n):
                for j in range(n):
                    dx = coords[i, 0] - coords[j, 0]
                    dy = coords[i, 1] - coords[j, 1]
                    dist[i, j] = round(math.sqrt(dx * dx + dy * dy))
        return dist, n, coords

    # Case 2: Explicit matrix
    n = dimension
    dist = np.zeros((n, n), dtype=np.float32)

    if edge_weight_format == "FULL_MATRIX":
        # Full matrix format: n x n values
        idx = 0
        for i in range(n):
            for j in range(n):
                if idx < len(dist_values):
                    dist[i, j] = dist_values[idx]
                    idx += 1
    else:
        # LOWER_DIAG_ROW format (default)
        idx = 0
        for i in range(n):
            for j in range(i + 1):
                dist[i, j] = dist_values[idx]
                dist[j, i] = dist_values[idx]
                idx += 1

    return dist, n, None


def _geo_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate geographic distance using TSPLIB GEO specification.

    Coordinates are given in degrees (latitude, longitude).
    Returns distance in kilometers (rounded to nearest integer as per TSPLIB).
    """
    PI = 3.141592
    RRR = 6378.388  # Earth radius in km

    # Convert latitude/longitude from degrees to radians (TSPLIB format)
    # Format: DDD.MM where DDD is degrees and MM is minutes
    def to_radians(coord):
        deg = int(coord)
        min_val = coord - deg
        return PI * (deg + 5.0 * min_val / 3.0) / 180.0

    lat1_rad = to_radians(lat1)
    lon1_rad = to_radians(lon1)
    lat2_rad = to_radians(lat2)
    lon2_rad = to_radians(lon2)

    # Calculate distance using spherical law of cosines
    q1 = math.cos(lon1_rad - lon2_rad)
    q2 = math.cos(lat1_rad - lat2_rad)
    q3 = math.cos(lat1_rad + lat2_rad)

    distance = RRR * math.acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0

    return int(distance)




def load_csv_distances(filepath: str) -> Tuple[np.ndarray, int]:
    """Load CSV distance matrix"""
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        rows = list(reader)

    n = len(rows)
    dist = np.zeros((n, n))
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            try:
                dist[i, j] = float(val.strip())
            except:
                pass

    return dist, n


def load_jobs(filepath: str, n: int, agents_override: int = None, resources_override: int = None) -> Tuple[np.ndarray, int, int, int]:
    """
    Load jobs file with optional agent/resource override.

    Supports two formats:
    1. CSV format (.csv): customer_id,job_time header with data rows
    2. TXT format (.txt): Legacy format with DEPOT, AGENTS, RESOURCES headers

    Args:
        filepath: Path to jobs file (.csv or .txt)
        n: Number of nodes/customers
        agents_override: Optional override for number of agents
        resources_override: Optional override for resources per agent

    Returns:
        Tuple of (processing_times, depot_index, num_agents, resources_per_agent)
    """
    proc = np.zeros(n)

    # Auto-adjust agents and resources based on problem size
    # Small problems (< 24 customers): 3 agents, 5 resources
    # Larger problems (>= 24 customers): 6 agents, 4 resources
    num_customers = n - 1  # Assuming depot is one of the nodes
    if num_customers < 24:
        depot, agents, resources = 0, 3, 5
    else:
        depot, agents, resources = 0, 6, 4

    # Detect file format by extension
    if filepath.endswith('.csv'):
        # CSV format: customer_id,job_time
        with open(filepath, 'r', encoding='utf-8-sig') as f:
            reader = csv.reader(f)
            header = next(reader, None)  # Skip header row

            for row in reader:
                if len(row) >= 2:
                    try:
                        customer_id = int(row[0].strip())
                        job_time = float(row[1].strip())

                        # customer_id is 1-indexed, convert to 0-indexed
                        #node_0idx = customer_id - 1
                        if 0 <= customer_id < n:
                            proc[customer_id] = job_time
                    except (ValueError, IndexError):
                        continue

        # CSV format doesn't include depot/agents/resources info
        # Use defaults (can be overridden by parameters)

    else:
        # TXT format (legacy): supports both simple and full format
        reading = False

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                if line.startswith('DEPOT:'):
                    depot_1idx = int(line.split(':')[1].strip())
                    depot = depot_1idx - 1
                elif line.startswith('NUM_AGENTS:') or line.startswith('AGENTS:'):
                    agents = int(line.split(':')[1].strip())
                elif line.startswith('RESOURCES_PER_AGENT:') or line.startswith('RESOURCES:'):
                    resources = int(line.split(':')[1].strip())
                elif 'JOB' in line and 'SECTION' in line:
                    reading = True
                elif line == 'EOF':
                    reading = False
                elif reading:
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            node_idx = int(parts[0])
                            #node_0idx = node_1idx - 1
                            if 0 <= node_idx < n:
                                proc[node_idx] = float(parts[1])
                        except:
                            pass
                else:
                    # Simple format: just job times, one per line (1-indexed by line number)
                    try:
                        job_time = float(line)
                        # Line-based indexing for simple format
                        # This is handled differently - let's keep the existing logic
                    except ValueError:
                        pass

    if agents_override is not None:
        agents = agents_override
    if resources_override is not None:
        resources = resources_override

    return proc, depot, agents, resources


def load_solution_from_json(filepath: str, instance: VRPRPDInstance) -> Tuple[np.ndarray, float]:
    """
    Load a solution chromosome from a JSON file.
    Supports BRKGA format, Hexaly format, and Heuristic format.

    Args:
        filepath: Path to the JSON solution file
        instance: VRPRPDInstance to validate compatibility

    Returns:
        Tuple of (chromosome, makespan) from the JSON file

    Raises:
        ValueError: If the JSON file is incompatible with the instance
    """
    with open(filepath, 'r') as f:
        solution_data = json.load(f)

    # Detect format: Check for heuristic format first
    if 'all_heuristics' in solution_data and 'best_solution' in solution_data:
        # Heuristic format - extract best solution
        problem = solution_data.get('problem', {})
        best_heuristics = solution_data['all_heuristics']

        if not best_heuristics:
            raise ValueError("Heuristic JSON has no solutions in 'all_heuristics'")

        # First entry is the best (sorted by makespan)
        best = best_heuristics[0]

        # Validate compatibility
        if problem.get('num_customers') and problem['num_customers'] != instance.num_customers:
            raise ValueError(f"Incompatible solution: expected {instance.num_customers} customers, "
                            f"got {problem['num_customers']}")
        if problem.get('num_agents') and problem['num_agents'] != instance.m:
            raise ValueError(f"Incompatible solution: expected {instance.m} agents, "
                            f"got {problem['num_agents']}")
        if problem.get('resources_per_agent') and problem['resources_per_agent'] != instance.k:
            raise ValueError(f"Incompatible solution: expected {instance.k} resources, "
                            f"got {problem['resources_per_agent']}")

        # Extract chromosome directly (heuristic format already has it)
        chromosome = np.array(best['chromosome'], dtype=np.float32)
        makespan = best['makespan']

        print(f"Loaded heuristic solution: {best['heuristic']} with makespan {makespan:.2f}")
        return chromosome, makespan

    elif 'solution' in solution_data:
        # BRKGA format
        solution = solution_data['solution']
        problem = solution_data.get('problem', {})
        makespan = solution.get('makespan', float('inf'))
        routes = solution.get('routes', [])

        # Validate compatibility
        if problem.get('num_customers') and problem['num_customers'] != instance.num_customers:
            raise ValueError(f"Incompatible solution: expected {instance.num_customers} customers, "
                            f"got {problem['num_customers']}")
        if problem.get('num_agents') and problem['num_agents'] != instance.m:
            raise ValueError(f"Incompatible solution: expected {instance.m} agents, "
                            f"got {problem['num_agents']}")
        if problem.get('resources_per_agent') and problem['resources_per_agent'] != instance.k:
            raise ValueError(f"Incompatible solution: expected {instance.k} resources, "
                            f"got {problem['resources_per_agent']}")

        # Extract stops using BRKGA format
        def get_operation(stop):
            return stop.get('operation', '')

    elif 'makespan' in solution_data and 'routes' in solution_data:
        # Hexaly format
        makespan = solution_data.get('makespan', float('inf'))
        routes = solution_data.get('routes', [])
        problem = solution_data.get('problem', {})

        # Validate compatibility
        if problem.get('n_customers') and problem['n_customers'] != instance.num_customers:
            raise ValueError(f"Incompatible solution: expected {instance.num_customers} customers, "
                            f"got {problem['n_customers']}")
        if problem.get('n_agents') and problem['n_agents'] != instance.m:
            raise ValueError(f"Incompatible solution: expected {instance.m} agents, "
                            f"got {problem['n_agents']}")
        if problem.get('resources_per_agent') and problem['resources_per_agent'] != instance.k:
            raise ValueError(f"Incompatible solution: expected {instance.k} resources, "
                            f"got {problem['resources_per_agent']}")

        # Extract stops using Hexaly format (op: "D" or "P")
        def get_operation(stop):
            op = stop.get('op', '')
            return 'dropoff' if op == 'D' else 'pickup' if op == 'P' else ''
    else:
        raise ValueError(f"Invalid solution file: unrecognized format (missing 'solution' or 'makespan'/'routes' keys)")

    # Reconstruct chromosome from routes
    if not routes:
        raise ValueError(f"Invalid solution file: missing or empty 'routes' data")

    # Initialize chromosome: [num_customers x 4]
    # columns: [dropoff_agent, pickup_agent, dropoff_key, pickup_key]
    num_customers = instance.num_customers
    chromosome = np.zeros((num_customers, 4), dtype=np.float32)

    # Extract customer assignments and create keys based on route order
    # CRITICAL: The keys must preserve the exact sequence of events in the route
    for route in routes:
        agent = route['agent']
        stops = route['stops']

        # Process stops in sequence order
        position = 0
        for stop in stops:
            cust = stop['node']
            if cust == instance.depot:
                continue

            op = get_operation(stop)
            cust_idx = instance.customers.index(cust) if cust in instance.customers else -1
            if cust_idx < 0:
                continue

            # Assign keys based on position in the sequence
            # Key value = position / total_stops to preserve exact ordering
            key_value = (position + 0.5) / (len(stops) + 1)

            if op == 'dropoff':
                chromosome[cust_idx, 0] = float(agent)  # dropoff agent
                chromosome[cust_idx, 2] = key_value      # dropoff key
            elif op == 'pickup':
                chromosome[cust_idx, 1] = float(agent)  # pickup agent
                chromosome[cust_idx, 3] = key_value      # pickup key

            position += 1

    print(f"Loaded solution from {filepath}: makespan = {makespan:.2f}")
    return chromosome, makespan


def build_chromosome_from_tours(tours: Dict, num_customers: int, m: int, debug: bool = False) -> np.ndarray:
    """
    Build a chromosome from tour dictionary that will decode to the EXACT same tour.

    The decoder sorts events by key within each agent, so we assign sequential keys
    within each agent's tour to preserve the exact execution order.

    Args:
        tours: Dict[agent_id] -> List[(cust_loc, op, c_idx)]
        num_customers: Number of customers
        m: Number of agents
        debug: Print debug info

    Returns:
        Chromosome array of shape (num_customers, 4)
    """
    chrom = np.zeros((num_customers, 4), dtype=np.float32)

    # Track assignments
    drop_agent = {}
    pick_agent = {}
    drop_key = {}
    pick_key = {}

    if debug:
        print("\n=== BUILD CHROMOSOME FROM TOURS ===")

    # Assign keys PER-AGENT based on position within that agent's tour
    # This ensures the decoder's sort preserves the exact order
    for a in range(m):
        tour = tours.get(a, [])
        tour_len = len(tour)
        if tour_len == 0:
            continue

        if debug:
            print(f"Agent {a} tour ({tour_len} events):")

        for pos, item in enumerate(tour):
            if len(item) == 3:
                cust_loc, op, c_idx = item
            else:
                cust_loc, op = item
                c_idx = cust_loc - 1  # Assume 0-indexed depot

            # Key = position / (tour_length + 1) to ensure keys in [0, 1)
            key = (pos + 0.5) / (tour_len + 1)

            if op == 'D':
                drop_agent[c_idx] = a
                drop_key[c_idx] = key
                if debug:
                    print(f"  pos {pos}: D{c_idx} (loc={cust_loc}) -> key={key:.4f}")
            else:
                pick_agent[c_idx] = a
                pick_key[c_idx] = key
                if debug:
                    print(f"  pos {pos}: P{c_idx} (loc={cust_loc}) -> key={key:.4f}")

    # Build chromosome
    for c_idx in range(num_customers):
        da = drop_agent.get(c_idx, 0)
        pa = pick_agent.get(c_idx, da)
        dk = drop_key.get(c_idx, 0.5)
        pk = pick_key.get(c_idx, 0.6)

        chrom[c_idx, 0] = da
        chrom[c_idx, 1] = pa
        chrom[c_idx, 2] = dk
        chrom[c_idx, 3] = pk

    if debug:
        print("\nChromosome (first 5 customers):")
        for c_idx in range(min(5, num_customers)):
            print(f"  C{c_idx}: da={int(chrom[c_idx,0])}, pa={int(chrom[c_idx,1])}, "
                  f"dk={chrom[c_idx,2]:.4f}, pk={chrom[c_idx,3]:.4f}")
        print("=== END BUILD ===\n")

    return chrom


def save_heuristic_solution_json(
    filepath: str,
    tours: Dict,
    makespan: float,
    instance,
    heuristic_name: str
):
    """
    Save heuristic solution to JSON file with full tour information.
    """
    routes = []
    for a in range(instance.m):
        route_stops = []
        for item in tours.get(a, []):
            if len(item) == 3:
                cust_loc, op, c_idx = item
            else:
                cust_loc, op = item
                c_idx = cust_loc - 1 if instance.depot == 0 else cust_loc

            route_stops.append({
                'node': int(cust_loc),
                'operation': 'dropoff' if op == 'D' else 'pickup',
                'customer_index': int(c_idx)
            })
        routes.append({
            'agent': a,
            'stops': route_stops
        })

    solution = {
        'heuristic': heuristic_name,
        'makespan': float(makespan),
        'num_customers': instance.num_customers,
        'num_agents': instance.m,
        'resources_per_agent': instance.k,
        'depot': instance.depot,
        'routes': routes
    }

    with open(filepath, 'w') as f:
        json.dump(solution, f, indent=2)

    print(f"  Saved heuristic solution to {filepath}")


def load_heuristic_solution_json(filepath: str, instance) -> Tuple[np.ndarray, float]:
    """
    Load heuristic solution from JSON and rebuild chromosome with correct encoding.
    """
    with open(filepath, 'r') as f:
        data = json.load(f)

    makespan = data.get('makespan', float('inf'))
    routes = data.get('routes', [])

    # Rebuild tours dictionary
    tours = {}
    for route in routes:
        a = route['agent']
        tours[a] = []
        for stop in route['stops']:
            cust_loc = stop['node']
            op = 'D' if stop['operation'] == 'dropoff' else 'P'
            c_idx = stop.get('customer_index', cust_loc - 1 if instance.depot == 0 else cust_loc)
            tours[a].append((cust_loc, op, c_idx))

    # Build chromosome with correct global keys
    chrom = build_chromosome_from_tours(tours, instance.num_customers, instance.m)

    return chrom, makespan


def simulate_solution(tours: Dict, instance) -> Tuple[Dict, Dict, Dict, Dict]:
    """
    FIXED: Simulate solution using FEASIBILITY-PRESERVING event processing.

    This matches the GA's compute_makespan_numba decoder exactly:
    1. Events are sorted by key within each agent
    2. But execution order respects capacity (k) and precedence constraints
    3. Cross-agent pickups wait for processing completion

    Returns:
        job_times: Dict[cust_loc] -> timing info
        agent_tours: Dict[agent] -> list of (loc, op) in ACTUAL execution order
        agent_completion_times: Dict[agent] -> completion time
        customer_assignment: Dict[cust_loc] -> dropoff agent
    """
    m = instance.m
    k = instance.k
    depot = instance.depot
    dist = instance.dist
    proc = instance.proc

    # Build event lists from tours (already sorted by key from decode_chromosome)
    agent_events = {a: [] for a in range(m)}
    customer_assignment = {}

    for a in range(m):
        for pos, (cust_loc, op, cust_idx) in enumerate(tours.get(a, [])):
            agent_events[a].append({
                'pos': pos,
                'cust_loc': cust_loc,
                'op': op,
                'cust_idx': cust_idx,
                'done': False
            })
            if op == 'D':
                customer_assignment[cust_loc] = a

    # State tracking
    agent_time = [0.0] * m
    agent_loc = [depot] * m
    agent_deployed = [k] * m  # Start with k tokens

    dropoff_complete = {}  # cust_loc -> completion time
    dropped_by = {}        # cust_loc -> agent who dropped
    picked_up = set()

    job_times = {}
    agent_tours_actual = {a: [] for a in range(m)}  # Actual execution order!

    total_events = sum(len(events) for events in agent_events.values())
    completed = 0
    max_iterations = total_events * 3 + 100

    # Event-based simulation - process events in TIME order, respecting feasibility
    for iteration in range(max_iterations):
        if completed >= total_events:
            break

        # Find next feasible event across ALL agents
        best_agent = -1
        best_time = float('inf')
        best_event_idx = -1
        best_event = None

        for a in range(m):
            # Find first undone, FEASIBLE event for this agent
            for ev_idx, event in enumerate(agent_events[a]):
                if event['done']:
                    continue

                cust_loc = event['cust_loc']
                op = event['op']
                cust_idx = event['cust_idx']

                # Check feasibility
                if op == 'D':
                    if agent_deployed[a] <= 0:
                        continue  # No tokens - skip to find a pickup
                else:  # Pickup
                    if cust_loc not in dropoff_complete:
                        continue  # Job not dropped yet - skip
                    if cust_loc in picked_up:
                        continue  # Already picked up - skip
                    if agent_deployed[a] >= k:
                        continue  # At capacity - cannot pick up more

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
                    best_event = event

                # Only consider FIRST feasible event per agent (preserves key order)
                break

        # ===== FIX: Handle stranded pickups when no feasible event found =====
        if best_agent < 0:
            # Check for stranded pickups (dropped but not picked up)
            stranded_pickups = []
            for cust_loc_check in dropoff_complete:
                if cust_loc_check not in picked_up:
                    stranded_pickups.append(cust_loc_check)

            if not stranded_pickups:
                break  # All pickups done, safe to exit

            # Find ANY agent with capacity to handle a stranded pickup
            for stranded_loc in stranded_pickups:
                best_rescue_time = float('inf')
                best_rescue_agent = -1

                for a in range(m):
                    if agent_deployed[a] >= k:
                        continue  # This agent at capacity

                    travel = float(dist[agent_loc[a]][stranded_loc])
                    arrival = max(agent_time[a] + travel, dropoff_complete[stranded_loc])

                    if arrival < best_rescue_time:
                        best_rescue_time = arrival
                        best_rescue_agent = a

                if best_rescue_agent >= 0:
                    # Execute the rescue pickup
                    wait = max(0, dropoff_complete[stranded_loc] - (agent_time[best_rescue_agent] + float(dist[agent_loc[best_rescue_agent]][stranded_loc])))

                    if stranded_loc in job_times:
                        job_times[stranded_loc]['pickup'] = best_rescue_time
                        job_times[stranded_loc]['wait'] = wait
                        job_times[stranded_loc]['pickup_agent'] = best_rescue_agent

                    agent_time[best_rescue_agent] = best_rescue_time
                    agent_loc[best_rescue_agent] = stranded_loc
                    picked_up.add(stranded_loc)
                    agent_deployed[best_rescue_agent] += 1
                    agent_tours_actual[best_rescue_agent].append((stranded_loc, 'P'))

                    # Mark the original pickup event as done
                    for a in range(m):
                        for ev_idx, event in enumerate(agent_events[a]):
                            if event['cust_loc'] == stranded_loc and event['op'] == 'P' and not event['done']:
                                event['done'] = True
                                break

                    completed += 1
                    best_agent = best_rescue_agent  # Signal we did something
                    break  # Process one stranded pickup per iteration

            if best_agent < 0:
                break  # No agent can rescue - truly stuck
            continue  # Skip normal execution, already handled rescue
        # ===== END FIX =====

        # Execute the event
        cust_loc = best_event['cust_loc']
        op = best_event['op']
        cust_idx = best_event['cust_idx']

        if op == 'D':
            # Dropoff
            agent_deployed[best_agent] -= 1
            dropped_by[cust_loc] = best_agent
            proc_time = float(proc[cust_loc])
            completion = best_time + proc_time
            dropoff_complete[cust_loc] = completion

            job_times[cust_loc] = {
                'dropoff': best_time,
                'start': best_time,
                'end': completion,
                'dropoff_agent': best_agent
            }

            # Agent leaves immediately (doesn't wait for processing)
            agent_time[best_agent] = best_time
        else:
            # Pickup
            wait = max(0, dropoff_complete[cust_loc] - (agent_time[best_agent] + float(dist[agent_loc[best_agent]][cust_loc])))

            if cust_loc in job_times:
                job_times[cust_loc]['pickup'] = best_time
                job_times[cust_loc]['wait'] = wait
                job_times[cust_loc]['pickup_agent'] = best_agent

            agent_time[best_agent] = best_time
            picked_up.add(cust_loc)

            # Pick up token
            agent_deployed[best_agent] += 1

        agent_loc[best_agent] = cust_loc
        agent_events[best_agent][best_event_idx]['done'] = True
        agent_tours_actual[best_agent].append((cust_loc, op))
        completed += 1

    # Compute completion times (return to depot)
    agent_completion_times = {}
    for a in range(m):
        if agent_tours_actual[a]:
            completion = agent_time[a] + float(dist[agent_loc[a]][depot])
        else:
            completion = 0.0
        agent_completion_times[a] = completion

    return job_times, agent_tours_actual, agent_completion_times, customer_assignment


def verify_makespan(agent_completion_times: Dict) -> float:
    """Compute correct makespan as max of all agent completion times."""
    if not agent_completion_times:
        return 0.0
    return max(agent_completion_times.values())
