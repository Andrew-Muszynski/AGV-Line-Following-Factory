"""
Vehicle Routing Problem with Resource-Constrained Pickup and Delivery (VRP-RPD)
with Mixed Interleaving - OPTIMIZED & FLEXIBLE Gurobi Implementation

OPTIMIZATIONS IMPLEMENTED:
1. Indicator Constraints - Replaces Big-M for tighter LP relaxation
2. Optimized Gurobi Parameters - Tuned for VRP-RPD
3. JSON Warm Start Support - Reads heuristic solutions from paper_results
4. FLEXIBLE DATASET LOADING - Works with any dataset from datasets/ folder

NOTE: Symmetry breaking constraints removed to ensure warm start compatibility

Author: Generated for VRP-RPD Research (Optimized & Flexible Version)
Date: February 2026
"""

import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
import sys
import json
import argparse
from datetime import datetime
from pathlib import Path

# Import utilities from vrp_rpd package
sys.path.insert(0, str(Path(__file__).parent))
from vrp_rpd.utils import load_tsplib, load_csv_distances, load_jobs


class Logger:
    """
    Logger class that writes to both terminal and file simultaneously
    """
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, 'w', encoding='utf-8')

        # Write header to log file
        self.log.write(f"="*70 + "\n")
        self.log.write(f"VRP-RPD Solver Log (OPTIMIZED & FLEXIBLE)\n")
        self.log.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.log.write(f"="*70 + "\n\n")
        self.log.flush()

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        self.log.flush()

    def flush(self):
        self.terminal.flush()
        self.log.flush()

    def close(self):
        self.log.write("\n" + "="*70 + "\n")
        self.log.write(f"Ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.log.write(f"="*70 + "\n")
        self.log.close()


class VRPRPD_Optimized:
    """
    VRP-RPD with Mixed Interleaving Solver using Gurobi - OPTIMIZED VERSION
    """

    def __init__(self, n_customers, n_agents, capacity, distances, processing_times, big_M=10000):
        """
        Initialize the VRP-RPD problem instance

        Parameters:
        -----------
        n_customers : int
            Number of customers (nodes needing service, EXCLUDING depot)
        n_agents : int
            Number of agents/vehicles
        capacity : int
            Resource capacity per agent
        distances : numpy array (n_customers+1, n_customers+1)
            Distance/travel time matrix where:
            - Index 0 = depot
            - Indices 1 to n_customers = customers
        processing_times : numpy array (n_customers,)
            Processing time for each customer (0-indexed array)
        big_M : float
            Large constant for big-M constraints
        """
        self.n = n_customers
        self.m = n_agents
        self.k = capacity
        self.d = distances
        self.p = processing_times
        self.M = big_M

        # Validate inputs
        if distances.shape != (n_customers + 1, n_customers + 1):
            raise ValueError(
                f"Distance matrix shape {distances.shape} doesn't match "
                f"expected ({n_customers + 1}, {n_customers + 1}) for {n_customers} customers + 1 depot"
            )
        if len(processing_times) != n_customers:
            raise ValueError(
                f"Processing times array length {len(processing_times)} doesn't match "
                f"{n_customers} customers"
            )

        # Sets
        self.C = range(1, n_customers + 1)  # Customers: 1 to n
        self.V = range(0, n_customers + 1)  # Nodes: 0 (depot) to n
        self.A = range(1, n_agents + 1)     # Agents: 1 to m
        self.P = range(0, 2 * n_customers + 1)  # Positions: 0 to 2n

        # Model
        self.model = gp.Model("VRP-RPD-Optimized")
        self.model.Params.OutputFlag = 1

        # Variables (to be created)
        self.x_D = {}
        self.x_P = {}
        self.t_D = {}
        self.t_P = {}
        self.pi_D = {}
        self.pi_P = {}
        self.delta_D = {}
        self.delta_P = {}
        self.s = {}
        self.tau = {}
        self.hasEvent = {}
        self.L = {}
        self.T = {}
        self.T_max = None
        self.firstEvent = {}

    def create_variables(self):
        """Create all decision variables"""
        print("Creating variables...")

        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.firstEvent[a, pos] = self.model.addVar(vtype=GRB.BINARY, name=f"firstEvent_{a}_{pos}")

        # Assignment variables
        for c in self.C:
            for a in self.A:
                self.x_D[c, a] = self.model.addVar(vtype=GRB.BINARY, name=f"x_D_{c}_{a}")
                self.x_P[c, a] = self.model.addVar(vtype=GRB.BINARY, name=f"x_P_{c}_{a}")

        # Timing variables
        for c in self.C:
            self.t_D[c] = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f"t_D_{c}")
            self.t_P[c] = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f"t_P_{c}")

        # Position variables
        for a in self.A:
            for c in self.C:
                self.pi_D[a, c] = self.model.addVar(vtype=GRB.INTEGER, lb=0, ub=2*self.n, name=f"pi_D_{a}_{c}")
                self.pi_P[a, c] = self.model.addVar(vtype=GRB.INTEGER, lb=0, ub=2*self.n, name=f"pi_P_{a}_{c}")

        # Delta indicator variables
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.delta_D[a, c, pos] = self.model.addVar(vtype=GRB.BINARY, name=f"delta_D_{a}_{c}_{pos}")
                    self.delta_P[a, c, pos] = self.model.addVar(vtype=GRB.BINARY, name=f"delta_P_{a}_{c}_{pos}")

        # Tour variables
        for a in self.A:
            for pos in self.P:
                self.s[a, pos] = self.model.addVar(vtype=GRB.INTEGER, lb=0, ub=self.n, name=f"s_{a}_{pos}")
                self.tau[a, pos] = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f"tau_{a}_{pos}")
                self.hasEvent[a, pos] = self.model.addVar(vtype=GRB.BINARY, name=f"hasEvent_{a}_{pos}")

        # Capacity variables
        for a in self.A:
            for pos in self.P:
                self.L[a, pos] = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=self.k, name=f"L_{a}_{pos}")

        # Agent completion time
        for a in self.A:
            self.T[a] = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f"T_{a}")

        # Makespan
        self.T_max = self.model.addVar(vtype=GRB.CONTINUOUS, lb=0, name="T_max")

        self.model.update()
        print(f"Created {self.model.NumVars} variables")

    def add_constraints(self):
        """Add all constraints to the model"""
        print("Adding constraints...")

        # 1. SERVICE AND ASSIGNMENT CONSTRAINTS
        print("  - Service and assignment constraints...")

        # Each customer gets exactly one dropoff
        for c in self.C:
            self.model.addConstr(
                gp.quicksum(self.x_D[c, a] for a in self.A) == 1,
                name=f"one_dropoff_{c}"
            )

        # Each customer gets exactly one pickup
        for c in self.C:
            self.model.addConstr(
                gp.quicksum(self.x_P[c, a] for a in self.A) == 1,
                name=f"one_pickup_{c}"
            )

        # ===================================================================
        # SYMMETRY BREAKING CONSTRAINTS - DISABLED
        # ===================================================================
        # Note: Symmetry breaking constraints have been removed to allow
        # warm start solutions to be accepted without modification
        # print("  - [OPTIMIZATION] Symmetry breaking constraints...")
        # (constraints removed)

        # FIRST EVENT INDICATOR CONSTRAINTS
        print("  - First event indicator constraints...")

        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                # firstEvent[a, pos] can only be 1 if hasEvent[a, pos] = 1
                self.model.addConstr(
                    self.firstEvent[a, pos] <= self.hasEvent[a, pos],
                    name=f"firstEvent_needs_event_{a}_{pos}"
                )

                # firstEvent[a, pos] can only be 1 if no events at positions before pos
                for q in range(1, pos):
                    self.model.addConstr(
                        self.firstEvent[a, pos] <= 1 - self.hasEvent[a, q],
                        name=f"firstEvent_no_prior_{a}_{pos}_{q}"
                    )

            # Exactly one first event per agent
            self.model.addConstr(
                gp.quicksum(self.firstEvent[a, pos] for pos in range(1, 2*self.n + 1)) == 1,
                name=f"exactly_one_firstEvent_{a}"
            )

        # SEQUENTIAL POSITIONS (no gaps in agent's tour)
        print("  - Sequential position constraints...")
        for a in self.A:
            for pos in range(2, 2*self.n + 1):
                self.model.addConstr(
                    self.hasEvent[a, pos] <= self.hasEvent[a, pos-1],
                    name=f"sequential_pos_{a}_{pos}"
                )

        # 2. PRECEDENCE CONSTRAINTS
        print("  - Precedence constraints...")

        # Pickup must occur after dropoff + processing
        for c in self.C:
            self.model.addConstr(
                self.t_P[c] >= self.t_D[c] + self.p[c-1],
                name=f"precedence_{c}"
            )

        # 3. CAPACITY CONSTRAINTS
        print("  - Capacity constraints...")

        # Linking assignment to position
        for a in self.A:
            for c in self.C:
                self.model.addConstr(
                    self.pi_D[a, c] >= self.x_D[c, a],
                    name=f"link_pos_D_lb_{a}_{c}"
                )
                self.model.addConstr(
                    self.pi_D[a, c] <= 2 * self.n * self.x_D[c, a],
                    name=f"link_pos_D_ub_{a}_{c}"
                )
                self.model.addConstr(
                    self.pi_P[a, c] >= self.x_P[c, a],
                    name=f"link_pos_P_lb_{a}_{c}"
                )
                self.model.addConstr(
                    self.pi_P[a, c] <= 2 * self.n * self.x_P[c, a],
                    name=f"link_pos_P_ub_{a}_{c}"
                )

        # Position to indicator variables
        for a in self.A:
            for c in self.C:
                self.model.addConstr(
                    self.pi_D[a, c] == gp.quicksum(pos * self.delta_D[a, c, pos]
                                                     for pos in range(1, 2*self.n + 1)),
                    name=f"pos_to_delta_D_{a}_{c}"
                )
                self.model.addConstr(
                    self.pi_P[a, c] == gp.quicksum(pos * self.delta_P[a, c, pos]
                                                     for pos in range(1, 2*self.n + 1)),
                    name=f"pos_to_delta_P_{a}_{c}"
                )

        # Position uniqueness
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    gp.quicksum(self.delta_D[a, c, pos] for c in self.C) +
                    gp.quicksum(self.delta_P[a, c, pos] for c in self.C) <= 1,
                    name=f"pos_unique_{a}_{pos}"
                )

        # Initial load at depot
        for a in self.A:
            self.model.addConstr(
                self.L[a, 0] == self.k,
                name=f"initial_load_{a}"
            )

        # Load update after each position
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    self.L[a, pos] == self.L[a, pos-1] -
                    gp.quicksum(self.delta_D[a, c, pos] for c in self.C) +
                    gp.quicksum(self.delta_P[a, c, pos] for c in self.C),
                    name=f"load_update_{a}_{pos}"
                )

        # Global resource conservation
        self.model.addConstr(
            gp.quicksum(self.L[a, 2*self.n] for a in self.A) == self.m * self.k,
            name="resource_conservation"
        )

        # 4. TOUR STRUCTURE CONSTRAINTS
        print("  - Tour structure constraints...")

        # Depot start
        for a in self.A:
            self.model.addConstr(self.s[a, 0] == 0, name=f"depot_start_loc_{a}")
            self.model.addConstr(self.tau[a, 0] == 0, name=f"depot_start_time_{a}")

        # Event indicator
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    self.hasEvent[a, pos] ==
                    gp.quicksum(self.delta_D[a, c, pos] + self.delta_P[a, c, pos] for c in self.C),
                    name=f"has_event_{a}_{pos}"
                )

        # ===================================================================
        # OPTIMIZATION 2: INDICATOR CONSTRAINTS (Replace Big-M where possible)
        # ===================================================================
        print("  - [OPTIMIZATION] Using indicator constraints for location matching...")

        # Location matches customer using indicator constraints
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    # When delta_D[a,c,pos] = 1, then s[a,pos] = c
                    self.model.addGenConstrIndicator(
                        self.delta_D[a, c, pos], True, self.s[a, pos] == c,
                        name=f"loc_match_D_{a}_{c}_{pos}"
                    )
                    # When delta_P[a,c,pos] = 1, then s[a,pos] = c
                    self.model.addGenConstrIndicator(
                        self.delta_P[a, c, pos], True, self.s[a, pos] == c,
                        name=f"loc_match_P_{a}_{c}_{pos}"
                    )

        # Empty position handling
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    self.s[a, pos] <= self.n * self.hasEvent[a, pos],
                    name=f"empty_pos_{a}_{pos}"
                )

        print("  - [OPTIMIZATION] Using indicator constraints for time linking...")

        # Link position timing to event timing using indicator constraints
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    # When delta_D[a,c,pos] = 1, then tau[a,pos] = t_D[c]
                    self.model.addGenConstrIndicator(
                        self.delta_D[a, c, pos], True, self.tau[a, pos] == self.t_D[c],
                        name=f"time_link_D_{a}_{c}_{pos}"
                    )
                    # When delta_P[a,c,pos] = 1, then tau[a,pos] = t_P[c]
                    self.model.addGenConstrIndicator(
                        self.delta_P[a, c, pos], True, self.tau[a, pos] == self.t_P[c],
                        name=f"time_link_P_{a}_{c}_{pos}"
                    )

        # Agent completion time
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    self.T[a] >= self.tau[a, pos],
                    name=f"completion_time_{a}_{pos}"
                )

        # TRAVEL TIME FROM DEPOT TO FIRST EVENT
        print("  - Travel time from depot to first event...")
        for a in self.A:
            for c in self.C:
                self.model.addConstr(
                    self.tau[a, 1] >= self.d[0, c] - self.M * (1 - self.delta_D[a, c, 1] - self.delta_P[a, c, 1]),
                    name=f"depot_to_first_{a}_{c}"
                )

        # TRAVEL TIME BETWEEN CONSECUTIVE EVENTS
        print("  - Travel time between consecutive events...")
        for a in self.A:
            for pos in range(2, 2*self.n + 1):
                for i in self.C:
                    for j in self.C:
                        at_prev = self.delta_D[a, i, pos-1] + self.delta_P[a, i, pos-1]
                        at_curr = self.delta_D[a, j, pos] + self.delta_P[a, j, pos]

                        self.model.addConstr(
                            self.tau[a, pos] >= self.tau[a, pos-1] + self.d[i, j]
                            - self.M * (2 - at_prev - at_curr),
                            name=f"travel_{a}_{pos}_{i}_{j}"
                        )

        # TRAVEL TIME FROM LAST EVENT BACK TO DEPOT
        print("  - Travel time from last event to depot...")
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                for c in self.C:
                    at_pos = self.delta_D[a, c, pos] + self.delta_P[a, c, pos]

                    self.model.addConstr(
                        self.T[a] >= self.tau[a, pos] + self.d[c, 0] - self.M * (1 - at_pos),
                        name=f"return_depot_{a}_{pos}_{c}"
                    )

        # 5. MAKESPAN CONSTRAINTS
        print("  - Makespan constraints...")

        for a in self.A:
            self.model.addConstr(
                self.T_max >= self.T[a],
                name=f"makespan_{a}"
            )

        print(f"Added {self.model.NumConstrs} constraints")

    def set_objective(self):
        """Set the objective function"""
        self.model.setObjective(self.T_max, GRB.MINIMIZE)

    def optimize_parameters(self):
        """
        OPTIMIZATION 3: Set optimized Gurobi parameters for VRP-RPD
        """
        print("\n[OPTIMIZATION] Setting optimized Gurobi parameters...")

        # Presolve
        self.model.Params.Presolve = 2  # Aggressive presolve
        self.model.Params.Aggregate = 1  # Enable aggregation

        # Cuts
        self.model.Params.Cuts = 2  # Aggressive cut generation
        self.model.Params.MIPFocus = 1  # Focus on finding good feasible solutions

        # Branching
        self.model.Params.VarBranch = 3  # Strong branching

        # Heuristics
        self.model.Params.Heuristics = 0.2  # 20% time on heuristics
        self.model.Params.ImproveStartGap = 0.5  # Use improvement heuristic early

        # Parallel processing
        self.model.Params.Threads = 0  # Use all available cores

        print("Optimized parameters set successfully!")

    def solve(self, time_limit=3600, mip_gap=0.01):
        """
        Solve the model

        Parameters:
        -----------
        time_limit : int
            Time limit in seconds (default: 3600 = 1 hour)
        mip_gap : float
            MIP gap tolerance (default: 0.01 = 1%)
        """
        print("\n" + "="*70)
        print("SOLVING VRP-RPD WITH MIXED INTERLEAVING (OPTIMIZED)")
        print("="*70)
        print(f"Instance: {self.n} customers, {self.m} agents, capacity {self.k}")
        print(f"Time limit: {time_limit}s, MIP gap: {mip_gap}")
        print("="*70 + "\n")

        # Set parameters
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = mip_gap

        # Apply optimized parameters
        self.optimize_parameters()

        # Optimize
        start_time = time.time()
        self.model.optimize()
        solve_time = time.time() - start_time

        # Print results
        print("\n" + "="*70)
        print("SOLUTION SUMMARY")
        print("="*70)

        if self.model.status == GRB.OPTIMAL:
            print("Status: OPTIMAL")
        elif self.model.status == GRB.TIME_LIMIT:
            print("Status: TIME LIMIT REACHED")
        elif self.model.status == GRB.INFEASIBLE:
            print("Status: INFEASIBLE")
            return None
        else:
            print(f"Status: {self.model.status}")

        if self.model.SolCount > 0:
            print(f"Makespan: {self.T_max.X:.2f}")
            print(f"Solve time: {solve_time:.2f}s")
            print(f"MIP Gap: {self.model.MIPGap*100:.2f}%")
            print("="*70 + "\n")

            return self.extract_solution()
        else:
            print("No solution found")
            return None

    def extract_solution(self):
        """Extract and structure the solution"""
        solution = {
            'makespan': self.T_max.X,
            'agents': {},
            'customers': {}
        }

        # Extract agent tours
        for a in self.A:
            tour = []
            for pos in range(1, 2*self.n + 1):
                if self.hasEvent[a, pos].X > 0.5:
                    for c in self.C:
                        if self.delta_D[a, c, pos].X > 0.5:
                            tour.append({
                                'position': pos,
                                'customer': c,
                                'event': 'dropoff',
                                'time': self.tau[a, pos].X,
                                'load_after': self.L[a, pos].X
                            })
                        elif self.delta_P[a, c, pos].X > 0.5:
                            tour.append({
                                'position': pos,
                                'customer': c,
                                'event': 'pickup',
                                'time': self.tau[a, pos].X,
                                'load_after': self.L[a, pos].X
                            })

            solution['agents'][a] = {
                'tour': sorted(tour, key=lambda x: x['position']),
                'completion_time': self.T[a].X,
                'initial_load': self.k,
                'final_load': self.L[a, 2*self.n].X
            }

        # Extract customer assignments
        for c in self.C:
            dropoff_agent = None
            pickup_agent = None

            for a in self.A:
                if self.x_D[c, a].X > 0.5:
                    dropoff_agent = a
                if self.x_P[c, a].X > 0.5:
                    pickup_agent = a

            solution['customers'][c] = {
                'dropoff_agent': dropoff_agent,
                'pickup_agent': pickup_agent,
                'dropoff_time': self.t_D[c].X,
                'pickup_time': self.t_P[c].X,
                'processing_time': self.p[c-1]
            }

        return solution

    def load_json_warm_start(self, json_file_path):
        """
        Load warm start solution from JSON file (paper_results format)

        Parameters:
        -----------
        json_file_path : str or Path
            Path to JSON file containing heuristic solution
        """
        print("\n" + "="*70)
        print("LOADING WARM START FROM JSON")
        print("="*70)
        print(f"File: {json_file_path}")

        with open(json_file_path, 'r') as f:
            data = json.load(f)

        if 'solution' not in data:
            print("ERROR: No 'solution' key found in JSON file")
            return False

        solution = data['solution']
        routes = solution.get('routes', [])
        jobs = solution.get('jobs', {})

        print(f"Loaded makespan: {solution.get('makespan', 'N/A')}")
        print(f"Number of agent routes: {len(routes)}")
        print(f"Pre-calculated job timings: {len(jobs)} jobs")

        # Initialize all variables to zero
        self._initialize_variables_to_zero()

        # Parse and set warm start values using pre-calculated times
        self._set_values_from_json_solution(routes, jobs)

        self.model.update()
        print("Warm start values set successfully!")
        print("="*70 + "\n")

        return True

    def _initialize_variables_to_zero(self):
        """Initialize all binary/integer variables to 0."""

        for c in self.C:
            for a in self.A:
                self.x_D[c, a].Start = 0
                self.x_P[c, a].Start = 0

        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.delta_D[a, c, pos].Start = 0
                    self.delta_P[a, c, pos].Start = 0

        for a in self.A:
            for c in self.C:
                self.pi_D[a, c].Start = 0
                self.pi_P[a, c].Start = 0

        for a in self.A:
            for pos in self.P:
                self.s[a, pos].Start = 0
                self.tau[a, pos].Start = 0
                self.hasEvent[a, pos].Start = 0

        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.firstEvent[a, pos].Start = 0

        for a in self.A:
            for pos in self.P:
                self.L[a, pos].Start = self.k

    def _set_values_from_json_solution(self, routes, jobs):
        """
        Set variable values from JSON solution routes using PRE-CALCULATED times.

        The JSON files contain validated, correct solutions with pre-calculated
        timing information in the 'jobs' dict. We use these times directly.
        """

        # Extract pre-calculated times from jobs dict
        print("Extracting pre-calculated times from jobs data...")
        customer_dropoff_time = {}
        customer_pickup_time = {}

        for customer_str, job_info in jobs.items():
            customer = int(customer_str)
            if customer in self.C:
                customer_dropoff_time[customer] = float(job_info['dropoff_time'])
                customer_pickup_time[customer] = float(job_info['pickup_time'])

        print(f"  Extracted times for {len(customer_dropoff_time)} customers")

        # Now set variable values for each agent's route
        max_completion_time = 0.0

        for route in routes:
            agent_id = route['agent'] + 1  # JSON uses 0-indexed, model uses 1-indexed
            stops = route.get('stops', [])
            route_completion = float(route.get('completion_time', 0))

            if agent_id not in self.A:
                print(f"WARNING: Agent {agent_id} not in model's agent set. Skipping.")
                continue

            print(f"  Processing Agent {agent_id} (JSON Agent {route['agent']}): {len(stops)} stops")

            current_load = self.k

            for pos_idx, stop in enumerate(stops):
                pos = pos_idx + 1
                customer = stop['node']
                operation = stop['operation']

                if customer not in self.C:
                    print(f"  WARNING: Customer {customer} not in model's customer set. Skipping.")
                    continue

                # Use pre-calculated time from jobs dict
                if operation == 'dropoff':
                    event_time = customer_dropoff_time.get(customer, 0)
                else:  # pickup
                    event_time = customer_pickup_time.get(customer, 0)

                # Set all variable values
                self.hasEvent[agent_id, pos].Start = 1
                self.s[agent_id, pos].Start = customer
                self.tau[agent_id, pos].Start = event_time

                if operation == 'dropoff':
                    self.delta_D[agent_id, customer, pos].Start = 1
                    self.x_D[customer, agent_id].Start = 1
                    self.pi_D[agent_id, customer].Start = pos
                    current_load -= 1
                else:  # pickup
                    self.delta_P[agent_id, customer, pos].Start = 1
                    self.x_P[customer, agent_id].Start = 1
                    self.pi_P[agent_id, customer].Start = pos
                    current_load += 1

                self.L[agent_id, pos].Start = current_load

            # Set firstEvent for position 1
            if len(stops) > 0:
                self.firstEvent[agent_id, 1].Start = 1

            # Set remaining positions as empty
            for pos in range(len(stops) + 1, 2*self.n + 1):
                self.hasEvent[agent_id, pos].Start = 0
                self.L[agent_id, pos].Start = current_load

            # Use pre-calculated completion time from route
            self.T[agent_id].Start = route_completion
            max_completion_time = max(max_completion_time, route_completion)

        # Set customer timing variables
        for c in self.C:
            if c in customer_dropoff_time:
                self.t_D[c].Start = customer_dropoff_time[c]
            if c in customer_pickup_time:
                self.t_P[c].Start = customer_pickup_time[c]

        # Set makespan
        self.T_max.Start = max_completion_time

        print(f"Warm start makespan (recalculated): {max_completion_time:.2f}")

        # Verification
        dropoffs_set = sum(1 for c in self.C if c in customer_dropoff_time)
        pickups_set = sum(1 for c in self.C if c in customer_pickup_time)
        print(f"Customers with dropoff set: {dropoffs_set}/{self.n}")
        print(f"Customers with pickup set: {pickups_set}/{self.n}")

        # Verify precedence constraints
        print("Verifying precedence constraints...")
        violations = 0
        for c in self.C:
            if c in customer_dropoff_time and c in customer_pickup_time:
                dropoff = customer_dropoff_time[c]
                pickup = customer_pickup_time[c]
                processing = self.p[c - 1]
                if pickup < dropoff + processing - 0.001:
                    print(f"  VIOLATION at customer {c}: pickup={pickup:.2f} < dropoff={dropoff:.2f} + proc={processing:.2f}")
                    violations += 1

        if violations == 0:
            print("  All precedence constraints satisfied!")
        else:
            print(f"  WARNING: {violations} precedence violations detected!")

    def print_solution(self, solution):
        """Print the solution in a readable format"""
        if solution is None:
            print("No solution to print")
            return

        print("\n" + "="*70)
        print("DETAILED SOLUTION")
        print("="*70)

        print(f"\nMakespan: {solution['makespan']:.2f}")

        print("\n" + "-"*70)
        print("AGENT TOURS")
        print("-"*70)

        for a in sorted(solution['agents'].keys()):
            agent_data = solution['agents'][a]
            print(f"\nAgent {a}:")
            print(f"  Initial load: {agent_data['initial_load']}")
            print(f"  Final load: {agent_data['final_load']:.0f}")
            print(f"  Completion time: {agent_data['completion_time']:.2f}")
            print(f"  Tour:")

            for event in agent_data['tour']:
                event_type = event['event'].upper()
                print(f"    Pos {event['position']:2d}: {event_type:7s} at Customer {event['customer']:2d} "
                      f"(time={event['time']:6.2f}, load_after={event['load_after']:.0f})")

        print("\n" + "-"*70)
        print("CUSTOMER SERVICE")
        print("-"*70)

        for c in sorted(solution['customers'].keys()):
            cust_data = solution['customers'][c]
            print(f"\nCustomer {c}:")
            print(f"  Dropoff:  Agent {cust_data['dropoff_agent']} at time {cust_data['dropoff_time']:.2f}")
            print(f"  Processing: {cust_data['processing_time']:.2f} time units")
            print(f"  Pickup:   Agent {cust_data['pickup_agent']} at time {cust_data['pickup_time']:.2f}")

            if cust_data['dropoff_agent'] != cust_data['pickup_agent']:
                print(f"  ** Mixed interleaving: different agents **")


def load_dataset(dataset_name, variant, job_file=None, num_agents=None, resources_per_agent=None):
    """
    Load a dataset from the datasets folder.

    Parameters:
    -----------
    dataset_name : str
        Name of the dataset (e.g., 'gr21', 'berlin52', 'kroA100')
    variant : str
        Variant type (e.g., 'base', '1R10', '1R20', '2x', '5x')
    job_file : str, optional
        Specific job file name (e.g., 'job_times_1.csv'). If None, uses first available.
    num_agents : int, optional
        Override number of agents
    resources_per_agent : int, optional
        Override resources per agent

    Returns:
    --------
    Tuple: (distances, processing_times, n_customers, n_agents, capacity, depot)
    """
    dataset_dir = Path('datasets') / dataset_name
    variant_dir = dataset_dir / variant

    # Load TSP file (distance matrix)
    tsp_file = dataset_dir / f"{dataset_name}.tsp"
    if not tsp_file.exists():
        raise FileNotFoundError(f"TSP file not found: {tsp_file}")

    print(f"Loading TSP file: {tsp_file}")
    distances, n_nodes, coords = load_tsplib(str(tsp_file))
    print(f"  Loaded {n_nodes} nodes")

    # Load jobs file
    if job_file is None:
        # Find first available CSV file
        csv_files = list(variant_dir.glob('*.csv'))
        if not csv_files:
            raise FileNotFoundError(f"No CSV job files found in {variant_dir}")
        job_file = csv_files[0].name

    job_path = variant_dir / job_file
    if not job_path.exists():
        raise FileNotFoundError(f"Job file not found: {job_path}")

    print(f"Loading job file: {job_path}")
    processing_times_full, depot, agents, resources = load_jobs(
        str(job_path),
        n_nodes,
        agents_override=num_agents,
        resources_override=resources_per_agent
    )

    # Extract only customer processing times (exclude depot at index 0)
    # The load_jobs function returns an array of size n_nodes where:
    # - processing_times_full[0] = depot (always 0)
    # - processing_times_full[1..n] = customers 1 to n
    # We need array of size n_customers with processing times for customers 1 to n
    n_customers = n_nodes - 1  # Excluding depot
    processing_times = processing_times_full[1:n_nodes]  # Extract customers 1 to n

    print(f"  Loaded {n_customers} customers, {agents} agents, {resources} resources per agent")
    print(f"  Processing times array size: {len(processing_times)} (should equal {n_customers})")

    return distances, processing_times, n_customers, agents, resources, depot


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='VRP-RPD Optimized Solver with Flexible Dataset Loading')
    parser.add_argument('--dataset', type=str, default='gr21', help='Dataset name (e.g., gr21, berlin52, kroA100)')
    parser.add_argument('--variant', type=str, default='1R20', help='Variant (e.g., base, 1R10, 1R20, 2x, 5x)')
    parser.add_argument('--job-file', type=str, default=None, help='Specific job file (e.g., job_times_1.csv)')
    parser.add_argument('--agents', type=int, default=None, help='Number of agents (overrides default)')
    parser.add_argument('--capacity', type=int, default=None, help='Capacity per agent (overrides default)')
    parser.add_argument('--time-limit', type=int, default=7200, help='Time limit in seconds (default: 7200)')
    parser.add_argument('--mip-gap', type=float, default=0.05, help='MIP gap tolerance (default: 0.05)')
    parser.add_argument('--warm-start', type=str, default=None, help='Path to warm start JSON file')

    args = parser.parse_args()

    # Create log filename with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"gurobi_vrp_rpd_{args.dataset}_{args.variant}_optimized_{timestamp}.log"

    # Redirect stdout to both terminal and file
    logger = Logger(log_filename)
    sys.stdout = logger

    try:
        print("="*70)
        print("VRP-RPD WITH MIXED INTERLEAVING - OPTIMIZED & FLEXIBLE SOLVER")
        print("="*70)
        print(f"Log file: {log_filename}")
        print("\nOPTIMIZATIONS ENABLED:")
        print("  1. Indicator Constraints (replacing Big-M)")
        print("  2. Optimized Gurobi Parameters")
        print("  3. JSON Warm Start Support")
        print("  4. Flexible Dataset Loading")

        print(f"\nDataset: {args.dataset}")
        print(f"Variant: {args.variant}")
        if args.job_file:
            print(f"Job file: {args.job_file}")

        # Load dataset
        distances, processing_times, n, m, k, depot = load_dataset(
            args.dataset,
            args.variant,
            args.job_file,
            args.agents,
            args.capacity
        )

        print(f"\nInstance configuration: {n} customers, {m} agents, capacity {k}")
        print(f"Distance matrix shape: {distances.shape}")
        print(f"Processing times shape: {processing_times.shape}")

        # Calculate appropriate big-M
        max_dist = np.max(distances)
        total_proc = np.sum(processing_times)
        estimated_makespan = max_dist * n * 2 + total_proc
        big_M = estimated_makespan * 2

        print(f"Using big-M = {big_M:.2f}\n")

        # Create and solve problem
        vrp = VRPRPD_Optimized(n, m, k, distances, processing_times, big_M=big_M)
        vrp.create_variables()
        vrp.add_constraints()
        vrp.set_objective()

        # Load warm start if specified
        if args.warm_start:
            warm_start_path = Path(args.warm_start)
            if warm_start_path.exists():
                print(f"\nLoading warm start from: {warm_start_path}")
                vrp.load_json_warm_start(warm_start_path)
            else:
                print(f"\nWARNING: Warm start file not found: {warm_start_path}")
        else:
            # Try to find a default warm start file
            default_ws_dir = Path(f"paper_results/warm_start/blocks_only/{args.dataset}/{args.variant}")

            if default_ws_dir.exists():
                # Look for warm start file matching the job file or use dataset default
                if args.job_file:
                    job_base = args.job_file.replace('.csv', '').replace('.txt', '')
                    ws_file = default_ws_dir / f"{job_base}_seed0.json"
                else:
                    # Use dataset name default (e.g., gr17_seed0.json)
                    ws_file = default_ws_dir / f"{args.dataset}_seed0.json"

                if ws_file.exists():
                    print(f"\nLoading default warm start from: {ws_file}")
                    vrp.load_json_warm_start(ws_file)
                else:
                    print(f"\nNo warm start file found at: {ws_file}")
                    print(f"Available warm start files in {default_ws_dir}:")
                    for f in sorted(default_ws_dir.glob("*.json"))[:5]:
                        print(f"  - {f.name}")
            else:
                print(f"\nNo warm start directory found: {default_ws_dir}")

        # Solve with time limit
        solution = vrp.solve(time_limit=args.time_limit, mip_gap=args.mip_gap)

        # Print solution
        if solution:
            vrp.print_solution(solution)

    finally:
        # Restore stdout and close log file
        sys.stdout = logger.terminal
        logger.close()
        print(f"\nAll output has been saved to: {log_filename}")
