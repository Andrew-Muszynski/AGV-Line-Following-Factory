"""
Vehicle Routing Problem with Resource-Constrained Pickup and Delivery (VRP-RPD)
with Mixed Interleaving - Gurobi Implementation with Logging

Author: Generated for VRP-RPD Research
Date: January 2026
"""

import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
import sys
from datetime import datetime


class Logger:
    """
    Logger class that writes to both terminal and file simultaneously
    """
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, 'w', encoding='utf-8')
        
        # Write header to log file
        self.log.write(f"="*70 + "\n")
        self.log.write(f"VRP-RPD Solver Log\n")
        self.log.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.log.write(f"="*70 + "\n\n")
        self.log.flush()
    
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        self.log.flush()  # Ensure immediate write to file
    
    def flush(self):
        self.terminal.flush()
        self.log.flush()
    
    def close(self):
        # Write footer to log file
        self.log.write("\n" + "="*70 + "\n")
        self.log.write(f"Ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.log.write(f"="*70 + "\n")
        self.log.close()


class VRPRPD:
    """
    VRP-RPD with Mixed Interleaving Solver using Gurobi
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
            - processing_times[0] = processing time for customer 1
            - processing_times[n_customers-1] = processing time for customer n_customers
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
        self.model = gp.Model("VRP-RPD")
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
        # First event indicator
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
            
            # Exactly one first event per agent (since each agent must do at least one event in this problem)
            self.model.addConstr(
                gp.quicksum(self.firstEvent[a, pos] for pos in range(1, 2*self.n + 1)) == 1,
                name=f"exactly_one_firstEvent_{a}"
            )

        # SEQUENTIAL POSITIONS (no gaps in agent's tour)
        print("  - Sequential position constraints...")
        for a in self.A:
            for pos in range(2, 2*self.n + 1):
                # If agent has event at position pos, must have event at pos-1
                self.model.addConstr(
                    self.hasEvent[a, pos] <= self.hasEvent[a, pos-1],
                    name=f"sequential_pos_{a}_{pos}"
                )
        
        # 2. PRECEDENCE CONSTRAINTS
        print("  - Precedence constraints...")
        
        # Pickup must occur after dropoff + processing
        for c in self.C:
            self.model.addConstr(
                self.t_P[c] >= self.t_D[c] + self.p[c-1],  # c-1 because p is 0-indexed
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
        
        # Location matches customer (dropoff)
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.model.addConstr(
                        self.s[a, pos] >= c - self.M * (1 - self.delta_D[a, c, pos]),
                        name=f"loc_match_D_lb_{a}_{c}_{pos}"
                    )
                    self.model.addConstr(
                        self.s[a, pos] <= c + self.M * (1 - self.delta_D[a, c, pos]),
                        name=f"loc_match_D_ub_{a}_{c}_{pos}"
                    )
        
        # Location matches customer (pickup)
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.model.addConstr(
                        self.s[a, pos] >= c - self.M * (1 - self.delta_P[a, c, pos]),
                        name=f"loc_match_P_lb_{a}_{c}_{pos}"
                    )
                    self.model.addConstr(
                        self.s[a, pos] <= c + self.M * (1 - self.delta_P[a, c, pos]),
                        name=f"loc_match_P_ub_{a}_{c}_{pos}"
                    )
        
        # Empty position handling
        for a in self.A:
            for pos in range(1, 2*self.n + 1):
                self.model.addConstr(
                    self.s[a, pos] <= self.n * self.hasEvent[a, pos],
                    name=f"empty_pos_{a}_{pos}"
                )
        
        # Link position timing to event timing (dropoff)
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.model.addConstr(
                        self.tau[a, pos] >= self.t_D[c] - self.M * (1 - self.delta_D[a, c, pos]),
                        name=f"time_link_D_lb_{a}_{c}_{pos}"
                    )
                    self.model.addConstr(
                        self.tau[a, pos] <= self.t_D[c] + self.M * (1 - self.delta_D[a, c, pos]),
                        name=f"time_link_D_ub_{a}_{c}_{pos}"
                    )
        
        # Link position timing to event timing (pickup)
        for a in self.A:
            for c in self.C:
                for pos in range(1, 2*self.n + 1):
                    self.model.addConstr(
                        self.tau[a, pos] >= self.t_P[c] - self.M * (1 - self.delta_P[a, c, pos]),
                        name=f"time_link_P_lb_{a}_{c}_{pos}"
                    )
                    self.model.addConstr(
                        self.tau[a, pos] <= self.t_P[c] + self.M * (1 - self.delta_P[a, c, pos]),
                        name=f"time_link_P_ub_{a}_{c}_{pos}"
                    )
        
        # Agent completion time (simplified - return from last event)
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
                # If customer c is the first event (at position 1), 
                # then time must be at least travel time from depot
                self.model.addConstr(
                    self.tau[a, 1] >= self.d[0, c] - self.M * (1 - self.delta_D[a, c, 1] - self.delta_P[a, c, 1]),
                    name=f"depot_to_first_{a}_{c}"
                )

        # TRAVEL TIME BETWEEN CONSECUTIVE EVENTS
        print("  - Travel time between consecutive events...")
        for a in self.A:
            for pos in range(2, 2*self.n + 1):  # positions 2 to 2n
                for i in self.C:  # customer at previous position
                    for j in self.C:  # customer at current position
                        # Indicator: customer i at pos-1, customer j at pos
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
                    # If customer c is at position pos, agent must travel back to depot
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
        print("SOLVING VRP-RPD WITH MIXED INTERLEAVING")
        print("="*70)
        print(f"Instance: {self.n} customers, {self.m} agents, capacity {self.k}")
        print(f"Time limit: {time_limit}s, MIP gap: {mip_gap}")
        print("="*70 + "\n")
        
        # Set parameters
        self.model.Params.TimeLimit = time_limit
        self.model.Params.MIPGap = mip_gap
        
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
                if self.hasEvent[a, pos].X > 0.5:  # Event exists at this position
                    # Find which customer and event type
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
    
    def _parse_ga_solution(self, ga_string):
        """
        Parse the GA solution string into a structured format.
        """
        import re
        
        result = {
            'makespan': None,
            'agents': {}
        }
        
        # Extract makespan
        makespan_match = re.search(r'Makespan:\s*([\d.]+)', ga_string)
        if makespan_match:
            result['makespan'] = float(makespan_match.group(1))
        
        # Extract agent operations
        agent_pattern = r'Agent\s+(\d+):.*?Operations:\s*((?:[DP]\d+@[\d.]+\s*)+)'
        
        for match in re.finditer(agent_pattern, ga_string):
            agent_id = int(match.group(1))
            operations_str = match.group(2)
            
            operations = []
            op_pattern = r'([DP])(\d+)@([\d.]+)'
            
            for op_match in re.finditer(op_pattern, operations_str):
                op_type = op_match.group(1)
                customer = int(op_match.group(2))
                time = float(op_match.group(3))
                
                operations.append({
                    'type': op_type,
                    'customer': customer,
                    'time': time
                })
            
            operations.sort(key=lambda x: x['time'])
            result['agents'][agent_id] = operations
        
        return result

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

    def _set_values_from_parsed_solution(self, parsed):
        """
        Set variable values from the parsed GA solution.
        Uses TWO-PASS calculation to handle mixed interleaving properly.
        """
        
        # Pass 1: Calculate all DROPOFF times first
        print("Pass 1: Calculating dropoff times...")
        
        customer_dropoff_time = {}
        
        for ga_agent_id, operations in parsed['agents'].items():
            model_agent = ga_agent_id + 1
            
            if model_agent not in self.A:
                continue
            
            current_time = 0.0
            previous_location = 0  # depot
            
            for op in operations:
                customer = op['customer']
                op_type = op['type']
                
                if customer not in self.C:
                    continue
                
                # Calculate travel time
                travel_time = self.d[previous_location, customer]
                current_time = current_time + travel_time
                
                if op_type == 'D':
                    customer_dropoff_time[customer] = current_time
                else:
                    # For pickups in pass 1, we need to account for waiting
                    if customer in customer_dropoff_time:
                        dropoff_time = customer_dropoff_time[customer]
                        processing_time = self.p[customer - 1]
                        earliest_pickup = dropoff_time + processing_time
                        current_time = max(current_time, earliest_pickup)
                
                previous_location = customer
        
        print(f"  Dropoff times calculated for {len(customer_dropoff_time)} customers")
        
        # Pass 2: Calculate all times with proper precedence enforcement
        print("Pass 2: Calculating all times with precedence...")
        
        customer_pickup_time = {}
        max_completion_time = 0.0
        
        for ga_agent_id, operations in parsed['agents'].items():
            model_agent = ga_agent_id + 1
            
            if model_agent not in self.A:
                print(f"WARNING: Agent {model_agent} not in model's agent set. Skipping.")
                continue
            
            print(f"  Processing Agent {model_agent} (GA Agent {ga_agent_id}): {len(operations)} operations")
            
            current_load = self.k
            current_time = 0.0
            previous_location = 0  # Start at depot
            
            for pos_idx, op in enumerate(operations):
                pos = pos_idx + 1
                customer = op['customer']
                op_type = op['type']
                
                if customer not in self.C:
                    print(f"  WARNING: Customer {customer} not in model's customer set. Skipping.")
                    continue
                
                # Calculate travel time from previous location
                travel_time = self.d[previous_location, customer]
                earliest_arrival = current_time + travel_time
                
                if op_type == 'D':
                    # For dropoff, arrival time is sufficient
                    current_time = earliest_arrival
                    # Update dropoff time
                    customer_dropoff_time[customer] = current_time
                else:
                    # For pickup, must wait for dropoff + processing
                    if customer in customer_dropoff_time:
                        dropoff_time = customer_dropoff_time[customer]
                        processing_time = self.p[customer - 1]
                        earliest_pickup = dropoff_time + processing_time
                        current_time = max(earliest_arrival, earliest_pickup)
                    else:
                        print(f"  WARNING: No dropoff time found for customer {customer}")
                        current_time = earliest_arrival
                    
                    customer_pickup_time[customer] = current_time
                
                # Set all variable values
                self.hasEvent[model_agent, pos].Start = 1
                self.s[model_agent, pos].Start = customer
                self.tau[model_agent, pos].Start = current_time
                
                if op_type == 'D':
                    self.delta_D[model_agent, customer, pos].Start = 1
                    self.x_D[customer, model_agent].Start = 1
                    self.pi_D[model_agent, customer].Start = pos
                    current_load -= 1
                else:
                    self.delta_P[model_agent, customer, pos].Start = 1
                    self.x_P[customer, model_agent].Start = 1
                    self.pi_P[model_agent, customer].Start = pos
                    current_load += 1
                
                self.L[model_agent, pos].Start = current_load
                previous_location = customer
            
            # Set firstEvent for position 1
            if len(operations) > 0:
                self.firstEvent[model_agent, 1].Start = 1
            
            # Set remaining positions as empty
            for pos in range(len(operations) + 1, 2*self.n + 1):
                self.hasEvent[model_agent, pos].Start = 0
                self.L[model_agent, pos].Start = current_load
            
            # Calculate completion time (last event + return to depot)
            if len(operations) > 0:
                last_customer = operations[-1]['customer']
                return_time = self.d[last_customer, 0]
                completion_time = current_time + return_time
                self.T[model_agent].Start = completion_time
                max_completion_time = max(max_completion_time, completion_time)
        
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
                if pickup < dropoff + processing - 0.001:  # Small tolerance
                    print(f"  VIOLATION at customer {c}: pickup={pickup:.2f} < dropoff={dropoff:.2f} + proc={processing:.2f} = {dropoff+processing:.2f}")
                    violations += 1
        
        if violations == 0:
            print("  All precedence constraints satisfied!")
        else:
            print(f"  WARNING: {violations} precedence violations detected!")

    def set_warm_start(self, ga_solution_string):
        """
        Parse a GA solution string and set warm start values for all variables.
        """
        print("\n" + "="*70)
        print("SETTING WARM START FROM GA SOLUTION")
        print("="*70)
        
        # Parse the GA solution
        parsed = self._parse_ga_solution(ga_solution_string)
        
        if parsed is None:
            print("WARNING: Failed to parse GA solution. No warm start set.")
            return
        
        print(f"Parsed makespan: {parsed['makespan']}")
        print(f"Number of agents in GA solution: {len(parsed['agents'])}")
        
        # Initialize all binary variables to 0
        self._initialize_variables_to_zero()
        
        # Set values based on GA solution
        self._set_values_from_parsed_solution(parsed)
        
        self.model.update()
        print("Warm start values set successfully!")
        print("="*70 + "\n")
    
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


def create_example_instance():
    """Create a small example instance for testing"""
    n_customers = 5
    n_agents = 3
    capacity = 1

    # Distance matrix (depot=0, customers=1,2,3,4,5)
    distances = np.array([
        [0, 5, 9, 12, 10, 6],  # From depot
        [5, 0, 7, 9, 12, 10],  # From customer 1
        [9, 7, 0, 5, 10, 12],  # From customer 2
        [12, 9, 5, 0, 6, 10],  # From customer 3
        [10, 12, 10, 6, 0, 7], # From customer 4
        [6, 10, 12, 10, 7, 0]  # From customer 5
    ])

    # Processing times for each customer
    processing_times = np.array([33, 34, 22, 22, 20])

    return n_customers, n_agents, capacity, distances, processing_times


if __name__ == "__main__":
    # Create log filename with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"vrp_rpd_gr21_solver_2*2_{timestamp}.log"
    
    # Redirect stdout to both terminal and file
    logger = Logger(log_filename)
    sys.stdout = logger
    
    try:
        print("="*70)
        print("VRP-RPD WITH MIXED INTERLEAVING - GUROBI SOLVER")
        print("="*70)
        print(f"Log file: {log_filename}")

        # Choose which instance to run
        use_example = False  # Set to False to use Berlin52

        if use_example:
            print("\nCreating small example instance...")
            n, m, k, distances, processing_times = create_example_instance()
        else:
            print("\nLoading gr21 instance data...")
            
            # Load data files
            distances = np.loadtxt('datasets/gr21/gr21_TSPJ_TT.csv', delimiter=',')
            processing_times = np.loadtxt('datasets/gr21/1R20/full_jobs_1_proc_times.txt')
            
            # Configuration
            n = len(processing_times)  # Number of customers (51)
            m = 2  # Number of agents
            k = 2   # Capacity per agent
            
            print(f"Loaded: {distances.shape[0]} locations (1 depot + {n} customers)")
            print(f"Processing times: {len(processing_times)} values")

        print(f"\nInstance configuration: {n} customers, {m} agents, capacity {k}")
        print(f"Distance matrix shape: {distances.shape}")
        print(f"Processing times shape: {processing_times.shape}")
        
        # Calculate appropriate big-M
        max_dist = np.max(distances)
        total_proc = np.sum(processing_times)
        estimated_makespan = max_dist * n * 2 + total_proc
        big_M = estimated_makespan * 2
        
        print(f"Using big-M = {big_M:.2f}\n")

        # GA solution for warm start
        ga_solution = """=== RCMADP Interleaved Solution (Makespan + LS + WS) ===
Makespan: 3668.00
Total Travel Cost: 19905.00
Unserviced: 0 (0 dropoffs, 0 pickups)
Agent 0:  Finish Time: 3617.00  Travel Time: 3406.00  Operations: D7@427.0 D9@607.0 D8@690.0 D40@929.0 P8@1168.0 P9@1372.0 P7@1552.0 P40@1616.0 D18@1708.0 P18@1747.0 D44@1822.0 D37@2116.0 P38@2192.0 P39@2235.0 D36@2276.0 D47@2336.0 P23@2352.0 D34@2505.0 D35@2520.0 D31@2643.0 P44@2794.0 D17@3022.0 P2@3157.0 P17@3292.0 P21@3459.0 P48@3553.0
Agent 1:  Finish Time: 3645.0  Travel Time: 3332.0  Operations: D43@154.0 D15@286.0 D28@487.0 P28@663.0 D29@923.0 D22@1102.0 P29@1418.0 D19@1607.0 P49@1642.0 P15@1772.0 D46@2311.0 D25@2497.0 P26@2623.0 P25@2749.0 P27@2908.0 P47@3378.0
Agent 2:  Finish Time: 3564.0  Travel Time: 3203.0  Operations: D20@287.0 D41@577.0 D6@653.0 D1@698.0 P6@1085.0 P1@1130.0 P41@1252.0 D16@1660.0 D2@1877.0 P16@2094.0 D30@2390.0 P20@2540.0 P22@2727.0 P19@2821.0 D48@3085.0 D21@3179.0 P30@3283.0 P31@3473.0
Agent 3:  Finish Time: 3664.0  Travel Time: 3214.0  Operations: D45@280.0 D26@799.0 D12@984.0 D13@1190.0 P12@1396.0 D51@1675.0 P51@2125.0 P13@2444.0 P46@2809.0 P35@3539.0
Agent 4:  Finish Time: 3668.0  Travel Time: 3175.0  Operations: D38@166.0 D42@490.0 D32@855.0 P32@1348.0 D10@2054.0 D50@2339.0 P10@2624.0 P11@3011.0 P24@3256.0 P34@3546.0
Agent 5:  Finish Time: 3653.0  Travel Time: 3575.0  Operations: D33@135.0 D49@379.0 P45@645.0 D39@824.0 P33@963.0 P43@1043.0 D23@1227.0 D4@1259.0 D14@1284.0 P42@1526.0 D3@1771.0 P3@1802.0 D5@1872.0 P14@1912.0 P4@1937.0 D24@2087.0 D11@2332.0 D27@2514.0 P50@2851.0 P5@3316.0 P37@3402.0 P36@3445.0
Solve Time: 136.91 seconds
Solution Valid: YES"""

        # Create and solve problem
        vrp = VRPRPD(n, m, k, distances, processing_times, big_M=big_M)
        vrp.create_variables()
        vrp.add_constraints()
        vrp.set_objective()
        
        # SET WARM START FROM GA SOLUTION
        #vrp.set_warm_start(ga_solution)

        # Solve with time limit
        solution = vrp.solve(time_limit=7200, mip_gap=0.05)

        # Print solution
        if solution:
            vrp.print_solution(solution)
            
    finally:
        # Restore stdout and close log file
        sys.stdout = logger.terminal
        logger.close()
        print(f"\nAll output has been saved to: {log_filename}")