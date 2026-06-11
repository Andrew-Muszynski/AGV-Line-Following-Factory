#!/usr/bin/env python3
"""
VRP-RPD Data Structures

Contains the core data structures for the VRP-RPD problem.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class VRPRPDInstance:
    """Problem instance data"""
    distance_matrix: np.ndarray
    processing_times: np.ndarray
    num_agents: int
    resources_per_agent: int
    depot: int = 0
    coordinates: np.ndarray = None

    def __post_init__(self):
        self.n = len(self.distance_matrix)
        self.m = self.num_agents
        self.k = self.resources_per_agent
        self.num_customers = self.n - 1

        if self.depot == 0:
            self.customers = list(range(1, self.n))
        else:
            self.customers = [i for i in range(self.n) if i != self.depot]

        self.dist = self.distance_matrix
        self.proc = self.processing_times
        self.coords = self.coordinates
