"""
Schedule representation for VRP-RPD.

A Schedule assigns each AGV an ordered list of (node, op) pairs where
op is 'D' (dropoff) or 'P' (pickup). Every customer in the instance's
demand must appear exactly once as 'D' and once as 'P' across all AGV
routes (possibly on different AGVs).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path

from instance import Instance


class Op(str, Enum):
    DROPOFF = "D"
    PICKUP = "P"


@dataclass
class Stop:
    node: int
    op: Op

    def to_dict(self) -> dict:
        return {"node": self.node, "op": self.op.value}

    @classmethod
    def from_dict(cls, d: dict) -> "Stop":
        return cls(node=d["node"], op=Op(d["op"]))


@dataclass
class Schedule:
    """Per-AGV ordered list of stops. Does not include the depot (implicit start/end)."""
    # routes[v] = list of Stop for AGV v
    routes: list[list[Stop]] = field(default_factory=list)

    def num_agvs(self) -> int:
        return len(self.routes)

    # -------- Validation --------

    def validate(self, instance: Instance) -> list[str]:
        """Return a list of error strings. Empty list means valid."""
        errors: list[str] = []
        if len(self.routes) != instance.num_agvs:
            errors.append(
                f"Schedule has {len(self.routes)} AGVs, instance has {instance.num_agvs}"
            )
            return errors

        demand = set(instance.demand)
        drops: dict[int, int] = {}  # customer -> agv index that drops it
        picks: dict[int, int] = {}

        for v, route in enumerate(self.routes):
            num_drops = sum(1 for s in route if s.op == Op.DROPOFF)
            load = num_drops  # AGV starts with one resource per dropoff it'll perform
            if load > instance.capacity:
                errors.append(
                    f"AGV {v} initial load {load} exceeds capacity {instance.capacity}"
                )
            for stop in route:
                if stop.node == instance.depot_node:
                    errors.append(f"AGV {v} has depot in route (should be implicit)")
                    continue
                if stop.node not in demand:
                    errors.append(
                        f"AGV {v} visits node {stop.node} not in demand set"
                    )
                if stop.op == Op.DROPOFF:
                    if stop.node in drops:
                        errors.append(
                            f"Customer {stop.node} dropped off more than once"
                        )
                    drops[stop.node] = v
                    load -= 1
                    if load < 0:
                        errors.append(
                            f"AGV {v} dropoff at {stop.node} causes negative load"
                        )
                else:  # PICKUP
                    if stop.node in picks:
                        errors.append(
                            f"Customer {stop.node} picked up more than once"
                        )
                    picks[stop.node] = v
                    load += 1
                    if load > instance.capacity:
                        errors.append(
                            f"AGV {v} pickup at {stop.node} exceeds capacity "
                            f"{instance.capacity} (load={load})"
                        )

        missing_drops = demand - set(drops.keys())
        missing_picks = demand - set(picks.keys())
        if missing_drops:
            errors.append(f"Customers never dropped off: {sorted(missing_drops)}")
        if missing_picks:
            errors.append(f"Customers never picked up: {sorted(missing_picks)}")

        return errors

    def is_valid(self, instance: Instance) -> bool:
        return not self.validate(instance)

    # -------- Analysis --------

    def cross_agent_pct(self, instance: Instance) -> float:
        """Percentage of customers where dropoff and pickup AGVs differ."""
        drops: dict[int, int] = {}
        picks: dict[int, int] = {}
        for v, route in enumerate(self.routes):
            for stop in route:
                if stop.op == Op.DROPOFF:
                    drops[stop.node] = v
                else:
                    picks[stop.node] = v
        if not drops:
            return 0.0
        cross = sum(1 for c in drops if drops[c] != picks.get(c))
        return 100.0 * cross / len(drops)

    # -------- Serialization --------

    def to_dict(self) -> dict:
        return {"routes": [[s.to_dict() for s in r] for r in self.routes]}

    @classmethod
    def from_dict(cls, d: dict) -> "Schedule":
        return cls(routes=[[Stop.from_dict(s) for s in r] for r in d["routes"]])

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "Schedule":
        return cls.from_dict(json.loads(Path(path).read_text()))

    def pretty(self, instance: Instance) -> str:
        lines = []
        for v, route in enumerate(self.routes):
            parts = []
            for stop in route:
                r, c = instance.node_to_rc(stop.node)
                parts.append(f"{stop.op.value}{stop.node}({r},{c})")
            lines.append(f"  AGV {v}: depot -> " + " -> ".join(parts) + " -> depot")
        return "\n".join(lines)
