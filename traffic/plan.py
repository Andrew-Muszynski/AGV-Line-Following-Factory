"""Timed paths: a node sequence with the waits inserted along it.

Timing follows solver.js applyRouteTiming(): at every node a robot may wait,
then turns toward its next move (charged by heading change, except on depot
moves), then drives for the measured move time. So a plan is fully described
by (nodes, waits) -- the same two things the page dispatches as path_nodes
and wait_before_segment_sec.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .model import Label, Testbed


@dataclass
class Step:
    node: Label
    arrive: float
    wait: float = 0.0       # held at the node before turning toward the next move
    turn: float = 0.0       # turning at the node before the next move
    move: float = 0.0       # driving to the next node (0 on the final step)
    heading_in: str | None = None

    @property
    def turn_start(self) -> float:
        return self.arrive + self.wait

    @property
    def drive_start(self) -> float:
        return self.arrive + self.wait + self.turn


@dataclass
class TimedPath:
    agent: str
    steps: list[Step]
    prefix: int = 0          # steps[:prefix] are the fixed launch prefix
    meta: dict = field(default_factory=dict)

    @property
    def end(self) -> float:
        return self.steps[-1].arrive if self.steps else 0.0

    @property
    def nodes(self) -> list:
        return [s.node for s in self.steps]

    @property
    def waits(self) -> list[float]:
        return [s.wait for s in self.steps]


class TimingError(ValueError):
    pass


def time_path(tb: Testbed, agent: str, nodes: list, waits: list | None = None,
              t0: float = 0.0, heading0: str | None = None,
              prefix: int = 0, allow_reversal: bool = False) -> TimedPath:
    """Time a node sequence under the page's model. waits[i] is held at
    nodes[i] before turning toward nodes[i+1]."""
    waits = list(waits or [0.0] * len(nodes))
    if len(waits) < len(nodes):
        waits += [0.0] * (len(nodes) - len(waits))
    steps: list[Step] = []
    t = t0
    prev = heading0
    for i, node in enumerate(nodes):
        step = Step(node=node, arrive=t, heading_in=prev)
        if i + 1 < len(nodes):
            nxt = nodes[i + 1]
            edge = next((e for e in tb.neighbours(node) if e.to == nxt), None)
            if edge is None:
                raise TimingError(f"{agent}: {node!r} -> {nxt!r} is not an edge")
            if (not allow_reversal and tb.reverses(prev, edge.heading)
                    and not tb.dead_end(node)):
                raise TimingError(f"{agent}: about-face at {node!r} "
                                  f"({prev}->{edge.heading}) is banned mid-grid")
            step.wait = max(0.0, float(waits[i]))
            step.turn = 0.0 if edge.depot else tb.turn_seconds(prev, edge.heading)
            step.move = edge.seconds
            t = step.drive_start + step.move
            prev = edge.heading
        steps.append(step)
    return TimedPath(agent=agent, steps=steps, prefix=prefix)


def retime(tb: Testbed, path: TimedPath, waits: list[float]) -> TimedPath:
    """Same nodes, new waits, start time and heading kept."""
    s0 = path.steps[0]
    out = time_path(tb, path.agent, path.nodes, waits, t0=s0.arrive,
                    heading0=s0.heading_in, prefix=path.prefix,
                    allow_reversal=True)
    out.meta = dict(path.meta)
    return out


def concat(tb: Testbed, head: TimedPath, tail_nodes: list,
           tail_waits: list) -> TimedPath:
    """head followed by tail_nodes (tail_nodes[0] == head's last node)."""
    nodes = head.nodes + list(tail_nodes[1:])
    waits = head.waits[:-1] + list(tail_waits)
    return time_path(tb, head.agent, nodes, waits, t0=head.steps[0].arrive,
                     heading0=head.steps[0].heading_in, prefix=head.prefix,
                     allow_reversal=True)


def path_metrics(tb: Testbed, path: TimedPath, service_waits: dict | None = None
                 ) -> dict:
    """Per-path numbers for the DOE's secondary responses."""
    turns90 = sum(1 for s in path.steps if 0 < s.turn < 4.0)
    turns180 = sum(1 for s in path.steps if s.turn >= 4.0)
    hops = max(0, len(path.steps) - 1)
    total_wait = sum(s.wait for s in path.steps[path.prefix:-1]) \
        if len(path.steps) > 1 else 0.0
    service = sum((service_waits or {}).values())
    return {
        "end_sec": path.end,
        "hops": hops,
        "drive_sec": sum(s.move for s in path.steps),
        "turns_90": turns90,
        "turns_180": turns180,
        "wait_sec": total_wait,
        "traffic_wait_sec": max(0.0, total_wait - service),
    }


INF = math.inf
