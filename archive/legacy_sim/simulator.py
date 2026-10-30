"""
SimPy discrete-event simulator for VRP-RPD execution on a grid.

Each AGV follows its assigned route from the Schedule. Between stops,
the path is decomposed into cell-by-cell moves on the grid so we can
visualize AGVs moving and handle stop-and-wait cell contention.

Events emitted (via callback) for the visualizer:
  - ('move_start', t, agv, from_rc, to_rc)
  - ('move_end',   t, agv, at_rc)
  - ('dropoff',    t, agv, node, rc)
  - ('pickup',     t, agv, node, rc)
  - ('processing_start', t, node, rc, duration)
  - ('processing_end',   t, node, rc)
  - ('return',     t, agv, rc)
  - ('wait',       t, agv, at_rc, reason)
  - ('load_change', t, agv, new_load)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

import simpy

from instance import Instance
from schedule import Schedule, Stop, Op


Event = tuple  # (kind, time, *args)


def manhattan_path(
    instance: Instance, start_rc: tuple[int, int], end_rc: tuple[int, int]
) -> list[tuple[int, int]]:
    """Return the list of cells traversed from start to end (inclusive of end,
    exclusive of start). Prefers to move in row first then column."""
    path = []
    r, c = start_rc
    er, ec = end_rc
    # Row moves
    while r != er:
        r += 1 if er > r else -1
        path.append((r, c))
    # Column moves
    while c != ec:
        c += 1 if ec > c else -1
        path.append((r, c))
    return path


@dataclass
class SimConfig:
    enable_contention: bool = True
    contention_wait_tick: float = 0.2  # how long to wait before re-checking a cell
    # AGV staging: how long between successive AGVs leaving the queue
    agv_launch_stagger: float = 0.0
    dwell_time: float = 0.0  # pause at each node to simulate physical pickup/dropoff


@dataclass
class SimResult:
    makespan: float
    events: list[Event] = field(default_factory=list)
    agv_return_times: list[float] = field(default_factory=list)
    waits: dict[int, float] = field(default_factory=dict)  # agv -> total wait time


class GridSimulator:
    """
    Discrete-event simulator. One SimPy process per AGV plus one per active
    processing job. Cell occupancy is tracked for stop-and-wait contention.
    """

    def __init__(
        self,
        instance: Instance,
        schedule: Schedule,
        cfg: Optional[SimConfig] = None,
        event_callback: Optional[Callable[[Event], None]] = None,
    ):
        self.instance = instance
        self.schedule = schedule
        self.cfg = cfg or SimConfig()
        self.env = simpy.Environment()
        # Cell occupancy: (r, c) -> agv index currently there, or None
        self.cell_owner: dict[tuple[int, int], Optional[int]] = {}
        # Processing completion times: customer node -> sim time when ready
        self.drop_complete: dict[int, float] = {}
        # AGV state
        depot_rc = (instance.depot_row, instance.depot_col)
        self.agv_rc: list[tuple[int, int]] = [depot_rc] * instance.num_agvs
        self.agv_returned: list[float] = [math.inf] * instance.num_agvs
        self.wait_times: list[float] = [0.0] * instance.num_agvs
        self.agv_load: list[int] = [0] * instance.num_agvs
        self.events: list[Event] = []
        self.ext_cb = event_callback

    # -------- Logging --------

    def _emit(self, *args) -> None:
        ev = (args[0], float(self.env.now)) + args[1:]
        self.events.append(ev)
        if self.ext_cb:
            self.ext_cb(ev)

    # -------- Primitives --------

    def _acquire_cell(self, agv: int, rc: tuple[int, int]):
        """Wait until the cell is free (or owned by us), then claim it.
        The depot cell is always shared (it's the staging area)."""
        depot_rc = (self.instance.depot_row, self.instance.depot_col)
        if rc == depot_rc:
            return  # depot is never owned; generator with no yields
            yield  # unreachable, but makes this a generator

        waited = 0.0
        while True:
            owner = self.cell_owner.get(rc)
            if owner is None or owner == agv:
                self.cell_owner[rc] = agv
                if waited > 0:
                    self._emit("wait", agv, rc, f"acquired after {waited:.1f}s")
                return
            # Cell is occupied — wait a tick and retry
            if waited == 0.0:
                self._emit("wait", agv, rc, f"blocked by AGV {owner}")
            yield self.env.timeout(self.cfg.contention_wait_tick)
            waited += self.cfg.contention_wait_tick
            self.wait_times[agv] += self.cfg.contention_wait_tick

    def _release_cell(self, rc: tuple[int, int], agv: int) -> None:
        depot_rc = (self.instance.depot_row, self.instance.depot_col)
        if rc == depot_rc:
            return
        if self.cell_owner.get(rc) == agv:
            self.cell_owner[rc] = None

    def _travel(self, agv: int, to_rc: tuple[int, int]):
        """Move AGV cell-by-cell from its current rc to target rc."""
        path = manhattan_path(self.instance, self.agv_rc[agv], to_rc)
        for cell in path:
            from_rc = self.agv_rc[agv]
            self._emit("move_start", agv, from_rc, cell)
            yield self.env.timeout(self.instance.edge_time(from_rc, cell))
            # Release source cell BEFORE acquiring destination to prevent circular
            # hold-and-wait deadlock (A holds X wants Y, B holds Y wants X).
            if self.cfg.enable_contention:
                self._release_cell(from_rc, agv)
            if self.cfg.enable_contention:
                yield from self._acquire_cell(agv, cell)
            self.agv_rc[agv] = cell
            self._emit("move_end", agv, cell)

    # -------- Per-AGV process --------

    def _agv_process(self, agv: int):
        inst = self.instance
        depot_rc = (inst.depot_row, inst.depot_col)
        self.agv_rc[agv] = depot_rc
        # Staggered launch: use SimConfig override if set, else instance.load_time.
        # Each AGV k waits k * load_time before leaving the depot.
        stagger = self.cfg.agv_launch_stagger if self.cfg.agv_launch_stagger > 0 else inst.load_time
        if stagger > 0 and agv > 0:
            self._emit("staging_wait", agv, depot_rc, f"k={agv}, t={agv * stagger:.1f}s")
            yield self.env.timeout(agv * stagger)

        # Load = number of items to deliver (one per DROPOFF stop)
        initial_load = sum(1 for s in self.schedule.routes[agv] if s.op == Op.DROPOFF)
        self.agv_load[agv] = initial_load
        self._emit("depart_depot", agv, depot_rc)
        self._emit("load_change", agv, initial_load)

        for stop in self.schedule.routes[agv]:
            target_rc = inst.node_to_rc(stop.node)
            yield from self._travel(agv, target_rc)

            if stop.op == Op.DROPOFF:
                # Instantaneous dropoff; start processing process
                self._emit("dropoff", agv, stop.node, target_rc)
                if self.cfg.dwell_time > 0:
                    yield self.env.timeout(self.cfg.dwell_time)
                self.agv_load[agv] -= 1
                self._emit("load_change", agv, self.agv_load[agv])
                duration = inst.processing_times[stop.node]
                self.drop_complete[stop.node] = self.env.now + duration
                self._emit("processing_start", stop.node, target_rc, duration)
                self.env.process(self._processing_process(stop.node, target_rc, duration))
            else:
                # Pickup: wait until processing is complete
                ready = self.drop_complete.get(stop.node, math.inf)
                if self.env.now < ready:
                    wait = ready - self.env.now
                    self._emit("wait", agv, target_rc, f"processing p={wait:.1f}s")
                    self.wait_times[agv] += wait
                    yield self.env.timeout(wait)
                self._emit("pickup", agv, stop.node, target_rc)
                if self.cfg.dwell_time > 0:
                    yield self.env.timeout(self.cfg.dwell_time)
                self.agv_load[agv] += 1
                self._emit("load_change", agv, self.agv_load[agv])

        # Return to depot
        yield from self._travel(agv, depot_rc)
        self.agv_returned[agv] = self.env.now
        self.agv_load[agv] = 0
        self._emit("return", agv, depot_rc)
        self._emit("load_change", agv, 0)

    def _processing_process(self, node: int, rc: tuple[int, int], duration: float):
        yield self.env.timeout(duration)
        self._emit("processing_end", node, rc)

    # -------- Run --------

    def run(self) -> SimResult:
        for v in range(self.instance.num_agvs):
            self.env.process(self._agv_process(v))
        # Cap run time to avoid runaway on bad schedules
        self.env.run(until=1e7)

        finite = [t for t in self.agv_returned if t < math.inf]
        makespan = max(finite) if finite else 0.0
        return SimResult(
            makespan=makespan,
            events=self.events,
            agv_return_times=self.agv_returned,
            waits={i: w for i, w in enumerate(self.wait_times)},
        )
