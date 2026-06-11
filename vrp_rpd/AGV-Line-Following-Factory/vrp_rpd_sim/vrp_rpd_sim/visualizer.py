"""
Matplotlib animation of the VRP-RPD simulation.

Renders the 8x8 grid, the depot, machine stickers, and the AGVs moving
along Manhattan paths between intersections. Processing status at each
node is shown via color-coded rings.

The animation is driven by the event log produced by GridSimulator.
Between events, AGV positions are linearly interpolated along the grid
cells they traverse.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import numpy as np

from .instance import Instance
from .schedule import Schedule, Op
from .simulator import SimResult


AGV_COLORS = ["#E63946", "#457B9D", "#2A9D8F", "#F4A261", "#9D4EDD", "#E76F51"]


@dataclass
class AGVTraj:
    """Interpolated trajectory: times[i] -> position rc"""
    times: list[float]
    positions: list[tuple[float, float]]
    carrying_changes: list[tuple[float, int]]  # (time, load)

    def position_at(self, t: float) -> tuple[float, float]:
        if not self.times or t <= self.times[0]:
            return self.positions[0] if self.positions else (0.0, 0.0)
        if t >= self.times[-1]:
            return self.positions[-1]
        # Binary search
        lo, hi = 0, len(self.times) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.times[mid] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = self.times[lo], self.times[hi]
        p0, p1 = self.positions[lo], self.positions[hi]
        if t1 == t0:
            return p0
        alpha = (t - t0) / (t1 - t0)
        return (
            p0[0] + alpha * (p1[0] - p0[0]),
            p0[1] + alpha * (p1[1] - p0[1]),
        )

    def load_at(self, t: float) -> int:
        load = 0
        for tc, new_load in self.carrying_changes:
            if tc <= t:
                load = new_load
            else:
                break
        return load


def build_trajectories(
    instance: Instance, result: SimResult
) -> list[AGVTraj]:
    """
    Reconstruct per-AGV position-over-time from the event log.

    We use move_end events as keyframes: at time t the AGV is at cell rc.
    Between keyframes we linearly interpolate. move_start gives us the
    departure time from a cell.
    """
    depot_rc = (instance.depot_row, instance.depot_col)
    trajs = [
        AGVTraj(times=[0.0], positions=[(depot_rc[0], depot_rc[1])], carrying_changes=[(0.0, 0)])
        for _ in range(instance.num_agvs)
    ]
    loads = [0] * instance.num_agvs

    for ev in result.events:
        kind = ev[0]
        t = ev[1]
        if kind == "move_start":
            agv, from_rc, to_rc = ev[2], ev[3], ev[4]
            trajs[agv].times.append(t)
            trajs[agv].positions.append((float(from_rc[0]), float(from_rc[1])))
        elif kind == "move_end":
            agv, at_rc = ev[2], ev[3]
            trajs[agv].times.append(t)
            trajs[agv].positions.append((float(at_rc[0]), float(at_rc[1])))
        elif kind == "dropoff":
            agv = ev[2]
            loads[agv] -= 1
            trajs[agv].carrying_changes.append((t, loads[agv]))
        elif kind == "pickup":
            agv = ev[2]
            loads[agv] += 1
            trajs[agv].carrying_changes.append((t, loads[agv]))

    # Initialize loads: all AGVs start full (every customer in demand = 1 resource)
    # Actually, the paper says vehicles start at full capacity k, and dropoffs decrement.
    # But in our setup demand = m*k, and we might have fewer stops per AGV.
    # Set initial load = number of dropoffs that AGV performs.
    return trajs


class Visualizer:
    def __init__(
        self,
        instance: Instance,
        schedule: Schedule,
        result: SimResult,
        fps: int = 30,
        playback_speed: float = 4.0,
        figsize: tuple[float, float] = (10, 10),
        show_ghost_paths: bool = True,
    ):
        self.instance = instance
        self.schedule = schedule
        self.result = result
        self.fps = fps
        self.playback_speed = playback_speed
        self.figsize = figsize
        self.show_ghost_paths = show_ghost_paths

        self.trajs = build_trajectories(instance, result)
        self.makespan = result.makespan

        # Initialize AGV loads properly from the schedule
        for v, route in enumerate(schedule.routes):
            initial_load = sum(1 for s in route if s.op == Op.DROPOFF)
            self.trajs[v].carrying_changes.insert(0, (0.0, initial_load))
            # Re-derive subsequent loads from events
            load = initial_load
            new_changes = [(0.0, load)]
            for ev in result.events:
                if ev[0] == "dropoff" and ev[2] == v:
                    load -= 1
                    new_changes.append((ev[1], load))
                elif ev[0] == "pickup" and ev[2] == v:
                    load += 1
                    new_changes.append((ev[1], load))
            self.trajs[v].carrying_changes = new_changes

        # Processing status per node: (start_time, end_time) pairs
        self.proc_windows: dict[int, tuple[float, float]] = {}
        pending_starts: dict[int, float] = {}
        for ev in result.events:
            if ev[0] == "processing_start":
                node = ev[2]
                duration = ev[4]
                pending_starts[node] = ev[1]
                self.proc_windows[node] = (ev[1], ev[1] + duration)
            elif ev[0] == "processing_end":
                pass  # already captured from start+duration

        # State tracking for stop markers
        self.dropped: dict[int, float] = {}  # node -> time dropped off
        self.picked: dict[int, float] = {}
        for ev in result.events:
            if ev[0] == "dropoff":
                self.dropped[ev[3]] = ev[1]
            elif ev[0] == "pickup":
                self.picked[ev[3]] = ev[1]

    def _rc_to_plot(self, rc: tuple[float, float]) -> tuple[float, float]:
        """Convert matrix (row, col) to plot (x, y) with origin at bottom-left."""
        r, c = rc
        return (c, (self.instance.grid_rows - 1) - r)

    def _setup_axes(self, ax):
        inst = self.instance
        # Grid lines — x=col, y=physical row (bottom-left origin)
        ax.set_xlim(-0.5, inst.grid_cols - 0.5)
        ax.set_ylim(-0.5, inst.grid_rows - 0.5)  # y goes up with row (physical)
        ax.set_aspect("equal")
        ax.set_xticks(range(inst.grid_cols))
        ax.set_yticks(range(inst.grid_rows))
        ax.grid(True, linestyle="--", alpha=0.4)

        # Draw tape lines (as thicker light-gray lines)
        for r in range(inst.grid_rows):
            ax.axhline(r, color="#CCCCCC", linewidth=2, zorder=0)
        for c in range(inst.grid_cols):
            ax.axvline(c, color="#CCCCCC", linewidth=2, zorder=0)

        # Depot marker — convert to physical (x, y)
        depot_x, depot_y = inst.node_to_xy(inst.depot_node)
        ax.scatter(
            [depot_x], [depot_y],
            s=400, marker="s", c="#1D3557",
            edgecolors="black", linewidths=2, zorder=3, label="Depot"
        )
        ax.annotate(
            f"DEPOT\n({depot_x},{depot_y})", (depot_x, depot_y),
            textcoords="offset points", xytext=(0, -24),
            ha="center", fontsize=8, fontweight="bold", color="#1D3557"
        )

        # Customer nodes (labeled with their node index and (x,y))
        for c in inst.demand:
            x, y = inst.node_to_xy(c)
            ax.scatter(
                [x], [y], s=200, marker="o",
                facecolors="white", edgecolors="#333333", linewidths=1.5, zorder=2
            )
            ax.annotate(
                str(c), (x, y),
                ha="center", va="center", fontsize=7, zorder=4
            )

        # Ghost paths: outline each AGV's planned route in (x, y)
        if self.show_ghost_paths:
            for v, route in enumerate(self.schedule.routes):
                color = AGV_COLORS[v % len(AGV_COLORS)]
                pts_xy = [(depot_x, depot_y)]
                for stop in route:
                    pts_xy.append(inst.node_to_xy(stop.node))
                pts_xy.append((depot_x, depot_y))
                xs = [p[0] for p in pts_xy]
                ys = [p[1] for p in pts_xy]
                ax.plot(xs, ys, color=color, alpha=0.15, linewidth=3, zorder=1)

        # Axis labels
        ax.set_xlabel("x (column)")
        ax.set_ylabel("y (row, bottom = 0)")

    def animate(self, save_path: Optional[str] = None, show: bool = True):
        fig = plt.figure(figsize=self.figsize)
        # Main grid axes + side panel for stats
        gs = fig.add_gridspec(1, 1)
        ax = fig.add_subplot(gs[0, 0])
        self._setup_axes(ax)

        inst = self.instance

        # AGV markers
        agv_scatters = []
        agv_labels = []
        for v in range(inst.num_agvs):
            color = AGV_COLORS[v % len(AGV_COLORS)]
            sc = ax.scatter(
                [], [], s=350, marker="o",
                facecolors=color, edgecolors="black", linewidths=2, zorder=5,
            )
            agv_scatters.append(sc)
            lbl = ax.annotate(
                "", (0, 0),
                textcoords="offset points", xytext=(0, 12),
                ha="center", fontsize=8, fontweight="bold",
                color=color, zorder=6,
            )
            agv_labels.append(lbl)

        # Processing node overlays
        proc_patches: dict[int, plt.Circle] = {}
        for node in inst.demand:
            x, y = inst.node_to_xy(node)
            patch = mpatches.Circle(
                (x, y), 0.25, fill=False, linewidth=0, zorder=3
            )
            ax.add_patch(patch)
            proc_patches[node] = patch

        # Title with time and makespan
        title = ax.set_title("", fontsize=12)

        # Legend
        legend_handles = [
            mpatches.Patch(color=AGV_COLORS[v % len(AGV_COLORS)], label=f"AGV {v}")
            for v in range(inst.num_agvs)
        ]
        legend_handles.append(mpatches.Patch(color="#FFD166", label="Processing"))
        legend_handles.append(mpatches.Patch(color="#06D6A0", label="Ready for pickup"))
        legend_handles.append(mpatches.Patch(color="#888888", label="Completed"))
        ax.legend(handles=legend_handles, loc="upper right", fontsize=8, framealpha=0.9)

        # Duration: play through [0, makespan] at playback_speed
        duration = self.makespan + 1.0
        total_frames = int(duration * self.fps / self.playback_speed) + 1

        def frame(idx):
            t = idx * self.playback_speed / self.fps

            # Update AGV positions — trajs are in (row, col), convert to (x, y)
            for v, traj in enumerate(self.trajs):
                r, c = traj.position_at(t)
                x, y = self._rc_to_plot((r, c))
                agv_scatters[v].set_offsets([[x, y]])
                load = traj.load_at(t)
                agv_labels[v].xy = (x, y)
                agv_labels[v].set_text(f"A{v}\n[{load}]")

            # Update processing status per node
            for node, patch in proc_patches.items():
                window = self.proc_windows.get(node)
                drop_t = self.dropped.get(node, math.inf)
                pick_t = self.picked.get(node, math.inf)
                if t < drop_t:
                    patch.set_linewidth(0)
                elif window and t < window[1]:
                    # Processing
                    patch.set_edgecolor("#FFD166")
                    patch.set_linewidth(3)
                elif t < pick_t:
                    # Ready for pickup
                    patch.set_edgecolor("#06D6A0")
                    patch.set_linewidth(3)
                else:
                    patch.set_edgecolor("#888888")
                    patch.set_linewidth(2)

            title.set_text(
                f"t = {t:6.1f}s / makespan = {self.makespan:.1f}s  "
                f"(cross-agent: {self.schedule.cross_agent_pct(inst):.1f}%)"
            )
            return [*agv_scatters, *agv_labels, title, *proc_patches.values()]

        anim = FuncAnimation(
            fig, frame, frames=total_frames,
            interval=1000 / self.fps, blit=False, repeat=False,
        )

        if save_path:
            if save_path.endswith(".gif"):
                anim.save(save_path, writer="pillow", fps=self.fps)
            else:
                anim.save(save_path, fps=self.fps)
        if show:
            plt.show()
        return anim

    def static_overview(self, save_path: Optional[str] = None, show: bool = True):
        """Render a single static figure showing all planned routes."""
        fig, ax = plt.subplots(figsize=self.figsize)
        self._setup_axes(ax)
        inst = self.instance

        # Draw each AGV's route with arrows in (x, y) space
        for v, route in enumerate(self.schedule.routes):
            color = AGV_COLORS[v % len(AGV_COLORS)]
            depot_xy = inst.node_to_xy(inst.depot_node)
            cur_xy = depot_xy
            for i, stop in enumerate(route):
                nxy = inst.node_to_xy(stop.node)
                ax.annotate(
                    "", xy=nxy, xytext=cur_xy,
                    arrowprops=dict(arrowstyle="->", color=color, lw=2, alpha=0.7),
                    zorder=2,
                )
                label = f"{v}.{i+1}{stop.op.value}"
                ax.annotate(
                    label, nxy, textcoords="offset points",
                    xytext=(8, 8 + v * 10), fontsize=7, color=color, fontweight="bold",
                )
                cur_xy = nxy
            # Return to depot
            ax.annotate(
                "", xy=depot_xy, xytext=cur_xy,
                arrowprops=dict(arrowstyle="->", color=color, lw=2, alpha=0.4, linestyle="--"),
                zorder=2,
            )

        legend_handles = [
            mpatches.Patch(color=AGV_COLORS[v % len(AGV_COLORS)], label=f"AGV {v}")
            for v in range(inst.num_agvs)
        ]
        ax.legend(handles=legend_handles, loc="upper right", fontsize=9)
        ax.set_title(
            f"VRP-RPD Plan: {inst.num_agvs} AGVs, capacity {inst.capacity}, "
            f"{inst.num_customers} customers\n"
            f"Planned makespan: {self.makespan:.1f}s   "
            f"Cross-agent: {self.schedule.cross_agent_pct(inst):.1f}%"
        )

        if save_path:
            fig.savefig(save_path, dpi=120, bbox_inches="tight")
        if show:
            plt.show()
        return fig
