"""
Pygame Warehouse Visualization
--------------------------------
Animates cobots moving along their MAPF timed paths on the 10x10 grid.

Two modes:
  Interactive (default): opens a pygame window — requires a local display.
  Export (--export):     renders headlessly and saves a GIF you can copy
                         to your local machine and open in any browser/viewer.

Workstation stages (4 colours):
  Yellow  : waiting — no resource delivered yet
  Red     : active  — resource dropped off, processing underway
  Green   : ready   — processing complete, waiting for pickup
  Dark    : done    — resource picked up, workstation complete

Sidebar shows live resource count per agent at the current sim time.

Interactive controls:
  SPACE   : pause / resume
  R       : restart
  +/-     : speed up / slow down
  ESC/Q   : quit

Usage:
  # Interactive (needs display)
  python3 visualize.py

  # Export GIF — works on headless servers
  python3 visualize.py --export
  python3 visualize.py --export --dataset gr17 --variant 2x --output gr17_2x.gif
  python3 visualize.py --export --fps 10 --steps-per-frame 2
"""

from __future__ import annotations
import os
import sys
import argparse
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple, Optional

# ── Detect display availability before importing pygame ──────────────────────
def _has_display() -> bool:
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return True
    if sys.platform in ("darwin", "win32"):
        return True
    return False

HEADLESS = not _has_display()
if HEADLESS:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
from vrp_rpd.agv_testbed.grid_env import (
    WarehouseGrid, load_dataset_grid, node_rc, ROWS, COLS, DEPOT_NODE, N_NODES, SPUR_LEN
)
from vrp_rpd.agv_testbed.mapf_solver import TimedPath
from vrp_rpd.agv_testbed.vrp_solver import SolverResult


# ── Layout ───────────────────────────────────────────────────────────────────
CELL_PX = 64
MARGIN  = 44
INFO_W  = 260
NODE_R  = 9
WS_R    = 15
COBOT_R = 11

WIN_W = COLS * CELL_PX + 2 * MARGIN + INFO_W
WIN_H = ROWS * CELL_PX + 2 * MARGIN

# ── Colours ──────────────────────────────────────────────────────────────────
BG        = (245, 245, 240)
GRID_LINE = (185, 185, 185)
DEPOT_C   = (20,  20,  20)
NODE_C    = (150, 170, 200)
WS_BORDER = (50,  50,   50)
TEXT_C    = (30,  30,   30)
PANEL_BG  = (225, 225, 220)

# 4-stage workstation colours
WS_WAITING = (230, 200,  40)   # yellow  — not yet visited
WS_ACTIVE  = (210,  50,  50)   # red     — resource dropped, processing
WS_READY   = (50,  190,  80)   # green   — processing done, await pickup
WS_DONE    = (60,   60,  60)   # dark    — picked up, complete

AGENT_COLORS = [
    (52,  120, 220),
    (210,  60,  60),
    (50,  180,  90),
    (160,  80, 200),
    (20,  190, 200),
    (220, 140,  20),
]


def grid_to_px(node: int) -> Tuple[int, int]:
    """Pixel position for a transit grid node (0–99)."""
    r, c = node_rc(node)
    return MARGIN + c * CELL_PX, MARGIN + (ROWS - 1 - r) * CELL_PX


def node_to_px(node: int, grid: WarehouseGrid) -> Tuple[int, int]:
    """
    Pixel position for any node — transit, spur entry, or workstation.
    Spur entry : same pixel as its transit anchor (top edge of cell).
    Workstation: midpoint between cell_node and spur_transit_node = cell center.
    """
    if node < N_NODES:
        return grid_to_px(node)
    if node in grid._spur_entry_set:
        idx = grid.spur_entry_ids.index(node)
        cx, cy = grid_to_px(grid.cell_nodes[idx])
        _, ey  = grid_to_px(grid.spur_transit_nodes[idx])
        # Top-center of the cell: halfway between columns c and c+1, at spur transit row
        return cx + CELL_PX // 2, ey
    if node in grid._workstation_set:
        idx = grid.workstation_ids.index(node)
        cx, cy = grid_to_px(grid.cell_nodes[idx])
        _, ey  = grid_to_px(grid.spur_transit_nodes[idx])
        # True cell center: halfway between columns c and c+1, halfway between rows r and r+1
        return cx + CELL_PX // 2, cy - CELL_PX // 2
    return grid_to_px(DEPOT_NODE)


def lerp_node_px(a: int, b: int, frac: float, grid: WarehouseGrid) -> Tuple[int, int]:
    ax, ay = node_to_px(a, grid)
    bx, by = node_to_px(b, grid)
    return int(ax + (bx - ax) * frac), int(ay + (by - ay) * frac)


# ── Map each VRP event to its MAPF arrival time ──────────────────────────────

def _build_event_mapf_times(
    vrp_result: SolverResult,
    paths: Dict[int, TimedPath],
) -> Tuple[Dict[int, Dict[str, float]], Dict[int, List[Tuple[float, str, int]]]]:
    """
    Returns:
      ws_mapf_times : node -> {drop_t, proc_done_t, pick_t} all in MAPF time units
      agent_events  : aid  -> [(mapf_t, etype, rid), ...] sorted by mapf_t

    Strategy: events in plan.events are in visit order. Walk the agent's MAPF path
    in order, consuming the next visit to each event-node as we encounter it.
    This correctly handles nodes visited twice (first=dropoff, second=pickup).
    """
    ws_mapf: Dict[int, Dict[str, float]] = {}   # node -> drop_t, proc_done_t, pick_t
    agent_events: Dict[int, List[Tuple[float, str, int]]] = {}

    # Global scale factor: MAPF hops per VRP time unit
    mapf_max = max(p.path[-1][1] for p in paths.values())
    vrp_makespan = vrp_result.makespan
    global_scale = mapf_max / vrp_makespan if vrp_makespan > 0 else 1.0

    for aid, plan in vrp_result.agents.items():
        path = paths[aid].path   # list of (node, mapf_t)

        # Walk path in order; for each event consume the next matching path step.
        # path_idx advances forward so visiting a node twice picks up both visits.
        path_idx = 0
        last_mapf_t = 0.0
        timed: List[Tuple[float, str, int]] = []

        for node, etype, rid in plan.events:
            # find next occurrence of this node from current path_idx onward
            found = None
            for i in range(path_idx, len(path)):
                if path[i][0] == node:
                    found = i
                    break
            if found is not None:
                mapf_t = float(path[found][1])
                path_idx = found + 1   # consume; next event on same node gets next visit
            else:
                # Node not found ahead — same-step drop+pickup on the same node.
                # Use the last known time (the drop that just happened).
                mapf_t = last_mapf_t

            last_mapf_t = mapf_t
            timed.append((mapf_t, etype, rid))

            # Populate ws_mapf_times using global VRP→MAPF scale
            ct = vrp_result.customer_timing[node]
            if node not in ws_mapf:
                ws_mapf[node] = {}
            if etype == "D":
                ws_mapf[node]["drop_t"] = mapf_t
                # Clamp proc_done_t to mapf_max so it can't exceed the animation length
                ws_mapf[node]["proc_done_t"] = min(
                    mapf_t + ct.processing_time * global_scale, float(mapf_max)
                )
            else:
                ws_mapf[node]["pick_t"] = mapf_t

        agent_events[aid] = timed

    return ws_mapf, agent_events


# ── Per-agent resource count timeline ─────────────────────────────────────────

def _build_resource_timeline(
    agent_events: Dict[int, List[Tuple[float, str, int]]],
    resources_per_agent: int,
) -> Dict[int, List[Tuple[float, int]]]:
    """For each agent, sorted list of (mapf_time, resource_count) breakpoints."""
    timelines: Dict[int, List[Tuple[float, int]]] = {}
    for aid, events in agent_events.items():
        count = resources_per_agent
        breakpoints: List[Tuple[float, int]] = [(0.0, count)]
        for mapf_t, etype, rid in sorted(events):
            count += 1 if etype == "P" else -1
            breakpoints.append((mapf_t, count))
        timelines[aid] = breakpoints
    return timelines


def _resource_count_at(timeline: List[Tuple[float, int]], t: float) -> int:
    """Step-function lookup: last breakpoint at or before t. Clamped to >= 0."""
    count = timeline[0][1]
    for bp_t, bp_c in timeline:
        if bp_t <= t:
            count = bp_c
        else:
            break
    return max(0, count)


# ── Visualizer ───────────────────────────────────────────────────────────────

class WarehouseViz:
    def __init__(
        self,
        grid: WarehouseGrid,
        vrp_result: SolverResult,
        paths: Dict[int, TimedPath],
        steps_per_second: float = 4.0,
        resources_per_agent: int = 5,
    ):
        self.grid = grid
        self.vrp  = vrp_result
        self.paths = paths
        self.sps  = steps_per_second
        self.max_t = max((p.path[-1][1] if p.path else 0) for p in paths.values())
        self.agent_ids = sorted(paths.keys())

        # Derive MAPF-scale event times from actual path arrivals
        ws_mapf_times, agent_events = _build_event_mapf_times(vrp_result, paths)
        self._ws_timing = ws_mapf_times

        # Per-agent resource count timeline (MAPF time scale)
        self._res_timelines = _build_resource_timeline(agent_events, resources_per_agent)
        self._resources_per_agent = resources_per_agent

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("AGV Warehouse — VRP-RPD + MAPF")
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont("monospace", 12)
        self.font_m = pygame.font.SysFont("monospace", 14, bold=True)
        self.font_l = pygame.font.SysFont("monospace", 16, bold=True)
        self.sim_t  = 0.0
        self.paused = False
        self.done   = False

    def _ws_state(self, gnode: int, t: float) -> str:
        ev = self._ws_timing.get(gnode)
        if ev is None:               return "waiting"
        if t >= ev["pick_t"]:        return "done"
        if t >= ev["proc_done_t"]:   return "ready"
        if t >= ev["drop_t"]:        return "active"
        return "waiting"

    def _draw_grid(self):
        for i in range(ROWS):
            y = MARGIN + i * CELL_PX
            pygame.draw.line(self.screen, GRID_LINE,
                             (MARGIN, y), (MARGIN + (COLS-1)*CELL_PX, y), 1)
        for j in range(COLS):
            x = MARGIN + j * CELL_PX
            pygame.draw.line(self.screen, GRID_LINE,
                             (x, MARGIN), (x, MARGIN + (ROWS-1)*CELL_PX), 1)

    def _draw_nodes(self, t: float):
        state_color = {
            "waiting": WS_WAITING,
            "active":  WS_ACTIVE,
            "ready":   WS_READY,
            "done":    WS_DONE,
        }
        # Draw all transit nodes
        for nid in range(ROWS * COLS):
            px = grid_to_px(nid)
            if nid == DEPOT_NODE:
                r, c = node_rc(nid)
                rect = pygame.Rect(MARGIN + c*CELL_PX - 16,
                                   MARGIN + (ROWS-1-r)*CELL_PX - 16, 32, 32)
                pygame.draw.rect(self.screen, DEPOT_C, rect, border_radius=5)
                lbl = self.font_s.render("D", True, (255, 255, 255))
                self.screen.blit(lbl, (rect.x + 9, rect.y + 7))
            else:
                pygame.draw.circle(self.screen, NODE_C, px, NODE_R)

        # Draw workstations at cell center with spur line
        for idx, ws_id in enumerate(self.grid.workstation_ids):
            entry_px = node_to_px(self.grid.spur_entry_ids[idx], self.grid)
            ws_px    = node_to_px(ws_id, self.grid)
            pygame.draw.line(self.screen, WS_BORDER, entry_px, ws_px, 2)

            # Workstation circle at center
            state = self._ws_state(ws_id, t)
            color = state_color[state]
            text_color = (240, 240, 240) if state == "done" else (20, 20, 20)
            pygame.draw.circle(self.screen, color, ws_px, WS_R)
            pygame.draw.circle(self.screen, WS_BORDER, ws_px, WS_R, 2)
            lbl = self.font_s.render(str(idx + 1), True, text_color)
            self.screen.blit(lbl, (ws_px[0] - 5, ws_px[1] - 6))

    def _cobot_px(self, aid: int, t: float) -> Tuple[int, int]:
        path = self.paths[aid].path
        if not path:
            return grid_to_px(DEPOT_NODE)
        ti = int(t)
        fr = t - ti
        if ti >= len(path) - 1:
            return node_to_px(path[-1][0], self.grid)
        return lerp_node_px(path[ti][0], path[ti + 1][0], fr, self.grid)

    def _draw_cobots(self, t: float):
        for idx, aid in enumerate(self.agent_ids):
            color = AGENT_COLORS[idx % len(AGENT_COLORS)]
            px = self._cobot_px(aid, t)
            pygame.draw.circle(self.screen, (80, 80, 80),  (px[0]+2, px[1]+2), COBOT_R)
            pygame.draw.circle(self.screen, color,          px, COBOT_R)
            pygame.draw.circle(self.screen, (255, 255, 255), px, COBOT_R, 2)
            lbl = self.font_s.render(str(aid), True, (255, 255, 255))
            self.screen.blit(lbl, (px[0]-4, px[1]-6))

    def _draw_panel(self, t: float):
        px0 = WIN_W - INFO_W
        pygame.draw.rect(self.screen, PANEL_BG, (px0, 0, INFO_W, WIN_H))
        pygame.draw.line(self.screen, (140, 140, 140), (px0, 0), (px0, WIN_H), 2)

        y = 16
        def row(text, color=TEXT_C, font=None):
            nonlocal y
            surf = (font or self.font_s).render(text, True, color)
            self.screen.blit(surf, (px0 + 10, y))
            y += surf.get_height() + 3

        row("AGV WAREHOUSE", TEXT_C, self.font_l)
        row("VRP-RPD + MAPF", TEXT_C, self.font_m)
        y += 4
        row(f"VRP makespan : {self.vrp.makespan:.0f}")
        row(f"MAPF max hop : {self.max_t}")
        row(f"Sim step     : {t:.1f}")
        status_c = (200, 60, 60) if self.paused else (40, 160, 40)
        row("PAUSED" if self.paused else "RUNNING", status_c, self.font_m)
        y += 8

        # ── Agents with live resource count ──────────────────────────────────
        row("── Agents ──", TEXT_C, self.font_m)
        priority = self.vrp.priority_order()
        for rank, aid in enumerate(priority):
            plan  = self.vrp.agents[aid]
            color = AGENT_COLORS[self.agent_ids.index(aid) % len(AGENT_COLORS)]
            res_now = _resource_count_at(self._res_timelines[aid], t)
            pygame.draw.circle(self.screen, color, (px0 + 14, y + 7), 6)
            star = " ★" if rank == 0 else ""
            row(f"  A{aid}  res={res_now}/{self._resources_per_agent}"
                f"  @{plan.completion_time:.0f}{star}")

        # ── Workstation stage counts ──────────────────────────────────────────
        y += 8
        row("── Workstations ──", TEXT_C, self.font_m)
        states = [self._ws_state(g, t) for g in self.grid.workstations]
        counts = {s: states.count(s) for s in ("waiting", "active", "ready", "done")}
        for state, color, label in [
            ("waiting", WS_WAITING, "Waiting "),
            ("active",  WS_ACTIVE,  "Active  "),
            ("ready",   WS_READY,   "Ready   "),
            ("done",    WS_DONE,    "Done    "),
        ]:
            pygame.draw.circle(self.screen, color, (px0 + 14, y + 7), 6)
            row(f"  {label} {counts[state]:2d}/{len(self.grid.workstations)}")

        # ── Legend ───────────────────────────────────────────────────────────
        y += 8
        row("── Legend ──", TEXT_C, self.font_m)
        for color, label in [
            (WS_WAITING, "No resource yet"),
            (WS_ACTIVE,  "Processing"),
            (WS_READY,   "Ready for pickup"),
            (WS_DONE,    "Complete"),
        ]:
            pygame.draw.circle(self.screen, color, (px0 + 14, y + 7), 6)
            row(f"  {label}")

        if not HEADLESS:
            y += 8
            row("── Keys ──", TEXT_C, self.font_m)
            for line in ["SPACE  pause/resume", "R      restart",
                         "+/-    speed", "ESC    quit"]:
                row(f"  {line}")

    def _draw_progress(self):
        bar_w = WIN_W - INFO_W - 2 * MARGIN
        bar_y = WIN_H - MARGIN // 2
        pygame.draw.rect(self.screen, (180, 180, 180),
                         (MARGIN, bar_y, bar_w, 8), border_radius=4)
        prog = min(self.sim_t / max(self.max_t, 1), 1.0)
        pygame.draw.rect(self.screen, (70, 140, 220),
                         (MARGIN, bar_y, int(bar_w * prog), 8), border_radius=4)

    def render_frame(self, t: float):
        """Draw one frame at simulation time t. Returns the surface."""
        self.screen.fill(BG)
        self._draw_grid()
        self._draw_nodes(t)
        self._draw_cobots(t)
        self._draw_panel(t)
        self._draw_progress()
        return self.screen

    # ── Interactive loop ─────────────────────────────────────────────────────

    def run_interactive(self):
        while not self.done:
            dt = self.clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.done = True
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        self.done = True
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_r:
                        self.sim_t = 0.0; self.paused = False
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.sps = min(self.sps * 1.5, 60.0)
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.sps = max(self.sps / 1.5, 0.25)
            if not self.paused:
                self.sim_t = min(self.sim_t + dt * self.sps, float(self.max_t))
            self.render_frame(self.sim_t)
            pygame.display.flip()
        pygame.quit()

    # ── GIF export ───────────────────────────────────────────────────────────

    def export_gif(
        self,
        output_path: str = "warehouse_animation.gif",
        fps: int = 8,
        steps_per_frame: float = 1.0,
    ):
        from PIL import Image

        frames = []
        t = 0.0
        total_frames = int(self.max_t / steps_per_frame) + 2
        print(f"Rendering {total_frames} frames → {output_path} ...")

        while t <= self.max_t:
            self.render_frame(t)
            pygame.display.flip()
            raw = pygame.surfarray.array3d(self.screen)
            img = Image.fromarray(raw.transpose(1, 0, 2))
            frames.append(img.convert("P", palette=Image.ADAPTIVE, colors=128))
            t += steps_per_frame
            if len(frames) % 20 == 0:
                print(f"  {len(frames)}/{total_frames} frames rendered...")

        frame_duration = int(1000 / fps)
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            loop=0,
            duration=frame_duration,
            optimize=False,
        )
        pygame.quit()
        print(f"Saved: {output_path}  ({len(frames)} frames @ {fps} fps)")
        print(f"Copy to your machine:  scp <server>:{Path(output_path).resolve()} .")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Warehouse AGV Visualizer")
    parser.add_argument("--export", action="store_true",
                        help="Export GIF instead of interactive window")
    parser.add_argument("--dataset", default="bays29",
                        choices=["gr17","gr21","gr24","gr48","bays29","berlin52","eil51"],
                        help="Dataset to run (default: bays29)")
    parser.add_argument("--variant", default="base",
                        choices=["base","2x","5x","1R10","1R20"],
                        help="Processing-time variant (default: base)")
    parser.add_argument("--seed", type=int, default=42,
                        help="RNG seed for workstation placement")
    parser.add_argument("--output", default=None,
                        help="Output GIF path (default: <dataset>_<variant>.gif)")
    parser.add_argument("--fps", type=int, default=8,
                        help="GIF frames per second (default: 8)")
    parser.add_argument("--steps-per-frame", type=float, default=1.0,
                        help="Sim steps per GIF frame (default: 1.0)")
    parser.add_argument("--speed", type=float, default=3.0,
                        help="Interactive playback speed steps/sec (default: 3)")
    args = parser.parse_args()

    config = {
        "gr17": (3, 5), "gr21": (3, 5),
        "gr24": (4, 6), "gr48": (4, 6), "bays29": (4, 6),
        "berlin52": (4, 6), "eil51": (4, 6),
    }
    num_agents, resources = config[args.dataset]

    from vrp_rpd.agv_testbed.grid_env import load_dataset_grid
    from vrp_rpd.agv_testbed.pipeline import run_pipeline

    grid = load_dataset_grid(args.dataset, variant=args.variant, seed=args.seed)

    print(f"Dataset: {args.dataset} / {args.variant}  "
          f"(m={num_agents}, k={resources}, n={len(grid.workstations)})")
    print("Running VRP-RPD + MAPF pipeline...")

    result = run_pipeline(
        grid,
        num_agents=num_agents,
        resources_per_agent=resources,
        max_iterations=5,
        seed=args.seed,
        dataset_dir=args.dataset,
        variant=args.variant,
    )
    result.summary()

    viz = WarehouseViz(
        grid=result.vrp_result.grid,
        vrp_result=result.vrp_result,
        paths=result.mapf_result.paths,
        steps_per_second=args.speed,
        resources_per_agent=resources,
    )

    if args.export or HEADLESS:
        out = args.output or f"{args.dataset}_{args.variant}.gif"
        viz.export_gif(output_path=out, fps=args.fps,
                       steps_per_frame=args.steps_per_frame)
    else:
        print("Controls: SPACE=pause  R=restart  +/-=speed  ESC=quit")
        viz.run_interactive()


if __name__ == "__main__":
    main()
