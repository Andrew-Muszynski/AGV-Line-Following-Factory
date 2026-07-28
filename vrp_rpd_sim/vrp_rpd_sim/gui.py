"""
VRP-RPD Simulation GUI
Launch: python main.py
"""
from __future__ import annotations

import math
import queue
import random
import threading
import time
from pathlib import Path
from tkinter import filedialog, messagebox
import tkinter as tk
from tkinter import ttk

from instance import Instance, load_positions_xlsx, load_positions_csv
from schedule import Schedule, Op
from simulator import GridSimulator, SimConfig
from solver import ALNSConfig, ALNSResult, build_initial, evaluate_makespan, solve_alns

# ── Display constants ─────────────────────────────────────────────────────────
SPACING     = 74          # pixels between adjacent intersections
MARGIN      = 50          # canvas border (space for axis labels)
FRAME_MS    = 33          # animation period (~30 fps)
GRID_ROWS   = 8
GRID_COLS   = 8
CANVAS_SZ   = MARGIN * 2 + (GRID_COLS - 1) * SPACING   # 618 px

LANE_W      = 12          # tape lane line width (px)
LANE_COL    = "#78909C"   # lane colour
NODE_DOT    = "#E53935"   # red intersection dot fill
NODE_RING   = 8           # intersection dot radius (≈ AGV_R / 2)
AGV_R       = 16          # AGV circle radius
STATE_R     = NODE_RING + 7  # coloured state-ring radius around node dot

# AGV palette (up to 8 robots)
AGV_COLORS = [
    "#1565C0", "#C62828", "#2E7D32", "#6A1B9A",
    "#E65100", "#00695C", "#AD1457", "#4E342E",
]

# Node state colours during simulation
NODE_COL = {
    "pending":    "#90CAF9",
    "processing": "#FFB300",
    "ready":      "#81C784",
    "done":       "#B0BEC5",
}

DEPOT_FILL    = "#FFC107"
DEPOT_OUTLINE = "#FF8F00"
GRID_BG       = "#ECEFF1"
SEL_RING_COL  = "#1565C0"

SPEED_OPTS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]


# ─────────────────────────────────────────────────────────────────────────────
class SimGUI:
    # ─────────────────────────────── init ────────────────────────────────────
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("VRP-RPD Simulation Control")
        self.root.minsize(1200, 720)

        # runtime state
        self.instance:      Instance | None = None
        self.schedule:      Schedule | None = None
        self.events:        list            = []
        self.traj:          list            = []
        self.node_evs:      dict            = {}
        self.makespan:      float           = 0.0

        # animation
        self.playing:      bool             = False
        self.sim_time:     float            = 0.0
        self.speed_idx:    int              = 3
        self._anim_id                       = None
        self._wall_ref:    float | None     = None

        # destination selection (manual mode)
        self.manual_dest: set[int]          = set()

        # distance-matrix: file path + parsed positions (before instance is built)
        self.dist_path:     Path | None     = None
        self.raw_positions: list | None     = None

        # solver thread → main-thread queue
        self._q: queue.Queue                = queue.Queue()

        self._build_layout()
        self._redraw_static()
        self.root.after(150, self._poll_queue)

    # ─────────────────────────────── layout ──────────────────────────────────
    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.columnconfigure(2, weight=0)
        self.root.rowconfigure(0, weight=1)
        self._build_left()
        self._build_center()
        self._build_right()

    # ── Left panel (configuration) ───────────────────────────────────────────
    def _build_left(self) -> None:
        lf = ttk.Frame(self.root, padding=6)
        lf.grid(row=0, column=0, sticky="nsew")
        lf.rowconfigure(1, weight=1)

        ttk.Label(lf, text="Configuration",
                  font=("", 11, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 4))

        nb = ttk.Notebook(lf)
        nb.grid(row=1, column=0, sticky="nsew")
        self._tab_agvs(nb)
        self._tab_dest(nb)
        self._tab_solver(nb)
        self._tab_dist(nb)

        bf = ttk.Frame(lf)
        bf.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        bf.columnconfigure(0, weight=1)
        bf.columnconfigure(1, weight=1)
        self.btn_solve = ttk.Button(bf, text="Solve & Simulate",
                                     command=self.action_solve)
        self.btn_solve.grid(row=0, column=0, padx=2, sticky="ew")
        ttk.Button(bf, text="Reset All",
                   command=self.action_reset).grid(row=0, column=1, padx=2, sticky="ew")

        self.progress = ttk.Progressbar(lf, mode="indeterminate")
        self.progress.grid(row=3, column=0, sticky="ew", pady=(4, 0))

    def _tab_agvs(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="AGVs & Grid")
        f.columnconfigure(1, weight=1)

        def row(r, label, var, lo, hi, inc):
            ttk.Label(f, text=label).grid(row=r, column=0, sticky="w", pady=2)
            ttk.Spinbox(f, from_=lo, to=hi, increment=inc,
                        textvariable=var, width=8
                        ).grid(row=r, column=1, sticky="ew", padx=(6, 0))

        self.v_agvs   = tk.IntVar(value=3)
        self.v_cap    = tk.IntVar(value=4)
        self.v_ctm    = tk.DoubleVar(value=2.0)
        self.v_turn   = tk.DoubleVar(value=0.5)
        self.v_pmin   = tk.DoubleVar(value=5.0)
        self.v_pmax   = tk.DoubleVar(value=30.0)
        self.v_pscale = tk.DoubleVar(value=1.0)
        self.v_stag   = tk.DoubleVar(value=3.0)
        self.v_dwell  = tk.DoubleVar(value=0.0)
        self.v_seed   = tk.StringVar(value="")

        row(0, "Number of AGVs:",        self.v_agvs,   1,  8,   1)
        row(1, "Capacity per AGV:",      self.v_cap,    1, 20,   1)
        row(2, "Cell time (s):",         self.v_ctm,  0.1, 30, 0.1)
        row(3, "Turn penalty (s):",      self.v_turn,   0, 10, 0.1)
        row(4, "Processing min (s):",    self.v_pmin,   0, 300,  1)
        row(5, "Processing max (s):",    self.v_pmax,   0, 300,  1)
        row(6, "Processing scale ×:",    self.v_pscale, 0.1, 10, 0.1)
        row(7, "AGV stagger (s):",       self.v_stag,   0, 60, 0.5)
        row(8, "Dwell time / stop (s):", self.v_dwell,  0, 60, 0.5)

        ttk.Label(f, text="Seed (blank = random):").grid(
            row=9, column=0, sticky="w", pady=2)
        ttk.Entry(f, textvariable=self.v_seed, width=9).grid(
            row=9, column=1, sticky="ew", padx=(6, 0))

    def _tab_dest(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Destinations")
        f.columnconfigure(0, weight=1)
        f.rowconfigure(5, weight=1)

        self.v_mode = tk.StringVar(value="random")
        ttk.Radiobutton(f, text="Random N destinations",
                        variable=self.v_mode, value="random",
                        command=self._on_mode).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(f, text="Click grid to select",
                        variable=self.v_mode, value="manual",
                        command=self._on_mode).grid(row=1, column=0, sticky="w")

        rf = ttk.Frame(f)
        rf.grid(row=2, column=0, sticky="ew", pady=4)
        ttk.Label(rf, text="Count:").pack(side="left")
        self.v_dest_n = tk.IntVar(value=12)
        self.sb_dest_n = ttk.Spinbox(rf, from_=1, to=63,
                                      textvariable=self.v_dest_n, width=5)
        self.sb_dest_n.pack(side="left", padx=4)
        ttk.Button(rf, text="Randomize", command=self._randomize_dest).pack(side="left")

        ttk.Separator(f, orient="horizontal").grid(row=3, column=0, sticky="ew", pady=4)
        ttk.Label(f, text="Selected nodes:").grid(row=4, column=0, sticky="w")

        lbf = ttk.Frame(f)
        lbf.grid(row=5, column=0, sticky="nsew")
        lbf.rowconfigure(0, weight=1)
        lbf.columnconfigure(0, weight=1)
        self.lb_dest = tk.Listbox(lbf, height=7, selectmode="extended",
                                   exportselection=False, font=("Consolas", 8))
        sb = ttk.Scrollbar(lbf, command=self.lb_dest.yview)
        self.lb_dest.config(yscrollcommand=sb.set)
        self.lb_dest.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")

        bf = ttk.Frame(f)
        bf.grid(row=6, column=0, sticky="ew", pady=(4, 0))
        ttk.Button(bf, text="Remove selected",
                   command=self._remove_sel).pack(side="left", padx=2)
        ttk.Button(bf, text="Clear all",
                   command=self._clear_dest).pack(side="left")

    def _tab_solver(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Solver")
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="Algorithm:").grid(row=0, column=0, sticky="w", pady=3)
        self.v_algo = tk.StringVar(value="ALNS (Full)")
        ttk.Combobox(f, textvariable=self.v_algo, state="readonly", width=14,
                     values=["ALNS (Full)", "ALNS (Quick)", "Greedy Only"]
                     ).grid(row=0, column=1, sticky="ew", padx=(6, 0))

        ttk.Label(f, text="Max iterations:").grid(row=1, column=0, sticky="w", pady=3)
        self.v_iters = tk.IntVar(value=1500)
        ttk.Spinbox(f, from_=50, to=50000, increment=50,
                    textvariable=self.v_iters, width=8
                    ).grid(row=1, column=1, sticky="ew", padx=(6, 0))

        ttk.Label(f, text="Time limit (s):").grid(row=2, column=0, sticky="w", pady=3)
        self.v_tlim = tk.DoubleVar(value=30.0)
        ttk.Spinbox(f, from_=1.0, to=600.0, increment=5.0,
                    textvariable=self.v_tlim, width=8
                    ).grid(row=2, column=1, sticky="ew", padx=(6, 0))

        self.v_contention = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Cell contention (collision avoidance)",
                        variable=self.v_contention).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=6)

    def _tab_dist(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=8)
        nb.add(f, text="Distance")
        f.columnconfigure(0, weight=1)

        ttk.Label(f, text="Physical position matrix\n(.xlsx or .csv):",
                  justify="left").grid(row=0, column=0, sticky="w")
        self.lbl_dist = ttk.Label(f, text="Not loaded", foreground="grey",
                                   wraplength=230, justify="left")
        self.lbl_dist.grid(row=1, column=0, sticky="w", pady=4)

        bf = ttk.Frame(f)
        bf.grid(row=2, column=0, sticky="w")
        ttk.Button(bf, text="Load Matrix…",
                   command=self.on_load_matrix).pack(side="left", padx=(0, 4))
        ttk.Button(bf, text="Clear",
                   command=self._clear_dist).pack(side="left")

        ttk.Separator(f, orient="horizontal").grid(row=3, column=0, sticky="ew", pady=8)
        ttk.Button(f, text="Node Reference Table…",
                   command=self.show_node_reference).grid(
            row=4, column=0, sticky="w", pady=(0, 6))

        ttk.Label(f,
                  text="Expected format:\n"
                       "• 8×8 grid of cells\n"
                       "• Each cell: 'y_coord, x_coord'\n"
                       "• Row 0 = depot row (y = 0)\n"
                       "• Column 0 = depot column (x = 0)\n"
                       "• Headers auto-detected & skipped",
                  foreground="grey", justify="left").grid(row=5, column=0, sticky="w")

    # ── Center panel (canvas + playback controls) ─────────────────────────────
    def _build_center(self) -> None:
        cf = ttk.Frame(self.root, padding=4)
        cf.grid(row=0, column=1, sticky="nsew")
        cf.rowconfigure(0, weight=1)
        cf.columnconfigure(0, weight=1)

        cvf = ttk.LabelFrame(cf,
                              text="Grid Visualization  "
                                   "(click intersection in manual mode to toggle destination)")
        cvf.grid(row=0, column=0, sticky="n")
        self.canvas = tk.Canvas(cvf, width=CANVAS_SZ, height=CANVAS_SZ,
                                 bg=GRID_BG, cursor="crosshair", highlightthickness=0)
        self.canvas.pack(padx=4, pady=4)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Motion>", self.on_canvas_hover)

        # Hover info bar
        self.lbl_hover = ttk.Label(cf, text="", foreground="#455A64",
                                    font=("Consolas", 8))
        self.lbl_hover.grid(row=1, column=0, sticky="w", pady=(2, 0))

        # Legend
        leg = ttk.Frame(cf)
        leg.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        for txt, col in [("Pending",    NODE_COL["pending"]),
                         ("Processing", NODE_COL["processing"]),
                         ("Ready",      NODE_COL["ready"]),
                         ("Done",       NODE_COL["done"]),
                         ("Depot",      DEPOT_FILL)]:
            dot = tk.Canvas(leg, width=14, height=14, highlightthickness=0)
            dot.pack(side="left", padx=(6, 2))
            dot.create_oval(1, 1, 13, 13, fill=col, outline="")
            ttk.Label(leg, text=txt, font=("", 8)).pack(side="left", padx=(0, 8))

        # Playback controls
        ctrl = ttk.LabelFrame(cf, text="Playback", padding=4)
        ctrl.grid(row=3, column=0, sticky="ew", pady=(6, 0))
        ctrl.columnconfigure(4, weight=1)

        self.btn_play = ttk.Button(ctrl, text="▶  Play", width=10,
                                    command=self.toggle_play)
        self.btn_play.grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(ctrl, text="⟳  Restart",
                   command=self.action_restart).grid(row=0, column=1, padx=4)

        ttk.Label(ctrl, text="Speed:").grid(row=0, column=2, padx=(12, 2))
        self.v_speed = tk.IntVar(value=self.speed_idx)
        ttk.Scale(ctrl, from_=0, to=len(SPEED_OPTS) - 1, orient="horizontal",
                  variable=self.v_speed, command=self._on_speed,
                  length=140).grid(row=0, column=3, padx=2)
        self.lbl_speed = ttk.Label(ctrl, text=f"{SPEED_OPTS[self.speed_idx]:.2g}×",
                                    width=5)
        self.lbl_speed.grid(row=0, column=4, padx=2)

        ttk.Label(ctrl, text="Sim time:").grid(row=0, column=5, padx=(16, 2))
        self.lbl_time = ttk.Label(ctrl, text="0.00 s", width=10, font=("Consolas", 9))
        self.lbl_time.grid(row=0, column=6, padx=2)

    # ── Right panel (metrics, schedule, AGV status, log) ────────────────────
    def _build_right(self) -> None:
        rf = ttk.Frame(self.root, padding=6)
        rf.grid(row=0, column=2, sticky="nsew")
        rf.rowconfigure(1, weight=2)
        rf.rowconfigure(3, weight=1)
        rf.columnconfigure(0, weight=1)

        mf = ttk.LabelFrame(rf, text="Metrics", padding=6)
        mf.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.lbl_makespan   = ttk.Label(mf, text="Makespan:   —")
        self.lbl_crossagent = ttk.Label(mf, text="Cross-AGV:  —")
        self.lbl_makespan.pack(anchor="w")
        self.lbl_crossagent.pack(anchor="w")

        sf = ttk.LabelFrame(rf, text="Schedule", padding=4)
        sf.grid(row=1, column=0, sticky="nsew", pady=(0, 4))
        sf.rowconfigure(0, weight=1)
        sf.columnconfigure(0, weight=1)
        self.txt_sched = tk.Text(sf, width=36, height=10, state="disabled",
                                  font=("Consolas", 8), wrap="none")
        sbs = ttk.Scrollbar(sf, command=self.txt_sched.yview)
        self.txt_sched.configure(yscrollcommand=sbs.set)
        self.txt_sched.grid(row=0, column=0, sticky="nsew")
        sbs.grid(row=0, column=1, sticky="ns")

        af = ttk.LabelFrame(rf, text="AGV Status", padding=4)
        af.grid(row=2, column=0, sticky="ew", pady=(0, 4))
        self.lbl_agv = ttk.Label(af, text="No simulation loaded.",
                                  justify="left", font=("Consolas", 8))
        self.lbl_agv.pack(anchor="w")

        lf = ttk.LabelFrame(rf, text="Log", padding=4)
        lf.grid(row=3, column=0, sticky="nsew")
        lf.rowconfigure(0, weight=1)
        lf.columnconfigure(0, weight=1)
        self.txt_log = tk.Text(lf, width=36, height=7, state="disabled",
                                font=("Consolas", 8), wrap="word")
        sbl = ttk.Scrollbar(lf, command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=sbl.set)
        self.txt_log.grid(row=0, column=0, sticky="nsew")
        sbl.grid(row=0, column=1, sticky="ns")

    # ─────────────────────────────── coordinate helpers ──────────────────────
    def _rc_to_canvas(self, rc: tuple[int, int]) -> tuple[float, float]:
        """Array (row, col) → canvas pixel at that intersection.
        Row 0 is at top; depot (row 7, col 0) appears at bottom-left."""
        r, c = rc
        return float(MARGIN + c * SPACING), float(MARGIN + r * SPACING)

    def _canvas_to_rc(self, cx: float, cy: float) -> tuple[int, int] | None:
        """Snap canvas pixel to nearest intersection, or None if too far."""
        c = round((cx - MARGIN) / SPACING)
        r = round((cy - MARGIN) / SPACING)
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            ix = MARGIN + c * SPACING
            iy = MARGIN + r * SPACING
            if math.hypot(cx - ix, cy - iy) <= SPACING * 0.45:
                return (r, c)
        return None

    def _phys_xy(self, r: int, c: int) -> tuple[float, float] | None:
        """Return physical (x, y) in cm for grid position (r, c), if known."""
        inst = self.instance
        if inst and inst.node_xy_positions:
            p = inst.node_xy_positions[r * GRID_COLS + c]
            return float(p[0]), float(p[1])
        if self.raw_positions:
            p = self.raw_positions[r * GRID_COLS + c]
            return float(p[0]), float(p[1])
        return None

    # ─────────────────────────────── grid drawing ────────────────────────────
    def _redraw_static(self) -> None:
        """Draw fixed layer: tape lanes, intersection dots, depot, axis labels."""
        self.canvas.delete("all")
        inst = self.instance
        dr = inst.depot_row if inst else 7
        dc = inst.depot_col if inst else 0
        depot_n = dr * GRID_COLS + dc

        # Tape lane lines — horizontal
        x_start = MARGIN
        x_end   = MARGIN + (GRID_COLS - 1) * SPACING
        for r in range(GRID_ROWS):
            y = MARGIN + r * SPACING
            self.canvas.create_line(x_start, y, x_end, y,
                                     fill=LANE_COL, width=LANE_W, capstyle="round")

        # Tape lane lines — vertical
        y_start = MARGIN
        y_end   = MARGIN + (GRID_ROWS - 1) * SPACING
        for c in range(GRID_COLS):
            x = MARGIN + c * SPACING
            self.canvas.create_line(x, y_start, x, y_end,
                                     fill=LANE_COL, width=LANE_W, capstyle="round")

        # Column axis labels (top): physical x or column index
        for c in range(GRID_COLS):
            x = MARGIN + c * SPACING
            xy = self._phys_xy(0, c)
            label = f"x={xy[0]:.0f}" if xy else f"c{c}"
            self.canvas.create_text(x, MARGIN // 2,
                                     text=label, font=("", 7), fill="#546E7A")

        # Row axis labels (left): physical y or row index
        for r in range(GRID_ROWS):
            y = MARGIN + r * SPACING
            xy = self._phys_xy(r, 0)
            label = f"y={xy[1]:.0f}" if xy else f"r{r}"
            self.canvas.create_text(MARGIN // 2, y,
                                     text=label, font=("", 7), fill="#546E7A")

        # Intersection red dots (all non-depot nodes)
        demand = set(inst.demand) if inst else set()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if r == dr and c == dc:
                    continue
                ix, iy = self._rc_to_canvas((r, c))
                self.canvas.create_oval(ix - NODE_RING, iy - NODE_RING,
                                         ix + NODE_RING, iy + NODE_RING,
                                         fill=NODE_DOT, outline="#B71C1C", width=1)
                node = r * GRID_COLS + c
                # Node index label above the dot
                self.canvas.create_text(ix, iy - NODE_RING - 4,
                                         text=str(node), font=("", 6), fill="#546E7A")

        # Selection rings for demand nodes (pre-solve) or manual picks
        for node in (demand if inst else self.manual_dest):
            if node == depot_n:
                continue
            r_, c_ = divmod(node, GRID_COLS)
            cx_, cy_ = self._rc_to_canvas((r_, c_))
            self.canvas.create_oval(cx_ - STATE_R, cy_ - STATE_R,
                                     cx_ + STATE_R, cy_ + STATE_R,
                                     fill="", outline=SEL_RING_COL, width=2.5)

        # Depot marker (over its intersection)
        dx, dy = self._rc_to_canvas((dr, dc))
        s = NODE_RING + 5
        self.canvas.create_rectangle(dx - s, dy - s, dx + s, dy + s,
                                      fill=DEPOT_FILL, outline=DEPOT_OUTLINE, width=2)
        self.canvas.create_text(dx, dy, text="D", font=("", 9, "bold"), fill="#333")

    def _draw_manhattan_line(self, fr: tuple, to: tuple, color: str,
                              tags: str, dash: tuple = (4, 4),
                              width: float = 2.0) -> None:
        """Draw an L-shaped route segment along grid lanes (row-first)."""
        r0, c0 = fr
        r1, c1 = to
        x0, y0 = self._rc_to_canvas((r0, c0))
        xm, ym = self._rc_to_canvas((r1, c0))   # intermediate: change row first
        x1, y1 = self._rc_to_canvas((r1, c1))
        if r0 != r1:
            self.canvas.create_line(x0, y0, xm, ym,
                                     fill=color, dash=dash, width=width, tags=tags)
        if c0 != c1:
            self.canvas.create_line(xm, ym, x1, y1,
                                     fill=color, dash=dash, width=width, tags=tags)

    def _draw_frame(self, t: float) -> None:
        """Draw animated overlay: route ghosts, node states, AGV circles."""
        self.canvas.delete("anim")
        if not self.instance or not self.schedule:
            return

        inst  = self.instance
        sched = self.schedule
        depot = (inst.depot_row, inst.depot_col)

        # Route ghost lines (Manhattan paths)
        for v, route in enumerate(sched.routes):
            col  = AGV_COLORS[v % len(AGV_COLORS)]
            prev = depot
            for stop in route:
                nrc = inst.node_to_rc(stop.node)
                self._draw_manhattan_line(prev, nrc, col, "anim", dash=(4, 4), width=2.0)
                prev = nrc
            self._draw_manhattan_line(prev, depot, col, "anim", dash=(4, 4), width=2.0)

        # Demand-node state rings + red dot on top
        states = self._states_at(t)
        for node in inst.demand:
            if node == inst.depot_node:
                continue
            rc = inst.node_to_rc(node)
            cx_, cy_ = self._rc_to_canvas(rc)
            state = states.get(node, "pending")
            fill  = NODE_COL.get(state, NODE_COL["pending"])
            self.canvas.create_oval(cx_ - STATE_R, cy_ - STATE_R,
                                     cx_ + STATE_R, cy_ + STATE_R,
                                     fill=fill, outline="#455A64", width=1.5, tags="anim")
            # Redraw red dot on top of the state ring
            self.canvas.create_oval(cx_ - NODE_RING, cy_ - NODE_RING,
                                     cx_ + NODE_RING, cy_ + NODE_RING,
                                     fill=NODE_DOT, outline="#B71C1C", width=1, tags="anim")
            self.canvas.create_text(cx_, cy_, text=str(node),
                                     font=("", 7, "bold"), fill="white", tags="anim")

        # AGV circles
        for v in range(inst.num_agvs):
            px, py = self._agv_pos(v, t)
            col = AGV_COLORS[v % len(AGV_COLORS)]
            self.canvas.create_oval(px - AGV_R, py - AGV_R,
                                     px + AGV_R, py + AGV_R,
                                     fill=col, outline="white", width=2, tags="anim")
            self.canvas.create_text(px, py, text=str(v),
                                     font=("", 8, "bold"), fill="white", tags="anim")

    def _agv_pos(self, v: int, t: float) -> tuple[float, float]:
        """Interpolated canvas position for AGV v at simulation time t."""
        segs    = self.traj[v] if v < len(self.traj) else []
        depot   = (self.instance.depot_row, self.instance.depot_col)
        last_rc = depot
        for t0, t1, fr, to in segs:
            if t < t0:
                break
            if t0 <= t <= t1:
                alpha = (t - t0) / max(t1 - t0, 1e-9)
                fx, fy = self._rc_to_canvas(fr)
                tx, ty = self._rc_to_canvas(to)
                return fx + alpha * (tx - fx), fy + alpha * (ty - fy)
            last_rc = to
        return self._rc_to_canvas(last_rc)

    def _states_at(self, t: float) -> dict[int, str]:
        """Return {node: state_str} for every demand node at simulation time t."""
        inst = self.instance
        if not inst:
            return {}
        states = {n: "pending" for n in inst.demand}
        for node, evs in self.node_evs.items():
            for ev_t, state in evs:
                if ev_t <= t:
                    states[node] = state
                else:
                    break
        return states

    def _build_trajectories(self) -> None:
        """Pre-process events into per-AGV move segments and per-node state lists."""
        n_agvs        = self.instance.num_agvs if self.instance else 0
        self.traj     = [[] for _ in range(n_agvs)]
        self.node_evs = {}
        pending: dict[int, tuple] = {}

        for ev in self.events:
            kind, t = ev[0], ev[1]
            if kind == "move_start":
                agv, fr, to = ev[2], ev[3], ev[4]
                pending[agv] = (t, fr, to)
            elif kind == "move_end":
                agv, at = ev[2], ev[3]
                if agv in pending:
                    t0, fr, to = pending.pop(agv)
                    self.traj[agv].append((t0, t, fr, to))
            elif kind == "processing_start":
                node = ev[2]
                self.node_evs.setdefault(node, []).append((t, "processing"))
            elif kind == "processing_end":
                node = ev[2]
                self.node_evs.setdefault(node, []).append((t, "ready"))
            elif kind == "pickup":
                node = ev[3]
                self.node_evs.setdefault(node, []).append((t, "done"))

        for lst in self.node_evs.values():
            lst.sort(key=lambda x: x[0])

    # ─────────────────────────────── animation ───────────────────────────────
    def _start_anim(self) -> None:
        self.playing   = True
        self._wall_ref = time.monotonic() - self.sim_time / SPEED_OPTS[self.speed_idx]
        self.btn_play.config(text="⏸  Pause")
        self._anim_tick()

    def _anim_tick(self) -> None:
        if not self.playing:
            return
        self.sim_time = (time.monotonic() - self._wall_ref) * SPEED_OPTS[self.speed_idx]
        if self.sim_time >= self.makespan:
            self.sim_time = self.makespan
            self.playing  = False
            self.btn_play.config(text="▶  Play")
            self._log("Simulation complete.")

        self._draw_frame(self.sim_time)
        self.lbl_time.config(text=f"{self.sim_time:.2f} s")
        self._update_agv_label()

        if self.playing:
            self._anim_id = self.root.after(FRAME_MS, self._anim_tick)

    def toggle_play(self) -> None:
        if not self.events:
            messagebox.showinfo("Nothing to play",
                                "Solve a problem first, then press Play.")
            return
        if self.playing:
            self.playing = False
            if self._anim_id:
                self.root.after_cancel(self._anim_id)
            self.btn_play.config(text="▶  Play")
        else:
            if self.sim_time >= self.makespan:
                self.sim_time = 0.0
            self._start_anim()

    def action_restart(self) -> None:
        was          = self.playing
        self.playing = False
        if self._anim_id:
            self.root.after_cancel(self._anim_id)
        self.sim_time = 0.0
        self._draw_frame(0.0)
        self.lbl_time.config(text="0.00 s")
        self._update_agv_label()
        if was and self.events:
            self._start_anim()

    def _on_speed(self, *_) -> None:
        idx = round(float(self.v_speed.get()))
        idx = max(0, min(idx, len(SPEED_OPTS) - 1))
        self.speed_idx = idx
        self.lbl_speed.config(text=f"{SPEED_OPTS[idx]:.2g}×")
        if self.playing:
            self._wall_ref = time.monotonic() - self.sim_time / SPEED_OPTS[idx]

    # ─────────────────────────────── solve / simulate ────────────────────────
    def action_solve(self) -> None:
        try:
            inst = self._build_instance()
        except Exception as exc:
            messagebox.showerror("Configuration error", str(exc))
            return

        self.instance = inst
        self._redraw_static()

        algo  = self.v_algo.get()
        iters = self.v_iters.get()
        tlim  = self.v_tlim.get()

        self.btn_solve.config(state="disabled")
        self.progress.start(12)
        self._log(f"Solving [{algo}] — {inst.num_customers} customers, "
                  f"{inst.num_agvs} AGVs…")

        def _worker():
            try:
                rng = random.Random(inst.seed)
                if algo == "Greedy Only":
                    sched = build_initial(inst, rng)
                    mk    = evaluate_makespan(sched, inst)
                    self._q.put(("done", sched, mk, 0.0, 1))
                else:
                    quick = (algo == "ALNS (Quick)")
                    cfg = ALNSConfig(
                        max_iter=min(iters, 300) if quick else iters,
                        time_limit_sec=min(tlim, 15.0) if quick else tlim,
                        verbose=False,
                    )
                    res: ALNSResult = solve_alns(inst, cfg=cfg)
                    self._q.put(("done", res.schedule, res.makespan,
                                  res.elapsed_sec, res.iterations))
            except Exception as exc:
                self._q.put(("error", str(exc)))

        threading.Thread(target=_worker, daemon=True).start()

    def _build_instance(self) -> Instance:
        agvs   = self.v_agvs.get()
        cap    = self.v_cap.get()
        seed_s = self.v_seed.get().strip()
        seed   = int(seed_s) if seed_s else None

        mode = self.v_mode.get()
        if mode == "manual" and not self.manual_dest:
            raise ValueError(
                "Manual mode: click grid intersections to select destinations first.")

        inst = Instance.random_instance(
            grid_rows=8, grid_cols=8,
            num_agvs=agvs, capacity=cap,
            cell_traversal_time=self.v_ctm.get(),
            turn_penalty=self.v_turn.get(),
            load_time=self.v_stag.get(),
            processing_time_range=(self.v_pmin.get(), self.v_pmax.get()),
            processing_time_multiplier=self.v_pscale.get(),
            depot_row=7, depot_col=0,
            seed=seed,
        )

        if mode == "manual":
            depot_n = inst.depot_node
            demand  = sorted(n for n in self.manual_dest if n != depot_n)
            if len(demand) > agvs * cap:
                raise ValueError(
                    f"{len(demand)} destinations exceed fleet capacity "
                    f"{agvs} × {cap} = {agvs * cap}.")
            inst.demand = demand
            rng  = random.Random(seed)
            proc = [0.0] * inst.num_nodes
            for n in demand:
                proc[n] = (rng.uniform(self.v_pmin.get(), self.v_pmax.get())
                           * self.v_pscale.get())
            inst.processing_times = proc

        if self.dist_path:
            suf = self.dist_path.suffix.lower()
            pos = (load_positions_xlsx(self.dist_path) if suf == ".xlsx"
                   else load_positions_csv(self.dist_path))
            inst.set_node_positions(pos)
            self._log(f"Distance matrix applied: {self.dist_path.name}")

        return inst

    def _run_simulation(self, inst: Instance, sched: Schedule) -> None:
        cfg = SimConfig(
            enable_contention=self.v_contention.get(),
            agv_launch_stagger=inst.load_time,
            dwell_time=self.v_dwell.get(),
        )
        sim    = GridSimulator(inst, sched, cfg=cfg)
        result = sim.run()
        self.events   = result.events
        self.makespan = result.makespan
        self._build_trajectories()
        self.sim_time = 0.0
        self._draw_frame(0.0)
        self.lbl_time.config(text="0.00 s")
        self._update_agv_label()

    def _poll_queue(self) -> None:
        try:
            while True:
                msg = self._q.get_nowait()
                if msg[0] == "done":
                    _, sched, mk, elapsed, n_iters = msg
                    self.progress.stop()
                    self.btn_solve.config(state="normal")
                    self.schedule = sched
                    self._show_schedule(sched, mk)
                    self._log(f"Solved: makespan={mk:.1f}s "
                              f"({elapsed:.1f}s wall, {n_iters} iters)")
                    self._run_simulation(self.instance, sched)
                    self._log("Ready — press ▶ Play to animate.")
                elif msg[0] == "error":
                    self.progress.stop()
                    self.btn_solve.config(state="normal")
                    messagebox.showerror("Solver error", msg[1])
                    self._log(f"ERROR: {msg[1]}")
        except queue.Empty:
            pass
        self.root.after(150, self._poll_queue)

    # ─────────────────────────────── event handlers ──────────────────────────
    def on_canvas_click(self, event: tk.Event) -> None:
        if self.v_mode.get() != "manual":
            return
        rc = self._canvas_to_rc(event.x, event.y)
        if rc is None:
            return
        node    = rc[0] * GRID_COLS + rc[1]
        depot_n = 7 * GRID_COLS + 0
        if node == depot_n:
            return
        if node in self.manual_dest:
            self.manual_dest.discard(node)
        else:
            self.manual_dest.add(node)
        self._refresh_dest_lb()
        self._redraw_static()

    def on_canvas_hover(self, event: tk.Event) -> None:
        rc = self._canvas_to_rc(event.x, event.y)
        if rc is None:
            self.lbl_hover.config(text="")
            return
        r, c = rc
        node  = r * GRID_COLS + c
        parts = [f"Node {node}  (row={r}, col={c})"]
        xy = self._phys_xy(r, c)
        if xy:
            parts.append(f"x={xy[0]:.2f} cm, y={xy[1]:.2f} cm")
        inst = self.instance
        if inst and node in inst.demand:
            proc = (inst.processing_times[node]
                    if node < len(inst.processing_times) else 0.0)
            parts.append(f"proc={proc:.1f}s")
        self.lbl_hover.config(text="    ".join(parts))

    def on_load_matrix(self) -> None:
        path = filedialog.askopenfilename(
            title="Select position matrix",
            filetypes=[("Excel", "*.xlsx"), ("CSV", "*.csv *.tsv *.txt"),
                        ("All files", "*.*")])
        if not path:
            return
        self.dist_path = Path(path)
        try:
            suf = self.dist_path.suffix.lower()
            self.raw_positions = (load_positions_xlsx(self.dist_path)
                                  if suf == ".xlsx"
                                  else load_positions_csv(self.dist_path))
            self.lbl_dist.config(text=self.dist_path.name, foreground="#2E7D32")
            self._log(f"Distance matrix loaded: {self.dist_path.name}")
            self._redraw_static()  # update axis labels immediately
        except Exception as exc:
            self.raw_positions = None
            messagebox.showerror("Load error", str(exc))
            self._log(f"ERROR loading matrix: {exc}")

    def _clear_dist(self) -> None:
        self.dist_path     = None
        self.raw_positions = None
        self.lbl_dist.config(text="Not loaded", foreground="grey")
        self._redraw_static()

    def show_node_reference(self) -> None:
        top = tk.Toplevel(self.root)
        top.title("Node Reference Table")
        top.geometry("640x520")
        top.resizable(True, True)

        cols = ("Node", "Row", "Col", "Phys X (cm)", "Phys Y (cm)", "Proc Time (s)")
        frm  = ttk.Frame(top)
        frm.pack(fill="both", expand=True, padx=6, pady=6)
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)

        tree = ttk.Treeview(frm, columns=cols, show="headings", height=22)
        for col in cols:
            tree.heading(col, text=col)
            w = 60 if col == "Node" else 95
            tree.column(col, width=w, anchor="center", minwidth=50)

        sb = ttk.Scrollbar(frm, command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")

        inst    = self.instance
        depot_n = inst.depot_node if inst else 7 * GRID_COLS + 0

        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                node = r * GRID_COLS + c
                if node == depot_n:
                    continue
                xy    = self._phys_xy(r, c)
                x_str = f"{xy[0]:.2f}" if xy else "—"
                y_str = f"{xy[1]:.2f}" if xy else "—"
                proc_str = "—"
                if inst and node < len(inst.processing_times):
                    p = inst.processing_times[node]
                    if p > 0:
                        proc_str = f"{p:.1f}"
                tree.insert("", "end", values=(node, r, c, x_str, y_str, proc_str))

        ttk.Label(top, text="Depot excluded.  Positions from loaded matrix.",
                  foreground="grey", font=("", 8)).pack(pady=(0, 4))

    def action_reset(self) -> None:
        self.playing = False
        if self._anim_id:
            self.root.after_cancel(self._anim_id)
        self.instance = self.schedule = None
        self.events   = []
        self.traj     = []
        self.node_evs = {}
        self.makespan = self.sim_time = 0.0
        self.btn_play.config(text="▶  Play")
        self.lbl_time.config(text="0.00 s")
        self.lbl_makespan.config(text="Makespan:   —")
        self.lbl_crossagent.config(text="Cross-AGV:  —")
        self.lbl_agv.config(text="No simulation loaded.")
        self._set_text(self.txt_sched, "")
        self._set_text(self.txt_log, "")
        self._redraw_static()

    def _on_mode(self) -> None:
        state = "normal" if self.v_mode.get() == "random" else "disabled"
        self.sb_dest_n.config(state=state)
        self._redraw_static()

    def _randomize_dest(self) -> None:
        n     = self.v_dest_n.get()
        depot = 7 * GRID_COLS + 0
        pool  = [i for i in range(GRID_ROWS * GRID_COLS) if i != depot]
        self.manual_dest = set(random.sample(pool, min(n, len(pool))))
        self.v_mode.set("manual")
        self._on_mode()
        self._refresh_dest_lb()
        self._redraw_static()

    def _refresh_dest_lb(self) -> None:
        self.lb_dest.delete(0, tk.END)
        for n in sorted(self.manual_dest):
            r, c = divmod(n, GRID_COLS)
            xy   = self._phys_xy(r, c)
            if xy:
                self.lb_dest.insert(
                    tk.END,
                    f"Node {n:2d}  (r={r},c={c})  x={xy[0]:.1f} y={xy[1]:.1f}")
            else:
                self.lb_dest.insert(tk.END, f"Node {n:2d}  (row={r}, col={c})")

    def _remove_sel(self) -> None:
        sel   = self.lb_dest.curselection()
        nodes = sorted(self.manual_dest)
        for i in reversed(sel):
            if i < len(nodes):
                self.manual_dest.discard(nodes[i])
        self._refresh_dest_lb()
        self._redraw_static()

    def _clear_dest(self) -> None:
        self.manual_dest.clear()
        self.lb_dest.delete(0, tk.END)
        self._redraw_static()

    # ─────────────────────────────── status helpers ───────────────────────────
    def _show_schedule(self, sched: Schedule, makespan: float) -> None:
        inst  = self.instance
        lines = []
        for v, route in enumerate(sched.routes):
            stops = " → ".join(f"{s.op.value}{s.node}" for s in route)
            lines.append(f"AGV {v}: depot → {stops} → depot")
        self._set_text(self.txt_sched, "\n".join(lines))
        self.lbl_makespan.config(text=f"Makespan:   {makespan:.2f} s")
        if inst:
            pct = sched.cross_agent_pct(inst)
            self.lbl_crossagent.config(text=f"Cross-AGV:  {pct:.1f}%")

    def _update_agv_label(self) -> None:
        if not self.instance or not self.traj:
            return
        t     = self.sim_time
        lines = []
        for v in range(self.instance.num_agvs):
            segs   = self.traj[v]
            action = "idle @ depot"
            last   = (self.instance.depot_row, self.instance.depot_col)
            for t0, t1, fr, to in segs:
                if t < t0:
                    break
                if t0 <= t <= t1:
                    action = f"moving → ({to[0]},{to[1]})"
                    last   = to
                    break
                last   = to
                action = f"at ({last[0]},{last[1]})"
            lines.append(f"AGV {v}: {action}")
        self.lbl_agv.config(text="\n".join(lines))

    def _set_text(self, w: tk.Text, txt: str) -> None:
        w.config(state="normal")
        w.delete("1.0", tk.END)
        w.insert(tk.END, txt)
        w.config(state="disabled")

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.txt_log.config(state="normal")
        self.txt_log.insert(tk.END, f"[{ts}] {msg}\n")
        self.txt_log.see(tk.END)
        self.txt_log.config(state="disabled")


# ─── Entry point ──────────────────────────────────────────────────────────────
def launch() -> None:
    root = tk.Tk()

    style = ttk.Style(root)
    for theme in ("clam", "alt", "default"):
        if theme in style.theme_names():
            style.theme_use(theme)
            break

    SimGUI(root)
    root.mainloop()
