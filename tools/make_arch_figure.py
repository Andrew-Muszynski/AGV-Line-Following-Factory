#!/usr/bin/env python3
"""Regenerate figures/fig_arch.png -- the runtime stack diagram.

Committed so the figure stops being a PNG with no origin. Every fact on it is
sourced in a comment next to the string that states it, so the next person to
change the stack can see what needs updating.

Rebuilt 2026-09-23. Corrections over the 16 September version:
  * the per-robot topic box listed 2 of 5 topics -- _pose, _color and
    _wheel_cmd were missing (firmware lines 1727-1731)
  * "no colour sensing" was wrong twice: British spelling, and the sensor
    does run and publish at 6.7 Hz. Nothing subscribes to it, which is a
    different and more interesting statement
  * the advisor model was gemma4:e2b-it-qat; the 22 Sept factorial selected
    e4b
  * the firmware box named the v1 sketch; v2 is what is flashed
  * "wheel encoders ONLY" omitted the IMU, which is what the ROTATE_REL
    completion gate reads and what diagnosed the 22 Sept link stall
"""
from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                    # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch     # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "figures", "fig_arch.png")

INK = "#1b2733"
MUTE = "#5b6b7a"

COLS = [
    dict(x=1.0, w=23.0, bg="#eef4f8", edge="#17616e", fill="#e2eef4",
         head="WINDOWS LAPTOP", sub="192.168.0.162 (lab) - native"),
    dict(x=25.0, w=23.0, bg="#e9f2fb", edge="#2b8fc7", fill="#dcecfa",
         head="WSL2 on the same laptop", sub="Ubuntu 24.04 - no USB access"),
    dict(x=49.0, w=23.5, bg="#edf5ee", edge="#2e7d4f", fill="#e2efe5",
         head="LINUX LAPTOP", sub="192.168.0.212 - ROS 2 Jazzy, tmux"),
    dict(x=73.5, w=25.5, bg="#f8f4e9", edge="#a8821c", fill="#f7efd9",
         head="ROBOTS", sub="6x Arduino Alvik, ESP32-S3"),
]

# (column, y_top, height, title, [detail lines])
BOXES = [
    (0, 72.0, 12.0, "camera_bridge_windows.py",
     ["owns the NexiGo camera", "native MSMF capture", "--port 8765"]),
    (0, 57.5, 10.0, "solver .html page",
     ["planning UI + Live Monitor", "served by app.py"]),
    (0, 45.0, 10.0, "app.py   Flask :8000",
     ["metrics API + run history", "agv_metrics.sqlite3"]),
    (0, 32.5, 12.0, "ollama :11434",
     # 22 Sept factorial: e4b, 84.2%, p95 0.40 s, overlay off, JPEG 75.
     ["gemma4:e4b-it-qat", "VLM advisor (evaluated,", "not in the loop)"]),

    (1, 72.0, 12.0, "apriltag_localize.py",
     ["pupil_apriltags, CPU", "~35 Hz, 4 corner refs", "-> robot world poses"]),
    (1, 57.5, 10.0, "MJPEG stream :8081",
     ["/full.jpg", "frame source for the VLM"]),

    (2, 72.0, 12.0, "micro-ROS agent",
     ["UDP4 :8888", "the ONLY thing the", "robots talk to"]),
    (2, 57.5, 10.0, "rosbridge :9090",
     ["websocket <-> ROS 2", "browser + WSL2 clients"]),
    (2, 45.0, 10.0, "fleetSupervisor.py",
     ["job registry, launch timer,", "deadlock detect + yield"]),
    (2, 32.5, 10.0, "camera_grid_navigate.py",
     ["one navigator per robot", "camera-assisted driving"]),

    (3, 72.0, 12.0, "AGV_Factory_camera_correction_v2.ino",
     ["IMU + wheel encoders onboard", "rotations gated on IMU yaw,",
      "resynced to camera yaw at start"]),
    (3, 57.5, 32.0, "", []),        # the topic table draws its own content
    (3, 23.5, 14.0, "AprilTag on the robot",
     ["read by the OVERHEAD camera",
      "No onboard localization or line following",
      "Color published, nothing subscribes",
      "Position and heading come from (2) and (3)"]),
]

# topic | direction | rate | who consumes it.  Firmware lines 1727-1731.
TOPICS = [
    ("{robot}_status", "out", "on change", "supervisor, navigator, browser"),
    ("{robot}_pose", "out", "300 ms / 50 ms", "navigator: YAW only, never x/y"),
    ("{robot}_color", "out", "150 ms", "nothing subscribes"),
    ("{robot}_cmd", "in", "on demand", "BEST_EFFORT, BUSY-acked"),
    ("{robot}_wheel_cmd", "in", "streamed", "no ack, watchdog brakes"),
]

# n, column, text
HOPS = [
    (1, 0, "camera frames stay on Windows"),
    (5, 0, "HTTP POST  metrics events -> SQLite"),
    (2, 1, "TCP :8765  decoded frames -> WSL2"),
    (6, 1, "HTTP  frame grab -> VLM prompt"),
    (3, 2, "rosbridge WS  {robot}_vision_pose @ ~35 Hz"),
    (7, 2, "micro-ROS over WiFi/UDP :8888"),
    (4, 3, "WS  dispatch mission, read /fleet_status"),
    (8, 3, "drive_leg / rotate_rel commands"),
]

PLANNING = [
    ("grid_env.py", "graph + bfs_path"),
    ("vrp_solver.py", "run_brkga / heuristics"),
    ("mapf_solver.py", "prioritized A*, conflicts"),
    ("pipeline.py", "VRP->MAPF->reseed"),
]


def box(ax, x, y, w, h, fc, ec, lw=1.6, r=0.6, ls="solid", z=2):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
        facecolor=fc, edgecolor=ec, linewidth=lw, linestyle=ls, zorder=z))


def badge(ax, x, y, n, color):
    ax.add_patch(plt.Circle((x, y), 1.05, facecolor=color, edgecolor="white",
                            linewidth=1.2, zorder=6))
    ax.text(x, y - 0.03, str(n), ha="center", va="center", color="white",
            fontsize=8.5, fontweight="bold", zorder=7)


def main() -> None:
    fig, ax = plt.subplots(figsize=(17.65, 9.88), dpi=120)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    fig.patch.set_facecolor("white")

    # 27pt is ~3.8 axis units tall, so a title anchored at 96 ran into a
    # subtitle at 92. Raised and the gap widened.
    ax.text(1.0, 98.5, "AGV testbed runtime stack", fontsize=27,
            fontweight="bold", color=INK, va="top")
    ax.text(1.0, 92.6,
            "Every arrow between columns is a real network hop - nothing "
            "here shares a process.",
            fontsize=12.5, color=MUTE, va="top")

    for c in COLS:
        box(ax, c["x"], 7.8, c["w"], 81.2, c["bg"], "none", lw=0, r=0.8, z=1)
        cx = c["x"] + c["w"] / 2
        ax.text(cx, 87.2, c["head"], fontsize=13.5, fontweight="bold",
                color=c["edge"], ha="center", va="top")
        ax.text(cx, 84.3, c["sub"], fontsize=9.5, color=MUTE,
                ha="center", va="top")

    ax.text(1.6, 81.4, "CONNECTIONS", fontsize=10, fontweight="bold",
            color=INK, va="top")
    for n, ci, text in HOPS:
        c = COLS[ci]
        row = 79.0 if n in (1, 2, 3, 4) else 75.9
        badge(ax, c["x"] + 1.6, row, n, c["edge"])
        ax.text(c["x"] + 3.4, row - 0.05, text, fontsize=8.6, color=INK,
                va="center")

    for ci, ytop, h, title, lines in BOXES:
        c = COLS[ci]
        x, w = c["x"] + 0.8, c["w"] - 1.6
        box(ax, x, ytop - h, w, h, c["fill"], c["edge"])
        if not title:
            continue
        fs = 11.5 if len(title) < 32 else 9.8
        ax.text(x + w / 2, ytop - 2.9, title, fontsize=fs,
                fontweight="bold", color=INK, ha="center", va="center")
        for i, ln in enumerate(lines):
            ax.text(x + w / 2, ytop - 6.0 - i * 2.4, ln, fontsize=8.8,
                    color=MUTE, ha="center", va="center")

    # ---- the per-robot topic table -----------------------------------------
    c = COLS[3]
    x, w = c["x"] + 0.8, c["w"] - 1.6
    ax.text(x + w / 2, 54.6, "micro-ROS topics, per robot", fontsize=11.5,
            fontweight="bold", color=INK, ha="center", va="center")
    ax.text(x + w / 2, 52.0, "all five exist on every robot, every run",
            fontsize=8.4, color=MUTE, ha="center", va="center", style="italic")
    y = 48.6
    for name, direction, rate, note in TOPICS:
        dead = note == "nothing subscribes"
        arrow = "→" if direction == "out" else "←"
        col = "#b3471f" if dead else INK
        ax.text(x + 1.0, y, arrow, fontsize=10, color=col, va="center",
                fontweight="bold")
        ax.text(x + 2.6, y, name, fontsize=9.2, color=col, va="center",
                family="DejaVu Sans Mono")
        ax.text(x + w - 1.0, y, rate, fontsize=8.4, color=MUTE, va="center",
                ha="right")
        ax.text(x + 2.6, y - 1.85, note, fontsize=8.0,
                color="#b3471f" if dead else MUTE, va="center",
                style="italic" if dead else "normal")
        y -= 5.0

    # ---- hops ---------------------------------------------------------------
    def arrow(x1, y1, x2, y2, color, rad=0.0):
        ax.add_patch(FancyArrowPatch(
            (x1, y1), (x2, y2), connectionstyle=f"arc3,rad={rad}",
            arrowstyle="-|>", mutation_scale=13, linewidth=1.7,
            color=color, zorder=5))

    arrow(23.4, 66.0, 25.6, 66.0, COLS[0]["edge"])            # frames -> WSL2
    badge(ax, 24.5, 69.5, 2, COLS[1]["edge"])
    arrow(47.4, 68.0, 49.6, 58.5, COLS[1]["edge"], rad=-0.22)  # poses -> rosbridge
    badge(ax, 49.0, 65.5, 3, COLS[1]["edge"])
    arrow(23.6, 46.0, 49.4, 48.5, COLS[2]["edge"], rad=0.16)   # metrics POST
    badge(ax, 36.5, 45.6, 5, COLS[2]["edge"])
    arrow(26.4, 56.5, 24.6, 34.0, COLS[1]["edge"], rad=0.22)   # frame -> VLM
    badge(ax, 23.2, 44.0, 6, COLS[1]["edge"])
    arrow(71.8, 66.0, 74.0, 66.0, COLS[3]["edge"])             # agent -> robots
    badge(ax, 72.9, 69.5, 7, COLS[3]["edge"])
    arrow(66.5, 34.0, 74.0, 44.0, COLS[3]["edge"], rad=-0.28)  # commands
    badge(ax, 71.8, 40.5, 8, COLS[3]["edge"])

    # ---- planning strip ------------------------------------------------------
    box(ax, 1.0, 0.6, 71.5, 6.4, "#f5f2f9", "none", lw=0, r=0.6, z=1)
    ax.text(2.0, 5.9, "PLANNING - offline, before a run   "
                      "external/vrp_rpd/agv_testbed/   "
                      "(NOT wired into the supervisor)",
            fontsize=9.6, fontweight="bold", color="#5b4a86", va="center")
    pw = 16.6
    for i, (nm, det) in enumerate(PLANNING):
        px = 2.0 + i * (pw + 1.2)
        box(ax, px, 1.2, pw, 3.3, "white", "#8a76b8", lw=1.3, r=0.5)
        ax.text(px + pw / 2, 3.5, nm, fontsize=9.6, fontweight="bold",
                color=INK, ha="center", va="center")
        ax.text(px + pw / 2, 2.05, det, fontsize=8.0, color=MUTE,
                ha="center", va="center")

    box(ax, 74.0, 0.6, 25.0, 6.4, "#fdeced", "#c0392b", lw=1.8, r=0.6,
        ls=(0, (5, 3)))
    ax.text(86.5, 5.4, "MISSING: the JOINT pipeline", fontsize=11,
            fontweight="bold", color="#a93226", ha="center", va="center")
    ax.text(86.5, 2.9, "routing cost inside the VRP objective,\n"
                       "not applied after it. The treatment arm.",
            fontsize=8.6, color="#8c3a30", ha="center", va="center")

    fig.savefig(OUT, dpi=120, facecolor="white", bbox_inches="tight",
                pad_inches=0.18)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
