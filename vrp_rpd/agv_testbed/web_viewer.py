"""
Flask/MJPEG Web Viewer for AGV Warehouse Simulation
----------------------------------------------------
Runs the VRP-RPD + MAPF pipeline, then streams the animation to a browser
via MJPEG so it works on headless servers with no display.

Usage:
  python3 web_viewer.py --dataset bays29 --variant base --port 5000
  # then open http://<server-ip>:5000 in your local browser

Browser controls:
  Pause / Resume   — toggle playback
  Restart          — reset to t=0
  Speed +/-        — adjust playback rate
"""

from __future__ import annotations
import io
import json
import os
import sys
import time
import argparse
import threading
from pathlib import Path

# Force headless SDL before pygame is imported anywhere
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
from flask import Flask, Response, request, jsonify

sys.path.insert(0, str(Path(__file__).parent))

from vrp_rpd.agv_testbed.grid_env import load_dataset_grid
from vrp_rpd.agv_testbed.pipeline import run_pipeline
from vrp_rpd.agv_testbed.visualize import WarehouseViz


DATASET_CONFIG = {
    "gr17":     (3, 5),
    "gr21":     (3, 5),
    "gr24":     (4, 6),
    "gr48":     (4, 6),
    "bays29":   (4, 6),
    "berlin52": (4, 6),
    "eil51":    (4, 6),
}

# ── State shared between Flask routes and the render thread ──────────────────

class SimState:
    def __init__(self):
        self.paused = False
        self.sim_t  = 0.0
        self.sps    = 4.0        # simulation steps per second
        self.max_t  = 0.0
        self.lock   = threading.Lock()

    def toggle_pause(self):
        with self.lock:
            self.paused = not self.paused

    def restart(self):
        with self.lock:
            self.sim_t = 0.0
            self.paused = False

    def adjust_speed(self, factor: float):
        with self.lock:
            self.sps = max(0.25, min(self.sps * factor, 30.0))

    def advance(self, dt: float):
        with self.lock:
            if not self.paused:
                self.sim_t = min(self.sim_t + dt * self.sps, self.max_t)
            return self.sim_t


state = SimState()
viz: WarehouseViz | None = None
app = Flask(__name__)


# ── MJPEG stream ─────────────────────────────────────────────────────────────

def _frame_jpeg() -> bytes:
    """Render the current frame and return it as JPEG bytes."""
    t = state.sim_t
    surf = viz.render_frame(t)
    pygame.display.flip()   # required even in dummy mode
    raw = pygame.surfarray.array3d(surf)   # (W, H, 3)
    from PIL import Image
    img = Image.fromarray(raw.transpose(1, 0, 2))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def _generate_stream(target_fps: int = 20):
    """Generator that yields MJPEG frames, advancing sim time between frames."""
    frame_dt = 1.0 / target_fps
    prev = time.time()
    while True:
        now = time.time()
        dt  = now - prev
        prev = now
        state.advance(dt)
        jpg = _frame_jpeg()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )
        elapsed = time.time() - now
        sleep_t = max(0.0, frame_dt - elapsed)
        time.sleep(sleep_t)


@app.route("/stream")
def stream():
    return Response(
        _generate_stream(target_fps=20),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ── Control endpoints ─────────────────────────────────────────────────────────

@app.route("/pause", methods=["POST"])
def pause():
    state.toggle_pause()
    return jsonify(paused=state.paused)


@app.route("/restart", methods=["POST"])
def restart():
    state.restart()
    return jsonify(sim_t=0.0)


@app.route("/speed", methods=["POST"])
def speed():
    factor = float(request.json.get("factor", 1.0))
    state.adjust_speed(factor)
    return jsonify(sps=round(state.sps, 2))


@app.route("/status")
def status():
    with state.lock:
        return jsonify(
            paused=state.paused,
            sim_t=round(state.sim_t, 1),
            max_t=state.max_t,
            sps=round(state.sps, 2),
        )


# ── HTML UI ───────────────────────────────────────────────────────────────────

HTML = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>AGV Warehouse Viewer</title>
<style>
  body {{ background:#1a1a2e; color:#eee; font-family:monospace;
          margin:0; display:flex; flex-direction:column; align-items:center; }}
  h2   {{ margin:14px 0 6px; font-size:1.1rem; letter-spacing:2px; color:#8ec8f8; }}
  #stream {{ border:2px solid #444; max-width:100%; }}
  #controls {{ display:flex; gap:10px; margin:10px; flex-wrap:wrap; justify-content:center; }}
  button {{ background:#2a2a4a; color:#dde; border:1px solid #556; border-radius:5px;
            padding:7px 18px; font-size:0.95rem; cursor:pointer; }}
  button:hover {{ background:#3a3a6a; }}
  #status {{ font-size:0.8rem; color:#aaa; margin-bottom:10px; }}
  #info   {{ font-size:0.75rem; color:#777; margin-top:4px; }}
</style>
</head>
<body>
<h2>AGV WAREHOUSE &mdash; VRP-RPD + MAPF</h2>
<p id="info">Dataset: {dataset} / {variant} &nbsp;|&nbsp; {num_agents} agents &nbsp;|&nbsp; {num_ws} workstations</p>
<img id="stream" src="/stream">
<div id="controls">
  <button onclick="doPost('/pause').then(r=>r.json()).then(d=>updateStatus())">&#9646;&#9646; Pause / &#9654; Resume</button>
  <button onclick="doPost('/restart')">&#8635; Restart</button>
  <button onclick="doPost('/speed',{{factor:1.5}})">&#9654;&#9654; Speed +</button>
  <button onclick="doPost('/speed',{{factor:0.667}})">&#9654; Speed &minus;</button>
</div>
<div id="status">Loading...</div>
<script>
  function doPost(url, body) {{
    return fetch(url, {{
      method: 'POST',
      headers: {{'Content-Type': 'application/json'}},
      body: JSON.stringify(body || {{}})
    }});
  }}
  function updateStatus() {{
    fetch('/status').then(r=>r.json()).then(d=>{{
      const pct = d.max_t > 0 ? (d.sim_t/d.max_t*100).toFixed(0) : 0;
      document.getElementById('status').textContent =
        (d.paused ? '⏸ PAUSED' : '▶ RUNNING') +
        '  |  t=' + d.sim_t + '/' + d.max_t +
        '  (' + pct + '%)  |  speed=' + d.sps + ' steps/s';
    }});
  }}
  setInterval(updateStatus, 500);
  updateStatus();
</script>
</body>
</html>
"""


@app.route("/")
def index():
    with state.lock:
        num_ws = len(viz.grid.workstations) if viz else "?"
    return HTML.format(
        dataset=app.config["DATASET"],
        variant=app.config["VARIANT"],
        num_agents=app.config["NUM_AGENTS"],
        num_ws=num_ws,
    )


# ── Save / load pipeline results ─────────────────────────────────────────────

def save_result(path: str, grid, result, num_agents: int, resources: int,
                dataset: str, variant: str, seed: int):
    paths = result.mapf_result.paths
    vrp   = result.vrp_result
    data = {
        "meta": {
            "dataset": dataset, "variant": variant, "seed": seed,
            "num_agents": num_agents, "resources_per_agent": resources,
        },
        "cell_nodes": grid.cell_nodes,
        "processing_times": grid.processing_times,
        "makespan": vrp.makespan,
        "agents": {
            str(aid): {
                "completion_time": plan.completion_time,
                "visit_sequence":  plan.visit_sequence,
                "events": [[node, etype, rid] for node, etype, rid in plan.events],
            }
            for aid, plan in vrp.agents.items()
        },
        "customer_timing": {
            str(node): {
                "grid_node":       ct.grid_node,
                "dropoff_time":    ct.dropoff_time,
                "pickup_time":     ct.pickup_time,
                "processing_time": ct.processing_time,
            }
            for node, ct in vrp.customer_timing.items()
        },
        "mapf_paths": {
            str(aid): tp.path
            for aid, tp in paths.items()
        },
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved pipeline result → {path}")


def load_result(path: str):
    """
    Load a saved pipeline JSON and reconstruct the objects WarehouseViz needs.
    Returns (grid_stub, vrp_stub, paths_dict, num_agents, resources, meta).
    """
    from types import SimpleNamespace
    from vrp_rpd.agv_testbed.grid_env import WarehouseGrid, build_distance_matrix

    with open(path) as f:
        data = json.load(f)

    meta      = data["meta"]
    resources = meta["resources_per_agent"]
    num_agents = meta["num_agents"]

    # Rebuild a real WarehouseGrid so visualizer has all virtual ID info
    cell_nodes = data["cell_nodes"]
    proc_times = data["processing_times"]
    grid_stub  = WarehouseGrid(
        cell_nodes=cell_nodes,
        processing_times=proc_times,
        dist=build_distance_matrix(),
    )

    # Reconstruct customer_timing
    customer_timing = {}
    for node_str, ct in data["customer_timing"].items():
        customer_timing[int(node_str)] = SimpleNamespace(**ct)

    # Reconstruct agents
    agents = {}
    for aid_str, ap in data["agents"].items():
        agents[int(aid_str)] = SimpleNamespace(
            agent_id=int(aid_str),
            completion_time=ap["completion_time"],
            visit_sequence=ap["visit_sequence"],
            events=[(e[0], e[1], e[2]) for e in ap["events"]],
        )

    def priority_order():
        return sorted(agents.keys(), key=lambda a: -agents[a].completion_time)

    vrp_stub = SimpleNamespace(
        makespan=data["makespan"],
        agents=agents,
        customer_timing=customer_timing,
        grid=grid_stub,
        priority_order=priority_order,
    )

    # Reconstruct MAPF paths
    from vrp_rpd.agv_testbed.mapf_solver import TimedPath
    paths = {
        int(aid_str): TimedPath(agent_id=int(aid_str), path=[(step[0], step[1]) for step in path_list])
        for aid_str, path_list in data["mapf_paths"].items()
    }

    print(f"Loaded pipeline result ← {path}")
    print(f"  Dataset: {meta['dataset']}/{meta['variant']}  "
          f"agents={num_agents}  resources/agent={resources}")
    return grid_stub, vrp_stub, paths, num_agents, resources, meta


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="AGV Warehouse Web Viewer (MJPEG stream)")
    parser.add_argument("--dataset", default="bays29",
                        choices=list(DATASET_CONFIG.keys()),
                        help="Dataset to run (default: bays29)")
    parser.add_argument("--variant", default="base",
                        choices=["base", "2x", "5x", "1R10", "1R20"],
                        help="Processing-time variant (default: base)")
    parser.add_argument("--seed", type=int, default=42,
                        help="RNG seed (default: 42)")
    parser.add_argument("--port", type=int, default=5000,
                        help="HTTP port (default: 5000)")
    parser.add_argument("--speed", type=float, default=4.0,
                        help="Initial playback speed in sim-steps/sec (default: 4.0)")
    parser.add_argument("--save", default=None, metavar="FILE",
                        help="After solving, save pipeline result to FILE (e.g. gr17_base.json)")
    parser.add_argument("--load", default=None, metavar="FILE",
                        help="Skip solving — load a previously saved result from FILE")
    args = parser.parse_args()

    if args.load:
        # Fast path: skip the solver entirely
        grid_obj, vrp_obj, paths_obj, num_agents, resources, meta = load_result(args.load)
        dataset = meta["dataset"]
        variant = meta["variant"]
    else:
        num_agents, resources = DATASET_CONFIG[args.dataset]
        dataset = args.dataset
        variant = args.variant

        print(f"Dataset : {dataset} / {variant}  (m={num_agents}, k={resources})")
        print("Running VRP-RPD + MAPF pipeline...")

        grid_obj = load_dataset_grid(dataset, variant=variant, seed=args.seed)
        result = run_pipeline(
            grid_obj,
            num_agents=num_agents,
            resources_per_agent=resources,
            max_iterations=5,
            seed=args.seed,
            dataset_dir=dataset,
            variant=variant,
        )
        result.summary()

        vrp_obj   = result.vrp_result
        paths_obj = result.mapf_result.paths

        if args.save:
            save_result(args.save, grid_obj, result, num_agents, resources,
                        dataset, variant, args.seed)

    global viz
    viz = WarehouseViz(
        grid=grid_obj,
        vrp_result=vrp_obj,
        paths=paths_obj,
        steps_per_second=args.speed,
        resources_per_agent=resources,
    )
    state.max_t = float(viz.max_t)
    state.sps   = args.speed

    app.config["DATASET"]    = dataset
    app.config["VARIANT"]    = variant
    app.config["NUM_AGENTS"] = num_agents

    print(f"\nOpen in your browser:  http://<server-ip>:{args.port}")
    print("Controls available in the browser: Pause, Restart, Speed +/-")
    print("Press Ctrl+C to stop.\n")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
