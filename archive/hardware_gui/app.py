#!/usr/bin/env python3
"""
hardware_gui/app.py  —  Flask+SocketIO web GUI for AGV hardware control

Run on the Linux ROS2 VM:
    cd hardware_gui
    pip install -r requirements.txt
    python app.py [--host 0.0.0.0] [--port 5000]

Then open http://<vm-ip>:5000 from the Windows browser.

Architecture:
  - Flask-SocketIO serves the web page and handles events from the browser.
  - A background thread optionally creates an rclpy node to subscribe to
    AGV status topics and publish dispatch commands.
  - If rclpy is not available the GUI runs in "demo mode" with simulated data.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import yaml
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit

# ── App setup ─────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config['SECRET_KEY'] = 'agv-hardware-gui'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ── Shared state (guarded by _state_lock) ─────────────────────────────────

_state_lock = threading.Lock()

# Latest telemetry per AGV key ('agv_1', 'agv_2', 'agv_3')
agv_telemetry: Dict[str, dict] = {}
# Last seen timestamp per AGV
agv_last_seen: Dict[str, float] = {}
# Logging
logging_active = False
log_file = None
log_writer = None
log_path: Optional[Path] = None

# ── Config loading ─────────────────────────────────────────────────────────

CONFIG_PATH = Path(__file__).parent.parent / 'BaseAGV_v4' / 'agv_robots.yaml'

def load_agv_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            return yaml.safe_load(f)
    except Exception:
        return {'agvs': {
            'agv_1': {'name': 'Alvik1'},
            'agv_2': {'name': 'Alvik2'},
            'agv_3': {'name': 'Alvik3'},
        }}

_cfg = load_agv_config()
AGV_HW_NAMES = {
    k: v.get('name', f'Alvik{k.split("_")[1]}')
    for k, v in _cfg.get('agvs', {}).items()
}

# ── ROS2 bridge (optional) ─────────────────────────────────────────────────

_cmd_pub = None  # rclpy publisher for /agv_factory/command
_ros2_available = False


def _ros2_thread() -> None:
    global _cmd_pub, _ros2_available
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("[ros2] rclpy not available — running in demo mode", flush=True)
        _start_demo_mode()
        return

    rclpy.init()
    node = Node('hardware_gui_bridge')

    # Subscribe to each AGV status topic
    for agv_key, hw_name in AGV_HW_NAMES.items():
        node.create_subscription(
            String, f'{hw_name}_status',
            lambda msg, k=agv_key: _on_agv_status(k, msg.data),
            10,
        )

    # Subscribe to factory status
    node.create_subscription(
        String, '/agv_factory/status',
        lambda msg: _on_factory_status(msg.data),
        10,
    )

    _cmd_pub = node.create_publisher(String, '/agv_factory/command', 10)
    _ros2_available = True
    print("[ros2] bridge node started", flush=True)
    socketio.emit('ros2_status', {'connected': True})

    try:
        rclpy.spin(node)
    except Exception as exc:
        print(f"[ros2] spin exited: {exc}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        _ros2_available = False


def _on_agv_status(agv_key: str, json_str: str) -> None:
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError:
        return

    data['agv_key'] = agv_key
    now = time.time()

    with _state_lock:
        agv_telemetry[agv_key] = data
        agv_last_seen[agv_key] = now
        if logging_active and log_writer:
            _write_log_row(agv_key, data)

    socketio.emit('agv_status', data)


def _on_factory_status(status: str) -> None:
    socketio.emit('factory_status', {'status': status})


# ── Demo mode (no ROS2) ───────────────────────────────────────────────────

def _start_demo_mode() -> None:
    """Push synthetic telemetry to the browser so the UI is testable offline."""
    def _demo_loop():
        import math
        t = 0.0
        while True:
            for i, agv_key in enumerate(AGV_HW_NAMES):
                data = {
                    'agv_key': agv_key,
                    'state': 'IDLE',
                    'step': 0, 'total': 0,
                    'x': round(math.sin(t + i) * 20, 2),
                    'y': round(math.cos(t + i) * 20, 2),
                    'yaw': round(math.degrees(t + i) % 360, 1),
                    'L': 120, 'C': 900, 'R': 115,
                    'tof': [30.0, 28.0, 25.0, 28.0, 30.0],
                    'obj': 0,
                    'bat': max(0, 95 - int(t) % 96),
                    'color': 'WHITE',
                    'ms': int(t * 1000),
                    'demo': True,
                }
                with _state_lock:
                    agv_telemetry[agv_key] = data
                    agv_last_seen[agv_key] = time.time()
                socketio.emit('agv_status', data)
            t += 0.2
            time.sleep(0.2)

    threading.Thread(target=_demo_loop, daemon=True).start()


# ── Logging helpers ────────────────────────────────────────────────────────

import csv

CSV_COLUMNS = [
    'wall_time', 'agv', 'state', 'step', 'total',
    'x', 'y', 'yaw', 'L', 'C', 'R',
    'tof_fl', 'tof_fcl', 'tof_fc', 'tof_fcr', 'tof_fr',
    'obj', 'bat', 'color', 'ms',
]


def _write_log_row(agv_key: str, data: dict) -> None:
    tof = data.get('tof', [0] * 5)
    while len(tof) < 5:
        tof.append(0)
    row = {
        'wall_time': datetime.now().isoformat(timespec='milliseconds'),
        'agv': agv_key,
        'state': data.get('state', ''),
        'step': data.get('step', 0),
        'total': data.get('total', 0),
        'x': data.get('x', 0), 'y': data.get('y', 0),
        'yaw': data.get('yaw', 0),
        'L': data.get('L', 0), 'C': data.get('C', 0), 'R': data.get('R', 0),
        'tof_fl': tof[0], 'tof_fcl': tof[1], 'tof_fc': tof[2],
        'tof_fcr': tof[3], 'tof_fr': tof[4],
        'obj': data.get('obj', 0),
        'bat': data.get('bat', 0),
        'color': data.get('color', ''),
        'ms': data.get('ms', 0),
    }
    log_writer.writerow(row)
    log_file.flush()


# ── Flask routes ───────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html',
                            agv_names=AGV_HW_NAMES,
                            ros2_available=_ros2_available)


@app.route('/api/state')
def api_state():
    with _state_lock:
        now = time.time()
        connected = {k: (now - agv_last_seen.get(k, 0)) < 3.0
                     for k in AGV_HW_NAMES}
        return jsonify({
            'telemetry': agv_telemetry,
            'connected': connected,
            'logging': logging_active,
            'log_path': str(log_path) if log_path else None,
            'ros2': _ros2_available,
        })


# ── SocketIO events ────────────────────────────────────────────────────────

@socketio.on('connect')
def on_connect():
    with _state_lock:
        now = time.time()
        connected = {k: (now - agv_last_seen.get(k, 0)) < 3.0
                     for k in AGV_HW_NAMES}
    emit('init', {
        'agv_names': AGV_HW_NAMES,
        'ros2': _ros2_available,
        'connected': connected,
        'logging': logging_active,
    })


@socketio.on('dispatch')
def on_dispatch(payload: dict):
    """
    payload = {
      'routes': [[node_id, ...], ...],   # per-AGV stop node lists
      'processing_time': float,           # seconds
    }
    """
    if not _ros2_available or _cmd_pub is None:
        emit('error', {'msg': 'ROS2 not connected'})
        return

    from std_msgs.msg import String as ROSString

    # Build the factory node JSON
    # node_rcs: for each node 0..63, its (row, col) in the internal grid
    ROWS, COLS = 8, 8
    node_rcs = []
    for node_id in range(ROWS * COLS):
        phys_row = node_id // COLS
        col = node_id % COLS
        internal_row = ROWS - 1 - phys_row
        node_rcs.append([internal_row, col])

    # Depot is node 0 → internal row 7, col 0
    depot_rc = [7, 0]

    routes_payload = []
    for agv_stops in payload.get('routes', []):
        route = []
        for i, node_id in enumerate(agv_stops):
            # Alternate dropoff / pickup pairs
            op = 'D' if i % 2 == 0 else 'L'
            route.append({'node': node_id, 'op': op})
        routes_payload.append(route)

    factory_payload = {
        'routes': routes_payload,
        'depot_rc': depot_rc,
        'node_rcs': node_rcs,
        'dwell_time': float(payload.get('processing_time', 0.0)),
    }

    msg = ROSString()
    msg.data = f"DISPATCH {json.dumps(factory_payload)}"
    _cmd_pub.publish(msg)
    emit('dispatch_sent', {'num_agvs': len(routes_payload)})


@socketio.on('simple_route')
def on_simple_route(payload: dict):
    """
    payload = {
      'agv_key':    'agv_1',
      'nodes':      [5, 12, 27],
      'dwell_time': 2.0,
    }
    """
    agv_key    = payload.get('agv_key', 'agv_1')
    nodes      = payload.get('nodes', [])
    dwell_time = float(payload.get('dwell_time', 0.0))

    if not nodes:
        emit('error', {'msg': 'No destination nodes specified.'})
        return

    if not _ros2_available or _cmd_pub is None:
        # Demo mode: echo back a fake success so the UI responds
        emit('route_sent', {'agv_key': agv_key, 'num_nodes': len(nodes)})
        return

    from std_msgs.msg import String as ROSString
    cmd_payload = {'agv_key': agv_key, 'nodes': nodes, 'dwell_time': dwell_time}
    msg = ROSString()
    msg.data = f"SIMPLE_ROUTE {json.dumps(cmd_payload)}"
    _cmd_pub.publish(msg)
    emit('route_sent', {'agv_key': agv_key, 'num_nodes': len(nodes)})


@socketio.on('stop_all')
def on_stop_all():
    if _ros2_available and _cmd_pub:
        from std_msgs.msg import String as ROSString
        msg = ROSString()
        msg.data = 'STOP_ALL'
        _cmd_pub.publish(msg)
    emit('stopped')


@socketio.on('start_log')
def on_start_log():
    global logging_active, log_file, log_writer, log_path
    with _state_lock:
        if logging_active:
            emit('log_status', {'active': True, 'path': str(log_path)})
            return
        log_dir = Path(__file__).parent / 'logs'
        log_dir.mkdir(exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = log_dir / f'agv_telemetry_{ts}.csv'
        log_file = open(log_path, 'w', newline='', encoding='utf-8')
        log_writer = csv.DictWriter(log_file, fieldnames=CSV_COLUMNS, extrasaction='ignore')
        log_writer.writeheader()
        log_file.flush()
        logging_active = True
    emit('log_status', {'active': True, 'path': str(log_path)})


@socketio.on('stop_log')
def on_stop_log():
    global logging_active, log_file, log_writer
    with _state_lock:
        if not logging_active:
            emit('log_status', {'active': False})
            return
        logging_active = False
        if log_file:
            log_file.close()
            log_file = None
            log_writer = None
    emit('log_status', {'active': False, 'path': str(log_path)})


# ── Entry point ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description='AGV Hardware GUI')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--no-ros2', action='store_true',
                        help='Skip ROS2 bridge, use demo mode')
    args = parser.parse_args()

    if not args.no_ros2:
        threading.Thread(target=_ros2_thread, daemon=True).start()
    else:
        _start_demo_mode()

    print(f"[gui] Starting on http://{args.host}:{args.port}")
    socketio.run(app, host=args.host, port=args.port, debug=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
