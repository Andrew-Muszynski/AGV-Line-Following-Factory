#!/usr/bin/env python3
"""
AGV Factory Node  —  Step-Route Dispatcher

Dispatches VRP-RPD schedules to physical Alvik AGVs as sequences of
intersection moves (F / L / R).

Command protocol (published to <RobotName>_cmd):
  route F,R,F,L,F   Load move buffer on robot
  go                Start executing loaded buffer
  stop              Halt immediately

Status protocol (subscribed from <RobotName>_status):
  JSON: {"state":"IDLE","step":2,"total":6,"x":...,"y":...,...}
  state values: NOT_READY | IDLE | MOVING | ARRIVED

High-level commands (subscribed from /agv_factory/command):
  INITIALIZE_ALL              Send stop to all AGVs
  STOP_ALL                    Same as above
  STOP_AGV1                   Stop a single AGV
  DISPATCH <json_schedule>    Execute a full VRP schedule

The schedule JSON is the output of Schedule.to_dict() plus the instance
geometry needed to compute paths:
  {
    "routes": [[{"node":5,"op":"D"}, ...], ...],
    "depot_rc": [7, 0],
    "node_rcs": [[r, c], ...],   // index = node id
    "dwell_time": 2.0            // seconds to pause at each stop
  }

Path-to-moves conversion
  Heading codes: N S E W  (N = decreasing row = up on canvas)
  Initial heading: E  (AGV faces +col direction at depot)

  Turn table  (current → required → move):
    E→E=F  E→N=L  E→S=R
    N→N=F  N→W=L  N→E=R
    W→W=F  W→S=L  W→N=R
    S→S=F  S→E=L  S→W=R
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


# ── Path / move utilities (no external deps) ─────────────────────────────

_DELTA_TO_HDG: Dict[Tuple[int, int], str] = {
    (-1,  0): 'N',
    (+1,  0): 'S',
    ( 0, +1): 'E',
    ( 0, -1): 'W',
}

_TURN_TABLE: Dict[Tuple[str, str], str] = {
    ('E', 'E'): 'F', ('E', 'N'): 'L', ('E', 'S'): 'R',
    ('N', 'N'): 'F', ('N', 'W'): 'L', ('N', 'E'): 'R',
    ('W', 'W'): 'F', ('W', 'S'): 'L', ('W', 'N'): 'R',
    ('S', 'S'): 'F', ('S', 'E'): 'L', ('S', 'W'): 'R',
}


def manhattan_path(
    start: Tuple[int, int], end: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """Row-first Manhattan path from start to end (exclusive of start)."""
    path: List[Tuple[int, int]] = []
    r, c = start
    er, ec = end
    while r != er:
        r += 1 if er > r else -1
        path.append((r, c))
    while c != ec:
        c += 1 if ec > c else -1
        path.append((r, c))
    return path


def path_to_moves(
    start_rc: Tuple[int, int],
    path: List[Tuple[int, int]],
    heading: str = 'E',
) -> Tuple[List[str], str]:
    """
    Convert a cell path (as returned by manhattan_path) to F/L/R moves.

    Returns (moves, final_heading).
    """
    moves: List[str] = []
    prev = start_rc
    for cell in path:
        dr = cell[0] - prev[0]
        dc = cell[1] - prev[1]
        required = _DELTA_TO_HDG.get((dr, dc))
        if required is None:
            # Diagonal step — should not happen with Manhattan paths
            prev = cell
            continue
        move = _TURN_TABLE.get((heading, required), 'F')
        moves.append(move)
        heading = required
        prev = cell
    return moves, heading


# ── Factory node ─────────────────────────────────────────────────────────

class AGVFactoryNode(Node):

    def __init__(self) -> None:
        super().__init__('agv_factory_node')

        self.get_logger().info("AGV FACTORY NODE — step-route dispatcher — starting")

        self._load_config()
        self._setup_topics()

        # Per-AGV live state parsed from status messages
        self.agv_state:  Dict[str, str] = {k: 'UNKNOWN' for k in self._agv_keys()}
        self.agv_step:   Dict[str, int] = {k: 0         for k in self._agv_keys()}
        self.agv_total:  Dict[str, int] = {k: 0         for k in self._agv_keys()}

        # Threading events used by dispatch threads to wait for state changes
        self._state_events: Dict[str, threading.Event] = {
            k: threading.Event() for k in self._agv_keys()
        }
        self._state_lock = threading.Lock()

        # Active dispatch threads (one per AGV when a schedule is running)
        self._dispatch_threads: List[threading.Thread] = []

        # Resource ready-at times for VRP-RPD pickup gating.
        # D stop sets resource_ready_at[node] = wall_time + processing_time.
        # P stop blocks until wall_time >= resource_ready_at[node].
        self._resource_ready_at: Dict[int, float] = {}
        self._resource_lock = threading.Lock()

        self.get_logger().info("AGV FACTORY NODE READY")

    # ── Configuration ────────────────────────────────────────────────────

    def _load_config(self) -> None:
        try:
            config_path = os.path.join(os.path.dirname(__file__), 'agv_robots.yaml')
            with open(config_path) as f:
                cfg = yaml.safe_load(f)
            self.agv_ids: List[int] = []
            self.agv_hw_name: Dict[str, str] = {}
            for key, val in cfg.get('agvs', {}).items():
                if key.startswith('agv_') and key.split('_')[1].isdigit():
                    n = int(key.split('_')[1])
                    self.agv_ids.append(n)
                    self.agv_hw_name[f'agv_{n}'] = val.get('name', f'Alvik{n}')
            self.agv_ids.sort()
            self.get_logger().info(f"Loaded AGVs: {self.agv_ids}")
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({exc}), using defaults")
            self.agv_ids = [1, 2, 3]
            self.agv_hw_name = {f'agv_{i}': f'Alvik{i}' for i in self.agv_ids}

    def _agv_keys(self) -> List[str]:
        return [f'agv_{i}' for i in self.agv_ids]

    # ── ROS2 topics ──────────────────────────────────────────────────────

    def _setup_topics(self) -> None:
        self.cmd_pubs: Dict[str, any] = {}
        for agv_id in self.agv_ids:
            key  = f'agv_{agv_id}'
            name = self.agv_hw_name[key]
            self.cmd_pubs[key] = self.create_publisher(String, f'{name}_cmd', 10)
            self.create_subscription(
                String, f'{name}_status',
                lambda msg, k=key: self._on_status(k, msg), 10)
            self.get_logger().info(f"  pub={name}_cmd  sub={name}_status")

        self.create_subscription(
            String, '/agv_factory/command', self._on_high_level_cmd, 10)
        self.global_status_pub = self.create_publisher(
            String, '/agv_factory/status', 10)

    # ── Status callback ───────────────────────────────────────────────────

    def _on_status(self, agv_key: str, msg: String) -> None:
        try:
            data  = json.loads(msg.data)
            state = data.get('state', 'UNKNOWN')
            step  = int(data.get('step',  0))
            total = int(data.get('total', 0))

            with self._state_lock:
                old_state = self.agv_state.get(agv_key)
                self.agv_state[agv_key] = state
                self.agv_step[agv_key]  = step
                self.agv_total[agv_key] = total

            if old_state != state:
                self.get_logger().info(f"[{agv_key}] {state}  step={step}/{total}")
                # Wake any thread waiting on this AGV's state
                self._state_events[agv_key].set()

        except (json.JSONDecodeError, KeyError):
            pass

    # ── High-level command callback ───────────────────────────────────────

    def _on_high_level_cmd(self, msg: String) -> None:
        cmd = msg.data.strip()
        self.get_logger().info(f"HIGH-LEVEL CMD: {cmd}")

        upper = cmd.upper()

        if upper in ('INITIALIZE_ALL', 'STOP_ALL'):
            self._stop_all()

        elif upper.startswith('STOP_AGV'):
            try:
                n = int(upper.replace('STOP_AGV', '').replace('_', ''))
                self._send(f'agv_{n}', 'stop')
            except ValueError:
                self.get_logger().error(f"Cannot parse AGV id from: {cmd}")

        elif upper.startswith('DISPATCH '):
            payload = cmd[9:].strip()
            threading.Thread(
                target=self._dispatch_from_json, args=(payload,), daemon=True
            ).start()

        elif upper.startswith('SIMPLE_ROUTE '):
            payload = cmd[13:].strip()
            threading.Thread(
                target=self._simple_route, args=(payload,), daemon=True
            ).start()

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # ── Schedule dispatch ─────────────────────────────────────────────────

    def _dispatch_from_json(self, json_str: str) -> None:
        """
        Parse and execute a full schedule.

        Expected JSON keys:
          routes     list of per-AGV stop lists  [{"node":5,"op":"D"}, ...]
          depot_rc   [row, col] of depot
          node_rcs   list of [row, col] indexed by node id
          dwell_time seconds to pause at each stop (optional, default 0)
        """
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Bad schedule JSON: {exc}")
            return

        depot_rc   = tuple(data['depot_rc'])
        node_rcs   = [tuple(rc) for rc in data['node_rcs']]
        dwell_time = float(data.get('dwell_time', 0.0))
        routes     = data['routes']
        # processing_times: {node_id_str: seconds} — gates VRP-RPD pickups
        processing_times = {
            int(k): float(v)
            for k, v in data.get('processing_times', {}).items()
        }

        # Reset resource timers for the new schedule
        with self._resource_lock:
            self._resource_ready_at.clear()

        # One thread per AGV so all run in parallel
        threads = []
        for v, route in enumerate(routes):
            if not route:
                continue
            agv_key  = f'agv_{v + 1}'
            segments = self._build_segments(route, depot_rc, node_rcs)
            t = threading.Thread(
                target=self._run_agv_route,
                args=(agv_key, segments, dwell_time, processing_times),
                daemon=True,
            )
            threads.append(t)
            self._dispatch_threads.append(t)

        self.get_logger().info(
            f"Dispatching schedule: {len(threads)} AGVs, dwell={dwell_time}s")
        self._publish_status('DISPATCH_START')

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.get_logger().info("All AGVs returned to depot.")
        self._publish_status('DISPATCH_COMPLETE')

    def _simple_route(self, json_str: str) -> None:
        """
        Execute a simple visit route for one AGV.

        JSON keys:
          agv_key    e.g. "agv_1"
          nodes      list of node IDs to visit in order
          dwell_time seconds to pause at each stop (optional, default 0)
        """
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Bad SIMPLE_ROUTE JSON: {exc}")
            return

        agv_key    = str(data.get('agv_key', 'agv_1'))
        nodes      = [int(n) for n in data.get('nodes', [])]
        dwell_time = float(data.get('dwell_time', 0.0))

        if not nodes:
            self.get_logger().warn("SIMPLE_ROUTE: empty node list, nothing to do")
            return

        if agv_key not in self._agv_keys():
            self.get_logger().error(f"SIMPLE_ROUTE: unknown agv_key '{agv_key}'")
            return

        # 8×8 grid, node 0 = depot at bottom-left (row 7, col 0)
        ROWS, COLS = 8, 8
        depot_rc = (ROWS - 1, 0)
        node_rcs = [(ROWS - 1 - n // COLS, n % COLS) for n in range(ROWS * COLS)]

        route    = [{'node': n, 'op': 'VISIT'} for n in nodes]
        segments = self._build_segments(route, depot_rc, node_rcs)

        self.get_logger().info(
            f"[{agv_key}] SIMPLE_ROUTE: {nodes}  dwell={dwell_time}s")
        self._publish_status(f'SIMPLE_ROUTE_START {agv_key}')

        self._run_agv_route(agv_key, segments, dwell_time)

        self._publish_status(f'SIMPLE_ROUTE_COMPLETE {agv_key}')

    def _build_segments(
        self,
        route: list,
        depot_rc: Tuple[int, int],
        node_rcs: List[Tuple[int, int]],
    ) -> List[Tuple[Optional[int], str, List[str]]]:
        """
        Convert a route (list of {node, op} dicts) to a list of
        (node_id, op_label, moves) segments.

        The final segment is always the return to depot (node=None, op='RETURN').
        """
        segments = []
        heading  = 'E'   # all AGVs start facing east at depot
        prev_rc  = depot_rc

        stops = list(route) + [None]   # sentinel for return leg

        for stop in stops:
            if stop is None:
                target_rc  = depot_rc
                node_id    = None
                op_label   = 'RETURN'
            else:
                node_id    = int(stop['node'])
                op_label   = str(stop['op'])
                target_rc  = node_rcs[node_id]

            path  = manhattan_path(prev_rc, target_rc)
            if path:
                moves, heading = path_to_moves(prev_rc, path, heading)
            else:
                moves = []

            segments.append((node_id, op_label, moves))
            prev_rc = target_rc

        return segments

    def _run_agv_route(
        self,
        agv_key: str,
        segments: List[Tuple[Optional[int], str, List[str]]],
        dwell_time: float,
        processing_times: Optional[Dict[int, float]] = None,
    ) -> None:
        """
        Thread body: send each route segment to the AGV and wait for it
        to finish before sending the next one.

        VRP-RPD gating:
          D stop → record resource_ready_at[node] = now + processing_time
          P stop → block until resource_ready_at[node] is reached
        Falls back to fixed dwell_time when processing_times is not provided.
        """
        pt = processing_times or {}
        self.get_logger().info(f"[{agv_key}] Starting route  ({len(segments)} segments)")

        for seg_idx, (node_id, op_label, moves) in enumerate(segments):
            if not moves:
                continue

            move_str = ','.join(moves)
            label    = f"node={node_id} op={op_label}" if node_id is not None else "RETURN"
            self.get_logger().info(
                f"[{agv_key}] seg {seg_idx}: {label}  moves=[{move_str}]")

            # Load and start the segment
            self._send(agv_key, f'route {move_str}')
            time.sleep(0.15)   # brief gap so Arduino processes the route command
            self._send(agv_key, 'go')

            # Wait for the AGV to reach ARRIVED state
            ok = self._wait_for_state(agv_key, 'ARRIVED', timeout=180.0)
            if not ok:
                self.get_logger().error(
                    f"[{agv_key}] Timeout waiting for ARRIVED at seg {seg_idx}")
                self._send(agv_key, 'stop')
                return

            # VRP-RPD processing time gating
            if node_id is not None and op_label == 'D' and node_id in pt:
                ready_at = time.time() + pt[node_id]
                with self._resource_lock:
                    self._resource_ready_at[node_id] = ready_at
                self.get_logger().info(
                    f"[{agv_key}] D@{node_id}: resource ready in {pt[node_id]:.1f}s")

            elif node_id is not None and op_label == 'P':
                with self._resource_lock:
                    ready_at = self._resource_ready_at.get(node_id, 0.0)
                wait_s = ready_at - time.time()
                if wait_s > 0:
                    self.get_logger().info(
                        f"[{agv_key}] P@{node_id}: waiting {wait_s:.1f}s for resource")
                    time.sleep(wait_s)

            elif dwell_time > 0.0 and op_label not in ('D', 'P', 'RETURN'):
                # Legacy fixed dwell for non-VRP-RPD schedules
                self.get_logger().info(f"[{agv_key}] Dwelling {dwell_time}s at {label}")
                time.sleep(dwell_time)

        self.get_logger().info(f"[{agv_key}] Route complete")

    # ── Low-level helpers ────────────────────────────────────────────────

    def _stop_all(self) -> None:
        for key in self._agv_keys():
            self._send(key, 'stop')
        self.get_logger().info("Stop sent to all AGVs")

    def _send(self, agv_key: str, command: str) -> None:
        if agv_key not in self.cmd_pubs:
            self.get_logger().error(f"Unknown AGV key: {agv_key}")
            return
        msg      = String()
        msg.data = command
        self.cmd_pubs[agv_key].publish(msg)
        self.get_logger().debug(f"→ [{agv_key}] {command}")
        time.sleep(0.05)   # let micro-ROS agent flush

    def _wait_for_state(
        self, agv_key: str, target: str, timeout: float = 60.0
    ) -> bool:
        """Block until the AGV reports target state, or timeout expires."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._state_lock:
                if self.agv_state.get(agv_key) == target:
                    return True
            # Wait for the next status update (event fires on every state change)
            ev = self._state_events[agv_key]
            ev.clear()
            ev.wait(timeout=0.5)
        return False

    def _publish_status(self, status: str) -> None:
        msg      = String()
        msg.data = status
        self.global_status_pub.publish(msg)
        self.get_logger().info(f"FACTORY STATUS: {status}")


# ── Entry point ───────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AGVFactoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
