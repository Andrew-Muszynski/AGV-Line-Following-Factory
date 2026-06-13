#!/usr/bin/env python3
"""
dispatch_node.py  —  ROS 2 dispatcher for AGV_MULTI_WS_DISPATCH

This node owns all routing intelligence:
  • It receives high-level mission commands (which AGV visits which workstations).
  • It calls route_planner.py to convert workstation lists into token sequences.
  • It sends each AGV its token sequence via the 'run <tokens>' command.
  • It monitors status and can inject a new token sequence mid-mission (reroute).
  • It publishes a JSON summary of all AGV states.

High-level command topic:  /agv_dispatch/command   (std_msgs/String)
Global status topic:       /agv_dispatch/status    (std_msgs/String)
Per-AGV cmd topic:         <RobotName>_cmd         (std_msgs/String)
Per-AGV status topic:      <RobotName>_status      (std_msgs/String)

Command format
--------------
STOP_ALL
STOP <agv_key>                       e.g.  STOP agv_1
PAUSE <agv_key>
RESUME <agv_key>

DISPATCH <json>
  JSON schema:
  {
    "routes": [
      {"agv": "agv_1", "workstations": ["WS01", "WS04", "WS07"]},
      {"agv": "agv_2", "workstations": ["WS02", "WS05"]}
    ],
    "return_to_depot": true        // optional, default true
  }

RANDOM <json>
  JSON schema:
  {
    "agv": "agv_1",
    "n_stops": 3,
    "seed": 42,                    // optional
    "return_to_depot": true
  }

REROUTE <json>
  Replace the current token queue for one AGV (takes effect after current token).
  JSON schema:
  {
    "agv": "agv_1",
    "workstations": ["WS03", "WS06"],
    "return_to_depot": true
  }
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from route_planner import (
    RoutePlan,
    Workstation,
    load_workstations,
    random_route,
)

_WS_JSON = Path(__file__).parent / "workstations.json"


class DispatchNode(Node):

    def __init__(self) -> None:
        super().__init__("agv_dispatch_node")
        self.get_logger().info("AGV DISPATCH NODE starting")

        self._registry: Dict[str, Workstation] = load_workstations(_WS_JSON)
        self.get_logger().info(f"Loaded {len(self._registry)} workstations")

        self._load_config()
        self._setup_topics()

        # Per-AGV live state
        self._state:  Dict[str, str]  = {k: "UNKNOWN"  for k in self._keys}
        self._step:   Dict[str, int]  = {k: 0           for k in self._keys}
        self._total:  Dict[str, int]  = {k: 0           for k in self._keys}
        self._token:  Dict[str, str]  = {k: ""          for k in self._keys}
        self._ready:  Dict[str, bool] = {k: False        for k in self._keys}
        self._active: Dict[str, bool] = {k: False        for k in self._keys}

        # Current route plan per AGV (for status display)
        self._plans: Dict[str, Optional[RoutePlan]] = {k: None for k in self._keys}

        self._state_lock = threading.Lock()
        self._state_events: Dict[str, threading.Event] = {
            k: threading.Event() for k in self._keys
        }

        self.get_logger().info("AGV DISPATCH NODE ready")

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def _load_config(self) -> None:
        cfg_path = Path(__file__).parent / "agv_robots.yaml"
        try:
            with open(cfg_path) as f:
                cfg = yaml.safe_load(f)
            self._agv_ids: List[int] = []
            self._hw_name: Dict[str, str] = {}
            for key, val in cfg.get("agvs", {}).items():
                if key.startswith("agv_") and key.split("_")[1].isdigit():
                    n = int(key.split("_")[1])
                    self._agv_ids.append(n)
                    self._hw_name[f"agv_{n}"] = val.get("name", f"Alvik{n}")
            self._agv_ids.sort()
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({exc}), using defaults")
            self._agv_ids = [1, 2, 3]
            self._hw_name = {f"agv_{i}": f"Alvik{i}" for i in self._agv_ids}

        self._keys = [f"agv_{i}" for i in self._agv_ids]
        self.get_logger().info(f"AGVs: {self._keys}")

    # ------------------------------------------------------------------
    # ROS topics
    # ------------------------------------------------------------------

    def _setup_topics(self) -> None:
        self._cmd_pubs: Dict[str, any] = {}
        for agv_id in self._agv_ids:
            key  = f"agv_{agv_id}"
            name = self._hw_name[key]
            self._cmd_pubs[key] = self.create_publisher(String, f"{name}_cmd", 10)
            self.create_subscription(
                String, f"{name}_status",
                lambda msg, k=key: self._on_status(k, msg), 10,
            )
            self.get_logger().info(f"  pub={name}_cmd  sub={name}_status")

        self.create_subscription(
            String, "/agv_dispatch/command", self._on_command, 10
        )
        self._global_pub = self.create_publisher(String, "/agv_dispatch/status", 10)
        self.create_timer(1.0, self._publish_global_status)

    # ------------------------------------------------------------------
    # Status callback
    # ------------------------------------------------------------------

    def _on_status(self, key: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        state  = data.get("state", "UNKNOWN")
        step   = int(data.get("step",  0))
        total  = int(data.get("total", 0))
        token  = str(data.get("token", ""))
        ready  = bool(data.get("ready", 0))
        active = bool(data.get("active", 0))

        with self._state_lock:
            changed = self._state.get(key) != state
            self._state[key]  = state
            self._step[key]   = step
            self._total[key]  = total
            self._token[key]  = token
            self._ready[key]  = ready
            self._active[key] = active

        if changed:
            self.get_logger().info(
                f"[{key}] {state}  step={step}/{total}  token={token}"
            )
            self._state_events[key].set()

    # ------------------------------------------------------------------
    # High-level command callback
    # ------------------------------------------------------------------

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip()
        self.get_logger().info(f"CMD: {cmd}")
        upper = cmd.upper()

        if upper in ("STOP_ALL", "INITIALIZE_ALL"):
            self._stop_all()
            return

        if upper.startswith("STOP "):
            key = cmd.split(None, 1)[1].strip().lower()
            self._send(key, "stop")
            return

        if upper.startswith("PAUSE "):
            key = cmd.split(None, 1)[1].strip().lower()
            self._send(key, "pause")
            return

        if upper.startswith("RESUME "):
            key = cmd.split(None, 1)[1].strip().lower()
            self._send(key, "resume")
            return

        if upper.startswith("DISPATCH "):
            payload = cmd[9:].strip()
            threading.Thread(
                target=self._dispatch, args=(payload,), daemon=True
            ).start()
            return

        if upper.startswith("RANDOM "):
            payload = cmd[7:].strip()
            threading.Thread(
                target=self._random_dispatch, args=(payload,), daemon=True
            ).start()
            return

        if upper.startswith("REROUTE "):
            payload = cmd[8:].strip()
            threading.Thread(
                target=self._reroute, args=(payload,), daemon=True
            ).start()
            return

        if upper.startswith("GRID_TEST"):
            # GRID_TEST [agv_key]  — full serpentine traversal of 8x8 grid
            parts = cmd.split(None, 1)
            key = parts[1].strip().lower() if len(parts) > 1 else "agv_1"
            threading.Thread(
                target=self._grid_test, args=(key,), daemon=True
            ).start()
            return

        self.get_logger().warn(f"Unknown command: {cmd}")

    # ------------------------------------------------------------------
    # DISPATCH  (multi-AGV, parallel)
    # ------------------------------------------------------------------

    def _dispatch(self, json_str: str) -> None:
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"DISPATCH: bad JSON — {exc}")
            return

        routes         = data.get("routes", [])
        return_depot   = bool(data.get("return_to_depot", True))

        threads = []
        for route_spec in routes:
            key = str(route_spec.get("agv", "agv_1")).lower()
            ws_ids = [w.upper() for w in route_spec.get("workstations", [])]
            if not ws_ids:
                continue

            plan = RoutePlan(
                agv_id=key,
                workstation_ids=ws_ids,
                return_to_depot=return_depot,
            ).build(self._registry)

            with self._state_lock:
                self._plans[key] = plan

            self.get_logger().info(
                f"[{key}] Route: {ws_ids}  tokens={len(plan.tokens)}"
            )
            t = threading.Thread(
                target=self._run_plan, args=(key, plan), daemon=True
            )
            threads.append(t)

        self._publish_event("DISPATCH_START")
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self._publish_event("DISPATCH_COMPLETE")

    # ------------------------------------------------------------------
    # RANDOM  (single AGV random route)
    # ------------------------------------------------------------------

    def _random_dispatch(self, json_str: str) -> None:
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"RANDOM: bad JSON — {exc}")
            return

        key          = str(data.get("agv", "agv_1")).lower()
        n_stops      = int(data.get("n_stops", 3))
        seed         = data.get("seed", None)
        return_depot = bool(data.get("return_to_depot", True))

        ws_ids, tokens = random_route(
            n_stops, registry=self._registry,
            seed=seed, return_to_depot=return_depot,
        )
        plan = RoutePlan(agv_id=key, workstation_ids=ws_ids,
                         return_to_depot=return_depot)
        plan.tokens = tokens

        self.get_logger().info(
            f"[{key}] Random route ({n_stops} stops): {ws_ids}"
        )

        with self._state_lock:
            self._plans[key] = plan

        self._publish_event(f"RANDOM_ROUTE_START {key}")
        self._run_plan(key, plan)
        self._publish_event(f"RANDOM_ROUTE_COMPLETE {key}")

    # ------------------------------------------------------------------
    # REROUTE  (replace token script, send stop then new run)
    # ------------------------------------------------------------------

    def _reroute(self, json_str: str) -> None:
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"REROUTE: bad JSON — {exc}")
            return

        key      = str(data.get("agv", "agv_1")).lower()
        ws_ids   = [w.upper() for w in data.get("workstations", [])]
        return_d = bool(data.get("return_to_depot", True))

        if not ws_ids:
            self.get_logger().warn("REROUTE: empty workstation list")
            return

        plan = RoutePlan(
            agv_id=key, workstation_ids=ws_ids, return_to_depot=return_d
        ).build(self._registry)

        self.get_logger().info(
            f"[{key}] REROUTE to {ws_ids}  tokens={len(plan.tokens)}"
        )

        # Stop the current script and wait for IDLE, then send the new one
        self._send(key, "stop")
        ok = self._wait_for_state(key, "IDLE", timeout=10.0)
        if not ok:
            # Try NOT_READY state as well
            with self._state_lock:
                ok = self._state.get(key) in ("IDLE", "NOT_READY", "ARRIVED")
        if not ok:
            self.get_logger().error(f"[{key}] Did not reach IDLE after stop — aborting reroute")
            return

        with self._state_lock:
            self._plans[key] = plan

        self._send(key, plan.run_command)
        self._publish_event(f"REROUTE_SENT {key}")

    # ------------------------------------------------------------------
    # Execute one plan (blocking, called from thread)
    # ------------------------------------------------------------------

    def _run_plan(self, key: str, plan: RoutePlan) -> None:
        self.get_logger().info(f"[{key}] Waiting for AGV to be ready (IDLE)…")

        # Wait until the robot has confirmed its blue start marker
        ok = self._wait_for_ready(key, timeout=60.0)
        if not ok:
            self.get_logger().error(f"[{key}] Timed out waiting for ready state")
            return

        self.get_logger().info(
            f"[{key}] Sending: {plan.run_command[:80]}…"
        )
        self._send(key, plan.run_command)

        # Wait for the AGV to finish (ARRIVED state)
        ok = self._wait_for_state(key, "ARRIVED", timeout=600.0)
        if not ok:
            self.get_logger().error(f"[{key}] Timed out waiting for ARRIVED")
            self._send(key, "stop")
        else:
            self.get_logger().info(f"[{key}] Mission complete")

    # ------------------------------------------------------------------
    # GRID_TEST  (full 8x8 serpentine traversal)
    # ------------------------------------------------------------------

    # Serpentine route: col1 N (rows 1→8), row8 E (cols 1→8),
    # step S to row7 col8, row7 W (cols 8→2), step S to row6 col2,
    # row6 E (cols 2→8), ... continuing boustrophedon down to row1,
    # then row1 W back to depot (col1 row1).  64 RED + 48 CLEAR + 9 R + 6 L = 127 tokens.
    _GRID_TEST_TOKENS = (
        "RED,RED,RED,RED,RED,RED,RED,"   # col1 N  (7 reds)
        "R,"                             # → E
        "RED,RED,RED,RED,RED,RED,RED,"   # row8 E  (7 reds)
        "R,"                             # → S
        "RED,"                           # step to row7
        "R,"                             # → W
        "RED,RED,RED,RED,RED,RED,"       # row7 W  (6 reds)
        "L,"                             # → S
        "RED,"                           # step to row6
        "L,"                             # → E
        "RED,RED,RED,RED,RED,RED,"       # row6 E  (6 reds)
        "R,"                             # → S
        "RED,"                           # step to row5
        "R,"                             # → W
        "RED,RED,RED,RED,RED,RED,"       # row5 W  (6 reds)
        "L,"                             # → S
        "RED,"                           # step to row4
        "L,"                             # → E
        "RED,RED,RED,RED,RED,RED,"       # row4 E  (6 reds)
        "R,"                             # → S
        "RED,"                           # step to row3
        "R,"                             # → W
        "RED,RED,RED,RED,RED,RED,"       # row3 W  (6 reds)
        "L,"                             # → S
        "RED,"                           # step to row2
        "L,"                             # → E
        "RED,RED,RED,RED,RED,RED,"       # row2 E  (6 reds)
        "R,"                             # → S
        "RED,"                           # step to row1
        "R,"                             # → W
        "RED,RED,RED,RED,RED,RED,RED"    # row1 W  (7 reds, back to depot)
    )

    def _grid_test(self, key: str) -> None:
        self.get_logger().info(f"[{key}] GRID_TEST: waiting for AGV ready…")
        ok = self._wait_for_ready(key, timeout=60.0)
        if not ok:
            self.get_logger().error(f"[{key}] GRID_TEST: timed out waiting for ready")
            return

        tok_str = self._GRID_TEST_TOKENS.replace(" ", "").replace("\n", "")
        toks = tok_str.split(",")
        n = len(toks)
        self.get_logger().info(f"[{key}] GRID_TEST streaming {n} tokens (2-token lookahead)")

        # Send first two tokens as the initial script
        self._send(key, f"run {toks[0]},{toks[1]}")
        next_to_send = 2   # index of the next token not yet sent to the robot

        deadline = time.time() + 600.0
        last_step = 0

        while time.time() < deadline:
            with self._state_lock:
                state = self._state.get(key, "")
                step  = self._step.get(key, 0)

            if state == "ARRIVED":
                break

            # When the robot completes a token (step advances), send the next lookahead
            if step > last_step and next_to_send < n:
                self._send(key, f"append {toks[next_to_send]}")
                self.get_logger().info(
                    f"[{key}] step={step}/{n}  appended tok[{next_to_send}]={toks[next_to_send]}"
                )
                next_to_send += 1
                last_step = step

            ev = self._state_events[key]
            ev.clear()
            ev.wait(timeout=0.3)

        with self._state_lock:
            state = self._state.get(key, "")

        if state == "ARRIVED":
            self.get_logger().info(
                f"[{key}] GRID_TEST complete — check {self._hw_name.get(key, key)}_color for red_count"
            )
        else:
            self.get_logger().error(f"[{key}] GRID_TEST timed out")
            self._send(key, "stop")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop_all(self) -> None:
        for key in self._keys:
            self._send(key, "stop")
        self.get_logger().info("stop sent to all AGVs")

    def _send(self, key: str, command: str) -> None:
        if key not in self._cmd_pubs:
            self.get_logger().error(f"Unknown AGV key: {key}")
            return
        msg = String()
        msg.data = command
        self._cmd_pubs[key].publish(msg)
        self.get_logger().debug(f"→ [{key}] {command[:60]}")
        time.sleep(0.05)

    def _wait_for_state(self, key: str, target: str, timeout: float = 60.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._state_lock:
                if self._state.get(key) == target:
                    return True
            ev = self._state_events[key]
            ev.clear()
            ev.wait(timeout=0.5)
        return False

    def _wait_for_ready(self, key: str, timeout: float = 60.0) -> bool:
        """Block until the AGV reports ready=1 (blue start confirmed)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._state_lock:
                if self._ready.get(key):
                    return True
            ev = self._state_events[key]
            ev.clear()
            ev.wait(timeout=0.5)
        return False

    def _publish_event(self, event: str) -> None:
        msg = String()
        msg.data = event
        self._global_pub.publish(msg)
        self.get_logger().info(f"EVENT: {event}")

    def _publish_global_status(self) -> None:
        summary = {}
        with self._state_lock:
            for key in self._keys:
                plan = self._plans.get(key)
                summary[key] = {
                    "state":  self._state.get(key, "UNKNOWN"),
                    "step":   self._step.get(key, 0),
                    "total":  self._total.get(key, 0),
                    "token":  self._token.get(key, ""),
                    "ready":  self._ready.get(key, False),
                    "active": self._active.get(key, False),
                    "route":  plan.workstation_ids if plan else [],
                }
        msg = String()
        msg.data = json.dumps(summary)
        self._global_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
