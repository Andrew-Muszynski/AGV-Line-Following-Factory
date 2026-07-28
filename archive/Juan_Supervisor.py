#!/usr/bin/env python3
"""
juan_supervisor.py  —  Step-by-step command supervisor for juan_code.ino

juan_code.ino is a thin executor: it accepts ONE atomic command at a time on
<ROBOT_NAME>_cmd, executes it, and publishes status updates on
<ROBOT_NAME>_status (ROBOT_NAME is e.g. Alvik1, Alvik2, ... — each robot picks
its own name from its WiFi MAC address). The PC (this script) is the brain —
it holds the full route and sends the next command only after the robot
reports it has finished the current one.

Usage:
    python3 juan_supervisor.py --robot Alvik1 FORWARD_UNTIL_RED RIGHT_UNTIL_COLOR ...

    # or load a route from route_planner-style tokens (R/L/RED -> juan commands):
    python3 juan_supervisor.py --robot Alvik1 --tokens R,RED,RED,L,RED

    # or load a route from a file (one command per line, '#' comments allowed):
    python3 juan_supervisor.py --robot Alvik1 --file full_route_juan.txt

Run one instance of this script per robot (each with its own --robot name and
route) to control multiple AGVs at once.

Recognized juan_code.ino commands (sent verbatim on <ROBOT_NAME>_cmd):
    FORWARD_UNTIL_RED     FORWARD_UNTIL_COLOR    BACKWARD_UNTIL_COLOR
    FORWARD_UNTIL_YELLOW  BACKWARD_UNTIL_YELLOW
    FORWARD_UNTIL_BLUE    BACKWARD_UNTIL_BLUE
    RIGHT_UNTIL_COLOR     LEFT_UNTIL_COLOR       ROTATE_180
    DWELL                 STOP                   RESET_POSE    GET_STATUS

The robot reports "BUSY ..." while executing and "IDLE" (or "DETECTED ...",
"TURN COMPLETE", "STOPPED", "POSE_RESET") when ready for the next command.
This script waits for one of the "done" markers below before sending the
next command in the sequence.
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# Status strings that mean "robot is ready for the next command".
# "TURN COMPLETE" / "ROTATE_180 COMPLETE" are informational only — juan_code.ino
# always publishes "IDLE" right after them in the same step, so "IDLE" alone
# is the real done signal. Including the others here causes a double-trigger:
# the first message advances to the next command, then the still-in-flight
# "IDLE" gets misread as that *next* command's done-marker.
DONE_MARKERS = ("IDLE", "STOPPED", "POSE_RESET", "ERROR")

# Command delivery is best-effort (the sketch subscribes best-effort), so a
# published command CAN be silently lost — seen 2026-07-13: on a second run
# the first FORWARD_UNTIL_BLUE never arrived, the robot's 1 Hz idle heartbeat
# was mistaken for completion, and the next command (a turn) fired at the
# depot. The robot's immediate "BUSY <cmd>" status is our delivery ack:
# resend until it appears, and never count a done-marker before it.
ACK_TIMEOUT_SEC = 2.0
MAX_SEND_ATTEMPTS = 4

# Map route_planner-style tokens to juan_code.ino commands (best-effort —
# only covers tokens juan_code.ino actually understands).
TOKEN_TO_CMD = {
    "RED":    "FORWARD_UNTIL_RED",
    "YELLOW": "FORWARD_UNTIL_YELLOW",
    "BLUE":   "FORWARD_UNTIL_BLUE",
    "R":      "RIGHT_UNTIL_COLOR",
    "L":      "LEFT_UNTIL_COLOR",
}


class JuanSupervisor(Node):
    def __init__(self, robot_name: str, commands: list[str]):
        super().__init__(f"juan_supervisor_{robot_name.lower()}")
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # the sketch publishes status RELIABLE; subscribe reliable so BUSY
        # acks and done markers can't be dropped on our side
        qos_status = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._commands = commands
        self._index = 0
        self._done_event = threading.Event()
        self._ack_event = threading.Event()
        self._ack_seen = False
        self._skip_next_idle = False

        cmd_topic = f"{robot_name}_cmd"
        status_topic = f"{robot_name}_status"
        self._cmd_pub = self.create_publisher(String, cmd_topic, qos_cmd)
        self._status_sub = self.create_subscription(
            String, status_topic, self._on_status, qos_status)

        self.get_logger().info(
            f"Loaded {len(commands)} command(s) for {robot_name} "
            f"(cmd={cmd_topic}, status={status_topic})")

    def _on_status(self, msg: String):
        text = msg.data.strip()
        self.get_logger().info(f"status: {text}")

        if text.startswith("ERROR"):
            self.get_logger().error(f"Robot reported error: {text}")
            self._done_event.set()
            return

        # "BUSY <cmd>" is the robot confirming it received our command.
        if text.startswith("BUSY"):
            self._ack_seen = True
            self._ack_event.set()
            return

        # The robot sometimes double-publishes "IDLE" right after
        # "TURN COMPLETE"/"ROTATE_180 COMPLETE", and the duplicate can
        # arrive just after we've sent the next command. A bare "IDLE"
        # immediately after sending can't be a real completion (no
        # command finishes in milliseconds), so discard exactly one.
        if self._skip_next_idle and text == "IDLE":
            self._skip_next_idle = False
            return
        self._skip_next_idle = False

        # Until the robot has ACKed the current command, any "IDLE" is just
        # its 1 Hz idle heartbeat — counting it as done would advance the
        # route past a command the robot never received.
        if not self._ack_seen:
            return

        if any(text == m or text.startswith(m + " ") for m in DONE_MARKERS):
            self._done_event.set()

    def _send(self, cmd: str, attempt: int = 1):
        note = f" (resend #{attempt - 1})" if attempt > 1 else ""
        self.get_logger().info(f"-> {cmd}{note}")
        self._done_event.clear()
        self._ack_event.clear()
        self._ack_seen = False
        self._skip_next_idle = True
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)

    def _send_acked(self, cmd: str) -> bool:
        """Publish and resend until the robot confirms with 'BUSY <cmd>'."""
        for attempt in range(1, MAX_SEND_ATTEMPTS + 1):
            self._send(cmd, attempt)
            if self._ack_event.wait(timeout=ACK_TIMEOUT_SEC):
                return True
            self.get_logger().warning(
                f"no BUSY ack for '{cmd}' within {ACK_TIMEOUT_SEC:.0f}s — resending")
        return False

    def run(self, timeout_sec: float = 30.0) -> bool:
        """Send each command in order, waiting for a done marker after each.
        Returns True if all commands completed, False on timeout/error."""
        # A freshly created publisher takes a moment to match the robot's
        # subscriber; publishing before that silently drops the message.
        t0 = time.monotonic()
        while self._cmd_pub.get_subscription_count() == 0:
            if time.monotonic() - t0 > 10.0:
                self.get_logger().error(
                    "cmd topic has no subscriber after 10s — robot offline?")
                return False
            time.sleep(0.1)
        time.sleep(0.3)

        for cmd in self._commands:
            if not self._send_acked(cmd):
                self.get_logger().error(
                    f"Robot never acknowledged '{cmd}' after "
                    f"{MAX_SEND_ATTEMPTS} attempts — aborting.")
                return False
            if not self._done_event.wait(timeout=timeout_sec):
                self.get_logger().error(
                    f"Timed out waiting for robot to finish: {cmd}")
                return False
        self.get_logger().info("All commands completed.")
        return True


def commands_from_file(path: str) -> list[str]:
    """Read one command per line. Blank lines and lines starting with
    '#' (comments, e.g. '# --- WS visit #1 ---') are skipped."""
    commands = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            commands.append(line.upper())
    return commands


def tokens_to_commands(token_csv: str) -> list[str]:
    tokens = [t.strip().upper() for t in token_csv.split(",") if t.strip()]
    commands = []
    for tok in tokens:
        if tok not in TOKEN_TO_CMD:
            raise ValueError(
                f"Token '{tok}' has no juan_code.ino equivalent "
                f"(supported: {sorted(TOKEN_TO_CMD)})")
        commands.append(TOKEN_TO_CMD[tok])
    return commands


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)

    if args[0] != "--robot" or len(args) < 3:
        print("usage: juan_supervisor.py --robot AlvikN <commands | --tokens ... | --file ...>")
        sys.exit(1)
    robot_name = args[1]
    args = args[2:]

    if args[0] == "--tokens":
        if len(args) != 2:
            print("usage: juan_supervisor.py --robot AlvikN --tokens R,RED,RED,L,RED")
            sys.exit(1)
        commands = tokens_to_commands(args[1])
    elif args[0] == "--file":
        if len(args) != 2:
            print("usage: juan_supervisor.py --robot AlvikN --file route.txt")
            sys.exit(1)
        commands = commands_from_file(args[1])
    else:
        commands = [a.upper() for a in args]

    rclpy.init()
    node = JuanSupervisor(robot_name, commands)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()