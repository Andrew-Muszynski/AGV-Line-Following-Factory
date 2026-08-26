#!/usr/bin/env python3
"""Start and monitor the one-AGV station A deliver/process/pickup cycle."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATION_A_CYCLE = (
    "run "
    "RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5,EXIT,R,RED,R,RED,R,BLUE,R_FINAL,"
    "WAIT30,"
    "RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5,EXIT,R,RED,R,RED,R,BLUE,R_FINAL"
)

FIRST_TO_SECOND_DROPOFF = (
    "run "
    "RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5,"
    "EXIT,R,RED,L,RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5"
)

TWO_DROPOFFS_RETURN_DEPOT = (
    "run "
    "RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5,"
    "EXIT,R,RED,L,RED,R_YSEARCH,YENTRY,R_SPUR,YWORK,YAW0,DOCK,DWELL5,"
    "EXIT,L,RED,L,RED,CLEAR,RED,R,BLUE,R_FINAL"
)


class StationACycleTest(Node):
    def __init__(self, robot_name: str, command: str, timeout: float) -> None:
        super().__init__("station_a_cycle_test")
        self.robot_name = robot_name
        self.command = command
        self.timeout = timeout
        self.done = False
        self.last_phase = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(String, f"{robot_name}_cmd", qos)
        self.sub = self.create_subscription(
            String, f"{robot_name}_status", self._on_status, 10
        )

    def send(self) -> None:
        time.sleep(1.0)
        msg = String()
        msg.data = self.command
        self.pub.publish(msg)
        self.get_logger().info(f"sent {self.command!r} to {self.robot_name}_cmd")

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"non-JSON status: {msg.data}")
            return

        phase = status.get("phase", "UNKNOWN")
        state = status.get("state", "UNKNOWN")
        remaining_ms = int(status.get("processing_remaining_ms", 0))

        if phase != self.last_phase:
            self.last_phase = phase
            self.get_logger().info(
                f"status phase={phase} state={state} processing_remaining={remaining_ms / 1000:.1f}s"
            )

        if phase == "COMPLETE" and state == "ARRIVED":
            self.done = True

    def run_until_done(self) -> bool:
        self.send()
        deadline = time.time() + self.timeout
        while rclpy.ok() and time.time() < deadline and not self.done:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.done


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send station_a_cycle to an Alvik and monitor completion."
    )
    parser.add_argument("--robot", default="Alvik1", help="robot topic prefix")
    parser.add_argument(
        "--command",
        default=STATION_A_CYCLE,
        help="command to publish",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()

    rclpy.init()
    node = StationACycleTest(args.robot, args.command, args.timeout)
    try:
        ok = node.run_until_done()
        if ok:
            node.get_logger().info("station A cycle complete")
        else:
            node.get_logger().error("timed out waiting for station A cycle completion")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
