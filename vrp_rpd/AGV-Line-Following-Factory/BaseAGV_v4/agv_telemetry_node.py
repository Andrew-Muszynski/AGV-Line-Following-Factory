#!/usr/bin/env python3
"""
agv_telemetry_node.py  —  ROS2 telemetry subscriber + CSV logger

Subscribes to AlvikN_status topics for up to 3 AGVs and logs every
status message to a timestamped CSV file when demo logging is active.

Usage:
    ros2 run <package> agv_telemetry_node
    # or directly:
    python agv_telemetry_node.py

Control logging via the /agv_telemetry/control topic:
    ros2 topic pub /agv_telemetry/control std_msgs/msg/String "data: START"
    ros2 topic pub /agv_telemetry/control std_msgs/msg/String "data: STOP"
"""
from __future__ import annotations

import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CSV_COLUMNS = [
    "wall_time", "ros_ms", "agv",
    "state", "step", "total",
    "x", "y", "yaw",
    "L", "C", "R",
    "tof_fl", "tof_fcl", "tof_fc", "tof_fcr", "tof_fr",
    "obj", "bat", "color",
]


class AGVTelemetryNode(Node):

    def __init__(self) -> None:
        super().__init__('agv_telemetry_node')

        self._logging_active = False
        self._csv_file: Optional[object] = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._log_path: Optional[Path] = None

        self._setup_subscriptions()

        self.get_logger().info("AGV Telemetry Node ready.  "
                               "Publish START/STOP to /agv_telemetry/control to toggle logging.")

    def _setup_subscriptions(self) -> None:
        for i in range(1, 4):
            self.create_subscription(
                String, f'Alvik{i}_status',
                lambda msg, agv=i: self._on_status(agv, msg),
                10,
            )

        self.create_subscription(
            String, '/agv_telemetry/control',
            self._on_control, 10,
        )

        self.status_pub = self.create_publisher(
            String, '/agv_telemetry/status', 10,
        )

    def _on_control(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self._start_logging()
        elif cmd == 'STOP':
            self._stop_logging()
        else:
            self.get_logger().warn(f"Unknown control command: {cmd}")

    def _start_logging(self) -> None:
        if self._logging_active:
            self.get_logger().info("Logging already active.")
            return

        log_dir = Path(os.path.dirname(__file__)) / 'logs'
        log_dir.mkdir(exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = log_dir / f'agv_telemetry_{timestamp}.csv'

        self._csv_file = open(self._log_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS,
                                          extrasaction='ignore')
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self._logging_active = True
        self.get_logger().info(f"Logging started → {self._log_path}")
        self._publish_status(f"LOGGING_STARTED {self._log_path}")

    def _stop_logging(self) -> None:
        if not self._logging_active:
            self.get_logger().info("Logging not active.")
            return

        self._logging_active = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

        self.get_logger().info(f"Logging stopped.  File: {self._log_path}")
        self._publish_status(f"LOGGING_STOPPED {self._log_path}")

    def _on_status(self, agv_id: int, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        wall_time = datetime.now().isoformat(timespec='milliseconds')

        tof = data.get('tof', [0, 0, 0, 0, 0])
        while len(tof) < 5:
            tof.append(0)

        row = {
            'wall_time': wall_time,
            'ros_ms':    data.get('ms', 0),
            'agv':       agv_id,
            'state':     data.get('state', ''),
            'step':      data.get('step', 0),
            'total':     data.get('total', 0),
            'x':         data.get('x', 0.0),
            'y':         data.get('y', 0.0),
            'yaw':       data.get('yaw', 0.0),
            'L':         data.get('L', 0),
            'C':         data.get('C', 0),
            'R':         data.get('R', 0),
            'tof_fl':    tof[0],
            'tof_fcl':   tof[1],
            'tof_fc':    tof[2],
            'tof_fcr':   tof[3],
            'tof_fr':    tof[4],
            'obj':       data.get('obj', 0),
            'bat':       data.get('bat', 0),
            'color':     data.get('color', ''),
        }

        if self._logging_active and self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_file.flush()

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AGVTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
