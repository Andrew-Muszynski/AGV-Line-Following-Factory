#!/usr/bin/env python3
"""Are the robots ACTUALLY publishing? Run this before every mission.

Three ways to be told a robot is fine when it is not:

  GREEN LED      means the robot reached SOME agent once. It says nothing
                 about whether a session is alive now. Six green LEDs sat
                 there on 2026-09-30 with every session dead.

  ros2 topic info  reported "Publisher count: 1" for all six robots at the
                 same time -- the agent keeps the DDS publisher registered
                 after the XRCE-DDS session behind it has gone. It even
                 reported 0 and 1 for the SAME topic minutes apart, because
                 the ros2 daemon caches the graph.

  ros2 topic echo --once, per robot, in a loop  is what this file replaces.
                 Every invocation is a FRESH node that must redo DDS
                 discovery before it can receive, and a short timeout then
                 reports a perfectly healthy robot as SILENT. Run three
                 times in a row it blamed Alvik4, then Alvik1+Alvik4, then
                 Alvik3 -- all six were publishing at 19.6-20.0 Hz the
                 whole time. A different robot failing each run is the tell.

One node, one discovery round, real message counts. The firmware publishes
_pose at 20Hz continuously, so anything below ~19Hz is a real problem and
anything near zero is a dead session -- power-cycle that robot AFTER the
micro-ROS agent is up.

    python3 /tmp/check_robots_live.py        # on the machine hosting the agent
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import time

rclpy.init()
n = Node("pose_rate_check")
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
counts = {}
for i in range(1, 7):
    name = f"Alvik{i}"
    counts[name] = 0
    def cb(msg, nm=name):
        counts[nm] += 1
    n.create_subscription(String, f"{name}_pose", cb, qos)

print("discovering for 3s, then counting for 6s...")
t0 = time.monotonic()
while time.monotonic() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.05)
for k in counts:
    counts[k] = 0
t0 = time.monotonic()
while time.monotonic() - t0 < 6.0:
    rclpy.spin_once(n, timeout_sec=0.05)
el = time.monotonic() - t0
print()
for k in sorted(counts):
    hz = counts[k] / el
    print(f"  {k}: {counts[k]:4d} msgs  {hz:5.1f} Hz  "
          f"{'LIVE' if hz > 1.0 else '*** SILENT ***'}")
n.destroy_node(); rclpy.shutdown()
