#!/usr/bin/env python3
"""Does an rclpy subscription MATCH a publisher, and does it get DATA?

camera_grid_navigate.py gates startup on
`subscription.get_publisher_count() != 0`, and on 2026-09-29 that gate failed
for `/Alvik1_pose` -- "subscription never matched a publisher" -- while
`ros2 topic echo /Alvik1_pose --once` printed a message instantly from a
process started seconds later. Both use BEST_EFFORT/depth-10/VOLATILE, which
`ros2 topic info --verbose` confirms matches the publisher exactly.

So the question is narrow: for the same topic, same QoS, in one process, does
get_publisher_count() ever go non-zero, and do messages arrive regardless?
If data arrives while the count stays 0, the COUNT is the unreliable thing
and the startup gate is testing the wrong condition.

    python3 tools/probe_match.py /Alvik1_pose /Alvik1_vision_pose

Needs ROS sourced. Prints a line every 0.5s per topic, then a verdict.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].replace(
    ".", "", 1).isdigit() else 10.0


def probe(node: Node, topic: str, seconds: float) -> tuple[float | None, int]:
    """Returns (seconds until publisher_count>0 or None, messages received)."""
    got = {"n": 0}
    # Exactly what camera_grid_navigate.py uses for both pose topics.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    sub = node.create_subscription(
        String, topic, lambda _m: got.__setitem__("n", got["n"] + 1), qos)

    matched_at = None
    t0 = time.monotonic()
    last_print = 0.0
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        count = sub.get_publisher_count()
        if count and matched_at is None:
            matched_at = time.monotonic() - t0
        now = time.monotonic() - t0
        if now - last_print >= 0.5:
            last_print = now
            print(f"    t+{now:4.1f}s  publisher_count={count}  "
                  f"messages={got['n']}")
    node.destroy_subscription(sub)
    return matched_at, got["n"]


def main() -> int:
    topics = [a for a in sys.argv[1:] if a.startswith("/")]
    if not topics:
        topics = ["/Alvik1_pose", "/Alvik1_vision_pose"]
    rclpy.init()
    node = rclpy.create_node("probe_match")
    results = {}
    for topic in topics:
        print(f"\n{topic}")
        results[topic] = probe(node, topic, SECONDS)
    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 62)
    for topic, (matched_at, msgs) in results.items():
        m = f"{matched_at:.2f}s" if matched_at is not None else "NEVER"
        print(f"  {topic:26} matched {m:>7}   messages {msgs}")
    print()
    silent_but_flowing = [t for t, (m, n) in results.items()
                          if m is None and n > 0]
    if silent_but_flowing:
        print("  VERDICT: data arrives while get_publisher_count() stays 0 on")
        print(f"           {silent_but_flowing}.")
        print("           The COUNT is unreliable here -- the startup gate is")
        print("           testing the wrong condition and should gate on data.")
    elif any(m is None for m, _n in results.values()):
        print("  VERDICT: no match AND no data -- a real discovery failure,")
        print("           not a bad gate.")
    else:
        print("  VERDICT: everything matched. The failure is elsewhere in")
        print("           camera_grid_navigate.py, not in this gate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
