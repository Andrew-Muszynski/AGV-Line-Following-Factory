"""
testbed_to_factory.py  —  Bridge CLI
--------------------------------------
Converts an agv_testbed pipeline JSON into a factory node dispatch payload
and either saves it to a file or publishes it live via ROS2.

Usage:
  # Dry run — inspect the payload without touching any robot
  python3 testbed_to_factory.py --input gr21_base_42.json --dry-run

  # Save to file (for inspection or later dispatch)
  python3 testbed_to_factory.py --input gr21_base_42.json --output dispatch.json

  # Live dispatch to a running factory node via ROS2
  python3 testbed_to_factory.py --input gr21_base_42.json --dispatch
"""

from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

from path_translator import build_factory_payload


def _print_summary(payload: dict) -> None:
    print(f"\n=== Factory Dispatch Payload ===")
    print(f"  Depot         : {payload['depot_rc']}")
    print(f"  node_rcs size : {len(payload['node_rcs'])} entries")
    print(f"  dwell_time    : {payload['dwell_time']}s")
    print(f"  processing_times: {len(payload['processing_times'])} workstations")
    print()
    for v, route in enumerate(payload['routes']):
        drops = [s['node'] for s in route if s['op'] == 'D']
        picks = [s['node'] for s in route if s['op'] == 'P']
        print(f"  AGV {v+1}: {len(route)} stops")
        print(f"    Dropoffs: {drops}")
        print(f"    Pickups : {picks}")
    print()

    # Show node_rcs for workstation nodes so you can verify physical positions
    ws_nodes = sorted(
        {s['node'] for route in payload['routes'] for s in route}
    )
    print("  Workstation → physical grid intersection (factory row, col):")
    for n in ws_nodes:
        pt = payload['processing_times'].get(str(n), 0.0)
        print(f"    node {n:3d}  →  {payload['node_rcs'][n]}  proc={pt:.1f}s")
    print()


def _dispatch_via_ros2(payload: dict) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("[bridge] rclpy not available — cannot dispatch via ROS2.", file=sys.stderr)
        sys.exit(1)

    # processing_times must be included in the DISPATCH payload so the
    # factory node can gate pickups on per-node timers.
    command = f"DISPATCH {json.dumps(payload)}"

    rclpy.init()
    node = Node('testbed_factory_bridge')
    pub = node.create_publisher(String, '/agv_factory/command', 10)

    import time
    time.sleep(0.5)   # let DDS discover the factory node subscriber

    msg = String()
    msg.data = command
    pub.publish(msg)
    node.get_logger().info("Schedule dispatched to /agv_factory/command")
    time.sleep(0.3)

    node.destroy_node()
    rclpy.shutdown()
    print("[bridge] Dispatched.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert agv_testbed pipeline JSON → factory node dispatch")
    parser.add_argument('--input', required=True,
                        help="Path to pipeline JSON saved by web_viewer --save")
    parser.add_argument('--output', default=None,
                        help="Save dispatch payload to this JSON file")
    parser.add_argument('--dispatch', action='store_true',
                        help="Publish directly to /agv_factory/command via ROS2")
    parser.add_argument('--dry-run', action='store_true',
                        help="Print summary only, do not save or dispatch")
    args = parser.parse_args()

    payload = build_factory_payload(args.input)
    _print_summary(payload)

    if args.dry_run:
        print("[bridge] Dry run — no output written.")
        return

    if args.output:
        Path(args.output).write_text(json.dumps(payload, indent=2))
        print(f"[bridge] Saved payload → {args.output}")

    if args.dispatch:
        _dispatch_via_ros2(payload)


if __name__ == '__main__':
    main()
