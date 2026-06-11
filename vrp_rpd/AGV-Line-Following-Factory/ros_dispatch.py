"""
ros_dispatch.py  —  Bridge from VRP-RPD solver output to ROS2 factory node.

Usage (standalone):
    python ros_dispatch.py --instance demo_instance.json \
                           --schedule demo_schedule.json \
                           [--dwell 2.0]

Or import and call publish_schedule() from the GUI after solving.

The factory node must already be running and reachable via a live ROS2
environment (micro-ROS agent up, /agv_factory/command topic active).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def build_dispatch_payload(
    instance,          # Instance object (from instance.py)
    schedule,          # Schedule object (from schedule.py)
    dwell_time: float = 0.0,
) -> dict:
    """
    Convert a VRP-RPD instance + schedule into the JSON payload consumed by
    the factory node's DISPATCH command.

    Returns a dict with keys:
      routes      per-AGV stop lists  [{node, op}, ...]
      depot_rc    [row, col]
      node_rcs    [[row, col], ...]  indexed by node id (64 entries for 8×8)
      dwell_time  float
    """
    depot_rc = [instance.depot_row, instance.depot_col]

    # Build node_rcs: for each node id 0..num_nodes-1, its (row, col)
    node_rcs = []
    for node_id in range(instance.num_nodes):
        r, c = instance.node_to_rc(node_id)
        node_rcs.append([r, c])

    routes = []
    for route in schedule.routes:
        routes.append([{"node": s.node, "op": s.op.value} for s in route])

    return {
        "routes":     routes,
        "depot_rc":   depot_rc,
        "node_rcs":   node_rcs,
        "dwell_time": dwell_time,
    }


def publish_schedule(
    instance,
    schedule,
    dwell_time: float = 0.0,
) -> None:
    """
    Publish a solved schedule to the running factory node via ROS2.

    Requires rclpy to be available and a ROS2 environment to be sourced.
    Safe to call from a background thread (initialises its own rclpy context).
    """
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("[ros_dispatch] rclpy not available — ROS2 dispatch skipped.", file=sys.stderr)
        return

    payload      = build_dispatch_payload(instance, schedule, dwell_time)
    payload_json = json.dumps(payload)
    command      = f"DISPATCH {payload_json}"

    rclpy.init()
    node = Node('vrp_dispatch_bridge')
    pub  = node.create_publisher(String, '/agv_factory/command', 10)

    # Give DDS a moment to discover the factory node subscriber
    import time
    time.sleep(0.5)

    msg      = String()
    msg.data = command
    pub.publish(msg)

    node.get_logger().info("Schedule dispatched to /agv_factory/command")
    time.sleep(0.3)   # let DDS flush

    node.destroy_node()
    rclpy.shutdown()


def _cli() -> None:
    parser = argparse.ArgumentParser(
        description="Dispatch a VRP-RPD schedule to the AGV factory node via ROS2.")
    parser.add_argument("--instance", required=True,
                        help="Path to instance JSON file")
    parser.add_argument("--schedule", required=True,
                        help="Path to schedule JSON file")
    parser.add_argument("--dwell", type=float, default=0.0,
                        help="Dwell time in seconds at each stop (default 0)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the dispatch payload without sending")
    args = parser.parse_args()

    # Import here so the module works even without a full ROS2 install
    # when only build_dispatch_payload is needed.
    sys.path.insert(0, str(Path(__file__).parent))
    from instance import Instance
    from schedule import Schedule

    inst  = Instance.load(args.instance)
    sched = Schedule.load(args.schedule)

    payload = build_dispatch_payload(inst, sched, dwell_time=args.dwell)

    if args.dry_run:
        print(json.dumps(payload, indent=2))
        return

    print(f"Dispatching schedule: {len(sched.routes)} AGVs, dwell={args.dwell}s")
    publish_schedule(inst, sched, dwell_time=args.dwell)
    print("Done.")


if __name__ == '__main__':
    _cli()
