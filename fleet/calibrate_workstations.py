#!/usr/bin/env python3
"""Measure real workstation (bay) node positions from parked robots.

WHY THIS EXISTS
---------------
camera_grid_navigate.node_to_world() derives every workstation position
from ONE global offset:

    grid_x = c + 0.5
    grid_y = (row_up + 1) - 0.44

and that 0.44 was measured a single time, on bay 1 (node114/node65), on
2026-07-27, then applied to all (rows-1)*(cols-1) bays. The physical table
does not hold that constant -- bays sit at slightly different distances
from their row and slightly off the column midpoint. This tool replaces
the derived numbers with measured ones, one per bay.

WHAT IT MEASURES
----------------
The robot's own AprilTag pose while it is parked where it should sit when
serving that workstation. That is self-consistent by construction: the
same tag centre that drive_leg() steers toward is the thing being
recorded, so a leg driven to a calibrated node puts the robot exactly
where the calibration robot stood.

ENTRY NODES ARE REFUSED. They sit on the lattice row and the routing and
reservation model depends on that. Passing one is an error, not a warning
-- see WORKSTATION_WORLD_IN in camera_grid_navigate.py.

USAGE
-----
Park each robot on its bay, facing its normal service heading, then:

    python3 calibrate_workstations.py \\
        --assign Alvik1:65 Alvik2:73 Alvik3:74 Alvik4:81 Alvik5:86 Alvik6:88

Repeat for the remaining bays; the output file is MERGED, so earlier bays
are kept. Add --dry-run to see the measurements and deltas without
writing anything.

The JSON this writes is the RECORD, not the live values. To apply a
measurement, run --emit-python and paste the printed literal into BOTH
apriltag_localize.py and camera_grid_navigate.py -- they run on different
machines and cannot import each other, so both must carry it.

Requires the same stack as a route: camera bridge, apriltag_localize.py,
micro-ROS agent and rosbridge. It does NOT need fleetSupervisor.py, and
it never sends a command to any robot -- it only listens.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from camera_grid_navigate import (
    is_workstation_node,
    node_to_world,
)
from camera_trajectory import normalize_deg

# The measurement RECORD. Nothing reads this at runtime: the live values are
# the WORKSTATION_WORLD_IN literal embedded in BOTH apriltag_localize.py and
# camera_grid_navigate.py. Those two run on different machines and cannot
# import each other, so a JSON present on one but missing on the other would
# let the localiser and the navigator silently disagree about where every bay
# is -- exactly the class of bug that ends in a collision. The literal is the
# single source of truth; this file is provenance, and the input to
# --emit-python.
CALIBRATION_FILENAME = "workstation_calibration.json"


def emit_python(path: str) -> None:
    """Print the WORKSTATION_WORLD_IN literal for a measurement file.

    Paste the output into BOTH apriltag_localize.py and
    camera_grid_navigate.py, replacing the existing block, then re-run
    the cross-file check so the two cannot drift apart."""
    with open(path, encoding="utf-8") as fh:
        blob = json.load(fh)
    rows, cols = blob["rows"], blob["cols"]
    nodes_total = rows * cols
    recs = blob["nodes"]
    print(f"# Measured {len(recs)} workstation positions, world inches "
          f"({rows}x{cols} grid).")
    print(f"# Source: {os.path.basename(path)}, "
          f"updated {blob.get('updated_utc', '?')}")
    print("# Regenerate with: calibrate_workstations.py --emit-python")
    print("# Paste into BOTH apriltag_localize.py and "
          "camera_grid_navigate.py.")
    print("WORKSTATION_WORLD_IN: dict[int, tuple[float, float]] = {")
    for n in sorted(int(k) for k in recs):
        rec = recs[str(n)]
        b = (n - nodes_total) - 1
        row_up, c = b // (cols - 1), b % (cols - 1)
        print(f"    {n}: ({rec['x_in']:.2f}, {rec['y_in']:.2f}),"
              f"  # row {row_up} col {c}, {rec.get('robot', '?')}")
    print("}")


class WorkstationCalibrator(Node):
    """Listen-only: subscribes to every named robot's vision pose."""

    def __init__(self, robots: list[str]):
        super().__init__("workstation_calibrator")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.samples: dict[str, list[tuple[float, float, float]]] = {
            r: [] for r in robots}
        self.subs = {}
        for r in robots:
            self.subs[r] = self.create_subscription(
                String, f"{r}_vision_pose",
                self._make_cb(r), qos)

    def _make_cb(self, robot: str):
        def _cb(msg: String) -> None:
            try:
                d = json.loads(msg.data)
                self.samples[robot].append(
                    (float(d["x_in"]), float(d["y_in"]),
                     float(d["yaw_deg"])))
            except (ValueError, KeyError, TypeError):
                return
        return _cb

    def publishers_ready(self) -> list[str]:
        return [r for r, s in self.subs.items()
                if s.get_publisher_count() == 0]


def circular_mean_deg(values: list[float]) -> float:
    """Mean heading that does not break across the +/-180 seam."""
    sx = sum(math.sin(math.radians(v)) for v in values)
    sy = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(sx, sy))


def summarize(samples: list[tuple[float, float, float]]) -> dict:
    """Median position (robust to a stray frame) plus a spread measure.

    Spread is the max distance of any sample from the median, i.e. the
    worst single-frame disagreement -- the number that says whether the
    robot was genuinely parked or was still drifting."""
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    yaws = [s[2] for s in samples]
    mx = statistics.median(xs)
    my = statistics.median(ys)
    spread = max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys))
    yaw = circular_mean_deg(yaws)
    yaw_spread = max(abs(normalize_deg(v - yaw)) for v in yaws)
    return {
        "x_in": round(mx, 3),
        "y_in": round(my, 3),
        "yaw_deg": round(normalize_deg(yaw), 2),
        "samples": len(samples),
        "spread_in": round(spread, 3),
        "yaw_spread_deg": round(yaw_spread, 2),
    }


def parse_assignment(tokens: list[str], rows: int,
                     cols: int) -> dict[str, int]:
    """"Alvik1:65" -> {"Alvik1": 65}, rejecting anything not a bay."""
    out: dict[str, int] = {}
    for tok in tokens:
        if ":" not in tok:
            raise SystemExit(
                f"--assign expects ROBOT:NODE, got {tok!r}")
        robot, node_s = tok.split(":", 1)
        robot = robot.strip()
        try:
            node = int(node_s)
        except ValueError:
            raise SystemExit(f"--assign node must be an integer, got {tok!r}")
        if not is_workstation_node(node, rows, cols):
            raise SystemExit(
                f"node {node} is not a WORKSTATION node on a {rows}x{cols} "
                "grid. Entry nodes and lattice nodes are deliberately not "
                "calibratable -- they define the grid the bays hang off. "
                "See WORKSTATION_WORLD_IN in camera_grid_navigate.py.")
        if robot in out:
            raise SystemExit(f"{robot} assigned twice")
        if node in out.values():
            raise SystemExit(f"node {node} assigned twice")
        out[robot] = node
    if not out:
        raise SystemExit("--assign requires at least one ROBOT:NODE pair")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Measure workstation node positions from parked robots.")
    ap.add_argument("--emit-python", action="store_true",
                    help="print the WORKSTATION_WORLD_IN literal for the "
                         "existing measurement file and exit -- paste it into "
                         "BOTH apriltag_localize.py and "
                         "camera_grid_navigate.py")
    ap.add_argument("--assign", nargs="+", metavar="ROBOT:NODE",
                    help="e.g. --assign Alvik1:65 Alvik2:73 (workstation "
                         "nodes only; entry nodes are refused)")
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--cols", type=int, default=8)
    ap.add_argument("--samples", type=int, default=60,
                    help="pose samples to collect per robot (default 60, "
                         "~1s at 60Hz)")
    ap.add_argument("--max-spread-in", type=float, default=0.25,
                    help="reject a robot whose samples disagree by more than "
                         "this -- it was not actually parked (default 0.25in)")
    ap.add_argument("--discovery-timeout", type=float, default=15.0,
                    help="seconds to wait for every robot's pose publisher "
                         "to be discovered (default 15)")
    ap.add_argument("--collect-timeout", type=float, default=30.0,
                    help="seconds to wait for all samples (default 30)")
    ap.add_argument("--out", default=None,
                    help=f"output file (default: {CALIBRATION_FILENAME} "
                         "next to camera_grid_navigate.py)")
    ap.add_argument("--dry-run", action="store_true",
                    help="measure and report, write nothing")
    args = ap.parse_args()

    default_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                CALIBRATION_FILENAME)
    if args.emit_python:
        emit_python(args.out or default_path)
        return
    if not args.assign:
        raise SystemExit("--assign is required (or use --emit-python)")

    assignment = parse_assignment(args.assign, args.rows, args.cols)
    robots = sorted(assignment)

    out_path = args.out or default_path

    rclpy.init()
    node = WorkstationCalibrator(robots)
    try:
        # Stage 1: DDS discovery. A freshly-created subscription is not
        # instantly matched to apriltag_localize.py's publisher; racing that
        # against a flat data timeout burns the whole budget on discovery
        # (same failure already fixed in camera_grid_navigate's
        # wait_for_vision_ready -- do not collapse these two stages).
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.discovery_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            missing = node.publishers_ready()
            if not missing:
                break
        else:
            raise SystemExit(
                "no vision_pose publisher discovered for: "
                f"{', '.join(node.publishers_ready())}. Is "
                "apriltag_localize.py running and seeing those tags?")

        # Stage 2: collect.
        node.get_logger().info(
            f"collecting {args.samples} samples for {len(robots)} robot(s): "
            + ", ".join(f"{r}->node{assignment[r]}" for r in robots))
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.collect_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(len(node.samples[r]) >= args.samples for r in robots):
                break

        results: dict[int, dict] = {}
        problems: list[str] = []
        for r in robots:
            got = node.samples[r]
            if len(got) < args.samples:
                problems.append(
                    f"{r}: only {len(got)}/{args.samples} samples -- tag not "
                    "visible, or vision is dropping frames")
                continue
            rec = summarize(got[:args.samples])
            if rec["spread_in"] > args.max_spread_in:
                problems.append(
                    f"{r} (node {assignment[r]}): samples spread "
                    f"{rec['spread_in']:.3f}in > {args.max_spread_in:.2f}in "
                    "-- robot was moving, not parked")
                continue
            rec["robot"] = r
            results[assignment[r]] = rec

        # Report measured vs currently-derived, which is the whole point:
        # it shows which bays the single global offset was wrong about.
        print()
        print(f"{'node':>5}  {'robot':<8} {'measured x,y':>16} "
              f"{'derived x,y':>16} {'dx':>7} {'dy':>7} {'dist':>7} "
              f"{'spread':>7}")
        print("-" * 88)
        for n in sorted(results):
            rec = results[n]
            dx_ref, dy_ref = node_to_world(n, args.rows, args.cols)
            dx = rec["x_in"] - dx_ref
            dy = rec["y_in"] - dy_ref
            print(f"{n:>5}  {rec['robot']:<8} "
                  f"{rec['x_in']:>7.2f},{rec['y_in']:>7.2f} "
                  f"{dx_ref:>7.2f},{dy_ref:>7.2f} "
                  f"{dx:>+7.2f} {dy:>+7.2f} {math.hypot(dx, dy):>7.2f} "
                  f"{rec['spread_in']:>7.3f}")
        if problems:
            print()
            print("NOT RECORDED:")
            for p in problems:
                print(f"  - {p}")

        if not results:
            raise SystemExit("\nnothing measured; not writing")
        if args.dry_run:
            print("\n--dry-run: nothing written")
            return

        # Merge, so calibrating six bays at a time keeps the earlier six.
        blob = {"rows": args.rows, "cols": args.cols, "units": "inches",
                "nodes": {}}
        try:
            with open(out_path, encoding="utf-8") as fh:
                existing = json.load(fh)
            if (existing.get("rows") == args.rows
                    and existing.get("cols") == args.cols):
                blob["nodes"] = existing.get("nodes") or {}
            else:
                print(f"\nnote: {out_path} was recorded for a "
                      f"{existing.get('rows')}x{existing.get('cols')} grid "
                      "-- starting fresh rather than mixing grids")
        except (OSError, ValueError):
            pass

        for n, rec in results.items():
            rec["measured_utc"] = datetime.now(timezone.utc).isoformat(
                timespec="seconds")
            blob["nodes"][str(n)] = rec
        blob["updated_utc"] = datetime.now(timezone.utc).isoformat(
            timespec="seconds")

        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(blob, fh, indent=2, sort_keys=True)
            fh.write("\n")
        bays = (args.rows - 1) * (args.cols - 1)
        print(f"\nwrote {len(results)} node(s) to {out_path}")
        print("NOTE: this file is the record, not the live values. Run "
              "--emit-python and paste the")
        print("      literal into BOTH apriltag_localize.py and "
              "camera_grid_navigate.py to take effect.")
        print(f"calibrated {len(blob['nodes'])}/{bays} workstation nodes")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
