#!/usr/bin/env python3
"""Measure real table positions from parked robots.

Handles three kinds of target, each backed by its own table:
  workstation bays   65..113 on an 8x8   -> WORKSTATION_WORLD_IN
  depot slots        D1, D2, ...         -> DEPOT_SLOT_WORLD_IN
  depot entries      DE1, DE2, ...       -> DEPOT_ENTRY_WORLD_IN

Grid ENTRY nodes (the bay mouths, 114..162) and lattice nodes are
refused -- they define the lattice everything else hangs off. Those
are a different thing from DEPOT entries (DE1..DEn), which are real
measured positions and are calibratable.

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
import re
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
    """Print the measured tables as Python literals.

    Emits only the sections that actually hold measurements, so running
    this after calibrating just the depots does not print an empty
    workstation table. Paste each printed block over the matching table in
    BOTH apriltag_localize.py and camera_grid_navigate.py -- they run on
    different machines and cannot import each other, so both must carry
    the same values or the localiser and the navigator will disagree about
    where things are."""
    with open(path, encoding="utf-8") as fh:
        blob = json.load(fh)
    rows, cols = blob["rows"], blob["cols"]
    nodes_total = rows * cols

    print(f"# Measured positions, world inches ({rows}x{cols} grid).")
    print(f"# Source: {os.path.basename(path)}, "
          f"updated {blob.get('updated_utc', '?')}")
    print("# Regenerate with: calibrate_workstations.py --emit-python")
    print("# Paste into BOTH apriltag_localize.py and "
          "camera_grid_navigate.py.")

    for kind, (section, table) in SECTION_BY_KIND.items():
        recs = blob.get(section) or {}
        if not recs:
            continue
        print()
        if kind == "workstation":
            print(f"{table}: dict[int, tuple[float, float]] = {{")
            for n in sorted(int(k) for k in recs):
                rec = recs[str(n)]
                b = (n - nodes_total) - 1
                row_up, c = b // (cols - 1), b % (cols - 1)
                print(f"    {n}: ({rec['x_in']:.2f}, {rec['y_in']:.2f}),"
                      f"  # row {row_up} col {c}, {rec.get('robot', '?')}")
        else:
            print(f"{table}: dict[str, tuple[float, float]] = {{")
            for label in sorted(
                    recs, key=lambda k: int("".join(
                        ch for ch in k if ch.isdigit()) or 0)):
                rec = recs[label]
                print(f'    "{label}": ({rec["x_in"]:.2f}, '
                      f'{rec["y_in"]:.2f}),'
                      f'  # {rec.get("robot", "?")}')
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


# Three kinds of thing this tool can measure, each backed by its own table
# in BOTH apriltag_localize.py and camera_grid_navigate.py.
SECTION_BY_KIND = {
    "workstation": ("nodes", "WORKSTATION_WORLD_IN"),
    "depot_slot": ("depot_slots", "DEPOT_SLOT_WORLD_IN"),
    "depot_entry": ("depot_entries", "DEPOT_ENTRY_WORLD_IN"),
}
_DEPOT_SLOT_RE = re.compile(r"^D(\d+)$")
_DEPOT_ENTRY_RE = re.compile(r"^DE(\d+)$")


def classify_target(label: str, rows: int, cols: int) -> str:
    """"65" -> workstation, "D3" -> depot_slot, "DE3" -> depot_entry.

    Grid ENTRY nodes (the bay mouths, 114..162 on an 8x8) and lattice
    nodes are refused -- they define the lattice the bays hang off, and
    the routing/reservation model assumes they are exactly on it. Note
    those are a different thing from DEPOT entries (DE1..DEn), which are
    real measured positions and perfectly legitimate to calibrate."""
    if _DEPOT_ENTRY_RE.match(label):
        return "depot_entry"
    if _DEPOT_SLOT_RE.match(label):
        return "depot_slot"
    try:
        n = int(label)
    except ValueError:
        raise SystemExit(
            f"{label!r} is not a workstation node number, a depot slot "
            "(D1, D2, ...) or a depot entry (DE1, DE2, ...)")
    if not is_workstation_node(n, rows, cols):
        raise SystemExit(
            f"node {n} is not a WORKSTATION node on a {rows}x{cols} grid. "
            "Grid entry nodes and lattice nodes are deliberately not "
            "calibratable -- they define the grid the bays hang off. See "
            "WORKSTATION_WORLD_IN in camera_grid_navigate.py. (Depot "
            "entries are written DE1..DEn and ARE calibratable.)")
    return "workstation"


def reference_position(label: str, rows: int,
                       cols: int) -> tuple[float, float] | None:
    """Currently-configured position, or None if this is a brand-new
    target (e.g. a D7 slot that no table knows about yet)."""
    try:
        return node_to_world(label, rows, cols)
    except (KeyError, ValueError):
        return None


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
                     cols: int) -> dict[str, str]:
    """"Alvik1:65" -> {"Alvik1": "65"}; also accepts D1 / DE1 labels."""
    out: dict[str, str] = {}
    for tok in tokens:
        if ":" not in tok:
            raise SystemExit(f"--assign expects ROBOT:TARGET, got {tok!r}")
        robot, target = tok.split(":", 1)
        robot, target = robot.strip(), target.strip()
        classify_target(target, rows, cols)   # raises on anything invalid
        if robot in out:
            raise SystemExit(f"{robot} assigned twice")
        if target in out.values():
            raise SystemExit(f"target {target} assigned twice")
        out[robot] = target
    if not out:
        raise SystemExit("--assign requires at least one ROBOT:TARGET pair")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Measure workstation node positions from parked robots.")
    ap.add_argument("--emit-python", action="store_true",
                    help="print the measured tables as Python literals and "
                         "exit -- paste into BOTH apriltag_localize.py and "
                         "camera_grid_navigate.py")
    ap.add_argument("--assign", nargs="+", metavar="ROBOT:TARGET",
                    help="ROBOT:TARGET pairs. TARGET is a workstation node "
                         "number (65..113 on an 8x8), a depot slot (D1, D2, "
                         "...) or a depot entry (DE1, DE2, ...). e.g. "
                         "--assign Alvik1:65 Alvik2:D2 Alvik3:DE3. Grid "
                         "entry nodes and lattice nodes are refused.")
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
            + ", ".join(f"{r}->{assignment[r]}" for r in robots))
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.collect_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(len(node.samples[r]) >= args.samples for r in robots):
                break

        results: dict[str, dict] = {}
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
                    f"{r} ({assignment[r]}): samples spread "
                    f"{rec['spread_in']:.3f}in > {args.max_spread_in:.2f}in "
                    "-- robot was moving, not parked")
                continue
            rec["robot"] = r
            results[assignment[r]] = rec

        # Report measured vs currently-configured. That delta is the
        # whole point: it shows what the existing tables were wrong about,
        # and flags brand-new targets (a D7 no table knows yet) as "new"
        # rather than silently comparing against nothing.
        def _sortkey(label: str):
            kind = classify_target(label, args.rows, args.cols)
            order = {"workstation": 0, "depot_slot": 1, "depot_entry": 2}
            digits = "".join(ch for ch in label if ch.isdigit())
            return (order[kind], int(digits or 0))

        print()
        print(f"{'target':>7}  {'kind':<12} {'robot':<8} "
              f"{'measured x,y':>16} {'current x,y':>16} "
              f"{'dx':>7} {'dy':>7} {'dist':>7} {'spread':>7}")
        print("-" * 104)
        for label in sorted(results, key=_sortkey):
            rec = results[label]
            kind = classify_target(label, args.rows, args.cols)
            ref = reference_position(label, args.rows, args.cols)
            head = (f"{label:>7}  {kind:<12} {rec['robot']:<8} "
                    f"{rec['x_in']:>7.2f},{rec['y_in']:>7.2f} ")
            if ref is None:
                print(head + f"{'(new)':>16} {'-':>7} {'-':>7} {'-':>7} "
                             f"{rec['spread_in']:>7.3f}")
            else:
                dx, dy = rec["x_in"] - ref[0], rec["y_in"] - ref[1]
                print(head + f"{ref[0]:>7.2f},{ref[1]:>7.2f} "
                             f"{dx:>+7.2f} {dy:>+7.2f} "
                             f"{math.hypot(dx, dy):>7.2f} "
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

        # Merge, so measuring a row (or a depot) at a time keeps earlier
        # work. Each kind lands in its own section, mirroring the three
        # separate tables the values end up in.
        blob = {"rows": args.rows, "cols": args.cols, "units": "inches"}
        for section, _table in SECTION_BY_KIND.values():
            blob[section] = {}
        try:
            with open(out_path, encoding="utf-8") as fh:
                existing = json.load(fh)
            if (existing.get("rows") == args.rows
                    and existing.get("cols") == args.cols):
                for section, _table in SECTION_BY_KIND.values():
                    blob[section] = existing.get(section) or {}
            else:
                print(f"\nnote: {out_path} was recorded for a "
                      f"{existing.get('rows')}x{existing.get('cols')} grid "
                      "-- starting fresh rather than mixing grids")
        except (OSError, ValueError):
            pass

        stamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
        for label, rec in results.items():
            rec["measured_utc"] = stamp
            section, _table = SECTION_BY_KIND[
                classify_target(label, args.rows, args.cols)]
            blob[section][label] = rec
        blob["updated_utc"] = stamp

        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(blob, fh, indent=2, sort_keys=True)
            fh.write("\n")
        bays = (args.rows - 1) * (args.cols - 1)
        print(f"\nwrote {len(results)} target(s) to {out_path}")
        print("NOTE: this file is the record, not the live values. Run "
              "--emit-python and paste the")
        print("      literals into BOTH apriltag_localize.py and "
              "camera_grid_navigate.py to take effect.")
        print(f"  workstations : {len(blob['nodes'])}/{bays}")
        print(f"  depot slots  : {len(blob['depot_slots'])}")
        print(f"  depot entries: {len(blob['depot_entries'])}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
