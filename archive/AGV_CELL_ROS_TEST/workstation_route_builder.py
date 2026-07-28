#!/usr/bin/env python3
"""Build ROS primitive commands for workstation routes.

The Arduino sketch executes discrete primitives such as RED, R, YENTRY, and
DOCK. This helper translates a workstation node segment, such as WS04 between
nodes 19-20, into the exact primitive sequence to publish over ROS 2.
"""

from __future__ import annotations

import argparse
import json
from typing import Dict, List

from workstation_registry import Workstation, get_workstation


MAX_ARDUINO_SCRIPT_OPS = 192


def red_count_tokens(count: int) -> List[str]:
    """Return tokens for counting consecutive red markers in one direction."""
    if count < 0:
        raise ValueError(f"red marker count cannot be negative: {count}")

    tokens: List[str] = []
    for index in range(count):
        if index > 0:
            tokens.append("CLEAR")
        tokens.append("RED")
    return tokens


def to_station_tokens(ws: Workstation) -> List[str]:
    """Drive from the blue depot to the workstation and dwell there."""
    tokens: List[str] = []

    tokens.extend(red_count_tokens(ws.northbound_red_count))

    if ws.eastbound_red_count == 0:
        tokens.append("R_YSEARCH")
    else:
        tokens.append("R")
        tokens.extend(red_count_tokens(ws.eastbound_red_count))

    tokens.extend(["YENTRY", "R_SPUR", "YWORK", "YAW0", "DOCK", "DWELL5"])
    return tokens


def return_to_depot_tokens(ws: Workstation) -> List[str]:
    """Return from a docked workstation to the blue depot marker."""
    tokens = ["EXIT", "L"]
    tokens.extend(red_count_tokens(ws.westbound_red_count))
    tokens.extend(["L", "BLUE", "YAW0"])
    return tokens


def dropoff_return_tokens(ws: Workstation) -> List[str]:
    """Complete one workstation visit and return to the depot."""
    return [*to_station_tokens(ws), *return_to_depot_tokens(ws)]


def cycle_tokens(ws: Workstation) -> List[str]:
    """Deliver, return, wait for processing, pick up, and return."""
    one_visit = dropoff_return_tokens(ws)
    return [*one_visit, "WAIT30", *one_visit]


def build_tokens(ws: Workstation, mode: str) -> List[str]:
    if mode == "to-station":
        return to_station_tokens(ws)
    if mode == "dropoff-return":
        return dropoff_return_tokens(ws)
    if mode == "cycle":
        return cycle_tokens(ws)
    raise ValueError(f"unknown route mode: {mode}")


def command_from_tokens(tokens: List[str]) -> str:
    return "run " + ",".join(tokens)


def ros2_command(command: str, robot: str) -> str:
    topic = f"/{robot}_cmd"
    return (
        "ros2 topic pub --once --qos-reliability best_effort "
        f"{topic} std_msgs/msg/String \"{{data: '{command}'}}\""
    )


def route_summary(ws: Workstation, tokens: List[str], robot: str) -> Dict[str, object]:
    command = command_from_tokens(tokens)
    warnings: List[str] = []

    if len(tokens) > MAX_ARDUINO_SCRIPT_OPS:
        warnings.append(
            f"token count {len(tokens)} exceeds Arduino MAX_SCRIPT_OPS={MAX_ARDUINO_SCRIPT_OPS}"
        )

    return {
        "workstation": ws.id,
        "bay": ws.bay,
        "between_nodes": list(ws.between_nodes),
        "segment_start_node": ws.segment_start_node,
        "segment_end_node": ws.segment_end_node,
        "node_grid_row": ws.node_grid_row,
        "node_grid_col": ws.node_grid_col,
        "red_counts": {
            "northbound": ws.northbound_red_count,
            "eastbound": ws.eastbound_red_count,
            "westbound": ws.westbound_red_count,
        },
        "token_count": len(tokens),
        "tokens": tokens,
        "command": command,
        "ros2_command": ros2_command(command, robot),
        "warnings": warnings,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate primitive ROS commands from workstation node segments."
    )
    parser.add_argument("workstation", help="workstation id, number, or bay id")
    parser.add_argument(
        "--mode",
        choices=("to-station", "dropoff-return", "cycle"),
        default="dropoff-return",
        help="route pattern to generate",
    )
    parser.add_argument("--robot", default="Alvik1", help="robot topic prefix")
    parser.add_argument(
        "--format",
        choices=("tokens", "command", "ros2", "json"),
        default="ros2",
        help="output format",
    )
    args = parser.parse_args()

    ws = get_workstation(args.workstation)
    tokens = build_tokens(ws, args.mode)
    summary = route_summary(ws, tokens, args.robot)

    if args.format == "tokens":
        print(",".join(tokens))
    elif args.format == "command":
        print(summary["command"])
    elif args.format == "ros2":
        print(summary["ros2_command"])
    else:
        print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
