#!/usr/bin/env python3
"""Workstation registry for AGV routing and VRP-RPD dispatch."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


REGISTRY_PATH = Path(__file__).with_name("workstations.json")


@dataclass(frozen=True)
class Workstation:
    id: str
    bay: str
    row: int
    col: int
    between_nodes: Tuple[int, int]
    map_xy: Tuple[float, float]
    agv_xy: Tuple[float, float]
    access_node: str
    dock_node: str
    entry_side: str
    dock_heading: str

    @property
    def x_agv(self) -> float:
        return self.agv_xy[0]

    @property
    def y_agv(self) -> float:
        return self.agv_xy[1]

    @property
    def node_a(self) -> int:
        return self.between_nodes[0]

    @property
    def node_b(self) -> int:
        return self.between_nodes[1]

    @property
    def segment_start_node(self) -> int:
        return min(self.between_nodes)

    @property
    def segment_end_node(self) -> int:
        return max(self.between_nodes)

    @property
    def node_grid_row(self) -> int:
        return ((self.segment_start_node - 1) // 8) + 1

    @property
    def node_grid_col(self) -> int:
        return ((self.segment_start_node - 1) % 8) + 1

    @property
    def northbound_red_count(self) -> int:
        """Red markers from depot column before turning east to the segment row."""
        return self.node_grid_row - 1

    @property
    def eastbound_red_count(self) -> int:
        """Red markers after the east turn before searching the yellow entry."""
        return self.node_grid_col - 1

    @property
    def westbound_red_count(self) -> int:
        """Red markers from the entry segment back to the depot column."""
        return self.node_grid_col

    def validate_segment(self) -> None:
        a, b = self.between_nodes
        if not (1 <= a <= 64 and 1 <= b <= 64):
            raise ValueError(f"{self.id} between_nodes must be in 1..64: {self.between_nodes}")
        if abs(a - b) != 1:
            raise ValueError(f"{self.id} must be between adjacent horizontal nodes: {self.between_nodes}")
        if ((a - 1) // 8) != ((b - 1) // 8):
            raise ValueError(f"{self.id} nodes must be on the same grid row: {self.between_nodes}")


def _pair(values: Iterable[float]) -> Tuple[float, float]:
    x, y = values
    return float(x), float(y)


def _int_pair(values: Iterable[int]) -> Tuple[int, int]:
    a, b = values
    return int(a), int(b)


def normalize_workstation_name(name: str) -> str:
    text = name.strip().upper()
    if text.isdigit():
        return f"WS{int(text):02d}"
    if text.startswith("WS") and text[2:].isdigit():
        return f"WS{int(text[2:]):02d}"
    return text


def load_workstations(path: Path = REGISTRY_PATH) -> List[Workstation]:
    data = json.loads(path.read_text(encoding="utf-8"))
    workstations = [
        Workstation(
            id=str(row["id"]),
            bay=str(row["bay"]),
            row=int(row["row"]),
            col=int(row["col"]),
            between_nodes=_int_pair(row["between_nodes"]),
            map_xy=_pair(row["map_xy"]),
            agv_xy=_pair(row["agv_xy"]),
            access_node=str(row["access_node"]),
            dock_node=str(row["dock_node"]),
            entry_side=str(row["entry_side"]),
            dock_heading=str(row["dock_heading"]),
        )
        for row in data["workstations"]
    ]
    for ws in workstations:
        ws.validate_segment()
    return workstations


def index_by_id(path: Path = REGISTRY_PATH) -> Dict[str, Workstation]:
    return {ws.id: ws for ws in load_workstations(path)}


def index_by_bay(path: Path = REGISTRY_PATH) -> Dict[str, Workstation]:
    return {ws.bay: ws for ws in load_workstations(path)}


def get_workstation(name: str, path: Path = REGISTRY_PATH) -> Workstation:
    normalized = normalize_workstation_name(name)
    by_id = index_by_id(path)
    if normalized in by_id:
        return by_id[normalized]

    by_bay = index_by_bay(path)
    if normalized in by_bay:
        return by_bay[normalized]

    known = ", ".join(sorted([*by_id.keys(), *by_bay.keys()]))
    raise KeyError(f"unknown workstation {name!r}; known values: {known}")


def as_vrp_node_dict(path: Path = REGISTRY_PATH) -> Dict[str, dict]:
    return {
        ws.id: {
            "bay": ws.bay,
            "row": ws.row,
            "col": ws.col,
            "between_nodes": list(ws.between_nodes),
            "access_node": ws.access_node,
            "dock_node": ws.dock_node,
            "x": ws.x_agv,
            "y": ws.y_agv,
            "entry_side": ws.entry_side,
            "dock_heading": ws.dock_heading,
            "northbound_red_count": ws.northbound_red_count,
            "eastbound_red_count": ws.eastbound_red_count,
            "westbound_red_count": ws.westbound_red_count,
        }
        for ws in load_workstations(path)
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect workstation registry.")
    parser.add_argument("name", nargs="?", help="workstation id or bay id")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args()

    if args.name:
        ws = get_workstation(args.name)
        payload = {
            "id": ws.id,
            "bay": ws.bay,
            "row": ws.row,
            "col": ws.col,
            "between_nodes": ws.between_nodes,
            "access_node": ws.access_node,
            "dock_node": ws.dock_node,
            "map_xy": ws.map_xy,
            "agv_xy": ws.agv_xy,
            "entry_side": ws.entry_side,
            "dock_heading": ws.dock_heading,
            "northbound_red_count": ws.northbound_red_count,
            "eastbound_red_count": ws.eastbound_red_count,
            "westbound_red_count": ws.westbound_red_count,
        }
    else:
        payload = as_vrp_node_dict()

    if args.json:
        print(json.dumps(payload, indent=2))
        return

    if args.name:
        print(
            f"{payload['id']} {payload['bay']} between={payload['between_nodes']} "
            f"agv_xy={payload['agv_xy']} entry={payload['entry_side']} dock={payload['dock_heading']}"
        )
    else:
        for ws_id, row in payload.items():
            print(
                f"{ws_id} {row['bay']} between={row['between_nodes']} "
                f"agv_xy=({row['x']:.2f}, {row['y']:.2f}) "
                f"red_counts=N{row['northbound_red_count']}/E{row['eastbound_red_count']}/W{row['westbound_red_count']} "
                f"entry={row['entry_side']} dock={row['dock_heading']}"
            )


if __name__ == "__main__":
    main()
