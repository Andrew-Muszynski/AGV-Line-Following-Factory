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
    map_xy: Tuple[float, float]
    agv_xy: Tuple[float, float]
    entry_side: str
    dock_heading: str

    @property
    def x_agv(self) -> float:
        return self.agv_xy[0]

    @property
    def y_agv(self) -> float:
        return self.agv_xy[1]


def _pair(values: Iterable[float]) -> Tuple[float, float]:
    x, y = values
    return float(x), float(y)


def load_workstations(path: Path = REGISTRY_PATH) -> List[Workstation]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return [
        Workstation(
            id=str(row["id"]),
            bay=str(row["bay"]),
            map_xy=_pair(row["map_xy"]),
            agv_xy=_pair(row["agv_xy"]),
            entry_side=str(row["entry_side"]),
            dock_heading=str(row["dock_heading"]),
        )
        for row in data["workstations"]
    ]


def index_by_id(path: Path = REGISTRY_PATH) -> Dict[str, Workstation]:
    return {ws.id: ws for ws in load_workstations(path)}


def index_by_bay(path: Path = REGISTRY_PATH) -> Dict[str, Workstation]:
    return {ws.bay: ws for ws in load_workstations(path)}


def get_workstation(name: str, path: Path = REGISTRY_PATH) -> Workstation:
    by_id = index_by_id(path)
    if name in by_id:
        return by_id[name]

    by_bay = index_by_bay(path)
    if name in by_bay:
        return by_bay[name]

    known = ", ".join(sorted([*by_id.keys(), *by_bay.keys()]))
    raise KeyError(f"unknown workstation {name!r}; known values: {known}")


def as_vrp_node_dict(path: Path = REGISTRY_PATH) -> Dict[str, dict]:
    return {
        ws.id: {
            "bay": ws.bay,
            "x": ws.x_agv,
            "y": ws.y_agv,
            "entry_side": ws.entry_side,
            "dock_heading": ws.dock_heading,
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
            "map_xy": ws.map_xy,
            "agv_xy": ws.agv_xy,
            "entry_side": ws.entry_side,
            "dock_heading": ws.dock_heading,
        }
    else:
        payload = as_vrp_node_dict()

    if args.json:
        print(json.dumps(payload, indent=2))
        return

    if args.name:
        print(
            f"{payload['id']} {payload['bay']} "
            f"agv_xy={payload['agv_xy']} entry={payload['entry_side']} dock={payload['dock_heading']}"
        )
    else:
        for ws_id, row in payload.items():
            print(
                f"{ws_id} {row['bay']} "
                f"agv_xy=({row['x']:.2f}, {row['y']:.2f}) "
                f"entry={row['entry_side']} dock={row['dock_heading']}"
            )


if __name__ == "__main__":
    main()
