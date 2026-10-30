"""Write a planned result in the page's own export format.

The output is what agv_grid_workstation_solver.html's downloadJson() writes,
plus `traffic_method`. The page's "Import plan" recognises that field and
takes the routes exactly as planned -- path_nodes and
wait_before_segment_sec -- instead of rebuilding L-shaped routes and running
its own WAIT over them. From there "Generate commands" and "Send mission"
work unchanged, so a DOE plan reaches the robots by the same road as today.

Also reads a page export back into timed paths (load_page_paths), so the
incumbent's plans can be judged by the same validator as everything else.
"""
from __future__ import annotations

import json
from pathlib import Path

from .instance import Instance
from .model import Testbed
from .plan import TimedPath, time_path


def segment_type(tb: Testbed, a, b) -> str:
    """solver.js movementProfile() type names."""
    ka, kb = tb.kind(a), tb.kind(b)
    if ka == "slot" and kb == "lane":
        return "DEPOT_SLOT_TO_ENTRY"
    if ka == "lane" and kb == "lane":
        return "DEPOT_ENTRY_TO_ENTRY"
    if ka == "lane" and kb == "node0":
        return "DEPOT_ENTRY_TO_NODE_0"
    if a == 0 and b == 1:
        return "BLUE_TO_RED"
    if a == 1 and b == 0:
        return "RED_TO_BLUE_RETURN"
    if ka == "node0" and kb == "lane":
        return "NODE_0_TO_DEPOT_ENTRY"
    if ka == "lane" and kb == "slot":
        return "DEPOT_ENTRY_TO_SLOT"
    if ka == "grid" and kb == "grid":
        return "RED_TO_RED"
    if ka == "grid" and kb == "entry":
        return "RED_TO_YELLOW"
    if ka == "entry" and kb == "grid":
        return "YELLOW_TO_RED"
    return "YELLOW_TO_YELLOW"


def to_page_plan(inst: Instance, paths: dict, method: str,
                 report: dict | None = None, row: dict | None = None) -> dict:
    tb = inst.tb
    src = inst.source
    routes = []
    for task in inst.tasks:
        p: TimedPath = paths[task.name]
        waits = {str(i + 1): round(s.wait, 3)
                 for i, s in enumerate(p.steps[:-1]) if s.wait > 1e-9}
        segments = []
        for i, s in enumerate(p.steps[:-1]):
            nxt = p.steps[i + 1]
            segments.append({
                "segment_index": i + 1, "from": s.node, "to": nxt.node,
                "start_sec": round(s.turn_start, 3),
                "end_sec": round(nxt.arrive, 3),
                "wait_before_sec": round(s.wait, 3),
                "move_sec": s.move, "turn_sec": s.turn,
                "type": segment_type(tb, s.node, nxt.node)})
        routes.append({
            "agent": task.slot, "agv": task.name,
            "bays": [v["bay"] for v in task.visits],
            "visits": [{"bay": v["bay"], "mode": v["mode"]} for v in task.visits],
            "manual_waypoints": [], "launch_offset_sec": 0,
            "extra_wait_sec": 0,
            "dwell_wait_sec": round(sum(s.wait for s in p.steps[p.prefix:-1]), 3),
            "path_mode": f"traffic:{method}",
            "wait_before_segment_sec": waits,
            "duration_sec": round(p.end, 3),
            "path_nodes": p.nodes,
            "node_events": [{"node": s.node, "path_index": i,
                             "arrival_sec": round(s.arrive, 3),
                             "depart_sec": round(s.turn_start, 3),
                             "dwell_sec": round(s.wait, 3),
                             "turn_sec": s.turn}
                            for i, s in enumerate(p.steps)],
            "segments": segments,
        })
    out = {k: src[k] for k in ("grid", "capacity", "rpd", "depot", "workstations",
                               "collision_window_sec",
                               "edge_collision_window_sec")
           if k in src}
    out.setdefault("grid", {"rows": tb.rows, "cols": tb.cols})
    out["agents"] = len(inst.tasks)
    out.setdefault("rpd", {"enabled": inst.rpd, "process_sec": inst.process_sec})
    out["process_sec"] = inst.process_sec
    out["routes"] = routes
    out["makespan_sec"] = round(max(p.end for p in paths.values()), 3)
    out["traffic_method"] = method
    out["algorithm"] = f"traffic:{method}"
    out["traffic_scenario"] = inst.name
    if report is not None:
        out["traffic_valid"] = report["valid"]
        out["traffic_conflicts"] = [c.describe() for c in report["conflicts"]]
    if row is not None:
        out["traffic_metrics"] = row
    return out


def write_page_plan(path: str | Path, plan: dict) -> None:
    Path(path).write_text(json.dumps(plan, indent=2, default=str),
                          encoding="utf-8")


def load_page_paths(inst: Instance, page_plan: dict) -> dict:
    """Timed paths for a page export (e.g. the incumbent's JS WAIT plan),
    re-timed under this model so the validator compares like with like."""
    tb = inst.tb
    out = {}
    for r in page_plan["routes"]:
        name = r.get("agv") or f"Alvik{r['agent']}"
        nodes = r["path_nodes"]
        waits = [0.0] * len(nodes)
        waits[0] += float(r.get("launch_offset_sec") or 0)
        for k, v in (r.get("wait_before_segment_sec") or {}).items():
            waits[int(k) - 1] += float(v)
        task = inst.by_name.get(name)
        prefix = len(task.prefix.steps) - 1 if task else 0
        out[name] = time_path(tb, name, nodes, waits, prefix=prefix,
                              allow_reversal=True)
    return out
