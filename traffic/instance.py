"""A DOE scenario: frozen job allocation + the fixed launch, ready to plan.

The traffic planner is the ONLY thing the DOE varies, so everything upstream
of it is read from a saved plan and never changed:

  * which robot serves which bays, in which order (`visits`);
  * the launch: the rolling cascade (solver.js applyCascadeSchedule), ported
    exactly, so every method starts from the same timed prefix
    D# -> DE# -> ... -> DE1 -> 0 -> 1.

Accepts either saved-plan shape the project produces:
  * the page's export (generated_plans/, runs/*.json): routes[].visits;
  * plan_server's payload (solved_plans/*.json): visits{"AlvikN": [...]}.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .model import Testbed
from .plan import TimedPath, time_path


@dataclass
class Goal:
    node: object            # workstation node, or the robot's own slot last
    bay: int | None
    mode: str               # 'drop' | 'pick' | 'service' | 'home'
    min_dwell: float = 0.0  # service: the processing time, spent in the bay


@dataclass
class Task:
    name: str               # "Alvik3"
    slot: int               # 3 -> starts and ends in D3
    visits: list            # [{"bay": b, "mode": m}] after the service merge
    goals: list
    prefix: TimedPath


@dataclass
class Instance:
    tb: Testbed
    tasks: list
    rpd: bool
    process_sec: float
    name: str = ""
    source: dict = field(default_factory=dict)

    @property
    def by_name(self) -> dict:
        return {t.name: t for t in self.tasks}


def merge_service_visits(visits: list) -> list:
    """solver.js mergeServiceVisits(): an adjacent drop -> pick of the same
    bay is ONE stay ('service'), or the robot leaves the bay and has to
    about-face mid-grid to come back."""
    out = []
    i = 0
    while i < len(visits):
        v = dict(visits[i])
        nxt = visits[i + 1] if i + 1 < len(visits) else None
        if (nxt is not None and v.get("mode", "drop") == "drop"
                and nxt.get("mode") == "pick" and nxt["bay"] == v["bay"]):
            out.append({"bay": v["bay"], "mode": "service"})
            i += 2
            continue
        v.setdefault("mode", "drop")
        out.append(v)
        i += 1
    return out


def _is_lane(label) -> bool:
    return label == 0 or (isinstance(label, str) and label.startswith("DE"))


def cascade_prefixes(tb: Testbed, slots: list) -> dict:
    """Port of applyCascadeSchedule(): everyone pulls out D# -> DE# at once;
    every later lane hop waits until the robot ahead has ARRIVED at the node
    beyond that hop's target. Robots enter the grid in slot order."""
    out = {}
    ahead: dict | None = None
    for k in sorted(slots):
        nodes = [f"D{k}"] + [f"DE{j}" for j in range(k, 0, -1)] + [0, 1]
        natural = time_path(tb, f"Alvik{k}", nodes)
        waits = [0.0] * len(nodes)
        delay = 0.0
        mine: dict = {}
        for i in range(len(nodes) - 1):
            frm, to = nodes[i], nodes[i + 1]
            if not _is_lane(frm):
                continue                      # D# -> DE# is never gated
            gate = 0.0
            if ahead is not None and i + 2 < len(nodes):
                gate = ahead.get(nodes[i + 2], 0.0)
            start = natural.steps[i].drive_start + delay
            extra = max(0.0, gate - start)
            if extra > 0:
                waits[i] += extra
                delay += extra
            mine[to] = natural.steps[i + 1].arrive + delay
            if not _is_lane(to):
                break
        prefix = time_path(tb, f"Alvik{k}", nodes, waits, prefix=len(nodes))
        out[k] = prefix
        ahead = mine
    return out


def load(path: str | Path, rows: int | None = None, cols: int | None = None
         ) -> Instance:
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    return load_data(data, path.stem, rows, cols, origin=str(path))


def load_data(data: dict, name: str = "plan", rows: int | None = None,
              cols: int | None = None, origin: str = "") -> Instance:
    """Same as load(), from an already-parsed plan -- e.g. the page's export
    payload POSTed to app.py's /api/traffic/plan."""
    path = origin or name
    grid = data.get("grid") or {}
    rows = rows or int(grid.get("rows", 8))
    cols = cols or int(grid.get("cols", 8))

    visits_by_name: dict[str, list] = {}
    if isinstance(data.get("routes"), list) and data["routes"] and \
            "visits" in data["routes"][0]:
        for r in data["routes"]:
            name = r.get("agv") or f"Alvik{r['agent']}"
            visits_by_name[name] = list(r.get("visits") or [])
    elif isinstance(data.get("visits"), dict):
        for name, seq in data["visits"].items():
            visits_by_name[name] = list(seq)
    else:
        raise ValueError(f"{path}: no routes[].visits or visits{{}} -- not a "
                         "saved plan this loader understands")

    rpd = data.get("rpd", {})
    rpd_on = bool(rpd.get("enabled", True)) if isinstance(rpd, dict) else True
    process = float((rpd or {}).get("process_sec",
                                    data.get("process_sec", 15.0))
                    if isinstance(rpd, dict) else data.get("process_sec", 15.0))
    slots = sorted(int(n.replace("Alvik", "")) for n in visits_by_name)
    tb = Testbed(rows, cols, slots=max(6, max(slots)))
    prefixes = cascade_prefixes(tb, slots)

    tasks = []
    for k in slots:
        name = f"Alvik{k}"
        merged = merge_service_visits(visits_by_name[name])
        goals = []
        for v in merged:
            bay = int(v["bay"])
            mode = v.get("mode", "drop")
            dwell = process if (rpd_on and mode == "service") else 0.0
            goals.append(Goal(tb.ws_of(bay), bay, mode, dwell))
        goals.append(Goal(f"D{k}", None, "home"))
        tasks.append(Task(name, k, merged, goals, prefixes[k]))
    return Instance(tb, tasks, rpd_on, process, name=name, source=data)


def drop_times(inst: Instance, paths: dict) -> dict:
    """bay -> arrival at its workstation for the drop that makes it ready
    (solver.js pickupReadyViolations: drop or service arrival)."""
    ready: dict[int, float] = {}
    for task in inst.tasks:
        p = paths.get(task.name)
        if p is None:
            continue
        gi = 0
        for s in p.steps[p.prefix:]:
            if gi < len(task.goals) - 1 and s.node == task.goals[gi].node:
                g = task.goals[gi]
                if g.mode in ("drop", "service"):
                    ready[g.bay] = s.arrive
                gi += 1
    return ready


def readiness_violations(inst: Instance, paths: dict) -> list:
    """(agent, step index, seconds short) for every pick/service that leaves
    its workstation before the part is ready."""
    if not inst.rpd:
        return []
    drops = drop_times(inst, paths)
    out = []
    for task in inst.tasks:
        p = paths.get(task.name)
        if p is None:
            continue
        gi = 0
        for i in range(p.prefix, len(p.steps)):
            s = p.steps[i]
            if gi < len(task.goals) - 1 and s.node == task.goals[gi].node:
                g = task.goals[gi]
                if g.mode in ("pick", "service") and g.bay in drops:
                    ready = drops[g.bay] + inst.process_sec
                    if s.drive_start < ready - 1e-6 and i < len(p.steps) - 1:
                        out.append((task.name, i, ready - s.drive_start))
                gi += 1
    return out
