"""The independent validator and the DOE's per-plan numbers.

Every method's output goes through the same checks, so a difference in
validity is the method's, not a difference in how it was judged:

  * the launch prefix is exactly the fixed cascade (it may not be re-timed);
  * every bay is visited in the frozen order, and the robot ends in its own
    slot;
  * service stays last the processing time; no pick leaves before its part
    is ready (drop arrival + processing);
  * the moves are real edges, with no about-face mid-grid;
  * conflicts under conflicts.py's model, by kind.

A plan with conflicts is not discarded -- uncoordinated A* has them by
design, and WAIT can be left with some -- they are counted, because they are
what the runtime layer will have to resolve on the floor.
"""
from __future__ import annotations

from collections import Counter

from .conflicts import detect_conflicts
from .instance import Instance, readiness_violations, drop_times


def validate(inst: Instance, paths: dict) -> dict:
    tb = inst.tb
    errors: list[str] = []
    for task in inst.tasks:
        p = paths.get(task.name)
        if p is None:
            errors.append(f"{task.name}: no path")
            continue
        pre = task.prefix
        n = len(pre.nodes)
        if p.nodes[:n] != pre.nodes or any(
                abs(a - b) > 1e-6 for a, b in zip(p.waits[:n - 1], pre.waits[:-1])):
            errors.append(f"{task.name}: launch prefix differs from the cascade")
        gi = 0
        for i in range(p.prefix, len(p.steps)):
            s = p.steps[i]
            if gi < len(task.goals) and s.node == task.goals[gi].node:
                g = task.goals[gi]
                if g.min_dwell and i < len(p.steps) - 1 and \
                        s.drive_start - s.arrive < g.min_dwell - 1e-6:
                    errors.append(f"{task.name}: service at bay {g.bay} lasts "
                                  f"{s.drive_start - s.arrive:.1f}s < "
                                  f"{g.min_dwell:.1f}s")
                gi += 1
        if gi < len(task.goals):
            errors.append(f"{task.name}: reached {gi} of {len(task.goals)} "
                          "goals in order")
        if p.nodes[-1] != f"D{task.slot}":
            errors.append(f"{task.name}: ends at {p.nodes[-1]!r}, not "
                          f"D{task.slot}")
        for i in range(len(p.steps) - 1):
            a, b = p.steps[i].node, p.steps[i + 1].node
            edge = next((e for e in tb.neighbours(a) if e.to == b), None)
            if edge is None:
                errors.append(f"{task.name}: {a!r}->{b!r} is not an edge")
            elif (i > 0 and tb.reverses(p.steps[i].heading_in, edge.heading)
                  and not tb.dead_end(a)):
                errors.append(f"{task.name}: about-face at {a!r}")
    conflicts = detect_conflicts(tb, paths) if paths else []
    ready = readiness_violations(inst, paths) if paths else []
    return {"valid": not errors and not conflicts and not ready,
            "errors": errors, "conflicts": conflicts, "readiness": ready}


def metrics(inst: Instance, paths: dict, report: dict) -> dict:
    """Plan-level numbers for one (scenario, method)."""
    if not paths:
        return {}
    ends = [p.end for p in paths.values()]
    kinds = Counter(c.kind for c in report["conflicts"])
    drops = drop_times(inst, paths)
    by_name = inst.by_name
    wait_total = service = pick_hold = 0.0
    hops = turns90 = turns180 = 0
    drive = 0.0
    for name, p in paths.items():
        task = by_name[name]
        gi = 0
        for i in range(p.prefix, len(p.steps) - 1):
            s = p.steps[i]
            wait_total += s.wait
            if gi < len(task.goals) and s.node == task.goals[gi].node:
                g = task.goals[gi]
                need = g.min_dwell
                if g.mode == "pick" and g.bay in drops and inst.rpd:
                    need = max(need, drops[g.bay] + inst.process_sec - s.arrive)
                need = min(max(0.0, need), s.wait)
                if g.mode == "service":
                    service += need
                else:
                    pick_hold += need
                gi += 1
        hops += len(p.steps) - 1
        drive += sum(s.move for s in p.steps)
        turns90 += sum(1 for s in p.steps if 0 < s.turn < 4.0)
        turns180 += sum(1 for s in p.steps if s.turn >= 4.0)
    return {
        "planned_makespan_sec": round(max(ends), 2),
        "sum_completion_sec": round(sum(ends), 2),
        "conflicts": len(report["conflicts"]),
        "conflicts_node": kinds.get("node", 0),
        "conflicts_lane": kinds.get("lane", 0),
        "conflicts_rotation": kinds.get("rotation", 0),
        "readiness_violations": len(report["readiness"]),
        "job_errors": len(report["errors"]),
        "traffic_wait_sec": round(wait_total - service - pick_hold, 2),
        "service_wait_sec": round(service, 2),
        "pick_ready_wait_sec": round(pick_hold, 2),
        "hops": hops,
        "drive_sec": round(drive, 2),
        "turns_90": turns90,
        "turns_180": turns180,
    }
