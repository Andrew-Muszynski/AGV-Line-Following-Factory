"""Tests for the DOE traffic-planning package.

    python3 -m pytest -q traffic/test_traffic.py

Grouped by what each guarantee protects:
  * the model IS the page's model (geometry, lanes, timing to the 0.001s);
  * the conflict model catches each kind, and constraints built from it
    really keep a planned robot clear;
  * the single-robot planner is correct on its own;
  * every method preserves the frozen job plan and the fixed launch; the
    coordinated ones are conflict-free; the validator agrees with them;
  * a plan survives the trip to the page's format and back unchanged.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "fleet"))

from traffic import instance as I  # noqa: E402
from traffic.conflicts import (Constraints, Hold, detect_conflicts,  # noqa: E402
                               holds)
from traffic.export import load_page_paths, to_page_plan  # noqa: E402
from traffic.model import Testbed  # noqa: E402
from traffic.plan import time_path  # noqa: E402
from traffic.planners import METHODS, solve, wait_route_nodes  # noqa: E402
from traffic.sipp import plan_agent  # noqa: E402
from traffic.validate import metrics, validate  # noqa: E402

PLANS = ROOT / "generated_plans"
SC_1006 = PLANS / "nearest_2026-10-06_17-00-11.json"   # drop-only, current timing
SC_0909 = PLANS / "nearest_2026-09-09_20-53-58.json"   # drop-only
SC_0930 = PLANS / "ext-regret_30s_2026-09-30_14-41-15.json"   # RPD, picks
TB = Testbed(8, 8)


def need(p: Path):
    if not p.exists():
        pytest.skip(f"{p.name} not present")
    return p


# -- the model is the page's model -------------------------------------------
def test_geometry_matches_the_fleet():
    da = pytest.importorskip("dispatch_advisor")
    for n in list(range(1, 65)) + list(range(65, 163)):
        assert tuple(da.label_to_cell(n, 8, 8)) == pytest.approx(TB.pos[n]), n
    for e in range(114, 163):
        assert tuple(da.entry_edge_nodes(e, 8, 8)) == TB.flanks(e)
    for a, b in ((9, 10), (9, 114), (114, 10), (114, 65), (0, 1), ("DE1", 0)):
        assert da.corridor_key(a, b, 8, 8) == TB.corridor(a, b)


def test_squeeze_partners_are_the_measured_footprint():
    da = pytest.importorskip("dispatch_advisor")
    for e in range(114, 163):
        foot = da.entry_rotation_footprint(e, 8, 8)
        assert {str(x) for x in TB.squeeze_partners(e)} | {str(e)} == foot
    assert TB.squeeze_partners(10) == {114, 115}      # lattice: its two entries
    assert not TB.squeeze_partners(1)                  # bottom row: no entries


def test_timing_reproduces_the_page_to_the_millisecond():
    d = json.loads(need(SC_1006).read_text())
    worst = 0.0
    for r in d["routes"]:
        nodes = r["path_nodes"]
        waits = [0.0] * len(nodes)
        waits[0] += float(r.get("launch_offset_sec") or 0)
        for k, v in (r.get("wait_before_segment_sec") or {}).items():
            waits[int(k) - 1] += float(v)
        p = time_path(TB, r["agv"], nodes, waits, allow_reversal=True)
        for seg in r["segments"]:
            i = seg["segment_index"] - 1
            worst = max(worst, abs(p.steps[i].turn_start - seg["start_sec"]),
                        abs(p.steps[i + 1].arrive - seg["end_sec"]))
    assert worst < 1e-3


def test_return_hop_from_node_0_is_charged_a_turn_like_the_page():
    p = time_path(TB, "x", [2, 1, 0, "DE1"], allow_reversal=True)
    assert p.steps[1].turn == 0.0           # 1 -> 0 is a measured move
    assert p.steps[2].turn == pytest.approx(2.65)   # 0 -> DE1 is not


def test_about_face_is_banned_mid_grid_but_allowed_in_a_bay():
    with pytest.raises(ValueError):
        time_path(TB, "x", [9, 10, 9])
    time_path(TB, "x", [9, 114, 65, 114, 9])          # in and out of a bay


def test_cascade_launch_matches_the_rule():
    pre = I.cascade_prefixes(TB, [1, 2, 3, 4, 5, 6])
    assert pre[1].waits == [0.0] * len(pre[1].steps)  # the leader never waits
    for k in range(2, 7):
        # each follower may not start DE1 -> 0 before the one ahead is at 1
        de1 = pre[k].nodes.index("DE1")
        assert pre[k].steps[de1].drive_start >= pre[k - 1].end - 1e-6
    lane = {k: p for k, p in pre.items()}
    lane_conf = [c for c in detect_conflicts(TB, lane) if c.a.resource != 1]
    assert lane_conf == []


# -- the conflict model ------------------------------------------------------
def test_each_conflict_kind_is_detected():
    a = time_path(TB, "A", [9, 10, 11], [0, 0, 0])
    b = time_path(TB, "B", [3, 11, 10], [0, 0, 0], t0=a.steps[1].arrive)
    kinds = {c.kind for c in detect_conflicts(TB, {"A": a, "B": b})}
    assert "node" in kinds or "lane" in kinds
    # rotation: one robot turns at entry 114 while the other sits on node 9
    sitter = time_path(TB, "S", [1, 9], [0, 0])
    turner = time_path(TB, "T", [10, 114, 65], [0, 0, 0], t0=0.0)
    found = detect_conflicts(TB, {"S": sitter, "T": turner})
    assert any(c.kind == "rotation" for c in found)


def test_constraints_from_a_hold_keep_a_planned_robot_clear():
    from traffic.planners import _base_constraints
    inst = I.load(need(SC_1006))
    for i in range(len(inst.tasks) - 1):
        t1, t2 = inst.tasks[i], inst.tasks[i + 1]
        # t2 first (it must respect t1's fixed launch), then t1 against t2
        other = plan_agent(inst, t2, _base_constraints(inst, t2.name))
        cons = _base_constraints(inst, t1.name)
        cons.add_path(TB, other)
        p = plan_agent(inst, t1, cons)
        assert detect_conflicts(TB, {t1.name: p, t2.name: other}) == []


# -- the single-robot planner ------------------------------------------------
def test_alone_the_planner_is_never_slower_than_the_fixed_route():
    inst = I.load(need(SC_1006))
    for task in inst.tasks:
        sipp = plan_agent(inst, task, Constraints())
        nodes = task.prefix.nodes[:-1] + wait_route_nodes(inst, task)
        fixed = time_path(TB, task.name, nodes, task.prefix.waits[:-1]
                          + [0.0] * (len(nodes) - len(task.prefix.nodes) + 1),
                          prefix=len(task.prefix.steps) - 1,
                          allow_reversal=True)
        assert sipp.end <= fixed.end + 1e-6


def test_the_planner_waits_out_a_blocked_node():
    inst = I.load(need(SC_1006))
    task = inst.tasks[0]
    free = plan_agent(inst, task, Constraints())
    target = free.nodes[free.prefix + 2]
    t = free.steps[free.prefix + 2].arrive
    cons = Constraints()
    cons.node[target].append((t - 5.0, t + 20.0))
    p = plan_agent(inst, task, cons)
    for s in p.steps:
        if s.node == target:
            assert s.drive_start <= t - 5.0 + 1e-6 or s.arrive >= t + 20.0 - 1e-6
    assert p.end > free.end


def test_an_entry_is_only_a_doorway():
    inst = I.load(need(SC_0909))
    for m in ("astar", "pp", "cbs"):
        r = solve(inst, m, time_limit=60)
        for p in r.paths.values():
            for i in range(1, len(p.steps) - 1):
                if TB.kind(p.steps[i].node) == "entry":
                    nb = {TB.kind(p.steps[i - 1].node), TB.kind(p.steps[i + 1].node)}
                    assert "workstation" in nb, (m, p.agent, p.nodes)


# -- the methods -------------------------------------------------------------
@pytest.mark.parametrize("scenario", [SC_1006, SC_0909, SC_0930])
@pytest.mark.parametrize("method", METHODS)
def test_every_method_keeps_the_frozen_jobs_and_the_launch(scenario, method):
    inst = I.load(need(scenario))
    r = solve(inst, method, time_limit=60)
    if not r.paths:
        pytest.skip(f"{method} produced no plan within budget: {r.note}")
    rep = validate(inst, r.paths)
    assert rep["errors"] == [], rep["errors"]
    assert rep["readiness"] == []
    assert set(r.paths) == {t.name for t in inst.tasks}


@pytest.mark.parametrize("scenario", [SC_1006, SC_0909, SC_0930])
@pytest.mark.parametrize("method", ["pp", "cbs"])
def test_coordinated_methods_are_conflict_free(scenario, method):
    inst = I.load(need(scenario))
    r = solve(inst, method, time_limit=60)
    assert r.success, r.note
    assert validate(inst, r.paths)["valid"]


def test_uncoordinated_astar_leaves_conflicts_and_is_the_fastest_plan():
    inst = I.load(need(SC_0909))
    a = solve(inst, "astar")
    assert detect_conflicts(TB, a.paths)
    span = max(p.end for p in a.paths.values())
    for m in ("wait", "pp", "cbs"):
        r = solve(inst, m, time_limit=60)
        assert max(p.end for p in r.paths.values()) >= span - 1e-6


def test_wait_reports_what_it_could_not_separate():
    inst = I.load(need(SC_0930))
    r = solve(inst, "wait")
    left = len(detect_conflicts(TB, r.paths))
    assert r.success == (left == 0)
    assert r.stats["conflicts_left"] == left


def test_metrics_split_waiting_into_traffic_service_and_readiness():
    inst = I.load(need(SC_0930))
    r = solve(inst, "pp")
    m = metrics(inst, r.paths, validate(inst, r.paths))
    services = sum(1 for t in inst.tasks for v in t.visits if v["mode"] == "service")
    assert m["service_wait_sec"] == pytest.approx(services * inst.process_sec, abs=0.1)
    assert m["traffic_wait_sec"] >= -1e-6


# -- the road to the page ----------------------------------------------------
def test_export_round_trip_is_exact():
    inst = I.load(need(SC_0909))
    r = solve(inst, "pp")
    plan = to_page_plan(inst, r.paths, "pp", validate(inst, r.paths))
    assert plan["traffic_method"] == "pp"
    back = load_page_paths(inst, json.loads(json.dumps(plan)))
    for name, p in r.paths.items():
        q = back[name]
        assert q.nodes == p.nodes
        assert all(abs(a.arrive - b.arrive) < 1e-6 for a, b in zip(p.steps, q.steps))


def test_the_page_import_branch_exists_and_takes_routes_as_planned():
    js = (ROOT / "static" / "js" / "live-monitor.js").read_text(encoding="utf-8")
    assert "function importTrafficPlan(" in js
    assert "return importTrafficPlan(data, sourceName);" in js
    body = js.split("function importTrafficPlan(")[1].split("\nfunction ")[0]
    assert "polishCollisionPlan(" not in body and "buildAgentRoute(" not in body
    assert "applyRouteTiming(route)" in body


# -- Run & Monitor: a traffic planner per run ------------------------------
def test_app_endpoint_plans_the_page_plan_with_each_method(tmp_path):
    appmod = pytest.importorskip("app")
    client = appmod.create_app(str(tmp_path / "t.sqlite3")).test_client()
    assert client.get("/api/traffic/methods").get_json()["methods"] == list(METHODS)
    plan = json.loads(need(SC_1006).read_text())
    for m in METHODS:
        d = client.post("/api/traffic/plan",
                        json={"method": m, "plan": plan}).get_json()
        assert d["ok"] and d["plan"]["traffic_method"] == m
        assert d["plan"]["routes"][0]["path_nodes"]
        if m in ("pp", "cbs"):
            assert d["valid"] and d["conflicts"] == []
    assert client.post("/api/traffic/plan",
                       json={"method": "dijkstra", "plan": plan}).status_code == 400
    assert client.post("/api/traffic/plan",
                       json={"method": "pp"}).status_code == 400


def test_runs_record_and_compare_by_traffic_method(tmp_path):
    appmod = pytest.importorskip("app")
    client = appmod.create_app(str(tmp_path / "t.sqlite3")).test_client()
    for m in ("page", "pp", "cbs"):
        client.post("/api/runs", json={"algorithm": "BRKGA", "drive_mode": "vision",
                                       "robot_count": 6, "traffic_method": m})
    facets = client.get("/api/runs/facets").get_json()["traffic_method"]
    assert {f["value"] for f in facets} == {"page", "pp", "cbs"}
    listed = client.get("/api/runs?traffic_method=pp&outcome=include_nonexecuted"
                        ).get_json()["runs"]
    assert [r["traffic_method"] for r in listed] == ["pp"]
    head = client.get("/api/export/runs.csv?outcome=include_nonexecuted"
                      ).get_data(as_text=True).splitlines()[0]
    assert "traffic_method" in head.split(",")


def test_run_and_monitor_buttons_reach_the_mission_and_the_run_row():
    tpl = (ROOT / "templates" / "index.html").read_text(encoding="utf-8")
    app_js = (ROOT / "static" / "js" / "app.js").read_text(encoding="utf-8")
    live = (ROOT / "static" / "js" / "live-monitor.js").read_text(encoding="utf-8")
    met = (ROOT / "static" / "js" / "metrics.js").read_text(encoding="utf-8")
    for m in ("page", "astar", "wait", "pp", "cbs"):
        assert f'id="traffic-{m}"' in tpl
        assert f'getElementById("traffic-{m}").addEventListener("click", () => setTrafficMethod("{m}"))' in app_js
    assert "async function setTrafficMethod(" in live
    assert 'fetch("/api/traffic/plan"' in live
    assert "traffic_method: effectiveTrafficMethod()," in live      # mission
    assert 'traffic_method: mission.traffic_method || "page"' in met  # run row
    assert 'id="mf-traffic_method"' in met and '"traffic_method"' in met
