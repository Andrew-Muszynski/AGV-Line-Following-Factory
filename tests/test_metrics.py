"""Acceptance tests for the Page-4 metrics backend (spec section 26).

Synthetic event streams exercise the engine + API end to end against a
temporary SQLite file. Items 15 (browser reload) and 16 (Pages 1-3 behave
identically) are UI checks; their server-side halves are covered here
(persistence across app restarts; the served page still contains all four
views and the original markup).
"""
from __future__ import annotations

import os
import sys
import json

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import database                      # noqa: E402
import metrics_engine                # noqa: E402
from app import create_app           # noqa: E402


# ---------------------------------------------------------------------------
# helpers

class Stream:
    """Sequential event builder: auto seq, explicit elapsed_ms."""

    def __init__(self):
        self.events = []
        self._seq = 0

    def ev(self, elapsed_ms, event_type, **fields):
        self._seq += 1
        e = {"seq": self._seq, "elapsed_ms": elapsed_ms,
             "event_type": event_type}
        e.update(fields)
        self.events.append(e)
        return self


@pytest.fixture()
def client(tmp_path):
    app = create_app(db_path=str(tmp_path / "test.sqlite3"))
    app.config["TESTING"] = True
    with app.test_client() as c:
        yield c


def make_run(client, **overrides):
    payload = {"experimental_condition": "test", "algorithm": "built-in",
               "drive_mode": "vision", "robot_count": 2, "job_count": 2,
               "workstation_count": 2, "process_sec": 15, "capacity": 3,
               "random_seed": 42, "planned_makespan_sec": 100.0}
    payload.update(overrides)
    resp = client.post("/api/runs", json=payload)
    assert resp.status_code == 201
    return resp.get_json()["run_id"]


def post_events(client, run_id, stream):
    resp = client.post(f"/api/runs/{run_id}/events/batch",
                       json={"events": stream.events})
    assert resp.status_code == 200, resp.get_json()
    return resp.get_json()


def get_metrics(client, run_id):
    resp = client.get(f"/api/runs/{run_id}/metrics")
    assert resp.status_code == 200
    return resp.get_json()


def value_of(metrics, key, scope_type="run", scope_id=""):
    for v in metrics["values"]:
        if (v["metric_key"] == key and v["scope_type"] == scope_type
                and (v["scope_id"] or "") == scope_id):
            return v
    raise AssertionError(f"metric {key} [{scope_type}:{scope_id}] not found")


def full_two_robot_stream():
    """A clean two-robot, two-job run: 120s makespan, full state coverage."""
    s = Stream()
    s.ev(0, "RUN_STARTED")
    for r in ("Alvik1", "Alvik2"):
        s.ev(0, "ROBOT_REGISTERED", robot_id=r, battery_pct=90.0)
    # Alvik1: launch wait 5s, travel 40s, service 5s, hold 15s, travel 45s,
    # return 10s -> depot at 120s
    s.ev(5000, "ROBOT_STATE_CHANGED", robot_id="Alvik1",
         state_to="PRODUCTIVE_TRAVEL_LOADED")
    s.ev(5000, "JOB_RELEASED", job_id="J1", robot_id="Alvik1",
         workstation_id="WS65")
    s.ev(45000, "ROBOT_STATE_CHANGED", robot_id="Alvik1",
         state_to="DROPOFF_SERVICE")
    s.ev(45000, "PROCESSING_STARTED", job_id="J1", workstation_id="WS65")
    s.ev(50000, "ROBOT_STATE_CHANGED", robot_id="Alvik1",
         state_to="PROCESSING_HOLD")
    s.ev(60000, "DROPOFF_COMPLETED", job_id="J1", robot_id="Alvik1",
         workstation_id="WS65")
    s.ev(60000, "PROCESSING_COMPLETED", job_id="J1", workstation_id="WS65")
    s.ev(65000, "ROBOT_STATE_CHANGED", robot_id="Alvik1",
         state_to="PRODUCTIVE_TRAVEL_LOADED")
    s.ev(110000, "ROBOT_STATE_CHANGED", robot_id="Alvik1",
         state_to="RETURN_REPOSITION")
    s.ev(120000, "DEPOT_ARRIVED", robot_id="Alvik1", yaw_deg=1.5)
    s.ev(120000, "TERMINAL_HEADING_REACHED", robot_id="Alvik1", yaw_deg=1.5)
    # Alvik2: finishes early at 80s (terminal idle until 120s)
    s.ev(8000, "ROBOT_STATE_CHANGED", robot_id="Alvik2",
         state_to="PRODUCTIVE_TRAVEL_LOADED")
    s.ev(8000, "JOB_RELEASED", job_id="J2", robot_id="Alvik2",
         workstation_id="WS66")
    s.ev(40000, "DROPOFF_COMPLETED", job_id="J2", robot_id="Alvik2",
         workstation_id="WS66")
    s.ev(70000, "ROBOT_STATE_CHANGED", robot_id="Alvik2",
         state_to="RETURN_REPOSITION")
    s.ev(80000, "DEPOT_ARRIVED", robot_id="Alvik2", yaw_deg=-2.0)
    s.ev(80000, "ROBOT_STATE_CHANGED", robot_id="Alvik2",
         state_to="TERMINAL_IDLE")
    for r in ("Alvik1", "Alvik2"):
        s.ev(120000, "BATTERY_SAMPLE", robot_id=r, battery_pct=84.0)
    s.ev(120000, "RUN_COMPLETED")
    return s


# ---------------------------------------------------------------------------
# 1. fully successful two-robot run

def test_successful_two_robot_run(client):
    run_id = make_run(client)
    post_events(client, run_id, full_two_robot_stream())
    m = get_metrics(client, run_id)
    assert m["run_status"] == "completed"
    assert value_of(m, "actual_makespan_sec")["value"] == pytest.approx(120.0)
    jc = value_of(m, "job_completion_rate")
    assert jc["value"] == 1.0 and jc["numerator"] == 2 and jc["denominator"] == 2
    assert value_of(m, "mission_completion_success")["value"] == 1
    assert value_of(m, "throughput_jobs_per_min")["value"] == pytest.approx(1.0)
    # battery: 90 -> 84 on both robots
    assert value_of(m, "battery_drop_pct", "robot", "Alvik1")["value"] == \
        pytest.approx(6.0)


# ---------------------------------------------------------------------------
# 2. one failed rotation out of 256 keeps the other 255

def test_one_failed_rotation_of_256(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    t = 1000
    for i in range(255):
        s.ev(t, "ROTATION_ATTEMPT_STARTED", robot_id="Alvik1",
             details={"target_deg": 90.0})
        s.ev(t + 900, "ROTATION_ATTEMPT_COMPLETED", robot_id="Alvik1",
             details={"target_deg": 90.0, "error_deg": 1.0,
                      "duration_sec": 0.9, "within_tolerance": True})
        t += 1000
    s.ev(t, "ROTATION_ATTEMPT_STARTED", robot_id="Alvik1",
         details={"target_deg": 90.0})
    s.ev(t + 900, "ROTATION_ATTEMPT_FAILED", robot_id="Alvik1",
         reason="CONTROL_ERROR", details={"duration_sec": 0.9})
    s.ev(t + 2000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    comp = value_of(m, "rotation_completion_rate")
    assert comp["numerator"] == 255 and comp["denominator"] == 256
    fail = value_of(m, "rotation_failure_rate")
    assert fail["value"] == pytest.approx(1 / 256)
    mae = value_of(m, "yaw_mae_deg")
    assert mae["value"] == pytest.approx(1.0) and mae["valid_n"] == 255
    # failed attempt's error is NOT zero-substituted into the mean
    assert value_of(m, "yaw_mean_signed_error_deg")["value"] == \
        pytest.approx(1.0)


# ---------------------------------------------------------------------------
# 3. a completion outside yaw tolerance keeps its measured error

def test_outside_tolerance_rotation_and_terminal_heading(client):
    run_id = make_run(client, robot_count=1)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(0, "ROBOT_REGISTERED", robot_id="Alvik1", battery_pct=80.0)
    s.ev(1000, "ROTATION_ATTEMPT_STARTED", robot_id="Alvik1",
         details={"target_deg": 180.0})
    s.ev(2000, "ROTATION_ATTEMPT_COMPLETED", robot_id="Alvik1",
         details={"target_deg": 180.0, "error_deg": 12.0,
                  "duration_sec": 1.0, "within_tolerance": False})
    # robot arrives at depot 25 degrees off the required heading
    s.ev(30000, "DEPOT_ARRIVED", robot_id="Alvik1", yaw_deg=25.0)
    s.ev(30000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    # measured error retained, but marked unsuccessful in the tolerance rate
    assert value_of(m, "yaw_mae_deg")["value"] == pytest.approx(12.0)
    assert value_of(m, "rotation_within_tolerance_rate")["value"] == 0.0
    assert value_of(m, "rotation_completion_rate")["value"] == 1.0
    err = value_of(m, "terminal_heading_error_deg", "robot", "Alvik1")
    assert err["value"] == pytest.approx(25.0)
    # completed run, but the terminal heading tolerance was violated
    assert value_of(m, "mission_completion_success")["value"] == 0


# ---------------------------------------------------------------------------
# 4. aborted run: makespan null, observed duration recorded

def test_aborted_run_has_null_makespan(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(45000, "RUN_ABORTED", reason="OPERATOR_ABORT")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert m["run_status"] == "aborted"
    mk = value_of(m, "actual_makespan_sec")
    assert mk["value"] is None and mk["status"] == "missing"
    assert value_of(m, "observed_duration_sec")["value"] == pytest.approx(45.0)


# ---------------------------------------------------------------------------
# 5. three robots waiting 10s each -> 30 robot-seconds of traffic delay

def test_traffic_delay_robot_seconds(client):
    run_id = make_run(client, robot_count=3)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    for r in ("Alvik1", "Alvik2", "Alvik3"):
        s.ev(0, "ROBOT_REGISTERED", robot_id=r)
        s.ev(0, "ROBOT_STATE_CHANGED", robot_id=r,
             state_to="PRODUCTIVE_TRAVEL_LOADED")
        s.ev(20000, "ROBOT_STATE_CHANGED", robot_id=r,
             state_to="TRAFFIC_WAIT")
        s.ev(30000, "ROBOT_STATE_CHANGED", robot_id=r,
             state_to="PRODUCTIVE_TRAVEL_LOADED")
        s.ev(60000, "DEPOT_ARRIVED", robot_id=r, yaw_deg=0.0)
    s.ev(60000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "fleet_traffic_delay_robot_sec")["value"] == \
        pytest.approx(30.0)
    assert value_of(m, "traffic_stop_episodes")["value"] == 3
    assert value_of(m, "mean_traffic_stop_duration_sec")["value"] == \
        pytest.approx(10.0)


# ---------------------------------------------------------------------------
# 6. a robot finishing early accumulates terminal idle time

def test_terminal_idle_accumulates(client):
    run_id = make_run(client)
    post_events(client, run_id, full_two_robot_stream())
    m = get_metrics(client, run_id)
    frac = value_of(m, "state_fraction_pct", "robot", "Alvik2:TERMINAL_IDLE")
    # Alvik2 finished at 80s of a 120s run -> 40s terminal idle = 33.3%
    assert frac["value"] == pytest.approx(100.0 * 40.0 / 120.0, abs=0.5)


# ---------------------------------------------------------------------------
# 7. deadlock detected and automatically recovered

def test_deadlock_resolved(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(10000, "DEADLOCK_DETECTED", conflict_id="DL1",
         details={"robot_ids": ["Alvik1", "Alvik2"]})
    s.ev(18000, "DEADLOCK_RESOLVED", conflict_id="DL1", reason="auto")
    s.ev(60000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "deadlock_count")["value"] == 1
    assert value_of(m, "mean_deadlock_recovery_sec")["value"] == \
        pytest.approx(8.0)
    assert value_of(m, "unresolved_deadlock_count")["value"] == 0


# ---------------------------------------------------------------------------
# 8. an unresolved deadlock stays visible and blocks intervention-free

def test_deadlock_unresolved(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(10000, "DEADLOCK_DETECTED", conflict_id="DL1",
         details={"robot_ids": ["Alvik1", "Alvik2"]})
    s.ev(60000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "unresolved_deadlock_count")["value"] == 1
    rec = value_of(m, "mean_deadlock_recovery_sec")
    assert rec["value"] is None and rec["missing_n"] == 1
    assert value_of(m, "intervention_free_success")["value"] == 0


# ---------------------------------------------------------------------------
# 9. a missing pose interval becomes a dropout episode

def test_vision_dropout_episode(client):
    run_id = make_run(client, robot_count=1)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    t = 0
    while t <= 10000:  # 10s of clean 10Hz coverage
        s.ev(t, "POSE_SAMPLE", robot_id="Alvik1", x_m=t / 10000.0, y_m=0.0,
             pose_age_ms=50)
        t += 100
    t = 15000  # 5s gap, then clean again
    while t <= 30000:
        s.ev(t, "POSE_SAMPLE", robot_id="Alvik1", x_m=t / 10000.0, y_m=0.0,
             pose_age_ms=50)
        t += 100
    s.ev(30000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "vision_dropout_episodes")["value"] >= 1
    assert value_of(m, "vision_availability_pct")["value"] < 95.0


# ---------------------------------------------------------------------------
# 10. battery unchanged over a short run: drop is 0, not null

def test_battery_unchanged(client):
    run_id = make_run(client, robot_count=1)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(0, "ROBOT_REGISTERED", robot_id="Alvik1", battery_pct=88.0)
    s.ev(20000, "BATTERY_SAMPLE", robot_id="Alvik1", battery_pct=88.0)
    s.ev(20000, "DEPOT_ARRIVED", robot_id="Alvik1", yaw_deg=0.0)
    s.ev(20000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    drop = value_of(m, "battery_drop_pct", "robot", "Alvik1")
    assert drop["value"] == 0.0 and drop["status"] != "missing"


# ---------------------------------------------------------------------------
# 11. a failed job stays in the completion-rate denominator

def test_failed_job_stays_in_denominator(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(1000, "JOB_RELEASED", job_id="J1", robot_id="Alvik1")
    s.ev(1000, "JOB_RELEASED", job_id="J2", robot_id="Alvik2")
    s.ev(30000, "DROPOFF_COMPLETED", job_id="J1", robot_id="Alvik1")
    s.ev(35000, "COMMAND_FAILED", job_id="J2", robot_id="Alvik2",
         reason="VISION_DROPOUT")
    s.ev(60000, "RUN_FAILED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    jc = value_of(m, "job_completion_rate")
    assert jc["numerator"] == 1 and jc["denominator"] == 2
    assert jc["value"] == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# 12. historical runs retain their original definition versions

def test_definition_versions_retained(client, tmp_path):
    run_id = make_run(client)
    post_events(client, run_id, full_two_robot_stream())
    resp = client.post(f"/api/runs/{run_id}/finalize", json={})
    assert resp.status_code == 200
    # finalized values reference definition v1
    m = get_metrics(client, run_id)
    assert not m["provisional"]
    assert value_of(m, "actual_makespan_sec")["definition_version"] == 1
    # finalize refuses to overwrite; recalculate needs explicit confirm and
    # creates a NEW revision, leaving revision 0 untouched
    assert client.post(f"/api/runs/{run_id}/finalize",
                       json={}).status_code == 409
    assert client.post(f"/api/runs/{run_id}/recalculate",
                       json={}).status_code == 400
    resp = client.post(f"/api/runs/{run_id}/recalculate",
                       json={"confirm": True})
    assert resp.status_code == 200
    assert resp.get_json()["revision"] == 1


# ---------------------------------------------------------------------------
# 13. planned vs actual stay separate

def test_planned_vs_actual_separation(client):
    run_id = make_run(client, planned_makespan_sec=100.0)
    post_events(client, run_id, full_two_robot_stream())
    m = get_metrics(client, run_id)
    assert value_of(m, "planned_makespan_sec")["value"] == pytest.approx(100.0)
    assert value_of(m, "actual_makespan_sec")["value"] == pytest.approx(120.0)
    assert value_of(m, "makespan_error_sec")["value"] == pytest.approx(20.0)
    assert value_of(m, "makespan_error_pct")["value"] == pytest.approx(20.0)


# ---------------------------------------------------------------------------
# 14. state-time reconciliation

def test_state_time_reconciliation(client):
    run_id = make_run(client)
    post_events(client, run_id, full_two_robot_stream())
    m = get_metrics(client, run_id)
    for robot in ("Alvik1", "Alvik2"):
        recon = value_of(m, "state_time_reconciliation_error_sec",
                         "robot", robot)
        assert recon["value"] <= metrics_engine.STATE_RECONCILE_TOL_SEC
        # not flagged as a reconciliation error ('ok' is relabeled
        # 'provisional' until the run is finalized)
        assert recon["status"] != "error"


# ---------------------------------------------------------------------------
# 15. persistence across a server restart (same DB file, new app)

def test_persistence_across_restart(tmp_path):
    db_path = str(tmp_path / "persist.sqlite3")
    app1 = create_app(db_path=db_path)
    with app1.test_client() as c1:
        run_id = make_run(c1)
        post_events(c1, run_id, full_two_robot_stream())
        c1.post(f"/api/runs/{run_id}/finalize", json={})
    # brand-new app instance = server restart
    app2 = create_app(db_path=db_path)
    with app2.test_client() as c2:
        m = c2.get(f"/api/runs/{run_id}/metrics").get_json()
        assert not m["provisional"]
        assert value_of(m, "actual_makespan_sec")["value"] == \
            pytest.approx(120.0)
        hist = c2.get("/api/history").get_json()
        assert any(r["run_id"] == run_id for r in hist["runs"])


# ---------------------------------------------------------------------------
# 16. the served page still contains the original app (server-side half)

def test_index_page_serves_all_four_views(client):
    resp = client.get("/")
    assert resp.status_code == 200
    html = resp.get_data(as_text=True)
    for needle in ("view-setup", "view-plan", "view-run", "view-metrics",
                   "grid-svg", "decision-log", "dispatch-generate",
                   "sup-send", "live-feed",
                   "/static/js/solver.js", "/static/js/live-monitor.js",
                   "/static/js/metrics.js", "/static/js/app.js",
                   "/static/css/app.css", "/static/css/metrics.css"):
        assert needle in html, f"missing {needle}"


# ---------------------------------------------------------------------------
# extras: duplicate delivery, history filters, CSV export

def test_duplicate_events_are_skipped(client):
    run_id = make_run(client)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    s.ev(1000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    result = post_events(client, run_id, s)  # re-deliver the same batch
    assert result["inserted"] == 0 and result["duplicates"] == 2


def test_history_filters_and_csv(client):
    a = make_run(client, experimental_condition="cond-A")
    b = make_run(client, experimental_condition="cond-B")
    for run_id in (a, b):
        post_events(client, run_id, full_two_robot_stream())
        client.post(f"/api/runs/{run_id}/finalize", json={})
    hist = client.get(
        "/api/history?experimental_condition=cond-A").get_json()
    assert [r["run_id"] for r in hist["runs"]] == [a]
    grouped = client.get(
        "/api/history?group_by=experimental_condition").get_json()
    assert set(grouped["aggregates"].keys()) == {"cond-A", "cond-B"}
    agg = grouped["aggregates"]["cond-A"]["actual_makespan_sec"]
    assert agg["n"] == 1 and agg["contributing_run_ids"] == [a]
    csv_text = client.get("/api/export/runs.csv").get_data(as_text=True)
    assert "actual_makespan_sec" in csv_text.splitlines()[0]
    assert a in csv_text and b in csv_text
    run_json = client.get(f"/api/export/run/{a}.json").get_json()
    assert run_json["run"]["run_id"] == a
    assert run_json["metric_values"]


# ---------------------------------------------------------------------------
# deadlock conflicts reach the metrics (regression, 2026-08-31)

def test_unresolved_deadlock_is_counted_and_blocks_intervention_free(client):
    """The 6-robot run on 2026-08-31 held an Alvik3<->Alvik5 edge swap for
    78s and metrics reported deadlock_count = 0, because nothing in the
    supervisor ever emitted a conflict -- the advisor detected it, logged it
    ~100 times, and the metrics pipeline never heard about it. The backend
    was always ready (database.py handles DEADLOCK_DETECTED); only the
    emission was missing. This pins the whole chain."""
    run_id = make_run(client, robot_count=2)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    for r in ("Alvik3", "Alvik5"):
        s.ev(0, "ROBOT_REGISTERED", robot_id=r)
        s.ev(0, "ROBOT_STATE_CHANGED", robot_id=r,
             state_to="PRODUCTIVE_TRAVEL_LOADED")
        s.ev(10000, "ROBOT_STATE_CHANGED", robot_id=r, state_to="TRAFFIC_WAIT")
    s.ev(10000, "DEADLOCK_DETECTED", conflict_id="dl-Alvik3-Alvik5-10",
         details={"type": "deadlock", "severity": "error",
                  "robot_ids": ["Alvik3", "Alvik5"]})
    s.ev(30000, "RUN_ABORTED", reason="deadlock: Alvik3, Alvik5")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "deadlock_count")["value"] == 1
    assert value_of(m, "unresolved_deadlock_count")["value"] == 1
    # An unresolved deadlock must disqualify the run from intervention-free.
    assert value_of(m, "intervention_free_success")["value"] == 0


def test_deadlock_that_clears_is_resolved_not_unresolved(client):
    """A cycle that clears on its own is recorded as resolved, so recovery
    time has something to measure and the run is not wrongly disqualified."""
    run_id = make_run(client, robot_count=2)
    s = Stream()
    s.ev(0, "RUN_STARTED")
    for r in ("Alvik3", "Alvik5"):
        s.ev(0, "ROBOT_REGISTERED", robot_id=r)
        s.ev(0, "ROBOT_STATE_CHANGED", robot_id=r,
             state_to="PRODUCTIVE_TRAVEL_LOADED")
        s.ev(40000, "DEPOT_ARRIVED", robot_id=r, yaw_deg=0.0)
    s.ev(10000, "DEADLOCK_DETECTED", conflict_id="dl-1",
         details={"type": "deadlock", "severity": "error",
                  "robot_ids": ["Alvik3", "Alvik5"]})
    s.ev(18000, "DEADLOCK_RESOLVED", conflict_id="dl-1",
         reason="cycle cleared without intervention")
    s.ev(40000, "RUN_COMPLETED")
    post_events(client, run_id, s)
    m = get_metrics(client, run_id)
    assert value_of(m, "deadlock_count")["value"] == 1
    assert value_of(m, "unresolved_deadlock_count")["value"] == 0
    assert value_of(m, "mean_deadlock_recovery_sec")["value"] ==         pytest.approx(8.0)
