"""Deterministic metric computation for the AGV testbed metrics backend.

Source of truth: the events + telemetry_samples tables (plus the
runs/run_robots/jobs/conflicts projections built from them). Everything here
is a pure function of the database contents -- recomputing a run always
yields the same values, which is what makes finalize/recalculate revisions
meaningful.

Missing-data policy (spec section 21): a metric whose inputs are absent is
reported with value=None and status='missing' -- never zero. Failed attempts
always stay in reliability denominators; accuracy means use only valid
measured completions and carry their exact n in valid_n.

Canonical units: seconds, meters, degrees, percent, watt-hours.
"""
from __future__ import annotations

import json
import math
import sqlite3

from metrics_definitions import DEFINITIONS_BY_KEY

# ---- thresholds (single place; referenced by the Data Quality tab) ---------
POSE_FRESH_MAX_MS = 500.0        # a_max: pose older than this is not "fresh"
SEPARATION_WARN_M = 0.30         # d_warn  (~12 in)
SEPARATION_COLLISION_M = 0.10    # d_collision (~4 in)
SEPARATION_BUCKET_MS = 250       # pairwise-distance time alignment bucket
TERMINAL_HEADING_TOL_DEG = 10.0  # tau_psi
TERMINAL_POSITION_TOL_M = 0.10   # depot position tolerance
REQUIRED_DEPOT_HEADING_DEG = 0.0  # psi*: facing south = 0 in the testbed frame
STATE_RECONCILE_TOL_SEC = 0.5    # timestamp-rounding tolerance (spec sec 6)
YAW_TOLERANCE_DEG = 5.0          # default rotation tolerance when the event
                                 # details do not carry within_tolerance

PRODUCTIVE_STATES = {
    "PRODUCTIVE_TRAVEL_EMPTY", "PRODUCTIVE_TRAVEL_LOADED",
    "PICKUP_SERVICE", "DROPOFF_SERVICE",
}
ALL_STATES = PRODUCTIVE_STATES | {
    "NOT_RELEASED", "PROCESSING_HOLD", "TRAFFIC_WAIT", "RESOURCE_QUEUE",
    "RETURN_REPOSITION", "IDLE_READY", "FAULT_RECOVERY", "OFFLINE",
    "TERMINAL_IDLE",
}
OCCUPIED_STATES = PRODUCTIVE_STATES | {
    "PROCESSING_HOLD", "TRAFFIC_WAIT", "RESOURCE_QUEUE", "RETURN_REPOSITION",
}

TERMINAL_RUN_EVENTS = {"RUN_COMPLETED", "RUN_FAILED", "RUN_ABORTED"}


# ---------------------------------------------------------------------------
# small numeric helpers (no numpy dependency; deterministic)

def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def pstdev(xs):
    """Population sigma, None below 2 samples.

    None rather than 0.0 deliberately: a single measurement has no spread,
    and 0.0 would read as "perfectly consistent" -- the opposite of what one
    sample means. Same null-never-zero policy as the rest of the engine."""
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return None
    mu = sum(xs) / len(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / len(xs))


def median(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    n = len(xs)
    mid = n // 2
    return xs[mid] if n % 2 else (xs[mid - 1] + xs[mid]) / 2.0


def sample_std(xs):
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return None
    mu = sum(xs) / len(xs)
    return math.sqrt(sum((x - mu) ** 2 for x in xs) / (len(xs) - 1))


def percentile(xs, p):
    """Linear-interpolation percentile, p in [0, 100]."""
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    if len(xs) == 1:
        return xs[0]
    k = (len(xs) - 1) * (p / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return xs[int(k)]
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


# two-sided 95% Student-t critical values by degrees of freedom
_T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
        7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
        13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101,
        19: 2.093, 20: 2.086, 25: 2.060, 30: 2.042, 40: 2.021, 60: 2.000,
        120: 1.980}


def t_critical_95(df):
    if df <= 0:
        return None
    if df in _T95:
        return _T95[df]
    for bound in sorted(_T95):
        if df <= bound:
            return _T95[bound]
    return 1.96


def aggregate_stats(xs):
    """n / mean / median / std / cv / ci95 / min / max over a sample.
    Returns a dict; entries are None where undefined (n<2 etc.)."""
    xs = [x for x in xs if x is not None]
    n = len(xs)
    out = {"n": n, "mean": mean(xs), "median": median(xs),
           "std": sample_std(xs), "cv": None, "ci95_low": None,
           "ci95_high": None, "min": min(xs) if xs else None,
           "max": max(xs) if xs else None}
    if out["std"] is not None and out["mean"] not in (None, 0):
        out["cv"] = out["std"] / out["mean"]
    if n >= 2 and out["std"] is not None:
        t = t_critical_95(n - 1)
        half = t * out["std"] / math.sqrt(n)
        out["ci95_low"] = out["mean"] - half
        out["ci95_high"] = out["mean"] + half
    return out


def wrap_deg(theta):
    """Wrap a heading into [-180, 180)."""
    return (theta + 180.0) % 360.0 - 180.0


def proportion_ci95(k, n):
    """Wilson 95% interval for a proportion; (low, high) or (None, None)."""
    if not n:
        return None, None
    z = 1.96
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, center - half), min(1.0, center + half)


# ---------------------------------------------------------------------------
# data loading

def _rows(conn, sql, args=()):
    return conn.execute(sql, args).fetchall()


def load_run_bundle(conn: sqlite3.Connection, run_id: str) -> dict:
    run = conn.execute("SELECT * FROM runs WHERE run_id = ?",
                       (run_id,)).fetchone()
    if run is None:
        raise KeyError(f"unknown run {run_id}")
    events = _rows(conn, "SELECT * FROM events WHERE run_id = ? ORDER BY seq",
                   (run_id,))
    parsed = []
    for ev in events:
        d = dict(ev)
        try:
            d["details"] = json.loads(ev["details_json"]) if ev["details_json"] else {}
        except (TypeError, ValueError):
            d["details"] = {}
        parsed.append(d)
    return {
        "run": dict(run),
        "robots": [dict(r) for r in _rows(
            conn, "SELECT * FROM run_robots WHERE run_id = ? ORDER BY robot_id",
            (run_id,))],
        "jobs": [dict(r) for r in _rows(
            conn, "SELECT * FROM jobs WHERE run_id = ? ORDER BY job_id",
            (run_id,))],
        "events": parsed,
        "telemetry": [dict(r) for r in _rows(
            conn, """SELECT * FROM telemetry_samples WHERE run_id = ?
                     ORDER BY robot_id, elapsed_ms""", (run_id,))],
        "conflicts": [dict(r) for r in _rows(
            conn, "SELECT * FROM conflicts WHERE run_id = ? ORDER BY conflict_id",
            (run_id,))],
    }


# ---------------------------------------------------------------------------
# timeline / state model

def run_times(bundle) -> dict:
    """t0/tend (elapsed ms) and durations. Makespan only for completed runs;
    observed duration for failed/aborted (spec 7.1 / 7.2)."""
    events = bundle["events"]
    status = bundle["run"]["status"]
    t0 = next((e["elapsed_ms"] for e in events
               if e["event_type"] == "RUN_STARTED"), None)
    tend = next((e["elapsed_ms"] for e in events
                 if e["event_type"] in TERMINAL_RUN_EVENTS), None)
    makespan = None
    observed = None
    if t0 is not None and tend is not None:
        span = max(0.0, (tend - t0) / 1000.0)
        if status == "completed":
            makespan = span
        else:
            observed = span
    return {"t0_ms": t0, "tend_ms": tend, "makespan_sec": makespan,
            "observed_sec": observed,
            "span_sec": makespan if makespan is not None else observed}


def robot_state_durations(bundle) -> dict:
    """Per-robot {state: seconds} from ROBOT_STATE_CHANGED transitions.

    The timeline is closed: NOT_RELEASED is implicit from t0 until a robot's
    first transition, and the last state extends to the run's end event.
    Events with a null/unknown state_to accumulate under 'UNCLASSIFIED'
    (excluded from productive time; surfaced in Data Quality). Also returns
    per-robot episode counts of entries into each state."""
    times = run_times(bundle)
    t0, tend = times["t0_ms"], times["tend_ms"]
    out: dict[str, dict] = {}
    if t0 is None or tend is None:
        return out
    by_robot: dict[str, list] = {}
    for ev in bundle["events"]:
        if ev["event_type"] == "ROBOT_STATE_CHANGED" and ev["robot_id"]:
            by_robot.setdefault(ev["robot_id"], []).append(ev)
    robot_ids = {r["robot_id"] for r in bundle["robots"]} | set(by_robot)
    for robot in sorted(robot_ids):
        transitions = by_robot.get(robot, [])
        durations: dict[str, float] = {}
        entries: dict[str, int] = {}
        cur_state = "NOT_RELEASED"
        cur_at = t0
        issues = 0
        for ev in transitions:
            at = ev["elapsed_ms"]
            if at is None:
                issues += 1
                continue
            at = min(max(at, t0), tend)
            dur = (at - cur_at) / 1000.0
            if dur < 0:
                issues += 1
                dur = 0.0
            durations[cur_state] = durations.get(cur_state, 0.0) + dur
            cur_state = ev["state_to"] or "UNCLASSIFIED"
            entries[cur_state] = entries.get(cur_state, 0) + 1
            cur_at = at
        durations[cur_state] = durations.get(cur_state, 0.0) + \
            max(0.0, (tend - cur_at) / 1000.0)
        out[robot] = {"durations": durations, "entries": entries,
                      "issues": issues}
    return out


# ---------------------------------------------------------------------------
# metric-value assembly

def _def_version(key):
    d = DEFINITIONS_BY_KEY.get(key)
    return d["version"] if d else 1


def _unit(key):
    d = DEFINITIONS_BY_KEY.get(key)
    return d["unit"] if d else None


class Collector:
    def __init__(self):
        self.values: list[dict] = []

    def add(self, key, value, *, scope_type="run", scope_id="",
            numerator=None, denominator=None, valid_n=None, missing_n=None,
            status=None):
        if status is None:
            status = "ok" if value is not None else "missing"
        self.values.append({
            "metric_key": key,
            "definition_version": _def_version(key),
            "scope_type": scope_type,
            "scope_id": scope_id,
            "value": value,
            "unit": _unit(key),
            "numerator": numerator,
            "denominator": denominator,
            "valid_n": valid_n,
            "missing_n": missing_n,
            "status": status,
        })


def _ratio(num, den):
    if den in (None, 0):
        return None
    if num is None:
        return None
    return num / den


# ---------------------------------------------------------------------------
# the main computation

def compute_run_metrics(conn: sqlite3.Connection, run_id: str) -> list[dict]:
    bundle = load_run_bundle(conn, run_id)
    c = Collector()
    run = bundle["run"]
    times = run_times(bundle)
    T = times["makespan_sec"]
    span = times["span_sec"]  # makespan or observed duration
    events = bundle["events"]
    jobs = bundle["jobs"]
    robots = bundle["robots"]
    N = len(robots) or run.get("robot_count") or None

    # ---- 7.1 / 7.2 durations ----------------------------------------------
    c.add("actual_makespan_sec", T)
    c.add("observed_duration_sec", times["observed_sec"],
          status="ok" if times["observed_sec"] is not None else "missing")

    # ---- jobs ---------------------------------------------------------------
    JA = len(jobs)
    completed_jobs = [j for j in jobs if j["status"] == "completed"]
    JC = len(completed_jobs)
    c.add("job_completion_rate", _ratio(JC, JA), numerator=JC, denominator=JA,
          valid_n=JA)
    c.add("throughput_jobs_per_min",
          _ratio(60.0 * JC, T) if T else None,
          numerator=JC, denominator=T)
    c.add("observed_throughput_jobs_per_min",
          _ratio(60.0 * JC, times["observed_sec"]) if times["observed_sec"] else None,
          numerator=JC, denominator=times["observed_sec"],
          status="ok" if times["observed_sec"] else "missing")

    flow = [( (j["completion_elapsed_ms"] - j["release_elapsed_ms"]) / 1000.0)
            for j in completed_jobs
            if j["completion_elapsed_ms"] is not None
            and j["release_elapsed_ms"] is not None]
    miss_flow = JC - len(flow)
    c.add("job_flow_time_mean_sec", mean(flow), valid_n=len(flow),
          missing_n=miss_flow)
    c.add("job_flow_time_median_sec", median(flow), valid_n=len(flow))
    c.add("job_flow_time_std_sec", sample_std(flow), valid_n=len(flow))
    c.add("job_flow_time_max_sec", max(flow) if flow else None,
          valid_n=len(flow))
    c.add("job_flow_time_p95_sec", percentile(flow, 95), valid_n=len(flow))

    resp = [(j["pickup_elapsed_ms"] - j["release_elapsed_ms"]) / 1000.0
            for j in jobs
            if j["pickup_elapsed_ms"] is not None
            and j["release_elapsed_ms"] is not None]
    c.add("pickup_response_time_mean_sec", mean(resp), valid_n=len(resp))
    loaded = [(j["delivery_elapsed_ms"] - j["pickup_elapsed_ms"]) / 1000.0
              for j in jobs
              if j["delivery_elapsed_ms"] is not None
              and j["pickup_elapsed_ms"] is not None]
    c.add("loaded_transport_time_mean_sec", mean(loaded), valid_n=len(loaded))

    due_jobs = [j for j in jobs if j["due_elapsed_ms"] is not None
                and j["completion_elapsed_ms"] is not None]
    if due_jobs:
        tard = [max(0.0, (j["completion_elapsed_ms"] - j["due_elapsed_ms"]) / 1000.0)
                for j in due_jobs]
        c.add("job_tardiness_mean_sec", mean(tard), valid_n=len(tard))
    # (no due times -> metric intentionally absent, per spec 7.12)

    completions = [r["completion_elapsed_ms"] / 1000.0 for r in robots
                   if r["completion_elapsed_ms"] is not None]
    if len(completions) >= 2:
        c.add("robot_completion_spread_sec",
              max(completions) - min(completions), valid_n=len(completions))
        c.add("tail_completion_delay_sec",
              max(completions) - median(completions), valid_n=len(completions))
    else:
        c.add("robot_completion_spread_sec", None, valid_n=len(completions))
        c.add("tail_completion_delay_sec", None, valid_n=len(completions))

    # ---- success flags ------------------------------------------------------
    status = run["status"]
    robots_ok = robots and all(r["status"] == "completed" for r in robots)
    jobs_ok = (JA == 0) or (JC == JA)
    heading_checks = [r for r in robots if r["terminal_yaw_deg"] is not None]
    headings_ok = all(
        abs(wrap_deg(r["terminal_yaw_deg"] - REQUIRED_DEPOT_HEADING_DEG))
        <= TERMINAL_HEADING_TOL_DEG for r in heading_checks)
    mission_ok = 1 if (status == "completed" and robots_ok and jobs_ok
                       and headings_ok) else 0
    if status in ("created", "running"):
        c.add("mission_completion_success", None, status="missing")
    else:
        c.add("mission_completion_success", mission_ok)

    manual = [e for e in events if e["event_type"] == "MANUAL_INTERVENTION"]
    unresolved_deadlocks = [cf for cf in bundle["conflicts"]
                            if cf["type"] == "deadlock"
                            and cf["status"] != "resolved"]
    c.add("manual_intervention_count", len(manual), valid_n=len(manual))
    if status in ("created", "running"):
        c.add("intervention_free_success", None, status="missing")
        c.add("safe_completion_success", None, status="missing")
    else:
        c.add("intervention_free_success",
              1 if (mission_ok and not manual and not unresolved_deadlocks)
              else 0)
        collisions = [e for e in events if e["event_type"] == "COLLISION"]
        c.add("safe_completion_success",
              1 if (mission_ok and not collisions) else 0)

    collisions = [e for e in events if e["event_type"] == "COLLISION"]
    c.add("collision_count", len(collisions), valid_n=len(collisions))

    # ---- 8. state-model utilization ----------------------------------------
    state_model = robot_state_durations(bundle)
    total_productive = 0.0
    total_traffic = 0.0
    total_process = 0.0
    total_queue = 0.0
    traffic_entries = 0
    have_states = bool(state_model) and span
    for robot, sm in state_model.items():
        dur = sm["durations"]
        P = sum(dur.get(s, 0.0) for s in PRODUCTIVE_STATES)
        occupied = sum(dur.get(s, 0.0) for s in OCCUPIED_STATES)
        total_productive += P
        total_traffic += dur.get("TRAFFIC_WAIT", 0.0)
        total_process += dur.get("PROCESSING_HOLD", 0.0)
        total_queue += dur.get("RESOURCE_QUEUE", 0.0)
        traffic_entries += sm["entries"].get("TRAFFIC_WAIT", 0)
        if span:
            c.add("robot_productive_utilization_pct", 100.0 * P / span,
                  scope_type="robot", scope_id=robot,
                  numerator=P, denominator=span)
            c.add("occupied_utilization_pct", 100.0 * occupied / span,
                  scope_type="robot", scope_id=robot,
                  numerator=occupied, denominator=span)
            for state_name, secs in sorted(dur.items()):
                c.add("state_fraction_pct", 100.0 * secs / span,
                      scope_type="robot", scope_id=f"{robot}:{state_name}",
                      numerator=secs, denominator=span)
            fault = dur.get("FAULT_RECOVERY", 0.0)
            offline = dur.get("OFFLINE", 0.0)
            c.add("robot_online_availability_pct",
                  100.0 * (span - fault - offline) / span,
                  scope_type="robot", scope_id=robot,
                  numerator=span - fault - offline, denominator=span)
            # spec section 6: reconcile mutually-exclusive state durations
            recon = abs(span - sum(dur.values()))
            c.add("state_time_reconciliation_error_sec", recon,
                  scope_type="robot", scope_id=robot,
                  status="ok" if recon <= STATE_RECONCILE_TOL_SEC else "error")
    if have_states and N:
        c.add("fleet_productive_utilization_pct",
              100.0 * total_productive / (N * span),
              numerator=total_productive, denominator=N * span, valid_n=N)
    else:
        c.add("fleet_productive_utilization_pct", None)
    c.add("fleet_traffic_delay_robot_sec",
          total_traffic if have_states else None,
          valid_n=len(state_model) or None)
    c.add("traffic_delay_per_job_sec",
          _ratio(total_traffic, JC) if have_states else None,
          numerator=total_traffic if have_states else None, denominator=JC)
    c.add("processing_hold_per_job_sec",
          _ratio(total_process, JC) if have_states else None,
          numerator=total_process if have_states else None, denominator=JC)
    c.add("queue_delay_per_job_sec",
          _ratio(total_queue, JC) if have_states else None,
          numerator=total_queue if have_states else None, denominator=JC)
    c.add("traffic_stop_episodes",
          traffic_entries if have_states else None)
    c.add("mean_traffic_stop_duration_sec",
          _ratio(total_traffic, traffic_entries) if have_states else None,
          numerator=total_traffic if have_states else None,
          denominator=traffic_entries or None)

    # ---- 9. coordination ----------------------------------------------------
    conflicts = [cf for cf in bundle["conflicts"] if cf["type"] != "deadlock"]
    deadlocks = [cf for cf in bundle["conflicts"] if cf["type"] == "deadlock"]
    c.add("conflict_count", len(conflicts), valid_n=len(conflicts))
    c.add("conflict_rate_per_job", _ratio(len(conflicts), JC),
          numerator=len(conflicts), denominator=JC)
    auto_resolved = [cf for cf in conflicts if cf["status"] == "resolved"
                     and (cf["resolution"] or "").lower() != "manual"]
    c.add("auto_conflict_resolution_rate",
          _ratio(len(auto_resolved), len(conflicts)),
          numerator=len(auto_resolved), denominator=len(conflicts))
    safety_stops = [e for e in events if e["event_type"] == "SAFETY_STOP"]
    c.add("safety_stop_count", len(safety_stops), valid_n=len(safety_stops))

    reroutes_applied = [e for e in events if e["event_type"] == "REROUTE_APPLIED"]
    reroutes_failed = [e for e in events if e["event_type"] == "REROUTE_FAILED"]
    c.add("reroute_count", len(reroutes_applied))
    issued = len(reroutes_applied) + len(reroutes_failed)
    c.add("reroute_success_rate", _ratio(len(reroutes_applied), issued),
          numerator=len(reroutes_applied), denominator=issued)
    detour_d = [e["details"].get("detour_distance_m") for e in reroutes_applied]
    detour_t = [e["details"].get("detour_time_sec") for e in reroutes_applied]
    c.add("reroute_detour_distance_m", mean(detour_d),
          valid_n=len([x for x in detour_d if x is not None]))
    c.add("reroute_detour_time_sec", mean(detour_t),
          valid_n=len([x for x in detour_t if x is not None]))

    c.add("deadlock_count", len(deadlocks), valid_n=len(deadlocks))
    resolved_dl = [d for d in deadlocks if d["status"] == "resolved"
                   and d["resolved_elapsed_ms"] is not None
                   and d["detected_elapsed_ms"] is not None]
    c.add("mean_deadlock_recovery_sec",
          mean([(d["resolved_elapsed_ms"] - d["detected_elapsed_ms"]) / 1000.0
                for d in resolved_dl]),
          valid_n=len(resolved_dl),
          missing_n=len(deadlocks) - len(resolved_dl))
    c.add("unresolved_deadlock_count", len(unresolved_deadlocks),
          valid_n=len(unresolved_deadlocks))

    # ---- 10. separation / vision safety ------------------------------------
    telemetry = [t for t in bundle["telemetry"] if t["valid"]]
    min_sep, below_warn_sec = _separation_metrics(telemetry)
    c.add("min_fleet_separation_m", min_sep,
          valid_n=len(telemetry) or None,
          status="ok" if min_sep is not None else "missing")
    c.add("time_below_warning_separation_sec", below_warn_sec,
          status="ok" if below_warn_sec is not None else "missing")
    near_misses = [e for e in events if e["event_type"] == "NEAR_MISS_STARTED"]
    c.add("near_miss_count", len(near_misses), valid_n=len(near_misses))

    # ---- 11. distance -------------------------------------------------------
    dist_by_robot = _distance_by_robot(telemetry)
    D_fleet = sum(dist_by_robot.values()) if dist_by_robot else None
    for robot, d in sorted(dist_by_robot.items()):
        c.add("robot_distance_m", d, scope_type="robot", scope_id=robot)
    c.add("total_distance_m", D_fleet,
          valid_n=len(dist_by_robot) or None)
    c.add("distance_per_job_m", _ratio(D_fleet, JC),
          numerator=D_fleet, denominator=JC)

    # ---- 13. plan fidelity --------------------------------------------------
    planned = run["planned_makespan_sec"]
    c.add("planned_makespan_sec", planned)
    if planned and T is not None:
        c.add("makespan_error_sec", T - planned, numerator=T,
              denominator=planned)
        c.add("makespan_error_pct", 100.0 * (T - planned) / planned,
              numerator=T - planned, denominator=planned)
    else:
        c.add("makespan_error_sec", None)
        c.add("makespan_error_pct", None)

    # command latencies / reliability (17.1, 13.10, 13.11)
    sent = {e["command_id"]: e for e in events
            if e["event_type"] == "COMMAND_SENT" and e["command_id"]}
    acked = [e for e in events
             if e["event_type"] == "COMMAND_ACKNOWLEDGED" and e["command_id"] in sent]
    done = [e for e in events
            if e["event_type"] == "COMMAND_COMPLETED" and e["command_id"] in sent]
    failed_cmds = [e for e in events if e["event_type"] == "COMMAND_FAILED"]
    QA = len(sent) or len([e for e in events
                           if e["event_type"] == "COMMAND_SENT"])
    QC = len(done)
    c.add("command_success_rate", _ratio(QC, QA), numerator=QC,
          denominator=QA, valid_n=QA)
    ack_lat = [(e["elapsed_ms"] - sent[e["command_id"]]["elapsed_ms"]) / 1000.0
               for e in acked
               if e["elapsed_ms"] is not None
               and sent[e["command_id"]]["elapsed_ms"] is not None]
    c.add("command_ack_latency_sec", mean(ack_lat), valid_n=len(ack_lat))
    done_lat = [(e["elapsed_ms"] - sent[e["command_id"]]["elapsed_ms"]) / 1000.0
                for e in done
                if e["elapsed_ms"] is not None
                and sent[e["command_id"]]["elapsed_ms"] is not None]
    c.add("command_completion_latency_sec", mean(done_lat),
          valid_n=len(done_lat))

    # ---- movement breakdown: per robot, per kind (2026-09-01) -------------
    # command_completion_latency_sec above averages EVERY command together,
    # which hides the thing worth knowing: a drive leg, a rotation and a dwell
    # have nothing to do with one another, and the robots differ from each
    # other. fleetSupervisor's own timing calibration already showed Alvik4
    # running ~1.48x its predicted move duration against Alvik1's ~1.14x, with
    # no metric capturing it.
    #
    # No new instrumentation: COMMAND_SENT already carries details.kind
    # (move/turn/dwell) and COMMAND_COMPLETED closes it by command_id.
    #
    # Scope ids: "<robot>:<kind>" per kind, "<robot>" for that robot's total,
    # and the kind alone for the fleet rows.
    sent_by_id = {e["command_id"]: e for e in sent.values()} if isinstance(
        sent, dict) else {}
    durations: dict[tuple[str, str], list[float]] = {}
    dispatched: dict[tuple[str, str], int] = {}
    completed: dict[tuple[str, str], int] = {}

    def _kind_of(ev) -> str:
        d = ev.get("details") or {}
        k = d.get("kind")
        return str(k) if k else "unknown"

    for ev in sent.values():
        rb = ev.get("robot_id")
        if not rb:
            continue
        for key in ((rb, _kind_of(ev)), (rb, "")):
            dispatched[key] = dispatched.get(key, 0) + 1
    for ev in done:
        origin = sent.get(ev["command_id"])
        rb = ev.get("robot_id") or (origin or {}).get("robot_id")
        if not rb or origin is None:
            continue
        kind = _kind_of(origin)
        for key in ((rb, kind), (rb, "")):
            completed[key] = completed.get(key, 0) + 1
        if (ev["elapsed_ms"] is not None
                and origin["elapsed_ms"] is not None):
            d = (ev["elapsed_ms"] - origin["elapsed_ms"]) / 1000.0
            durations.setdefault((rb, kind), []).append(d)
            durations.setdefault((rb, ""), []).append(d)

    def _scope(rb: str, kind: str) -> str:
        return f"{rb}:{kind}" if kind else rb

    for (rb, kind), ds in sorted(durations.items()):
        sid = _scope(rb, kind)
        n_sent = dispatched.get((rb, kind), 0)
        n_done = completed.get((rb, kind), 0)
        c.add("movement_count", n_done, scope_type="robot", scope_id=sid,
              valid_n=n_done, missing_n=max(n_sent - n_done, 0))
        c.add("movement_duration_mean_sec", mean(ds), scope_type="robot",
              scope_id=sid, valid_n=len(ds),
              missing_n=max(n_done - len(ds), 0))
        c.add("movement_duration_median_sec", median(ds), scope_type="robot",
              scope_id=sid, valid_n=len(ds))
        # Population sigma, and only with >=2 samples: one movement has no
        # spread, and reporting 0.0 there would read as "perfectly consistent".
        c.add("movement_duration_std_sec",
              pstdev(ds) if len(ds) >= 2 else None,
              scope_type="robot", scope_id=sid, valid_n=len(ds),
              missing_n=0 if len(ds) >= 2 else 1)
        c.add("movement_success_rate", _ratio(n_done, n_sent),
              scope_type="robot", scope_id=sid,
              numerator=n_done, denominator=n_sent, valid_n=n_sent)

    # Fleet baseline per kind, then each robot's signed difference from it.
    kinds = sorted({k for (_r, k) in durations if k})
    for kind in kinds:
        per_robot = {rb: mean(ds) for (rb, k), ds in durations.items()
                     if k == kind and ds}
        per_robot = {rb: v for rb, v in per_robot.items() if v is not None}
        if not per_robot:
            continue
        fleet_mean = mean(list(per_robot.values()))
        c.add("fleet_movement_duration_mean_sec", fleet_mean,
              scope_id=kind, valid_n=len(per_robot))
        # A single robot has no fleet to differ from -- report missing rather
        # than a spread of 0.0, which would claim the fleet is homogeneous.
        if len(per_robot) >= 2:
            c.add("movement_duration_robot_spread_sec",
                  max(per_robot.values()) - min(per_robot.values()),
                  scope_id=kind, valid_n=len(per_robot))
            for rb, v in sorted(per_robot.items()):
                c.add("movement_duration_delta_sec", v - fleet_mean,
                      scope_type="robot", scope_id=f"{rb}:{kind}",
                      valid_n=len(durations[(rb, kind)]))
        else:
            c.add("movement_duration_robot_spread_sec", None, scope_id=kind,
                  valid_n=len(per_robot), missing_n=1)

    # fault episodes / recovery (17.2, 17.3)
    offline_evs = [e for e in events if e["event_type"] == "ROBOT_OFFLINE"]
    recovered_evs = [e for e in events if e["event_type"] == "ROBOT_RECOVERED"]
    c.add("robot_fault_count", len(offline_evs) + len(failed_cmds),
          valid_n=len(offline_evs) + len(failed_cmds))
    recov = []
    rec_by_robot: dict[str, list] = {}
    for e in recovered_evs:
        rec_by_robot.setdefault(e["robot_id"], []).append(e)
    for e in offline_evs:
        cands = [r for r in rec_by_robot.get(e["robot_id"], [])
                 if r["elapsed_ms"] is not None and e["elapsed_ms"] is not None
                 and r["elapsed_ms"] >= e["elapsed_ms"]]
        if cands:
            recov.append((cands[0]["elapsed_ms"] - e["elapsed_ms"]) / 1000.0)
    c.add("mean_recovery_time_sec", mean(recov), valid_n=len(recov),
          missing_n=len(offline_evs) - len(recov))

    # ---- 14. localization ---------------------------------------------------
    _vision_metrics(c, telemetry, span, robots)

    # terminal accuracy (14.6-14.8)
    term_ok = 0
    term_checked = 0
    for r in robots:
        if r["terminal_yaw_deg"] is not None:
            err = abs(wrap_deg(r["terminal_yaw_deg"]
                               - REQUIRED_DEPOT_HEADING_DEG))
            c.add("terminal_heading_error_deg", err, scope_type="robot",
                  scope_id=r["robot_id"])
            term_checked += 1
            if err <= TERMINAL_HEADING_TOL_DEG:
                term_ok += 1
    c.add("terminal_condition_success_rate",
          _ratio(term_ok, N) if term_checked else None,
          numerator=term_ok, denominator=N, valid_n=term_checked,
          missing_n=(N - term_checked) if N else None)
    depot_arrivals = [e for e in events if e["event_type"] == "DEPOT_ARRIVED"]
    c.add("docking_success_rate",
          _ratio(len({e["robot_id"] for e in depot_arrivals}), N)
          if depot_arrivals and N else None,
          numerator=len({e["robot_id"] for e in depot_arrivals}) or None,
          denominator=N)

    # ---- 15. rotation control ----------------------------------------------
    _rotation_metrics(c, events)

    # ---- 16. battery --------------------------------------------------------
    _battery_metrics(c, robots, dist_by_robot, state_model, span, JC,
                     telemetry)

    # ---- 12. workload balance ----------------------------------------------
    _fairness_metrics(c, robots, jobs, N)

    # ---- 18. workstations ---------------------------------------------------
    _workstation_metrics(c, events, run, span)

    # ---- 19. data quality (summary count; itemized via run_data_quality) ---
    dq = run_data_quality(conn, run_id, bundle=bundle)
    c.add("event_sequence_issues", len(dq["issues"]),
          valid_n=len(bundle["events"]))
    c.add("telemetry_coverage_pct", dq["telemetry_coverage_pct"],
          status="ok" if dq["telemetry_coverage_pct"] is not None else "missing")

    return c.values


# ---------------------------------------------------------------------------
# sub-computations

def _distance_by_robot(telemetry) -> dict:
    out: dict[str, float] = {}
    last: dict[str, tuple] = {}
    for t in telemetry:
        if t["x_m"] is None or t["y_m"] is None:
            continue
        robot = t["robot_id"]
        if robot in last:
            x0, y0 = last[robot]
            out[robot] = out.get(robot, 0.0) + math.hypot(
                t["x_m"] - x0, t["y_m"] - y0)
        else:
            out.setdefault(robot, 0.0)
        last[robot] = (t["x_m"], t["y_m"])
    return out


def _separation_metrics(telemetry):
    """(min pairwise separation, seconds below warning distance), aligned
    into SEPARATION_BUCKET_MS time buckets. None when <2 robots have data."""
    buckets: dict[int, dict[str, tuple]] = {}
    for t in telemetry:
        if t["x_m"] is None or t["y_m"] is None:
            continue
        b = int(t["elapsed_ms"] // SEPARATION_BUCKET_MS)
        buckets.setdefault(b, {})[t["robot_id"]] = (t["x_m"], t["y_m"])
    min_sep = None
    below = 0.0
    for b, robots in buckets.items():
        if len(robots) < 2:
            continue
        pts = list(robots.values())
        m = min(math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
                for i in range(len(pts)) for j in range(i + 1, len(pts)))
        if min_sep is None or m < min_sep:
            min_sep = m
        if m <= SEPARATION_WARN_M:
            below += SEPARATION_BUCKET_MS / 1000.0
    return min_sep, (below if min_sep is not None else None)


def _vision_metrics(c, telemetry, span, robots):
    ages = [t["pose_age_ms"] for t in telemetry if t["pose_age_ms"] is not None]
    c.add("pose_age_mean_ms", mean(ages), valid_n=len(ages))
    c.add("pose_age_median_ms", median(ages), valid_n=len(ages))
    c.add("pose_age_p95_ms", percentile(ages, 95), valid_n=len(ages))
    c.add("pose_age_max_ms", max(ages) if ages else None, valid_n=len(ages))
    if span:
        c.add("pose_update_rate_hz", len(telemetry) / span,
              numerator=len(telemetry), denominator=span)
    else:
        c.add("pose_update_rate_hz", None)
    # fresh coverage + dropout episodes per robot, merged fleet-wide
    if telemetry and span:
        by_robot: dict[str, list] = {}
        for t in telemetry:
            by_robot.setdefault(t["robot_id"], []).append(t)
        coverages = []
        dropouts = 0
        for robot, samples in by_robot.items():
            fresh_ms = 0.0
            in_dropout = False
            for i in range(1, len(samples)):
                gap = samples[i]["elapsed_ms"] - samples[i - 1]["elapsed_ms"]
                age = samples[i]["pose_age_ms"]
                stale = (gap > POSE_FRESH_MAX_MS or
                         (age is not None and age > POSE_FRESH_MAX_MS))
                if not stale:
                    fresh_ms += gap
                    in_dropout = False
                elif not in_dropout:
                    dropouts += 1
                    in_dropout = True
            coverages.append(min(100.0, 100.0 * (fresh_ms / 1000.0) / span))
        c.add("vision_availability_pct", mean(coverages),
              valid_n=len(coverages))
        c.add("vision_dropout_episodes", dropouts)
    else:
        c.add("vision_availability_pct", None)
        c.add("vision_dropout_episodes", None)


def _rotation_metrics(c, events):
    started = [e for e in events
               if e["event_type"] == "ROTATION_ATTEMPT_STARTED"]
    completed = [e for e in events
                 if e["event_type"] == "ROTATION_ATTEMPT_COMPLETED"]
    failed = [e for e in events
              if e["event_type"] == "ROTATION_ATTEMPT_FAILED"]
    nA = len(started) if started else (len(completed) + len(failed))
    valid = [e for e in completed
             if e["details"].get("error_deg") is not None]
    nV = len(valid)
    errors = [e["details"]["error_deg"] for e in valid]
    abs_errors = [abs(x) for x in errors]
    durations = [e["details"].get("duration_sec") for e in completed]
    durations_valid = [d for d in durations if d is not None]
    fail_durations = [e["details"].get("duration_sec") for e in failed]
    total_time = sum(d for d in durations + fail_durations if d is not None)

    def n_tau():
        n = 0
        for e in completed:
            wt = e["details"].get("within_tolerance")
            if wt is None:
                err = e["details"].get("error_deg")
                wt = err is not None and abs(err) <= YAW_TOLERANCE_DEG
            if wt:
                n += 1
        return n

    ntau = n_tau()
    if nA == 0:
        for key in ("yaw_mean_signed_error_deg", "yaw_mae_deg", "yaw_rmse_deg",
                    "yaw_error_std_deg", "yaw_abs_error_median_deg",
                    "yaw_abs_error_p95_deg", "yaw_abs_error_max_deg",
                    "rotation_completion_rate", "rotation_within_tolerance_rate",
                    "rotation_mean_duration_sec",
                    "acceptable_turn_throughput_per_min",
                    "rotation_time_error_product", "camera_correction_rate",
                    "rotation_failure_rate"):
            c.add(key, None)
        return
    c.add("yaw_mean_signed_error_deg", mean(errors), valid_n=nV,
          missing_n=nA - nV)
    mae = mean(abs_errors)
    c.add("yaw_mae_deg", mae, valid_n=nV, missing_n=nA - nV)
    c.add("yaw_rmse_deg",
          math.sqrt(sum(x * x for x in errors) / nV) if nV else None,
          valid_n=nV)
    # within-commanded-angle-group pooled std (spec 15.4)
    groups: dict[float, list] = {}
    for e in valid:
        target = e["details"].get("target_deg")
        groups.setdefault(round(target, 1) if target is not None else 0.0,
                          []).append(e["details"]["error_deg"])
    pooled_num = 0.0
    pooled_den = 0
    for xs in groups.values():
        s = sample_std(xs)
        if s is not None:
            pooled_num += (len(xs) - 1) * s * s
            pooled_den += len(xs) - 1
    c.add("yaw_error_std_deg",
          math.sqrt(pooled_num / pooled_den) if pooled_den else None,
          valid_n=nV)
    c.add("yaw_abs_error_median_deg", median(abs_errors), valid_n=nV)
    c.add("yaw_abs_error_p95_deg", percentile(abs_errors, 95), valid_n=nV)
    c.add("yaw_abs_error_max_deg", max(abs_errors) if abs_errors else None,
          valid_n=nV)
    c.add("rotation_completion_rate", _ratio(nV, nA), numerator=nV,
          denominator=nA, valid_n=nA)
    c.add("rotation_within_tolerance_rate", _ratio(ntau, nA), numerator=ntau,
          denominator=nA, valid_n=nA)
    mean_dur = mean(durations_valid)
    c.add("rotation_mean_duration_sec", mean_dur,
          valid_n=len(durations_valid))
    c.add("acceptable_turn_throughput_per_min",
          _ratio(60.0 * ntau, total_time) if total_time else None,
          numerator=ntau, denominator=total_time or None)
    c.add("rotation_time_error_product",
          mae * mean_dur if (mae is not None and mean_dur is not None) else None)
    eligible = [e for e in completed
                if e["details"].get("camera_assist_eligible")]
    corrected = [e for e in eligible if e["details"].get("corrected")]
    c.add("camera_correction_rate",
          _ratio(len(corrected), len(eligible)),
          numerator=len(corrected), denominator=len(eligible) or None)
    c.add("rotation_failure_rate", _ratio(nA - nV, nA), numerator=nA - nV,
          denominator=nA, valid_n=nA)


def _battery_metrics(c, robots, dist_by_robot, state_model, span, JC,
                     telemetry):
    drops = {}
    for r in robots:
        b0, b1 = r["start_battery_pct"], r["end_battery_pct"]
        if b0 is not None and b1 is not None:
            drop = b0 - b1  # negative rebounds stay visible (spec 16.1)
            drops[r["robot_id"]] = drop
            c.add("battery_drop_pct", drop, scope_type="robot",
                  scope_id=r["robot_id"], numerator=b0, denominator=b1)
            if span:
                c.add("battery_drain_rate_pct_per_min",
                      drop / (span / 60.0), scope_type="robot",
                      scope_id=r["robot_id"])
            sm = state_model.get(r["robot_id"])
            if sm:
                P = sum(sm["durations"].get(s, 0.0)
                        for s in PRODUCTIVE_STATES)
                if P > 0:
                    c.add("battery_drain_per_productive_min_pct",
                          drop / (P / 60.0), scope_type="robot",
                          scope_id=r["robot_id"])
        else:
            c.add("battery_drop_pct", None, scope_type="robot",
                  scope_id=r["robot_id"], status="missing")
    total_drop = sum(drops.values()) if drops else None
    c.add("battery_drain_per_job_pct", _ratio(total_drop, JC),
          numerator=total_drop, denominator=JC, valid_n=len(drops))
    D = sum(dist_by_robot.values()) if dist_by_robot else None
    c.add("battery_drain_per_meter_pct", _ratio(total_drop, D),
          numerator=total_drop, denominator=D)
    norm = [drops[r] / dist_by_robot[r] for r in drops
            if dist_by_robot.get(r)]
    s = sample_std(norm)
    m = mean(norm)
    c.add("battery_drain_imbalance_cv",
          (s / m) if (s is not None and m not in (None, 0)) else None,
          valid_n=len(norm))
    # electrical energy (16.6) -- only when voltage+current telemetry exists
    energy: dict[str, float] = {}
    last_t: dict[str, int] = {}
    for t in telemetry:
        if t["voltage_v"] is None or t["current_a"] is None:
            continue
        robot = t["robot_id"]
        if robot in last_t:
            dt = (t["elapsed_ms"] - last_t[robot]) / 1000.0
            energy[robot] = energy.get(robot, 0.0) + \
                t["voltage_v"] * t["current_a"] * dt / 3600.0
        else:
            energy.setdefault(robot, 0.0)
        last_t[robot] = t["elapsed_ms"]
    for robot, e in sorted(energy.items()):
        c.add("energy_wh", e, scope_type="robot", scope_id=robot)
    E = sum(energy.values()) if energy else None
    c.add("energy_per_job_wh", _ratio(E, JC), numerator=E, denominator=JC)
    c.add("energy_per_meter_wh", _ratio(E, D), numerator=E, denominator=D)


def _fairness_metrics(c, robots, jobs, N):
    """Default workload measure x_i = completed jobs per robot (the API's
    workload=... parameter can recompute with other measures client-side
    from the per-robot rows)."""
    if not robots or not N:
        for key in ("jain_fairness_index", "workload_cv",
                    "max_min_workload_ratio"):
            c.add(key, None)
        return
    x = []
    for r in robots:
        n = len([j for j in jobs if j["assigned_robot_id"] == r["robot_id"]
                 and j["status"] == "completed"])
        x.append(n)
    total = sum(x)
    if total <= 0:
        c.add("jain_fairness_index", None, valid_n=len(x),
              status="missing")
        c.add("workload_cv", None, valid_n=len(x), status="missing")
        c.add("max_min_workload_ratio", None, valid_n=len(x),
              status="missing")
        return
    c.add("jain_fairness_index",
          (total ** 2) / (len(x) * sum(v * v for v in x)),
          numerator=float(total ** 2),
          denominator=float(len(x) * sum(v * v for v in x)), valid_n=len(x))
    m = mean(x)
    s = sample_std(x)
    c.add("workload_cv", (s / m) if (s is not None and m) else None,
          valid_n=len(x))
    c.add("max_min_workload_ratio",
          (max(x) / min(x)) if min(x) > 0 else None, valid_n=len(x),
          status="ok" if min(x) > 0 else "missing")


def _workstation_metrics(c, events, run, span):
    starts: dict[str, list] = {}
    proc_time: dict[str, float] = {}
    dwells: dict[str, list] = {}
    for e in events:
        ws = e["workstation_id"]
        if not ws:
            continue
        if e["event_type"] == "PROCESSING_STARTED":
            starts.setdefault(ws, []).append(e["elapsed_ms"])
        elif e["event_type"] == "PROCESSING_COMPLETED":
            pending = starts.get(ws) or []
            if pending and e["elapsed_ms"] is not None and pending[0] is not None:
                t0 = pending.pop(0)
                dur = (e["elapsed_ms"] - t0) / 1000.0
                proc_time[ws] = proc_time.get(ws, 0.0) + dur
                dwells.setdefault(ws, []).append(dur)
    for ws in sorted(proc_time):
        if span:
            c.add("workstation_processing_utilization_pct",
                  100.0 * proc_time[ws] / span, scope_type="workstation",
                  scope_id=ws, numerator=proc_time[ws], denominator=span)
        prescribed = run.get("process_sec")
        if prescribed is not None and dwells.get(ws):
            c.add("excess_workstation_dwell_sec",
                  mean(dwells[ws]) - prescribed, scope_type="workstation",
                  scope_id=ws, valid_n=len(dwells[ws]))


# ---------------------------------------------------------------------------
# data quality (spec section 19)

def run_data_quality(conn: sqlite3.Connection, run_id: str,
                     bundle: dict | None = None) -> dict:
    if bundle is None:
        bundle = load_run_bundle(conn, run_id)
    events = bundle["events"]
    issues: list[dict] = []

    def issue(kind, detail):
        issues.append({"kind": kind, "detail": detail})

    seqs = [e["seq"] for e in events if e["seq"] is not None]
    if seqs:
        expected = set(range(min(seqs), max(seqs) + 1))
        missing = sorted(expected - set(seqs))
        if missing:
            issue("missing_sequence_numbers",
                  f"{len(missing)} missing seq value(s): {missing[:20]}")
        if len(seqs) != len(set(seqs)):
            issue("duplicate_events", "duplicate seq values present")
    prev = None
    for e in events:
        if e["elapsed_ms"] is None:
            continue
        if prev is not None and e["elapsed_ms"] < prev:
            issue("nonmonotonic_elapsed",
                  f"seq {e['seq']} elapsed_ms {e['elapsed_ms']} < previous {prev}")
        prev = e["elapsed_ms"]
    types = [e["event_type"] for e in events]
    if any(t in TERMINAL_RUN_EVENTS for t in types) and "RUN_STARTED" not in types:
        issue("end_without_start", "run end event without RUN_STARTED")
    for j in bundle["jobs"]:
        if j["completion_elapsed_ms"] is not None and j["release_elapsed_ms"] is None:
            issue("job_completion_without_release", f"job {j['job_id']}")
    depot_robots = {e["robot_id"] for e in events
                    if e["event_type"] == "DEPOT_ARRIVED"}
    for r in bundle["robots"]:
        if r["status"] == "completed" and r["robot_id"] not in depot_robots:
            issue("robot_completion_without_depot_arrival", r["robot_id"])
    for t in bundle["telemetry"]:
        if t["valid"] and t["x_m"] is not None and (
                not math.isfinite(t["x_m"]) or not math.isfinite(t["y_m"] or 0)):
            issue("invalid_position", f"{t['robot_id']} @ {t['elapsed_ms']}ms")
            break
    state_model = robot_state_durations(bundle)
    times = run_times(bundle)
    span = times["span_sec"]
    for robot, sm in state_model.items():
        if sm["issues"]:
            issue("state_transition_anomaly",
                  f"{robot}: {sm['issues']} out-of-order/blank transition(s)")
        if span:
            recon = abs(span - sum(sm["durations"].values()))
            if recon > STATE_RECONCILE_TOL_SEC:
                issue("state_time_reconciliation",
                      f"{robot}: states sum to {sum(sm['durations'].values()):.1f}s "
                      f"vs run span {span:.1f}s")

    # telemetry coverage (19.3)
    coverage = None
    if span and bundle["telemetry"]:
        covered: set[int] = set()
        for t in bundle["telemetry"]:
            if t["valid"]:
                covered.add(int(t["elapsed_ms"] // 1000))
        coverage = min(100.0, 100.0 * len(covered) / max(1.0, span))

    # attempt accounting (19.1) per measurement family
    def family(attempted, valid, completed, failed, aborted=0, missing=0,
               excluded=0):
        return {"attempted": attempted, "valid": valid,
                "completed": completed, "failed": failed,
                "aborted": aborted, "missing": missing,
                "excluded_with_reason": excluded}

    rot_started = len([e for e in events
                       if e["event_type"] == "ROTATION_ATTEMPT_STARTED"])
    rot_done = [e for e in events
                if e["event_type"] == "ROTATION_ATTEMPT_COMPLETED"]
    rot_failed = len([e for e in events
                      if e["event_type"] == "ROTATION_ATTEMPT_FAILED"])
    rot_attempted = rot_started or (len(rot_done) + rot_failed)
    rot_valid = len([e for e in rot_done
                     if e["details"].get("error_deg") is not None])
    cmds_sent = len([e for e in events if e["event_type"] == "COMMAND_SENT"])
    cmds_done = len([e for e in events
                     if e["event_type"] == "COMMAND_COMPLETED"])
    cmds_failed = len([e for e in events
                       if e["event_type"] == "COMMAND_FAILED"])
    jobs = bundle["jobs"]
    accounting = {
        "rotations": family(rot_attempted, rot_valid, len(rot_done),
                            rot_failed,
                            missing=len(rot_done) - rot_valid),
        "commands": family(cmds_sent, cmds_done, cmds_done, cmds_failed),
        "jobs": family(len(jobs),
                       len([j for j in jobs if j["status"] == "completed"]),
                       len([j for j in jobs if j["status"] == "completed"]),
                       len([j for j in jobs if j["status"] == "failed"]),
                       aborted=len([j for j in jobs
                                    if j["status"] == "aborted"])),
        "robots": family(len(bundle["robots"]),
                         len([r for r in bundle["robots"]
                              if r["status"] == "completed"]),
                         len([r for r in bundle["robots"]
                              if r["status"] == "completed"]),
                         len([r for r in bundle["robots"]
                              if r["status"] in ("failed", "offline")])),
    }
    return {"issues": issues, "attempt_accounting": accounting,
            "telemetry_coverage_pct": coverage,
            "thresholds": {
                "pose_fresh_max_ms": POSE_FRESH_MAX_MS,
                "separation_warn_m": SEPARATION_WARN_M,
                "separation_collision_m": SEPARATION_COLLISION_M,
                "terminal_heading_tol_deg": TERMINAL_HEADING_TOL_DEG,
                "state_reconcile_tol_sec": STATE_RECONCILE_TOL_SEC,
                "yaw_tolerance_deg": YAW_TOLERANCE_DEG,
            }}
