"""SQLite access layer for the AGV metrics backend.

Design rules (see the Page-4 metrics spec):
  * WAL journal mode, foreign keys enforced, parameterized SQL only.
  * Raw events + telemetry are the source of truth; everything else derives.
  * Event ingestion also *projects* rows into runs / run_robots / jobs /
    conflicts / telemetry_samples so those tables never need hand-writing.
  * Batched writes: ingest_events() takes a whole batch inside ONE
    transaction (executemany where possible). The robot-side never talks to
    SQLite at all -- fleet/metrics_events.py queues events in a background
    thread and POSTs batches over HTTP, so database work can never block a
    ROS callback.
"""
from __future__ import annotations

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DB_PATH = os.path.join(BASE_DIR, "data", "agv_metrics.sqlite3")
SCHEMA_PATH = os.path.join(BASE_DIR, "schema.sql")

RUN_STATUSES = ("created", "running", "completed", "failed", "aborted")

# elapsed-ms values are monotonic-clock durations from the emitter; wall
# timestamps (time_utc) are display-only, never used for duration math.


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def connect(db_path: str | None = None) -> sqlite3.Connection:
    """Open (creating if needed) the metrics DB with WAL + FK enforcement."""
    path = db_path or DEFAULT_DB_PATH
    if path != ":memory:":
        os.makedirs(os.path.dirname(path), exist_ok=True)
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode = WAL")
    conn.execute("PRAGMA foreign_keys = ON")
    conn.execute("PRAGMA busy_timeout = 5000")
    with open(SCHEMA_PATH, "r", encoding="utf-8") as fh:
        conn.executescript(fh.read())
    conn.commit()
    return conn


def new_run_id() -> str:
    return str(uuid.uuid4())


# --------------------------------------------------------------------------
# runs

RUN_FIELDS = (
    "experimental_condition", "algorithm", "drive_mode", "robot_count",
    "job_count", "workstation_count", "process_sec", "capacity",
    "random_seed", "planned_makespan_sec", "plan_json", "notes",
)


def create_run(conn: sqlite3.Connection, payload: dict) -> str:
    run_id = str(payload.get("run_id") or new_run_id())
    plan_json = payload.get("plan_json")
    if isinstance(plan_json, (dict, list)):
        plan_json = json.dumps(plan_json)
    values = {k: payload.get(k) for k in RUN_FIELDS}
    values["plan_json"] = plan_json
    conn.execute(
        """INSERT INTO runs (run_id, created_at_utc, status,
               experimental_condition, algorithm, drive_mode, robot_count,
               job_count, workstation_count, process_sec, capacity,
               random_seed, planned_makespan_sec, plan_json, notes,
               definition_set_version)
           VALUES (?, ?, 'created', ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
        (run_id, utc_now_iso(),
         values["experimental_condition"], values["algorithm"],
         values["drive_mode"], values["robot_count"], values["job_count"],
         values["workstation_count"], values["process_sec"],
         values["capacity"], values["random_seed"],
         values["planned_makespan_sec"], values["plan_json"],
         values["notes"], int(payload.get("definition_set_version") or 1)))
    conn.commit()
    return run_id


def get_run(conn: sqlite3.Connection, run_id: str) -> sqlite3.Row | None:
    return conn.execute("SELECT * FROM runs WHERE run_id = ?",
                        (run_id,)).fetchone()


def update_run(conn: sqlite3.Connection, run_id: str, patch: dict) -> bool:
    """PATCH a run row. Status transitions are idempotent-friendly:
    started_at/ended_at are only filled if currently NULL (COALESCE) so the
    browser fallback and the supervisor's authoritative event can both fire
    without clobbering each other."""
    allowed = set(RUN_FIELDS) | {"status", "started_at_utc", "ended_at_utc"}
    sets, args = [], []
    for key, value in patch.items():
        if key not in allowed:
            continue
        if key == "status" and value not in RUN_STATUSES:
            raise ValueError(f"bad status {value!r}")
        if key in ("started_at_utc", "ended_at_utc"):
            sets.append(f"{key} = COALESCE({key}, ?)")
        else:
            sets.append(f"{key} = ?")
        if key == "plan_json" and isinstance(value, (dict, list)):
            value = json.dumps(value)
        args.append(value)
    if not sets:
        return False
    args.append(run_id)
    cur = conn.execute(f"UPDATE runs SET {', '.join(sets)} WHERE run_id = ?",
                       args)
    conn.commit()
    return cur.rowcount > 0


# --------------------------------------------------------------------------
# event ingestion + projection

EVENT_COLUMNS = (
    "seq", "elapsed_ms", "time_utc", "event_type", "robot_id", "job_id",
    "workstation_id", "conflict_id", "command_id", "state_from", "state_to",
    "reason", "x_m", "y_m", "yaw_deg", "battery_pct", "pose_age_ms",
)

TERMINAL_EVENT_STATUS = {
    "RUN_COMPLETED": "completed",
    "RUN_FAILED": "failed",
    "RUN_ABORTED": "aborted",
}


def ingest_events(conn: sqlite3.Connection, run_id: str,
                  events: list[dict]) -> dict:
    """Insert a batch of structured events in one transaction, projecting
    side effects into runs / run_robots / jobs / conflicts /
    telemetry_samples. Duplicate (run_id, seq) rows are skipped (rosbridge /
    HTTP retries may re-deliver), counted in the result."""
    inserted = 0
    duplicates = 0
    errors: list[str] = []
    try:
        for ev in events:
            if not isinstance(ev, dict) or not ev.get("event_type"):
                errors.append(f"bad event (no event_type): {ev!r}")
                continue
            details = ev.get("details")
            details_json = json.dumps(details) if details not in (None, {}) else None
            row = tuple(ev.get(c) for c in EVENT_COLUMNS)
            try:
                conn.execute(
                    f"""INSERT INTO events (run_id, {', '.join(EVENT_COLUMNS)},
                            details_json)
                        VALUES (?{', ?' * len(EVENT_COLUMNS)}, ?)""",
                    (run_id, *row, details_json))
            except sqlite3.IntegrityError:
                duplicates += 1
                continue
            inserted += 1
            _project_event(conn, run_id, ev)
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    return {"inserted": inserted, "duplicates": duplicates, "errors": errors}


def _ensure_robot(conn, run_id, robot_id) -> None:
    if not robot_id:
        return
    conn.execute(
        """INSERT OR IGNORE INTO run_robots (run_id, robot_id, status)
           VALUES (?, ?, 'registered')""", (run_id, robot_id))


def _ensure_job(conn, run_id, job_id) -> None:
    if not job_id:
        return
    conn.execute(
        """INSERT OR IGNORE INTO jobs (run_id, job_id, status)
           VALUES (?, ?, 'released')""", (run_id, job_id))


def _project_event(conn: sqlite3.Connection, run_id: str, ev: dict) -> None:
    """Apply an event's side effects to the projection tables. Raw events
    stay the source of truth -- projections can always be rebuilt from them."""
    etype = ev.get("event_type")
    robot = ev.get("robot_id")
    job = ev.get("job_id")
    elapsed = ev.get("elapsed_ms")
    details = ev.get("details") or {}

    if etype == "RUN_STARTED":
        conn.execute(
            """UPDATE runs SET status = 'running',
                   started_at_utc = COALESCE(started_at_utc, ?)
               WHERE run_id = ? AND status IN ('created', 'running')""",
            (ev.get("time_utc") or utc_now_iso(), run_id))
    elif etype in TERMINAL_EVENT_STATUS:
        conn.execute(
            """UPDATE runs SET status = ?,
                   ended_at_utc = COALESCE(ended_at_utc, ?)
               WHERE run_id = ?""",
            (TERMINAL_EVENT_STATUS[etype],
             ev.get("time_utc") or utc_now_iso(), run_id))

    if robot:
        _ensure_robot(conn, run_id, robot)
    if etype == "ROBOT_REGISTERED":
        conn.execute(
            """UPDATE run_robots SET status = 'active', capacity = ?,
                   start_battery_pct = COALESCE(start_battery_pct, ?)
               WHERE run_id = ? AND robot_id = ?""",
            (details.get("capacity"), ev.get("battery_pct"), run_id, robot))
    elif etype == "ROBOT_OFFLINE":
        conn.execute(
            """UPDATE run_robots SET status = 'offline', failure_reason = ?
               WHERE run_id = ? AND robot_id = ?""",
            (ev.get("reason"), run_id, robot))
    elif etype == "COMMAND_FAILED" and robot:
        conn.execute(
            """UPDATE run_robots SET status = 'failed',
                   failure_reason = COALESCE(failure_reason, ?)
               WHERE run_id = ? AND robot_id = ?""",
            (ev.get("reason"), run_id, robot))
    elif etype == "BATTERY_SAMPLE" and robot:
        conn.execute(
            """UPDATE run_robots SET
                   start_battery_pct = COALESCE(start_battery_pct, ?),
                   end_battery_pct = ?
               WHERE run_id = ? AND robot_id = ?""",
            (ev.get("battery_pct"), ev.get("battery_pct"), run_id, robot))
    elif etype in ("DEPOT_ARRIVED", "TERMINAL_HEADING_REACHED") and robot:
        conn.execute(
            """UPDATE run_robots SET status = 'completed',
                   completion_elapsed_ms = COALESCE(?, completion_elapsed_ms),
                   terminal_x_m = COALESCE(?, terminal_x_m),
                   terminal_y_m = COALESCE(?, terminal_y_m),
                   terminal_yaw_deg = COALESCE(?, terminal_yaw_deg)
               WHERE run_id = ? AND robot_id = ?""",
            (elapsed, ev.get("x_m"), ev.get("y_m"), ev.get("yaw_deg"),
             run_id, robot))

    if job:
        _ensure_job(conn, run_id, job)
    job_time_col = {
        "JOB_RELEASED": "release_elapsed_ms",
        "PICKUP_COMPLETED": "pickup_elapsed_ms",
        "DROPOFF_COMPLETED": "delivery_elapsed_ms",
    }.get(etype)
    if job_time_col and job:
        conn.execute(
            f"""UPDATE jobs SET {job_time_col} = COALESCE({job_time_col}, ?),
                    assigned_robot_id = COALESCE(assigned_robot_id, ?),
                    workstation_id = COALESCE(workstation_id, ?)
                WHERE run_id = ? AND job_id = ?""",
            (elapsed, robot, ev.get("workstation_id"), run_id, job))
    if etype == "JOB_RELEASED" and job and details.get("due_elapsed_ms") is not None:
        conn.execute(
            """UPDATE jobs SET due_elapsed_ms = ? WHERE run_id = ? AND job_id = ?""",
            (details.get("due_elapsed_ms"), run_id, job))
    if etype == "PROCESSING_COMPLETED" and job:
        conn.execute(
            """UPDATE jobs SET status = 'completed',
                   completion_elapsed_ms = COALESCE(completion_elapsed_ms, ?)
               WHERE run_id = ? AND job_id = ?""",
            (elapsed, run_id, job))
    if etype == "DROPOFF_COMPLETED" and job:
        # A pure-delivery job completes at drop-off unless processing /
        # pickup events later refine it.
        conn.execute(
            """UPDATE jobs SET status = 'completed',
                   completion_elapsed_ms = COALESCE(completion_elapsed_ms, ?)
               WHERE run_id = ? AND job_id = ? AND status != 'failed'""",
            (elapsed, run_id, job))
    if etype == "COMMAND_FAILED" and job:
        conn.execute(
            """UPDATE jobs SET status = 'failed',
                   failure_reason = COALESCE(failure_reason, ?)
               WHERE run_id = ? AND job_id = ?""",
            (ev.get("reason"), run_id, job))

    conflict = ev.get("conflict_id")
    if etype in ("CONFLICT_DETECTED", "NEAR_MISS_STARTED",
                 "DEADLOCK_DETECTED") and conflict:
        conn.execute(
            """INSERT OR IGNORE INTO conflicts (run_id, conflict_id, type,
                   severity, detected_elapsed_ms, robot_ids_json,
                   resource_id, minimum_separation_m, status)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'open')""",
            (run_id, conflict,
             details.get("type") or ("deadlock" if "DEADLOCK" in etype
                                     else "near_miss" if "NEAR_MISS" in etype
                                     else "conflict"),
             details.get("severity"), elapsed,
             json.dumps(details.get("robot_ids") or ([robot] if robot else [])),
             details.get("resource_id"),
             details.get("minimum_separation_m")))
    elif etype in ("CONFLICT_RESOLVED", "NEAR_MISS_ENDED",
                   "DEADLOCK_RESOLVED") and conflict:
        conn.execute(
            """UPDATE conflicts SET status = 'resolved',
                   resolved_elapsed_ms = COALESCE(resolved_elapsed_ms, ?),
                   resolution = COALESCE(resolution, ?)
               WHERE run_id = ? AND conflict_id = ?""",
            (elapsed, ev.get("reason") or details.get("resolution"),
             run_id, conflict))

    if etype == "POSE_SAMPLE":
        conn.execute(
            """INSERT INTO telemetry_samples (run_id, robot_id, elapsed_ms,
                   x_m, y_m, yaw_deg, battery_pct, voltage_v, current_a,
                   pose_age_ms, source, valid)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (run_id, robot, elapsed or 0, ev.get("x_m"), ev.get("y_m"),
             ev.get("yaw_deg"), ev.get("battery_pct"),
             details.get("voltage_v"), details.get("current_a"),
             ev.get("pose_age_ms"), details.get("source") or "vision",
             1 if details.get("valid", True) else 0))


# --------------------------------------------------------------------------
# metric values

def store_metric_values(conn: sqlite3.Connection, run_id: str,
                        values: list[dict], status: str,
                        revision: int = 0) -> int:
    """Write computed metric values. 'provisional' rows for the same
    revision are replaced freely; 'final' rows are only written by
    finalize/recalculate paths (finalize refuses if final rows exist,
    recalculate bumps the revision -- enforced by the API layer)."""
    now = utc_now_iso()
    count = 0
    for mv in values:
        conn.execute(
            """INSERT OR REPLACE INTO metric_values
                   (run_id, scope_type, scope_id, metric_key,
                    definition_version, revision, value, unit, numerator,
                    denominator, valid_n, missing_n, status, computed_at_utc)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (run_id, mv["scope_type"], mv.get("scope_id") or "",
             mv["metric_key"], mv["definition_version"], revision,
             mv.get("value"), mv.get("unit"), mv.get("numerator"),
             mv.get("denominator"), mv.get("valid_n"), mv.get("missing_n"),
             mv.get("status") or status, now))
        count += 1
    conn.commit()
    return count


def max_final_revision(conn: sqlite3.Connection, run_id: str) -> int | None:
    row = conn.execute(
        """SELECT MAX(revision) AS rev FROM metric_values
           WHERE run_id = ? AND status = 'final'""", (run_id,)).fetchone()
    return row["rev"] if row and row["rev"] is not None else None
