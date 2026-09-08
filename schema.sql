-- AGV testbed performance-metrics database (SQLite, WAL mode).
-- Canonical units everywhere: seconds, meters, degrees, percent, watt-hours.
-- Raw events + telemetry are the source of truth; metric_values are derived
-- and versioned against metric_definitions (never overwritten once final).

PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS runs (
    run_id                 TEXT PRIMARY KEY,
    created_at_utc         TEXT NOT NULL,
    started_at_utc         TEXT,
    ended_at_utc           TEXT,
    -- created | running | completed | failed | aborted
    status                 TEXT NOT NULL DEFAULT 'created',
    experimental_condition TEXT,
    algorithm              TEXT,
    drive_mode             TEXT,
    robot_count            INTEGER,
    job_count              INTEGER,
    workstation_count      INTEGER,
    process_sec            REAL,
    capacity               INTEGER,
    random_seed            INTEGER,
    planned_makespan_sec   REAL,
    plan_json              TEXT,
    notes                  TEXT,
    definition_set_version INTEGER NOT NULL DEFAULT 1
);

CREATE TABLE IF NOT EXISTS run_robots (
    run_id                TEXT NOT NULL REFERENCES runs(run_id),
    robot_id              TEXT NOT NULL,
    capacity              INTEGER,
    start_battery_pct     REAL,
    end_battery_pct       REAL,
    completion_elapsed_ms INTEGER,
    terminal_x_m          REAL,
    terminal_y_m          REAL,
    terminal_yaw_deg      REAL,
    -- registered | active | completed | failed | offline
    status                TEXT NOT NULL DEFAULT 'registered',
    failure_reason        TEXT,
    PRIMARY KEY (run_id, robot_id)
);

CREATE TABLE IF NOT EXISTS jobs (
    run_id                TEXT NOT NULL REFERENCES runs(run_id),
    job_id                TEXT NOT NULL,
    assigned_robot_id     TEXT,
    workstation_id        TEXT,
    release_elapsed_ms    INTEGER,
    pickup_elapsed_ms     INTEGER,
    delivery_elapsed_ms   INTEGER,
    completion_elapsed_ms INTEGER,
    due_elapsed_ms        INTEGER,
    -- released | in_progress | completed | failed | aborted
    status                TEXT NOT NULL DEFAULT 'released',
    failure_reason        TEXT,
    PRIMARY KEY (run_id, job_id)
);

CREATE TABLE IF NOT EXISTS events (
    event_id       INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id         TEXT NOT NULL REFERENCES runs(run_id),
    seq            INTEGER NOT NULL,
    elapsed_ms     INTEGER,
    time_utc       TEXT,
    event_type     TEXT NOT NULL,
    robot_id       TEXT,
    job_id         TEXT,
    workstation_id TEXT,
    conflict_id    TEXT,
    command_id     TEXT,
    state_from     TEXT,
    state_to       TEXT,
    reason         TEXT,
    x_m            REAL,
    y_m            REAL,
    yaw_deg        REAL,
    battery_pct    REAL,
    pose_age_ms    REAL,
    details_json   TEXT,
    UNIQUE (run_id, seq)
);
CREATE INDEX IF NOT EXISTS idx_events_run_type ON events(run_id, event_type);
CREATE INDEX IF NOT EXISTS idx_events_run_robot ON events(run_id, robot_id);

CREATE TABLE IF NOT EXISTS telemetry_samples (
    run_id      TEXT NOT NULL REFERENCES runs(run_id),
    robot_id    TEXT NOT NULL,
    elapsed_ms  INTEGER NOT NULL,
    x_m         REAL,
    y_m         REAL,
    yaw_deg     REAL,
    battery_pct REAL,
    voltage_v   REAL,
    current_a   REAL,
    pose_age_ms REAL,
    source      TEXT,
    valid       INTEGER NOT NULL DEFAULT 1
);
CREATE INDEX IF NOT EXISTS idx_telemetry_run_robot
    ON telemetry_samples(run_id, robot_id, elapsed_ms);

CREATE TABLE IF NOT EXISTS conflicts (
    run_id               TEXT NOT NULL REFERENCES runs(run_id),
    conflict_id          TEXT NOT NULL,
    type                 TEXT,
    severity             TEXT,
    detected_elapsed_ms  INTEGER,
    resolved_elapsed_ms  INTEGER,
    robot_ids_json       TEXT,
    resource_id          TEXT,
    minimum_separation_m REAL,
    resolution           TEXT,
    -- open | resolved | unresolved
    status               TEXT NOT NULL DEFAULT 'open',
    PRIMARY KEY (run_id, conflict_id)
);

CREATE TABLE IF NOT EXISTS metric_definitions (
    metric_key           TEXT NOT NULL,
    version              INTEGER NOT NULL,
    display_name         TEXT NOT NULL,
    -- run | robot | job | workstation | conflict | aggregate
    scope                TEXT NOT NULL,
    description          TEXT NOT NULL,
    formula_latex        TEXT,
    unit                 TEXT,
    -- lower | higher | neutral
    direction            TEXT NOT NULL DEFAULT 'neutral',
    required_fields_json TEXT,
    null_policy          TEXT,
    effective_at_utc     TEXT NOT NULL,
    PRIMARY KEY (metric_key, version)
);

CREATE TABLE IF NOT EXISTS metric_values (
    run_id             TEXT NOT NULL REFERENCES runs(run_id),
    -- run | robot | job | workstation | conflict
    scope_type         TEXT NOT NULL,
    scope_id           TEXT NOT NULL DEFAULT '',
    metric_key         TEXT NOT NULL,
    definition_version INTEGER NOT NULL,
    revision           INTEGER NOT NULL DEFAULT 0,
    value              REAL,
    unit               TEXT,
    numerator          REAL,
    denominator        REAL,
    valid_n            INTEGER,
    missing_n          INTEGER,
    -- provisional | final | missing
    status             TEXT NOT NULL DEFAULT 'provisional',
    computed_at_utc    TEXT NOT NULL,
    PRIMARY KEY (run_id, scope_type, scope_id, metric_key, definition_version, revision)
);
