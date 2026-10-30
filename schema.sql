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
    -- Snapshot of the CONTROL TUNING this run executed under: the 47 motion
    -- parameters handed to each navigator, plus the coordination, vision and
    -- localization constants. Captured at run start, because these change
    -- between runs and nothing else in this row records which values were
    -- live -- three different motion controllers were used across the runs
    -- recorded before 2026-10-14 and the data cannot say which was which.
    tuning_json            TEXT,
    -- The DOE's treatment: which traffic planner produced the dispatched
    -- routes (page_wait | astar | wait | pp | cbs). Separate from
    -- `algorithm`, which names the JOB planner whose allocation was frozen.
    traffic_method         TEXT,
    -- The saved plan file the run was dispatched from (e.g. DOE_6r_cbs.json):
    -- with frozen DOE plans, this IS the treatment, exactly.
    plan_file              TEXT,
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

-- One row per recorded video of the annotated overhead camera stream
-- (video_recorder.py). The file lives in data/videos/; this row is the index
-- the Recordings page reads. run_id is NULL for a manual recording made with
-- no run armed.
CREATE TABLE IF NOT EXISTS run_videos (
    video_id           INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id             TEXT REFERENCES runs(run_id),
    title              TEXT NOT NULL,
    file_name          TEXT NOT NULL,
    -- recording | ready | failed | interrupted
    status             TEXT NOT NULL DEFAULT 'recording',
    -- auto (a run's start/end signals) | manual (Record now)
    started_by         TEXT,
    stop_reason        TEXT,
    started_at_utc     TEXT NOT NULL,
    ended_at_utc       TEXT,
    duration_sec       REAL,
    fps                REAL,
    width              INTEGER,
    height             INTEGER,
    frame_count        INTEGER,
    -- distinct frames that arrived from the stream (the rest are repeats)
    camera_frames      INTEGER,
    stall_count        INTEGER,
    max_stall_sec      REAL,
    codec              TEXT,
    browser_playable   INTEGER,
    bytes              INTEGER,
    -- Video time (s) at which the run's elapsed clock reads 0: an event at
    -- elapsed_ms plays at run_t0_offset_sec + elapsed_ms / 1000.
    run_t0_offset_sec  REAL,
    -- supervisor (RUN_STARTED) | browser (the Start button)
    offset_source      TEXT,
    stream_url         TEXT,
    error              TEXT,
    notes              TEXT,
    -- The same run seen by the second overhead camera, recorded on its own
    -- laptop (overhead_cam/overhead_recorder.py) and played from there.
    -- second_offset_sec: this video's time at which the second video's
    -- 0:00 plays.
    second_camera_url  TEXT,
    second_camera_file TEXT,
    second_offset_sec  REAL,
    second_error       TEXT
);
CREATE INDEX IF NOT EXISTS idx_run_videos_run ON run_videos(run_id);
