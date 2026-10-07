"""Flask backend for the AGV solver + Page 4 Performance Metrics.

Serves the (refactored) solver HTML and the metrics API from one origin:

    python app.py            # http://localhost:8000/

The solver/simulator (Pages 1-3) never depend on any /api endpoint -- if
this process is down the static page can still be served by any web server
and Page 4 alone shows "Metrics backend unavailable".

The supervisor on the Linux laptop POSTs structured events here (see
fleet/metrics_events.py); the server listens on 0.0.0.0 so that works over
the LAN. Windows Firewall must allow inbound TCP 8000 for that (same
procedure as the port-8081 stream rule):
    New-NetFirewallRule -DisplayName "AGV metrics backend (TCP 8000)" `
        -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8000
"""
from __future__ import annotations

import csv
import io
import json
import sys

from flask import Flask, g, jsonify, redirect, render_template, request

import database
import metrics_engine
import vlm_chat
from metrics_definitions import (DEFINITION_SET_VERSION, DEFINITIONS,
                                 seed_definitions)

DEFAULT_PORT = 8000


def create_app(db_path: str | None = None) -> Flask:
    app = Flask(__name__)
    app.config["DB_PATH"] = db_path or database.DEFAULT_DB_PATH

    # Seed the definition registry once at startup (INSERT OR IGNORE -- an
    # existing (metric_key, version) row is never modified).
    conn = database.connect(app.config["DB_PATH"])
    seed_definitions(conn)
    conn.close()

    def db():
        if "db" not in g:
            g.db = database.connect(app.config["DB_PATH"])
        return g.db

    @app.teardown_appcontext
    def close_db(_exc):
        conn = g.pop("db", None)
        if conn is not None:
            conn.close()

    @app.after_request
    def cors(resp):
        # Local-tool convenience: lets the page keep working against this
        # API even when served by a different local server (http.server).
        if request.path.startswith("/api/"):
            resp.headers["Access-Control-Allow-Origin"] = "*"
            resp.headers["Access-Control-Allow-Methods"] = "GET, POST, PATCH, OPTIONS"
            resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return resp

    @app.route("/api/<path:_any>", methods=["OPTIONS"])
    def options_ok(_any):
        return "", 204

    # ---- pages -------------------------------------------------------------

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/agv_grid_workstation_solver.html")
    def legacy_url():
        # Old http.server bookmark keeps working.
        return redirect("/", code=302)

    # ---- VLM chat ----------------------------------------------------------
    # Lazily constructed so importing app.py (tests) never touches Ollama or
    # the camera, and deliberately isolated: every handler below returns a
    # JSON error rather than raising, so a VLM or camera outage can never
    # take down the metrics API that the supervisor is posting to.
    def vlm():
        if "vlm" not in app.config:
            app.config["vlm"] = vlm_chat.VlmChat()
        return app.config["vlm"]

    @app.route("/api/vlm/status")
    def vlm_status():
        try:
            return jsonify(vlm().status())
        except Exception as exc:
            return jsonify({"ollama_ok": False, "camera_ok": False,
                            "error": f"{type(exc).__name__}: {exc}"})

    @app.route("/api/vlm/chat", methods=["POST"])
    def vlm_chat_route():
        payload = request.get_json(force=True, silent=True) or {}
        question = str(payload.get("question") or "").strip()
        if not question:
            return jsonify({"ok": False, "error": "question is required"}), 400
        if len(question) > 2000:
            return jsonify({"ok": False,
                            "error": "question too long (max 2000 chars)"}), 400
        try:
            return jsonify(vlm().ask(
                question,
                state=payload.get("state") or {},
                history=payload.get("history") or []))
        except Exception as exc:
            return jsonify({"ok": False,
                            "error": f"{type(exc).__name__}: {exc}"})

    @app.route("/api/health")
    def health():
        row = db().execute("SELECT COUNT(*) AS n FROM runs").fetchone()
        return jsonify({"ok": True, "runs": row["n"],
                        "definition_set_version": DEFINITION_SET_VERSION})

    # ---- runs --------------------------------------------------------------

    @app.route("/api/runs", methods=["POST"])
    def create_run():
        payload = request.get_json(force=True, silent=True) or {}
        # Re-arming a mission must NOT strand the row created by the previous
        # arm. If the caller names a run it already owns and that run never
        # started and carries no events, rewrite it in place and hand back
        # the same id. Anything else -- a started run, a run with telemetry,
        # an unknown id -- falls through to a fresh row, so a genuinely new
        # mission after a finished one still gets its own record.
        reuse = str(payload.pop("reuse_run_id", "") or "")
        if reuse:
            prior = database.get_run(db(), reuse)
            has_events = db().execute(
                "SELECT 1 FROM events WHERE run_id = ? LIMIT 1",
                (reuse,)).fetchone() is not None
            if (prior is not None and prior["started_at_utc"] is None
                    and prior["status"] == "created" and not has_events):
                database.update_run(db(), reuse, payload)
                return jsonify({"run_id": reuse, "reused": True}), 200
        run_id = database.create_run(db(), payload)
        return jsonify({"run_id": run_id}), 201

    def _run_filters(executed_default=False):
        """WHERE clause + args from query params (shared by list/history/
        export). All comparisons parameterized.

        executed_default gates the EXECUTED-RUN RULE: a row counts as an
        attempt only if it has a start timestamp and at least one event.
        A run row is created when a mission is ARMED so the supervisor can be
        handed a run_id, which means arming twice, or arming and walking
        away, leaves a row that never corresponded to robots moving. As of
        2026-09-22 the database held 50 rows of which 17 were of that kind,
        three of them recorded as 'completed' with zero events, and a naive
        aggregate over them silently returns NULL makespans.

        The rule is applied to the ANALYSIS surfaces (history, export) and
        deliberately NOT to /api/runs: the run selector has to show a run
        that has just been armed and has not started yet, which is precisely
        what this rule excludes. `outcome=include_nonexecuted` turns it off.
        Nothing is ever deleted -- the rows stay, and the count of what was
        hidden is reported alongside the results.
        """
        q = request.args
        where, args = [], []
        if executed_default and q.get("outcome") != "include_nonexecuted":
            where.append("r.started_at_utc IS NOT NULL")
            where.append("EXISTS (SELECT 1 FROM events e "
                         "WHERE e.run_id = r.run_id)")
        simple = {
            "run_id": "r.run_id", "status": "r.status",
            "experimental_condition": "r.experimental_condition",
            "algorithm": "r.algorithm", "drive_mode": "r.drive_mode",
        }
        for param, col in simple.items():
            if q.get(param):
                where.append(f"{col} = ?")
                args.append(q.get(param))
        numeric = {
            "robot_count": "r.robot_count",
            "workstation_count": "r.workstation_count",
            "capacity": "r.capacity", "random_seed": "r.random_seed",
        }
        for param, col in numeric.items():
            if q.get(param):
                where.append(f"{col} = ?")
                args.append(int(q.get(param)))
        if q.get("process_sec"):
            where.append("r.process_sec = ?")
            args.append(float(q.get("process_sec")))
        if q.get("date_from"):
            where.append("r.created_at_utc >= ?")
            args.append(q.get("date_from"))
        if q.get("date_to"):
            where.append("r.created_at_utc <= ?")
            args.append(q.get("date_to") + ("T23:59:59" if len(q.get("date_to")) == 10 else ""))
        if q.get("outcome") == "successful":
            where.append("r.status = 'completed'")
        elif q.get("outcome") == "failed":
            where.append("r.status = 'failed'")
        elif q.get("outcome") == "aborted":
            where.append("r.status = 'aborted'")
        if q.get("robot"):
            where.append("EXISTS (SELECT 1 FROM run_robots rr "
                         "WHERE rr.run_id = r.run_id AND rr.robot_id = ?)")
            args.append(q.get("robot"))
        return (" WHERE " + " AND ".join(where)) if where else "", args

    def _paginate():
        limit = min(500, max(1, int(request.args.get("limit", 100))))
        offset = max(0, int(request.args.get("offset", 0)))
        return limit, offset

    @app.route("/api/runs")
    def list_runs():
        where, args = _run_filters()
        limit, offset = _paginate()
        total = db().execute(f"SELECT COUNT(*) AS n FROM runs r{where}",
                             args).fetchone()["n"]
        rows = db().execute(
            f"""SELECT r.run_id, r.created_at_utc, r.started_at_utc,
                       r.ended_at_utc, r.status, r.experimental_condition,
                       r.algorithm, r.drive_mode, r.robot_count, r.job_count,
                       r.workstation_count, r.process_sec, r.capacity,
                       r.random_seed, r.planned_makespan_sec, r.notes
                FROM runs r{where}
                ORDER BY r.created_at_utc DESC LIMIT ? OFFSET ?""",
            (*args, limit, offset)).fetchall()
        return jsonify({"total": total, "limit": limit, "offset": offset,
                        "runs": [dict(r) for r in rows]})

    @app.route("/api/runs/<run_id>")
    def get_run(run_id):
        row = database.get_run(db(), run_id)
        if row is None:
            return jsonify({"error": "unknown run"}), 404
        out = dict(row)
        if request.args.get("include_plan") != "1":
            out.pop("plan_json", None)
        return jsonify(out)

    @app.route("/api/runs/<run_id>", methods=["PATCH"])
    def patch_run(run_id):
        if database.get_run(db(), run_id) is None:
            return jsonify({"error": "unknown run"}), 404
        patch = request.get_json(force=True, silent=True) or {}
        # A status move to a terminal state stamps ended_at; running stamps
        # started_at (both COALESCE -- idempotent with supervisor events).
        if patch.get("status") == "running":
            patch.setdefault("started_at_utc", database.utc_now_iso())
        if patch.get("status") in ("completed", "failed", "aborted"):
            patch.setdefault("ended_at_utc", database.utc_now_iso())
        # A row that never started cannot have finished. The browser's
        # /fleet_status fallback fires on the supervisor reporting "finished"
        # and used to stamp whichever run the session last created, which is
        # how three zero-event rows came to be recorded as 'completed'.
        if patch.get("status") in ("completed", "failed"):
            row = database.get_run(db(), run_id)
            if row is not None and row["started_at_utc"] is None \
                    and patch.get("started_at_utc") is None:
                return jsonify({
                    "error": "run never started; refusing to mark it "
                             f"{patch['status']}",
                    "run_id": run_id}), 409
        try:
            database.update_run(db(), run_id, patch)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify(dict(database.get_run(db(), run_id)))

    # ---- events ------------------------------------------------------------

    TERMINAL_EVENTS = ("RUN_COMPLETED", "RUN_FAILED", "RUN_ABORTED")

    def _autofinalize(run_id):
        """Store final metric values for a run the supervisor just ended.

        Finalization used to happen in exactly one place: the browser's
        /fleet_status fallback, for the run THAT browser session created. Any
        run that ended while page 4 was closed, or that was aborted, or whose
        session had moved on, kept status='provisional' forever even with
        every event present -- 18 of 33 executed runs as of 2026-09-22,
        including the r4 reference run, which is why its makespan read as a
        dash on a page that held all the data needed to compute it.

        The supervisor is the authority on when a run ended, so the run's own
        terminal event is the right trigger. Never overwrites: a run that
        already has a final revision is left alone, and /recalculate remains
        the only way to create another.
        """
        try:
            if database.max_final_revision(db(), run_id) is not None:
                return
            run = database.get_run(db(), run_id)
            if run is None or run["status"] in ("created", "running"):
                return
            values = metrics_engine.compute_run_metrics(db(), run_id)
            for v in values:
                if v["status"] == "ok":
                    v["status"] = "final"
            database.store_metric_values(db(), run_id, values, "final",
                                         revision=0)
        except Exception:                       # never break event ingest
            app.logger.exception("auto-finalize failed for %s", run_id)

    def _ingest(run_id, events):
        if database.get_run(db(), run_id) is None:
            return jsonify({"error": "unknown run"}), 404
        result = database.ingest_events(db(), run_id, events)
        if any(str(e.get("event_type")) in TERMINAL_EVENTS
               for e in events if isinstance(e, dict)):
            _autofinalize(run_id)
        code = 200 if not result["errors"] else 207
        return jsonify(result), code

    @app.route("/api/runs/<run_id>/events", methods=["POST"])
    def post_event(run_id):
        body = request.get_json(force=True, silent=True)
        if body is None:
            return jsonify({"error": "JSON body required"}), 400
        events = body if isinstance(body, list) else [body]
        return _ingest(run_id, events)

    @app.route("/api/runs/<run_id>/events/batch", methods=["POST"])
    def post_event_batch(run_id):
        body = request.get_json(force=True, silent=True) or {}
        events = body.get("events")
        if not isinstance(events, list):
            return jsonify({"error": "body must be {\"events\": [...]}"}), 400
        return _ingest(run_id, events)

    @app.route("/api/runs/<run_id>/events")
    def list_events(run_id):
        limit, offset = _paginate()
        where = "WHERE run_id = ?"
        args = [run_id]
        if request.args.get("event_type"):
            where += " AND event_type = ?"
            args.append(request.args.get("event_type"))
        if request.args.get("robot_id"):
            where += " AND robot_id = ?"
            args.append(request.args.get("robot_id"))
        total = db().execute(f"SELECT COUNT(*) AS n FROM events {where}",
                             args).fetchone()["n"]
        rows = db().execute(
            f"SELECT * FROM events {where} ORDER BY seq LIMIT ? OFFSET ?",
            (*args, limit, offset)).fetchall()
        return jsonify({"total": total, "limit": limit, "offset": offset,
                        "events": [dict(r) for r in rows]})

    @app.route("/api/runs/<run_id>/robots")
    def list_robots(run_id):
        rows = db().execute(
            "SELECT * FROM run_robots WHERE run_id = ? ORDER BY robot_id",
            (run_id,)).fetchall()
        return jsonify({"robots": [dict(r) for r in rows]})

    @app.route("/api/runs/<run_id>/jobs")
    def list_jobs(run_id):
        rows = db().execute(
            "SELECT * FROM jobs WHERE run_id = ? ORDER BY job_id",
            (run_id,)).fetchall()
        return jsonify({"jobs": [dict(r) for r in rows]})

    # ---- metrics -----------------------------------------------------------

    def _stored_final(run_id):
        rev = database.max_final_revision(db(), run_id)
        if rev is None:
            return None, None
        rows = db().execute(
            """SELECT * FROM metric_values
               WHERE run_id = ? AND status != 'provisional' AND revision = ?
               ORDER BY metric_key, scope_type, scope_id""",
            (run_id, rev)).fetchall()
        return [dict(r) for r in rows], rev

    @app.route("/api/runs/<run_id>/metrics")
    def run_metrics(run_id):
        run = database.get_run(db(), run_id)
        if run is None:
            return jsonify({"error": "unknown run"}), 404
        stored, rev = _stored_final(run_id)
        if stored is not None and request.args.get("live") != "1":
            values, provisional = stored, False
        else:
            values = metrics_engine.compute_run_metrics(db(), run_id)
            provisional = True
            for v in values:
                if v["status"] == "ok":
                    v["status"] = "provisional"
        dq = metrics_engine.run_data_quality(db(), run_id)
        return jsonify({
            "run_id": run_id,
            "run_status": run["status"],
            "provisional": provisional,
            "revision": rev,
            "values": values,
            "data_quality": dq,
        })

    @app.route("/api/runs/<run_id>/finalize", methods=["POST"])
    def finalize_run(run_id):
        run = database.get_run(db(), run_id)
        if run is None:
            return jsonify({"error": "unknown run"}), 404
        stored, _rev = _stored_final(run_id)
        if stored is not None:
            return jsonify({"error": "already finalized — POST "
                            "/recalculate with {\"confirm\": true} to "
                            "create a new revision"}), 409
        body = request.get_json(force=True, silent=True) or {}
        status = body.get("status")
        if status:
            if status not in ("completed", "failed", "aborted"):
                return jsonify({"error": f"bad final status {status!r}"}), 400
            database.update_run(db(), run_id, {
                "status": status, "ended_at_utc": database.utc_now_iso()})
        elif run["status"] in ("created", "running"):
            return jsonify({"error": "run has no terminal status — pass "
                            "{\"status\": \"completed|failed|aborted\"} or "
                            "send a RUN_COMPLETED/RUN_FAILED/RUN_ABORTED "
                            "event first"}), 400
        values = metrics_engine.compute_run_metrics(db(), run_id)
        for v in values:
            if v["status"] == "ok":
                v["status"] = "final"
        n = database.store_metric_values(db(), run_id, values, "final",
                                         revision=0)
        return jsonify({"finalized": n, "revision": 0})

    @app.route("/api/runs/<run_id>/recalculate", methods=["POST"])
    def recalculate_run(run_id):
        run = database.get_run(db(), run_id)
        if run is None:
            return jsonify({"error": "unknown run"}), 404
        body = request.get_json(force=True, silent=True) or {}
        if body.get("confirm") is not True:
            # Finalized values are never silently overwritten -- an explicit
            # administrative confirmation creates a NEW revision instead.
            return jsonify({"error": "recalculation requires "
                            "{\"confirm\": true} and creates a new "
                            "revision; existing revisions are kept"}), 400
        prev = database.max_final_revision(db(), run_id)
        revision = 0 if prev is None else prev + 1
        values = metrics_engine.compute_run_metrics(db(), run_id)
        for v in values:
            if v["status"] == "ok":
                v["status"] = "final"
        n = database.store_metric_values(db(), run_id, values, "final",
                                         revision=revision)
        return jsonify({"recalculated": n, "revision": revision})

    @app.route("/api/metric-definitions")
    def metric_definitions():
        rows = db().execute(
            """SELECT * FROM metric_definitions
               ORDER BY metric_key, version""").fetchall()
        return jsonify({"definition_set_version": DEFINITION_SET_VERSION,
                        "definitions": [dict(r) for r in rows]})

    # ---- history / aggregates ----------------------------------------------

    DEFAULT_HISTORY_METRICS = [
        "actual_makespan_sec", "observed_duration_sec",
        "throughput_jobs_per_min", "job_completion_rate",
        "fleet_productive_utilization_pct", "fleet_traffic_delay_robot_sec",
        "mission_completion_success", "intervention_free_success",
        "collision_count", "deadlock_count", "min_fleet_separation_m",
        "makespan_error_sec", "yaw_mae_deg",
        "rotation_within_tolerance_rate", "battery_drain_per_job_pct",
        "command_success_rate", "total_distance_m",
    ]

    def _values_for_runs(run_ids, keys):
        """Latest stored value per (run, key): highest final revision wins,
        else the provisional row."""
        if not run_ids:
            return {}
        marks = ",".join("?" * len(run_ids))
        kmarks = ",".join("?" * len(keys))
        rows = db().execute(
            f"""SELECT * FROM metric_values
                WHERE run_id IN ({marks}) AND metric_key IN ({kmarks})
                  AND scope_type = 'run'
                ORDER BY run_id, metric_key,
                         CASE status WHEN 'final' THEN 1 ELSE 0 END,
                         revision""",
            (*run_ids, *keys)).fetchall()
        out: dict[tuple, dict] = {}
        for r in rows:  # later rows (final, higher revision) overwrite
            out[(r["run_id"], r["metric_key"])] = dict(r)
        return out

    @app.route("/api/history")
    def history():
        where, args = _run_filters(executed_default=True)
        limit, offset = _paginate()
        # What the rule removed, so the page can say so rather than just
        # showing a shorter list.
        w_all, a_all = _run_filters(executed_default=False)
        n_all = db().execute(
            f"SELECT COUNT(*) AS n FROM runs r{w_all}", a_all).fetchone()["n"]
        n_exec = db().execute(
            f"SELECT COUNT(*) AS n FROM runs r{where}", args).fetchone()["n"]
        keys = [k for k in
                (request.args.get("metrics") or "").split(",") if k] \
            or DEFAULT_HISTORY_METRICS
        runs = [dict(r) for r in db().execute(
            f"""SELECT r.run_id, r.created_at_utc, r.status,
                       r.experimental_condition, r.algorithm, r.drive_mode,
                       r.robot_count, r.job_count, r.workstation_count,
                       r.process_sec, r.random_seed, r.planned_makespan_sec,
                       -- Wall-clock time the attempt actually ran. Defined
                       -- for every started run, unlike makespan, which only
                       -- exists once robots report completion -- so this is
                       -- the column that separates a run that died at launch
                       -- from one that died near the end.
                       ROUND((julianday(r.ended_at_utc)
                              - julianday(r.started_at_utc)) * 86400.0, 1)
                           AS attempt_sec
                FROM runs r{where}
                ORDER BY r.created_at_utc DESC LIMIT ? OFFSET ?""",
            (*args, limit, offset)).fetchall()]
        values = _values_for_runs([r["run_id"] for r in runs], keys)
        for r in runs:
            r["metrics"] = {}
            for k in keys:
                mv = values.get((r["run_id"], k))
                r["metrics"][k] = None if mv is None else {
                    "value": mv["value"], "status": mv["status"],
                    "numerator": mv["numerator"],
                    "denominator": mv["denominator"],
                    "valid_n": mv["valid_n"], "missing_n": mv["missing_n"],
                }
        group_by = request.args.get("group_by")
        group_col = {"experimental_condition": "experimental_condition",
                     "algorithm": "algorithm", "drive_mode": "drive_mode",
                     "robot_count": "robot_count",
                     "random_seed": "random_seed"}.get(group_by)
        aggregates = {}
        groups: dict[str, list] = {}
        for r in runs:
            gkey = str(r.get(group_col)) if group_col else "all"
            groups.setdefault(gkey, []).append(r)
        for gkey, gruns in groups.items():
            per_metric = {}
            for k in keys:
                xs, contributing, missing = [], [], 0
                for r in gruns:
                    mv = r["metrics"].get(k)
                    if mv and mv["value"] is not None:
                        xs.append(mv["value"])
                        contributing.append(r["run_id"])
                    else:
                        missing += 1
                stats = metrics_engine.aggregate_stats(xs)
                stats["missing_n"] = missing
                stats["attempted_n"] = len(gruns)
                stats["contributing_run_ids"] = contributing
                per_metric[k] = stats
            statuses = [r["status"] for r in gruns]
            n_success = statuses.count("completed")
            lo, hi = metrics_engine.proportion_ci95(n_success, len(gruns))
            per_metric["run_success_rate"] = {
                "n": len(gruns), "mean": (n_success / len(gruns)) if gruns else None,
                "numerator": n_success, "denominator": len(gruns),
                "ci95_low": lo, "ci95_high": hi,
                "failed_n": statuses.count("failed"),
                "aborted_n": statuses.count("aborted"),
                "contributing_run_ids": [r["run_id"] for r in gruns],
            }
            aggregates[gkey] = per_metric
        return jsonify({"runs": runs, "metric_keys": keys,
                        "group_by": group_by, "aggregates": aggregates,
                        "limit": limit, "offset": offset,
                        "nonexecuted_hidden": max(0, n_all - n_exec)})

    @app.route("/api/runs/facets")
    def runs_facets():
        """Distinct values of the free-text run columns, with counts.

        These columns hold values nobody can be expected to retype -- an
        algorithm reads "external (nearest_2026-09-09_20-53-58.json)" -- and
        _run_filters() compares them with =, so a partial string silently
        matches nothing. The filter UI offers these as dropdowns instead.
        Unfiltered on purpose: the list must show what COULD be selected,
        not what survives the selection already in force.
        """
        out = {}
        for col in ("experimental_condition", "algorithm", "drive_mode"):
            rows = db().execute(
                f"""SELECT {col} AS v, COUNT(*) AS n FROM runs
                    WHERE {col} IS NOT NULL AND {col} != ''
                    GROUP BY {col} ORDER BY n DESC, v ASC""").fetchall()
            out[col] = [{"value": r["v"], "count": r["n"]} for r in rows]
        return jsonify(out)

    # ---- exports -----------------------------------------------------------

    @app.route("/api/export/runs.csv")
    def export_runs_csv():
        where, args = _run_filters(executed_default=True)
        runs = [dict(r) for r in db().execute(
            f"""SELECT r.* FROM runs r{where}
                ORDER BY r.created_at_utc DESC""", args).fetchall()]
        keys = DEFAULT_HISTORY_METRICS
        values = _values_for_runs([r["run_id"] for r in runs], keys)
        buf = io.StringIO()
        writer = csv.writer(buf, lineterminator="\n")
        head = ["run_id", "created_at_utc", "status",
                "experimental_condition", "algorithm", "drive_mode",
                "robot_count", "job_count", "workstation_count",
                "process_sec", "capacity", "random_seed",
                "planned_makespan_sec"]
        writer.writerow(head + keys)
        for r in runs:
            row = [r.get(h) for h in head]
            for k in keys:
                mv = values.get((r["run_id"], k))
                row.append("" if mv is None or mv["value"] is None
                           else mv["value"])
            writer.writerow(row)
        return app.response_class(
            buf.getvalue(), mimetype="text/csv",
            headers={"Content-Disposition":
                     "attachment; filename=agv_runs.csv"})

    @app.route("/api/export/run/<run_id>.csv")
    def export_run_csv(run_id):
        """One run as CSV, in blank-line-separated blocks.

        Blocks rather than one wide table because a run has four different
        shapes of record (the run row, its computed metrics, per-robot and
        per-job) and flattening them into shared columns makes all four
        harder to read. The in-page viewer renders each block as its own
        table; a spreadsheet opens it as one sheet with gaps, which is the
        conventional reading of this layout.
        """
        run = database.get_run(db(), run_id)
        if run is None:
            return jsonify({"error": "unknown run"}), 404
        buf = io.StringIO()
        w = csv.writer(buf, lineterminator="\n")

        w.writerow(["field", "value"])
        run_keys = run.keys()
        for k in ("run_id", "created_at_utc", "started_at_utc",
                  "ended_at_utc", "status", "experimental_condition",
                  "algorithm", "drive_mode", "robot_count", "job_count",
                  "workstation_count", "process_sec", "capacity",
                  "random_seed", "planned_makespan_sec", "notes"):
            w.writerow([k, run[k] if k in run_keys else ""])

        stored, _rev = _stored_final(run_id)
        if stored is None:
            stored = metrics_engine.compute_run_metrics(db(), run_id)
        w.writerow([])
        w.writerow(["metric", "value", "unit", "status"])
        for v in (stored or []):
            w.writerow([v.get("key", ""), v.get("value", ""),
                        v.get("unit", ""), v.get("status", "")])

        rcols = ["robot_id", "capacity", "start_battery_pct",
                 "end_battery_pct", "completion_elapsed_ms", "terminal_x_m",
                 "terminal_y_m", "terminal_yaw_deg", "status",
                 "failure_reason"]
        w.writerow([])
        w.writerow(rcols)
        for r in db().execute(
                "SELECT * FROM run_robots WHERE run_id=? ORDER BY robot_id",
                (run_id,)).fetchall():
            w.writerow([r[c] for c in rcols])

        jcols = ["job_id", "assigned_robot_id", "workstation_id",
                 "release_elapsed_ms", "pickup_elapsed_ms",
                 "delivery_elapsed_ms", "completion_elapsed_ms",
                 "due_elapsed_ms", "status", "failure_reason"]
        w.writerow([])
        w.writerow(jcols)
        for r in db().execute(
                "SELECT * FROM jobs WHERE run_id=? ORDER BY job_id",
                (run_id,)).fetchall():
            w.writerow([r[c] for c in jcols])

        # Every string the robots published, in order. This is the block a
        # breakdown is read from: the firmware's own words, timestamped
        # against the run, rather than whatever the supervisor chose to log.
        w.writerow([])
        w.writerow(["elapsed_ms", "robot_id", "status"])
        for r in db().execute(
                """SELECT elapsed_ms, robot_id, reason FROM events
                   WHERE run_id = ? AND event_type = 'ROBOT_STATUS'
                   ORDER BY seq""", (run_id,)).fetchall():
            w.writerow([r["elapsed_ms"], r["robot_id"], r["reason"]])

        return app.response_class(
            buf.getvalue(), mimetype="text/csv",
            headers={"Content-Disposition":
                     f"inline; filename=run_{run_id[:8]}.csv"})

    @app.route("/api/export/run/<run_id>.json")
    def export_run_json(run_id):
        run = database.get_run(db(), run_id)
        if run is None:
            return jsonify({"error": "unknown run"}), 404
        bundle = metrics_engine.load_run_bundle(db(), run_id)
        stored, rev = _stored_final(run_id)
        if stored is None:
            stored = metrics_engine.compute_run_metrics(db(), run_id)
            for v in stored:
                if v["status"] == "ok":
                    v["status"] = "provisional"
        run_out = dict(run)
        try:
            run_out["plan_json"] = json.loads(run_out["plan_json"]) \
                if run_out.get("plan_json") else None
        except (TypeError, ValueError):
            pass
        return jsonify({
            "run": run_out,
            "robots": bundle["robots"],
            "jobs": bundle["jobs"],
            "events": [{k: v for k, v in e.items() if k != "details_json"}
                       for e in bundle["events"]],
            "conflicts": bundle["conflicts"],
            "telemetry_sample_count": len(bundle["telemetry"]),
            "metric_values": stored,
            "revision": rev,
            "data_quality": metrics_engine.run_data_quality(
                db(), run_id, bundle=bundle),
        })

    return app


app = create_app()

if __name__ == "__main__":
    port = DEFAULT_PORT
    for i, arg in enumerate(sys.argv):
        if arg == "--port" and i + 1 < len(sys.argv):
            port = int(sys.argv[i + 1])
    print(f"AGV solver + metrics backend: http://localhost:{port}/")
    print(f"metric definitions registered: {len(DEFINITIONS)}")
    # 0.0.0.0 so the Linux-laptop supervisor can POST events over the LAN.
    app.run(host="0.0.0.0", port=port, threaded=True)
