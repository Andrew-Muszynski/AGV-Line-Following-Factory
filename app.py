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
        run_id = database.create_run(db(), payload)
        return jsonify({"run_id": run_id}), 201

    def _run_filters():
        """WHERE clause + args from query params (shared by list/history/
        export). All comparisons parameterized."""
        q = request.args
        where, args = [], []
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
        try:
            database.update_run(db(), run_id, patch)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify(dict(database.get_run(db(), run_id)))

    # ---- events ------------------------------------------------------------

    def _ingest(run_id, events):
        if database.get_run(db(), run_id) is None:
            return jsonify({"error": "unknown run"}), 404
        result = database.ingest_events(db(), run_id, events)
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
        where, args = _run_filters()
        limit, offset = _paginate()
        keys = [k for k in
                (request.args.get("metrics") or "").split(",") if k] \
            or DEFAULT_HISTORY_METRICS
        runs = [dict(r) for r in db().execute(
            f"""SELECT r.run_id, r.created_at_utc, r.status,
                       r.experimental_condition, r.algorithm, r.drive_mode,
                       r.robot_count, r.job_count, r.workstation_count,
                       r.process_sec, r.random_seed, r.planned_makespan_sec
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
                        "limit": limit, "offset": offset})

    # ---- exports -----------------------------------------------------------

    @app.route("/api/export/runs.csv")
    def export_runs_csv():
        where, args = _run_filters()
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
