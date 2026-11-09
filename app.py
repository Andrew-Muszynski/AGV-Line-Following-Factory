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

import atexit
import csv
import io
import json
import os
import sys

from flask import (Flask, g, jsonify, redirect, render_template, request,
                   send_file)

import database
import metrics_engine
import plan_store
import video_recorder
import vlm_chat
from metrics_definitions import (DEFINITION_SET_VERSION, DEFINITIONS,
                                 seed_definitions)

DEFAULT_PORT = 8000
#: The Linux laptop running the supervisor. supervisor_metrics_url() asks
#: the OS which of this laptop's addresses routes there.
SUPERVISOR_HOST = os.environ.get("AGV_SUPERVISOR_HOST", "192.168.0.212")


def supervisor_metrics_url(port: int = DEFAULT_PORT) -> str | None:
    """This backend's URL as the supervisor can reach it, or None.

    The supervisor posts every run event here, so it needs a LAN address,
    and this laptop's changes with the connection: 192.168.0.162 on WiFi,
    192.168.0.174 on Ethernet (2026-11-03: a run on Ethernet stored zero
    events because the supervisor was still posting to the WiFi address).
    A UDP connect() sends nothing; it only makes the OS pick the route.
    """
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sk:
            sk.connect((SUPERVISOR_HOST, 9))
            ip = sk.getsockname()[0]
    except OSError:
        return None
    return None if ip.startswith(("0.", "127.")) else f"http://{ip}:{port}"


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

    def recorder():
        """The run-video recorder, or None. Only `python app.py` attaches
        one (see __main__): an app built by a test or an import must never
        open the camera stream and encode a file because it saw RUN_STARTED."""
        return app.extensions.get("video_recorder")

    def _recorder_hook(fn):
        rec = recorder()
        if rec is None:
            return
        try:
            fn(rec)
        except Exception:               # a recording must never break the API
            app.logger.exception("video recorder hook failed")

    @app.after_request
    def cors(resp):
        # Local-tool convenience: lets the page keep working against this
        # API even when served by a different local server (http.server).
        if request.path.startswith("/api/"):
            resp.headers["Access-Control-Allow-Origin"] = "*"
            resp.headers["Access-Control-Allow-Methods"] = "GET, POST, PATCH, DELETE, OPTIONS"
            resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return resp

    @app.route("/api/<path:_any>", methods=["OPTIONS"])
    def options_ok(_any):
        return "", 204

    # ---- pages -------------------------------------------------------------

    @app.route("/")
    def index():
        # Cache-buster for the page's scripts: the newest JS edit time, so a
        # deployed change is loaded without anyone remembering Ctrl+F5
        # (2026-10-30: bay colours were still drawn by a stale solver.js).
        js_dir = os.path.join(app.static_folder, "js")
        try:
            asset_v = int(max(os.path.getmtime(os.path.join(js_dir, f))
                              for f in os.listdir(js_dir) if f.endswith(".js")))
        except (OSError, ValueError):
            asset_v = 0
        return render_template("index.html", asset_v=asset_v)

    @app.route("/agv_grid_workstation_solver.html")
    def legacy_url():
        # Old http.server bookmark keeps working.
        return redirect("/", code=302)

    # ---- traffic planning (DOE) -------------------------------------------
    # The Run & Monitor tab's "Traffic planner" buttons send the page's current
    # plan here and get back the same JOB allocation with routes and waits
    # planned by the chosen method (traffic/planners.py), in the page's own
    # export format, which the page then dispatches exactly as planned. This
    # is what lets back-to-back runs differ ONLY in the traffic planner.

    @app.route("/api/traffic/methods")
    def traffic_methods():
        from traffic.planners import METHODS
        return jsonify({"methods": list(METHODS)})

    @app.route("/api/traffic/plan", methods=["POST"])
    def traffic_plan():
        from traffic import instance as traffic_instance
        from traffic.export import to_page_plan
        from traffic.planners import METHODS, solve
        from traffic.validate import metrics as traffic_metrics
        from traffic.validate import validate as traffic_validate

        body = request.get_json(force=True, silent=True) or {}
        method = str(body.get("method", "")).strip().lower()
        if method not in METHODS:
            return jsonify({"error": f"method must be one of {list(METHODS)}"}), 400
        plan = body.get("plan")
        if not isinstance(plan, dict):
            return jsonify({"error": "missing 'plan' (the page's export payload)"}), 400
        try:
            inst = traffic_instance.load_data(plan, str(body.get("name") or "page"))
        except (ValueError, KeyError, TypeError) as exc:
            return jsonify({"error": f"cannot read the plan: {exc}"}), 400
        limit = min(120.0, max(1.0, float(body.get("time_limit") or 30.0)))
        res = solve(inst, method, time_limit=limit, seed=int(body.get("seed") or 0))
        if not res.paths:
            return jsonify({"ok": False, "method": method, "note": res.note,
                            "planning_sec": round(res.planning_sec, 3),
                            "stats": res.stats})
        report = traffic_validate(inst, res.paths)
        row = traffic_metrics(inst, res.paths, report)
        return jsonify({
            "ok": True, "method": method,
            "plan": to_page_plan(inst, res.paths, method, report, row),
            "valid": report["valid"],
            "conflicts": [c.describe() for c in report["conflicts"]],
            "errors": report["errors"],
            "metrics": row, "planning_sec": round(res.planning_sec, 3),
            "note": res.note, "stats": res.stats,
        })

    # ---- saved plans -------------------------------------------------------
    # The solver page saves a solved plan into generated_plans/ and reloads
    # it from a dropdown, so a BRKGA solve worth minutes of CPU is kept
    # without a download/import round trip. plan_server.py serves the same
    # store under /plans for the start_dashboard.bat flow, where this Flask
    # app is not running -- see plan_store.py.

    @app.route("/api/plans")
    def plans_list():
        return jsonify({"dir": str(plan_store.PLANS_DIR),
                        "plans": plan_store.list_plans()})

    @app.route("/api/plans/<name>")
    def plans_get(name):
        try:
            return jsonify(plan_store.read_plan(name))
        except FileNotFoundError:
            return jsonify({"error": f"no such plan: {name}"}), 404
        except plan_store.PlanStoreError as exc:
            return jsonify({"error": str(exc)}), 400

    @app.route("/api/plans", methods=["POST"])
    def plans_save():
        body = request.get_json(silent=True) or {}
        try:
            saved = plan_store.save_plan(body.get("name", ""),
                                         body.get("plan"),
                                         overwrite=bool(body.get("overwrite")))
        except plan_store.PlanStoreError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"ok": True, **saved}), 201

    @app.route("/api/plans/<name>", methods=["DELETE"])
    def plans_delete(name):
        try:
            return jsonify({"ok": True, "deleted": plan_store.delete_plan(name)})
        except FileNotFoundError:
            return jsonify({"error": f"no such plan: {name}"}), 404
        except plan_store.PlanStoreError as exc:
            return jsonify({"error": str(exc)}), 400

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
        # Name for this run's video, typed in Run & Monitor. Not a run column:
        # the recorder holds it until the run starts.
        video_label = str(payload.pop("video_label", "") or "").strip()
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
                _recorder_hook(lambda r: r.arm(reuse, video_label,
                                               payload.get("plan_file")))
                return jsonify({"run_id": reuse, "reused": True,
                                "supervisor_metrics_url":
                                supervisor_metrics_url(request.host.partition(":")[2]
                                                       or DEFAULT_PORT)}), 200
        run_id = database.create_run(db(), payload)
        _recorder_hook(lambda r: r.arm(run_id, video_label,
                                       payload.get("plan_file")))
        return jsonify({"run_id": run_id,
                        "supervisor_metrics_url":
                        supervisor_metrics_url(request.host.partition(":")[2]
                                               or DEFAULT_PORT)}), 201

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
            "traffic_method": "r.traffic_method",
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
                       r.random_seed, r.planned_makespan_sec,
                       r.traffic_method, r.plan_file, r.notes,
                       (SELECT MAX(v.video_id) FROM run_videos v
                        WHERE v.run_id = r.run_id
                          AND v.status IN ('ready', 'recording')) AS video_id
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
        # The supervisor is the authority on how a run ended. 2026-10-23: the
        # browser's /fleet_status fallback read "complete: none; errored:
        # none" after a collision abort as a clean finish and overwrote
        # RUN_ABORTED/RUN_FAILED with 'completed' -- a 10-of-12-jobs run then
        # showed a 165.8 s makespan. A logged abort or failure is never
        # upgraded to completed.
        if patch.get("status") == "completed":
            ended = db().execute(
                "SELECT event_type FROM events WHERE run_id = ? AND "
                "event_type IN ('RUN_ABORTED', 'RUN_FAILED') ORDER BY seq "
                "LIMIT 1", (run_id,)).fetchone()
            if ended is not None:
                return jsonify({
                    "error": f"the supervisor logged {ended['event_type']} "
                             "for this run; refusing to mark it completed",
                    "run_id": run_id}), 409
        try:
            database.update_run(db(), run_id, patch)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        # Video: the Start button is the earliest sign a run is under way;
        # a terminal status ends its recording (after the tail).
        status = patch.get("status")
        if status == "running":
            _recorder_hook(lambda r: r.on_run_started(run_id, "browser"))
        elif status in ("completed", "failed", "aborted"):
            _recorder_hook(lambda r: r.on_run_ended(run_id, f"run {status}"))
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

    def _video_triggers(run_id, events):
        """RUN_STARTED starts (or re-anchors) the run's recording; a terminal
        event ends it. Events after RUN_STARTED in the same batch prove it
        was emitted at least that much earlier than this POST arrived, which
        is the lag the recorder subtracts from its time anchor."""
        evs = [e for e in events if isinstance(e, dict)]
        types = [str(e.get("event_type")) for e in evs]
        if "RUN_STARTED" in types:
            i = types.index("RUN_STARTED")
            t0 = evs[i].get("elapsed_ms")
            later = [e.get("elapsed_ms") for e in evs[i + 1:]
                     if isinstance(e.get("elapsed_ms"), (int, float))]
            lag = (max(later) - t0) / 1000.0 \
                if later and isinstance(t0, (int, float)) else 0.0
            _recorder_hook(lambda r: r.on_run_started(
                run_id, "supervisor", max(0.0, lag)))
        ended = next((t for t in types if t in TERMINAL_EVENTS), None)
        if ended:
            _recorder_hook(lambda r: r.on_run_ended(run_id, ended))

    def _ingest(run_id, events):
        if database.get_run(db(), run_id) is None:
            return jsonify({"error": "unknown run"}), 404
        result = database.ingest_events(db(), run_id, events)
        _video_triggers(run_id, events)
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

    # ---- run videos ----------------------------------------------------------
    # Recording itself is video_recorder.py. These routes are its controls
    # (Run & Monitor) and the library the Recordings page reads.

    def _no_recorder():
        return jsonify({"error": "video recording is not enabled in this app "
                                 "instance -- start it with python app.py"}), 409

    def _video_out(row):
        v = dict(row)
        rec = recorder()
        v["file_exists"] = bool(rec and rec.find_file(v["file_name"]))
        v["url"] = f"/api/videos/{v['video_id']}/file"
        return v

    @app.route("/api/recorder")
    def recorder_status():
        rec = recorder()
        if rec is None:
            return jsonify({"enabled": False, "available": False,
                            "reason": "no recorder in this app instance"})
        return jsonify({**rec.status(), "enabled": True})

    @app.route("/api/recorder/settings", methods=["POST"])
    def recorder_settings():
        rec = recorder()
        if rec is None:
            return _no_recorder()
        body = request.get_json(force=True, silent=True) or {}
        if "auto" in body:
            rec.set_auto(bool(body["auto"]))
        if "second" in body:
            rec.set_second(bool(body["second"]))
        if "hq" in body:
            rec.set_hq(bool(body["hq"]))
        return jsonify({**rec.status(), "enabled": True})

    @app.route("/api/recorder/start", methods=["POST"])
    def recorder_start():
        rec = recorder()
        if rec is None:
            return _no_recorder()
        body = request.get_json(force=True, silent=True) or {}
        run_id = body.get("run_id") or None
        run = database.get_run(db(), run_id) if run_id else None
        if run_id and run is None:
            return jsonify({"error": "unknown run", "run_id": run_id}), 404
        # The page offers its session run, which may have finished long ago;
        # a recording made after that must not inherit its events.
        if run is not None and run["status"] in ("completed", "failed",
                                                 "aborted"):
            run_id = None
        try:
            out = rec.start(run_id=run_id, label=body.get("label"))
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409
        return jsonify({**out, "enabled": True})

    @app.route("/api/recorder/stop", methods=["POST"])
    def recorder_stop():
        rec = recorder()
        if rec is None:
            return _no_recorder()
        return jsonify({**rec.stop("stopped by hand", wait=True),
                        "enabled": True})

    @app.route("/api/videos")
    def list_videos():
        where, args = [], []
        for param in ("run_id", "status"):
            if request.args.get(param):
                where.append(f"v.{param} = ?")
                args.append(request.args.get(param))
        clause = (" WHERE " + " AND ".join(where)) if where else ""
        limit, offset = _paginate()
        total = db().execute(
            f"SELECT COUNT(*) AS n FROM run_videos v{clause}",
            args).fetchone()["n"]
        rows = db().execute(
            f"""SELECT v.*, r.robot_count, r.traffic_method,
                       r.experimental_condition, r.status AS run_status,
                       r.drive_mode, r.algorithm, r.planned_makespan_sec
                FROM run_videos v LEFT JOIN runs r ON r.run_id = v.run_id
                {clause}
                ORDER BY v.video_id DESC LIMIT ? OFFSET ?""",
            (*args, limit, offset)).fetchall()
        videos = [_video_out(r) for r in rows]
        values = _values_for_runs(
            sorted({v["run_id"] for v in videos if v["run_id"]}),
            ["actual_makespan_sec"])
        for v in videos:
            mv = values.get((v["run_id"], "actual_makespan_sec"))
            v["actual_makespan_sec"] = mv["value"] if mv else None
        return jsonify({"total": total, "limit": limit, "offset": offset,
                        "videos": videos})

    @app.route("/api/videos/<int:video_id>")
    def get_video(video_id):
        row = database.get_video(db(), video_id)
        if row is None:
            return jsonify({"error": "unknown video"}), 404
        return jsonify(_video_out(row))

    @app.route("/api/videos/<int:video_id>", methods=["PATCH"])
    def patch_video(video_id):
        if database.get_video(db(), video_id) is None:
            return jsonify({"error": "unknown video"}), 404
        body = request.get_json(force=True, silent=True) or {}
        patch = {k: body[k] for k in ("title", "notes") if k in body}
        if "second_offset_sec" in body:
            # The page's sync nudge for the second camera.
            try:
                patch["second_offset_sec"] = round(
                    float(body["second_offset_sec"]), 3)
            except (TypeError, ValueError):
                return jsonify({"error": "second_offset_sec must be a "
                                         "number"}), 400
        if "title" in patch:
            patch["title"] = str(patch["title"] or "").strip()[:200]
            if not patch["title"]:
                return jsonify({"error": "a video needs a name"}), 400
        database.update_video(db(), video_id, patch)
        return jsonify(_video_out(database.get_video(db(), video_id)))

    @app.route("/api/videos/<int:video_id>", methods=["DELETE"])
    def delete_video(video_id):
        row = database.get_video(db(), video_id)
        if row is None:
            return jsonify({"error": "unknown video"}), 404
        if row["status"] == "recording":
            return jsonify({"error": "stop the recording before deleting "
                                     "it"}), 409
        rec = recorder()
        if rec is None:
            return _no_recorder()
        path = rec.find_file(row["file_name"])
        if path:
            os.remove(path)
        database.delete_video(db(), video_id)
        return jsonify({"deleted": video_id, "file_removed": bool(path)})

    @app.route("/api/videos/<int:video_id>/file")
    def video_file(video_id):
        row = database.get_video(db(), video_id)
        if row is None:
            return jsonify({"error": "unknown video"}), 404
        if row["status"] == "recording":
            return jsonify({"error": "still recording -- the file can be "
                                     "played once the recording stops"}), 409
        rec = recorder()
        path = rec.find_file(row["file_name"]) if rec else None
        if not path:
            return jsonify({"error": "the video file is missing"}), 404
        ext = os.path.splitext(path)[1].lower()
        name = (video_recorder.slug(row["title"], limit=120)
                or os.path.splitext(row["file_name"])[0]) + ext
        # conditional=True answers Range requests, which is what lets the
        # player seek without downloading the whole file first.
        return send_file(path, conditional=True,
                         mimetype="video/mp4" if ext == ".mp4"
                         else "video/x-msvideo",
                         as_attachment=request.args.get("download") == "1",
                         download_name=name)

    def _marker_text(e, details):
        bits = []
        robots = e["robot_id"] or ", ".join(
            details.get("robot_ids") or details.get("robots") or [])
        if e["workstation_id"]:
            bits.append(f"WS {e['workstation_id']}")
        if e["job_id"]:
            bits.append(e["job_id"])
        if details.get("gap_in") is not None:
            bits.append(f"gap {details['gap_in']} in")
        note = e["reason"] or details.get("note") or details.get("reason")
        if note:
            bits.append(str(note))
        return robots, " · ".join(bits)

    @app.route("/api/videos/<int:video_id>/markers")
    def video_markers(video_id):
        """The run's notable events placed on the video's own timeline."""
        row = database.get_video(db(), video_id)
        if row is None:
            return jsonify({"error": "unknown video"}), 404
        offset = row["run_t0_offset_sec"]
        out = {"video_id": video_id, "run_id": row["run_id"],
               "offset_sec": offset, "duration_sec": row["duration_sec"],
               "markers": []}
        if not row["run_id"]:
            return jsonify(out)
        category = {t: cat for cat, types in
                    video_recorder.MARKER_EVENTS.items() for t in types}
        # elapsed_ms counts from RUN_STARTED; anything earlier was timed
        # from configuration and cannot be placed on the video.
        first = db().execute(
            "SELECT MIN(seq) AS s FROM events WHERE run_id = ? "
            "AND event_type = 'RUN_STARTED'", (row["run_id"],)).fetchone()["s"]
        marks = ",".join("?" * len(category))
        rows = db().execute(
            f"""SELECT seq, elapsed_ms, event_type, robot_id, job_id,
                       workstation_id, reason, details_json
                FROM events
                WHERE run_id = ? AND seq >= ? AND event_type IN ({marks})
                ORDER BY seq LIMIT 3000""",
            (row["run_id"], first or 0, *category)).fetchall()
        for e in rows:
            try:
                details = json.loads(e["details_json"] or "{}") or {}
            except ValueError:
                details = {}
            robots, text = _marker_text(e, details)
            elapsed = None if e["elapsed_ms"] is None \
                else e["elapsed_ms"] / 1000.0
            out["markers"].append({
                "seq": e["seq"], "event_type": e["event_type"],
                "category": category[e["event_type"]],
                "elapsed_sec": elapsed, "robots": robots, "text": text,
                "t_video": None if offset is None or elapsed is None
                else round(offset + elapsed, 2)})
        return jsonify(out)

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

    def _is_stale(run_id):
        """True when data landed after the stored metrics were computed.

        Finalization fires on the run's terminal event, but the supervisor
        keeps flushing pose batches for a long time afterwards -- on the
        2026-10-13 reference run the last event arrived 71 minutes later.
        Everything computed from telemetry (distance, pose rate, vision
        availability) was therefore frozen against a partial timeline, and
        a run finalized before its terminal event was visible kept a null
        makespan forever. 64 of 98 finalized runs were affected.

        Comparing the newest event's wall clock against the newest
        computed_at is one indexed MAX each and costs nothing on a page
        load, so staleness is detected rather than waited for.
        """
        row = db().execute(
            """SELECT (SELECT MAX(time_utc) FROM events WHERE run_id = :r)
                          AS last_event,
                      (SELECT MAX(computed_at_utc) FROM metric_values
                        WHERE run_id = :r) AS last_computed""",
            {"r": run_id}).fetchone()
        if row is None or row["last_event"] is None or row["last_computed"] is None:
            return False
        return str(row["last_event"]) > str(row["last_computed"])

    def _refresh_if_stale(run_id):
        """Recompute into a NEW revision when late data has arrived.

        A new revision rather than an overwrite, so the original numbers
        stay auditable -- the same rule /recalculate follows.
        """
        try:
            if not _is_stale(run_id):
                return False
            prev = database.max_final_revision(db(), run_id)
            if prev is None:
                return False
            values = metrics_engine.compute_run_metrics(db(), run_id)
            for v in values:
                if v["status"] == "ok":
                    v["status"] = "final"
            database.store_metric_values(db(), run_id, values, "final",
                                         revision=prev + 1)
            app.logger.info("refreshed stale metrics for %s -> revision %d",
                            run_id, prev + 1)
            return True
        except Exception:                       # never fail a page load
            app.logger.exception("stale refresh failed for %s", run_id)
            return False

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
        _refresh_if_stale(run_id)
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
        # The control tuning this run executed under, captured by the
        # supervisor at mission configure. Null for runs recorded before
        # 2026-10-14, when nothing recorded it -- the page says so rather
        # than showing today's constants, which would assert something
        # false about an older run.
        tuning = None
        raw = run["tuning_json"] if "tuning_json" in run.keys() else None
        if raw:
            try:
                tuning = json.loads(raw)
            except (TypeError, ValueError):
                tuning = {"error": "stored tuning is not valid JSON"}
        return jsonify({
            "run_id": run_id,
            "run_status": run["status"],
            "provisional": provisional,
            "revision": rev,
            "values": values,
            "data_quality": dq,
            "tuning": tuning,
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
        "workstation_completion_rate",
        "fleet_productive_utilization_pct", "fleet_traffic_delay_robot_sec",
        "mission_completion_success", "intervention_free_success",
        "collision_count", "deadlock_count", "total_deadlock_sec",
        "min_fleet_separation_m",
        "makespan_error_sec", "yaw_mae_deg",
        "rotation_within_tolerance_rate", "battery_drain_per_job_pct",
        "command_success_rate", "total_distance_m",
    ]

    def _values_for_runs(run_ids, keys):
        """Latest stored value per (run, key): the HIGHEST REVISION wins,
        whatever its value status; a provisional row only when nothing
        final exists. Same choice as the run detail view (_stored_final).

        It used to sort on status == 'final' before revision, so an older
        revision's 'final' number beat a newer revision's 'n/a'. 2026-10-23:
        a collision run re-labelled from completed to failed and recomputed
        (revision 3: makespan n/a) still showed revision 1's 165.8 s makespan
        here -- a red dot on the makespan chart."""
        if not run_ids:
            return {}
        marks = ",".join("?" * len(run_ids))
        kmarks = ",".join("?" * len(keys))
        rows = db().execute(
            f"""SELECT * FROM metric_values
                WHERE run_id IN ({marks}) AND metric_key IN ({kmarks})
                  AND scope_type = 'run'
                ORDER BY run_id, metric_key,
                         CASE status WHEN 'provisional' THEN 0 ELSE 1 END,
                         revision""",
            (*run_ids, *keys)).fetchall()
        out: dict[tuple, dict] = {}
        for r in rows:  # later rows (non-provisional, higher revision) overwrite
            out[(r["run_id"], r["metric_key"])] = dict(r)
        return out

    @app.route("/api/history")
    def history():
        where, args = _run_filters(executed_default=True)
        # ALL matching runs by default (2026-10-23). This used the shared
        # 100-row page size, so the history showed only the newest 100 of
        # 142 runs -- and the success rates and means below, computed over
        # the returned rows, silently left the rest out. An explicit
        # ?limit= still pages.
        limit = min(10000, max(1, int(request.args.get("limit", 10000))))
        offset = max(0, int(request.args.get("offset", 0)))
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
                       r.traffic_method,
                       -- Wall-clock time the attempt actually ran. Defined
                       -- for every started run, unlike makespan, which only
                       -- exists once robots report completion -- so this is
                       -- the column that separates a run that died at launch
                       -- from one that died near the end.
                       ROUND((julianday(r.ended_at_utc)
                              - julianday(r.started_at_utc)) * 86400.0, 1)
                           AS attempt_sec,
                       (SELECT MAX(v.video_id) FROM run_videos v
                        WHERE v.run_id = r.run_id
                          AND v.status IN ('ready', 'recording')) AS video_id
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
                     "traffic_method": "traffic_method",
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
                        "total": n_exec,
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
        for col in ("experimental_condition", "algorithm", "drive_mode",
                    "traffic_method"):
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
                "planned_makespan_sec", "traffic_method"]
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
                  "random_seed", "planned_makespan_sec", "traffic_method",
                  "notes"):
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
    # Per-run video recording. Attached here rather than in create_app so
    # tests and imports never open the camera stream (see recorder()).
    try:
        _rec = video_recorder.VideoRecorder.for_app(app.config["DB_PATH"])
        app.extensions["video_recorder"] = _rec
        atexit.register(_rec.shutdown)
        print(f"run videos: {_rec.video_dir} (stream {_rec.stream_url}, "
              f"auto-record {'on' if _rec.auto else 'off'})")
    except Exception as exc:
        print(f"run videos DISABLED: {exc}")
    # 0.0.0.0 so the Linux-laptop supervisor can POST events over the LAN.
    app.run(host="0.0.0.0", port=port, threaded=True)
