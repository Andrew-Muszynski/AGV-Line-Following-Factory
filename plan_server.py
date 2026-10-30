#!/usr/bin/env python3
"""
plan_server.py — localhost solver service for the solver HTML.

Lets the page's Route-mode dropdown run the external vrp_rpd algorithms
(NN / max-regret / greedy-defer / best-of-3 / BRKGA) directly on Solve — no
file export/import round trip. The browser POSTs the current instance
(grid, agents, capacity, processing time, selected bays) to /solve and gets
back the visit plan, which the page applies exactly like an imported plan.

    python plan_server.py                # port 8082, repo external/vrp_rpd
    python plan_server.py --port 9000 --repo path\to\vrp_rpd

Endpoints:
    GET    /health       -> {"ok": true, "algorithms": [...]}
    POST   /solve        <- {"algorithm": "regret", "config": {<page JSON>}}
                         -> {"algorithm": "Max Regret", "visits": {...}, ...}
    GET    /plans        -> {"dir": ..., "plans": [{name, agents, bays, ...}]}
    GET    /plans/<name> -> that saved plan, for the page's dropdown
    POST   /plans        <- {"name": "...", "plan": {...}}
    DELETE /plans/<name>

The /plans endpoints are the generated_plans/ store (plan_store.py), shared
with app.py so the dropdown works whichever server happens to be running.

Runs the solvers one at a time (they are CPU-heavy); the HTML waits on the
fetch. Reuses export_plan_json.py for all solver plumbing, so the dropdown
and the offline exporter can never disagree.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import export_plan_json as epj
import plan_store

_solve_lock = threading.Lock()


def solve_request(body: dict, repo: str, sec_per_hop: float,
                  brkga_kwargs: dict | None = None) -> dict:
    algorithm = str(body.get("algorithm", "")).strip()
    if algorithm not in epj.ALGORITHMS:
        raise ValueError(f"unknown algorithm '{algorithm}' "
                         f"(choose from {', '.join(epj.ALGORITHMS)})")
    cfg = body.get("config")
    if not isinstance(cfg, dict):
        raise ValueError("missing 'config' (the solver HTML's instance JSON)")
    for key in ("grid", "agents", "capacity", "workstations"):
        if key not in cfg:
            raise ValueError(f"config has no '{key}'")
    if not cfg["workstations"]:
        raise ValueError("config has no selected workstations")

    process_sec = float(cfg.get("rpd", {}).get("process_sec", 15.0))
    with _solve_lock:
        result, grid = epj.solve(cfg, algorithm, process_sec, sec_per_hop,
                                 seed=int(body.get("seed", 42)), repo=repo,
                                 brkga_kwargs=brkga_kwargs)
        visits = epj.events_to_visits(result, grid, cfg)

    payload = {
        "algorithm": result.heuristic,
        "source": "plan_server (romeshprasad/vrp_rpd pipeline_completed)",
        "process_sec": process_sec,
        "capacity": int(cfg["capacity"]),
        "solver_makespan_hops": float(result.makespan),
        "solver_makespan_sec_estimate": float(result.makespan) * sec_per_hop,
        "visits": visits,
    }

    # Archive every solve so a stochastic run can be replayed exactly later
    # via the HTML's "Import plan" (same JSON either way).
    import re
    import time
    plans_dir = Path(__file__).resolve().parent / "solved_plans"
    plans_dir.mkdir(exist_ok=True)
    slug = re.sub(r"[^a-z0-9]+", "_", result.heuristic.lower()).strip("_")
    out = plans_dir / f"plan_{slug}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    payload["saved_to"] = str(out)
    return payload


def main() -> None:
    ap = argparse.ArgumentParser(description="Solver service for the HTML "
                                             "route-mode dropdown.")
    ap.add_argument("--port", type=int, default=8082)
    ap.add_argument("--repo",
                    default=str(Path(__file__).resolve().parent / "external" / "vrp_rpd"),
                    help="vrp_rpd checkout on pipeline_completed "
                         "(default: ./external/vrp_rpd)")
    ap.add_argument("--sec-per-hop", type=float,
                    default=epj.SEC_PER_HOP_DEFAULT)
    ap.add_argument("--brkga", default="",
                    help="comma-separated solver kwargs applied to every "
                         "brkga solve, e.g. 'num_cpu_workers=2,"
                         "total_generations=300' (empty = warm-start only, "
                         "which just returns the best construction heuristic)")
    # Early stop for BRKGA. Without it the solver runs all 5000 generations
    # -- over 15 minutes on this GPU (~4 gen/s), for a best that last moved
    # at the warm start (2026-10-16: 63.85 at gen 0, still 63.85 at gen
    # 1600). Measured convergence agrees: the incumbent stops improving
    # within the first few dozen generations. Either limit can be overridden
    # here or inside --brkga (time_limit_sec=..., stagnation_sec=...).
    ap.add_argument("--brkga-time-limit", type=float, default=20.0,
                    help="hard cap on BRKGA evolution, seconds "
                         "(default 20; 0 = no cap)")
    ap.add_argument("--brkga-stagnation", type=float, default=6.0,
                    help="stop BRKGA once the best makespan has not improved "
                         "for this many seconds (default 6; 0 = off)")
    args = ap.parse_args()
    brkga_kwargs = epj.parse_kv_args(args.brkga)
    brkga_kwargs.setdefault("time_limit_sec", args.brkga_time_limit)
    brkga_kwargs.setdefault("stagnation_sec", args.brkga_stagnation)

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)

        def _cors(self) -> None:
            # The page is opened from file:// (origin "null") — allow it.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self):  # noqa: N802 - CORS preflight
            self.send_response(204)
            self._cors()
            self.end_headers()

        def _plan_name(self) -> str:
            from urllib.parse import unquote
            return unquote(self.path.rstrip("/").split("/plans/", 1)[1])

        def do_GET(self):  # noqa: N802
            path = self.path.rstrip("/")
            if path in ("", "/health"):
                self._send(200, {"ok": True, "algorithms": list(epj.ALGORITHMS)})
            elif path == "/plans":
                self._send(200, {"dir": str(plan_store.PLANS_DIR),
                                 "plans": plan_store.list_plans()})
            elif path.startswith("/plans/"):
                try:
                    self._send(200, plan_store.read_plan(self._plan_name()))
                except FileNotFoundError:
                    self._send(404, {"error": "no such plan"})
                except plan_store.PlanStoreError as exc:
                    self._send(400, {"error": str(exc)})
            else:
                self._send(404, {"error": "unknown endpoint"})

        def do_DELETE(self):  # noqa: N802
            if not self.path.rstrip("/").startswith("/plans/"):
                self._send(404, {"error": "unknown endpoint"})
                return
            try:
                self._send(200, {"ok": True,
                                 "deleted": plan_store.delete_plan(self._plan_name())})
            except FileNotFoundError:
                self._send(404, {"error": "no such plan"})
            except plan_store.PlanStoreError as exc:
                self._send(400, {"error": str(exc)})

        def do_POST(self):  # noqa: N802
            path = self.path.rstrip("/")
            if path == "/plans":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    saved = plan_store.save_plan(
                        body.get("name", ""), body.get("plan"),
                        overwrite=bool(body.get("overwrite")))
                except plan_store.PlanStoreError as exc:
                    self._send(400, {"error": str(exc)})
                    return
                except Exception as exc:
                    self._send(400, {"error": f"{type(exc).__name__}: {exc}"})
                    return
                print(f"saved plan: generated_plans/{saved['name']}")
                self._send(201, {"ok": True, **saved})
                return
            if path != "/solve":
                self._send(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                t_solve = time.monotonic()
                payload = solve_request(body, args.repo, args.sec_per_hop,
                                        brkga_kwargs=brkga_kwargs)
                payload["solve_wall_sec"] = round(time.monotonic() - t_solve, 1)
            except SystemExit as exc:  # export_plan_json validation errors
                self._send(400, {"error": str(exc)})
                return
            except Exception as exc:
                self._send(400, {"error": f"{type(exc).__name__}: {exc}"})
                return
            print(f"solved: {payload['algorithm']} — "
                  f"{payload['solver_makespan_hops']:.1f} hops "
                  f"(~{payload['solver_makespan_sec_estimate']:.0f}s) "
                  f"in {payload['solve_wall_sec']:.1f}s")
            self._send(200, payload)

        def log_message(self, *a):
            pass  # keep the console to one line per solve

    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"plan server ready on http://localhost:{args.port} "
          f"(algorithms: {', '.join(epj.ALGORITHMS)})")
    print(f"solver repo: {args.repo}")
    print(f"brkga early stop: time limit {brkga_kwargs['time_limit_sec']:g}s, "
          f"or best unchanged for {brkga_kwargs['stagnation_sec']:g}s")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
