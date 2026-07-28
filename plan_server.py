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
    GET  /health -> {"ok": true, "algorithms": [...]}
    POST /solve  <- {"algorithm": "regret", "config": {<Download JSON shape>}}
                 -> {"algorithm": "Max Regret", "visits": {...}, ...}

Runs the solvers one at a time (they are CPU-heavy); the HTML waits on the
fetch. Reuses export_plan_json.py for all solver plumbing, so the dropdown
and the offline exporter can never disagree.
"""

from __future__ import annotations

import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import export_plan_json as epj

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
    args = ap.parse_args()
    brkga_kwargs = epj.parse_kv_args(args.brkga)

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
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self):  # noqa: N802 - CORS preflight
            self.send_response(204)
            self._cors()
            self.end_headers()

        def do_GET(self):  # noqa: N802
            if self.path.rstrip("/") in ("", "/health"):
                self._send(200, {"ok": True, "algorithms": list(epj.ALGORITHMS)})
            else:
                self._send(404, {"error": "unknown endpoint"})

        def do_POST(self):  # noqa: N802
            if self.path.rstrip("/") != "/solve":
                self._send(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                payload = solve_request(body, args.repo, args.sec_per_hop,
                                        brkga_kwargs=brkga_kwargs)
            except SystemExit as exc:  # export_plan_json validation errors
                self._send(400, {"error": str(exc)})
                return
            except Exception as exc:
                self._send(400, {"error": f"{type(exc).__name__}: {exc}"})
                return
            print(f"solved: {payload['algorithm']} — "
                  f"{payload['solver_makespan_hops']:.1f} hops "
                  f"(~{payload['solver_makespan_sec_estimate']:.0f}s)")
            self._send(200, payload)

        def log_message(self, *a):
            pass  # keep the console to one line per solve

    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"plan server ready on http://localhost:{args.port} "
          f"(algorithms: {', '.join(epj.ALGORITHMS)})")
    print(f"solver repo: {args.repo}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
