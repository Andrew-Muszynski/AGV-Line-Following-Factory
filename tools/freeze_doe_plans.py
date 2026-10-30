"""Freeze the DOE's dispatch plans: one file per robot count x traffic method.

    python3 tools/freeze_doe_plans.py --plan6 generated_plans/<6-robot plan>.json --plan4 generated_plans/<4-robot plan>.json

Every DOE run must drive exactly the same routes for its treatment. The job
allocation is already frozen in the two saved plans (BRKGA itself is NOT
reproducible from its seed: parallel workers and a wall-clock stop). This
tool plans each of them once with A*, WAIT, PP and CBS -- the same call the
Run & Monitor buttons make -- and saves the 8 results into generated_plans/
as DOE_6r_astar.json ... DOE_4r_cbs.json. Each run then loads its file from
the Saved plans dropdown instead of re-planning.

It also writes docs/planning/DOE_FROZEN_MANIFEST.json: a SHA-256 of every input and
output, the planner results, and the code version, so it can be shown later
that every run of a treatment used the same routes. Re-running with the
same inputs and code reproduces the same hashes (the planners were checked
deterministic on 2026-10-19: identical output across processes and hash
seeds, each finishing in under 0.2 s of its 30 s limit).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from traffic import instance as traffic_instance  # noqa: E402
from traffic.export import to_page_plan  # noqa: E402
from traffic.planners import solve  # noqa: E402
from traffic.validate import metrics as traffic_metrics  # noqa: E402
from traffic.validate import validate as traffic_validate  # noqa: E402

METHODS = ("astar", "wait", "pp", "cbs")
TIME_LIMIT_SEC = 30.0       # the same limit the Run & Monitor buttons use


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def code_version() -> dict:
    files = ["traffic/planners.py", "traffic/sipp.py", "traffic/conflicts.py",
             "traffic/instance.py", "traffic/model.py", "traffic/export.py"]
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = None
    return {"git_commit": commit,
            "files_sha256": {f: sha256(ROOT / f) for f in files if (ROOT / f).exists()}}


def freeze(plan_path: Path, robots: int, out_dir: Path, prefix: str) -> list:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if int(plan.get("agents", 0)) != robots:
        sys.exit(f"{plan_path.name} has {plan.get('agents')} agents, expected {robots}")
    rows = []
    for method in METHODS:
        inst = traffic_instance.load_data(plan, plan_path.stem)
        res = solve(inst, method, time_limit=TIME_LIMIT_SEC, seed=0)
        if not res.paths:
            sys.exit(f"{robots}r {method}: no plan -- {res.note}")
        report = traffic_validate(inst, res.paths)
        row = traffic_metrics(inst, res.paths, report)
        page = to_page_plan(inst, res.paths, method, report, row)
        page["job_planner"] = plan.get("route_mode") or "unknown"
        page["doe"] = {"robots": robots, "method": method,
                       "job_plan": plan_path.name,
                       "job_plan_sha256": sha256(plan_path)}
        out = out_dir / f"{prefix}_{robots}r_{method}.json"
        out.write_text(json.dumps(page, indent=1), encoding="utf-8")
        rows.append({"file": out.name, "sha256": sha256(out), "robots": robots,
                     "method": method, "valid": report["valid"],
                     "conflicts_left": len(report["conflicts"]),
                     "planned_makespan_sec": row.get("planned_makespan_sec"),
                     "planning_sec": round(res.planning_sec, 3),
                     "note": res.note})
        print(f"  {out.name:24s} planned makespan {row.get('planned_makespan_sec'):7.1f} s"
              f"  conflicts left {len(report['conflicts']):2d}  {res.note or ''}")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--plan6", type=Path, help="the frozen 6-robot job plan")
    ap.add_argument("--plan4", type=Path, help="the frozen 4-robot job plan")
    ap.add_argument("--out", type=Path, default=ROOT / "generated_plans")
    ap.add_argument("--prefix", default="DOE")
    args = ap.parse_args()
    if not args.plan6 and not args.plan4:
        ap.error("give --plan6 and/or --plan4")
    # Beside the design document, not in generated_plans/: the Saved plans
    # dropdown lists every .json there and cannot load a manifest.
    manifest_path = ROOT / "docs" / "planning" / f"{args.prefix}_FROZEN_MANIFEST.json"
    manifest = (json.loads(manifest_path.read_text(encoding="utf-8"))
                if manifest_path.exists() else {"job_plans": {}, "plans": []})
    for robots, path in ((6, args.plan6), (4, args.plan4)):
        if not path:
            continue
        path = path if path.is_absolute() else ROOT / path
        print(f"{robots} robots: {path.name}")
        manifest["job_plans"][f"{robots}r"] = {"file": path.name, "sha256": sha256(path)}
        manifest["plans"] = [p for p in manifest["plans"] if p["robots"] != robots]
        manifest["plans"] += freeze(path, robots, args.out, args.prefix)
    for r in (6, 4):
        pp = next((p for p in manifest["plans"] if p["robots"] == r and p["method"] == "pp"), None)
        if pp:
            manifest.setdefault("deadlines_sec", {})[f"{r}r"] = round(2 * pp["planned_makespan_sec"], 1)
    manifest["frozen_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    manifest["time_limit_sec"] = TIME_LIMIT_SEC
    manifest["code"] = code_version()
    manifest_path.write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    print(f"deadlines (2 x PP planned makespan): {manifest.get('deadlines_sec')}")
    print(f"manifest: {manifest_path}")


if __name__ == "__main__":
    main()
