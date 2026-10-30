#!/usr/bin/env python3
"""plan_store.py -- the generated_plans/ folder, shared by both servers.

The solver page can save a solved plan straight into the repo and reload it
later from a dropdown, instead of downloading to Downloads/ and importing the
file back by hand. That matters most for BRKGA, where a solve is minutes of
CPU and the result is worth keeping.

WHY A SEPARATE FOLDER. The repo already holds three kinds of JSON and mixing
a fourth into any of them would be confusing:

    generated_plans/  THIS STORE -- full page exports, saved deliberately
                      from the dashboard. Hand-picked, meant to be re-run.
    solved_plans/     every plan_server /solve result, archived automatically
                      so a stochastic run can be replayed. Append-only, noisy.
    runs/             records of runs that were actually executed.
    fleet/routes/     per-robot route files for the supervisor, not plans.

WHY BOTH SERVERS IMPORT THIS. START_TESTBED.bat runs app.py (Flask, :8000)
and not plan_server.py; start_dashboard.bat runs plan_server.py (:8082) and
not app.py. Whichever is up has to answer, so the logic lives here once and
each server exposes it under its own prefix:

    app.py        GET  /api/plans            POST /api/plans
                  GET  /api/plans/<name>     DELETE /api/plans/<name>
    plan_server   GET  /plans                POST /plans
                  GET  /plans/<name>         DELETE /plans/<name>
"""

from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path

#: Sits next to this file, i.e. the repo root, so both servers agree on it
#: no matter which directory they were started from.
PLANS_DIR = Path(__file__).resolve().parent / "generated_plans"

#: Conservative on purpose. The name arrives from a browser and is used to
#: build a path, so anything outside this set is rejected rather than
#: sanitised -- a silent rewrite would let two different requests land on one
#: file. Rejecting also blocks "..", absolute paths and NTFS alternate
#: streams without needing a separate check for each.
_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,120}$")

MAX_PLAN_BYTES = 32 * 1024 * 1024


class PlanStoreError(ValueError):
    """Bad request: unusable name, missing plan, oversized body."""


def ensure_dir() -> Path:
    PLANS_DIR.mkdir(parents=True, exist_ok=True)
    return PLANS_DIR


def safe_name(name: str) -> str:
    """Validate a client-supplied filename and return it, or raise.

    Anything path-like is REJECTED, not trimmed down to its basename.
    Reducing "../etc/passwd" to "passwd.json" keeps the write inside the
    folder, so it is not a traversal, but it stores the plan under a name
    the caller never asked for -- and two different requests can collapse
    onto one file that way. A rejection is the honest answer.
    """
    raw = str(name or "").strip()
    if any(ch in raw for ch in ("/", "\\", ":")) or raw != os.path.basename(raw):
        raise PlanStoreError(
            f"plan name must be a bare filename, not a path: {name!r}")
    base = raw if raw.lower().endswith(".json") else raw + ".json"
    if not _SAFE_NAME.match(base) or base.startswith("."):
        raise PlanStoreError(
            f"unusable plan name {name!r}: use letters, digits, dot, dash "
            f"and underscore only")
    resolved = (ensure_dir() / base).resolve()
    if resolved.parent != PLANS_DIR.resolve():
        raise PlanStoreError(f"plan name escapes the store: {name!r}")
    return base


def _summarise(path: Path) -> dict:
    """One dropdown row. Never raises -- a corrupt file still gets listed,
    flagged, so it can be found and deleted rather than silently vanishing."""
    st = path.stat()
    row = {
        "name": path.name,
        "size": st.st_size,
        "modified": time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.localtime(st.st_mtime)),
        "modified_epoch": st.st_mtime,
    }
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        row["error"] = f"{type(exc).__name__}: {exc}"
        return row
    if not isinstance(data, dict):
        row["error"] = "not a JSON object"
        return row

    routes = data.get("routes")
    visits = data.get("visits")
    if isinstance(routes, list) and routes:
        row["agents"] = len(routes)
        row["bays"] = sum(len(r.get("visits") or []) for r in routes
                          if isinstance(r, dict))
    elif isinstance(visits, dict) and visits:
        row["agents"] = len(visits)
        row["bays"] = sum(len(v or []) for v in visits.values())

    for key in ("route_mode", "algorithm", "generated_at"):
        if isinstance(data.get(key), str):
            row[key] = data[key]
    for key in ("makespan_sec", "solver_makespan_sec_estimate", "capacity"):
        val = data.get(key)
        if isinstance(val, (int, float)):
            row[key] = round(float(val), 2)
    rpd = data.get("rpd")
    if isinstance(rpd, dict):
        row["rpd"] = bool(rpd.get("enabled"))
        if isinstance(rpd.get("process_sec"), (int, float)):
            row["process_sec"] = float(rpd["process_sec"])
    return row


def list_plans() -> list[dict]:
    """Newest first -- the plan you just saved is the one you want next."""
    ensure_dir()
    rows = [_summarise(p) for p in PLANS_DIR.glob("*.json") if p.is_file()]
    rows.sort(key=lambda r: r["modified_epoch"], reverse=True)
    return rows


def read_plan(name: str) -> dict:
    path = PLANS_DIR / safe_name(name)
    if not path.is_file():
        raise FileNotFoundError(name)
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_plan(name: str, plan: dict, overwrite: bool = False) -> dict:
    """Write `plan`, returning {name, path, bytes}.

    Without `overwrite`, an existing name gets _2, _3 ... appended rather
    than being replaced. Saving twice from the page is far more likely to be
    a second variant worth keeping than an intent to destroy the first.
    """
    if not isinstance(plan, dict) or not plan:
        raise PlanStoreError("missing 'plan' object")
    body = json.dumps(plan, indent=2)
    if len(body.encode("utf-8")) > MAX_PLAN_BYTES:
        raise PlanStoreError(
            f"plan is larger than the {MAX_PLAN_BYTES // (1024*1024)} MB limit")

    final = safe_name(name)
    path = ensure_dir() / final
    if path.exists() and not overwrite:
        stem, suffix = final[:-5], ".json"
        for n in range(2, 1000):
            cand = f"{stem}_{n}{suffix}"
            if not (PLANS_DIR / cand).exists():
                final, path = cand, PLANS_DIR / cand
                break
        else:
            raise PlanStoreError(f"too many plans named like {final}")

    # Write to a temp file in the same folder, then replace, so an
    # interrupted save cannot leave a half-written plan that lists fine and
    # fails on load.
    tmp = path.with_suffix(".json.part")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(body)
    os.replace(tmp, path)
    return {"name": final, "path": str(path), "bytes": len(body)}


def delete_plan(name: str) -> str:
    final = safe_name(name)
    path = PLANS_DIR / final
    if not path.is_file():
        raise FileNotFoundError(name)
    path.unlink()
    return final
