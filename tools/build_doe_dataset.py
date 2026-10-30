#!/usr/bin/env python3
"""Export the traffic-planning DOE from the metrics database for analysis.

    python3 tools/build_doe_dataset.py

Reads (never writes) data/agv_metrics.sqlite3, using the 40 run ids ticked
off in docs/planning/DOE_RUN_ORDER.txt. Writes data/doe/:

    doe_runs.csv           one row per DOE run (40): design columns, the
                           responses and the covariates
    doe_attempts.csv       every attempt of a DOE file from round 1 on, with
                           its classification (completion rate uses this)
    doe_robots.csv         one row per robot per DOE run
    doe_metrics_long.csv   every run-level metric of every DOE run
"""
import csv
import datetime as dt
import json
import re
import sqlite3
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DB = ROOT / "data" / "agv_metrics.sqlite3"
RUN_ORDER = ROOT / "docs" / "planning" / "DOE_RUN_ORDER.txt"
OUT = ROOT / "data" / "doe"
ROUND1_START = "2026-10-23T17:14"          # first DOE attempt (DOE_6r_pp_1)

METHODS = {"A*": "astar", "WAIT": "wait", "PP": "pp", "CBS": "cbs"}

# Attempts that are not one of the 40 runs, classified from their logs under
# the infrastructure-fault rule (DOE_TRAFFIC_PLANNING_2026-10-19.txt) before
# looking at the method. Anything aborted within 30 s with no robot failure
# is a false start and is classified automatically.
CLASSIFIED = {
    "2a6f2226": ("infrastructure_fault",
                 "execution defect: Alvik3 drove off on 140->38 (stale-pose "
                 "filter dropped its poses); fixed, repeated"),
    "e347b87a": ("infrastructure_fault",
                 "execution defect: Alvik3 stopped late out of bay 91, "
                 "straight-line brake never fired; fixed, repeated"),
    "b6603f8f": ("infrastructure_fault",
                 "robot non-execution (operator, 2026-10-28): a robot failed "
                 "to move and the fleet deadlocked, so robots could not "
                 "return; aborted at 279.7 s and repeated as DOE_4r_pp_6. Log: "
                 "Alvik3 STALLED_IDLE on 0->1, a fleet-wide camera stall at "
                 "227.6-230.9 s, then VISION_LOST (Alvik2, Alvik3) and "
                 "MODE_ACK_LOST (Alvik4)"),
}
BY_NOTE = [  # notes already written on the database row
    (r"Alvik1 WHEEL_FOLLOW_MODE", "infrastructure_fault",
     "Alvik1 WHEEL_FOLLOW_MODE ~3 s late twice, MODE_ACK_LOST then firmware "
     "WHEEL_CMD_TIMEOUT; fixed (8 s ack wait), repeated"),
]

RUN_METRICS = [
    "actual_makespan_sec", "planned_makespan_sec", "makespan_error_sec",
    "makespan_error_pct", "job_completion_rate", "mission_completion_success",
    "fleet_traffic_delay_robot_sec", "traffic_delay_per_job_sec",
    "traffic_stop_episodes", "mean_traffic_stop_duration_sec",
    "safety_stop_count", "conflict_count", "deadlock_count", "collision_count",
    "near_miss_count", "min_fleet_separation_m", "reroute_count",
    "robot_completion_spread_sec", "tail_completion_delay_sec",
    "job_flow_time_mean_sec", "job_flow_time_max_sec",
    "fleet_movement_duration_mean_sec", "total_distance_m",
    "vision_availability_pct", "vision_dropout_episodes",
    "pose_update_rate_hz", "command_success_rate", "robot_fault_count",
]


def read_run_order():
    """(run_no, round, order_in_round, file, robots, method, video_expected,
    run_id_prefix, video_noted, note) for every ticked row."""
    rows, rnd, order = [], 0, 0
    for line in RUN_ORDER.read_text(encoding="utf-8").splitlines():
        m = re.match(r"ROUND (\d)", line)
        if m:
            rnd, order = int(m.group(1)), 0
            continue
        m = re.match(r"\s+(\d+)\s+(DOE_\w+\.json)\s+(\d)\s+(\S+)\s+(DOE_\w+)"
                     r"\s+\[x\]\s+(\w{8})\s+(DOE_\w+)\s*(.*)$", line)
        if m:
            order += 1
            rows.append(dict(run_no=int(m.group(1)), round=rnd,
                             order_in_round=order, plan_file=m.group(2),
                             robots=int(m.group(3)),
                             method=METHODS[m.group(4)],
                             video_expected=m.group(5),
                             run_id_prefix=m.group(6),
                             video_noted=m.group(7),
                             sheet_note=m.group(8).strip()))
    return rows


def metric(c, run_id, key):
    """Highest revision, non-provisional first (as the metrics page does)."""
    r = c.execute("select value from metric_values where run_id=? and "
                  "scope_type='run' and metric_key=? order by "
                  "(status='provisional'), revision desc limit 1",
                  (run_id, key)).fetchone()
    return None if r is None else r[0]


def seconds(a, b):
    if not a or not b:
        return None
    return (dt.datetime.fromisoformat(b)
            - dt.datetime.fromisoformat(a)).total_seconds()


def write(name, rows):
    with open(OUT / name, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"  {name}: {len(rows)} rows")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    c = sqlite3.connect(f"file:{DB.as_posix()}?mode=ro", uri=True)
    order = read_run_order()
    assert len(order) == 40, f"expected 40 ticked runs, found {len(order)}"

    runs, robots, long_rows, doe_ids = [], [], [], set()
    for o in order:
        hit = c.execute("select * from runs where run_id like ?",
                        (o["run_id_prefix"] + "%",)).fetchall()
        assert len(hit) == 1, f"run {o['run_no']}: {len(hit)} db rows"
        cols = [d[0] for d in c.execute("select * from runs limit 0").description]
        r = dict(zip(cols, hit[0]))
        rid = r["run_id"]
        doe_ids.add(rid)
        assert r["status"] == "completed", f"run {o['run_no']} is {r['status']}"
        assert r["plan_file"] == o["plan_file"], f"run {o['run_no']} file"
        videos = c.execute("select title, file_name from run_videos where "
                           "run_id=? order by video_id", (rid,)).fetchall()
        bats = c.execute("select robot_id, start_battery_pct, end_battery_pct,"
                         " completion_elapsed_ms, terminal_yaw_deg, status "
                         "from run_robots where run_id=? order by robot_id",
                         (rid,)).fetchall()
        start_b = [b[1] for b in bats if b[1] is not None]
        jobs = c.execute("select count(*), sum(status='completed') from jobs "
                         "where run_id=?", (rid,)).fetchone()
        row = dict(
            run_no=o["run_no"], round=o["round"],
            order_in_round=o["order_in_round"], robots=o["robots"],
            method=o["method"], treatment=f"{o['robots']}r_{o['method']}",
            plan_file=o["plan_file"], run_id=rid,
            video_title=videos[0][0] if videos else "",
            video_files=";".join(v[1] for v in videos),
            video_in_run_order=o["video_noted"],
            started_at_utc=r["started_at_utc"], ended_at_utc=r["ended_at_utc"],
            jobs_total=jobs[0], jobs_completed=jobs[1],
            battery_start_min_pct=min(start_b) if start_b else None,
            battery_start_mean_pct=(round(sum(start_b) / len(start_b), 1)
                                    if start_b else None),
        )
        for k in RUN_METRICS:
            row[k] = metric(c, rid, k)
        # Counted from the event log: the collision guard's emergency stops
        # (COLLISION_AVOIDED), detected deadlocks, and failed robot commands.
        for col, ev in (("guard_interventions", "COLLISION_AVOIDED"),
                        ("deadlocks_detected", "DEADLOCK_DETECTED"),
                        ("command_failures", "COMMAND_FAILED")):
            row[col] = c.execute("select count(*) from events where run_id=? "
                                 "and event_type=?", (rid, ev)).fetchone()[0]
        runs.append(row)
        for b in bats:
            robots.append(dict(run_no=o["run_no"], round=o["round"],
                               robots=o["robots"], method=o["method"],
                               run_id=rid, robot_id=b[0],
                               start_battery_pct=b[1], end_battery_pct=b[2],
                               completion_sec=(None if b[3] is None
                                               else b[3] / 1000.0),
                               terminal_yaw_deg=b[4], status=b[5]))
        for k, v, s, rev in c.execute(
                "select metric_key, value, status, revision from metric_values"
                " where run_id=? and scope_type='run' order by metric_key, "
                "revision", (rid,)):
            long_rows.append(dict(run_no=o["run_no"], round=o["round"],
                                  robots=o["robots"], method=o["method"],
                                  run_id=rid, metric_key=k, value=v,
                                  status=s, revision=rev))

    # Every attempt of a DOE file from round 1 on, plus the one wrong-file run.
    attempts = []
    run_no_of = {r["run_id"]: r["run_no"] for r in runs}
    for rid, created, started, ended, status, pf, notes, rc, tm in c.execute(
            "select run_id, created_at_utc, started_at_utc, ended_at_utc, "
            "status, plan_file, notes, robot_count, traffic_method from runs "
            "where created_at_utc >= ? and (plan_file like 'DOE_%' or "
            "notes like 'DOE EXCLUDE - not a DOE run%') order by created_at_utc",
            (ROUND1_START,)):
        dur = seconds(started, ended)
        fails = c.execute("select count(*) from events where run_id=? and "
                          "event_type='COMMAND_FAILED'", (rid,)).fetchone()[0]
        video = c.execute("select title from run_videos where run_id=? order "
                          "by video_id limit 1", (rid,)).fetchone()
        if rid in doe_ids:
            cls, why = "valid", ""
        elif rid[:8] in CLASSIFIED:
            cls, why = CLASSIFIED[rid[:8]]
        elif notes and notes.startswith("DOE EXCLUDE - not a DOE run"):
            cls, why = "not_a_doe_run", notes
        elif notes and any(re.search(p, notes) for p, _, _ in BY_NOTE):
            cls, why = next((k, w) for p, k, w in BY_NOTE if re.search(p, notes))
        elif status in ("aborted", "created") and (dur is None or dur < 30) \
                and fails == 0:
            cls, why = "false_start", ("never started" if started is None else
                                       f"aborted after {dur:.0f} s, no robot "
                                       "failure")
        else:
            cls, why = "unclassified", notes or ""
        robots_n = int(pf[4]) if pf else rc
        method = pf[7:-5] if pf else tm
        attempts.append(dict(
            created_at_utc=created, run_id=rid, plan_file=pf or "",
            robots=robots_n, method=method, status=status,
            duration_sec=None if dur is None else round(dur, 1),
            command_failures=fails, video_title=video[0] if video else "",
            doe_run_no=run_no_of.get(rid, ""), classification=cls,
            reason=why))

    write("doe_runs.csv", runs)
    write("doe_attempts.csv", attempts)
    write("doe_robots.csv", robots)
    write("doe_metrics_long.csv", long_rows)
    odd = [r for r in runs if r["video_title"] != r["video_in_run_order"]]
    for r in odd:
        print(f"  note: run {r['run_no']} video is {r['video_title']} in the "
              f"database, {r['video_in_run_order']} in DOE_RUN_ORDER.txt")
    open_ = [a for a in attempts if a["classification"] == "unclassified"]
    for a in open_:
        print(f"  UNCLASSIFIED attempt {a['run_id'][:8]} {a['plan_file']}: "
              f"{a['reason'][:90]}")


if __name__ == "__main__":
    main()
