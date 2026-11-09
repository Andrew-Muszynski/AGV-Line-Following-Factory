#!/usr/bin/env python3
"""Experiment 2: where the critical robot's time goes, per counted run.

    python3 tools/exp2_critical_robot.py [--logs data/exp2/logs] [--csv out.csv]
    python3 tools/exp2_critical_robot.py --table      # Table 5.9 of the thesis
    python3 tools/exp2_critical_robot.py --example 7I # one run, interval by interval

The critical robot is the one that finishes its plan last; its finish sets
the makespan. Each of its seconds, from the start of the run to its own
"plan complete", gets exactly ONE label, in this priority order (a moment
covered by two intervals takes the higher one):

  recovery     a failed leg until it is redispatched, a guard back-off and
               the hold after it, a deadlock yield until it ends
  processing   a supervisor hold waiting for a part (pick-up readiness)
  traffic      any other supervisor hold, and a rotation held for clearance
  planned      a planned wait from the plan, executed as a timed DWELL <ms>
               before the move it precedes (the dashboard converts each
               segment's wait_before_sec into these commands)
  service      the untimed DWELL at a workstation
  motion       driving a leg, and turning (including the settle passes)
  other        everything left. It is broken down further by what the robot
               did last before each such second (after a drive, after a turn,
               after a dispatch, after a hold was released, ...); see
               OTHER_AFTER.

Source: the supervisor transcripts archived in data/exp2/logs/ (one per run,
copied unchanged from the Linux laptop's runs/ folder). The metrics
database's state stream cannot do this: it records holds but not travel, so
a "processing" state there runs on through the next drive.
"""
import argparse
import csv
import json
import os
import re
import statistics as st

COUNTED = [  # (pair, form, log prefix), form I = integrated (joint)
    (1, "I", "20261103-1601"), (1, "S", "20261103-1607"),
    (2, "S", "20261103-1614"), (2, "I", "20261103-1625"),
    (3, "S", "20261103-1632"), (3, "I", "20261103-1638"),
    (4, "I", "20261103-1644"), (4, "S", "20261103-1702"),
    (5, "S", "20261109-0859"), (5, "I", "20261109-0915"),
    (6, "I", "20261109-0927"), (6, "S", "20261109-0933"),
    (7, "S", "20261109-0939"), (7, "I", "20261109-0950"),
    (8, "I", "20261109-0955"), (8, "S", "20261109-1002"),
]
PLAN = {"I": "generated_plans/ALNS24_joint.json",
        "S": "generated_plans/ALNS24_seq.json"}
PRIORITY = ["recovery", "processing", "traffic", "planned", "service", "motion"]
#: What the robot last did before an unclassified second.
OTHER_AFTER = [
    ("after a drive (brake to next step)", lambda n, m: n == "nav" and "brake fired" in m),
    ("after a turn or settle", lambda n, m: n == "nav" and ("turn complete" in m or ": settled " in m or "skipping turn" in m or "already within" in m)),
    ("stop verification", lambda n, m: n == "nav" and "STOP verified" in m),
    ("after a DWELL", lambda n, m: n == "nav" and "DWELL" in m),
    ("camera outage", lambda n, m: n == "nav" and "no camera pose" in m),
    ("dispatch to motion", lambda n, m: n == "sup" and "PLAN t=" in m),
    ("release to dispatch", lambda n, m: n == "sup" and (" wait over after" in m or "released" in m)),
]
LINE = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
PROCESSING_RE = re.compile(r"pick-up|part ready|waiting in bay|part has not been")
DT = 0.05


def parse(path):
    out = []
    for raw in open(path, encoding="utf-8", errors="replace"):
        m = LINE.match(raw.rstrip("\n"))
        if m:
            out.append((float(m.group(2)), m.group(3), m.group(4)))
    return out


def timeline(lines, robot):
    nav = f"camera_grid_navigate_{robot.lower()}"
    tag = f"[{robot}]"
    t0 = next(t for t, _n, msg in lines if msg.startswith("Advisor mode"))
    end = next((t for t, _n, msg in lines
                if msg.startswith(f"{tag} plan complete")), None)
    iv = []                                    # (start, end, label)
    open_ = {}                                 # kind -> start

    def close(kind, t, label):
        s = open_.pop(kind, None)
        if s is not None and t > s:
            iv.append((s, t, label))

    events = []                                # (t, "nav"|"sup", msg)
    for t, node, msg in lines:
        if node == nav:
            events.append((t, "nav", msg))
        elif node == "fleet_supervisor" and msg.startswith(tag):
            events.append((t, "sup", msg))
        if node == nav:
            if re.search(r": (driving|REVERSING) from", msg):
                open_["drive"] = t
            elif "brake fired" in msg:
                close("drive", t, "motion")
            if "turning from" in msg or "settle pass 1/" in msg:
                open_.setdefault("turn", t)
            elif ("turn complete" in msg or re.search(r": settled ", msg)
                  or "OUT OF TOLERANCE" in msg):
                close("turn", t, "motion")
            # A DWELL ends at the robot's next step, not at a firmware
            # status line (those arrive while it is still dwelling).
            step = "DWELL" not in msg and not msg.lstrip().startswith("[status]")
            if "dwell (DWELL): DWELL" in msg:
                open_["dwell"] = t
            elif "dwell" in open_ and step:
                close("dwell", t, "service")
            if re.search(r"dwell \(DWELL \d+\): DWELL \d+", msg):
                open_["planned"] = t
            elif "planned" in open_ and step:
                close("planned", t, "planned")
            if "rot" in open_ and re.search(
                    r"turning from|already within|backing up|REVERSING", msg):
                close("rot", t, "traffic")
        elif node == "fleet_supervisor":
            if msg.startswith(tag):
                # A hold runs from its FIRST "SUPERVISOR WAIT" line to its
                # release. "wait over after" alone undercounts: the
                # supervisor restarts that timer when a hold's reason changes.
                if " SUPERVISOR WAIT " in msg:
                    open_.setdefault("hold", t)
                m = re.match(re.escape(tag) + r" wait over after ([\d.]+)s \((.*)\)$", msg)
                if m:
                    dur, why = float(m.group(1)), m.group(2)
                    lab = ("recovery" if why.startswith("guard hold") else
                           "processing" if PROCESSING_RE.search(why) else "traffic")
                    start = min(open_.pop("hold", t - dur), t - dur)
                    iv.append((start, t, lab))
                if ": holding rotation --" in msg:
                    open_.setdefault("rot", t)
                if ("vision leg failed" in msg or "guard-stopped" in msg
                        or "yield leg failed" in msg):
                    # The primitive that failed (a turn that never moved, a
                    # drive stopped short) is recovery from its own start,
                    # not motion.
                    starts = [open_.pop(k) for k in ("turn", "drive") if k in open_]
                    open_.setdefault("rec", min(starts + [t]))
                if ("PLAN t=" in msg or "yield complete" in msg
                        or "yield abandoned" in msg or "guard hold over" in msg):
                    close("rec", t, "recovery")
            if re.match(r"DEADLOCK RESOLUTION: (sidestep|backtrack) " + robot + r"\b", msg):
                open_.setdefault("rec", t)
        if end is not None and t >= end:
            break
    for k in list(open_):
        close(k, end, {"drive": "motion", "turn": "motion", "dwell": "service",
                       "rot": "traffic", "rec": "recovery", "hold": "traffic",
                       "planned": "planned"}[k])
    n = int(round((end - t0) / DT))
    lab = ["other"] * n
    rank = {p: i for i, p in enumerate(PRIORITY)}
    for s, e, l in iv:
        a, b = max(0, int((s - t0) / DT)), min(n, int((e - t0) / DT))
        for k in range(a, b):
            if lab[k] == "other" or rank[l] < rank[lab[k]]:
                lab[k] = l
    out = {p: lab.count(p) * DT for p in PRIORITY + ["other"]}
    out["finish"] = end - t0
    out["intervals"] = sorted(iv)
    out["t0"] = t0
    # Unclassified seconds, by what the robot did last before them.
    ev = [(t, n, m) for t, n, m in events if t <= end]
    k_ev, last = 0, "other"
    after = {name: 0.0 for name, _f in OTHER_AFTER}
    after["other"] = 0.0
    for k in range(n):
        tk = t0 + k * DT
        while k_ev < len(ev) and ev[k_ev][0] <= tk:
            _t, src, m = ev[k_ev]
            for name, f in OTHER_AFTER:
                if f(src, m):
                    last = name
                    break
            k_ev += 1
        if lab[k] == "other":
            after[last] += DT
    out["other_after"] = after
    return out


def planned(form, robot):
    p = json.load(open(PLAN[form], encoding="utf-8"))
    route = next(r for r in p["routes"] if r["agv"] == robot)
    segs = route["segments"]
    return {"finish": max(s["end_sec"] for s in segs),
            "move": sum(s["move_sec"] for s in segs),
            "turn": sum(s["turn_sec"] for s in segs),
            "wait": sum(s["wait_before_sec"] for s in segs),
            "planned_critical": max(
                (max(s["end_sec"] for s in r["segments"]), r["agv"])
                for r in p["routes"])[1]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default=os.path.join("data", "exp2", "logs"))
    ap.add_argument("--csv", default=None)
    ap.add_argument("--table", action="store_true",
                    help="print Table 5.9: means, paired differences, 95%% CIs")
    ap.add_argument("--example", default=None,
                    help="pair and form, e.g. 7I: list the critical robot's "
                         "labeled intervals for that run")
    a = ap.parse_args()
    examples = {}
    rows = []
    for pair, form, pre in COUNTED:
        path = next(os.path.join(a.logs, f) for f in os.listdir(a.logs)
                    if f.startswith(pre))
        lines = parse(path)
        robots = sorted({m.group(1) for _t, _n, msg in lines
                         for m in [re.match(r"\[(Alvik\d)\] plan complete", msg)] if m})
        fin = {r: timeline(lines, r) for r in robots}
        crit = max(fin, key=lambda r: fin[r]["finish"])
        row = {"pair": pair, "form": form, "critical": crit, **{
            k: round(v, 1) for k, v in fin[crit].items()
            if k not in ("other_after", "intervals", "t0")}}
        row.update({f"other: {k}": round(v, 1)
                    for k, v in fin[crit]["other_after"].items()})
        pl = planned(form, crit)
        row.update({"planned_finish": round(pl["finish"], 1),
                    "planned_move": round(pl["move"], 1),
                    "planned_turn": round(pl["turn"], 1),
                    "planned_wait": round(pl["wait"], 1),
                    "planned_critical": pl["planned_critical"]})
        rows.append(row)
        examples[f"{pair}{form}"] = (crit, fin[crit])
    keys = list(rows[0])
    if a.example:
        crit, f = examples[a.example]
        print(f"pair {a.example[:-1]}, form {a.example[-1]}: critical robot {crit}, "
              f"finish {f['finish']:.1f} s")
        for s0, e0, lab in f["intervals"]:
            print(f"  {s0 - f['t0']:7.2f} - {e0 - f['t0']:7.2f} s  {e0 - s0:6.2f} s  {lab}")
        print("  totals:", {k: round(f[k], 1) for k in PRIORITY + ["other"]})
        return
    if a.table:
        from scipy import stats
        tc = stats.t.ppf(0.975, 7)
        groups = [(k, [k]) for k in ["motion", "traffic", "planned", "recovery",
                                     "service", "processing", "other"]]
        groups += [("other: " + name, ["other: " + name]) for name, _f in OTHER_AFTER]
        groups += [("other: other", ["other: other"]), ("finish", ["finish"])]
        print(f"{'category':45s} {'I':>7s} {'S':>7s} {'I-S':>7s}  95% CI            I<S ties")
        for g, ks in groups:
            I = {r["pair"]: sum(r[k] for k in ks) for r in rows if r["form"] == "I"}
            S = {r["pair"]: sum(r[k] for k in ks) for r in rows if r["form"] == "S"}
            d = [I[q] - S[q] for q in sorted(I)]
            m, sd = st.mean(d), st.stdev(d)
            h = tc * sd / len(d) ** 0.5
            print(f"{g:45s} {st.mean(I.values()):7.1f} {st.mean(S.values()):7.1f} "
                  f"{m:+7.1f}  ({m - h:+6.1f} to {m + h:+6.1f})  "
                  f"{sum(x < -1e-9 for x in d)}   {sum(abs(x) < 1e-9 for x in d)}")
        return
    print("  ".join(f"{k:>10}" for k in keys))
    for r in rows:
        print("  ".join(f"{str(r[k]):>10}" for k in keys))
    for form in ("I", "S"):
        sub = [r for r in rows if r["form"] == form]
        print(form, "mean", {k: round(st.mean(r[k] for r in sub), 1)
                             for k in PRIORITY + ["other", "finish"]},
              "critical robots", [r["critical"] for r in sub])
        print("   other by last event:", {k[7:]: round(st.mean(r[k] for r in sub), 1)
                                          for k in keys if k.startswith("other: ")})
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
