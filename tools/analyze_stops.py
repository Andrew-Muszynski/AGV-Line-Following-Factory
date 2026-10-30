"""Per-robot stopping distance and steering bias from real runs.

    python3 tools/analyze_stops.py <supervisor_live.log>
    python3 tools/analyze_stops.py <supervisor_live.log> --write

Fetch the log from the Linux laptop first (it is ~/supervisor_live.log
there). Poses come from the run database data/agv_metrics.sqlite3 (POSE_SAMPLE
events, stamped on the same Linux clock as the log).

STOPPING. drive_leg() fires one STOP when the robot is brake_lead_in from its
target and the robot coasts to rest; nothing corrects position afterwards.
Per leg:
    d_fire  how far from the target the brake fired (logged)
    rest    signed distance from the target at rest, along the leg
            (+ = past, - = short); the last pose before that robot's next
            navigator action, within 1.5 s of the brake
    coast   d_fire + rest = distance travelled after the STOP
The brake lead that centres the stops is the median coast. It does not depend
on the lead the data was taken with.

STEERING. drive_leg() steers on heading error alone (kp_yaw, proportional
only), so a robot whose wheels pull holds a standing heading error and bows
to one side of every leg. Measured mid-leg on lattice legs (> 0.4 s into the
leg, > 2.5 in from the target). The feed-forward that removes it is
    steer_trim_rpm = trim in force + kp_yaw * median heading error.

--write puts both into fleet/robot_drive_calibration.json. Only use data
taken under ONE calibration: --since defaults to that file's "applied" time.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sqlite3
import statistics as st
from bisect import bisect_left, bisect_right
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAL = ROOT / "fleet" / "robot_drive_calibration.json"
LINE = re.compile(r"\[(\d{10}\.\d+)\] \[camera_grid_navigate_(alvik\d)\]: (.*)")
DRIVE = re.compile(r"leg (\S+)->(\S+): driving from \(([-\d.]+),([-\d.]+)\) to \(([-\d.]+),([-\d.]+)\)")
BRAKE = re.compile(r"leg (\S+)->(\S+): brake fired \(([\d.]+)in short")
REST_WINDOW_SEC = 1.5


def leg_kind(a: str, b: str) -> str:
    def k(n):
        if n.startswith("DE"): return "depot_entry"
        if n.startswith("D"): return "depot_slot"
        if n == "0": return "node0"
        v = int(n)
        if v <= 64: return "lattice"
        if v <= 113: return "workstation"
        return "entry"
    ka, kb = k(a), k(b)
    if kb == "workstation": return "into_workstation"
    if kb == "depot_slot": return "into_depot_slot"
    if ka == "workstation": return "out_of_workstation"
    if kb == "entry" or ka == "entry": return "via_entry"
    if "depot" in ka or "depot" in kb or "node0" in (ka, kb): return "depot_lane"
    return "lattice"


def load_legs(log_path: str, since: float) -> list[dict]:
    legs, open_ = [], {}
    nxt = defaultdict(list)          # robot -> times of navigator lines
    for line in open(log_path, encoding="utf-8", errors="replace"):
        m = LINE.search(line)
        if not m:
            continue
        t, robot, msg = float(m.group(1)), m.group(2).capitalize(), m.group(3)
        if t < since:
            continue
        nxt[robot].append(t)
        d = DRIVE.search(msg)
        if d:
            open_[robot] = (t, d.group(1), d.group(2), *map(float, d.groups()[2:]))
            continue
        b = BRAKE.search(msg)
        if b and robot in open_ and open_[robot][1:3] == (b.group(1), b.group(2)):
            t0, a_, b_, x0, y0, tx, ty = open_.pop(robot)
            legs.append({"robot": robot, "from": a_, "to": b_, "t0": t0, "t": t,
                         "x0": x0, "y0": y0, "tx": tx, "ty": ty,
                         "d_fire": float(b.group(3)), "kind": leg_kind(a_, b_)})
    for leg in legs:
        times = nxt[leg["robot"]]
        i = bisect_right(times, leg["t"])
        leg["t_next"] = times[i] if i < len(times) else leg["t"] + REST_WINDOW_SEC
    return legs


def load_poses(db_path: str, since: float) -> dict:
    """robot -> (sorted times, [(x_in, y_in, yaw_deg)])"""
    c = sqlite3.connect(db_path)
    t_min = datetime.fromtimestamp(since - 5, timezone.utc).isoformat()[:19]
    raw = defaultdict(list)
    for rid, tu, xm, ym, yaw in c.execute(
            """select robot_id, time_utc, x_m, y_m, yaw_deg from events
               where event_type='POSE_SAMPLE' and time_utc >= ? and x_m is not null""",
            (t_min,)):
        try:
            ts = datetime.fromisoformat(tu.replace("Z", "+00:00"))
        except ValueError:
            continue
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        raw[rid].append((ts.timestamp(), xm / 0.0254, ym / 0.0254, yaw))
    out = {}
    for rid, v in raw.items():
        v.sort()
        out[rid] = ([p[0] for p in v], [p[1:] for p in v])
    return out


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def attach(legs: list[dict], poses: dict) -> list[dict]:
    for leg in legs:
        ts, ps = poses.get(leg["robot"], ([], []))
        ux, uy = leg["tx"] - leg["x0"], leg["ty"] - leg["y0"]
        length = math.hypot(ux, uy)
        if length < 1.0:
            continue
        ux, uy = ux / length, uy / length
        leg["length"] = length
        # Rest pose.
        i = bisect_right(ts, min(leg["t_next"], leg["t"] + REST_WINDOW_SEC)) - 1
        if i >= 0 and ts[i] > leg["t"]:
            x, y, _ = ps[i]
            rest = (x - leg["tx"]) * ux + (y - leg["ty"]) * uy          # + past
            lateral = -(x - leg["tx"]) * uy + (y - leg["ty"]) * ux      # + left
            if abs(rest) <= 4 and abs(lateral) <= 4:                     # a real stop
                leg.update(rest=rest, lateral=lateral, coast=leg["d_fire"] + rest)
        # Final resting place for arrivals that stay put (depot slot,
        # workstation): 6 s after the brake, i.e. after the square-up turn.
        # Measured against the TRUE node coordinate the log prints, so it is
        # the residual left over after any aim correction already in force.
        if leg["kind"] in ("into_depot_slot", "into_workstation"):
            j = bisect_right(ts, leg["t"] + 6.0) - 1
            if j >= 0 and ts[j] > leg["t"] + 3.0:
                x, y, _ = ps[j]
                fl = -(x - leg["tx"]) * uy + (y - leg["ty"]) * ux
                fa = (x - leg["tx"]) * ux + (y - leg["ty"]) * uy     # + past
                if abs(fl) <= 3 and abs(fa) <= 3:
                    leg["final_left"] = fl
                    leg["final_along"] = fa
        # Mid-leg heading error, as drive_leg() computes it.
        if leg["kind"] == "lattice" and length >= 8:
            errs = []
            for k in range(bisect_left(ts, leg["t0"] + 0.4), bisect_right(ts, leg["t"])):
                x, y, yaw = ps[k]
                dx, dy = leg["tx"] - x, leg["ty"] - y
                if math.hypot(dx, dy) >= 2.5:
                    errs.append(_wrap(math.degrees(math.atan2(dx, -dy)) - yaw))
            leg["heading_err"] = errs
    return legs


def next_trim(cur: dict, trim_now: float, err_now: float, kp: float) -> float:
    """The trim that should zero the standing heading error.

    First pass (no earlier measurement): trim_now + kp * err -- assumes the
    error responds to trim at 1/kp deg per RPM. It does not quite: the first
    pass on 2026-10-22 over-corrected every robot (Alvik3 -4.75deg at trim 0
    became +2.79deg at -7.72 RPM; the real response is ~1 deg/RPM, not
    0.62). So once two trims have been measured, use the straight line
    through them (secant), which is that robot's measured response."""
    samples = [tuple(s) for s in cur.get("trim_samples", [])]
    if not samples and "heading_err_deg_before" in cur:
        samples = [(0.0, float(cur["heading_err_deg_before"]))]
    prev = [s for s in samples if abs(s[0] - trim_now) > 0.2]
    if prev:
        t0, e0 = prev[-1]
        if abs(err_now - e0) > 0.3:
            return round(trim_now - err_now * (trim_now - t0) / (err_now - e0), 2)
    return round(trim_now + kp * err_now, 2)


def summary(vals):
    return {"n": len(vals), "median": round(st.median(vals), 3),
            "mean": round(st.mean(vals), 3), "sd": round(st.pstdev(vals), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--since", help="ISO date/time (local); default: the "
                    "calibration file's 'applied' time, else 2026-10-15")
    ap.add_argument("--db", default=str(ROOT / "data" / "agv_metrics.sqlite3"))
    ap.add_argument("--kp-yaw", type=float, default=1.625,
                    help="kp_yaw the data was driven with (fleetSupervisor default)")
    ap.add_argument("--min-legs", type=int, default=100)
    ap.add_argument("--trim-only", action="store_true",
                    help="propose/write steering trims only; keep the brake leads")
    ap.add_argument("--aim-only", action="store_true",
                    help="propose/write the per-robot aim corrections only")
    ap.add_argument("--min-aim-legs", type=int, default=5,
                    help="arrivals of one kind needed before an aim is proposed")
    ap.add_argument("--write", action="store_true",
                    help=f"write the proposal to {CAL.relative_to(ROOT)}")
    ap.add_argument("--json", help="also dump the full summary here")
    a = ap.parse_args()

    current = json.loads(CAL.read_text(encoding="utf-8")) if CAL.exists() else {"robots": {}}
    since_s = a.since or current.get("applied") or "2026-10-15"
    since = datetime.fromisoformat(since_s).timestamp()
    legs = attach(load_legs(a.log, since), load_poses(a.db, since))
    stops = [l for l in legs if "coast" in l]
    print(f"{len(stops)} legs with a measured rest pose since {since_s}\n")

    out = {"since": since_s, "robots": {}, "by_kind": {}}
    proposal = {}
    print("robot   legs  coast med (sd)  rest med  lateral med  heading err med   "
          "-> brake_lead_in  steer_trim_rpm")
    for robot in sorted({l["robot"] for l in stops}):
        v = [l for l in stops if l["robot"] == robot]
        he = [e for l in legs if l["robot"] == robot for e in l.get("heading_err", [])]
        co = summary([l["coast"] for l in v])
        rs = summary([l["rest"] for l in v])
        la = summary([l["lateral"] for l in v])
        hs = summary(he) if he else None
        cur = current.get("robots", {}).get(robot, {})
        trim0 = float(cur.get("steer_trim_rpm", 0.0))
        lead = round(co["median"], 2) if not a.trim_only else cur.get("brake_lead_in", round(co["median"], 2))
        trim = next_trim(cur, trim0, hs["median"], a.kp_yaw) if hs else trim0
        out["robots"][robot] = {"coast": co, "rest": rs, "lateral": la, "heading_err": hs}
        if co["n"] >= a.min_legs:
            proposal[robot] = {"brake_lead_in": lead, "steer_trim_rpm": trim}
        print(f"{robot}  {co['n']:4d}   {co['median']:5.2f} ({co['sd']:.2f})  "
              f"{rs['median']:+5.2f}     {la['median']:+5.2f}        "
              f"{(hs or {}).get('median', float('nan')):+5.2f} deg        "
              f"{lead:5.2f}          {trim:+5.2f}"
              + ("" if co["n"] >= a.min_legs else f"   (under {a.min_legs} legs: not proposed)"))
    print("\nby leg kind (all robots)")
    for k in sorted({l["kind"] for l in stops}):
        v = [l for l in stops if l["kind"] == k]
        out["by_kind"][k] = {"coast": summary([l["coast"] for l in v]),
                             "rest": summary([l["rest"] for l in v])}
        print(f"  {k:20s} n={len(v):4d}  coast {st.median([l['coast'] for l in v]):5.2f}  "
              f"rest {st.median([l['rest'] for l in v]):+5.2f}  "
              f"length {st.median([l['length'] for l in v]):5.1f} in")
    # Aim corrections: where each robot finally rests beside its depot slot /
    # workstation targets, + = to its left. New aim = aim in force + residual.
    # Below 0.15in it is noise-chasing, so a robot with no aim yet keeps none.
    print("\nfinal resting offset after any aim in force"
          " (left: + = robot's left; along: + = past the node)")
    aim_prop, along_prop = {}, {}
    for robot in sorted({l["robot"] for l in legs}):
        rcal = current.get("robots", {}).get(robot, {})
        for field, cal_key, store in (("final_left", "aim_lateral_in", aim_prop),
                                      ("final_along", "aim_along_in", along_prop)):
            cur = rcal.get(cal_key, {}) or {}
            new = dict(cur)
            for kind, key in (("into_depot_slot", "depot_slot"), ("into_workstation", "workstation")):
                v = [l[field] for l in legs if l["robot"] == robot
                     and l["kind"] == kind and field in l]
                if len(v) < a.min_aim_legs:
                    continue
                res = st.median(v)
                old = float(cur.get(key, 0.0))
                prop = round(old + res, 2)
                if old == 0.0 and abs(res) < 0.15:
                    prop = 0.0
                new[key] = prop
                print(f"  {robot} {key:12s} {field[6:]:5s} n={len(v):3d} residual {res:+.2f} "
                      f"(sd {st.pstdev(v):.2f})  aim {old:+.2f} -> {prop:+.2f}")
            store[robot] = {k: v for k, v in new.items() if abs(v) > 0}
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1), encoding="utf-8")
    if a.write and a.aim_only:
        robots = dict(current.get("robots", {}))
        for r, aim in aim_prop.items():
            robots[r] = {**robots.get(r, {}), "aim_lateral_in": aim,
                         "aim_along_in": along_prop.get(r, {})}
        blob = {**{k: v for k, v in current.items() if k != "robots"},
                "applied": datetime.now().isoformat(timespec="seconds"),
                "derived_from": f"{Path(a.log).name}, legs since {since_s} (aim)",
                "robots": robots}
        CAL.write_text(json.dumps(blob, indent=2) + "\n", encoding="utf-8")
        print(f"\nwrote aim corrections to {CAL}")
        return
    if a.write:
        robots = dict(current.get("robots", {}))
        for r, p in proposal.items():
            old = robots.get(r, {})
            he = (out["robots"][r]["heading_err"] or {}).get("median")
            samples = [list(s) for s in old.get("trim_samples", [])]
            if not samples and "heading_err_deg_before" in old:
                samples = [[0.0, old["heading_err_deg_before"]]]
            if he is not None:
                samples.append([float(old.get("steer_trim_rpm", 0.0)), he])
            p = {**p, "trim_samples": samples[-6:]}
            robots[r] = {**old, **p,
                         "legs": out["robots"][r]["coast"]["n"],
                         "coast_sd_in": out["robots"][r]["coast"]["sd"],
                         "heading_err_deg_before": (out["robots"][r]["heading_err"] or {}).get("median"),
                         "lateral_in_before": out["robots"][r]["lateral"]["median"]}
        blob = {**{k: v for k, v in current.items() if k != "robots"},
                "applied": datetime.now().isoformat(timespec="seconds"),
                "derived_from": f"{Path(a.log).name}, legs since {since_s}, kp_yaw {a.kp_yaw}",
                "robots": robots}
        CAL.write_text(json.dumps(blob, indent=2) + "\n", encoding="utf-8")
        print(f"\nwrote {CAL}")


if __name__ == "__main__":
    main()
