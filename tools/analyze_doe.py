#!/usr/bin/env python3
"""Statistical comparison for the traffic-planning DOE.

    python3 tools/build_doe_dataset.py     (first, to refresh data/doe/)
    python3 tools/analyze_doe.py [--margin 10]

Design: 4 methods x 2 robot counts, 5 rounds as complete blocks (balanced,
one run per treatment per round), so every sum of squares below is exact.

    y = round + robots + method + robots:method + error      (28 error df)
    and, per robot count, y = round + method + error          (12 error df)

Writes data/doe/DOE_ANALYSIS.txt (key results first) and prints it.
"""
import argparse
import csv
import itertools
import math
import statistics as st
from collections import defaultdict
from pathlib import Path

from scipy import stats

ROOT = Path(__file__).resolve().parent.parent
DOE = ROOT / "data" / "doe"
METHODS = ["astar", "wait", "pp", "cbs"]
LABEL = {"astar": "A*", "wait": "WAIT", "pp": "PP", "cbs": "CBS"}
ROBOTS = [4, 6]
ROUNDS = [1, 2, 3, 4, 5]

SECONDARY = [
    ("makespan_error_sec", "prediction error (actual - planned), s"),
    ("makespan_error_pct", "prediction error, % of planned"),
    ("guard_interventions", "collision-guard interventions"),
    ("traffic_stop_episodes", "traffic stop episodes"),
    ("fleet_traffic_delay_robot_sec", "fleet traffic delay, robot-s"),
    ("mean_traffic_stop_duration_sec", "mean traffic stop, s"),
    ("robot_completion_spread_sec", "first-to-last robot finish spread, s"),
    ("tail_completion_delay_sec", "tail completion delay, s"),
    ("job_flow_time_mean_sec", "mean job flow time, s"),
    ("total_distance_m", "total distance, m (pose noise, unreliable)"),
    ("min_fleet_separation_m", "minimum separation, m"),
    ("vision_dropout_episodes", "vision dropout episodes (nuisance)"),
    ("command_failures", "failed robot commands (nuisance)"),
]


def load():
    rows = list(csv.DictReader(open(DOE / "doe_runs.csv", encoding="utf-8")))
    for r in rows:
        r["robots"], r["round"] = int(r["robots"]), int(r["round"])
    return rows


def num(v):
    return None if v in ("", None) else float(v)


def two_way(rows, key):
    """Blocked 2x4 factorial ANOVA. Returns {term: (SS, df, MS, F, p)}."""
    y = {(r["round"], r["robots"], r["method"]): num(r[key]) for r in rows}
    if any(v is None for v in y.values()) or len(y) != 40:
        return None
    g = st.mean(y.values())
    mean = lambda f: st.mean(v for k, v in y.items() if f(k))
    b = {i: mean(lambda k: k[0] == i) for i in ROUNDS}
    a = {i: mean(lambda k: k[1] == i) for i in ROBOTS}
    m = {i: mean(lambda k: k[2] == i) for i in METHODS}
    cell = {(i, j): mean(lambda k: k[1] == i and k[2] == j)
            for i in ROBOTS for j in METHODS}
    ss_b = 8 * sum((v - g) ** 2 for v in b.values())
    ss_a = 20 * sum((v - g) ** 2 for v in a.values())
    ss_m = 10 * sum((v - g) ** 2 for v in m.values())
    ss_c = 5 * sum((v - g) ** 2 for v in cell.values())
    ss_am = ss_c - ss_a - ss_m
    ss_t = sum((v - g) ** 2 for v in y.values())
    ss_e = ss_t - ss_b - ss_c
    df = {"round": 4, "robots": 1, "method": 3, "robots:method": 3, "error": 28}
    ss = {"round": ss_b, "robots": ss_a, "method": ss_m,
          "robots:method": ss_am, "error": ss_e}
    mse = ss_e / 28
    out = {}
    for t in ("round", "robots", "method", "robots:method"):
        f = (ss[t] / df[t]) / mse if mse > 0 else math.inf
        out[t] = (ss[t], df[t], ss[t] / df[t], f, stats.f.sf(f, df[t], 28))
    out["error"] = (ss_e, 28, mse, None, None)
    resid = {k: v - cell[(k[1], k[2])] - b[k[0]] + g for k, v in y.items()}
    return out, resid


def one_way_blocked(rows, robots, key):
    """y = round + method within one robot count."""
    y = {(r["round"], r["method"]): num(r[key]) for r in rows
         if r["robots"] == robots}
    g = st.mean(y.values())
    b = {i: st.mean(y[(i, j)] for j in METHODS) for i in ROUNDS}
    m = {j: st.mean(y[(i, j)] for i in ROUNDS) for j in METHODS}
    ss_b = 4 * sum((v - g) ** 2 for v in b.values())
    ss_m = 5 * sum((v - g) ** 2 for v in m.values())
    ss_t = sum((v - g) ** 2 for v in y.values())
    ss_e = ss_t - ss_b - ss_m
    mse = ss_e / 12
    f = (ss_m / 3) / mse
    return dict(means=m, mse=mse, F=f, p=stats.f.sf(f, 3, 12),
                F_round=(ss_b / 4) / mse, p_round=stats.f.sf((ss_b / 4) / mse, 4, 12),
                y=y)


def tukey(res, n=5, df=12, k=4, conf=0.95):
    """Tukey HSD on the blocked error term. [(a, b, diff, lo, hi, p)]"""
    se = math.sqrt(res["mse"] / n)
    q = stats.studentized_range.ppf(conf, k, df)
    out = []
    for a, b in itertools.combinations(METHODS, 2):
        d = res["means"][a] - res["means"][b]
        p = stats.studentized_range.sf(abs(d) / se, k, df)
        out.append((a, b, d, d - q * se, d + q * se, p))
    return out


def tost(res, margin, n=5, df=12):
    """90% CI of each pairwise difference (blocked error), equivalence if
    inside +/- margin. [(a, b, diff, lo, hi, equivalent)]"""
    se = math.sqrt(2 * res["mse"] / n)
    t = stats.t.ppf(0.95, df)
    out = []
    for a, b in itertools.combinations(METHODS, 2):
        d = res["means"][a] - res["means"][b]
        lo, hi = d - t * se, d + t * se
        out.append((a, b, d, lo, hi, -margin < lo and hi < margin))
    return out


def friedman(res):
    y = res["y"]
    s = stats.friedmanchisquare(*[[y[(i, j)] for i in ROUNDS] for j in METHODS])
    return s.statistic, s.pvalue


def completion(path):
    """Per treatment: attempts, completed; infrastructure faults, unclassified."""
    t = defaultdict(lambda: defaultdict(int))
    for a in csv.DictReader(open(path, encoding="utf-8")):
        if a["classification"] in ("false_start", "not_a_doe_run"):
            continue
        k = (int(a["robots"]), a["method"])
        t[k]["attempts"] += 1
        t[k][a["classification"]] += 1
    return t


def write_xl(name, header, rows):
    """CSV with a UTF-8 BOM so Excel opens it with the right characters."""
    with open(DOE / name, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def r4(v):
    return "" if v is None else round(v, 4)


def pv(v):
    return "" if v is None else float(f"{v:.3g}")


def export_excel(rows, aov, per, cells, planned, comp, margin):
    """DOE_results.csv, DOE_cell_summary.csv, DOE_anova_table.csv,
    DOE_pairwise.csv - all for Excel."""
    order = {m: i for i, m in enumerate(METHODS)}
    srt = sorted(rows, key=lambda r: (r["robots"], order[r["method"]],
                                      r["round"]))
    cols = [("Run #", "run_no"), ("Round", "round"),
            ("Order in round", "order_in_round"), ("Robots", "robots"),
            ("Method", "method"), ("Video", "video_title"),
            ("Actual makespan (s)", "actual_makespan_sec"),
            ("Planned makespan (s)", "planned_makespan_sec"),
            ("Prediction error (s)", "makespan_error_sec"),
            ("Prediction error (%)", "makespan_error_pct"),
            ("Jobs completed", "jobs_completed"),
            ("Traffic stop episodes", "traffic_stop_episodes"),
            ("Mean traffic stop (s)", "mean_traffic_stop_duration_sec"),
            ("Fleet traffic delay (robot-s)", "fleet_traffic_delay_robot_sec"),
            ("Robot finish spread (s)", "robot_completion_spread_sec"),
            ("Tail completion delay (s)", "tail_completion_delay_sec"),
            ("Mean job flow time (s)", "job_flow_time_mean_sec"),
            ("Max job flow time (s)", "job_flow_time_max_sec"),
            ("Min separation (m)", "min_fleet_separation_m"),
            ("Collisions", "collision_count"), ("Deadlocks", "deadlock_count"),
            ("Vision dropout episodes", "vision_dropout_episodes"),
            ("Vision availability (%)", "vision_availability_pct"),
            ("Pose rate (Hz)", "pose_update_rate_hz"),
            ("Lowest start battery (%)", "battery_start_min_pct"),
            ("Mean start battery (%)", "battery_start_mean_pct"),
            ("Started (UTC)", "started_at_utc"), ("Run ID", "run_id")]
    out = []
    for r in srt:
        line = []
        for _, k in cols:
            v = r[k]
            if k == "method":
                v = LABEL[v]
            elif k in ("run_no", "round", "order_in_round", "robots",
                       "jobs_completed"):
                v = int(float(v))
            elif k not in ("video_title", "started_at_utc", "run_id") \
                    and v not in ("", None):
                v = round(float(v), 3)
            line.append(v)
        out.append(line)
    write_xl("DOE_results.csv", [c for c, _ in cols], out)

    summ = []
    for R in ROBOTS:
        for m in METHODS:
            v = cells[(R, m)]
            se = st.stdev(v) / math.sqrt(len(v))
            t = stats.t.ppf(0.975, len(v) - 1)
            c = comp[(R, m)]
            summ.append([R, LABEL[m], len(v), r4(st.mean(v)), r4(st.median(v)),
                         r4(st.stdev(v)), r4(se), r4(st.mean(v) - t * se),
                         r4(st.mean(v) + t * se), r4(min(v)), r4(max(v)),
                         r4(planned[(R, m)]), r4(st.mean(v) - planned[(R, m)]),
                         c["attempts"], c["valid"], c["infrastructure_fault"],
                         r4(c["valid"] / c["attempts"]),
                         r4(c["valid"] / (c["attempts"]
                                          - c["infrastructure_fault"]))])
    write_xl("DOE_cell_summary.csv",
             ["Robots", "Method", "n", "Mean makespan (s)", "Median (s)",
              "SD (s)", "SE (s)", "95% CI low (s)", "95% CI high (s)",
              "Min (s)", "Max (s)", "Planned (s)", "Mean - planned (s)",
              "Attempts", "Valid runs", "Infrastructure faults",
              "Completion rate (all attempts)",
              "Completion rate (excl. infrastructure faults)"], summ)

    tab = []
    ss_e = aov["error"][0]
    total_ss = sum(s[0] for s in aov.values())
    for t, s in aov.items():
        tab.append(["Two-way, both robot counts (n=40)", t, r4(s[0]), s[1],
                    r4(s[2]), r4(s[3]), pv(s[4]),
                    "" if s[3] is None else r4(s[0] / (s[0] + ss_e))])
    tab.append(["Two-way, both robot counts (n=40)", "total", r4(total_ss),
                39, "", "", "", ""])
    for R in ROBOTS:
        r = per[R]
        y = r["y"]
        g = st.mean(y.values())
        ss_t = sum((v - g) ** 2 for v in y.values())
        ss_m = 5 * sum((v - g) ** 2 for v in r["means"].values())
        ss_e1 = r["mse"] * 12
        ss_b = ss_t - ss_m - ss_e1
        name = f"One-way blocked, {R} robots (n=20)"
        tab += [[name, "round", r4(ss_b), 4, r4(ss_b / 4), r4(r["F_round"]),
                 pv(r["p_round"]), r4(ss_b / (ss_b + ss_e1))],
                [name, "method", r4(ss_m), 3, r4(ss_m / 3), r4(r["F"]),
                 pv(r["p"]), r4(ss_m / (ss_m + ss_e1))],
                [name, "error", r4(ss_e1), 12, r4(r["mse"]), "", "", ""],
                [name, "total", r4(ss_t), 19, "", "", "", ""]]
        fs, fp = friedman(r)
        tab.append([f"Friedman (rank-based), {R} robots", "method", "", 3, "",
                    f"chi2 = {fs:.3f}", pv(fp), ""])
    write_xl("DOE_anova_table.csv",
             ["Model", "Source", "SS", "df", "MS", "F", "p-value",
              "Partial eta squared"], tab)

    pw = []
    for R in ROBOTS:
        eq = {(a, b): (lo, hi, e) for a, b, _, lo, hi, e in tost(per[R], margin)}
        for a, b, d, lo, hi, p in tukey(per[R]):
            elo, ehi, e = eq[(a, b)]
            pw.append([R, LABEL[a], LABEL[b], r4(d), r4(lo), r4(hi), pv(p),
                       "yes" if p < 0.05 else "no", r4(elo), r4(ehi),
                       "yes" if e else "no"])
    write_xl("DOE_pairwise.csv",
             ["Robots", "Method A", "Method B", "Mean A - B (s)",
              "Tukey 95% CI low", "Tukey 95% CI high", "Tukey p",
              "Significant (0.05)", "90% CI low (TOST)", "90% CI high (TOST)",
              f"Equivalent within +/-{margin:g} s"], pw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type=float, default=10.0,
                    help="equivalence margin for makespan, s (default 10)")
    args = ap.parse_args()
    rows = load()
    L = []
    p = L.append

    ms = "actual_makespan_sec"
    cells = {(R, m): [num(r[ms]) for r in rows
                      if r["robots"] == R and r["method"] == m]
             for R in ROBOTS for m in METHODS}
    planned = {(R, m): num(next(r["planned_makespan_sec"] for r in rows
                                if r["robots"] == R and r["method"] == m))
               for R in ROBOTS for m in METHODS}
    aov, resid = two_way(rows, ms)
    per = {R: one_way_blocked(rows, R, ms) for R in ROBOTS}

    p("TRAFFIC-PLANNING DOE - STATISTICAL COMPARISON")
    p("Data: data/doe/doe_runs.csv (40 runs), doe_attempts.csv")
    p("Model: blocked 2x4 factorial, rounds as blocks, 5 runs per cell")
    p("")
    p("KEY RESULTS")
    p("=" * 70)
    for t in ("robots", "method", "robots:method", "round"):
        s = aov[t]
        p(f"  {t:14s} F({s[1]},28) = {s[3]:7.2f}   p = {s[4]:.4g}")
    for R in ROBOTS:
        r = per[R]
        best = min(METHODS, key=lambda m: r["means"][m])
        worst = max(METHODS, key=lambda m: r["means"][m])
        p(f"  {R} robots: method F(3,12) = {r['F']:.2f}, p = {r['p']:.4g};"
          f" fastest {LABEL[best]} {r['means'][best]:.1f} s,"
          f" slowest {LABEL[worst]} {r['means'][worst]:.1f} s")
    p("")

    p("CELL MEANS - ACTUAL MAKESPAN (s)")
    p("=" * 70)
    p(f"  {'robots':>6} {'method':6} {'n':>2} {'mean':>7} {'median':>7} "
      f"{'sd':>6} {'min':>7} {'max':>7} {'planned':>8} {'gap':>6}")
    for R in ROBOTS:
        for m in METHODS:
            v = cells[(R, m)]
            p(f"  {R:>6} {LABEL[m]:6} {len(v):>2} {st.mean(v):7.1f} "
              f"{st.median(v):7.1f} {st.stdev(v):6.1f} {min(v):7.1f} "
              f"{max(v):7.1f} {planned[(R, m)]:8.1f} "
              f"{st.mean(v) - planned[(R, m)]:6.1f}")
    p("  gap = mean actual - planned")
    p("")
    p("  Runs by round (s):")
    for R in ROBOTS:
        for m in METHODS:
            y = per[R]["y"]
            p(f"    {R}r {LABEL[m]:5s} " + "  ".join(
                f"R{i} {y[(i, m)]:6.1f}" for i in ROUNDS))
    p("")

    p("COMPLETION RATE (attempts exclude false starts and the wrong-file run)")
    p("=" * 70)
    comp = completion(DOE / "doe_attempts.csv")
    p(f"  {'cell':9} {'attempts':>8} {'valid':>6} {'infra':>6} {'open':>5} "
      f"{'rate all':>9} {'rate excl infra':>16}")
    for R in ROBOTS:
        for m in METHODS:
            c = comp[(R, m)]
            n, ok = c["attempts"], c["valid"]
            inf, op = c["infrastructure_fault"], c["unclassified"]
            excl = n - inf
            p(f"  {R}r {LABEL[m]:5s} {n:>8} {ok:>6} {inf:>6} {op:>5} "
              f"{ok / n:9.2f} {ok / excl:16.2f}")
    p("  open = unclassified attempt (counted as a failure in both rates);")
    p("  see doe_attempts.csv reason column.")
    p("")

    p("TWO-WAY ANOVA - ACTUAL MAKESPAN")
    p("=" * 70)
    p(f"  {'term':14} {'SS':>10} {'df':>3} {'MS':>10} {'F':>8} {'p':>10}")
    for t, s in aov.items():
        f = "" if s[3] is None else f"{s[3]:8.2f}"
        pv = "" if s[4] is None else f"{s[4]:10.4g}"
        p(f"  {t:14} {s[0]:10.1f} {s[1]:>3} {s[2]:10.1f} {f:>8} {pv:>10}")
    sw = stats.shapiro(list(resid.values()))
    bf = stats.levene(*cells.values(), center="median")
    p(f"  residual normality (Shapiro-Wilk): W = {sw.statistic:.3f}, "
      f"p = {sw.pvalue:.3g}")
    p(f"  equal variance across cells (Brown-Forsythe): F = {bf.statistic:.2f}"
      f", p = {bf.pvalue:.3g}")
    title = {(r["round"], r["robots"], r["method"]): r["video_title"]
             for r in rows}
    big = sorted(((e, title[k]) for k, e in resid.items()),
                 key=lambda t: -abs(t[0]))
    p("")

    for R in ROBOTS:
        r = per[R]
        p(f"{R} ROBOTS - METHOD COMPARISON (blocked by round)")
        p("=" * 70)
        p(f"  method F(3,12) = {r['F']:.2f}, p = {r['p']:.4g};  round "
          f"F(4,12) = {r['F_round']:.2f}, p = {r['p_round']:.4g};  "
          f"error sd = {math.sqrt(r['mse']):.1f} s")
        fs, fp = friedman(r)
        p(f"  Friedman (rank-based, blocked): chi2(3) = {fs:.2f}, p = {fp:.4g}")
        p("  Tukey HSD, 95% (difference = first - second, s):")
        for a, b, d, lo, hi, pv in tukey(r):
            mark = "  *" if pv < 0.05 else ""
            p(f"    {LABEL[a]:5s} - {LABEL[b]:5s} {d:7.1f}  "
              f"[{lo:7.1f}, {hi:7.1f}]  p = {pv:.4f}{mark}")
        p(f"  Equivalence (TOST, 90% CI inside +/-{args.margin:g} s):")
        for a, b, d, lo, hi, eq in tost(r, args.margin):
            p(f"    {LABEL[a]:5s} - {LABEL[b]:5s} {d:7.1f}  "
              f"[{lo:7.1f}, {hi:7.1f}]  "
              f"{'equivalent' if eq else 'not shown equivalent'}")
        p("")

    p("SECONDARY RESPONSES (same two-way model; p-values)")
    p("=" * 70)
    p(f"  {'response':40} {'robots':>8} {'method':>8} {'inter.':>8}")
    for key, name in SECONDARY:
        res = two_way(rows, key)
        if res is None:
            p(f"  {name:40} (missing values)")
            continue
        a = res[0]
        p(f"  {name:40} {a['robots'][4]:8.3g} {a['method'][4]:8.3g} "
          f"{a['robots:method'][4]:8.3g}")
    p("  Cell means:")
    for key, name in SECONDARY:
        p(f"    {name}")
        for R in ROBOTS:
            vals = []
            for m in METHODS:
                v = [num(r[key]) for r in rows
                     if r["robots"] == R and r["method"] == m]
                v = [x for x in v if x is not None]
                vals.append(f"{LABEL[m]} {st.mean(v):8.2f}" if v else
                            f"{LABEL[m]}     n/a")
            p(f"      {R}r  " + "   ".join(vals))
    p("")

    p("PER-ROBOT FINISH TIME (s, mean over 5 rounds)")
    p("=" * 70)
    rob = defaultdict(list)
    for r in csv.DictReader(open(DOE / "doe_robots.csv", encoding="utf-8")):
        if r["completion_sec"]:
            rob[(int(r["robots"]), r["method"], r["robot_id"])].append(
                float(r["completion_sec"]))
    for R in ROBOTS:
        p(f"  {R} robots     " + "".join(f"{LABEL[m]:>9}" for m in METHODS))
        for i in range(1, R + 1):
            a = f"Alvik{i}"
            p(f"    {a:10} " + "".join(
                f"{st.mean(rob[(R, m, a)]):9.1f}" if rob[(R, m, a)] else
                f"{'n/a':>9}" for m in METHODS))
    p("")

    p("NOTES")
    p("=" * 70)
    p("  - Rounds are complete blocks; run order was randomised within each")
    p("    round (seed 20261019).")
    p("  - Makespan is analysed on completed runs only; failures enter")
    p("    through the completion rate.")
    p("  - With 4 robots, PP and CBS ran the identical frozen plan, so their")
    p("    difference measures execution noise alone.")
    p(f"  - The +/-{args.margin:g} s equivalence margin was set after the data")
    p("    were seen; report it as such.")
    p("  - Largest absolute residuals (s): " + ", ".join(
        f"{t} {e:+.1f}" for e, t in big[:4]))
    p("  - Residuals are not normal (one long run per cell drives it), so")
    p("    read the Friedman tests alongside the F tests.")
    p("  - total distance is integrated from camera poses and is dominated")
    p("    by pose noise: every method drives the same number of hops.")

    export_excel(rows, aov, per, cells, planned, comp, args.margin)

    text = "\n".join(L) + "\n"
    (DOE / "DOE_ANALYSIS.txt").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
