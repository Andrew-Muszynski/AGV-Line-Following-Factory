#!/usr/bin/env python3
"""One Excel workbook for the traffic-planning DOE.

    python3 tools/build_doe_dataset.py
    python3 tools/analyze_doe.py
    python3 tools/plot_doe.py
    python3 tools/build_doe_workbook.py

Writes data/doe/DOE_traffic_planning_results.xlsx with tabs:
Summary, Runs, Cell summary, ANOVA makespan, Pairwise makespan,
Secondary tests, Secondary means, Robot finish, Attempts, Figures.
"""
import csv
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

from openpyxl import Workbook
from openpyxl.drawing.image import Image
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_doe as A  # noqa: E402

DOE = A.DOE
OUT = DOE / "DOE_traffic_planning_results.xlsx"
HEAD = PatternFill("solid", fgColor="DCE6F2")
SIG = PatternFill("solid", fgColor="E2F0D9")
BOLD = Font(bold=True)


def read_csv(name):
    with open(DOE / name, encoding="utf-8-sig") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def typed(v):
    try:
        f = float(v)
        return int(f) if f.is_integer() and "." not in v else f
    except (TypeError, ValueError):
        return v


def sheet(wb, title, header, rows, widths=None, p_cols=()):
    ws = wb.create_sheet(title)
    ws.append(header)
    for c in ws[1]:
        c.font, c.fill = BOLD, HEAD
        c.alignment = Alignment(wrap_text=True, vertical="top")
    for r in rows:
        ws.append([typed(v) if isinstance(v, str) else v for v in r])
    ws.freeze_panes = "A2"
    for i, h in enumerate(header, 1):
        w = (widths or {}).get(h, max(10, min(28, len(str(h)) + 2)))
        ws.column_dimensions[get_column_letter(i)].width = w
    for col in p_cols:                       # shade p < 0.05
        j = header.index(col) + 1
        for row in ws.iter_rows(min_row=2, min_col=j, max_col=j):
            v = row[0].value
            if isinstance(v, (int, float)) and v < 0.05:
                row[0].fill = SIG
    return ws


def secondary(rows):
    """Per response: two-way p (robots, method, interaction) and, per robot
    count, blocked one-way p and Friedman p, plus the method means."""
    tests, means = [], []
    for key, name in [("actual_makespan_sec", "actual makespan, s")] \
            + A.SECONDARY:
        a = A.two_way(rows, key)[0]
        line = [name, key, a["robots"][4], a["method"][4],
                a["robots:method"][4]]
        for R in A.ROBOTS:
            r = A.one_way_blocked(rows, R, key)
            try:
                _, fp = A.friedman(r)
            except ValueError:                # all ties
                fp = None
            line += [r["p"], fp]
            means.append([name, R] + [round(r["means"][m], 3)
                                      for m in A.METHODS])
        tests.append([v if isinstance(v, str) or v is None
                      else float(f"{v:.3g}") for v in line])
    return tests, means


def summary(wb, rows, tests):
    ws = wb.active
    ws.title = "Summary"
    ws.column_dimensions["A"].width = 125
    COL = {"robots": 2, "method": 3, "inter": 4, "4F": 5, "4Fr": 6,
           "6F": 7, "6Fr": 8}
    T = {t[1]: t for t in tests}
    P = lambda key, which: T[key][COL[which]]
    per = {R: A.one_way_blocked(rows, R, "actual_makespan_sec")
           for R in A.ROBOTS}
    aov = A.two_way(rows, "actual_makespan_sec")[0]
    m4, m6 = per[4]["means"], per[6]["means"]
    mean = lambda R: st.mean(float(r["actual_makespan_sec"]) for r in rows
                             if r["robots"] == R)
    tk6 = {(a, b): (d, p) for a, b, d, _, _, p in A.tukey(per[6])}
    guard = {R: st.mean(int(r["guard_interventions"]) for r in rows
                        if r["robots"] == R) for R in A.ROBOTS}
    L = lambda m: A.LABEL[m]
    f = lambda p: f"{p:.3g}"

    text = [
        ("TRAFFIC-PLANNING DOE: SUMMARY AND CONCLUSIONS", "title"),
        ("4 methods (A*, WAIT, PP, CBS) x 2 fleet sizes (4, 6 robots), "
         "5 rounds as blocks, 40 completed runs, 12 jobs each. Every run is "
         "kept, including DOE_4r_pp_2 (343.1 s).", None),
        ("", None),
        ("HOW TO READ THE P-VALUES", "head"),
        ("- Each p-value answers one question: could a difference this big "
         "have come from run-to-run noise alone? Below 0.05 = unlikely to be "
         "noise.", None),
        ("- The robot-count p-values are tiny because 4 and 6 robots are "
         "different jobs: fleet size moves everything. That is expected and "
         "is not the question of the DOE.", None),
        ("- The question is method: look at the 'method' column within each "
         "robot count (tabs Secondary tests and Pairwise makespan).", None),
        ("- Friedman = rank-based version of the same test. It ignores how "
         "far a run is off, so one long run (343 s) cannot dominate it.",
         None),
        ("", None),
        ("ANSWERS", "head"),
        ("1. Are the methods' makespans different?", "q"),
        (f"- With 6 robots: YES (method p = {f(per[6]['p'])}, Friedman p = "
         f"{f(P('actual_makespan_sec', '6Fr'))}). WAIT is slowest "
         f"({m6['wait']:.1f} s), significantly slower than A* "
         f"({m6['astar']:.1f} s, p = {f(tk6[('astar', 'wait')][1])}), "
         f"PP ({m6['pp']:.1f} s, p = {f(tk6[('wait', 'pp')][1])}) and CBS "
         f"({m6['cbs']:.1f} s, p = {f(tk6[('wait', 'cbs')][1])}). PP is "
         f"fastest, but its lead over A* (p = {f(tk6[('astar', 'pp')][1])})"
         f" and CBS (p = {f(tk6[('pp', 'cbs')][1])}) is not significant. "
         "A* and CBS are statistically equivalent (within 10 s).", None),
        (f"- With 4 robots: NOT on average (method p = {f(per[4]['p'])}). "
         f"Means: A* {m4['astar']:.1f}, WAIT {m4['wait']:.1f}, PP "
         f"{m4['pp']:.1f}, CBS {m4['cbs']:.1f} s. But A* was the fastest "
         "method in all 5 rounds, so the rank-based test IS significant "
         f"(Friedman p = {f(P('actual_makespan_sec', '4Fr'))}). A* is the only "
         "plan with no planned waits; with 4 robots the supervisor absorbs "
         "its 3 conflicts at least as cheaply as the planners' waits "
         "cost (a likely explanation, not tested here).", None),
        (f"- Both fleets together: method alone p = {f(aov['method'][4])}, "
         f"method x robots p = {f(aov['robots:method'][4])}. The ranking "
         "changes with fleet size (PP last with 4, first with 6), a "
         "suggestive interaction but not significant at 0.05.", None),
        (f"- Fleet size: 6 robots finish in {mean(6):.1f} s on average vs "
         f"{mean(4):.1f} s for 4 (p = {f(aov['robots'][4])}).", None),
        ("2. Prediction error (actual - planned makespan)", "q"),
        ("- Every method runs longer than planned: +16 to +32 s (6-12%). "
         f"Overall, method does not change it (p = "
         f"{f(P('makespan_error_sec', 'method'))}) and neither does fleet size "
         f"(p = {f(P('makespan_error_sec', 'robots'))}). The timing model "
         "underestimates by a roughly constant amount: driving and turning "
         "take longer than modelled, the same for every planner.", None),
        (f"- With 6 robots PP predicts best (+17.8 s vs +22 to +25 s, p = "
         f"{f(P('makespan_error_sec', '6F'))}, F test only: Friedman p = "
         f"{f(P('makespan_error_sec', '6Fr'))}); with 4 robots A* predicts best "
         f"(+15.9 s, Friedman p = {f(P('makespan_error_sec', '4Fr'))}). These "
         "mirror the makespan results, because the planned makespans are "
         "nearly identical.", None),
        ("3. Waiting (runtime traffic holds by the supervisor)", "q"),
        (f"- Traffic-stop episodes: with 6 robots PP has the most "
         f"(17.2 vs 10.6-12.6, p = {f(P('traffic_stop_episodes', '6F'))}) "
         "but they are SHORT: mean stop 24.6 s vs 36-41 s "
         f"(p = {f(P('mean_traffic_stop_duration_sec', '6F'))}). PP holds "
         "robots more often but briefly. With 4 robots there is no "
         f"difference (p = {f(P('traffic_stop_episodes', '4F'))}).", None),
        (f"- Total robot-seconds waiting does not differ by method (6r p = "
         f"{f(P('fleet_traffic_delay_robot_sec', '6F'))}, 4r p = "
         f"{f(P('fleet_traffic_delay_robot_sec', '4F'))}). All methods wait "
         "about the same total; what differs is WHO waits.", None),
        (f"- Who waits: PP narrows the gap between first and last robot "
         f"home (6r spread 62 s vs 79-85 s, p = "
         f"{f(P('robot_completion_spread_sec', '6F'))}; interaction p = "
         f"{f(P('robot_completion_spread_sec', 'inter'))}). It delays "
         "the robots that finish early (Alvik1, Alvik2) so the robot that "
         "sets the makespan (Alvik4) gets through sooner. See tab Robot "
         "finish and Figure 3.", None),
        ("4. Collision-guard interventions (emergency stops)", "q"),
        (f"- Rare: 51 in 40 runs, 0-3 per run. Method makes no difference "
         f"(p = {f(P('guard_interventions', 'method'))}; 4r p = "
         f"{f(P('guard_interventions', '4F'))}, 6r p = "
         f"{f(P('guard_interventions', '6F'))}). Uncoordinated A*, which "
         "leaves conflicts in its plan for the supervisor, needed no more "
         "interventions than the planners.", None),
        (f"- More interventions with 4 robots ({guard[4]:.1f} per run) than "
         f"with 6 ({guard[6]:.1f}), p = {f(P('guard_interventions', 'robots'))}. "
         "Not explained yet; worth checking which robot pairs trigger them.",
         None),
        ("- No collisions in any of the 40 runs. One deadlock was detected "
         "(DOE_4r_wait_2) and cleared itself in 1.1 s.", None),
        ("5. Completion", "q"),
        ("- 40 of 44 real attempts completed (91%; false starts excluded)."
         " All 4 failures are infrastructure faults under the declared "
         "rule: two execution defects fixed during round 1 (DOE_6r_pp_1, "
         "_2), Alvik1's command-link delay (DOE_4r_cbs_2) and robot "
         "non-execution followed by deadlock (DOE_4r_pp_5). Excluding "
         "them, every method completed 100% of its attempts.",
         None),
        ("", None),
        ("CONCLUSIONS", "head"),
        ("- With this workload (12 bays, allocation frozen, each robot "
         "serving its own bays) the choice of traffic planner matters "
         "little: the largest difference between methods is 11.7 s (4.7%) "
         "with 6 robots, and none is detectable on average with 4.", None),
        ("- WAIT is the one method to avoid at 6 robots: its fixed L-shaped "
         "returns funnel every robot along the bottom row and add turns. It "
         "is consistently ~7-12 s slower.", None),
        ("- PP's advantage at 6 robots comes from execution, not the plan: "
         "its planned makespan equals A* and CBS. Its priority order keeps "
         "the critical robot moving. That planned makespan alone did not "
         "predict this is itself a finding.", None),
        ("- The joint planner (CBS) did not beat the sequential one (PP) in "
         "any response. With few conflicts there is little for joint "
         "search to win, which motivates the next experiment: more "
         "workstations / more conflicts, joint vs sequential.", None),
        ("- Execution noise is as large as the method effects: the 4-robot "
         "PP and CBS plans are IDENTICAL yet took 281-343 s. Execution "
         "faults (Alvik1's no-motion turns) dominate the 4-robot spread.",
         None),
        ("", None),
        ("CAVEATS", "head"),
        ("- DOE_4r_pp_2 (343.1 s, Alvik1 no-motion turns held it ~60 s at a "
         "workstation) is kept as declared. Replacing it with the median of "
         "its cell would make the overall method effect significant "
         "(p = 0.33 -> 0.002): report it, and lean on the Friedman tests.",
         None),
        ("- Residuals are not normal (that run); Friedman tests are given "
         "next to every F test. Variances are equal across cells "
         "(Brown-Forsythe p = 0.53).", None),
        ("- The 10 s equivalence margin was chosen after seeing the data.",
         None),
        ("- Minimum separation differs statistically with 6 robots but by "
         "under 4 mm (122-126 mm); total distance is pose noise and should not be "
         "used.", None),
    ]
    style = {"title": Font(bold=True, size=14), "head": Font(bold=True,
             size=12, color="1F4E79"), "q": Font(bold=True)}
    for i, (t, s) in enumerate(text, 1):
        c = ws.cell(row=i, column=1, value=t)
        c.alignment = Alignment(wrap_text=True, vertical="top")
        if s:
            c.font = style[s]


def main():
    rows = A.load()
    for r in rows:   # load() leaves the new count columns as text
        for k in ("guard_interventions", "deadlocks_detected",
                  "command_failures"):
            r[k] = int(r[k])
    tests, means = secondary(rows)
    wb = Workbook()
    summary(wb, rows, tests)

    h, r = read_csv("DOE_results.csv")
    extra = {x["run_id"]: x for x in rows}
    i = h.index("Run ID")
    h = h + ["Guard interventions", "Deadlocks detected", "Failed commands"]
    r = [x + [extra[x[i]]["guard_interventions"],
              extra[x[i]]["deadlocks_detected"],
              extra[x[i]]["command_failures"]] for x in r]
    sheet(wb, "Runs", h, r, widths={"Started (UTC)": 30, "Run ID": 38,
                                    "Video": 18})
    h, r = read_csv("DOE_cell_summary.csv")
    sheet(wb, "Cell summary", h, r)
    h, r = read_csv("DOE_anova_table.csv")
    sheet(wb, "ANOVA makespan", h, r, widths={"Model": 36, "Source": 15,
                                              "F": 14}, p_cols=["p-value"])
    h, r = read_csv("DOE_pairwise.csv")
    sheet(wb, "Pairwise makespan", h, r, p_cols=["Tukey p"])
    hdr = ["Response", "Column", "Robots p (2-way)", "Method p (2-way)",
           "Robots x method p (2-way)", "4 robots: method p (F)",
           "4 robots: Friedman p", "6 robots: method p (F)",
           "6 robots: Friedman p"]
    sheet(wb, "Secondary tests", hdr, tests,
          widths={"Response": 40, "Column": 30}, p_cols=hdr[2:])
    sheet(wb, "Secondary means",
          ["Response", "Robots"] + [A.LABEL[m] for m in A.METHODS], means,
          widths={"Response": 40})

    fin = defaultdict(list)
    for x in csv.DictReader(open(DOE / "doe_robots.csv", encoding="utf-8")):
        if x["completion_sec"]:
            fin[(int(x["robots"]), x["robot_id"], x["method"])].append(
                float(x["completion_sec"]))
    rf = [[R, f"Alvik{i}"] + [round(st.mean(fin[(R, f"Alvik{i}", m)]), 1)
                              for m in A.METHODS]
          for R in A.ROBOTS for i in range(1, R + 1)]
    sheet(wb, "Robot finish", ["Robots", "Robot"] +
          [f"{A.LABEL[m]} mean finish (s)" for m in A.METHODS], rf)

    with open(DOE / "doe_attempts.csv", encoding="utf-8") as f:
        att = list(csv.reader(f))
    sheet(wb, "Attempts", att[0], att[1:],
          widths={"reason": 90, "created_at_utc": 30, "run_id": 38})

    ws = wb.create_sheet("Figures")
    row = 1
    for png in sorted((DOE / "figures").glob("*.png")):
        ws.cell(row=row, column=1, value=png.stem).font = BOLD
        img = Image(str(png))
        scale = 900 / img.width
        img.width, img.height = img.width * scale, img.height * scale
        ws.add_image(img, f"A{row + 1}")
        row += int(img.height / 20) + 4
    wb.save(OUT)
    print(OUT)


if __name__ == "__main__":
    main()
