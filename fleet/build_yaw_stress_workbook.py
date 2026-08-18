"""Builds a findings workbook (raw data + formula-driven summary/pivot-style
tables + per-robot sign-test table + drift table + native Excel charts +
a permanent written Notes sheet) from a fleet_yaw_stress_test.py output
directory -- so the findings live in one durable file instead of a
markdown doc or an artifact (the user has lost both of those once each).

Usage:
    python3 build_yaw_stress_workbook.py <run_dir> [output.xlsx]

Example:
    python3 build_yaw_stress_workbook.py fleet_yaw_stress_20260818_173337 \\
        YAW_STRESS_TEST_FINDINGS.xlsx

Requires: pandas, openpyxl (pip install openpyxl if missing).
"""
import glob
import re
import sys

import pandas as pd
from openpyxl import Workbook
from openpyxl.chart import BarChart, LineChart, Reference
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.table import Table, TableStyleInfo

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
RUN_DIR = sys.argv[1].rstrip("/\\")
OUT_PATH = sys.argv[2] if len(sys.argv) > 2 else "YAW_STRESS_TEST_FINDINGS.xlsx"

MODES = ["encoder", "camera_assist", "camera_only", "encoder_drift"]

FNAME_RE = re.compile(r"^(Alvik\d+)_stage(\d)_(\w+)\.csv$")

# ---------------------------------------------------------------- load data

frames = []
robots_seen: dict[str, int] = {}  # robot -> lowest stage index seen, for sort order
latin: dict[str, list[str]] = {}  # robot -> [stage1_mode, stage2_mode, stage3_mode]
for path in sorted(glob.glob(f"{RUN_DIR}/*.csv")):
    fname = path.split("/")[-1].split("\\")[-1]
    m = FNAME_RE.match(fname)
    if not m:
        continue
    robot, stage, mode = m.group(1), int(m.group(2)), m.group(3)
    robots_seen.setdefault(robot, stage)
    if mode != "encoder_drift":
        latin.setdefault(robot, []).append(mode)
    df = pd.read_csv(path)
    df.insert(0, "robot", robot)
    df.insert(1, "stage", stage)
    df["abs_error"] = df["final_error"].abs()
    df["rotation_index"] = range(1, len(df) + 1)
    frames.append(df)

if not frames:
    print(f"No matching CSVs found in {RUN_DIR!r} (expected "
          f"<Robot>_stage<N>_<mode>.csv) -- nothing to build.")
    sys.exit(1)

# ROBOTS: derived from whichever robots actually have data, sorted by
# Alvik number -- NOT hardcoded to 6, so this also works for a smaller
# test (e.g. a 2-robot dry run) without editing the script.
ROBOTS = sorted(robots_seen, key=lambda r: int(re.search(r"\d+", r).group()))

raw = pd.concat(frames, ignore_index=True)
# abs_error_sq: lets summary_by_mode compute a filtered STDEV with plain
# SUMIFS/COUNTIFS (Var = E[x^2] - E[x]^2) instead of a legacy CSE array
# formula (STDEV(IF(...))), which only auto-calculates in Excel 365/2021+
# dynamic-array mode and silently misbehaves in older Excel without
# Ctrl+Shift+Enter -- this keeps the workbook correct in any Excel version.
raw["abs_error_sq"] = raw["abs_error"] ** 2
raw = raw[["robot", "stage", "mode", "rotation_index", "leg_label",
           "target_heading", "start_yaw", "commanded_rel_deg", "final_yaw",
           "final_error", "abs_error", "abs_error_sq", "corrected", "elapsed_sec"]]
print(f"Loaded {len(raw)} rotations across {raw['robot'].nunique()} robots, "
      f"{raw['mode'].nunique()} modes")

# ---------------------------------------------------------------- styling helpers

HEADER_FILL = PatternFill("solid", fgColor="1F4E78")
HEADER_FONT = Font(color="FFFFFF", bold=True, size=11)
TITLE_FONT = Font(bold=True, size=16, color="1F4E78")
SUBTITLE_FONT = Font(italic=True, size=10, color="595959")
SECTION_FONT = Font(bold=True, size=13, color="1F4E78")
BODY_FONT = Font(size=10.5)
THIN = Side(style="thin", color="D9D9D9")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
GOOD_FILL = PatternFill("solid", fgColor="E2EFDA")
BAD_FILL = PatternFill("solid", fgColor="FCE4E4")


def style_header_row(ws, row, ncols, start_col=1):
    for c in range(start_col, start_col + ncols):
        cell = ws.cell(row=row, column=c)
        cell.fill = HEADER_FILL
        cell.font = HEADER_FONT
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        cell.border = BORDER


def autosize(ws, widths):
    for i, w in enumerate(widths, start=1):
        ws.column_dimensions[get_column_letter(i)].width = w


wb = Workbook()

# ================================================================== Notes

ws = wb.active
ws.title = "Notes"
ws["A1"] = "Yaw-Source Stress Test — Findings"
ws["A1"].font = TITLE_FONT
ws["A2"] = "6-robot simultaneous run, counterbalanced 6x3 Latin square, 2026-08-18"
ws["A2"].font = SUBTITLE_FONT
ws.merge_cells("A1:H1")
ws.merge_cells("A2:H2")

notes = [
    ("", ""),
    ("Goal", ""),
    ("", "Compare three ways of sizing a robot's ROTATE_REL turn command — encoder (onboard odometry only), "
         "camera_assist (odometry-sized, then one corrective ROTATE_REL from vision if still off-tolerance), "
         "and camera_only (vision-sized every time) — under real 6-robot simultaneous camera/rosbridge "
         "contention, not just one robot alone on a bench. A 4th stage, encoder_drift, runs last with NO "
         "vision correction at all, to measure how badly raw onboard-encoder yaw drifts over a long "
         "uncorrected run."),
    ("", ""),
    ("Method", ""),
    ("", "128 there-and-back ROTATE_REL rotations per stage per robot (8 repeat angles x 8 magnitudes x "
         "there+back). Every robot runs all 3 comparable modes, but in a DIFFERENT order per robot (Latin "
         "square below) so battery/motor-heat/run-order effects can't be confused with which mode is better. "
         "All 6 robots are gated to start each stage together. encoder_drift runs last for everyone, with no "
         "reset and no resync between rotations, so error can accumulate freely."),
    ("", ""),
    ("Headline findings", ""),
    ("1.", "camera_assist is more accurate than encoder on EVERY one of the 6 robots (6/6 sign test) — "
           "mean error ~1.0-1.3° vs ~1.8-3.4° — at a real cost of ~40-50% longer per rotation "
           "(~1.3s -> ~1.8-1.9s), from the added corrective ROTATE_REL firing on 60-100% of rotations."),
    ("2.", "encoder is more accurate than camera_only on EVERY robot (6/6) — sizing the turn from vision "
           "alone is measurably WORSE than plain onboard odometry in this setup, not just \"no better.\""),
    ("3.", "Raw onboard-encoder drift (encoder_drift stage) is real and large without correction: every "
           "robot's error grows over the 128-rotation run (positive slope), from a first-10 mean of "
           "~1.9-2.7° up to a last-10 mean of ~9.4-21.7°. This is the evidence that justifies paying "
           "camera_assist's extra time cost — encoder alone is not safe to trust over a long, uncorrected "
           "route."),
    ("4.", "Drift severity varies a lot by robot: Alvik3 and Alvik5 drifted fastest (slope +0.05°/rotation, "
           "~5° per 100 rotations); Alvik4 and Alvik6 barely drifted at all (slope ~0.00-0.003°/rotation). "
           "Worth investigating whether this tracks wheel/encoder calibration differences between units."),
    ("5.", "publish_enqueue (the rosbridge-facing stage of the vision pipeline) stayed flat throughout a "
           "long run while the LOCAL read/apriltag detection stages slowed down over time (from CPU "
           "contention, not the Linux-laptop network hop) — see the Vision Pipeline Notes below."),
    ("", ""),
    ("Decision", ""),
    ("", "turn_to_heading_rotate_rel() (fleet/camera_grid_navigate.py) already defaults to camera_assist "
         "behavior, based on an earlier single-robot benchmark. This run — the first multi-robot, "
         "counterbalanced test — confirms that decision holds up under real fleet-wide contention. No "
         "change needed to the current default."),
    ("", ""),
    ("Vision pipeline notes (context, not from this specific run)", ""),
    ("", "Best known apriltag_localize.py config: Windows capture bridge -> WSL2 detection, JPEG quality 75, "
         "16 detector threads, calibration frozen after 45 observations (~50Hz single-robot bench result, "
         "2026-07-27/28). Under real 6-robot sustained load, per-robot pose rate settles around 20-27Hz, "
         "which is still comfortably above the ~2Hz threshold where vision would start going stale for "
         "turn-sizing purposes. A live 30-50% CPU utilization / 4000MHz reading during one run ruled out "
         "thermal throttling (that CPU boosts well above 4GHz) and pointed to scheduling contention from "
         "running camera_bridge_windows.py + WSL2 + 6 robots' full ROS traffic together — a condition the "
         "original single-robot thread-count benchmark never tested."),
    ("", ""),
    ("Sheets in this workbook", ""),
    ("raw_data", "Every rotation from all 24 CSVs (6 robots x 4 stages), combined into one Excel Table — "
                 "select any cell inside it and use Insert > PivotTable to build your own views."),
    ("summary_by_mode", "Per-robot and pooled accuracy/timing stats per mode, formula-driven from raw_data "
                        "(AVERAGEIFS/COUNTIFS — recalculates if raw_data ever changes), plus bar charts."),
    ("sign_test", "Per-robot mode ranking and the 6/6 sign-test table (the real fleet-level statistic — "
                  "see Notes above for why pooling 768 rotations as independent would overstate confidence)."),
    ("drift", "encoder_drift trend per robot: slope, first-10 vs last-10 mean error, and a line chart."),
]
r = 4
for label, text in notes:
    if label and not text:
        cell = ws.cell(row=r, column=1, value=label)
        cell.font = SECTION_FONT
    elif label in ("1.", "2.", "3.", "4.", "5."):
        ws.cell(row=r, column=1, value=label).font = Font(bold=True, size=10.5)
        c2 = ws.cell(row=r, column=2, value=text)
        c2.font = BODY_FONT
        c2.alignment = Alignment(wrap_text=True, vertical="top")
        ws.merge_cells(start_row=r, start_column=2, end_row=r, end_column=8)
        ws.row_dimensions[r].height = 60
    elif label:
        ws.cell(row=r, column=1, value=label).font = Font(bold=True, size=10.5)
        c2 = ws.cell(row=r, column=2, value=text)
        c2.font = BODY_FONT
        c2.alignment = Alignment(wrap_text=True, vertical="top")
        ws.merge_cells(start_row=r, start_column=2, end_row=r, end_column=8)
        ws.row_dimensions[r].height = 75
    r += 1

autosize(ws, [14, 14, 10, 10, 10, 10, 10, 10])

# Latin square table -- built from the ACTUAL mode order found in each
# robot's own CSV filenames (the `latin` dict populated during data
# loading above), not a hardcoded assignment, so this stays correct for
# any robot count/subset and for a future run using a different square.
r += 2
ws.cell(row=r, column=1, value=f"Per-robot mode order ({len(ROBOTS)}x3 Latin square, from this run's actual data)").font = SECTION_FONT
r += 1
latin_header = ["Robot", "Stage 1", "Stage 2", "Stage 3", "Stage 4"]
for i, h in enumerate(latin_header, start=1):
    ws.cell(row=r, column=i, value=h)
style_header_row(ws, r, len(latin_header))
for robot in ROBOTS:
    r += 1
    seq = latin.get(robot, ["?", "?", "?"])
    row = [robot, *seq, "encoder_drift"]
    for i, v in enumerate(row, start=1):
        cell = ws.cell(row=r, column=i, value=v)
        cell.border = BORDER
        cell.font = BODY_FONT

ws.sheet_view.showGridLines = False

# ================================================================== raw_data

ws = wb.create_sheet("raw_data")
cols = list(raw.columns)
for i, c in enumerate(cols, start=1):
    ws.cell(row=1, column=i, value=c)
style_header_row(ws, 1, len(cols))
for ridx, row in enumerate(raw.itertuples(index=False), start=2):
    for cidx, val in enumerate(row, start=1):
        ws.cell(row=ridx, column=cidx, value=val)

last_col = get_column_letter(len(cols))
last_row = len(raw) + 1
tbl = Table(displayName="RawData", ref=f"A1:{last_col}{last_row}")
tbl.tableStyleInfo = TableStyleInfo(
    name="TableStyleMedium2", showRowStripes=True, showColumnStripes=False)
ws.add_table(tbl)
DEFAULT_COL_WIDTH = 12
autosize(ws, [9, 7, 14, 15, 30, 13, 11, 15, 11, 12, 11, 13, 10, 12][:len(cols)]
         + [DEFAULT_COL_WIDTH] * max(0, len(cols) - 14))
ws.freeze_panes = "A2"


def col_range(col_name: str) -> str:
    """A1-style raw_data!$<col>$2:$<col>$<last_row> reference for a named
    raw_data column -- derived from cols.index() instead of a hardcoded
    letter, so this stays correct if raw_data's columns are ever
    reordered/added (bit us once already with abs_error_sq -- see its
    comment above)."""
    letter = get_column_letter(cols.index(col_name) + 1)
    return f"raw_data!${letter}$2:${letter}${last_row}"

# ================================================================== summary_by_mode

ws = wb.create_sheet("summary_by_mode")
ws["A1"] = "Per-robot, per-mode statistics"
ws["A1"].font = SECTION_FONT
ws.merge_cells("A1:K1")
ws["A2"] = "Formula-driven from raw_data (AVERAGEIFS / COUNTIFS / MINIFS / MAXIFS / SUMIFS-based stdev) — will recalculate if raw_data changes."
ws["A2"].font = SUBTITLE_FONT
ws.merge_cells("A2:K2")

headers = ["Robot", "Mode", "n", "mean |err| (deg)", "min", "max", "stdev",
           "mean elapsed (s)", "min elapsed", "max elapsed", "corrected %"]
hr = 4
for i, h in enumerate(headers, start=1):
    ws.cell(row=hr, column=i, value=h)
style_header_row(ws, hr, len(headers))

robot_range = col_range("robot")
mode_range = col_range("mode")
abs_err_range = col_range("abs_error")
abs_err_sq_range = col_range("abs_error_sq")
elapsed_range = col_range("elapsed_sec")
corrected_range = col_range("corrected")

# MINIFS/MAXIFS need the "_xlfn." internal-namespace prefix in the raw
# formula text openpyxl writes -- they're Excel-2016+ functions, and the
# OOXML spec still requires that prefix in the file's XML for them to
# resolve (Excel DISPLAYS them back as plain MINIFS/MAXIFS once opened,
# the prefix is invisible in the UI). openpyxl does not add this
# automatically. Confirmed on hardware 2026-08-21: without it, Excel
# either shows #NAME? (pooled table, direct formula) or silently
# autocorrects to @MINIFS (implicit intersection) and returns blank
# (per-robot table, wrapped in IFERROR) -- both are this exact bug, not
# two different problems. AVERAGEIFS/COUNTIFS/SUMIFS are Excel-2007
# functions and do NOT need this prefix.
XLFN_MINIFS = "_xlfn.MINIFS"
XLFN_MAXIFS = "_xlfn.MAXIFS"

row = hr + 1
for robot in ROBOTS:
    for mode in MODES:
        crit = f'{robot_range},"{robot}",{mode_range},"{mode}"'
        n_cell = f"C{row}"
        ws.cell(row=row, column=1, value=robot)
        ws.cell(row=row, column=2, value=mode)
        ws.cell(row=row, column=3, value=f'=COUNTIFS({crit})')
        ws.cell(row=row, column=4, value=f'=IFERROR(AVERAGEIFS({abs_err_range},{crit}),"")')
        ws.cell(row=row, column=5, value=f'=IFERROR({XLFN_MINIFS}({abs_err_range},{crit}),"")')
        ws.cell(row=row, column=6, value=f'=IFERROR({XLFN_MAXIFS}({abs_err_range},{crit}),"")')
        # Sample stdev via SUMIFS (Var = (Sum(x^2) - Sum(x)^2/n) / (n-1)) --
        # NOT STDEV(IF(...)), which needs Ctrl+Shift+Enter / dynamic-array
        # Excel to calculate correctly. Needs n>=2, guarded by IFERROR.
        sumx = f'SUMIFS({abs_err_range},{crit})'
        sumx2 = f'SUMIFS({abs_err_sq_range},{crit})'
        ws.cell(row=row, column=7, value=(
            f'=IFERROR(SQRT(({sumx2}-({sumx})^2/{n_cell})/({n_cell}-1)),"")'))
        ws.cell(row=row, column=8, value=f'=IFERROR(AVERAGEIFS({elapsed_range},{crit}),"")')
        ws.cell(row=row, column=9, value=f'=IFERROR({XLFN_MINIFS}({elapsed_range},{crit}),"")')
        ws.cell(row=row, column=10, value=f'=IFERROR({XLFN_MAXIFS}({elapsed_range},{crit}),"")')
        ws.cell(row=row, column=11, value=(
            f'=IFERROR(COUNTIFS({crit},{corrected_range},TRUE)/{n_cell},"")'))
        for c in range(1, len(headers) + 1):
            cell = ws.cell(row=row, column=c)
            cell.border = BORDER
            cell.font = BODY_FONT
            if c in (4, 5, 6, 7, 8, 9, 10):
                cell.number_format = "0.00"
            if c == 11:
                cell.number_format = "0%"
        row += 1

autosize(ws, [10, 15, 6, 16, 8, 8, 8, 16, 11, 11, 12])

# Pooled-by-mode table (descriptive)
pooled_header_row = row + 2
ws.cell(row=pooled_header_row - 1, column=1,
        value="Pooled across all robots, per mode (descriptive — see sign_test sheet for the real statistic)").font = SECTION_FONT
pheaders = ["Mode", "n", "mean |err| (deg)", "min", "max", "stdev", "mean elapsed (s)"]
for i, h in enumerate(pheaders, start=1):
    ws.cell(row=pooled_header_row, column=i, value=h)
style_header_row(ws, pooled_header_row, len(pheaders))
prow = pooled_header_row + 1
for mode in MODES:
    crit = f'{mode_range},"{mode}"'
    n_cell = f"B{prow}"
    ws.cell(row=prow, column=1, value=mode)
    ws.cell(row=prow, column=2, value=f'=COUNTIF({mode_range},"{mode}")')
    ws.cell(row=prow, column=3, value=f'=AVERAGEIF({mode_range},"{mode}",{abs_err_range})')
    ws.cell(row=prow, column=4, value=f'={XLFN_MINIFS}({abs_err_range},{crit})')
    ws.cell(row=prow, column=5, value=f'={XLFN_MAXIFS}({abs_err_range},{crit})')
    # Same SUMIFS-based sample stdev as the per-robot table above -- no
    # legacy array formula, works in every Excel version.
    sumx = f'SUMIFS({abs_err_range},{crit})'
    sumx2 = f'SUMIFS({abs_err_sq_range},{crit})'
    ws.cell(row=prow, column=6, value=(
        f'=SQRT(({sumx2}-({sumx})^2/{n_cell})/({n_cell}-1))'))
    ws.cell(row=prow, column=7, value=f'=AVERAGEIF({mode_range},"{mode}",{elapsed_range})')
    for c in range(1, len(pheaders) + 1):
        cell = ws.cell(row=prow, column=c)
        cell.border = BORDER
        cell.font = BODY_FONT
        if c in (3, 4, 5, 6, 7):
            cell.number_format = "0.00"
    prow += 1

# Bar chart: mean error by mode (pooled)
chart1 = BarChart()
chart1.title = "Mean |final_error| by mode (pooled, descriptive)"
chart1.y_axis.title = "degrees"
chart1.x_axis.title = "mode"
chart1.style = 10
data = Reference(ws, min_col=3, min_row=pooled_header_row, max_row=prow - 1)
cats = Reference(ws, min_col=1, min_row=pooled_header_row + 1, max_row=prow - 1)
chart1.add_data(data, titles_from_data=True)
chart1.set_categories(cats)
chart1.width, chart1.height = 14, 9
ws.add_chart(chart1, f"A{prow + 2}")

chart2 = BarChart()
chart2.title = "Mean elapsed time per rotation by mode (pooled)"
chart2.y_axis.title = "seconds"
chart2.x_axis.title = "mode"
chart2.style = 11
data2 = Reference(ws, min_col=7, min_row=pooled_header_row, max_row=prow - 1)
chart2.add_data(data2, titles_from_data=True)
chart2.set_categories(cats)
chart2.width, chart2.height = 14, 9
ws.add_chart(chart2, f"I{prow + 2}")

ws.sheet_view.showGridLines = False

# ================================================================== sign_test

ws = wb.create_sheet("sign_test")
ws["A1"] = "Statistics: per-robot ranking + sign test"
ws["A1"].font = SECTION_FONT
ws.merge_cells("A1:E1")
ws["A2"] = ("The fleet-level test statistic. Each robot = ONE unit of replication (n=6), not 768 "
            "independent rotations. Ranks each robot's 3 modes by its OWN mean |final_error|, then a "
            "sign test counts how many of 6 robots favor each mode in a pairwise comparison.")
ws["A2"].font = SUBTITLE_FONT
ws["A2"].alignment = Alignment(wrap_text=True)
ws.merge_cells("A2:E2")
ws.row_dimensions[2].height = 30

rank_stats = (raw[raw["mode"].isin(["encoder", "camera_assist", "camera_only"])]
              .groupby(["robot", "mode"])["abs_error"].mean().reset_index())

rr = 5
headers = ["Robot", "1st (lowest error)", "2nd", "3rd"]
for i, h in enumerate(headers, start=1):
    ws.cell(row=rr, column=i, value=h)
style_header_row(ws, rr, len(headers))
sign_wins = {("encoder", "camera_assist"): [0, 0], ("encoder", "camera_only"): [0, 0],
             ("camera_assist", "camera_only"): [0, 0]}
rr += 1
for robot in ROBOTS:
    sub = rank_stats[rank_stats["robot"] == robot].set_values = None
    sub = rank_stats[rank_stats["robot"] == robot].sort_values("abs_error")
    means = dict(zip(sub["mode"], sub["abs_error"]))
    ranked = sub["mode"].tolist()
    ws.cell(row=rr, column=1, value=robot)
    for i, mode in enumerate(ranked, start=2):
        ws.cell(row=rr, column=i, value=f"{mode} ({means[mode]:.2f}°)")
    for (a, b), wins in sign_wins.items():
        if means[a] < means[b]:
            wins[0] += 1
        elif means[b] < means[a]:
            wins[1] += 1
    for c in range(1, 5):
        cell = ws.cell(row=rr, column=c)
        cell.border = BORDER
        cell.font = BODY_FONT
    rr += 1

autosize(ws, [10, 24, 24, 24])

rr += 2
ws.cell(row=rr, column=1, value="Sign test across robots, per mode pair").font = SECTION_FONT
rr += 1
sheaders = ["Comparison", "X more accurate", "Y more accurate", "Result"]
for i, h in enumerate(sheaders, start=1):
    ws.cell(row=rr, column=i, value=h)
style_header_row(ws, rr, len(sheaders))
rr += 1
for (a, b), (wa, wb_) in sign_wins.items():
    n = wa + wb_
    if wa == n:
        result = f"{a} more accurate on EVERY robot ({wa}/{n})"
    elif wb_ == n:
        result = f"{b} more accurate on EVERY robot ({wb_}/{n})"
    else:
        result = f"mixed ({wa}/{n} favor {a}, {wb_}/{n} favor {b}) — not robot-independent"
    ws.cell(row=rr, column=1, value=f"{a} vs {b}")
    ws.cell(row=rr, column=2, value=wa)
    ws.cell(row=rr, column=3, value=wb_)
    ws.cell(row=rr, column=4, value=result)
    fill = GOOD_FILL if n in (wa, wb_) else BAD_FILL
    for c in range(1, 5):
        cell = ws.cell(row=rr, column=c)
        cell.border = BORDER
        cell.font = BODY_FONT
        cell.fill = fill
    rr += 1

ws.sheet_view.showGridLines = False

# ================================================================== drift

ws = wb.create_sheet("drift")
ws["A1"] = "encoder_drift: real accumulated drift over the run"
ws["A1"].font = SECTION_FONT
ws.merge_cells("A1:E1")
ws["A2"] = ("Stage 4, NO reset / NO per-rotation resync — every other stage resyncs onboard yaw to vision "
            "after each rotation, so this is the only stage where raw drift is actually visible. Slope = "
            "linear trend of |final_error| vs. rotation index (least squares).")
ws["A2"].font = SUBTITLE_FONT
ws["A2"].alignment = Alignment(wrap_text=True)
ws.merge_cells("A2:E2")
ws.row_dimensions[2].height = 30

drift_df = raw[raw["mode"] == "encoder_drift"].sort_values(["robot", "rotation_index"])
dr = 5
dheaders = ["Robot", "n", "slope (deg/rotation)", "first-10 mean err", "last-10 mean err"]
for i, h in enumerate(dheaders, start=1):
    ws.cell(row=dr, column=i, value=h)
style_header_row(ws, dr, len(dheaders))
dr += 1
slope_start = dr
for robot in ROBOTS:
    sub = drift_df[drift_df["robot"] == robot]
    n = len(sub)
    x = sub["rotation_index"].to_numpy(dtype=float)
    y = sub["abs_error"].to_numpy(dtype=float)
    xbar, ybar = x.mean(), y.mean()
    denom = ((x - xbar) ** 2).sum()
    slope = ((x - xbar) * (y - ybar)).sum() / denom if denom else 0.0
    first10 = sub["abs_error"].iloc[:10].mean()
    last10 = sub["abs_error"].iloc[-10:].mean()
    ws.cell(row=dr, column=1, value=robot)
    ws.cell(row=dr, column=2, value=n)
    ws.cell(row=dr, column=3, value=round(slope, 4))
    ws.cell(row=dr, column=4, value=round(first10, 2))
    ws.cell(row=dr, column=5, value=round(last10, 2))
    for c in range(1, 6):
        cell = ws.cell(row=dr, column=c)
        cell.border = BORDER
        cell.font = BODY_FONT
    dr += 1
slope_end = dr - 1

autosize(ws, [10, 8, 20, 18, 18])

# Line chart: per-robot abs_error trend across the 128 drift rotations
dr += 2
chart_hdr_row = dr
ws.cell(row=dr, column=1, value="Per-robot |final_error| across the 128 encoder_drift rotations").font = SECTION_FONT
dr += 1
table_top = dr
ws.cell(row=dr, column=1, value="rotation_index")
for i, robot in enumerate(ROBOTS, start=2):
    ws.cell(row=dr, column=i, value=robot)
style_header_row(ws, dr, len(ROBOTS) + 1)
dr += 1
pivot_wide = drift_df.pivot(index="rotation_index", columns="robot", values="abs_error")
for idx, rot in enumerate(sorted(pivot_wide.index), start=0):
    row_i = table_top + 1 + idx
    ws.cell(row=row_i, column=1, value=int(rot))
    for ci, robot in enumerate(ROBOTS, start=2):
        val = pivot_wide.loc[rot, robot] if robot in pivot_wide.columns and rot in pivot_wide.index else None
        ws.cell(row=row_i, column=ci, value=None if pd.isna(val) else round(float(val), 2))
table_bottom = table_top + len(pivot_wide)

line = LineChart()
line.title = "encoder_drift: |final_error| vs. rotation (per robot)"
line.y_axis.title = "abs(final_error), deg"
line.x_axis.title = "rotation index"
line.style = 12
data = Reference(ws, min_col=2, max_col=1 + len(ROBOTS), min_row=table_top, max_row=table_bottom)
cats = Reference(ws, min_col=1, min_row=table_top + 1, max_row=table_bottom)
line.add_data(data, titles_from_data=True)
line.set_categories(cats)
line.width, line.height = 26, 14
ws.add_chart(line, f"H{chart_hdr_row}")

ws.sheet_view.showGridLines = False

wb.save(OUT_PATH)
print(f"Saved {OUT_PATH}")
