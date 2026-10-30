"""Capture clean, overlay-free frames for the camera-pose solve and the
whole-table localization accuracy check.

Opens the camera the same way camera_bridge_windows.py does (MSMF, MJPG,
1920x1080), so the testbed must NOT be running: the bridge holds the camera.

    python3 tools/grab_clean_frames.py            # walk the capture plan
    python3 tools/grab_clean_frames.py --sheet    # write the measurement workbook and exit

A preview window shows the live stream with the detected tags marked (the
marks are drawn on the preview only; saved frames are untouched) and the
placement for the current step. Keys:

    SPACE / ENTER   capture this step (--frames PNGs), then advance
    B               go back one step (to redo it)
    S               skip this step
    Q / ESC         quit

Every capture is logged to manifest.csv in the output folder.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import date, datetime
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from apriltag_detect import open_camera  # noqa: E402

CORNER_TAGS = (20, 21, 22, 23)
ROBOTS = (1, 2, 3, 4, 5, 6, 7)

# Lattice: node = 1 + col + 8*row, node 1 at (13.5, 16.75) in, 10 in pitch
# (tape-measured anchor, fleet/testbed_coordinates.json).
NODE1_X, NODE1_Y, PITCH = 13.5, 16.75, 10.0


def node_xy(node: int) -> tuple[float, float]:
    row, col = divmod(node - 1, 8)
    return NODE1_X + PITCH * col, NODE1_Y + PITCH * row


def row_nodes(row: int, skip_col: int) -> list[int]:
    return [1 + c + 8 * row for c in range(8) if c != skip_col]


# Five full-width rows, each missing a different column so all eight columns
# are covered. Robots shift two slots per row, so no robot keeps the same
# column (avoids the robot/column confound seen in the 2026-08-26 bay survey).
ROWS = [(0, 3), (2, 6), (4, 1), (6, 4), (7, 2)]


def layout(k: int) -> dict[int, int]:
    nodes = row_nodes(*ROWS[k])
    return {r: nodes[(i + 2 * k) % 7] for i, r in enumerate(ROBOTS)}


def build_plan() -> list[dict]:
    plan = [dict(label="empty", heading=None, place={},
                 note="Table empty: corner tags only, no robots")]
    for k, (row, _) in enumerate(ROWS):
        name = f"R{row}"
        plan.append(dict(label=name, heading=0, place=layout(k),
                         note=f"Row {row}: tag centres over the nodes, all facing EAST"))
        if row == 4:
            for deg in (90, 180, 270):
                plan.append(dict(label=f"{name}_r{deg}", heading=deg,
                                 place=layout(k),
                                 note=f"Same nodes, every robot turned to {deg} deg "
                                      f"CCW from east ({ {90:'NORTH',180:'WEST',270:'SOUTH'}[deg] })"))
    return plan


def write_sheet(plan: list[dict], path: Path) -> None:
    from openpyxl import Workbook
    from openpyxl.styles import Font, PatternFill

    wb = Workbook()
    ws = wb.active
    ws.title = "Node measurements"
    head = ["node", "row", "col", "expected_x_in", "expected_y_in",
            "measured_x_in", "measured_y_in", "notes"]
    ws.append(head)
    nodes = sorted({n for s in plan for n in s["place"].values()})
    for n in nodes:
        row, col = divmod(n - 1, 8)
        ws.append([n, row, col, *node_xy(n), None, None, None])
    ws.append([])
    ws.append(["corner tag", "", "", "expected_x_in", "expected_y_in",
               "measured_x_in", "measured_y_in", "notes"])
    for t, xy in zip(CORNER_TAGS, [(2.75, 2.75), (93.25, 2.75), (2.75, 93.25), (93.25, 93.25)]):
        ws.append([f"tag {t} centre", "", "", *xy, None, None,
                   "expected = code's 97 in assumption"])
    fill = PatternFill("solid", fgColor="FFF2CC")
    for r in ws.iter_rows(min_row=2):
        for c in r[5:7]:
            c.fill = fill
    for c in ws[1]:
        c.font = Font(bold=True)
    for col, w in zip("ABCDEFGH", (12, 6, 6, 14, 14, 14, 14, 36)):
        ws.column_dimensions[col].width = w

    wp = wb.create_sheet("Placement plan")
    wp.append(["step", "label", "heading_deg"] + [f"robot {r} node" for r in ROBOTS] + ["note"])
    for i, s in enumerate(plan, 1):
        wp.append([i, s["label"], s["heading"]] +
                  [s["place"].get(r) for r in ROBOTS] + [s["note"]])
    for c in wp[1]:
        c.font = Font(bold=True)
    wp.column_dimensions["K"].width = 60
    ws.freeze_panes = wp.freeze_panes = "A2"
    wb.save(path)
    print(f"wrote {path}")


def describe(step: dict) -> list[str]:
    if not step["place"]:
        return [step["note"]]
    by_node = sorted(step["place"].items(), key=lambda kv: kv[1])
    return [step["note"]] + ["   ".join(f"A{r}->{n}" for r, n in by_node)]


def draw_preview(frame, dets, step: dict, idx: int, total: int, scale: float):
    view = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    for d in dets:
        pts = (d.corners * scale).astype(int)
        cv2.polylines(view, [pts.reshape(-1, 1, 2)], True, (0, 200, 255), 2)
        c = tuple((d.center * scale).astype(int))
        cv2.putText(view, str(d.tag_id), (c[0] - 8, c[1] + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    want = set(CORNER_TAGS) | set(step["place"])
    seen = {d.tag_id for d in dets}
    missing, extra = sorted(want - seen), sorted(seen - want)
    lines = [f"Step {idx + 1}/{total}: {step['label']}"] + describe(step)
    ok = not missing and not extra
    lines.append("All expected tags seen - SPACE to capture" if ok else
                 f"missing {missing}  unexpected {extra}")
    # Text goes in a strip ABOVE the image, so it never hides the far rows.
    row_h = 24
    strip = np.zeros((row_h * len(lines) + 8, view.shape[1], 3), np.uint8)
    for i, text in enumerate(lines):
        color = (255, 255, 255) if i < len(lines) - 1 else             ((80, 220, 80) if ok else (60, 60, 255))
        cv2.putText(strip, text, (6, row_h * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1)
    view = np.vstack([strip, view])
    return view, ok, missing, extra


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=1)
    ap.add_argument("--frames", type=int, default=20, help="frames saved per step")
    ap.add_argument("--out", default=str(ROOT / "data" / "camera_checks"
                                         / f"accuracy_{date.today():%Y%m%d}"))
    ap.add_argument("--start", default=None,
                    help="label of the step to start at, e.g. R7")
    ap.add_argument("--sheet", action="store_true",
                    help="write the measurement workbook and exit")
    args = ap.parse_args()

    plan = build_plan()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sheet = out / "accuracy_measurements.xlsx"
    if args.sheet or not sheet.exists():
        write_sheet(plan, sheet)
        if args.sheet:
            return

    cap = open_camera(args.camera, "msmf")
    if cap is None:
        sys.exit(f"could not open camera {args.camera}; is the testbed still running?")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (w, h) != (1920, 1080):
        sys.exit(f"camera opened at {w}x{h}, expected 1920x1080")
    print(f"camera {args.camera} at {w}x{h}; saving to {out}")

    # Full-resolution detection, as in the analysis: at quad_decimate 2 the
    # far rows' tags are small enough to drop out intermittently.
    det = Detector(families="tag36h11", quad_decimate=1.0, nthreads=8)
    manifest = out / "manifest.csv"
    new_manifest = not manifest.exists()
    mf = open(manifest, "a", newline="")
    log = csv.writer(mf)
    if new_manifest:
        log.writerow(["time", "label", "heading_deg", "frames", "missing_tags",
                      "unexpected_tags"] + [f"robot{r}_node" for r in ROBOTS])

    labels = [p["label"] for p in plan]
    if args.start and args.start not in labels:
        sys.exit(f"--start must be one of {labels}")
    idx = labels.index(args.start) if args.start else 0
    win = "accuracy capture"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)   # resizable; drag the corner
    cv2.resizeWindow(win, 960, 640)
    print(f"Step {idx + 1}/{len(plan)}: {plan[idx]['label']}")
    for line in describe(plan[idx]):
        print(line)
    while idx < len(plan):
        step = plan[idx]
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        dets = det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        view, _, missing, extra = draw_preview(frame, dets, step, idx, len(plan), 0.5)
        cv2.imshow(win, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (ord("b"),) and idx > 0:
            idx -= 1
        elif key == ord("s"):
            idx += 1
        elif key in (32, 13):
            saved = 0
            while saved < args.frames:
                ok, f = cap.read()
                if ok:
                    cv2.imwrite(str(out / f"{step['label']}_{saved:02d}.png"), f)
                    saved += 1
            log.writerow([datetime.now().isoformat(timespec="seconds"), step["label"],
                          step["heading"], saved, " ".join(map(str, missing)),
                          " ".join(map(str, extra)),
                          *[step["place"].get(r) for r in ROBOTS]])
            mf.flush()
            print(f"captured {step['label']}: {saved} frames"
                  + (f"  (WARNING missing {missing})" if missing else ""))
            idx += 1
        else:
            continue
        if idx < len(plan):
            print(f"\nStep {idx + 1}/{len(plan)}: {plan[idx]['label']}")
            for line in describe(plan[idx]):
                print("  " + line)
    if idx >= len(plan):
        print("\nAll steps captured.")
    mf.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
