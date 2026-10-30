"""Write the DOE run order: 5 rounds x 8 runs, randomized within each round.

    python3 tools/make_doe_run_order.py

Outputs (docs/planning/):
  DOE_RUN_ORDER.txt   the printable run order and per-run checklist
  DOE_RUN_SHEET.csv   the same 40 rows as a tick-off sheet

Every row names the exact saved plan to select (DOE_<4|6>r_<method>.json)
and the video name the app will give it automatically: the plan's name plus
the repeat number (DOE_6r_cbs_1 in round 1, DOE_6r_cbs_2 in round 2, ...).
The order is fixed by SEED, so re-running this script reproduces it.
"""
from __future__ import annotations

import csv
import json
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "planning"
SEED = 20261019
ROUNDS = 5
METHODS = [("astar", "A*"), ("wait", "WAIT"), ("pp", "PP"), ("cbs", "CBS")]
ROBOTS = [4, 6]
MIN_PER_RUN = {6: 13, 4: 9}        # measured attempt incl. reset; 4r estimated


def build() -> list[dict]:
    rng = random.Random(SEED)
    combos = [(r, m, label) for r in ROBOTS for m, label in METHODS]
    rows, reps = [], {}
    for rnd in range(1, ROUNDS + 1):
        order = list(combos)
        rng.shuffle(order)
        for robots, method, label in order:
            plan = f"DOE_{robots}r_{method}"
            reps[plan] = reps.get(plan, 0) + 1
            rows.append({"run": len(rows) + 1, "round": rnd, "robots": robots,
                         "method": label, "file": f"{plan}.json",
                         "video": f"{plan}_{reps[plan]}"})
    return rows


def main() -> None:
    manifest = json.loads((OUT / "DOE_FROZEN_MANIFEST.json").read_text(encoding="utf-8"))
    deadline = {int(k[:-1]): v for k, v in manifest.get("deadlines_sec", {}).items()}
    rows = build()

    with open(OUT / "DOE_RUN_SHEET.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "round", "robots", "method", "file_to_select",
                    "video_name_expected", "deadline_sec", "done",
                    "outcome", "video_name_actual", "lowest_battery_pct",
                    "notes"])
        for r in rows:
            w.writerow([r["run"], r["round"], r["robots"], r["method"], r["file"],
                        r["video"], deadline.get(r["robots"], ""), "", "", "", "", ""])

    def mmss(sec):
        return f"{int(sec // 60)}:{int(sec % 60):02d}"

    L = []
    L += ["DOE RUN ORDER",
          "Traffic planner (A*, WAIT, PP, CBS) x robots (4, 6), 5 rounds of 8",
          "Design: DOE_TRAFFIC_PLANNING_2026-10-19.txt",
          "Tick-off sheet (same rows): DOE_RUN_SHEET.csv",
          f"Order fixed by seed {SEED} (tools/make_doe_run_order.py).",
          "", ""]
    L += ["EVERY RUN", "=" * 70, "",
          "  1. Robots in their own slots (Alvik N in DN), squared up, at 60%",
          "     battery or more (was 70% until 2026-10-23; Round 1 showed no",
          "     effect down to 67%, and Alvik3's stops stayed short down to 69%.",
          "     Below 60% Alvik3 coasts ~1.70in, its whole brake lead -- charge",
          "     it). For a 4-robot run, Alvik5 and Alvik6 stay parked in D5 and D6.",
          "  2. Saved plans: select the run's FILE from the table below.",
          "  3. Run & Monitor: Drive mode must read Camera + Encoder, and the",
          "     Traffic planner row must highlight the run's METHOD. Do not",
          "     click a traffic button.",
          "  4. Recording row: leave the name box EMPTY. The status line must",
          "     read: the next run records as \"<VIDEO>\" after Send mission.",
          "  5. Send mission -> wait for \"loaded\" -> Start.",
          "  6. Hands off. Abort only at the DEADLINE or for safety.",
          "  7. Tick the row: outcome (completed / failed / aborted /",
          "     infrastructure fault), and anything odd with its time.",
          "  8. Reset the robots to their slots.",
          "",
          "  If a run must be repeated (infrastructure fault), repeat the SAME",
          "  file straight away and keep both rows. Its video number goes up by",
          "  one, and so does every later video of that file: write the name",
          "  the status line shows into video_name_actual.",
          "",
          "  Deadlines (2 x the PP planned makespan): "
          + ", ".join(f"{r} robots {deadline[r]:.0f} s ({mmss(deadline[r])})"
                      for r in sorted(deadline, reverse=True)),
          "", ""]
    for rnd in range(1, ROUNDS + 1):
        part = [r for r in rows if r["round"] == rnd]
        est = sum(MIN_PER_RUN[r["robots"]] for r in part)
        L += [f"ROUND {rnd}   (about {est // 60} h {est % 60:02d} min)", "=" * 70, "",
              "   Run  FILE TO SELECT        Robots  Method  Video name (automatic)   Done",
              "   ---  --------------------  ------  ------  -----------------------  ----"]
        for r in part:
            L.append(f"   {r['run']:>3}  {r['file']:<20}  {r['robots']:>6}  "
                     f"{r['method']:<6}  {r['video']:<23}  [ ]")
        L += ["", ""]
    L += ["BEFORE ROUND 1", "=" * 70, "",
          "  - Shakedown, not data: one 6-robot run each of WAIT, PP and CBS",
          "    (A* done 2026-10-19). Type SHAKE WAIT etc. in the name box so",
          "    these videos are not numbered as DOE runs.",
          "  - Declare in the design document: drive mode, and the",
          "    infrastructure-fault rule.",
          "  - Restart the testbed at the start of each lab day. If time runs",
          "    out, stop at the end of a round, never in the middle of one.",
          ""]
    (OUT / "DOE_RUN_ORDER.txt").write_text("\n".join(L), encoding="utf-8")
    print(f"wrote {OUT / 'DOE_RUN_ORDER.txt'} and DOE_RUN_SHEET.csv ({len(rows)} runs)")


if __name__ == "__main__":
    main()
