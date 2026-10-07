"""Self-test: question construction, graders and the report, no hardware.

Builds a synthetic tick shaped exactly like a captured one -- six robots on
measured node coordinates, a wait-for edge, job queues and slack -- then
checks that every question appears, that the graders accept a right answer
in each of the shapes a real model produces, and that the report renders.
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, "tools")

import vlm_live as vl

# Node coordinates from tools/testbed_layout.txt: node 1 at (13.5, 16.75),
# 10in pitch, row-major from the south-west.
def node(n):
    i = n - 1
    return (13.5 + 10.0 * (i % 8), 16.75 + 10.0 * (i // 8))

POSES = {"Alvik1": node(19), "Alvik2": node(20), "Alvik3": node(3),
         "Alvik4": node(44), "Alvik5": node(52), "Alvik6": node(61)}

SAMPLE = {
    "tick": 1,
    "t_iso": "2026-09-15T14:02:11",
    "truth": {
        "robot_count": 6,
        "robots": {n: {"x_in": x, "y_in": y, "yaw_deg": 90.0}
                   for n, (x, y) in POSES.items()},
    },
    "status": {
        "state": "running",
        "jobs": {"total": 12, "done": 3, "pending": 9, "orphaned": 0},
        "robots": {
            # Alvik1 wants node 20, where Alvik2 is standing -> held.
            "Alvik1": {"node": "19", "next_node": "20", "jobs_left": 2,
                       "next_ws": 22, "blocked_by": "Alvik2",
                       "slack_sec": 0.0, "phase": "waiting"},
            "Alvik2": {"node": "20", "next_node": "21", "jobs_left": 1,
                       "next_ws": 5, "slack_sec": 4.0, "phase": "busy"},
            "Alvik3": {"node": "3", "next_node": "4", "jobs_left": 3,
                       "next_ws": 7, "slack_sec": 30.0, "phase": "busy"},
            "Alvik4": {"node": "44", "next_node": "45", "jobs_left": 2,
                       "next_ws": 31, "slack_sec": 12.0, "phase": "busy"},
            "Alvik5": {"node": "52", "next_node": "53", "jobs_left": 0,
                       "slack_sec": 18.0, "phase": "busy"},
            "Alvik6": {"node": "61", "next_node": "62", "jobs_left": 2,
                       "next_ws": 40, "slack_sec": 9.0, "phase": "busy"},
        },
    },
}

qs = vl.build_ops_questions(SAMPLE)
print(f"{len(qs)} question(s) built\n")
for q in qs:
    print(f"  {q['id']:<24} truth={q['expected']}")

ids = {q["id"] for q in qs}
by = {q["id"]: q for q in qs}

# every graded question present
for want in ("ops:blocked", "ops:blocker", "ops:capacity", "ops:reassign",
             "ops:status"):
    assert want in ids, f"missing {want}"
ws_q = [i for i in ids if i.startswith("ops:closest_ws:")]
assert ws_q, "no workstation question"
print(f"\nworkstation question: {ws_q[0]}")

# -- ground truth is what the deterministic layer says -----------------------
assert by["ops:blocked"]["expected"] == "Alvik1", by["ops:blocked"]["expected"]
assert by["ops:blocker"]["expected"] == "Alvik2"
# Alvik5 has 0 jobs left, uniquely fewest
assert by["ops:capacity"]["expected"] == "Alvik5"
# Alvik3 has the most jobs (3) so it is the one declared down; among the rest
# Alvik5 has the most slack (18.0 vs Alvik4's 12.0), a 6.0s margin
assert by["ops:reassign"]["expected"] == "Alvik5", by["ops:reassign"]["expected"]
print("ground truth: as computed by the supervisor  OK")

# -- the prompt shows intent but never the answer ---------------------------
p = by["ops:blocked"]["prompt"]
assert "next heading to node 20" in p, "next_node missing from prompt"
assert "2 job(s) left" in p
assert "blocked_by" not in p and "waiting" not in p and "slack" not in p, \
    "prompt leaks the answer"
assert "Workstation bays 65-113" in p, "layout block missing"
print("prompt: carries intent, withholds blocked_by/phase/slack  OK")

# -- graders accept the shapes real models produce --------------------------
CASES = [
    ("ops:blocked", "Alvik1", True),
    ("ops:blocked", "**Alvik1** is being held.", True),
    ("ops:blocked", "Looking at Alvik2, Alvik3 and Alvik4... the robot being "
                    "held is **Alvik1**.", True),
    ("ops:blocked", "Alvik1 is at node 19 and Alvik2 at node 20, so the answer "
                    "is Alvik2.", False),
    ("ops:blocked", "No robot is blocked.", False),
    ("ops:blocker", "Alvik2 is in its way.", True),
    ("ops:blocker", "Alvik7", False),          # phantom name rejected
    ("ops:capacity", "The answer is **Alvik5** (0 jobs left).", True),
    ("ops:reassign", "ASSIGN: Alvik5\nIt has the most slack.", True),
    ("ops:reassign", "Alvik4 should take them.", False),
]
for qid, answer, want in CASES:
    got = bool(by[qid]["check"](answer))
    flag = "ok " if got == want else "FAIL"
    print(f"  {flag} {qid:<14} {want!s:<5} {answer[:46]!r}")
    assert got == want, (qid, answer, got, want)

assert by["ops:status"]["check"] is None, "status must be ungraded"
print("\ngraders: verdict-last, verdict-first, phantom names  OK")

# -- the clear case: nobody blocked ----------------------------------------
clear = json.loads(json.dumps(SAMPLE))
del clear["status"]["robots"]["Alvik1"]["blocked_by"]
cq = {q["id"]: q for q in vl.build_ops_questions(clear)}
assert cq["ops:blocked"]["expected"] == "NONE"
assert "ops:blocker" not in cq, "blocker asked with nobody blocked"
assert cq["ops:blocked"]["check"]("NONE") is True
assert cq["ops:blocked"]["check"]("No robot is currently blocked.") is True
assert cq["ops:blocked"]["check"]("None, though Alvik1 is close.") is False
print("clear case: NONE expected, blocker question dropped  OK")

# -- ambiguity is skipped, not guessed ------------------------------------
tie = json.loads(json.dumps(SAMPLE))
for r in tie["status"]["robots"].values():
    r["jobs_left"] = 2
tq = {q["id"] for q in vl.build_ops_questions(tie)}
assert "ops:capacity" not in tq, "capacity asked on a tie"
print("tie in jobs_left: capacity question skipped  OK")

near = json.loads(json.dumps(SAMPLE))
for r in near["status"]["robots"].values():
    r["slack_sec"] = 5.0
nq = {q["id"] for q in vl.build_ops_questions(near)}
assert "ops:reassign" not in nq, "reassign asked on a slack tie"
print("tie in slack: reassign question skipped  OK")

# -- no status at all: still asks what poses can answer --------------------
bare = {"tick": 1, "truth": SAMPLE["truth"], "status": None}
bq = {q["id"] for q in vl.build_ops_questions(bare)}
assert "ops:blocked" in bq and "ops:status" in bq
assert "ops:capacity" not in bq and "ops:reassign" not in bq
print("no /fleet_status: plan questions skipped, pose questions kept  OK")

# -- bay geometry -----------------------------------------------------------
bays = vl.bay_positions()
assert len(bays) == 49, len(bays)
assert bays[1] == (18.3, 21.76), bays[1]
print(f"bays: {len(bays)} measured, ws1 at {bays[1]}  OK")

# -- percentiles ----------------------------------------------------------
p50, p90, p95 = vl._pcts([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0])
assert (p50, p90, p95) == (5.5, 9.0, 10.0), (p50, p90, p95)
print(f"percentiles: p50 {p50} p90 {p90} p95 {p95}  OK")

# -- report renders -------------------------------------------------------
run = os.path.join("tools", "vlm_live", "_selftest")
d = os.path.join(run, "t0001")
os.makedirs(d, exist_ok=True)
# a 1x1 jpeg is enough; the report only needs the file to exist
open(os.path.join(d, "frame.jpg"), "wb").write(bytes.fromhex(
    "ffd8ffe000104a46494600010100000100010000ffdb004300"
    "0806060706050806070707090908" + "0a" * 50 + "ffd9"))
json.dump(SAMPLE, open(os.path.join(d, "state.json"), "w"), indent=1)
for model, answer, secs, ok in (("qwen3.5:4b", "Alvik1", 2.1, True),
                                ("gemma4:e2b-it-qat", "Alvik2", 0.3, False),
                                ("qwen3-vl:4b", "**Alvik1**", 7.4, True)):
    rows = []
    for q in qs:
        rows.append({"tick": 1, "question_id": q["id"],
                     "category": q["category"], "prompt": q["prompt"],
                     "answer": answer, "seconds": secs,
                     "expected": q["expected"],
                     "correct": None if q["check"] is None else ok,
                     "error": None, "eval_count": 5,
                     "prompt_eval_count": 900, "thinking_chars": 0})
    json.dump({"model": model, "url": "http://x", "run": "_selftest",
               "ollama_version": "0.13.5", "rows": rows},
              open(os.path.join(run, f"answers__{vl._safe(model)}.json"), "w"))

class A:
    run = "_selftest"
    out = None
    embed = False
    state_only = False
vl.RUNS_DIR = os.path.join("tools", "vlm_live")
vl.cmd_report(A())
html = open(os.path.join(run, "report.html"), encoding="utf-8").read()
for must in ("ops:blocked", "qwen3-vl:4b", "7.40s", "correct", "wrong",
             "Model summary", "By question type"):
    assert must in html, f"report missing {must}"
assert "<script" not in html.lower()
print(f"report: {len(html) / 1000:.0f}kB, all sections present  OK")
print("\nall self-tests passed")
