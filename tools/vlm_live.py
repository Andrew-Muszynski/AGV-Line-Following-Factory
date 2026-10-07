#!/usr/bin/env python3
"""Stage 2: live temporal model selection on the running testbed.

WHAT THIS IS FOR
----------------
The controlled benchmark (vlm_capture.py + vlm_bench.py) narrowed ten models
to three: qwen3.5:4b, gemma4:e2b-it-qat and qwen3-vl:4b, at 87%, 87% and 88%.
Accuracy has stopped separating them. What has to separate them now is
whether their answers are USABLE during a run: right, fast enough to act on,
and about questions the supervisor actually needs answered.

So the questions change. vlm_bench asks "how many robots are there" -- a
perception probe. This asks "which robot cannot move because another is in
its way", "which robot has the fewest jobs left", "Alvik4 just died, who
should take its work". Every one of those has an exact answer already
computed by the deterministic layer, which is what makes them gradeable
rather than a vibe check.

WHY CAPTURE AND ASK ARE SEPARATE COMMANDS
-----------------------------------------
The three models are 3.30, 4.34 and 3.39 GB. That is 11.03 GB of weights and
the laptop has 8 GB of VRAM, so they cannot be resident together. A
round-robin that asks all three at each timestamp -- the obvious design, and
the one a counterbalanced execution order implies -- forces an Ollama model
swap on every single call. A cold load is seconds, it would dominate the
latency measurement, and latency is the entire reason this experiment exists.

So: capture once, live, during a real run. Then ask each model in its own
pass over the identical saved ticks, resident the whole time. That gives

  * byte-identical inputs to all three, which is stronger than
    counterbalancing an order -- there is no order left to confound,
  * no swap cost inside any measurement,
  * and each model measured in the configuration it would be DEPLOYED in,
    which is one model resident. The final system contains one VLM.

Running the fleet three times, once per model, would also work but is
weaker: the robots do not land on the same squares twice, so the three
models would be answering about three different scenes.

WHAT THE MODEL SEES, AND WHAT IT DOES NOT
-----------------------------------------
Given: the camera frame, every robot's measured position and yaw, the static
layout, and each robot's current node, next node and remaining job count.
That is the digital twin -- the same material a deployed advisor would get.

Withheld, because they ARE the answers: blocked_by, slack_sec, phase and the
deadlock list. phase matters most: phase == "waiting" literally means "this
robot is held", so publishing it would turn the blocked question into
retrieval, and the controlled benchmark already showed retrieval is 100%
while derivation is where these models fail (49 of 110 coordinate-to-node
mappings wrong). The interesting question is whether a model can DERIVE a
blockage from positions and intent.

USAGE
-----
During a six-robot run, from WSL on the camera laptop (the stream lives in
WSL's network namespace and Windows cannot reach it):

    python3 tools/vlm_live.py capture --run r1 --interval 20 \\
        --stream http://127.0.0.1:8081/full.jpg \\
        --rosbridge 192.168.0.212:9090 \\
        --robots Alvik1 Alvik2 Alvik3 Alvik4 Alvik5 Alvik6

Afterwards, one model at a time (each pass loads one model and keeps it):

    python3 tools/vlm_live.py ask --run r1 --model qwen3.5:4b
    python3 tools/vlm_live.py ask --run r1 --model gemma4:e2b-it-qat
    python3 tools/vlm_live.py ask --run r1 --model qwen3-vl:4b

Then read the answers:

    python3 tools/vlm_live.py report --run r1

which writes tools/vlm_live/r1/report.html -- every question, every model's
answer in full, side by side, with latency and the measured truth.

Nothing here ever commands a robot. capture subscribes and looks; ask reads
files.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import statistics
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vlm_bench import (            # noqa: E402  the scoring core, unchanged
    DEFAULT_URL, _phantom_names, _says_none, _verdict_chunks, ask,
    parse_model_spec, state_text,
)
from vlm_capture import grab_jpeg, jpeg_size, truth_for  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(HERE, "vlm_live")

#: A workstation question is only asked when the nearest robot is at least
#: this much closer than the runner-up. Same 4in guard the controlled
#: benchmark uses for closest-pair: inside that, "which is closest" is not a
#: fair visual call and a wrong answer says nothing about the model.
CLOSEST_MARGIN_IN = 4.0

#: Reassignment is only graded when one candidate has at least this much more
#: slack than the next. Below it, two robots are equally good choices and
#: there is no single right answer to score against.
SLACK_MARGIN_SEC = 2.0

#: Per-question ceiling. Deployment target is 3s and the cap is 5s (a 20s
#: sampling interval is not an acceptable inference latency), but the call is
#: allowed to finish so the real number lands on disk -- a timeout would
#: record 60.0 for every slow answer and hide the distribution.
REQUEST_TIMEOUT_SEC = 120.0


# ------------------------------------------------------------------ geometry

_BAYS_CACHE: dict = {}


def bay_positions() -> dict[int, tuple[float, float]]:
    """{solver workstation id: (x_in, y_in)} from the measured calibration.

    Keyed by the SOLVER's workstation id (1-49), not the grid node (65-113).
    node = workstation + 64; the schedule, the job registry and an operator
    all say "workstation 7", so that is the key. Read from
    fleet/workstation_calibration.json rather than recomputed from pitch: all
    49 bays were physically measured on 2026-08-26 and the old arithmetic
    offset (0.44) was wrong by 0.06in.
    """
    if not _BAYS_CACHE:
        path = os.path.join(os.path.dirname(HERE), "fleet",
                            "workstation_calibration.json")
        try:
            with open(path, encoding="utf-8") as fh:
                nodes = (json.load(fh).get("nodes") or {})
            for key, rec in nodes.items():
                try:
                    _BAYS_CACHE[int(key) - 64] = (float(rec["x_in"]),
                                                  float(rec["y_in"]))
                except (KeyError, TypeError, ValueError):
                    continue
        except OSError:
            pass
    return _BAYS_CACHE


# ------------------------------------------------------------------- answers

_NAME_RE = re.compile(r"\bAlvik\s*([1-9])\b", re.I)


def names_in(text: str) -> list[str]:
    """Robot names in order of first appearance, deduplicated."""
    out: list[str] = []
    for digit in _NAME_RE.findall(text or ""):
        name = f"Alvik{digit}"
        if name not in out:
            out.append(name)
    return out


#: Phrases a model puts immediately before its conclusion. Searched from the
#: END of the answer, so the LAST cue wins -- a model that reasons then
#: concludes has its conclusion after the final cue, not the first one.
_CUES = ("answer is", "answer:", "assign:", "assign ", "therefore",
         "held is", "being held is", "blocked is", "in its way is",
         "closest is", "closest to", "should take", "so ", "-> ", "= ")


def _after_cue(text: str) -> str:
    """Whatever follows the last conclusion cue, or "" if there is none."""
    low = (text or "").lower()
    best = -1
    for cue in _CUES:
        idx = low.rfind(cue)
        if idx > best:
            best, width = idx, len(cue)
    if best < 0:
        return ""
    return text[best + width:]


def verdict_name(answer: str, robots: list[str],
                 ignore: set | None = None) -> str | None:
    """The one robot an answer CONCLUDES with, or None.

    Two rules, both learned the hard way.

    First, chunk narrowing, via vlm_bench's _verdict_chunks: a model that
    reasons aloud names every candidate it considered before committing, so
    matching the full transcript reads a hedge where there is a clear verdict.
    That cost 63 of 114 facing answers and the whole closest-pair template in
    one run of the controlled benchmark.

    Second -- and this is the addition here -- a chunk only counts as a
    verdict if it names EXACTLY ONE robot. Every question in this set asks
    for one name, so a chunk holding two names is reasoning, not an answer.
    Without that rule the first name in a one-line reply wins, and
    "Alvik1 is at node 19 and Alvik2 at node 20, so the answer is Alvik2"
    scores as Alvik1 -- crediting the model for an answer it did not give.
    A single line has no last-line or last-sentence to narrow to, so the
    exactly-one rule is the only thing standing between a verbose reply and
    a wrong credit.

    Third: `ignore` holds names the QUESTION itself supplies, and they are
    struck out before the exactly-one rule is applied. Measured on r4,
    2026-09-15 -- without this, ops:reassign scored 0/8 for every model, and
    the failure was the grader, not the models. That prompt says "AlvikN has
    broken down" and asks for "one robot name, then one sentence of
    reasoning", so a correct reply necessarily names two robots:

        "Alvik4 should take the remaining jobs from Alvik1 because..."

    Two names, so the chunk was discarded as reasoning and a right answer
    scored wrong. The downed robot is not a competing verdict -- it is the
    question being restated. Striking it leaves exactly one name and the
    rule works as intended. Naming the ignored robot as the ANSWER still
    scores wrong: the chunk then holds no candidate at all and falls through
    to None, which is the correct verdict for "give the jobs to the robot
    that just broke down".

    An unreadable answer returns None and is scored wrong rather than
    guessed at. Under-crediting a verbose but correct reply is the safe
    direction: the full text is in the report, where a systematic pattern of
    it would be visible -- which is exactly how the ops:reassign bug above
    was eventually caught.
    """
    text = answer or ""
    skip = {s.lower() for s in (ignore or set())}
    chunks = []
    cue = _after_cue(text)
    if cue.strip():
        chunks.append(cue)
    chunks.extend(_verdict_chunks(text))
    for chunk in chunks:
        found = [n for n in names_in(chunk) if n.lower() not in skip]
        if len(set(found)) != 1:
            continue
        if _phantom_names(chunk, robots):
            continue
        return found[0]
    return None


# ----------------------------------------------------------------- questions

def plan_state_text(status: dict) -> str:
    """Current node, next node and remaining work, as prose.

    This is the half of the digital twin the camera cannot see. Without it
    "which robot is blocked" has no answer derivable from the inputs --
    proximity alone is not a blockage, intent is what makes it one.
    """
    robots = (status or {}).get("robots") or {}
    lines = []
    for name in sorted(robots):
        r = robots[name] or {}
        bits = []
        if r.get("node") not in (None, "None"):
            bits.append(f"at node {r['node']}")
        if r.get("next_node") not in (None, "None"):
            bits.append(f"next heading to node {r['next_node']}")
        else:
            bits.append("no further moves planned")
        left = r.get("jobs_left")
        if left is not None:
            bits.append(f"{left} job(s) left")
        if r.get("next_ws") is not None:
            bits.append(f"next job at workstation {r['next_ws']}")
        lines.append(f"- {name}: " + ", ".join(bits))
    if not lines:
        return ""
    return ("Fleet plan state (from the supervisor's route plans):\n"
            + "\n".join(lines))


def blocked_robots(status: dict) -> dict[str, str]:
    """{waiter: blocker} straight off the advisor's wait-for graph."""
    out = {}
    for name, r in ((status or {}).get("robots") or {}).items():
        blocker = (r or {}).get("blocked_by")
        if blocker:
            out[name] = str(blocker)
    return out


def build_ops_questions(sample: dict) -> list[dict]:
    """The frozen operational question set for one captured tick.

    FROZEN before the experiment, deliberately, so that a disappointing
    result cannot be answered by rewording the question until the number
    improves. Five graded questions and one ungraded status summary; a
    question is SKIPPED (not guessed at) whenever the truth is ambiguous,
    because a coin-flip question measures nothing.

    Each entry: {id, category, prompt, expected, check}. check(answer) ->
    bool; expected is carried for the report, not for grading.
    """
    truth = sample.get("truth") or {}
    status = sample.get("status") or {}
    poses = truth.get("robots") or {}
    robots = sorted(poses)
    srobots = (status.get("robots") or {})
    questions: list[dict] = []
    if not robots:
        return questions

    context = state_text(truth, include_layout=True)
    plan = plan_state_text(status)
    if plan:
        context = context + "\n" + plan

    def q(qid: str, category: str, question: str, expected, check) -> None:
        questions.append({
            "id": qid, "category": category, "expected": expected,
            "check": check,
            "prompt": context + "\n\n" + question,
        })

    # -- 1. blocked: the question the supervisor answers every tick --------
    blocked = blocked_robots(status)
    if blocked:
        waiters = sorted(blocked)

        def _check_blocked(a: str, waiters=waiters) -> bool:
            name = verdict_name(a, robots)
            return name in waiters
        q("ops:blocked", "traffic",
          "One robot cannot move right now because another robot is in the "
          "way of where it needs to go next. Which robot is being held? "
          "Answer with one robot name only.",
          ", ".join(waiters), _check_blocked)
    else:
        def _check_clear(a: str) -> bool:
            # Both halves required: "no" with a name attached is not a
            # usable answer for something that would act on it.
            return _says_none(a) and not names_in(a)
        q("ops:blocked", "traffic",
          "Is any robot currently unable to move because another robot is in "
          "the way of where it needs to go next? If one is, name it. If none "
          "is, answer NONE.",
          "NONE", _check_clear)

    # -- 2. blocker: only when there is exactly one waiter -----------------
    if len(blocked) == 1:
        waiter, blocker = next(iter(blocked.items()))

        def _check_blocker(a: str, blocker=blocker, waiter=waiter) -> bool:
            # The waiter is named in the prompt; restating it is not a
            # second candidate -- see verdict_name's third rule.
            return verdict_name(a, robots, ignore={waiter}) == blocker
        q("ops:blocker", "traffic",
          f"{waiter} cannot move to its next node right now. Which robot is "
          f"in its way? Answer with one robot name only.",
          blocker, _check_blocker)

    # -- 3. spare capacity: fewest jobs left, when it is not a tie ---------
    left = {n: srobots[n]["jobs_left"] for n in srobots
            if isinstance((srobots[n] or {}).get("jobs_left"), int)}
    if len(left) >= 2:
        order = sorted(left.items(), key=lambda kv: kv[1])
        if order[0][1] < order[1][1]:
            want = order[0][0]

            def _check_fewest(a: str, want=want) -> bool:
                return verdict_name(a, robots) == want
            q("ops:capacity", "dispatch",
              "Which robot has the fewest jobs left to do, and therefore the "
              "most spare capacity for extra work? Answer with one robot "
              "name only.",
              want, _check_fewest)

    # -- 4. closest to a workstation that actually matters -----------------
    bays = bay_positions()
    targets = [r["next_ws"] for r in srobots.values()
               if isinstance((r or {}).get("next_ws"), int)
               and r["next_ws"] in bays]
    if targets:
        # The busiest robot's next bay: a real destination in this run, not
        # an arbitrary one, so the answer is operationally meaningful.
        ws = targets[0]
        if left:
            busiest = max(left.items(), key=lambda kv: kv[1])[0]
            cand = (srobots.get(busiest) or {}).get("next_ws")
            if isinstance(cand, int) and cand in bays:
                ws = cand
        bx, by = bays[ws]
        dist = sorted(
            ((math.hypot(poses[n]["x_in"] - bx, poses[n]["y_in"] - by), n)
             for n in robots))
        if len(dist) >= 2 and dist[1][0] - dist[0][0] >= CLOSEST_MARGIN_IN:
            want = dist[0][1]

            def _check_closest(a: str, want=want) -> bool:
                return verdict_name(a, robots) == want
            q(f"ops:closest_ws:{ws}", "spatial",
              f"Which robot is physically closest to workstation {ws}? "
              f"Answer with one robot name only.",
              want, _check_closest)

    # -- 5. downed robot: Experiment 3's core decision, scored one-shot ----
    slack = {n: srobots[n]["slack_sec"] for n in srobots
             if isinstance((srobots[n] or {}).get("slack_sec"), (int, float))}
    if len(slack) >= 3 and left:
        down = max(left.items(), key=lambda kv: kv[1])[0]
        others = sorted(((v, k) for k, v in slack.items() if k != down),
                        reverse=True)
        if len(others) >= 2 and others[0][0] - others[1][0] >= SLACK_MARGIN_SEC:
            want = others[0][1]
            njobs = left.get(down)

            def _check_reassign(a: str, want=want, down=down) -> bool:
                # `down` is named in the prompt, so the answer restating it
                # is not a second candidate -- see verdict_name's third rule.
                return verdict_name(a, robots, ignore={down}) == want
            # The deterministic answer is the robot with the most slack
            # against the CURRENT critical path -- the same basis
            # resolve_deadlock() uses to pick a yielder, and for the same
            # reason: delaying a robot that is not setting the makespan
            # costs the fleet nothing.
            q("ops:reassign", "recovery",
              f"{down} has broken down and cannot move again. Its {njobs} "
              f"remaining job(s) have to be given to another robot. Which "
              f"one robot should take them so the whole fleet finishes as "
              f"early as possible? Answer with one robot name, then one "
              f"sentence of reasoning.",
              want, _check_reassign)

    # -- 6. ungraded: the sentence a human would actually read -------------
    q("ops:status", "narrative",
      "In one sentence, describe what the fleet is doing right now.",
      None, None)
    return questions


# ------------------------------------------------------------------- capture

def cmd_capture(args: argparse.Namespace) -> None:
    from vlm_capture import RosbridgePoseSnapshot

    out = os.path.join(RUNS_DIR, args.run)
    os.makedirs(out, exist_ok=True)

    poses = RosbridgePoseSnapshot(args.robots, args.rosbridge_host,
                                  args.rosbridge_port)
    status = _StatusListener(poses)
    print(f"capturing every {args.interval:g}s into {out}")
    print("ctrl-c to stop\n")

    tick = _next_tick(out)
    t_next = time.monotonic()
    try:
        while args.ticks == 0 or tick <= args.ticks:
            now = time.monotonic()
            if now < t_next:
                poses.pump(min(0.2, t_next - now))
                continue
            t_next = now + args.interval

            fresh = poses.fresh(args.max_pose_age)
            if len(fresh) < args.min_robots:
                print(f"  tick {tick}: only {len(fresh)}/{args.min_robots} "
                      f"robots with fresh vision -- skipped")
                continue
            snap = status.snapshot()
            if snap is None and not args.allow_no_status:
                print(f"  tick {tick}: no /fleet_status yet -- skipped "
                      f"(is the run started? --allow-no-status to capture "
                      f"anyway)")
                continue
            try:
                frame = grab_jpeg(args.stream)
            except Exception as exc:
                print(f"  tick {tick}: frame grab failed: {exc!r}")
                continue

            d = os.path.join(out, f"t{tick:04d}")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "frame.jpg"), "wb") as fh:
                fh.write(frame)
            size = jpeg_size(frame) or (None, None)
            sample = {
                "tick": tick,
                "t_wall": time.time(),
                "t_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "truth": truth_for(fresh),
                "status": snap,
                "status_age_sec": status.age(),
                "frame_bytes": len(frame),
                "frame_width": size[0], "frame_height": size[1],
            }
            with open(os.path.join(d, "state.json"), "w",
                      encoding="utf-8") as fh:
                json.dump(sample, fh, indent=1, sort_keys=True)
            held = blocked_robots(snap or {})
            print(f"  tick {tick}: {len(fresh)} robots, "
                  f"{len(frame) / 1000:.0f}kB, "
                  f"{len(build_ops_questions(sample))} questions"
                  + (f", HELD: {', '.join(sorted(held))}" if held else ""))
            tick += 1
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        status.close()
        poses.close()
    print(f"\n{tick - 1} tick(s) in {out}")
    print(f"next: python3 tools/vlm_live.py ask --run {args.run} "
          f"--model qwen3.5:4b")


class _StatusListener:
    """Latest /fleet_status payload, over the rosbridge client we already have.

    Reuses RosbridgePoses' connection rather than opening a second one --
    rosbridge is on the Linux laptop and every extra websocket is another
    thing to time out mid-run.
    """

    def __init__(self, poses) -> None:
        self._latest = None
        self._at = 0.0
        self._topic = None
        try:
            import roslibpy
            self._topic = roslibpy.Topic(poses._client, "/fleet_status",
                                         "std_msgs/String")
            self._topic.subscribe(self._cb)
        except Exception as exc:
            print(f"warning: cannot subscribe /fleet_status ({exc!r}); "
                  f"plan state and grading truth will be missing")

    def _cb(self, msg) -> None:
        try:
            self._latest = json.loads(msg["data"])
            self._at = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def snapshot(self):
        return self._latest

    def age(self) -> float | None:
        return None if self._at == 0.0 else round(time.monotonic() - self._at, 2)

    def close(self) -> None:
        try:
            if self._topic is not None:
                self._topic.unsubscribe()
        except Exception:
            pass


def _next_tick(out: str) -> int:
    """Resume numbering, so a restarted capture appends instead of clobbering."""
    existing = [int(n[1:]) for n in os.listdir(out)
                if re.fullmatch(r"t\d{4}", n)] if os.path.isdir(out) else []
    return (max(existing) + 1) if existing else 1


# ----------------------------------------------------------------------- ask

def load_ticks(run: str) -> list[tuple[str, dict]]:
    out = os.path.join(RUNS_DIR, run)
    if not os.path.isdir(out):
        raise SystemExit(f"no run at {out}")
    ticks = []
    for name in sorted(os.listdir(out)):
        d = os.path.join(out, name)
        path = os.path.join(d, "state.json")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as fh:
                ticks.append((d, json.load(fh)))
    if not ticks:
        raise SystemExit(f"{out} holds no captured ticks")
    return ticks


def ollama_version(url: str) -> str | None:
    """Recorded with every pass. A reported qwen3.5 VQA regression between
    Ollama releases means the backend version is experimental metadata, not
    an uncontrolled variable."""
    try:
        with urllib.request.urlopen(f"{url}/api/version", timeout=5) as resp:
            return (json.loads(resp.read()) or {}).get("version")
    except Exception:
        return None


def cmd_ask(args: argparse.Namespace) -> None:
    model, url = parse_model_spec(args.model, args.url)
    ticks = load_ticks(args.run)
    out = os.path.join(RUNS_DIR, args.run)
    # The condition is part of the filename, not just the metadata inside:
    # a state-only control pass and a frame+state pass are two different
    # measurements of the same model, and sharing one path silently replaces
    # the first with the second.
    tag = _safe(model) + ("__stateonly" if args.no_image else "")
    dest = os.path.join(out, f"answers__{tag}.json")

    total = sum(len(build_ops_questions(s)) for _d, s in ticks)
    print(f"{model} @ {url}: {len(ticks)} tick(s), {total} question(s)")
    ver = ollama_version(url)
    print(f"ollama {ver or 'version unknown'}\n")

    # Warm up OFF the record. The cold load is real but it is not the
    # latency of answering a question, and it lands entirely on whichever
    # question happens to be first. Measured 2026-09-15: gemma4's first call
    # took 96.8s against a 0.24s median for every call after it, which would
    # have put both the p90 and the p95 of that pass at 96.8s. ask() sets
    # keep_alive to 30m, so one warm-up covers the whole pass.
    with open(os.path.join(ticks[0][0], "frame.jpg"), "rb") as fh:
        warm = base64.b64encode(fh.read()).decode()
    t_warm = time.monotonic()
    _a, _s, warm_err, _m = ask(url, model, "Reply with the word ready.",
                               None if args.no_image else warm,
                               REQUEST_TIMEOUT_SEC, num_ctx=args.num_ctx,
                               num_predict=args.num_predict)
    print(f"warm-up: model resident in {time.monotonic() - t_warm:.1f}s"
          + (f" (warning: {warm_err})" if warm_err else ""))
    print()

    rows: list[dict] = []
    done = 0
    t0 = time.monotonic()
    for d, sample in ticks:
        with open(os.path.join(d, "frame.jpg"), "rb") as fh:
            img = base64.b64encode(fh.read()).decode()
        for q in build_ops_questions(sample):
            # num_predict bounds the answer channel. Why 512 is the working
            # value, measured on r4 (43 questions x 3 models, 2026-09-15):
            # output length is BIMODAL, not spread. gemma4 answers in a
            # median of 4 tokens and never exceeds 77, so no cap in this
            # range touches it at all. qwen3.5 answers 22 of 43 questions in
            # under 256 tokens and runs the other 21 to ~4900. qwen3-vl runs
            # to ~4900 on 10 of 11. Almost nothing lands in between: raising
            # 512 to 2048 rescues 4 rows out of 43, on one model, and costs
            # roughly 4x the wall time -- and a truncated answer scores wrong
            # at either cap because it stops mid-sentence before any verdict.
            answer, secs, err, meta = ask(
                url, model, q["prompt"], None if args.no_image else img,
                REQUEST_TIMEOUT_SEC, num_ctx=args.num_ctx,
                num_predict=args.num_predict)
            correct = None
            if q["check"] is not None and answer is not None:
                try:
                    correct = bool(q["check"](answer))
                except Exception as exc:          # a grader bug is not a
                    correct = None                # wrong answer
                    err = err or f"grader: {exc!r}"
            elif q["check"] is not None:
                correct = False                   # no answer is a wrong answer
            rows.append({
                "tick": sample["tick"], "question_id": q["id"],
                "category": q["category"], "prompt": q["prompt"],
                "answer": answer, "seconds": round(secs, 3),
                "expected": q["expected"], "correct": correct, "error": err,
                "eval_count": meta.get("eval_count"),
                "prompt_eval_count": meta.get("prompt_eval_count"),
                "thinking_chars": meta.get("thinking_chars"),
                "thinking_text": meta.get("thinking_text"),
            })
            done += 1
            mark = "?" if correct is None else ("ok " if correct else "WRONG")
            print(f"  [{done}/{total}] t{sample['tick']:04d} "
                  f"{q['id']:<22} {secs:6.2f}s {mark:<6}"
                  f"{_one_line(answer or err or '', 52)}")
            _save(dest, model, url, args.run, ver, rows, args)
    elapsed = time.monotonic() - t0
    print(f"\n{done} question(s) in {elapsed / 60:.1f} min -> {dest}")
    _model_summary(model, rows)
    print(f"\nwhen every model has a pass: "
          f"python3 tools/vlm_live.py report --run {args.run}")


def _save(dest: str, model: str, url: str, run: str, ver, rows, args) -> None:
    """Checkpointed after every answer. A 14.5-hour run was nearly lost to a
    crash in the REPORT once; the data has been written as it arrives ever
    since."""
    tmp = dest + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump({"model": model, "url": url, "run": run,
                   "ollama_version": ver,
                   "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                  time.gmtime()),
                   "condition": "state-only" if args.no_image else "frame+state",
                   "num_ctx": args.num_ctx,
                   # Recorded so a pass says what it ran under: comparing a
                   # capped pass against an uncapped one is not a comparison.
                   "num_predict": args.num_predict,
                   "rows": rows}, fh, indent=1)
    os.replace(tmp, dest)


def _safe(model: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]", "_", model)


def _one_line(text: str, width: int) -> str:
    flat = " ".join((text or "").split())
    return flat[:width] + ("..." if len(flat) > width else "")


def _pcts(values: list[float]) -> tuple[float, float, float]:
    """(p50, p90, p95). Computed by rank on the sorted sample rather than by
    interpolation: with 60-90 calls per model the difference is noise, and
    rank means every printed number is a latency that actually occurred."""
    s = sorted(values)
    if not s:
        return (0.0, 0.0, 0.0)

    def at(p: float) -> float:
        return s[min(len(s) - 1, max(0, int(math.ceil(p * len(s))) - 1))]
    return (statistics.median(s), at(0.90), at(0.95))


def _model_summary(model: str, rows: list[dict]) -> None:
    graded = [r for r in rows if r.get("correct") is not None]
    lat = [r["seconds"] for r in rows if r.get("error") is None]
    p50, p90, p95 = _pcts(lat)
    hits = sum(1 for r in graded if r["correct"])
    errs = sum(1 for r in rows if r.get("error"))
    over = sum(1 for s in lat if s > 5.0)
    print(f"{model}: {hits}/{len(graded)} graded correct"
          + (f" ({100.0 * hits / len(graded):.0f}%)" if graded else "")
          + f"  p50 {p50:.2f}s  p90 {p90:.2f}s  p95 {p95:.2f}s"
          + f"  errors {errs}  over-5s {over}/{len(lat)}")


# -------------------------------------------------------------------- report

def cmd_report(args: argparse.Namespace) -> None:
    out = os.path.join(RUNS_DIR, args.run)
    ticks = load_ticks(args.run)
    passes = []
    for name in sorted(os.listdir(out)):
        if not (name.startswith("answers__") and name.endswith(".json")):
            continue
        if name.endswith("__stateonly.json") and not args.state_only:
            continue          # the control condition is its own report
        if args.state_only and not name.endswith("__stateonly.json"):
            continue
        with open(os.path.join(out, name), encoding="utf-8") as fh:
            passes.append(json.load(fh))
    if not passes:
        raise SystemExit(f"{out} has no answers__*.json -- run `ask` first")

    dest = args.out or os.path.join(out, "report.html")
    html = _render(args.run, ticks, passes, embed=args.embed)
    with open(dest, "w", encoding="utf-8") as fh:
        fh.write(html)

    print(f"{len(passes)} model pass(es), {len(ticks)} tick(s)\n")
    for p in passes:
        _model_summary(p["model"], p["rows"])
    print(f"\nwrote {dest}")


def _render(run: str, ticks, passes, embed: bool) -> str:
    models = [p["model"] for p in passes]
    by: dict = {}
    for p in passes:
        for r in p["rows"]:
            by.setdefault((r["tick"], r["question_id"]), {})[p["model"]] = r

    e = _esc
    parts = [_HEAD.replace("{run}", e(run))]

    # ---- summary
    parts.append('<section class="card"><h2>Model summary</h2>'
                 '<div class="scroll"><table><thead><tr>'
                 '<th>model</th><th class="n">graded</th><th class="n">correct</th>'
                 '<th class="n">p50</th><th class="n">p90</th><th class="n">p95</th>'
                 '<th class="n">&gt;5s</th><th class="n">errors</th>'
                 '<th>ollama</th></tr></thead><tbody>')
    for p in passes:
        rows = p["rows"]
        graded = [r for r in rows if r.get("correct") is not None]
        lat = [r["seconds"] for r in rows if r.get("error") is None]
        p50, p90, p95 = _pcts(lat)
        hits = sum(1 for r in graded if r["correct"])
        pct = f"{100.0 * hits / len(graded):.0f}%" if graded else "—"
        parts.append(
            f'<tr><td class="model">{e(p["model"])}</td>'
            f'<td class="n">{len(graded)}</td>'
            f'<td class="n"><strong>{hits}</strong> <span class="dim">'
            f'{pct}</span></td>'
            f'<td class="n">{p50:.2f}s</td>'
            f'<td class="n">{p90:.2f}s</td>'
            f'<td class="n {"bad" if p95 > 5 else ""}">{p95:.2f}s</td>'
            f'<td class="n">{sum(1 for s in lat if s > 5.0)}</td>'
            f'<td class="n">{sum(1 for r in rows if r.get("error"))}</td>'
            f'<td class="dim">{e(p.get("ollama_version") or "?")}</td></tr>')
    parts.append('</tbody></table></div>'
                 '<p class="note">Deployment target is 3s with a 5s ceiling. '
                 'p95 is the number that matters for a robot waiting on an '
                 'answer, not p50.</p></section>')

    # ---- per-category accuracy
    cats: dict = {}
    for p in passes:
        for r in p["rows"]:
            if r.get("correct") is None:
                continue
            slot = cats.setdefault(r["category"], {}).setdefault(
                p["model"], [0, 0])
            slot[1] += 1
            slot[0] += 1 if r["correct"] else 0
    if cats:
        parts.append('<section class="card"><h2>By question type</h2>'
                     '<div class="scroll"><table><thead><tr><th>category</th>')
        for m in models:
            parts.append(f'<th class="n">{e(m)}</th>')
        parts.append('</tr></thead><tbody>')
        for cat in sorted(cats):
            parts.append(f'<tr><td>{e(cat)}</td>')
            for m in models:
                hit, n = cats[cat].get(m, [0, 0])
                cell = (f'{hit}/{n}' if n else '—')
                cls = "n" + (" bad" if n and hit * 2 < n else "")
                parts.append(f'<td class="{cls}">{cell}</td>')
            parts.append('</tr>')
        parts.append('</tbody></table></div></section>')

    # ---- per-tick transcripts
    for d, sample in ticks:
        tick = sample["tick"]
        qs = build_ops_questions(sample)
        if not qs:
            continue
        rel = os.path.basename(d) + "/frame.jpg"
        src = rel
        if embed:
            with open(os.path.join(d, "frame.jpg"), "rb") as fh:
                src = "data:image/jpeg;base64," + base64.b64encode(
                    fh.read()).decode()
        held = blocked_robots(sample.get("status") or {})
        jobs = ((sample.get("status") or {}).get("jobs") or {})
        meta = [f'{sample["truth"].get("robot_count", "?")} robots',
                e(sample.get("t_iso") or "")]
        if jobs:
            meta.append(f'{jobs.get("done", "?")}/{jobs.get("total", "?")} '
                        f'jobs done')
        if held:
            meta.append('held: ' + e(", ".join(f"{k}&larr;{v}"
                                               for k, v in sorted(held.items()))))
        parts.append(
            f'<section class="card tick"><h2>Tick {tick}</h2>'
            f'<p class="note">{" &middot; ".join(meta)}</p>'
            f'<div class="split"><a href="{e(src)}" target="_blank">'
            f'<img src="{e(src)}" alt="frame at tick {tick}" loading="lazy">'
            f'</a><div class="qs">')
        for q in qs:
            answers = by.get((tick, q["id"]), {})
            exp = q["expected"]
            parts.append(f'<div class="q"><div class="qh">'
                         f'<span class="qid">{e(q["id"])}</span>'
                         + (f'<span class="exp">truth: {e(str(exp))}</span>'
                            if exp is not None else
                            '<span class="exp dim">not graded</span>')
                         + '</div>')
            for m in models:
                r = answers.get(m)
                if r is None:
                    parts.append(f'<div class="a"><div class="am">{e(m)}'
                                 f'</div><div class="at dim">not asked</div>'
                                 f'</div>')
                    continue
                badge = ('<span class="b none">—</span>'
                         if r.get("correct") is None else
                         ('<span class="b ok">correct</span>'
                          if r["correct"] else
                          '<span class="b no">wrong</span>'))
                slow = ' bad' if r["seconds"] > 5.0 else ''
                body = r.get("answer")
                if body is None:
                    body = (f'[no answer: {r.get("error")}]'
                            + (f'\n\nthinking buffer held:\n'
                               + (r.get("thinking_text") or "")
                               if r.get("thinking_text") else ""))
                parts.append(
                    f'<div class="a"><div class="am">{e(m)} {badge}'
                    f'<span class="lat{slow}">{r["seconds"]:.2f}s</span>'
                    f'</div><div class="at">{e(body)}</div></div>')
            parts.append('</div>')
        parts.append('</div></div></section>')

    parts.append('</main></body>')
    return "".join(parts)


def _esc(text) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


_HEAD = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VLM live run {run}</title>
<style>
:root{
  --ink:#16181d; --dim:#6b7280; --line:#dfe3ea; --bg:#f6f7f9; --card:#fff;
  --ok:#1a7f4b; --no:#b4232b; --accent:#1f4d8f;
}
@media (prefers-color-scheme:dark){:root{
  --ink:#e7e9ee; --dim:#9aa3b2; --line:#2c313a; --bg:#14161a; --card:#1b1e24;
  --ok:#54c98a; --no:#ef7f86; --accent:#7ba7e8;
}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 ui-sans-serif,system-ui,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding-block:28px}
body{padding:0 16px}
h1{font-size:22px;margin:0 0 4px}
h2{font-size:15px;margin:0 0 10px;letter-spacing:.02em;text-transform:uppercase;
  color:var(--dim)}
.sub{color:var(--dim);margin:0 0 22px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;
  padding:18px;margin-bottom:18px}
.scroll{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--line)}
th{font-size:12px;color:var(--dim);font-weight:600}
td.n,th.n{text-align:right}
.model{font-family:ui-monospace,SFMono-Regular,Consolas,monospace}
.dim{color:var(--dim)}
.bad{color:var(--no)}
.note{color:var(--dim);font-size:13px;margin:10px 0 0}
.split{display:grid;grid-template-columns:minmax(0,340px) minmax(0,1fr);gap:18px}
@media (max-width:760px){.split{grid-template-columns:1fr}}
.split img{width:100%;border-radius:8px;border:1px solid var(--line);display:block}
.qs{display:flex;flex-direction:column;gap:14px;min-width:0}
.q{border-left:2px solid var(--line);padding-left:12px}
.qh{display:flex;flex-wrap:wrap;gap:10px;align-items:baseline;margin-bottom:6px}
.qid{font-family:ui-monospace,Consolas,monospace;font-size:12px;
  color:var(--accent)}
.exp{font-size:12px;color:var(--dim)}
.a{margin:6px 0}
.am{display:flex;flex-wrap:wrap;gap:8px;align-items:baseline;
  font-family:ui-monospace,Consolas,monospace;font-size:12px;color:var(--dim)}
.at{white-space:pre-wrap;overflow-wrap:anywhere;margin-top:2px}
.b{font-size:11px;padding:0 6px;border-radius:9px;border:1px solid var(--line)}
.b.ok{color:var(--ok);border-color:var(--ok)}
.b.no{color:var(--no);border-color:var(--no)}
.lat{margin-left:auto;font-variant-numeric:tabular-nums}
</style></head><body><main>
<h1>VLM live run {run}</h1>
<p class="sub">One capture, replayed to each model with the model resident.
Identical frames and identical state, so nothing here is an ordering
effect.</p>
"""


# ---------------------------------------------------------------------- main

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Live operational VLM comparison on the AGV testbed.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="see the module docstring for the experimental design")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="record ticks during a live fleet run")
    c.add_argument("--run", required=True, help="run name, e.g. r1")
    c.add_argument("--stream", required=True,
                   help="frame URL, e.g. http://127.0.0.1:8081/full.jpg")
    c.add_argument("--rosbridge", default="192.168.0.212:9090",
                   metavar="HOST[:PORT]")
    c.add_argument("--robots", nargs="+",
                   default=["Alvik1", "Alvik2", "Alvik3", "Alvik4", "Alvik5",
                            "Alvik6"])
    c.add_argument("--interval", type=float, default=20.0,
                   help="seconds between ticks (default 20: comfortably "
                        "longer than any candidate's inference time, and long "
                        "enough that the scene actually changes)")
    c.add_argument("--ticks", type=int, default=0,
                   help="stop after N ticks (0 = until ctrl-c)")
    c.add_argument("--min-robots", type=int, default=1,
                   help="skip a tick with fewer robots than this in view")
    c.add_argument("--max-pose-age", type=float, default=1.0)
    c.add_argument("--allow-no-status", action="store_true",
                   help="capture even without /fleet_status; the plan-state "
                        "questions cannot be graded without it")
    c.set_defaults(fn=cmd_capture)

    a = sub.add_parser("ask", help="one model, one pass, model resident")
    a.add_argument("--run", required=True)
    a.add_argument("--model", required=True,
                   help="qwen3.5:4b, or model@http://host:11434")
    a.add_argument("--url", default=DEFAULT_URL)
    a.add_argument("--num-ctx", type=int, default=None,
                   help="Ollama context window; leave unset unless a model "
                        "spills to CPU")
    a.add_argument("--num-predict", type=int, default=None,
                   help="cap on OUTPUT tokens. Unset means unbounded, and a "
                        "reasoning model will run to the context ceiling: "
                        "measured 2026-09-15, qwen3-vl:4b spent ~4900 tokens "
                        "and 60s on EVERY question and left all of them in "
                        "the thinking buffer. num_ctx does not bound this and "
                        "neither does think:false -- this is the only knob "
                        "that does. 512 is the working default (see the note "
                        "in cmd_ask).")
    a.add_argument("--no-image", action="store_true",
                   help="state-only control: how much does the frame add?")
    a.set_defaults(fn=cmd_ask)

    r = sub.add_parser("report", help="readable side-by-side HTML")
    r.add_argument("--run", required=True)
    r.add_argument("--out", default=None)
    r.add_argument("--state-only", action="store_true",
                   help="report the no-image control passes instead of the "
                        "frame+state passes; the two are never mixed into one "
                        "table")
    r.add_argument("--embed", action="store_true",
                   help="inline the frames so the file travels on its own "
                        "(larger, but one file to send)")
    r.set_defaults(fn=cmd_report)

    args = ap.parse_args()
    if args.cmd == "capture":
        host, _, port = args.rosbridge.partition(":")
        args.rosbridge_host = host
        args.rosbridge_port = int(port or 9090)
    args.fn(args)


if __name__ == "__main__":
    main()
