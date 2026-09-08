#!/usr/bin/env python3
"""Score several local VLMs against captured testbed frames.

WHAT THIS ANSWERS
-----------------
"Which model actually understands the state of the system" -- measured, not
eyeballed. Every question is generated from a sample's measured ground truth
(vlm_capture.py), so each answer is right or wrong automatically.

Three things get measured, and the second two matter as much as the first:

  ACCURACY    per question category, so a model that counts well but cannot
              reason spatially is visible as exactly that rather than an
              averaged score.
  LATENCY     a 9B that is right and takes 6s is a different tool from a 4B
              that is right and takes 1s.
  CONSISTENCY the same question, same image, asked --repeats times. A model
              that answers correctly half the time at random is useless for
              anything that acts on the answer, and a single-shot benchmark
              cannot tell that apart from competence.

HALLUCINATION CONTROL
---------------------
Every run includes negative-control questions whose true answer is "no" or
"none" -- a robot that is not there, a region that is empty. Models that
agree with whatever they are asked score well on ordinary questions and fail
these, which is precisely the failure that would matter here.

WHAT IS SENT
------------
The annotated frame plus the measured state as TEXT. The model is given
coordinates rather than asked to produce them: partly because that is the
real deployment shape (the system knows where its robots are), and partly
because coordinate grounding is broken in Ollama's GGUF path for at least
one major family -- see llama.cpp issue #17131.

USAGE
-----
    python3 vlm_bench.py --models qwen3.5:4b minicpm-v4.5:8b \\
        --samples-dir vlm_samples --repeats 3

    python3 vlm_bench.py --report results.json      # re-print a comparison
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import statistics
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone

DEFAULT_URL = "http://192.168.0.162:11434"
CARDINAL_WORDS = ("north", "south", "east", "west")


# ---------------------------------------------------------------- questions

def _spread(items: list, cap: int) -> list:
    """Up to `cap` items, evenly spaced across the list rather than the first
    few. Taking the head kept picking the same alphabetically-first robots
    every sample, so one robot dominated the orientation questions and the
    rest were never asked about."""
    if cap is None or cap <= 0 or len(items) <= cap:
        return items
    step = len(items) / cap
    return [items[int(i * step)] for i in range(cap)]


def build_questions(truth: dict, max_per_category: int | None = None
                    ) -> list[dict]:
    """Generate every question this sample can score automatically.

    Each entry: {id, category, prompt, expect, check} where check(answer)
    returns True/False. Questions the sample cannot answer unambiguously are
    simply not generated -- see vlm_capture.truth_for()."""
    qs: list[dict] = []
    robots = sorted(truth.get("robots") or {})

    def num_in(text: str):
        r"""First number, but only if it is a whole one.

        A bare r"-?\d+" match read "3.5" as 3 and scored it correct against
        an expected count of 3. A count is an integer; a fractional answer is
        wrong, not rounded."""
        m = re.search(r"-?\d+(?:\.\d+)?", text.replace(",", ""))
        if not m:
            return None
        v = float(m.group())
        return int(v) if v.is_integer() else None

    def names_in(text: str) -> list[str]:
        """Robot names in ORDER OF APPEARANCE, word-boundary matched.

        Word boundaries because a plain substring test makes "Alvik1" match
        inside "Alvik10". Order because a correct answer is often phrased
        "Alvik1, not Alvik2" -- requiring the mentioned set to be exactly
        {Alvik1} would mark that wrong, which measures terseness rather than
        understanding. The checks below take the first name(s) instead."""
        hits = []
        for r in robots:
            m = re.search(rf"\b{re.escape(r)}\b", text, re.I)
            if m and not _negated_before(text, m.start()):
                hits.append((m.start(), r))
        return [r for _, r in sorted(hits)]

    # -- counting -----------------------------------------------------------
    qs.append({
        "id": "count", "category": "counting",
        "prompt": "How many robots are on the table? "
                  "Answer with a single number and nothing else.",
        "expect": truth["robot_count"],
        "check": lambda a: num_in(a) == truth["robot_count"],
    })

    # -- orientation --------------------------------------------------------
    for r, facing in _spread(sorted((truth.get("facing") or {}).items()),
                             max_per_category):
        qs.append({
            "id": f"facing:{r}", "category": "orientation",
            "prompt": f"Which direction is {r} facing? Answer with exactly "
                      "one word: north, south, east, or west.",
            "expect": facing,
            "check": (lambda a, f=facing: _single_cardinal(a) == f),
        })

    # -- spatial extremes ---------------------------------------------------
    # Only ask when the extreme is CLEAR. facing/ and north_of already refuse
    # coin flips (30deg from a cardinal, 6in of separation); extremes did not,
    # so two robots 0.3in apart in x produced a "which is furthest east"
    # question with a definite truth and no visually answerable answer. That
    # measures luck, not understanding.
    EXTREME_MARGIN_IN = 4.0
    axis_of = {"east": ("x_in", 1), "west": ("x_in", -1),
               "north": ("y_in", 1), "south": ("y_in", -1)}
    for direction, who in _spread(
            sorted((truth.get("extremes") or {}).items()), max_per_category):
        if len(robots) < 2:
            continue
        axis, sign = axis_of.get(direction, (None, 1))
        if axis:
            vals = sorted((sign * float(truth["robots"][r][axis])
                           for r in robots), reverse=True)
            if len(vals) >= 2 and (vals[0] - vals[1]) < EXTREME_MARGIN_IN:
                continue
        qs.append({
            "id": f"extreme:{direction}", "category": "spatial",
            "prompt": f"Which robot is furthest {direction}? Answer with "
                      "just the robot name.",
            "expect": who,
            "check": (lambda a, w=who: names_in(a)[:1] == [w]),
        })

    # -- pairwise relations -------------------------------------------------
    for key, is_north in _spread(
            sorted((truth.get("north_of") or {}).items()),
            max_per_category if max_per_category else 4):
        a_name, b_name = key.split("|")
        qs.append({
            "id": f"northof:{key}", "category": "relation",
            "prompt": f"Is {a_name} north of {b_name}? Answer yes or no.",
            "expect": "yes" if is_north else "no",
            "check": (lambda a, e=is_north: _yesno(a) is not None
                      and _yesno(a) == e),
        })

    # -- closest pair -------------------------------------------------------
    cp = truth.get("closest_pair")
    if cp and len(robots) >= 3:
        want = sorted(cp["robots"])
        qs.append({
            "id": "closest", "category": "spatial",
            "prompt": "Which two robots are closest together? Answer with "
                      "just the two robot names.",
            "expect": want,
            "check": (lambda a, w=want: sorted(names_in(a)[:2]) == w),
        })

    # -- controls, BALANCED --------------------------------------------------
    # Two of these expect "no" and two expect "yes". With all-negative
    # controls a model that simply always answers "no" scored 100% on the
    # category that exists to catch exactly that behaviour.
    absent = _absent_name(robots)
    qs.append({
        "id": "ctl:absent-robot", "category": "control",
        "prompt": f"Is {absent} among the tracked robots? Answer yes or no.",
        "expect": "no",
        "check": lambda a: _yesno(a) is False,
    })
    qs.append({
        "id": "ctl:count-wrong", "category": "control",
        "prompt": f"Are there exactly {truth['robot_count'] + 3} tracked "
                  "robots? Answer yes or no.",
        "expect": "no",
        "check": lambda a: _yesno(a) is False,
    })
    if robots:
        qs.append({
            "id": "ctl:present-robot", "category": "control",
            "prompt": f"Is {robots[0]} among the tracked robots? "
                      "Answer yes or no.",
            "expect": "yes",
            "check": lambda a: _yesno(a) is True,
        })
    qs.append({
        "id": "ctl:count-right", "category": "control",
        "prompt": f"Are there exactly {truth['robot_count']} tracked robots? "
                  "Answer yes or no.",
        "expect": "yes",
        "check": lambda a: _yesno(a) is True,
    })
    return qs


# Words that flip the meaning of whatever follows them. Checked in the
# short window before a match, because "not north" and "Alvik9, not Alvik2"
# were both being scored CORRECT -- the parser saw the right token and never
# noticed it was being denied. Confirmed against the live grader 2026-09-08.
_NEGATIONS = ("not", "isn't", "is not", "aren't", "no longer", "never",
              "rather than", "instead of", "n't")


def _negated_before(text: str, idx: int, window: int = 24) -> bool:
    """True if a negation appears close before position idx."""
    prefix = text[max(0, idx - window):idx].lower()
    return any(w in prefix for w in _NEGATIONS)


def _single_cardinal(answer: str):
    """The cardinal named, or None if the answer names several or none.

    Accepts "northern"/"northward" as north: the thing being measured is
    whether the model knows the direction, not whether it obeyed "answer in
    one word". Deliberately does NOT accept "northeast" -- that is not one
    of the four options, and the suffix-and-boundary pattern below makes it
    match nothing rather than silently counting as "north".

    Several cardinals means a hedge ("north or south"), which must score
    wrong: accepting it would credit a coin flip as understanding."""
    low = answer.lower()
    found = []
    for w in CARDINAL_WORDS:
        m = re.search(rf"\b{w}(ern|ward|wards)?\b", low)
        if m and not _negated_before(low, m.start()):
            found.append(w)
    return found[0] if len(found) == 1 else None


def _yesno(answer: str):
    low = answer.lower()
    yes = re.search(r"\b(yes|correct|true)\b", low) is not None
    no = re.search(r"\b(no|not|incorrect|false)\b", low) is not None
    if yes == no:
        return None          # both or neither -> unscoreable, counted wrong
    return yes


def _absent_name(robots: list[str]) -> str:
    for cand in ("Alvik6", "Alvik5", "Alvik4", "Alvik9"):
        if cand not in robots:
            return cand
    return "Alvik9"


def state_text(truth: dict) -> str:
    """Measured state as prose. The model is TOLD positions rather than
    asked to extract them -- see this module's docstring."""
    lines = ["Measured system state (from overhead AprilTag tracking):",
             f"- robots tracked: {truth['robot_count']}"]
    for name, p in sorted((truth.get("robots") or {}).items()):
        lines.append(f"- {name}: x={p['x_in']}in, y={p['y_in']}in, "
                     f"yaw={p['yaw_deg']}deg")
    lines.append("Coordinate frame: +x is east (right), +y is north (up in "
                 "the image). Yaw 0=south, 90=east, 180=north, 270=west.")
    return "\n".join(lines)


# ---------------------------------------------------------------- ollama

def ask(url: str, model: str, prompt: str, image_b64: str | None,
        timeout: float) -> tuple[str | None, float, str | None]:
    """One question. image_b64 None runs the state-only control condition."""
    msg: dict = {"role": "user", "content": prompt}
    if image_b64 is not None:
        msg["images"] = [image_b64]
    body = json.dumps({
        "model": model,
        "messages": [msg],
        "stream": False,
        "keep_alive": "30m",
        # Reasoning OFF. qwen3.5 is a thinking model: measured on this
        # testbed it spent ~2300-3500 tokens of chain-of-thought to answer
        # "how many robots" with the single character "6" -- 76s per
        # question against 7s with thinking disabled, and at the longer
        # state-context prompt it sometimes exhausted its budget mid-think
        # and returned EMPTY content, which then scored as a wrong answer.
        # A 10x speedup and the same answer. Models that ignore this field
        # are unaffected; any thinking they emit anyway is captured below.
        "think": False,
        # Deterministic: consistency is being MEASURED, so sampling noise
        # must not be mistaken for it. Any variation left is the model's.
        "options": {"temperature": 0.0, "seed": 42},
    }).encode()
    req = urllib.request.Request(f"{url}/api/chat", data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read())
        dt = time.monotonic() - t0
        msg = data.get("message") or {}
        content = (msg.get("content") or "").strip()
        if not content and msg.get("thinking"):
            # Answer never made it out of the reasoning buffer. Scoring that
            # as simply "wrong" would blame the model for a budget problem,
            # so it is surfaced as its own error instead.
            return None, dt, "empty content (answer left in thinking buffer)"
        return content, dt, None
    except Exception as exc:
        return None, time.monotonic() - t0, repr(exc)


def check_models(url: str, models: list[str]) -> None:
    try:
        with urllib.request.urlopen(f"{url}/api/tags", timeout=10) as resp:
            tags = json.loads(resp.read())
    except Exception as exc:
        raise SystemExit(f"cannot reach Ollama at {url}: {exc}")
    have = [m.get("name", "") for m in tags.get("models", [])]
    missing = [m for m in models
               if not any(h == m or h.startswith(m + ":") for h in have)]
    if missing:
        raise SystemExit(
            "not pulled on the Ollama server:\n  "
            + "\n  ".join(f"{m}   (ollama pull {m})" for m in missing)
            + f"\n\navailable: {', '.join(sorted(have)) or '(none)'}")


# ---------------------------------------------------------------- reporting

def report(results: dict) -> None:
    models = results["models"]
    rows = results["rows"]
    cats = sorted({r["category"] for r in rows})

    print()
    print("=" * 78)
    print(f"condition: {results.get('condition', 'state+image')}")
    print(f"VLM comparison — {results['sample_count']} sample(s), "
          f"{results['repeats']} repeat(s), "
          f"{len(rows) // max(len(models), 1)} question-instances per model")
    print("=" * 78)

    print(f"\n{'model':<22}{'overall':>17}{'latency':>10}{'consist':>9}"
          f"{'errors':>8}")
    print("-" * 66)
    for m in models:
        mine = [r for r in rows if r["model"] == m]
        ok = [r for r in mine if r["correct"]]
        lat = [r["seconds"] for r in mine if r["error"] is None]
        errs = [r for r in mine if r["error"] is not None]
        cons = _consistency(mine)
        print(f"{m:<22}{_pct(len(ok), len(mine)):>17}"
              f"{(f'{statistics.median(lat):.2f}s' if lat else '—'):>10}"
              f"{(f'{cons:.0%}' if cons is not None else '—'):>9}"
              f"{len(errs):>8}")

    _category_matrix(models, rows, cats)

    print("\ncontrols (balanced yes/no -- catches a model that just agrees,"
          "\nor just disagrees, with whatever it is asked):")
    for m in models:
        mine = [r for r in rows if r["model"] == m
                and r["category"] == "control"]
        ok = [r for r in mine if r["correct"]]
        print(f"  {m:<24}{_pct(len(ok), len(mine))}")

    # A model answering the SAME thing every time can post a respectable
    # accuracy purely from the base rate of that answer being right --
    # qwen3.5:2b said "north" to 16/16 orientation questions and scored 31%,
    # minicpm said "west" to 12/16 and scored 6%. Nothing in the percentages
    # above separates that from understanding, and it is the exact failure
    # this benchmark exists to catch, so it gets its own line.
    print("\nanswer diversity (one repeated answer = the model is not "
          "reading the\nquestion; its accuracy there is only the base rate):")
    flagged = False
    for m in models:
        for cat in cats:
            mine = [r for r in rows if r["model"] == m
                    and r["category"] == cat and r["answer"]]
            if len(mine) < 4:
                continue
            counts = {}
            for r in mine:
                k = _normalise_answer(r["answer"])
                counts[k] = counts.get(k, 0) + 1
            top, n = max(counts.items(), key=lambda kv: kv[1])
            if n / len(mine) >= 0.75:
                flagged = True
                ok = sum(1 for r in mine if r["correct"])
                print(f"  {m:<22}{cat:<13}said {top!r} for {n}/{len(mine)}"
                      f" -- scored {ok}/{len(mine)}")
    if not flagged:
        print("  (none -- every model varied its answers in every category)")

    _cardinal_breakdown(models, rows)
    print()


def _majority_truth(rows: list[dict]) -> tuple[str, int]:
    """The most common CORRECT answer in a set of question-instances.

    A model that always gives this one answer scores exactly its share while
    understanding nothing, so that share is the floor every accuracy in the
    category has to be read against. The 2026-09-08 run is the cautionary
    case: all three models scored 100% on counting, and all three did it by
    answering "6" every time -- all 32 captured samples happened to hold six
    robots, so the floor was 100% too and the category measured nothing."""
    counts: dict[str, int] = {}
    for r in rows:
        exp = r.get("expected")
        key = (json.dumps(exp) if isinstance(exp, (list, dict, tuple))
               else str(exp))
        counts[key] = counts.get(key, 0) + 1
    if not counts:
        return "-", 0
    return max(counts.items(), key=lambda kv: kv[1])


def _category_matrix(models: list[str], rows: list[dict],
                     cats: list[str]) -> None:
    """Per-category accuracy printed over the always-same-answer floor.

    The floor is a property of the SAMPLE SET, not of any model, so it is one
    row underneath the table rather than a column repeated per model."""
    w = 12
    print()
    print(f"{'model':<22}" + "".join(f"{c[:11]:>{w}}" for c in cats))
    rule = "-" * (22 + w * len(cats))
    print(rule)
    for m in models:
        line = f"{m:<22}"
        for c in cats:
            mine = [r for r in rows if r["model"] == m and r["category"] == c]
            cell = (f"{100 * sum(1 for r in mine if r['correct']) / len(mine):.0f}%"
                    if mine else "-")
            line += f"{cell:>{w}}"
        print(line)
    print(rule)

    floor = f"{'always-same-answer':<22}"
    which = f"{'  (the answer)':<22}"
    thin = []
    for c in cats:
        mine = [r for r in rows
                if r["model"] == models[0] and r["category"] == c]
        top, n = _majority_truth(mine)
        base = n / len(mine) if mine else 0.0
        floor += f"{(f'{100 * base:.0f}%' if mine else '-'):>{w}}"
        which += f"{top[:w - 1]:>{w}}"
        best = 0.0
        for m in models:
            his = [r for r in rows if r["model"] == m and r["category"] == c]
            if his:
                best = max(best, sum(1 for r in his if r["correct"]) / len(his))
        if mine and best - base <= 0.05:
            thin.append(c)
    print(floor)
    print(which)

    if thin:
        print()
        print(f"  !! {', '.join(thin)}: the best model clears that floor by 5"
              " points or less.")
        print("     Nothing there is evidence of understanding. Capture"
              " samples with a wider")
        print("     spread of correct answers before reading those scores.")


def _cardinal_breakdown(models: list[str], rows: list[dict]) -> None:
    """Orientation accuracy split by the TRUE facing.

    One orientation percentage hides which axis a model can actually read.
    On 2026-09-08 qwen3.5:4b scored 80% overall and 0/21 on the true-west
    cases -- it answered "south" to every single one of them. It reads the
    north/south axis at about 90% and cannot separate east from west at all.
    That split, not the 80%, decides whether the VLM can be asked about
    heading."""
    orient = [r for r in rows if r["category"] == "orientation"]
    if not orient:
        return
    cards = [c for c in CARDINAL_WORDS
             if any(str(r.get("expected")) == c for r in orient)]
    if len(cards) < 2:
        return
    print()
    print("orientation by TRUE facing (a good overall score can still hide")
    print("an axis the model cannot resolve at all):")
    print(f"  {'model':<22}" + "".join(f"{c:>13}" for c in cards))
    weak = []
    for m in models:
        line = f"  {m:<22}"
        for c in cards:
            mine = [r for r in orient
                    if r["model"] == m and str(r.get("expected")) == c]
            if not mine:
                line += f"{'-':>13}"
                continue
            ok = sum(1 for r in mine if r["correct"])
            line += f"{f'{ok}/{len(mine)} {100 * ok / len(mine):.0f}%':>13}"
            if ok / len(mine) < 0.5:
                weak.append((m, c, ok, mine))
        print(line)
    for m, c, ok, mine in weak:
        said: dict[str, int] = {}
        for r in mine:
            if r.get("answer"):
                key = _normalise_answer(r["answer"])
                said[key] = said.get(key, 0) + 1
        if not said:
            continue
        top, n = max(said.items(), key=lambda kv: kv[1])
        print(f"    {m} on true {c}: {ok}/{len(mine)} -- said {top!r}"
              f" {n}/{len(mine)} of the time")



def _pct(n: int, d: int) -> str:
    return f"{n}/{d} ({n / d:.0%})" if d else "—"


def _consistency(rows: list[dict]) -> float | None:
    """Fraction of (sample, question) groups where every VALID repeat agreed.

    Failed calls are excluded, not treated as an answer. Three timeouts in a
    row used to score 100% consistency -- all three "answers" were None, the
    set had one element, and the model looked perfectly stable while having
    said nothing. Failures are reported separately in the errors column.

    Equivalent answers are normalised so "yes"/"true" and "no"/"false" are
    not counted as the model contradicting itself."""
    groups: dict[tuple, list] = {}
    for r in rows:
        if r.get("error") is not None or r.get("answer") is None:
            continue
        groups.setdefault((r["sample"], r["question_id"]), []).append(r)
    multi = [g for g in groups.values() if len(g) > 1]
    if not multi:
        return None
    stable = sum(1 for g in multi
                 if len({_normalise_answer(x["answer"]) for x in g}) == 1)
    return stable / len(multi)


def _normalise_answer(text: str) -> str:
    """Collapse equivalent phrasings so consistency measures the ANSWER, not
    the wording. yes/true and no/false are the same claim."""
    low = (text or "").strip().lower()
    yn = _yesno(low)
    if yn is True:
        return "@yes"
    if yn is False:
        return "@no"
    card = _single_cardinal(low)
    if card:
        return f"@{card}"
    return re.sub(r"[^a-z0-9]+", " ", low).strip()


# ---------------------------------------------------------------- main

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Score local VLMs on captured testbed frames.")
    ap.add_argument("--models", nargs="+",
                    help="Ollama model tags, e.g. qwen3.5:4b minicpm-v4.5:8b")
    ap.add_argument("--samples-dir", default=None)
    ap.add_argument("--url", default=os.environ.get("OLLAMA_URL", DEFAULT_URL))
    ap.add_argument("--max-per-category", type=int, default=3,
                    help="cap orientation/spatial/relation questions per "
                         "sample (default 3). Counting, closest-pair and the "
                         "four balanced controls are always included.")
    ap.add_argument("--limit", type=int, default=None,
                    help="use only the first N samples (evenly spaced across "
                         "the set, so a subset still spans the whole capture "
                         "rather than one moment of it)")
    ap.add_argument("--no-image", action="store_true",
                    help="CONTROL CONDITION: ask the same questions with the "
                         "measured state but NO image. If accuracy matches "
                         "the with-image run, the picture is contributing "
                         "nothing and a text model would do -- which is the "
                         "single most useful thing this benchmark can tell "
                         "you, and it cannot be known without running it.")
    ap.add_argument("--repeats", type=int, default=3,
                    help="times each question is asked (consistency; "
                         "default 3)")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--out", default=None, help="results JSON path")
    ap.add_argument("--report", default=None,
                    help="re-print the comparison from a results JSON and "
                         "exit")
    args = ap.parse_args()

    if args.report:
        with open(args.report, encoding="utf-8") as fh:
            report(json.load(fh))
        return
    if not args.models:
        raise SystemExit("--models is required (or use --report)")

    samples_dir = args.samples_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vlm_samples")
    sample_ids = sorted(
        d for d in os.listdir(samples_dir)
        if os.path.isfile(os.path.join(samples_dir, d, "truth.json")))
    if not sample_ids:
        raise SystemExit(f"no samples in {samples_dir} — run vlm_capture.py")
    if args.limit and args.limit < len(sample_ids):
        # Evenly spaced, not the first N: samples are timestamped in capture
        # order, so the first N would all come from the opening seconds of
        # one run and miss whatever happened later.
        step = len(sample_ids) / args.limit
        sample_ids = [sample_ids[int(i * step)] for i in range(args.limit)]
        print(f"using {len(sample_ids)} of the available samples "
              "(evenly spaced)")

    check_models(args.url, args.models)

    # Load every sample ONCE, then iterate MODEL-OUTERMOST.
    #
    # The first version looped sample -> model, which cycles all models
    # inside every sample. These three total ~11.3GB resident against ~7.5GB
    # of VRAM, so they cannot be co-resident: Ollama evicts and reloads on
    # each switch. That is samples x models loads -- 96 for a 32-sample run
    # -- at 10-35s each, which dwarfed the inference entirely.
    #
    # Model-outermost loads each model once (3 loads total) and keeps it warm
    # through every sample. Identical questions, identical scoring; only the
    # order changes.
    loaded = []
    for sid in sample_ids:
        d = os.path.join(samples_dir, sid)
        with open(os.path.join(d, "truth.json"), encoding="utf-8") as fh:
            truth = json.load(fh)
        with open(os.path.join(d, "frame.jpg"), "rb") as fh:
            image_b64 = base64.b64encode(fh.read()).decode()
        loaded.append((sid, build_questions(truth, args.max_per_category),
                       state_text(truth), image_b64))
    total = sum(len(q) for _s, q, _c, _i in loaded)
    print(f"{len(loaded)} sample(s), {total} question(s) each pass, "
          f"{len(args.models)} model(s), {args.repeats} repeat(s) "
          f"-> {total * len(args.models) * args.repeats} calls")

    rows: list[dict] = []
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"vlm_results_{datetime.now().strftime('%Y%m%d_%H%M')}.json")

    def _save(partial: bool) -> dict:
        res = {
            "generated_utc": datetime.now(timezone.utc).isoformat(
                timespec="seconds"),
            "url": args.url, "models": args.models, "repeats": args.repeats,
            "sample_count": len(loaded), "question_count": total,
            "condition": "state-only" if args.no_image else "state+image",
            "partial": partial,
            "rows": rows,
        }
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=2)
            fh.write(chr(10))
        return res

    interrupted = False
    for model in args.models:
        print(f"\n{model}")
        if interrupted:
            break
        for sid, questions, context, image_b64 in loaded:
            if interrupted:
                break
            for q in questions:
                for rep in range(args.repeats):
                    prompt = (f"{context}\n\nLook at the camera image of "
                              f"this robot testbed.\n{q['prompt']}")
                    try:
                        answer, secs, err = ask(
                            args.url, model, prompt,
                            None if args.no_image else image_b64,
                            args.timeout)
                    except KeyboardInterrupt:
                        # Everything answered so far is still real data, and
                        # a long run should not be all-or-nothing.
                        interrupted = True
                        break
                    correct = False
                    if answer is not None:
                        try:
                            correct = bool(q["check"](answer))
                        except Exception:
                            correct = False
                    rows.append({
                        "sample": sid, "model": model,
                        "question_id": q["id"], "category": q["category"],
                        "prompt": q["prompt"],
                        "expected": q["expect"] if not isinstance(
                            q["expect"], (list, dict)) else json.dumps(
                                q["expect"]),
                        "answer": answer, "correct": correct,
                        "seconds": round(secs, 3), "error": err,
                        "repeat": rep,
                    })
                    sys.stdout.write("." if correct else ("E" if err else "x"))
                    sys.stdout.flush()
            sys.stdout.write(f"  {sid}\n")

    results = _save(partial=interrupted)
    if interrupted:
        print()
        print(f"INTERRUPTED -- {len(rows)} answer(s) kept and saved.")
    report(results)
    print(f"full per-answer detail: {out}")


if __name__ == "__main__":
    main()
