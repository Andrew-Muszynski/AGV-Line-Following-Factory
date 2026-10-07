#!/usr/bin/env python3
"""Benchmark the Lambda-hosted qwen3-vl:32b model from Windows.

This is the single-model remote counterpart to vlm_bench.py. It reads captured
samples from a vlm_samples directory, sends each frame.jpg plus the measured
state from truth.json to qwen3-vl:32b, and applies the same question
generation, prompts, automatic scoring, latency measurement, consistency
checks, controls, and reporting as the original benchmark.

DEFAULT WINDOWS LAYOUT
----------------------
Place this script in:

    C:/Users/muszy/VRP/tools/

It will then read:

    C:/Users/muszy/VRP/tools/vlm_samples/<sample_id>/frame.jpg
    C:/Users/muszy/VRP/tools/vlm_samples/<sample_id>/truth.json

The sample location can be overridden with --samples-dir.

REMOTE OLLAMA CONNECTION
------------------------
Keep the SSH tunnel open in a separate PowerShell window:

    ssh -N -L 127.0.0.1:21435:127.0.0.1:11435 andrewm@10.10.48.219

The benchmark connects to http://127.0.0.1:21435 by default and to
qwen3-vl:32b unless --model says otherwise. Both remote tags are served from
one L40S: the dense 32B, and qwen3-vl:30b, a mixture-of-experts model that
activates a subset of its parameters per token and runs about five times
faster at comparable accuracy. This script stays remote-only so it cannot
benchmark one of the laptop models.

USAGE
-----
Quick smoke test:

    python vlm_bench_qwen3vl_remote.py --limit 2 --repeats 1

Faster eight-sample screening run:

    python vlm_bench_qwen3vl_remote.py --limit 8 --repeats 1 \
        --max-per-category 1

Full benchmark:

    python vlm_bench_qwen3vl_remote.py --repeats 3

Explicit sample directory:

    python vlm_bench_qwen3vl_remote.py \
        --samples-dir C:/Users/muszy/VRP/tools/vlm_samples

Reprint an existing result:

    python vlm_bench_qwen3vl_remote.py --report qwen3_vl_32b_results.json
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

DEFAULT_URL = "http://127.0.0.1:21435"
MODEL = "qwen3-vl:32b"
CARDINAL_WORDS = ("north", "south", "east", "west")


# ---------------------------------------------------------------- questions

def _phantom_names(text: str, robots: list) -> bool:
    """True if the answer names a robot that is not in this scene.

    Crediting "Alvik9, Alvik6" for a truth of Alvik6 rewards an answer that
    invented a robot. names_in() cannot see this by construction -- it
    searches only for robots that exist -- so the check has to look for the
    naming pattern itself.
    """
    known = {r.lower() for r in robots}
    for m in re.finditer(r"[Aa]lvik\d+", text or ""):
        if m.group().lower() not in known:
            return True
    return False


def _spread(items: list, cap: int, offset: int = 0) -> list:
    """Up to `cap` items, evenly spaced, ROTATED by a per-sample offset.

    Even spacing alone is not enough and this was measured, not theorised:
    with 6 robots and a cap of 3 it selects indices 0/2/4 in every sample
    forever. Across 159 captured samples that meant facing was asked about
    Alvik1 156 times, Alvik3 132, Alvik5 115, Alvik2 47, Alvik4 14, and
    Alvik6 exactly zero times -- so a reported "facing 65%" described three
    robots, not six. extreme:west was likewise never generated at all,
    leaving "spatial" covering three of four directions.

    The offset is derived from the sample id, so selection stays fully
    deterministic and reproducible for a given capture set while coverage
    evens out across it.
    """
    if cap is None or cap <= 0 or len(items) <= cap:
        return items
    n = len(items)
    start = offset % n
    rotated = items[start:] + items[:start]
    step = n / cap
    return [rotated[int(i * step)] for i in range(cap)]


def _sample_offset(truth: dict) -> int:
    """A stable small integer per sample, for rotating _spread()."""
    sid = str(truth.get("sample_id") or truth.get("captured_utc") or "")
    return sum(ord(c) for c in sid)


# Words that mean "the empty set" in answer to a "which robots..." question.
# An empty NAME list is not by itself evidence of a correct "none": "I cannot
# tell from this image" also contains no names, and scoring that as correct
# would credit a refusal as an answer.
_NONE_WORDS = ("none", "no robot", "nobody", "no one", "not any", "zero",
               "empty")


def _says_none(text: str) -> bool:
    low = (text or "").lower()
    return any(w in low for w in _NONE_WORDS)


def _final_answer(text: str) -> str:
    """The text after the last ANSWER: marker, or the whole reply without one.

    Occupancy questions ask the model to reason and then commit to a verdict
    on a final line. Parsing only that line means _yesno never has to decide
    which "no" inside a 1200-character explanation was the actual answer --
    the failure that made ws:any unscoreable for 11 of 12 questions on
    2026-09-08.

    Falling back to the whole reply matters: a model that ignores the format
    should be scored on what it said, not given zero for disobedience.
    """
    body = text or ""
    marker = body.lower().rfind("answer:")
    if marker == -1:
        return body
    tail = body[marker + len("answer:"):]
    cut = tail.find(chr(10))
    return tail if cut == -1 else tail[:cut]


def _template_of(qid: str) -> str:
    """Question id with its per-robot part stripped: the unit diversity is
    actually meaningful at.

    ctl:* collapses to one template deliberately -- telling the four control
    questions apart IS the skill being tested, so a model answering them all
    the same way should be flagged, not excused.
    """
    for prefix, name in (("depot:is:", "depot:is"), ("facing:", "facing"),
                         ("northof:", "northof"), ("ctl:", "ctl"),
                         ("yaw:", "yaw"), ("pos:", "pos")):
        if qid.startswith(prefix):
            return name
    return qid          # count, closest, extreme:<dir>, depot:which, ws:*


def build_questions(truth: dict, max_per_category: int | None = None
                    ) -> list[dict]:
    """Generate every question this sample can score automatically.

    Each entry: {id, category, prompt, expect, check} where check(answer)
    returns True/False. Questions the sample cannot answer unambiguously are
    simply not generated -- see vlm_capture.truth_for()."""
    qs: list[dict] = []
    # A sample whose truth disagrees with its own picture scores nothing.
    # backfill_truth.py flags vision dropouts -- frames where a robot was on
    # the table but not tracked -- and every question here keys off the robot
    # set, so one dropout would corrupt counting, extremes, closest-pair and
    # occupancy together.
    if truth.get("suspect"):
        return qs
    robots = sorted(truth.get("robots") or {})

    def num_in(text: str):
        r"""First number, but only if it is a whole one.

        A bare r"-?\d+" match read "3.5" as 3 and scored it correct against
        an expected count of 3. A count is an integer; a fractional answer is
        wrong, not rounded."""
        # Strip robot names first: "Alvik6 is the only robot." must not
        # read as a count of 6. Reproduced from the 2026-09-09 review.
        cleaned = re.sub(r"[A-Za-z]+\d+", " ", text.replace(",", ""))
        m = re.search(r"-?\d+(?:\.\d+)?", cleaned)
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

    def verdict_names(text: str, count: int | None = None):
        """(chunk, names) from the CONCLUSION of an answer, not all of it.

        A model that reasons aloud names every robot it considered before
        committing to one. Matching the whole transcript then sees a hedge
        where there is a clear verdict. Measured 2026-09-10: the
        closest-pair check scored 0/36 on answers ending "Alvik2, Alvik3"
        because the body said "closer to each other than either is to
        Alvik1", and the extremes lost points the same way.

        count=None takes the first chunk naming anything; count=n takes the
        first naming exactly n distinct robots, which keeps the property
        that a pair question wants a PAIR and three names is a different
        answer rather than a verbose right one."""
        for chunk in _verdict_chunks(text):
            found = names_in(chunk)
            if not found:
                continue
            if count is None or len(set(found)) == count:
                return chunk, found
        return "", []

    def extreme_ok(a: str, w: str) -> bool:
        chunk, found = verdict_names(a)
        return bool(found) and not _phantom_names(chunk, robots) \
            and found[0] == w

    def closest_ok(a: str, w: list) -> bool:
        chunk, found = verdict_names(a, count=2)
        return bool(found) and not _phantom_names(chunk, robots) \
            and sorted(set(found)) == w

    # -- counting -----------------------------------------------------------
    qs.append({
        "id": "count", "category": "counting",
        "prompt": "How many robots are on the table? "
                  "Answer with a single number and nothing else.",
        "expect": truth["robot_count"],
        "check": lambda a: num_in(a) == truth["robot_count"],
    })

    # -- orientation --------------------------------------------------------
    # -- coordinate retrieval: the control beneath every derived question ---
    # Every other question here is DERIVED from numbers the prompt already
    # supplies: facing is a yaw lookup, extremes compare x or y, occupancy is
    # a threshold test. So a wrong answer has two possible causes -- the model
    # could not read the number, or could not do the thing with it -- and
    # until now nothing separated them.
    #
    # These ask for the raw value. With the state text present that is pure
    # retrieval and should sit near 100%; anything lower means every derived
    # score above it is measuring transcription, not reasoning. Under
    # --no-image they are the retrieval floor; with an image and no state text
    # they would instead test whether the model can read the pupil-apriltags
    # HUD, which is the reason /full.jpg exists and is not measured yet.
    #
    # Scored with a tolerance, not exact match: 5 degrees and 2 inches are
    # well inside the 4in workstation and 3in depot thresholds, so a pass here
    # guarantees the derived questions had usable inputs.
    for r in _spread(robots, max_per_category, _sample_offset(truth)):
        p = (truth.get("robots") or {})[r]
        yaw = round(float(p["yaw_deg"]) % 360.0, 2)
        rx, ry = float(p["x_in"]), float(p["y_in"])
        qs.append({
            "id": f"yaw:{r}", "category": "retrieval",
            "prompt": f"What is {r}'s yaw in degrees? Answer with a single "
                      "number.",
            "expect": yaw,
            "check": (lambda a, v=yaw: _yaw_within(a, v, 5.0)),
        })
        qs.append({
            "id": f"pos:{r}", "category": "retrieval",
            "prompt": f"What are {r}'s x and y coordinates in inches? "
                      "Answer with two numbers: x, y",
            "expect": [rx, ry],
            "check": (lambda a, ax=rx, ay=ry: _xy_within(a, ax, ay, 2.0)),
        })

    for r, facing in _spread(sorted((truth.get("facing") or {}).items()),
                             max_per_category, _sample_offset(truth)):
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
            sorted((truth.get("extremes") or {}).items()),
            max_per_category, _sample_offset(truth)):
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
            # A hallucinated robot invalidates the answer. names_in()
            # only finds robots that exist in the scene, so "Alvik9,
            # Alvik6" used to score correct with the phantom silently
            # dropped. Reproduced from the 2026-09-09 review.
            "check": (lambda a, w=who: extreme_ok(a, w)),
        })

    # -- pairwise relations -------------------------------------------------
    for key, is_north in _spread(
            sorted((truth.get("north_of") or {}).items()),
            max_per_category if max_per_category else 4,
            _sample_offset(truth)):
        a_name, b_name = key.split("|")
        qs.append({
            "id": f"northof:{key}", "category": "relation",
            "prompt": f"Is {a_name} north of {b_name}? Answer yes or no.",
            "expect": "yes" if is_north else "no",
            "check": (lambda a, e=is_north: _yesno(a) is not None
                      and _yesno(a) == e),
        })

    # -- closest pair -------------------------------------------------------
    # Only ask when the winning pair is CLEARLY closest. facing refuses a yaw
    # within 30deg of two cardinals, extremes refuse a 4in margin, north_of
    # refuses 6in of separation -- closest_pair had no such guard, and it was
    # the only unguarded question left.
    #
    # Measured 2026-09-10 on the 40-sample set: 15 of 36 closest questions had
    # a runner-up within 2in, and the tightest five were 0.1in apart --
    # 6.2in versus 6.3in, which is exactly the depot slot pitch. Robots parked
    # in adjacent slots are equidistant by construction, so those questions
    # scored a coin flip and called it spatial reasoning.
    #
    # The gap is recomputed from robots here rather than stored by
    # vlm_capture, so all 182 already-captured samples get the guard without
    # re-capture and old runs can be rescored under it.
    CLOSEST_MARGIN_IN = 4.0
    cp = truth.get("closest_pair")
    if cp and len(robots) >= 3:
        pos = truth.get("robots") or {}
        gaps = sorted(((pos[a]["x_in"] - pos[b]["x_in"]) ** 2
                       + (pos[a]["y_in"] - pos[b]["y_in"]) ** 2) ** 0.5
                      for i, a in enumerate(robots) for b in robots[i + 1:]
                      if a in pos and b in pos)
        clear = len(gaps) >= 2 and (gaps[1] - gaps[0]) >= CLOSEST_MARGIN_IN
    else:
        clear = False
    if cp and len(robots) >= 3 and clear:
        want = sorted(cp["robots"])
        qs.append({
            "id": "closest", "category": "spatial",
            "prompt": "Which two robots are closest together? Answer with "
                      "just the two robot names.",
            "expect": want,
            # Exactly two, and no phantoms: the question asks for a
            # PAIR, so three names is a different (wrong) answer, not a
            # verbose right one. Reproduced from the 2026-09-09 review.
            "check": (lambda a, w=want: closest_ok(a, w)),
        })

    # -- occupancy: depot and workstation ------------------------------------
    # Replaces "how many robots are active". Activity is a temporal property
    # and a single still frame cannot show it; "parked in the depot" is
    # decidable from one picture. Truth comes from backfill_truth.py and
    # vlm_capture.py, computed against the measured bay and depot tables in
    # apriltag_localize.py, which refuse the ambiguous band -- so a sample
    # with a robot sitting between states is not asked about rather than
    # guessed at.
    #
    # Measured 2026-09-08: on the first 32 captures in_depot has ONE distinct
    # value across every sample, so these questions can only start measuring
    # something once the capture set contains robots that actually leave and
    # return. The floor row in the report says so on its own.
    DEPOT_HINT = ("The depot is the row of parking slots along the south "
                  "(bottom) edge of the table, where robots wait when they "
                  "are not running a job. ")
    WS_HINT = ("A robot is at a workstation when it is stopped at one of the "
               "workstation bays, rather than in the depot or partway along "
               "the grid. ")

    # Reason first, then commit to a verdict on its own final line. The
    # earlier "answer yes or no, then stop" made qwen3.5:9b answer "no" to
    # all 64 depot:is questions; allowed to reason it had 54 of 56 scoreable
    # answers right. Suppressing the reasoning to help the parser destroyed
    # the accuracy being measured, so the parser is made to cope instead.
    YESNO_TAIL = ("Think it through if you need to, then end your reply with "
                  "a final line of exactly: ANSWER: yes   or   ANSWER: no")
    LIST_TAIL = ("Think it through if you need to, then end your reply with "
                 "a final line of exactly: ANSWER: <robot names separated by "
                 "commas>   or   ANSWER: none")


    def _set_check(expected):
        want = set(expected)

        def check(a: str) -> bool:
            verdict = _final_answer(a)
            got = set(names_in(verdict))
            if not want:
                return not got and _says_none(verdict)
            return got == want

        return check

    in_depot = truth.get("in_depot")
    if in_depot is not None and not truth.get("in_depot_unclear"):
        qs.append({
            "id": "depot:which", "category": "occupancy", "layout": True,
            "prompt": DEPOT_HINT + "Which robots are parked in the depot? "
                      + LIST_TAIL,
            "expect": in_depot,
            "check": _set_check(in_depot),
        })
        # One robot that is in and one that is not, so this pair is balanced
        # by construction rather than by whatever the capture happened to
        # contain -- the same reason the controls are balanced.
        outside = [r for r in robots if r not in in_depot]
        pairs = []
        if in_depot:
            pairs.append((in_depot[0], True))
        if outside:
            pairs.append((outside[0], False))
        for name, want in pairs:
            qs.append({
                "id": f"depot:is:{name}", "category": "occupancy", "layout": True,
                "prompt": DEPOT_HINT + f"Is {name} parked in the depot? "
                          + YESNO_TAIL,
                "expect": "yes" if want else "no",
                "check": (lambda a, w=want: _yesno(_final_answer(a)) is w),
            })

    at_ws = truth.get("at_workstation")
    if at_ws is not None and not truth.get("at_workstation_unclear"):
        qs.append({
            "id": "ws:any", "category": "occupancy", "layout": True,
            "prompt": WS_HINT + "Is any robot at a workstation right now? "
                      + YESNO_TAIL,
            "expect": "yes" if at_ws else "no",
            "check": (lambda a, w=bool(at_ws): _yesno(_final_answer(a)) is w),
        })
        qs.append({
            "id": "ws:which", "category": "occupancy", "layout": True,
            "prompt": WS_HINT + "Which robots are at a workstation? "
                      + LIST_TAIL,
            "expect": at_ws,
            "check": _set_check(at_ws),
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
    for q in qs:
        q["template"] = _template_of(q["id"])
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

    Several cardinals IN THE CONCLUSION means a hedge ("north or south"),
    which must score wrong: accepting it would credit a coin flip as
    understanding. Several across the whole answer means the model showed
    its work, which must not.

    2026-09-10: this read the whole transcript and cost a run. The yaw
    convention line now spells out all four mappings, so a model that
    reasons aloud restates all four before committing:

        - The problem states that 0 (or 360) is South, 90 is East,
          180 is North, and 270 is West.
        - A yaw of 88.76 is extremely close to 90.
        Therefore, Alvik2 is facing **east**.       <- expected east

    Four cardinals found, None returned, scored wrong. All 114 facing
    answers in vlm_results_yawfix_image were prose like this, and the run
    reported orientation at 33% against a 42% floor -- which read as a
    model that cannot do orientation at all. It was the grader, and the
    grader was penalising the prompt fix that made the model verbose."""
    def _found_in(chunk: str) -> list:
        found = []
        for w in CARDINAL_WORDS:
            m = re.search(rf"\b{w}(ern|ward|wards)?\b", chunk)
            if m and not _negated_before(chunk, m.start()):
                found.append(w)
        return found

    low = (answer or "").lower()
    # Narrowest unit that names exactly one cardinal wins: last line, then
    # last sentence, then the whole answer. If none of them does, this
    # still returns None, so a genuine hedge still scores wrong.
    lines = [ln for ln in low.splitlines() if ln.strip()]
    for chunk in ([lines[-1]] if lines else []) + [_last_sentence(low), low]:
        found = _found_in(chunk)
        if len(found) == 1:
            return found[0]
    return None


def _last_sentence(text: str) -> str:
    """Text after the last sentence break, where a verdict usually sits.

    The mirror of _first_sentence(): terse models put the verdict first,
    models that reason put it last, and this benchmark sees both.
    """
    parts = [p for p in re.split(r"[.!?\n]+", text or "") if p.strip()]
    return parts[-1] if parts else (text or "")


def _numbers_in(text: str) -> list[float]:
    """Every number in a chunk, robot names stripped first.

    "Alvik6" must not read as a 6, the same reason num_in() strips names.
    Commas become separators rather than being deleted, so "23.1, 26.3"
    yields two numbers instead of one bogus 23.126.
    """
    cleaned = re.sub(r"[A-Za-z]+\d+", " ", (text or "").replace(",", " "))
    return [float(m.group())
            for m in re.finditer(r"-?\d+(?:\.\d+)?", cleaned)]


def _yaw_within(answer: str, want: float, tol_deg: float) -> bool:
    """The answer states a yaw within tol_deg of want, compared ON THE CIRCLE.

    359 and 1 are two degrees apart, not 358. Scoring these linearly would
    manufacture exactly the wrap failure this benchmark just spent a day
    diagnosing in the prompt.

    A verdict that commits to a number and gets it wrong stops the search
    rather than falling through to wider chunks -- otherwise a reply that
    quotes the state text on its way to a wrong answer scores correct.
    """
    for chunk in _verdict_chunks(answer):
        nums = _numbers_in(chunk)
        if not nums:
            continue
        for v in nums:
            d = abs((v - want) % 360.0)
            if min(d, 360.0 - d) <= tol_deg:
                return True
        return False
    return False


def _xy_within(answer: str, wx: float, wy: float, tol_in: float) -> bool:
    """The answer states an (x, y) within tol_in of (wx, wy), x first.

    Two numbers are required: a reply naming one coordinate has not answered
    the question, and guessing which axis it meant would invent a result.
    """
    for chunk in _verdict_chunks(answer):
        nums = _numbers_in(chunk)
        if len(nums) < 2:
            continue
        return abs(nums[0] - wx) <= tol_in and abs(nums[1] - wy) <= tol_in
    return False


def _verdict_chunks(text: str) -> list[str]:
    """The answer narrowed to where a verdict sits, widest attempt last.

    Every checker here has now been bitten by the same thing: a model that
    reasons aloud walks through the option set before committing, so
    matching the full transcript sees a hedge where there is a clear
    conclusion. On 2026-09-10 this cost 63 of 114 facing answers and the
    whole closest-pair template (0/36) in a single 80-minute run, and both
    were scored wrong against answers that plainly stated the right thing.

    Order matters and was settled empirically, not by taste. Both answer
    shapes occur in the same run:

        verdict LAST   "...therefore Alvik2 is facing **east**."
        verdict FIRST  "The furthest south is **Alvik5**. Breakdown:
                        Alvik1: 10.2 ... Alvik6: 16.8"

    so last-line-first alone scores the second one as Alvik6. What both
    share is that the model BOLDS its answer, which is why bold is tried
    ahead of position. Every chunk is only accepted if it yields a
    well-formed answer, so a bad guess falls through instead of sticking.
    """
    t = text or ""
    chunks = []
    bold = re.findall(r"\*\*(.+?)\*\*", t, re.S)
    if bold:
        chunks.append(bold[-1])
        if len(bold) > 1:
            chunks.append(" ".join(bold))
    lines = [ln for ln in t.splitlines() if ln.strip()]
    if lines:
        chunks.append(lines[-1])
    for extra in (_last_sentence(t), _first_sentence(t), t):
        if extra not in chunks:
            chunks.append(extra)
    return chunks


def _first_sentence(text: str) -> str:
    """Text up to the first sentence break, where a verdict usually sits."""
    for i, ch in enumerate(text or ""):
        if ch in (".", "!", "?", chr(10)):
            return (text or "")[:i]
    return text or ""


def _yesno(answer: str):
    """yes/no, tolerating a verdict followed by an explanation.

    The whole-text scan alone returned None for "**Yes**, there are robots at
    workstations. Here is the breakdown..." because the explanation that
    followed contained "not" -- and an unscoreable answer counts as wrong, so
    a model that answered correctly and then justified itself was marked
    down. Measured on the 2026-09-08 occupancy run, that alone accounted for
    most of ws:any scoring 0/12.

    2026-09-10: first-sentence-then-whole-text was still not enough, and it
    cost two control questions in the final image run -- the questions whose
    entire job is catching a model that agrees with anything.

        "...Since the system reports only 6 tracked robots ... the correct
         answer is: **no** --- Final Answer: **no**"        expected no

    scored WRONG, because _yesno_scan's affirmative pattern includes
    "correct", so the stock phrase "the correct answer is" set the yes flag
    while "no" set the other, and both-flags means unscoreable. The verdict
    itself ("**no**") contains exactly one of the two.

    So the chunks come from _verdict_chunks() now, same as the cardinal and
    name checks: bolded text first, then last line, last sentence, first
    sentence, whole answer. A real hedge ("yes and no") still resolves to
    None in every chunk and is still counted wrong, which is the property
    this function exists to protect.
    """
    for chunk in _verdict_chunks(answer):
        verdict = _yesno_scan(chunk)
        if verdict is not None:
            return verdict
    return None


def _yesno_scan(answer: str):
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


_INCLUDE_LAYOUT = False
# "auto" | "always" | "never" -- see --layout and the per-question flag.
_LAYOUT_MODE = "auto"
_LAYOUT_CACHE: list = []


def testbed_layout() -> str:
    """The static layout block, from tools/testbed_layout.txt.

    ADDED 2026-09-09. Until now the prompt gave robot positions but never
    said where anything ELSE was, so ws:any and ws:which scored 51 and 54
    points BELOW their own base rate -- all six models answered with the
    depot set, because nothing in the prompt distinguished a workstation bay
    from a depot slot. That was a missing-information failure, not a
    perception failure.

    Generated by tools/make_layout.py from apriltag_localize.py's measured
    tables, so a recalibration flows through instead of being retyped here.
    Costs about 270 tokens against the image's ~2050 at 1920px. A missing
    file means no block rather than a crash, which keeps the bench runnable
    on a machine that has no apriltag_localize to generate from.
    """
    if not _LAYOUT_CACHE:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "testbed_layout.txt")
        try:
            with open(path, encoding="utf-8") as fh:
                _LAYOUT_CACHE.append(fh.read().strip())
        except OSError:
            _LAYOUT_CACHE.append("")
    return _LAYOUT_CACHE[0]



def state_text(truth: dict, include_layout: bool | None = None) -> str:
    """Measured state as prose. The model is TOLD positions rather than
    asked to extract them -- see this module's docstring."""
    lines = ["Measured system state (from overhead AprilTag tracking):",
             f"- robots tracked: {truth['robot_count']}"]
    for name, p in sorted((truth.get("robots") or {}).items()):
        # 0..360 to match the convention line below. The tracker reports
        # -180..180, so west arrived as about -90 while the prompt claimed
        # "270=west" -- a value that never appeared in the data. Measured
        # 2026-09-09 on 682 state-only questions: the model answered "west"
        # zero times out of 116 orientation questions, 0/15 correct where the
        # true facing was west. north (+/-180) and east (+90) sit on the
        # positive half and were unaffected, which is why this hid.
        yaw = round(float(p["yaw_deg"]) % 360.0, 2)
        lines.append(f"- {name}: x={p['x_in']}in, y={p['y_in']}in, "
                     f"yaw={yaw}deg")
    # RANGES, not the four cardinal points, and no negation.
    #
    # Stating it as points ("0=south, 90=east, 180=north, 270=west") leaves
    # the model to do modular arithmetic for the one direction that straddles
    # the wrap. Measured 2026-09-10, state-only, on 48 true-south poses:
    # 0/48 correct on BOTH machines, while north/east/west scored 82-95%.
    # South is the largest class in the capture set (423 of 861 poses).
    #
    # The previous attempt to fix it added "yaw 355 is 5 degrees short of
    # south, not close to west" -- and the laptop's dominant wrong answer on
    # true south became west, 25 of 48. Naming a wrong answer inside a
    # negation primes it. The desktop said north instead, which is why the
    # two GPUs disagreed on how to be wrong.
    #
    # Four wordings, measured head to head on the same 114 state-only facing
    # questions, 2026-09-10, qwen3.5:4b, no layout block:
    #
    #                       overall   north   south    east    west
    #   points               45%      22/22   11/48   16/22    2/22
    #   points + wrap note   54%      21/22   12/48    9/22   20/22  <- this
    #   ranges (315-45 etc)  36%       5/22   23/48    4/22    9/22
    #   nearest-of-four      40%      17/22    9/48   11/22    9/22
    #   floor (always south) 42%
    #
    # Ranges and nearest-value both land BELOW the constant-answer floor:
    # interval membership and nearest-neighbour are each harder for this model
    # than matching a stated value. The bare point form loses west almost
    # entirely (2/22) because west is the one anchor no yaw sits exactly on.
    #
    # The wrap sentence names west inside a negation, which looked like it
    # ought to prime the wrong answer -- an earlier run had west failing and
    # that was the suspicion. It does the opposite: this wording has the best
    # west score of the four. The west failures came from the layout block,
    # which is why --layout defaults to auto and facing does not receive it.
    # Same run, wording held constant: facing 43% with the block, 54% without.
    #
    # 54% against a 42% floor is a weak result and is meant to read as one.
    # yaw retrieval is 100% in this same condition, so the model reads the
    # number perfectly and converts it barely above baseline; south, the
    # largest class, is 25%. With the image it reaches 89% because nothing
    # has to be converted. Do not tune this sentence further to move the
    # number -- that fits the prompt to the model and the contrast above is
    # the actual finding.
    lines.append("Coordinate frame: +x is east (right), +y is north (up in "
                 "the image). Yaw is 0-360 degrees: 0 (or 360)=south, "
                 "90=east, 180=north, 270=west. Values wrap, so yaw 355 "
                 "is 5 degrees short of south, not close to west.")
    # include_layout None defers to --layout; a question can override it.
    # See _LAYOUT_MODE and the occupancy questions' "layout" flag: the block
    # is worth +95 points on ws:which and costs -92 on counting, so "always"
    # and "never" are both the wrong global answer.
    want = _INCLUDE_LAYOUT if include_layout is None else include_layout
    if want:
        layout = testbed_layout()
        if layout:
            lines.append(layout)
    return "\n".join(lines)


# ---------------------------------------------------------------- ollama

def _options(num_ctx: int | None) -> dict:
    opts: dict = {"temperature": 0.0, "seed": 42}
    if num_ctx is not None:
        opts["num_ctx"] = num_ctx
    return opts


def ask(url: str, model: str, prompt: str, image_b64: str | None,
        timeout: float, num_ctx: int | None = None) -> tuple[str | None, float, str | None]:
    """One question. image_b64 None runs the state-only control condition."""
    msg: dict = {"role": "user", "content": prompt}
    if image_b64 is not None:
        msg["images"] = [image_b64]
    body = json.dumps({
        "model": model,
        "messages": [msg],
        "stream": False,
        "keep_alive": "30m",
        # Reasoning OFF. Qwen vision models can otherwise spend thousands of
        # reasoning tokens answering a short, automatically scored question.
        # Keeping this disabled preserves the latency-focused deployment
        # condition used by the original benchmark and prevents an answer from
        # being lost when the reasoning budget is exhausted. Models that ignore
        # this field are unaffected; any thinking they emit is captured below.
        "think": False,
        # Deterministic: consistency is being MEASURED, so sampling noise
        # must not be mistaken for it. Any variation left is the model's.
        #
        # num_ctx is sent only when asked for. Ollama otherwise picks the
        # window itself, and on an 8GB card its choice of 8192 pushed
        # qwen3.5:9b to a 12%/88% CPU/GPU split -- 6.3GB resident with 800MB
        # served from system RAM over PCIe, and prompt eval at 2.78s. At 2048
        # the same model sits at 5.4GB, 100% GPU, and 0.60s. Real prompts
        # here measure 815 tokens, so 2048 leaves 60% of the window unused
        # and cannot truncate anything.
        "options": _options(num_ctx),
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
            return None, dt, "empty content (answer left in thinking buffer)", {}
        return content, dt, None, {
            "eval_count": data.get("eval_count"),
            "prompt_eval_count": data.get("prompt_eval_count"),
            "done_reason": data.get("done_reason"),
            "thinking_chars": len(msg.get("thinking") or ""),
        }
    except urllib.error.HTTPError as exc:
        try:
            detail = exc.read().decode("utf-8", errors="replace").strip()
        except Exception:
            detail = ""
        message = f"HTTP {exc.code} {exc.reason}"
        if detail:
            message += f": {detail}"
        return None, time.monotonic() - t0, message, {}
    except Exception as exc:
        return None, time.monotonic() - t0, repr(exc), {}


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

    if rows and not any(r.get("error") is None for r in rows):
        print("\nNO ACCURACY RESULT: every API request failed before the model "
              "returned an answer.")
        unique_errors = []
        for row in rows:
            err = row.get("error")
            if err and err not in unique_errors:
                unique_errors.append(err)
        for err in unique_errors[:3]:
            print(f"  - {err}")
        print()
        return

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
    print()
    print("answer diversity (one repeated answer = the model is not reading")
    print("the question; its accuracy there is only the base rate). Grouped by")
    print("question TEMPLATE, not category: on 2026-09-08 qwen3.5:9b answered")
    print("no to all 64 depot:is questions and the occupancy category still")
    print("looked varied, because depot:which and ws:which did vary.")
    flagged = False

    def _tmpl(r):
        return r.get("template") or r["category"]

    templates = sorted({_tmpl(r) for r in rows})
    for m in models:
        for tmpl in templates:
            mine = [r for r in rows if r["model"] == m
                    and _tmpl(r) == tmpl and r["answer"]]
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
                print(f"  {m:<20}{tmpl:<14}said {top!r} for {n}/{len(mine)}"
                      f" -- scored {ok}/{len(mine)}")
    if not flagged:
        print("  (none -- every model varied its answers in every template)")

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


def _duration(seconds: float) -> str:
    """Compact elapsed-time/ETA display."""
    total = max(0, int(round(seconds)))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    if hours:
        return f"{hours:d}h{minutes:02d}m"
    if minutes:
        return f"{minutes:d}m{secs:02d}s"
    return f"{secs:d}s"


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
        description="Score a remote Ollama VLM on captured testbed frames.")
    ap.add_argument(
        "--samples-dir", default=None,
        help="vlm_samples directory (default: vlm_samples beside this script)")
    ap.add_argument(
        "--model", default=MODEL,
        help="remote model tag (default: %(default)s). The other tag pulled "
             "on the Lambda box is qwen3-vl:30b, the MoE model, which is "
             "roughly 5x faster than the dense 32B.")
    ap.add_argument(
        "--url", default=os.environ.get("OLLAMA_URL", DEFAULT_URL),
        help="Ollama API base URL (default: %(default)s). This is the SSH "
             "tunnel, not a local Ollama.")
    ap.add_argument("--max-per-category", type=int, default=3,
                    help="cap orientation/spatial/relation questions per "
                         "sample (default 3). Counting, closest-pair and the "
                         "four balanced controls are always included.")
    ap.add_argument("--layout", choices=("auto", "always", "never"),
                    default="auto",
                    help="who gets the static testbed layout (bay and depot "
                         "coordinates). auto (default) gives it only to the "
                         "occupancy questions, which need it: measured "
                         "2026-09-10 on 40 samples, state-only, it is worth "
                         "+95pt on ws:which, +63 on ws:any and +59 on "
                         "depot:which, while costing -92pt on counting -- a "
                         "question the state text answers outright with "
                         "'robots tracked: 6'. Rewording the block to remove "
                         "every subtraction moved counting only 8%% -> 21%%, "
                         "so that is context distraction, not one bad "
                         "phrase. always/never force one setting for every "
                         "question and are kept for measuring the effect.")
    ap.add_argument("--labels", nargs="+", default=None,
                    help="only use samples captured under these --label "
                         "values, e.g. --labels posed-2robots posed-3robots "
                         "run-6robot-yieldtest. Lets one capture pool serve "
                         "several experiments without deleting anything -- "
                         "a session that went badly is still valid scenes.")
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
    ap.add_argument("--num-ctx", type=int, default=None,
                    help="context window per request. Ollama's own choice can "
                         "spill a model into system RAM on a small card; 2048 "
                         "fits the 815-token prompts here with room to spare. "
                         "Must be identical across models being compared.")
    ap.add_argument(
        "--max-output-tokens", type=int, default=None,
        help="deprecated compatibility option; ignored because the Lambda "
             "Ollama 0.13.5 server rejected capped requests")
    ap.add_argument("--out", default=None, help="results JSON path")
    ap.add_argument("--report", default=None,
                    help="re-print the comparison from a results JSON and "
                         "exit")
    args = ap.parse_args()

    global _INCLUDE_LAYOUT, _LAYOUT_MODE
    _LAYOUT_MODE = args.layout
    # _INCLUDE_LAYOUT still drives a bare state_text(truth) call, which the
    # rescorer and any external caller use; "auto" means such a caller gets
    # the block only when it asks for it explicitly.
    _INCLUDE_LAYOUT = (args.layout == "always")

    if args.report:
        with open(args.report, encoding="utf-8") as fh:
            report(json.load(fh))
        return
    if args.max_output_tokens is not None:
        print("note: --max-output-tokens is ignored for compatibility with "
              "the Lambda Ollama server")
    samples_dir = args.samples_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vlm_samples")
    sample_ids = sorted(
        d for d in os.listdir(samples_dir)
        if os.path.isfile(os.path.join(samples_dir, d, "truth.json")))
    if not sample_ids:
        raise SystemExit(f"no samples in {samples_dir} — run vlm_capture.py")
    if args.labels:
        wanted, kept = set(args.labels), []
        for sid in sample_ids:
            with open(os.path.join(samples_dir, sid, "truth.json"),
                      encoding="utf-8") as fh:
                if (json.load(fh).get("label") or "") in wanted:
                    kept.append(sid)
        if not kept:
            raise SystemExit(
                f"no samples with label(s) {sorted(wanted)}")
        print(f"using {len(kept)} of {len(sample_ids)} samples "
              f"(labels: {', '.join(sorted(wanted))})")
        sample_ids = kept

    if args.limit and args.limit < len(sample_ids):
        # Evenly spaced, not the first N: samples are timestamped in capture
        # order, so the first N would all come from the opening seconds of
        # one run and miss whatever happened later.
        step = len(sample_ids) / args.limit
        sample_ids = [sample_ids[int(i * step)] for i in range(args.limit)]
        print(f"using {len(sample_ids)} of the available samples "
              "(evenly spaced)")

    print(f"model: {args.model}")
    print(f"Ollama endpoint: {args.url}")
    check_models(args.url, [args.model])

    # Load each frame and truth file once before starting requests. This keeps
    # disk I/O outside the per-question latency measurement.
    loaded = []
    for sid in sample_ids:
        d = os.path.join(samples_dir, sid)
        with open(os.path.join(d, "truth.json"), encoding="utf-8") as fh:
            truth = json.load(fh)
        with open(os.path.join(d, "frame.jpg"), "rb") as fh:
            image_b64 = base64.b64encode(fh.read()).decode()
        # Both state variants, built once per sample. Which one a question
        # gets depends on --layout and the question's own "layout" flag; see
        # state_text(). Building the string twice is free next to inference.
        loaded.append((sid, build_questions(truth, args.max_per_category),
                       (state_text(truth, False), state_text(truth, True)),
                       image_b64))
    total = sum(len(q) for _s, q, _c, _i in loaded)
    print(f"{len(loaded)} sample(s), {total} question(s) each pass, "
          f"1 model, {args.repeats} repeat(s) "
          f"-> {total * args.repeats} calls")

    rows: list[dict] = []
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"qwen3_vl_32b_results_{datetime.now().strftime('%Y%m%d_%H%M')}.json")

    def _save(partial: bool) -> dict:
        res = {
            "generated_utc": datetime.now(timezone.utc).isoformat(
                timespec="seconds"),
            "url": args.url, "models": [args.model],
            "endpoints": {args.model: args.url},
            "repeats": args.repeats,
            "sample_count": len(loaded), "question_count": total,
            "condition": "state-only" if args.no_image else "state+image",
            "labels": args.labels,
            "num_ctx": args.num_ctx,
            "partial": partial,
            "rows": rows,
        }
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=2)
            fh.write(chr(10))
        return res

    interrupted = False
    stopped_early = False
    consecutive_errors = 0
    total_calls = total * args.repeats
    completed_calls = 0
    ok_calls = 0
    run_started = time.monotonic()
    for model in [args.model]:
        print(f"\n{model}")
        if interrupted or stopped_early:
            break
        warmed = False
        sys.stdout.write("  loading the model on the server, the first "
                         "answer can take 10-30s ... ")
        sys.stdout.flush()
        for sid, questions, context, image_b64 in loaded:
            if interrupted or stopped_early:
                break
            for q in questions:
                if interrupted or stopped_early:
                    break
                for rep in range(args.repeats):
                    # auto: only questions that need bay/depot coordinates get
                    # them. Measured 2026-09-10, state-only, same 40 samples:
                    # the block is worth +95pt on ws:which, +63 on ws:any, +59
                    # on depot:which, and costs -92pt on counting, where the
                    # state text already says "robots tracked: 6" outright.
                    # Rewording it to remove every subtraction did not help
                    # (8% -> 21%), so this is ordinary context distraction
                    # rather than one bad phrase, and no single global setting
                    # is right for every question.
                    if _LAYOUT_MODE == "always":
                        state = context[1]
                    elif _LAYOUT_MODE == "never":
                        state = context[0]
                    else:
                        state = context[1] if q.get("layout") else context[0]
                    prompt = (f"{state}\n\nLook at the camera image of "
                              f"this robot testbed.\n{q['prompt']}")
                    try:
                        answer, secs, err, meta = ask(
                            args.url, model, prompt,
                            None if args.no_image else image_b64,
                            args.timeout, args.num_ctx)
                    except KeyboardInterrupt:
                        # Everything answered so far is still real data, and
                        # a long run should not be all-or-nothing.
                        interrupted = True
                        break
                    if not warmed:
                        warmed = True
                        sys.stdout.write(
                            "ready in "
                            + _duration(time.monotonic() - run_started)
                            + chr(10))
                        sys.stdout.flush()
                    correct = False
                    if answer is not None:
                        try:
                            correct = bool(q["check"](answer))
                        except Exception:
                            correct = False
                    rows.append({
                        "sample": sid, "model": model,
                        "question_id": q["id"], "category": q["category"],
                        "template": q.get("template", q["category"]),
                        "prompt": q["prompt"],
                        "expected": q["expect"] if not isinstance(
                            q["expect"], (list, dict)) else json.dumps(
                                q["expect"]),
                        "answer": answer, "correct": correct,
                        "seconds": round(secs, 3), "error": err,
                        **meta,
                        "repeat": rep,
                    })
                    completed_calls += 1
                    ok_calls += bool(correct)
                    if err:
                        consecutive_errors += 1
                        sys.stdout.write("E")
                        sys.stdout.flush()
                        if consecutive_errors == 1:
                            print(f"\n  API error: {err}")
                        if consecutive_errors >= 3:
                            print("\nStopping after three consecutive API "
                                  "errors; no further samples were sent.")
                            stopped_early = True
                            break
                    else:
                        consecutive_errors = 0
                        sys.stdout.write("." if correct else "x")
                    sys.stdout.flush()
            elapsed = time.monotonic() - run_started
            average = elapsed / completed_calls if completed_calls else 0.0
            eta = average * max(0, total_calls - completed_calls)
            sys.stdout.write(
                f"  {sid}  [{completed_calls}/{total_calls}, "
                f"{ok_calls} correct ({100 * ok_calls / max(completed_calls, 1):.0f}%), "
                f"elapsed {_duration(elapsed)}, ETA {_duration(eta)}]" + chr(10))
            # Checkpoint every sample. A run that is killed rather than
            # interrupted cleanly used to leave nothing on disk at all.
            _save(partial=True)

    results = _save(partial=interrupted or stopped_early)
    if interrupted:
        print()
        print(f"INTERRUPTED -- {len(rows)} answer(s) kept and saved.")
    elif stopped_early:
        print()
        print(f"STOPPED EARLY -- {len(rows)} failed call(s) saved for "
              "diagnosis; this is not an accuracy result.")
    report(results)
    print(f"full per-answer detail: {out}")


if __name__ == "__main__":
    main()
