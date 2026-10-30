#!/usr/bin/env python3
"""One standardized pipeline from every VLM benchmark JSON to one master table.

WHAT THIS IS
------------
Every benchmark run on this testbed writes its own JSON. Answering "did JPEG
95 help, and where" meant opening several of them by hand and retyping
percentages, which is how the overlay finding was wrong the first time. This
reads all of them, joins each answer back to the scene it was asked about,
and emits one long-format table plus the summaries and paired contrasts.

RAW RESULTS ARE NEVER TOUCHED. Every output goes under --out-dir (default
analysis/), every master row carries source_file + row_index, so any
percentage in any summary traces back to one model's one response to one
question about one frame. Nothing here writes to tools/ or to
"data/stats_for_paper/".

GRADING IS NOT REDONE. `correct` is read from the benchmark file as-is.
This module adds only classification on top: which capability family a
question belongs to, and -- for wrong answers -- what kind of wrong it was.
The failure classifier reuses vlm_bench.py's own extractors (_yesno,
_single_cardinal, _numbers_in, _phantom_names ...) so its reading of an
answer cannot drift from the grader's.

THE THREE THINGS THAT ARE NOT IN THE RESULT FILES
-------------------------------------------------
Measured 2026-09-21 across all 17 static benchmark files: none of them
record overlay, JPEG quality, blank-reference-image, or the machine that ran
them, and `url` is "localhost" in every single one, so the backend is not
derivable either. Where a condition can be recovered it is:

  overlay       DERIVED  from the sample's truth.json `label` ("B02-overlay")
  jpeg_quality  MEASURED from the frame's own JPEG quantization tables
  context       DERIVED  from the file's `condition` field
  block         DERIVED  from the label prefix -- the RCBD pairing unit

and where it cannot, it is DECLARED in tools/vlm_runs.json (seeded by
--seed-manifest, then edited by hand) or left unknown. Every condition
column's provenance is written to conditions.csv, so a declared value is
never mistaken for a measured one.

WHY JPEG QUALITY IS MEASURED AND NOT READ OFF THE LABEL
-------------------------------------------------------
tools/vlm_samples holds a 2026-09-10 pilot labelled A1-q75, A1-q95, A2-q75,
A2-q95, A3-q75, A3-q95. Every one of those 25 frames measures q95 by its own
quantization table, and the "q75" files are only ~4% smaller than their "q95"
pairs where a real q75 is about half the size. The labels are wrong: frames
are captured by vlm_capture.py straight from apriltag_localize.py's
/full.jpg, which re-encodes at a hardcoded FULL_JPEG_QUALITY = 95
(apriltag_localize.py:2123), and vlm_capture.py has no quality flag at all.
Whatever was set on camera_bridge_windows.py never reached the saved file.

That pilot therefore compared q95 against q95 and measured nothing. Trusting
the label would have produced a confident JPEG-quality finding out of two
identical arms -- which is exactly why this module reads the quantization
tables instead. The contrast machinery correctly reports "no paired data" for
jpeg on that pool rather than inventing a result.

USAGE
-----
    python3 tools/vlm_aggregate.py --seed-manifest    # once, then edit it
    python3 tools/vlm_aggregate.py                    # build everything
    python3 tools/vlm_aggregate.py --contrast overlay --model gemma4:e2b-it-qat
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vlm_bench as VB  # noqa: E402  -- reuse the grader's own extractors

try:                       # the live suite's own verdict reader, incl. its
    import vlm_live as VL  # `ignore` rule for names the question supplies
except Exception:          # pragma: no cover
    VL = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Where benchmark JSONs live. Read-only, every one of them.
SOURCE_GLOBS = [
    "data/stats_for_paper/*.json",
    "tools/*.json",
    "tools/vlm_live/*/*.json",
    "runs_linux/*.json",
]

#: Sample pools searched for a sample_id's truth.json + frame.jpg.
SAMPLE_DIRS = [
    "tools/rcbd_2x2",
    "tools/rcbd_samples",
    "tools/vlm_samples",
    "tools/vlm_samples_960_archive_20260909",
    "tools/vlm_live",
]

MANIFEST = "tools/vlm_runs.json"

# --------------------------------------------------------------------------
# Capability families.
#
# The point of splitting these is that orientation and numeric readback are
# different skills from perception, and pooling them hides both. Measured on
# this question set: `retrieval` is not one thing -- yaw:* asks the model to
# read an angle back out of the state text and pos:* asks it to read two
# coordinates, and those fail for different reasons and at different rates.
# They are separated here for the same reason orientation is kept out of the
# headline: so a model that cannot read a number off a line of text does not
# get scored as though it cannot see.
# --------------------------------------------------------------------------
FAMILY_BY_QID_PREFIX = {
    "count":    "counting",
    "facing":   "orientation",
    "yaw":      "yaw_readback",
    "pos":      "position_readback",
    "extreme":  "spatial",
    "closest":  "spatial",
    "northof":  "relation",
    "depot":    "occupancy",
    "ws":       "occupancy",
    "ctl":      "control",
}

#: Families excluded from the "core perception" headline. Orientation and the
#: two readback families are reported on their own so they cannot carry the
#: overall number. `control` is excluded for a different reason: those four
#: questions are balanced hallucination probes, not a capability -- a model
#: scoring 50% on them is guessing, not half-competent.
NON_CORE_FAMILIES = {"orientation", "yaw_readback", "position_readback",
                     "control"}

#: Live-rig categories. A disjoint question set from the static suite (ops:*
#: questions asked of a running fleet), so it is aggregated alongside but
#: NEVER pooled with it -- `suite` keeps them apart in every summary.
LIVE_FAMILY = {
    "traffic":   "fleet_traffic",
    "dispatch":  "fleet_dispatch",
    "recovery":  "fleet_recovery",
    "narrative": "fleet_narrative",
    "spatial":   "spatial",
}

# Grading tolerances, read off the question builders in vlm_bench.py
# (lines 282 and 289). Duplicated here only so the failure classifier can
# tell "named a number, wrong by 40 degrees" from "named no number at all".
YAW_TOL_DEG = 5.0
XY_TOL_IN = 2.0

# Standard JPEG Annex K luminance quantization table, for the quality
# estimate below.
STD_LUMA = [
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
]


def jpeg_quality(path: str) -> int | None:
    """libjpeg quality recovered from the file's own quantization table.

    Inverts libjpeg's table scaling: scale = 200 - 2q for q >= 50, and
    table[i] = clamp(round((std[i]*scale + 50)/100), 1, 255). Entries that
    saturated at 1 or 255 carry no information about the scale and are
    dropped. This MEASURES the encode quality rather than trusting a
    filename, which matters because no result file records it.

    Returns None for a non-JPEG or a file using tables this cannot invert.
    """
    try:
        from PIL import Image
        with Image.open(path) as im:
            tables = getattr(im, "quantization", None)
    except Exception:
        return None
    if not tables or 0 not in tables:
        return None
    table = list(tables[0])
    if len(table) < 64:
        return None
    scales = []
    for value, std in zip(table[:64], STD_LUMA):
        if value <= 1 or value >= 255:
            continue
        scales.append((value * 100.0 - 50.0) / std)
    if not scales:
        return None
    scale = statistics.median(scales)
    if scale <= 0:
        return 100
    quality = (200.0 - scale) / 2.0 if scale <= 100 else 5000.0 / scale
    return max(1, min(100, int(round(quality))))


# --------------------------------------------------------------------------
# Scene index: sample_id -> truth dict + frame path
# --------------------------------------------------------------------------
class Scenes:
    """Every sample pool on disk, indexed by sample_id.

    Carries the label (hence overlay arm and RCBD block), the robot list
    (needed to tell a hallucinated name from a wrong one), and the frame path
    (hence the measured JPEG quality).
    """

    def __init__(self, root: str, dirs: list[str]):
        self.truth: dict[str, dict] = {}
        self.frame: dict[str, str] = {}
        self.pool: dict[str, str] = {}
        self._quality_cache: dict[str, int | None] = {}
        for rel in dirs:
            base = os.path.join(root, rel)
            if not os.path.isdir(base):
                continue
            for name in sorted(os.listdir(base)):
                d = os.path.join(base, name)
                tpath = os.path.join(d, "truth.json")
                if not os.path.isfile(tpath):
                    continue
                try:
                    t = json.load(open(tpath, encoding="utf-8"))
                except Exception:
                    continue
                sid = str(t.get("sample_id") or name)
                # First pool wins: SAMPLE_DIRS is in precedence order, so a
                # sample re-captured into a newer pool is not shadowed by an
                # archive copy with different conditions.
                if sid in self.truth:
                    continue
                self.truth[sid] = t
                self.pool[sid] = rel
                fpath = os.path.join(d, "frame.jpg")
                if os.path.isfile(fpath):
                    self.frame[sid] = fpath

    def label(self, sid: str) -> str | None:
        t = self.truth.get(sid)
        lab = t.get("label") if t else None
        return None if lab in (None, "None", "") else str(lab)

    def block(self, sid: str) -> str | None:
        """RCBD block from a "B02-overlay" style label.

        The block, not the scene, is the pairing unit: each block was posed
        once and then captured under both overlay arms, so a raw frame and an
        overlay frame from the same block show the same robot placement. Two
        frames per block-arm cell.
        """
        lab = self.label(sid)
        if not lab:
            return None
        m = re.match(r"^(B\d+)", lab)
        return m.group(1) if m else None

    def overlay(self, sid: str) -> str | None:
        lab = self.label(sid)
        if not lab:
            return None
        # TOKEN match, not suffix: make_jpeg_arms.py appends the quality arm
        # ("B02-overlay-q75"), and a suffix test would stop resolving the
        # overlay arm the moment a second factor was added to the label.
        parts = lab.lower().split("-")
        if "overlay" in parts:
            return "on"
        if "raw" in parts:
            return "off"
        return None

    def jpeg_q(self, sid: str) -> int | None:
        if sid not in self._quality_cache:
            path = self.frame.get(sid)
            self._quality_cache[sid] = jpeg_quality(path) if path else None
        return self._quality_cache[sid]

    def robot_count(self, sid: str):
        t = self.truth.get(sid) or {}
        n = t.get("robot_count")
        return n if isinstance(n, int) else ""

    def robots(self, sid: str) -> list[str]:
        t = self.truth.get(sid) or {}
        r = t.get("robots") or {}
        return sorted(r) if isinstance(r, dict) else []


# --------------------------------------------------------------------------
# Conditions, with provenance
# --------------------------------------------------------------------------
CONDITION_COLUMNS = ["model", "overlay", "jpeg_quality", "blank_image",
                     "context", "layout", "num_predict", "num_ctx",
                     "dataset", "run", "machine", "backend"]


def normalise_context(condition: str | None) -> str:
    """The image/state factor, spelled one way.

    vlm_bench writes "state+image"/"state-only"; vlm_live writes
    "frame+state". Same two levels, three spellings.
    """
    c = (condition or "").lower()
    if c in ("state+image", "frame+state", "image+state"):
        return "state+image"
    if c in ("state-only", "state_only", "no-image", "noimage"):
        return "state-only"
    return c or "unknown"


def load_manifest(root: str) -> dict:
    path = os.path.join(root, MANIFEST)
    if not os.path.isfile(path):
        return {}
    data = json.load(open(path, encoding="utf-8"))
    return data.get("files", data) if isinstance(data, dict) else {}


def resolve_conditions(meta: dict, rel: str, declared: dict,
                       overlays: set, qualities: set, pools: set) -> dict:
    """One row per source file: each condition's value and where it came from.

    Precedence is deliberate: MEASURED beats DERIVED beats DECLARED. A
    declared JPEG quality cannot override the quantization tables in the
    frames actually used -- if they disagree the manifest is wrong, and the
    conflict is reported rather than silently resolved.
    """
    out: dict[str, tuple] = {}
    # A null in the manifest is the stub's placeholder, not a declaration.
    # Treating it as declared wrote "None" into the machine column where
    # "unknown" belongs, and "None" reads like a measured value.
    declared = {k: v for k, v in (declared or {}).items() if v is not None}

    def put(col, value, how):
        out[col] = (value, how)

    # -- derived from the sample pool the answers actually referenced
    if len(overlays) == 1:
        put("overlay", next(iter(overlays)), "derived:sample-label")
    elif overlays:
        put("overlay", "mixed", "derived:sample-label")
    else:
        put("overlay", declared.get("overlay", "unknown"),
            "declared" if "overlay" in declared else "unknown")

    if len(qualities) == 1:
        put("jpeg_quality", next(iter(qualities)), "measured:quant-table")
    elif qualities:
        put("jpeg_quality", "mixed:" + ",".join(str(q) for q in sorted(qualities)),
            "measured:quant-table")
    else:
        put("jpeg_quality", declared.get("jpeg_quality", "unknown"),
            "declared" if "jpeg_quality" in declared else "unknown")

    put("dataset", "|".join(sorted(pools)) if pools
        else declared.get("dataset", "unknown"),
        "derived:sample-pool" if pools else "unknown")

    # -- derived from the file's own metadata
    put("context", normalise_context(meta.get("condition")),
        "derived:condition-field")

    for col, key in (("num_ctx", "num_ctx"), ("num_predict", "num_predict"),
                     ("run", "run")):
        if meta.get(key) is not None:
            put(col, meta[key], "recorded")
        elif key in declared:
            put(col, declared[key], "declared")
        else:
            put(col, "unknown", "unknown")

    # -- not recoverable from any result file written before 2026-09-21
    for col in ("blank_image", "layout", "machine"):
        if col in declared:
            put(col, declared[col], "declared")
        elif meta.get(col) is not None:
            put(col, meta[col], "recorded")
        else:
            put(col, "unknown", "unknown")

    url = meta.get("url") or ""
    host = re.sub(r"^https?://", "", url).split("/")[0] or "unknown"
    put("backend", host, "recorded")
    put("model", "per-row", "recorded")
    return out


def seed_manifest(root: str, files: list[tuple[str, dict]]) -> str:
    """Write a manifest stub with one entry per source file.

    Only the conditions that cannot be recovered from disk are listed, with
    null where the value is genuinely not known. Editing a null to a value
    is how a run gets its machine and blank-image arm on the record; leaving
    it null keeps it honestly "unknown" in every summary.
    """
    path = os.path.join(root, MANIFEST)
    entries = {}
    for rel, meta in files:
        entries[rel] = {
            "machine": None,
            "blank_image": None,
            "layout": None,
            "note": "",
        }
    payload = {
        "_readme": [
            "Conditions that no benchmark file records. Edit the nulls.",
            "machine: which computer served inference, e.g.",
            "  'rtx5060-laptop-8gb' or 'rtx3080-remote'.",
            "  or 'rtx3080-remote'. Latency is only comparable within one.",
            "blank_image: true/false -- was a blank-testbed reference image",
            "  supplied alongside the scene frame. false for every run before",
            "  2026-09-21; the feature did not exist.",
            "layout: 'auto' | 'always' | 'never' -- the --layout setting.",
            "Anything left null stays 'unknown' and is excluded from",
            "contrasts on that factor rather than guessed.",
        ],
        "files": entries,
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return path


# --------------------------------------------------------------------------
# Failure taxonomy
# --------------------------------------------------------------------------
CARDINALS = ("north", "south", "east", "west")

#: Live-suite families whose answer is a single robot name.
LIVE_NAME_FAMILIES = {"fleet_traffic", "fleet_dispatch", "fleet_recovery",
                      "spatial"}


def _exp_str(expected) -> str:
    """`expected` as text, WITHOUT treating a falsy value as absent.

    The integer 0 is a real expected value here -- it is the truth for every
    counting question about the 20 empty-table frames -- and `expected or ""`
    silently turned it into "no truth value".
    """
    return "" if expected is None else str(expected).strip()


def _num(v: float) -> str:
    """5.0 -> "5", 63.48 -> "63.48"."""
    return str(int(v)) if float(v).is_integer() else str(v)


def extract_given(row: dict, family: str, robots: list[str]) -> str:
    """What the model actually committed to, normalized for comparison.

    The point is that `expected` and `given` sit side by side in the master
    table, so "it said Alvik3 and the answer was Alvik5" or "it said 5 and
    the answer was 6" is readable without opening the raw answer text. The
    extraction uses the SAME readers the grader used, so `given` cannot
    disagree with `correct`: where the grader saw no commitment, this returns
    "" rather than guessing at one.

    Filled for correct rows too -- a correct row's given should equal its
    expected, which makes the column self-checking.
    """
    answer = row.get("answer") or ""
    if not answer.strip():
        return ""
    final = VB._final_answer(answer)
    qid = str(row.get("question_id") or "")

    if family in ("counting", "yaw_readback"):
        nums = VB._numbers_in(final)
        return _num(nums[0]) if nums else ""
    if family == "position_readback":
        nums = VB._numbers_in(final)
        return f"{_num(nums[0])}, {_num(nums[1])}" if len(nums) >= 2 else ""
    if family == "orientation":
        card = VB._single_cardinal(final)
        if card:
            return card
        multi = [c for c in CARDINALS if c in final.lower()]
        return "|".join(multi) if multi else ""
    if family in ("relation", "control"):
        v = VB._yesno(final)
        return "" if v is None else ("yes" if v else "no")
    if family == "occupancy":
        v = VB._yesno(final)
        if v is not None:
            return "yes" if v else "no"
        if VB._says_none(final):
            return "none"
        named = [n for n in robots if n.lower() in answer.lower()]
        return "|".join(named) if named else ""
    if family == "spatial" and row.get("suite") != "live":
        named = [n for n in robots if n.lower() in answer.lower()]
        return "|".join(named) if named else ""

    # Live suite: the verdict is whatever vlm_live's own reader concludes,
    # including its `ignore` rule for names the question itself supplies.
    if VL is not None:
        try:
            verdict = VL.verdict_name(answer, robots, ignore=_live_ignore(qid, row))
        except Exception:
            verdict = None
        if verdict:
            return verdict
    chunks = VB._verdict_chunks(answer) or [final]
    if VB._says_none(chunks[0]):
        return "NONE"
    named = [n for n in robots if n.lower() in answer.lower()]
    return "|".join(named) if named else ""


def _live_ignore(qid: str, row: dict) -> set:
    """Names the live question itself supplies, struck before reading a verdict.

    ops:reassign names the downed robot and ops:blocker names the waiter, and
    both appear in the question ID after the second colon. Mirrors
    vlm_live._check_reassign / _check_blocker.
    """
    parts = qid.split(":")
    if len(parts) >= 3 and parts[2].startswith("Alvik"):
        return {parts[2]}
    return set()


def classify_failure(row: dict, family: str, robots: list[str],
                     given: str = "") -> str:
    """What kind of wrong a wrong answer was. "" for a correct answer.

    Only ever called on rows the benchmark already scored incorrect, so this
    never disagrees with `correct`. It reuses vlm_bench's extractors, so
    "did the model name a cardinal at all" is decided by the same code that
    decided whether the cardinal was right.

    The distinction that matters most here is refusal-shaped failure
    (unparseable, truncated, answer stranded in the thinking buffer) versus
    a confident wrong answer. Pooling them makes a model that emits garbage
    look like a model that misreads the scene.
    """
    answer = row.get("answer") or ""
    err = row.get("error")
    qid = str(row.get("question_id") or "")

    if err:
        low = str(err).lower()
        if "thinking" in low:
            return "answer_in_thinking_buffer"
        if "timeout" in low or "timed out" in low:
            return "timeout"
        return "request_error"
    if row.get("done_reason") == "length":
        return "truncated"
    if not answer.strip():
        return "empty_answer"
    # Gibberish only where NOTHING could be read out. Checking the text alone
    # contradicted the extractor: replies ending "the robots are not tracked"
    # were called gibberish while `given` had already read "no" out of them,
    # because the content test missed "not". A reply the grader could read a
    # verdict from is a wrong answer, not an unreadable one.
    if not given and looks_like_gibberish(answer):
        return "gibberish_no_answer_token"

    final = VB._final_answer(answer)

    # A name that does not exist in the scene is a hallucination regardless
    # of which family asked -- check it before the per-family reasoning.
    if VB._phantom_names(answer, robots):
        return "hallucinated_robot"

    if family == "control":
        # The four balanced probes. Direction matters: inventing a robot that
        # is not there and denying one that is are different defects, and a
        # model biased to "yes" scores 50% while being useless.
        verdict = VB._yesno(final)
        if verdict is None:
            return "unparseable_yesno"
        expected_yes = str(row.get("expected", "")).strip().lower() == "yes"
        return "false_positive" if verdict and not expected_yes \
            else "false_negative"

    if family == "counting":
        nums = VB._numbers_in(final)
        if not nums:
            return "no_number_given"
        try:
            want = float(row.get("expected"))
        except (TypeError, ValueError):
            return "wrong_count"
        return "count_over" if nums[0] > want else "count_under"

    if family == "orientation":
        card = VB._single_cardinal(final)
        if card is None:
            low = final.lower()
            if any(c in low for c in CARDINALS):
                return "multiple_cardinals"
            return "no_cardinal_given"
        want = str(row.get("expected", "")).strip().lower()
        if want in CARDINALS and card in CARDINALS:
            opposite = {"north": "south", "south": "north",
                        "east": "west", "west": "east"}
            if card == opposite[want]:
                return "orientation_reversed"
            return "orientation_perpendicular"
        return "wrong_orientation"

    if family == "yaw_readback":
        nums = VB._numbers_in(final)
        if not nums:
            return "no_number_given"
        try:
            want = float(row.get("expected"))
        except (TypeError, ValueError):
            return "wrong_yaw"
        best = min((min(abs(n - want), 360.0 - abs(n - want)) for n in nums),
                   default=None)
        if best is None:
            return "wrong_yaw"
        if best <= 2 * YAW_TOL_DEG:
            return "yaw_near_miss"
        if abs(best - 180.0) <= 2 * YAW_TOL_DEG:
            return "yaw_sign_flipped"
        return "yaw_wrong"

    if family == "position_readback":
        nums = VB._numbers_in(final)
        if len(nums) < 2:
            return "incomplete_coordinates"
        want = _expected_pair(row.get("expected"))
        if want is None:
            return "position_wrong"
        # Reading the right pair in the wrong order is a specific, fixable
        # defect (the prompt asks for x first), not general inaccuracy.
        if (abs(nums[0] - want[1]) <= XY_TOL_IN
                and abs(nums[1] - want[0]) <= XY_TOL_IN):
            return "coordinates_swapped"
        if (abs(nums[0] - want[0]) <= 2 * XY_TOL_IN
                and abs(nums[1] - want[1]) <= 2 * XY_TOL_IN):
            return "position_near_miss"
        return "position_wrong"

    if family == "relation":
        verdict = VB._yesno(final)
        if verdict is None:
            return "unparseable_yesno"
        return "relation_inverted"

    if family == "spatial":
        named = [n for n in robots if n.lower() in answer.lower()]
        if not named:
            return "no_robot_named"
        want = _expected_names(row.get("expected"))
        if qid.startswith("closest"):
            if len(named) < 2:
                return "incomplete_pair"
            if want and len(set(named) & set(want)) == 1:
                return "pair_half_right"
            return "wrong_pair"
        if len(named) > 1:
            return "multiple_robots_named"
        return "wrong_robot"

    if family in LIVE_NAME_FAMILIES:
        # Live ops questions all want exactly one robot name (or NONE).
        # Before this branch existed every one of these fell through to
        # "wrong_other", which is 401 rows saying nothing.
        given = extract_given(row, family, robots)
        want = _exp_str(row.get("expected"))
        if not given:
            if VB._phantom_names(answer, robots):
                return "hallucinated_robot"
            named = [n for n in robots if n.lower() in answer.lower()]
            if len(named) > 1:
                return "no_single_verdict"
            ignored = _live_ignore(qid, row)
            if ignored and any(n.lower() in answer.lower() for n in ignored):
                # Answered with the robot the question itself named -- "give
                # the work to the robot that just broke down".
                return "echoed_question_robot"
            return "no_robot_named"
        if want.upper() == "NONE" and given.upper() == "NONE":
            # Both read as NONE yet the grader scored it wrong, so this
            # module's reading and vlm_live's disagree. Surfaced as its own
            # label rather than dressed up as a model failure -- it is a
            # measurement problem and belongs in view.
            return "grader_extractor_mismatch"
        if want.upper() == "NONE":
            return "false_alarm_named_robot"
        if given.upper() == "NONE":
            return "missed_said_none"
        return "wrong_robot"

    if family == "occupancy":
        verdict = VB._yesno(final)
        if verdict is not None:
            expected_yes = str(row.get("expected", "")).strip().lower() == "yes"
            return "occupancy_false_positive" if verdict and not expected_yes \
                else "occupancy_false_negative"
        if VB._says_none(final):
            return "occupancy_said_none"
        named = [n for n in robots if n.lower() in answer.lower()]
        return "wrong_occupancy_set" if named else "unparseable_occupancy"

    return "wrong_other"


#: An answer holding none of these carries nothing the question could use.
_CONTENT = re.compile(r"[Aa]lvik\d+|[-+]?\d|\b(north|south|east|west|yes|no|"
                      r"none|true|false|correct|incorrect)\b", re.I)


def looks_like_gibberish(answer: str) -> bool:
    """No robot name, no number, no cardinal, no yes/no -- anywhere in it.

    Separated from the "model committed to the wrong thing" failures because
    it is a different defect with a different fix: a wrong robot is a
    perception or reasoning error, whereas a reply with no answer-shaped
    token in it at all is the model failing to respond to the task. Repetition
    loops, refusals and template echoes all land here.
    """
    text = (answer or "").strip()
    if not text:
        return False           # empty_answer is its own, earlier verdict
    return _CONTENT.search(text) is None


def _expected_pair(expected) -> tuple[float, float] | None:
    """(x, y) out of an expected field that is a JSON STRING, not a list.

    Learned the hard way: `expected` for pos:* is the string "[63.48, 76.78]".
    An isinstance(expected, list) check silently matches nothing and every
    row falls through to a default -- which is exactly how an earlier regrade
    script reported no changes while doing nothing at all.
    """
    if isinstance(expected, (list, tuple)) and len(expected) >= 2:
        try:
            return float(expected[0]), float(expected[1])
        except (TypeError, ValueError):
            return None
    nums = re.findall(r"-?\d+(?:\.\d+)?", str(expected or ""))
    if len(nums) >= 2:
        return float(nums[0]), float(nums[1])
    return None


def _expected_names(expected) -> list[str]:
    return re.findall(r"Alvik\d+", str(expected or ""))


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------
def family_of(row: dict, suite: str) -> str:
    qid = str(row.get("question_id") or "")
    cat = str(row.get("category") or "")
    if suite == "live":
        return LIVE_FAMILY.get(cat, cat or "unknown")
    prefix = qid.split(":")[0]
    return FAMILY_BY_QID_PREFIX.get(prefix, cat or "unknown")


def discover(root: str) -> list[str]:
    import glob
    found = []
    for pat in SOURCE_GLOBS:
        for path in glob.glob(os.path.join(root, pat)):
            rel = os.path.relpath(path, root).replace(os.sep, "/")
            if any(part.startswith("_") for part in rel.split("/")):
                continue
            found.append(rel)
    return sorted(set(found))


def suite_of(meta: dict, rel: str) -> str | None:
    """Which benchmark wrote this file, or None if it is not one.

    The static suite is identified by vlm_bench's own metadata block rather
    than by path, so a results file that gets moved still classifies. A JSON
    that is a sample pool, a layout dump or a plan is skipped, not guessed at.
    """
    if not isinstance(meta, dict) or not isinstance(meta.get("rows"), list):
        return None
    if "sample_count" in meta and "question_count" in meta:
        return "static"
    if "run" in meta and "model" in meta:
        return "live"
    return None


def load_all(root: str, scenes: Scenes, manifest: dict,
             only: list[str] | None = None) -> tuple[list[dict], list[dict]]:
    master: list[dict] = []
    conditions: list[dict] = []
    for rel in discover(root):
        if only and not any(o in rel for o in only):
            continue
        try:
            meta = json.load(open(os.path.join(root, rel), encoding="utf-8"))
        except Exception:
            continue
        suite = suite_of(meta, rel)
        if suite is None:
            continue
        rows = meta.get("rows") or []
        sids = {str(r.get("sample")) for r in rows if r.get("sample")}
        overlays = {o for o in (scenes.overlay(s) for s in sids) if o}
        qualities = {q for q in (scenes.jpeg_q(s) for s in sids) if q}
        pools = {p for p in (scenes.pool.get(s) for s in sids) if p}
        resolved = resolve_conditions(meta, rel, manifest.get(rel, {}),
                                     overlays, qualities, pools)

        crow = {"source_file": rel, "suite": suite,
                "generated_utc": meta.get("generated_utc"),
                "n_rows": len(rows)}
        for col, (value, how) in resolved.items():
            crow[col] = value
            crow[col + "__provenance"] = how
        conditions.append(crow)

        file_model = meta.get("model")
        for i, r in enumerate(rows):
            sid = str(r.get("sample") or "")
            fam = family_of(r, suite)
            # `correct` is None for questions that were never auto-graded --
            # all 222 of the live suite's open-ended ops:status narrative
            # answers. bool(None) is False, so scoring them as WRONG silently
            # counted 222 ungraded answers as failures and dragged every
            # model's live accuracy down. They are excluded from accuracy
            # instead, and carry failure="" rather than a made-up category.
            graded = r.get("correct") is not None
            correct = bool(r.get("correct"))
            robots = scenes.robots(sid) or [f"Alvik{n}" for n in range(1, 7)]
            given = extract_given(dict(r, suite=suite), fam, robots)
            rec = {
                "source_file": rel,
                "row_index": i,
                "suite": suite,
                "model": r.get("model") or file_model or "unknown",
                "overlay": scenes.overlay(sid) or resolved["overlay"][0],
                "jpeg_quality": scenes.jpeg_q(sid) or resolved["jpeg_quality"][0],
                "blank_image": resolved["blank_image"][0],
                "context": resolved["context"][0],
                "layout": resolved["layout"][0],
                "num_predict": resolved["num_predict"][0],
                "num_ctx": resolved["num_ctx"][0],
                "dataset": scenes.pool.get(sid) or resolved["dataset"][0],
                "run": resolved["run"][0],
                "machine": resolved["machine"][0],
                "backend": r.get("endpoint") or resolved["backend"][0],
                "block": scenes.block(sid) or "",
                # The scene label is a covariate, not a condition: the pre-RCBD
                # pool is labelled by what was posed ("posed-4robots",
                # "run-6robot-yieldtest", "empty-table"), which is the scene
                # difficulty rather than a treatment. Counting accuracy in
                # particular is only interpretable against robot_count.
                "scene_label": scenes.label(sid) or "",
                "robot_count": scenes.robot_count(sid),
                "sample": sid,
                "tick": r.get("tick", ""),
                "question_id": r.get("question_id"),
                "category": r.get("category"),
                "family": fam,
                "core": fam not in NON_CORE_FAMILIES,
                "expected": r.get("expected"),
                # What the model actually committed to, read with the same
                # extractors the grader used. Sits next to `expected` so
                # "said Alvik3, answer was Alvik5" is legible without opening
                # the raw text. On a correct row it should equal expected.
                "given": given,
                "answer": (r.get("answer") or "")[:2000],
                "graded": graded,
                "correct": correct,
                "failure": ("" if (correct or not graded)
                            else classify_failure(r, fam, robots, given)),
                "seconds": r.get("seconds"),
                "eval_count": r.get("eval_count"),
                "prompt_eval_count": r.get("prompt_eval_count"),
                "done_reason": r.get("done_reason"),
                "error": r.get("error"),
                "repeat": r.get("repeat", 0),
            }
            master.append(rec)
    return master, conditions


# --------------------------------------------------------------------------
# Summaries
# --------------------------------------------------------------------------
def pct(ok: int, n: int) -> float | str:
    return round(100.0 * ok / n, 1) if n else ""


def accuracy_table(master: list[dict], keys: list[str]) -> list[dict]:
    agg = collections.defaultdict(lambda: [0, 0])
    for r in master:
        if not r.get("graded", True):
            continue
        k = tuple(r.get(x) for x in keys)
        agg[k][0] += int(r["correct"])
        agg[k][1] += 1
    out = []
    for k, (ok, n) in sorted(agg.items(), key=lambda kv: [str(x) for x in kv[0]]):
        row = dict(zip(keys, k))
        row.update({"correct": ok, "n": n, "accuracy_pct": pct(ok, n)})
        out.append(row)
    return out


def latency_table(master: list[dict]) -> list[dict]:
    """Latency, keyed by MACHINE as well as model.

    Kept out of every accuracy table on purpose. Inference speed is a
    property of the box that served it, so two models timed on different
    computers are not comparable numbers and must not end up in one column.
    """
    agg = collections.defaultdict(list)
    for r in master:
        s = r.get("seconds")
        if isinstance(s, (int, float)):
            agg[(r["machine"], r["backend"], r["model"], r["suite"],
                 r["family"])].append(float(s))
    out = []
    for k, vals in sorted(agg.items(), key=lambda kv: [str(x) for x in kv[0]]):
        vals.sort()
        def q(p):
            if not vals:
                return ""
            return round(vals[min(len(vals) - 1, int(p * len(vals)))], 3)
        out.append(dict(zip(("machine", "backend", "model", "suite", "family"), k),
                        n=len(vals), p50=q(.50), p90=q(.90), p95=q(.95),
                        mean=round(sum(vals) / len(vals), 3), max=round(vals[-1], 3)))
    return out


def failure_table(master: list[dict]) -> list[dict]:
    agg = collections.Counter()
    totals = collections.Counter()
    for r in master:
        if not r.get("graded", True):
            continue
        totals[(r["model"], r["family"])] += 1
        if not r["correct"]:
            agg[(r["model"], r["family"], r["failure"])] += 1
    out = []
    for (model, fam, fail), n in sorted(agg.items(), key=lambda kv: -kv[1]):
        out.append({"model": model, "family": fam, "failure": fail,
                    "n": n, "of_asked": totals[(model, fam)],
                    "share_of_asked_pct": pct(n, totals[(model, fam)])})
    return out


#: Families whose `expected` is a single scalar, so given-vs-expected is a
#: fair equality test. Set-valued answers (closest pair, occupancy lists) and
#: tolerance-graded ones (yaw, position) are excluded: they are not wrong for
#: failing string equality.
SCALAR_FAMILIES = {"counting", "orientation", "relation", "control",
                   "fleet_traffic", "fleet_dispatch", "fleet_recovery"}


def given_agreement(master: list[dict]) -> list[dict]:
    """How often the extracted `given` agrees with the recorded `correct`.

    A self-check on this module, not on the models. `given` is only useful if
    it says the same thing the grader said: on a correct row it should equal
    `expected`, and on an incorrect row it should not. Where those disagree,
    the extractor is reading the answer differently from the grader and any
    failure label built on it is suspect -- so the rate is reported rather
    than assumed. Non-zero is expected and fine; large is a bug.
    """
    agg = collections.defaultdict(lambda: collections.Counter())
    for r in master:
        if not r.get("graded", True) or r["family"] not in SCALAR_FAMILIES:
            continue
        accepted = {p.strip().lower()
                    for p in _exp_str(r["expected"]).split(",") if p.strip()}
        giv = str(r["given"] or "").strip().lower()
        hit = giv in accepted if accepted else (giv == "")
        a = agg[(r["suite"], r["family"])]
        a["n"] += 1
        if r["correct"]:
            a["correct_and_given_matches" if hit
              else "correct_but_given_differs"] += 1
        else:
            a["wrong_and_given_differs" if not hit
              else "wrong_but_given_matches"] += 1
    out = []
    for (suite, fam), a in sorted(agg.items()):
        bad = a["correct_but_given_differs"] + a["wrong_but_given_matches"]
        out.append({"suite": suite, "family": fam, "n": a["n"],
                    "agree": a["n"] - bad,
                    "disagree": bad,
                    "disagree_pct": pct(bad, a["n"]),
                    "correct_but_given_differs": a["correct_but_given_differs"],
                    "wrong_but_given_matches": a["wrong_but_given_matches"]})
    return out


def failure_examples(master: list[dict], per_mode: int = 4) -> list[dict]:
    """Verbatim answers, a few per (model, family, failure).

    The taxonomy says a failure was "wrong_robot"; this says the model
    answered Alvik2 when the answer was Alvik5, and shows the sentence it
    said it in. Without it every category still has to be spot-checked by
    hand against the 16MB master table before it can be quoted.

    Sampled across the whole failure set rather than taking the first few, so
    the examples are not all from one model's first sample.
    """
    buckets = collections.defaultdict(list)
    for r in master:
        if r.get("graded", True) and not r["correct"] and r["failure"]:
            buckets[(r["model"], r["family"], r["failure"])].append(r)
    out = []
    for key, rows in sorted(buckets.items()):
        step = max(1, len(rows) // per_mode)
        for r in rows[::step][:per_mode]:
            answer = " ".join((r["answer"] or "").split())
            out.append({
                "model": r["model"], "family": r["family"],
                "failure": r["failure"], "occurrences": len(rows),
                "question_id": r["question_id"],
                "expected": r["expected"], "given": r["given"],
                "answer_excerpt": answer[:300],
                "answer_chars": len(r["answer"] or ""),
                "done_reason": r["done_reason"], "error": r["error"],
                "scene_label": r["scene_label"],
                "robot_count": r["robot_count"],
                "source_file": r["source_file"], "row_index": r["row_index"],
            })
    return out


#: Families whose answer is a small closed set, so expected-vs-given is a
#: readable cross-tab. Continuous answers (yaw, position) are not: their
#: failure sizes are already in the taxonomy as near-miss / reversed / wrong.
CONFUSABLE = {"counting", "orientation", "spatial", "relation", "control",
              "occupancy", "fleet_traffic", "fleet_dispatch",
              "fleet_recovery"}


def confusion(master: list[dict]) -> list[dict]:
    """expected -> given counts, per model and family.

    Answers "when it got the robot wrong, WHICH robot did it say" and "when
    it miscounted, did it say 5 or 7" directly, which the failure label alone
    cannot. An empty `given` is kept as "(no commitment)" rather than dropped
    -- how often a model fails to commit at all is part of the picture.
    """
    agg = collections.Counter()
    for r in master:
        if not r.get("graded", True) or r["family"] not in CONFUSABLE:
            continue
        exp = _exp_str(r["expected"]) or "(none)"
        giv = str(r["given"] or "").strip() or "(no commitment)"
        agg[(r["model"], r["family"], exp, giv, bool(r["correct"]))] += 1
    out = []
    for (model, fam, exp, giv, ok), n in sorted(
            agg.items(), key=lambda kv: (kv[0][0], kv[0][1], -kv[1])):
        out.append({"model": model, "family": fam, "expected": exp,
                    "given": giv, "correct": ok, "n": n})
    return out


# --------------------------------------------------------------------------
# Paired contrasts -- the part that answers the Wednesday questions
# --------------------------------------------------------------------------
CONTRASTS = {
    # name: (column, high arm, low arm)  -- reported as high MINUS low
    "overlay": ("overlay", "on", "off"),
    "jpeg": ("jpeg_quality", 95, 75),
    "blank_image": ("blank_image", True, False),
    "context": ("context", "state+image", "state-only"),
}


def _block_key(r: dict) -> tuple:
    """The unit a paired difference is taken over.

    Block, not scene: the same placement was captured under both overlay
    arms, so pairing on the block compares like with like. Falling back to
    the sample id for pools that were never blocked keeps those runs in the
    pooled difference while leaving them out of the by-block test.
    """
    return (r["block"] or ("sample:" + r["sample"]),)


def contrast(master: list[dict], name: str, model: str | None = None,
             suite: str = "static") -> dict | None:
    """Paired high-minus-low difference, overall and decomposed by family.

    Two numbers are reported for a reason. The POOLED difference is what a
    reader means by "it improved by X points". The BY-BLOCK difference is
    what carries a p-value: each block contributes one paired difference, so
    the test respects the design instead of treating 2000 correlated
    questions as 2000 independent trials.

    `share_of_overall_pct` is the decomposition the whole table exists for:
    each family's contribution to the pooled difference, as a percentage of
    that difference. It is what turns "95 beat 75 by 4 points" into "and
    almost all of it was orientation".
    """
    if name not in CONTRASTS:
        raise SystemExit(f"unknown contrast {name!r}; have {sorted(CONTRASTS)}")
    col, hi, lo = CONTRASTS[name]
    rows = [r for r in master if r["suite"] == suite
            and r.get("graded", True)
            and (model is None or r["model"] == model)]
    # Everything that is NOT the factor under test must be held fixed, or a
    # difference in something else gets attributed to it.
    held = [c for c in ("overlay", "jpeg_quality", "blank_image", "context",
                        "layout", "num_predict", "num_ctx", "dataset") if c != col]

    def arm_of(r):
        v = r[col]
        if v == hi:
            return "hi"
        if v == lo:
            return "lo"
        return None

    cells = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))
    for r in rows:
        arm = arm_of(r)
        if arm is None:
            continue
        key = (r["model"], r["question_id"], _block_key(r),
               tuple(r[h] for h in held))
        cells[key][arm][0] += int(r["correct"])
        cells[key][arm][1] += 1

    paired = [(k, v["hi"], v["lo"]) for k, v in cells.items()
              if v["hi"][1] and v["lo"][1]]
    if not paired:
        return None

    fam_of_qid = {}
    for r in rows:
        fam_of_qid.setdefault(r["question_id"], r["family"])

    def summarise(subset) -> dict:
        hi_ok = sum(h[0] for _, h, _ in subset)
        hi_n = sum(h[1] for _, h, _ in subset)
        lo_ok = sum(l[0] for _, _, l in subset)
        lo_n = sum(l[1] for _, _, l in subset)
        if not hi_n or not lo_n:
            return {}
        hi_pct = 100.0 * hi_ok / hi_n
        lo_pct = 100.0 * lo_ok / lo_n
        # by-block: one paired difference per block
        byblock = collections.defaultdict(lambda: [0, 0, 0, 0])
        for k, h, l in subset:
            b = k[2][0]
            byblock[b][0] += h[0]; byblock[b][1] += h[1]
            byblock[b][2] += l[0]; byblock[b][3] += l[1]
        diffs = [100.0 * (a / b) - 100.0 * (c / d)
                 for a, b, c, d in byblock.values() if b and d]
        p = ""
        if len(diffs) >= 6 and any(abs(x) > 1e-9 for x in diffs):
            try:
                from scipy import stats
                p = round(float(stats.wilcoxon(diffs).pvalue), 5)
            except Exception:
                p = ""
        ci = ""
        if len(diffs) > 2:
            try:
                import statistics as _st
                from scipy import stats as _sp
                sem = _st.stdev(diffs) / (len(diffs) ** 0.5)
                t = _sp.t.ppf(0.975, len(diffs) - 1)
                mu = _st.fmean(diffs)
                # float() not numpy scalars: json.dump cannot serialise
                # np.float64 and the failure surfaces far from here.
                ci = [round(float(mu - t * sem), 3),
                      round(float(mu + t * sem), 3)]
            except Exception:
                ci = ""
        return {
            "hi_correct": hi_ok, "hi_n": hi_n, "hi_pct": round(hi_pct, 1),
            "lo_correct": lo_ok, "lo_n": lo_n, "lo_pct": round(lo_pct, 1),
            "delta_pp": round(hi_pct - lo_pct, 2),
            "block_diffs": [round(x, 4) for x in diffs],
            "ci95": ci,
            "blocks": len(diffs),
            "by_block_mean_delta_pp": round(sum(diffs) / len(diffs), 2) if diffs else "",
            "wilcoxon_p": p,
        }

    overall = summarise(paired)
    families: dict[str, dict] = {}
    for fam in sorted({fam_of_qid.get(k[1], "unknown") for k, _, _ in paired}):
        subset = [t for t in paired if fam_of_qid.get(t[0][1]) == fam]
        s = summarise(subset)
        if s:
            families[fam] = s

    # Holm across families, so a family-level claim survives having looked
    # at seven of them.
    ps = sorted(((f, s["wilcoxon_p"]) for f, s in families.items()
                 if isinstance(s["wilcoxon_p"], float)), key=lambda x: x[1])
    m = len(ps)
    running = 0.0
    for i, (f, p) in enumerate(ps):
        adj = min(1.0, max(running, (m - i) * p))
        running = adj
        families[f]["holm_p"] = round(adj, 5)

    # Contribution of each family to the overall pooled delta.
    total_n = overall.get("hi_n", 0) + overall.get("lo_n", 0)
    denom = overall.get("delta_pp") or 0.0
    # A share of a near-zero overall delta is arithmetically fine and
    # useless: e4b's overlay delta is +0.12pp, which made one family "165% of
    # overall". Below this floor the decomposition is suppressed -- the honest
    # statement is "nothing moved", not a percentage of nothing.
    SHARE_FLOOR_PP = 0.5
    for fam, s in families.items():
        w = (s["hi_n"] + s["lo_n"]) / total_n if total_n else 0.0
        contrib = w * s["delta_pp"]
        s["weight_pct"] = round(100.0 * w, 1)
        s["contribution_pp"] = round(contrib, 2)
        s["share_of_overall_pct"] = (
            round(100.0 * contrib / denom, 1)
            if abs(denom) >= SHARE_FLOOR_PP else "")
        # Fewer than 30 instances in either arm cannot carry a family-level
        # claim. _spread() rotates which relation questions each sample gets,
        # so relation pairs up far less often than the others.
        s["underpowered"] = min(s["hi_n"], s["lo_n"]) < 30

    core = [t for t in paired
            if fam_of_qid.get(t[0][1]) not in NON_CORE_FAMILIES]
    return {"contrast": name, "column": col, "hi": hi, "lo": lo,
            "suite": suite, "model": model or "ALL",
            "overall": overall, "core_only": summarise(core),
            "families": families}


def explain(res: dict) -> list[str]:
    """The contrast in sentences, ordered by who moved the needle."""
    if not res:
        return ["no paired data for that contrast"]
    o, c = res["overall"], res.get("core_only") or {}
    lines = [
        f"{res['contrast']}: {res['hi']} minus {res['lo']}   "
        f"model={res['model']}  suite={res['suite']}",
        f"  overall      {o['lo_pct']:>5.1f}% -> {o['hi_pct']:>5.1f}%   "
        f"{o['delta_pp']:+.2f} pp   (n={o['lo_n']}+{o['hi_n']}, "
        f"{o['blocks']} blocks, Wilcoxon p={o['wilcoxon_p']})",
    ]
    if c:
        lines.append(
            f"  core only    {c['lo_pct']:>5.1f}% -> {c['hi_pct']:>5.1f}%   "
            f"{c['delta_pp']:+.2f} pp   "
            "(excludes orientation, both readback families, controls)")
    if abs(o.get("delta_pp") or 0) < 0.5:
        lines.append("  overall move is below 0.5 pp -- treat this as no "
                     "effect; per-family shares are suppressed")
    lines.append("  by family, largest contribution to the overall move first:")
    fams = sorted(res["families"].items(),
                  key=lambda kv: -abs(kv[1].get("contribution_pp") or 0))
    for fam, s in fams:
        share = s.get("share_of_overall_pct")
        share_s = f"{share:>6}%" if share != "" else "     --"
        lines.append(
            f"    {fam:<20} {s['lo_pct']:>5.1f}% -> {s['hi_pct']:>5.1f}%  "
            f"{s['delta_pp']:+6.2f} pp  n={s['lo_n']}+{s['hi_n']:<5} "
            f"contributes {s['contribution_pp']:+5.2f} pp = {share_s} of overall"
            f"   p={s.get('wilcoxon_p','')} holm={s.get('holm_p','')}"
            + ("   [UNDERPOWERED n<30]" if s.get("underpowered") else ""))
    return lines


# --------------------------------------------------------------------------
# Screening -- pick the 4-5 models for the expensive run
# --------------------------------------------------------------------------
CELL_COLUMNS = ("context", "overlay", "jpeg_quality", "blank_image",
                "layout", "num_predict", "num_ctx")


def condition_cell(r: dict) -> tuple:
    return tuple(r[c] for c in CELL_COLUMNS)


def screen(master: list[dict], top: int = 5, ceiling_sec: float = 5.0,
           cell: tuple | None = None) -> tuple[list[dict], tuple, int]:
    """Rank models on existing results so the 2x2 runs on a few, not all 13.

    Screens INSIDE ONE CONDITION CELL. Pooling a model's rows across every
    file it appears in looks like more data and is not: qwen3.5:4b appears in
    16 files spanning state-only and state+image, two machines and different
    per-category caps, so its pooled accuracy is an average over conditions no
    other model was measured under. Ranked that way it came first; inside the
    one cell where ten models were actually measured against each other it
    does not. The cell used is the one covering the most models, and it is
    returned so the caller can name it.

    Accuracy is core-only (see NON_CORE_FAMILIES) because a model that is
    merely bad at reading angles back should not be screened out for it.
    Latency is p90, not mean: a fleet waiting on an answer waits out the tail.
    """
    static = [r for r in master if r["suite"] == "static"]
    if cell is None:
        by_cell = collections.defaultdict(set)
        for r in static:
            by_cell[condition_cell(r)].add(r["model"])
        if not by_cell:
            return [], (), 0
        cell = max(by_cell.items(), key=lambda kv: (len(kv[1]), kv[0] is not None))[0]
    rows = [r for r in static if condition_cell(r) == cell]

    agg = collections.defaultdict(lambda: {"ok": 0, "n": 0, "core_ok": 0,
                                           "core_n": 0, "sec": [],
                                           "machines": set(), "files": set()})
    for r in rows:
        if not r.get("graded", True):
            continue
        a = agg[r["model"]]
        a["n"] += 1
        a["ok"] += int(r["correct"])
        if r["core"]:
            a["core_n"] += 1
            a["core_ok"] += int(r["correct"])
        if isinstance(r.get("seconds"), (int, float)):
            a["sec"].append(float(r["seconds"]))
        a["machines"].add(r["machine"])
        a["files"].add(r["source_file"])
    out = []
    for model, a in agg.items():
        sec = sorted(a["sec"])
        p90 = sec[min(len(sec) - 1, int(.90 * len(sec)))] if sec else None
        mach = sorted(str(m) for m in a["machines"])
        out.append({
            "model": model,
            "core_accuracy_pct": pct(a["core_ok"], a["core_n"]),
            "overall_accuracy_pct": pct(a["ok"], a["n"]),
            "n_core": a["core_n"], "n_total": a["n"],
            "p50_sec": round(sec[len(sec) // 2], 2) if sec else "",
            "p90_sec": round(p90, 2) if p90 is not None else "",
            "under_5s_ceiling": (p90 is not None and p90 <= ceiling_sec),
            "machines": "|".join(mach),
            "latency_comparable": len(mach) == 1 and mach[0] != "unknown",
            "n_source_files": len(a["files"]),
        })
    out.sort(key=lambda r: -(r["core_accuracy_pct"] or 0))
    for i, r in enumerate(out):
        r["rank_by_core_accuracy"] = i + 1
        r["shortlist"] = (r["rank_by_core_accuracy"] <= top
                          and r["under_5s_ceiling"])
    return out, cell, len(rows)


def screen_groups(master: list[dict], top: int = 5,
                  min_models: int = 3) -> list[dict]:
    """Rank within EVERY comparable cell, not just the richest one.

    Screening one cell alone drops models that were never measured in it:
    gemma4:e4b-it-qat, minicpm-v4.6:1b and qwen3.5:0.8b exist only in the RCBD
    pool, and e4b is the highest scorer there. Ranking each cell separately
    and tagging rows with the cell keeps every model in the table while
    refusing to put two cells' numbers in one ranking.
    """
    static = [r for r in master if r["suite"] == "static"
              and r.get("graded", True)]
    by_cell = collections.defaultdict(set)
    for r in static:
        by_cell[condition_cell(r)].add(r["model"])
    cells = sorted((c for c, ms in by_cell.items() if len(ms) >= min_models),
                   key=lambda c: -len(by_cell[c]))
    out = []
    for i, cell in enumerate(cells, 1):
        rows, _c, n = screen(master, top, cell=cell)
        desc = ", ".join(f"{k}={v}" for k, v in zip(CELL_COLUMNS, cell))
        for r in rows:
            r["cell_id"] = f"C{i}"
            r["cell_models"] = len(by_cell[cell])
            r["cell"] = desc
            out.append(r)
    return out


def screen_all_conditions(master: list[dict]) -> list[dict]:
    """Every model x condition cell, so the confound above is visible."""
    agg = collections.defaultdict(lambda: [0, 0, 0, 0, []])
    for r in master:
        if r["suite"] != "static" or not r.get("graded", True):
            continue
        a = agg[(r["model"],) + condition_cell(r)]
        a[1] += 1
        a[0] += int(r["correct"])
        if r["core"]:
            a[3] += 1
            a[2] += int(r["correct"])
        if isinstance(r.get("seconds"), (int, float)):
            a[4].append(float(r["seconds"]))
    out = []
    for k, (ok, n, cok, cn, sec) in sorted(agg.items(),
                                           key=lambda kv: [str(x) for x in kv[0]]):
        sec.sort()
        out.append(dict(zip(("model",) + CELL_COLUMNS, k),
                        n=n, accuracy_pct=pct(ok, n),
                        n_core=cn, core_accuracy_pct=pct(cok, cn),
                        p50_sec=round(sec[len(sec) // 2], 2) if sec else "",
                        p90_sec=round(sec[min(len(sec) - 1, int(.9 * len(sec)))], 2)
                        if sec else ""))
    return out


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------
def write_csv(path: str, rows: list[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        open(path, "w", encoding="utf-8").close()
        return
    cols: list[str] = []
    for r in rows:
        for k in r:
            if k not in cols:
                cols.append(k)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_json(path: str, obj) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2, default=str)
        fh.write("\n")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=ROOT)
    ap.add_argument("--out-dir", default=None,
                    help="where to write (default <root>/analysis). Raw "
                         "benchmark files are never written to.")
    ap.add_argument("--only", nargs="*", default=None,
                    help="substring filter on source file paths")
    ap.add_argument("--seed-manifest", action="store_true",
                    help="write tools/vlm_runs.json stub and exit")
    ap.add_argument("--contrast", default=None,
                    help=f"print one contrast: {sorted(CONTRASTS)}")
    ap.add_argument("--model", default=None, help="restrict --contrast")
    ap.add_argument("--suite", default="static", choices=("static", "live"))
    ap.add_argument("--screen-top", type=int, default=5)
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    out_dir = args.out_dir or os.path.join(root, "analysis")
    scenes = Scenes(root, SAMPLE_DIRS)
    manifest = load_manifest(root)

    if args.seed_manifest:
        files = []
        for rel in discover(root):
            try:
                meta = json.load(open(os.path.join(root, rel), encoding="utf-8"))
            except Exception:
                continue
            if suite_of(meta, rel):
                files.append((rel, meta))
        path = seed_manifest(root, files)
        print(f"wrote {path} with {len(files)} entries -- edit the nulls")
        return

    master, conditions = load_all(root, scenes, manifest, args.only)
    if not master:
        raise SystemExit("no benchmark rows found")

    if args.contrast:
        res = contrast(master, args.contrast, args.model, args.suite)
        print("\n".join(explain(res)))
        return

    write_csv(os.path.join(out_dir, "vlm_master.csv"), master)
    write_json(os.path.join(out_dir, "vlm_master.json"), master)
    write_csv(os.path.join(out_dir, "conditions.csv"), conditions)
    write_csv(os.path.join(out_dir, "summary_model_family.csv"),
              accuracy_table(master, ["suite", "model", "family"]))
    write_csv(os.path.join(out_dir, "summary_model_condition_family.csv"),
              accuracy_table(master, ["suite", "model", "overlay",
                                      "jpeg_quality", "blank_image",
                                      "context", "family"]))
    write_csv(os.path.join(out_dir, "summary_model_overall.csv"),
              accuracy_table(master, ["suite", "model"]))
    write_csv(os.path.join(out_dir, "latency_by_machine.csv"),
              latency_table(master))
    write_csv(os.path.join(out_dir, "failures.csv"), failure_table(master))
    write_csv(os.path.join(out_dir, "failure_examples.csv"),
              failure_examples(master))
    write_csv(os.path.join(out_dir, "confusion.csv"), confusion(master))
    agree = given_agreement(master)
    write_csv(os.path.join(out_dir, "given_agreement.csv"), agree)
    shortlist, cell, n_cell = screen(master, args.screen_top)
    write_csv(os.path.join(out_dir, "screening.csv"),
              screen_groups(master, args.screen_top))
    write_csv(os.path.join(out_dir, "screening_all_conditions.csv"),
              screen_all_conditions(master))

    report_lines: list[str] = []
    contrasts_out = {}
    models = sorted({r["model"] for r in master if r["suite"] == "static"})
    for name in CONTRASTS:
        for m in [None] + models:
            res = contrast(master, name, m, "static")
            if not res:
                continue
            contrasts_out[f"{name}|{m or 'ALL'}"] = res
            report_lines += explain(res) + [""]
    write_json(os.path.join(out_dir, "contrasts.json"), contrasts_out)
    with open(os.path.join(out_dir, "contrasts.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(report_lines) + "\n")

    n_files = len({r["source_file"] for r in master})
    print(f"read {n_files} benchmark files, {len(master)} question-instances")
    print(f"wrote 14 files to {out_dir}  (no raw file was modified)")
    print(os.linesep
          + "screening cell (the condition covering the most models): "
          + ", ".join(f"{c}={v}" for c, v in zip(CELL_COLUMNS, cell))
          + f"   [{n_cell} rows, {len(shortlist)} models]")
    ung = sum(1 for r in master if not r.get("graded", True))
    if ung:
        print(f"{ung} row(s) were never auto-graded (correct=null) -- "
              f"excluded from accuracy, not counted wrong")
    worst = sorted(agree, key=lambda a: -(a["disagree_pct"] or 0))[:3]
    if worst and (worst[0]["disagree_pct"] or 0) > 0:
        print("given-vs-grader disagreement (self-check on the extractor): "
              + ", ".join(f"{a['family']} {a['disagree_pct']}%" for a in worst))

    unknown = [c["source_file"] for c in conditions
               if c.get("machine") in (None, "unknown")]
    if unknown:
        print(f"\n{len(unknown)} file(s) have no declared machine -- latency "
              f"from these is not comparable across runs. Fill "
              f"{MANIFEST}:")
        for u in unknown[:8]:
            print("   " + u)
    print()
    for line in report_lines[:24]:
        print(line)


if __name__ == "__main__":
    main()
