#!/usr/bin/env python3
"""Every run records the turn method it actually used.

    python3 test_tuning_capture.py

2026-10-21: every run since 2026-10-15 recorded effective_turn_method
"unknown" -- the snapshot was taken at mission configure, before any
navigator existed -- and in listen mode the navigator args were kept from
the FIRST mission only, so a later Drive mode change was never recorded.
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import metrics_events as M  # noqa: E402
import tuning_snapshot as T  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


e = M.MetricsEmitter()
pushed = []
e.push_tuning = lambda args=None: pushed.append(T.effective_turn_method(args))
e._base_url = "http://x"

e._run_id = "run-1"
e.remember_nav_args(types.SimpleNamespace(mission_turn_method="rotate_rel", turn_method="staged"))
e.remember_nav_args(types.SimpleNamespace(mission_turn_method="rotate_rel", turn_method="staged"))
check("first navigator of a run re-sends the snapshot, once per run",
      pushed == ["rotate_rel"], str(pushed))

e._run_id = "run-2"                       # next mission, Drive mode changed
e.remember_nav_args(types.SimpleNamespace(mission_turn_method="staged", turn_method="staged"))
check("a later mission records ITS turn method, not the first one's",
      pushed == ["rotate_rel", "staged"], str(pushed))
check("the stored args are the newest", e._nav_args.mission_turn_method == "staged")

e._run_id = None
e.remember_nav_args(types.SimpleNamespace(mission_turn_method="staged"))
check("an untracked mission sends nothing", len(pushed) == 2)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
