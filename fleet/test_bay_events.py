#!/usr/bin/env python3
"""The page's workstation colours come from what the robots actually did.

    python3 test_bay_events.py

Operator, 2026-10-30: "the html page will update the status of workstations
earlier than they are actually ready ... The status of the workstations in
the html must match reality". The page coloured bays off the plan's clock.
The supervisor now publishes a "bay" event at each real transition and the
page colours from those alone in Real/Live mode.
"""
import json
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = type("String", (), {})

import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))


sup = object.__new__(F.FleetSupervisor)
sup._event_seq = 0
sup._event_pub = _Pub()
sup._rpd_process_sec = 30.0

print("1. the event the page reads")
sup._bay_event(12, "processing", "Alvik3")
e = sup._event_pub.sent[-1]
check("kind 'bay', so the page keeps it out of the Decision Log",
      e["kind"] == "bay", str(e))
check("carries the solver bay number and state",
      e["data"]["bay"] == 12 and e["data"]["state"] == "processing")
check("processing carries process_sec, so the page can turn it READY",
      e["data"]["process_sec"] == 30.0)
sup._bay_event(12, "picked", "Alvik5")
check("picked has no process_sec", "process_sec" not in
      sup._event_pub.sent[-1]["data"])
sup._event("wait", "plain log line")
check("ordinary events are unchanged (no data field)",
      "data" not in sup._event_pub.sent[-1])

src = open(F.__file__, encoding="utf-8").read()
print("\n2. sent from the moments the supervisor itself acts on")
check("processing starts at the end of the drop's DWELL (2026-10-29 rule)",
      'self._drop_done[bay] = now\n' in src
      and src.index('self._drop_done[bay] = now')
      < src.index('self._bay_event(bay, "processing", name)',
                  src.index('self._drop_done[bay] = now')))
check("...or on arrival for plans without that rule",
      'elif not (getattr(self, "_rpd_after_drop", False)' in src)
check("picked when the collecting robot LEAVES the workstation",
      'self._bay_event(job.workstation, "picked",' in src)
check("delivered when there is no processing at all",
      'self._bay_event(bay, "delivered", name)' in src)

js = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "static", "js", "solver.js"), encoding="utf-8").read()
lm = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "static", "js", "live-monitor.js"), encoding="utf-8").read()
print("\n3. the page")
check("a live run colours from the live map in EVERY view, not the plan clock",
      "(liveDataMode() || state.liveRun)\n    ? liveBayStatus(id)" in js
      and js.count("BAY_STATUS_STYLE[bayStatusNow(id,") == 2
      and "BAY_STATUS_STYLE[bayStatusAt(" not in js)
check("bay events feed the live map and skip the Decision Log",
      'if (e.kind === "bay") {\n    state.liveRun = true;\n    ingestBayEvent(e.data);' in lm)
check("a new run clears the map",
      'e.text.startsWith("run started")' in lm)
check("loading another plan hands the colours back to that plan",
      lm.count("state.liveRun = false;") == 2)
check("dedupe survives a supervisor restart (boot stamp in the key)",
      "const key = `${e.boot ?? \"\"}:${e.seq}`;" in lm
      and '"boot": getattr(self, "_event_boot", 0),' in src)
e = sup._event_pub.sent[-1]
check("every event carries the boot stamp", "boot" in e)
html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                         "templates", "index.html"), encoding="utf-8").read()
check("the page's scripts carry a version tag (no stale solver.js)",
      html.count("?v={{ asset_v }}") == 6)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
