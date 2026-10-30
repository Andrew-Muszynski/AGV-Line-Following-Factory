#!/usr/bin/env python3
"""Waiting robots are not flooded, and a wait is logged once.

    python3 test_wait_traffic.py

2026-10-22, operator: one depot launch produced 64 events, the same five
waits repeating every 3 s while Alvik1 sat at node 0 failing its turn, and
asked whether the supervisor talks to the robots too much during a vision leg
failure. It did: every waiting robot was sent a zero wheel command on every
20 ms loop pass (~250 msg/s for five robots) although, since 2026-08-20,
waiting robots are IDLE and their firmware ignores wheel_cmd. Alvik1's turn
failures: 3.6% with no robot waiting, 10% with 1-2, 13% with 3-5.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "fleetSupervisor.py"), encoding="utf-8").read()
vision = src.split("Advisor mode (VISION DRIVE)", 1)[0].rsplit("def log_wait", 1)[1]

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


print("\n1. keep-alive only for a robot still in wheel-follow, at 10 Hz")
block = src.split('elif phase[name] == "waiting":', 1)[1].split('elif phase[name] == "busy":', 1)[0]
check("no unconditional send_wheel(0,0) left in the waiting branch",
      not re.search(r"^\s*worker\.nav\.send_wheel\(0\.0, 0\.0\)\s*$", block, re.M))
check("gated on the firmware having reported BUSY WHEEL_FOLLOW", 'startswith(\n                                "BUSY WHEEL_FOLLOW")' in block
      or '"BUSY WHEEL_FOLLOW"' in block)
check("rate-limited to 0.1 s", "keepalive_at.get(name, -1.0) >= 0.1" in block)

print("\n2. a wait is logged when it starts or changes, and closed once")
check("same reason -> not re-logged (no 3 s repeat)",
      "if previous == reason:\n                    return" in vision
      and "HOLD_LOG_THROTTLE_SEC" not in vision)
check("release line with the duration", "wait over after" in src and "def log_release" in src)
check("released when the robot is dispatched", "log_release(name)\n" in src)


# Behaviour of the dedupe rule itself, replayed on today's depot launch:
# five robots, same reason, rechecked every 0.2 s for 26 s.
def replay(rule):
    logged, last = 0, {}
    for k in range(int(26 / 0.2)):
        now = k * 0.2
        for r in ("Alvik2", "Alvik3", "Alvik4", "Alvik5", "Alvik6"):
            reason = f"depot lane: robot ahead of {r} has not cleared yet"
            if rule(last.get(r), reason, now):
                last[r] = (reason, now)
                logged += 1
    return logged


old = replay(lambda p, reason, now: not (p and p[0] == reason and now - p[1] < 3.0))
new = replay(lambda p, reason, now: not (p and p[0] == reason))
check("today's launch: one WAIT per robot instead of one every 3 s",
      new == 5 and old > 40, f"old rule {old} events, new rule {new}")

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
