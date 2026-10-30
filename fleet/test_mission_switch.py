#!/usr/bin/env python3
"""Switching plans between robot counts must never need a restart.

    python3 test_mission_switch.py

DOE, 2026-10-23..27: after a 4-robot mission, Alvik5/6 coming back online
published IDLE on their status topics. _on_status indexed the CURRENT
mission's channels, raised KeyError, and the exception ended rclpy.spin's
thread -- so the next 6-robot mission and START were never heard (runs
f69cc590, 15919e3c, a742e135: created or started, zero events) until the
supervisor was restarted.
"""
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
sys.modules["std_msgs.msg"].String = object

import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


def channels(n):
    return {f"Alvik{k}": F.RobotChannel(name=f"Alvik{k}", commands=[])
            for k in range(1, n + 1)}


sup = object.__new__(F.FleetSupervisor)
sup._memit = lambda *a, **k: None

print("\n1. status from a robot outside the current mission")
sup._channels = channels(4)                  # a 4-robot mission is loaded
try:
    sup._on_status("Alvik5", types.SimpleNamespace(data="IDLE"))
    check("Alvik5 IDLE during a 4-robot mission is ignored, no exception", True)
except Exception as exc:                     # noqa: BLE001
    check("Alvik5 IDLE during a 4-robot mission is ignored, no exception",
          False, repr(exc))
check("and it does not sneak into the mission", "Alvik5" not in sup._channels)

print("\n2. once a mission names it, its status counts again")
sup._channels = channels(6)                  # the 6-robot mission arrives
sup._on_status("Alvik5", types.SimpleNamespace(data="IDLE"))
check("Alvik5 IDLE is recorded", sup._channels["Alvik5"].last_status == "IDLE")
check("and marks it alive", sup._channels["Alvik5"].last_seen > 0)

print("\n3. the spin thread survives a callback that raises")


class FakeLogger:
    def __init__(self):
        self.lines = []

    def error(self, text):
        self.lines.append(text)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


calls = {"spin": 0}


def fake_spin(node):
    calls["spin"] += 1
    if calls["spin"] <= 5:
        raise KeyError("Alvik5")              # a bad callback, five times
    fake_rclpy.alive = False                  # then a normal shutdown


fake_rclpy = types.SimpleNamespace(alive=True)
fake_rclpy.ok = lambda: fake_rclpy.alive
fake_rclpy.spin = fake_spin
real = F.rclpy
F.rclpy = fake_rclpy
try:
    node = FakeNode()
    F._spin_forever(node)
finally:
    F.rclpy = real
check("kept spinning through 5 exceptions until shutdown", calls["spin"] == 6,
      f"spin called {calls['spin']}x")
check("printed the first 3 in full, then stayed quiet",
      len(node.logger.lines) == 3
      and all("KeyError" in t and "spinning on" in t for t in node.logger.lines))

src = open(F.__file__, encoding="utf-8").read()
check("main() spins through _spin_forever, not bare rclpy.spin",
      "target=_spin_forever" in src and "target=rclpy.spin" not in src)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
