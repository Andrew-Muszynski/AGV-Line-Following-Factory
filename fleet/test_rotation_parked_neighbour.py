#!/usr/bin/env python3
"""A turn next to a PARKED, square-on neighbour is judged against its real
footprint, not against a second turning circle.

    python3 test_rotation_parked_neighbour.py

ALNS24_seq, November 3, 2026, 13:13: Alvik5 back in depot slot D5 held its
square-up for 88s -- "Alvik4 is 5.61in away, a turn sweeps 5.625in" -- until
the stall watchdog ended the run. Alvik4 was parked square-on; Alvik5's
sweep clears it by 0.78in, and the operator's hand-sent ROTATE_REL 167.7
turned it without contact.
"""
import sys
import types

for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

import fleetSupervisor as F         # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Vis:
    def __init__(self, poses, speeds, turning=(), driving=()):
        self.p, self.s = poses, speeds
        self.t, self.d = set(turning), set(driving)

    def snapshot_in(self):
        return self.p

    def speed_in(self, n):
        return self.s.get(n)

    def is_turning(self, n):
        return n in self.t

    def driving_leg(self, n):
        return n in self.d

    def claim_rotation(self, n, sec=None):
        self.t.add(n)


def sup(vis):
    s = object.__new__(F.FleetSupervisor)
    s.vision = vis
    s.advisor = None
    return s


# Telemetry at the hold (inches, navigator yaw).
A5 = (44.4, 10.8, -167.7, 0.0)
A4 = (38.8, 10.6, 0.88, 0.0)
still = {"Alvik4": 0.1, "Alvik5": 0.1}

print("1. who blocks a turn")
b = sup(_Vis({"Alvik5": A5, "Alvik4": A4}, still)).rotation_sweep_blocker("Alvik5")
check("the 2026-11-03 D5 case: parked square-on Alvik4 does not block",
      b is None, str(b))
gap = F.footprint_gap_in(A4[0] - A5[0], A4[1] - A5[1], None, A4[2])
check("its real clearance is about 0.8in", 0.6 < gap < 0.9, f"{gap:.2f}in")
check("Alvik4 turning: blocks (two circles)",
      sup(_Vis({"Alvik5": A5, "Alvik4": A4}, still, turning={"Alvik4"}))
      .rotation_sweep_blocker("Alvik5") is not None)
check("Alvik4 on a leg: blocks",
      sup(_Vis({"Alvik5": A5, "Alvik4": A4}, still, driving={"Alvik4"}))
      .rotation_sweep_blocker("Alvik5") is not None)
check("Alvik4 moving: blocks",
      sup(_Vis({"Alvik5": A5, "Alvik4": A4}, {"Alvik4": 2.0}))
      .rotation_sweep_blocker("Alvik5") is not None)
check("Alvik4 speed unknown: blocks",
      sup(_Vis({"Alvik5": A5, "Alvik4": A4}, {}))
      .rotation_sweep_blocker("Alvik5") is not None)
skew = (38.8, 10.6, 30.0, 0.0)
check("Alvik4 parked 30deg skewed: its corner reaches the sweep -> blocks",
      sup(_Vis({"Alvik5": A5, "Alvik4": skew}, still))
      .rotation_sweep_blocker("Alvik5") is not None)
close = (39.6, 10.6, 0.0, 0.0)
check("parked square-on but 4.8in away -> blocks",
      sup(_Vis({"Alvik5": A5, "Alvik4": close}, still))
      .rotation_sweep_blocker("Alvik5") is not None)

print("\n2. two neighbours asking at once: only one turns")
v = _Vis({"Alvik5": A5, "Alvik4": A4}, still)
s = sup(v)
logs = types.SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)
s.get_logger = lambda: logs
s._event = lambda *a, **k: None
ok5, _ = F.FleetSupervisor.await_rotation_clearance(s, "Alvik5", "test",
                                                    timeout_sec=0.0)
check("Alvik5 is let turn", ok5)
ok4, why = F.FleetSupervisor.await_rotation_clearance(s, "Alvik4", "test",
                                                      timeout_sec=0.0)
check("Alvik4, asking while Alvik5 turns, is held", not ok4, why)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
sys.exit(1 if FAIL else 0)
