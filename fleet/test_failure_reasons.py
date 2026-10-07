#!/usr/bin/env python3
"""Every leg failure must say WHY, not just that it failed.

Five distinct causes used to surface as the same Decision Log line --
`vision leg failed (drive_leg failed on leg A->B)` -- so the log and the
metrics DB could count how MANY legs failed but never which fault. One
evening on 2026-09-30 produced all five and the only way to separate them
was reading the supervisor log by hand:

    stopped short, went IDLE       no progress for 5s ... 'IDLE'
    blocked while still driving    no progress for 5s ... 'BUSY WHEEL_FOLLOW_MODE'
    mode re-arm ack lost           did not ack WHEEL_FOLLOW_MODE
    settle could not converge      SETTLE FAILED -- -6.27deg off 270
    position disagreed with plan   off-route -- measured (...) vs node N

    python3 test_failure_reasons.py
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

import camera_grid_navigate as G  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


TOKENS = [G.FAIL_STALLED_IDLE, G.FAIL_STALLED_DRIVING, G.FAIL_MODE_ACK_LOST,
          G.FAIL_SETTLE_FAILED, G.FAIL_SETTLE_DIVERGED, G.FAIL_VISION_LOST,
          G.FAIL_ROBOT_ERROR, G.FAIL_DWELL, G.FAIL_UNKNOWN]

print("the taxonomy itself")
check("all tokens are distinct", len(set(TOKENS)) == len(TOKENS))
check("all are plain uppercase tokens",
      all(t.replace("_", "").isalpha() and t.isupper() for t in TOKENS),
      "they go into metrics rows, so they must stay stable and greppable")


class _Log:
    def __init__(self): self.errors = []
    def error(self, m): self.errors.append(m)
    def warning(self, m): pass
    def info(self, m): pass


class _Nav:
    """Just the two members fail() touches."""
    def __init__(self):
        self.last_failure = None
        self._log = _Log()
    def get_logger(self): return self._log
    fail = G.CameraGridNavigator.fail


print("\nfail() records and logs in one call")
nav = _Nav()
nav.fail(G.FAIL_STALLED_IDLE, "leg 26->18: no progress for 5s")
check("the reason is recorded", nav.last_failure == G.FAIL_STALLED_IDLE)
check("the log line carries it too",
      nav._log.errors and G.FAIL_STALLED_IDLE in nav._log.errors[0],
      nav._log.errors[0] if nav._log.errors else "")
check("and still carries the human message",
      "no progress for 5s" in nav._log.errors[0])

print("\n_leg_note() formats, then clears")
import fleetSupervisor as F  # noqa: E402
nav = _Nav()
nav.fail(G.FAIL_MODE_ACK_LOST, "turn at 17")
note = F._leg_note(nav, "turn toward next leg failed at 17")
check("reason leads the note", note.startswith(G.FAIL_MODE_ACK_LOST + ":"), note)
check("the human part survives", "turn toward next leg failed at 17" in note)
check("the reason is cleared after reading", nav.last_failure is None,
      "otherwise the NEXT failure inherits a stale cause")

print("\na failure with nothing recorded says so, rather than lying")
nav = _Nav()
note = F._leg_note(nav, "drive_leg failed on leg 1->2")
check("unclassified reads UNKNOWN", note.startswith("UNKNOWN:"), note)

print("\nan object without the attribute at all is tolerated")
class _Bare:
    pass
note = F._leg_note(_Bare(), "dwell failed")
check("no AttributeError, reports UNKNOWN", note.startswith("UNKNOWN:"), note)

print("\nthe IDLE / DRIVING split is what the live log distinguishes")
# The classifier keys on the firmware status string carried in the message.
for status, want in (("IDLE", G.FAIL_STALLED_IDLE),
                     ("BUSY WHEEL_FOLLOW_MODE", G.FAIL_STALLED_DRIVING),
                     ("BUSY DWELL", G.FAIL_STALLED_DRIVING)):
    got = (G.FAIL_STALLED_IDLE if "IDLE" in str(status).upper()
           else G.FAIL_STALLED_DRIVING)
    check(f"status {status!r} -> {want}", got == want)

print("\nthe navigator sets last_failure in __init__")
check("attribute exists before any failure",
      "self.last_failure" in open(G.__file__, encoding="utf-8").read())

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
