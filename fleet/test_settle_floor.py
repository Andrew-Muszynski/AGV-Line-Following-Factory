#!/usr/bin/env python3
"""settle_floor_update(): stiction and overshoot must move the crawl floor
in OPPOSITE directions.

THE FAILURE THIS PINS (hardware, Alvik1 2026-09-30). The old test asked
whether abs(prev_err) - abs(err) < moved_eps -- the change in the MAGNITUDE
of the heading error. That is not how far the robot turned, and the two
disagree whenever a pass crosses the target. Going -2.86 -> +10.54deg is
13.4deg of real rotation, but the magnitude test reads -7.68, concludes the
robot never moved, and answers by RAISING the crawl:

    -2.86  -> +10.54deg   "did not move" -> raised to 8 RPM
    +10.54 -> -23.37deg   "did not move" -> raised to 12 RPM

Each escalation made the next overshoot bigger. The robot ended up 20deg+
off heading and drove off the taped grid onto the bare table.

Alvik1 was ALSO failing to stop rotating on command that run, which is what
kept triggering this -- but the amplifier is a genuine bug on its own and
would bite any robot that overshoots a settle pass.

    python3 test_settle_floor.py
"""
import sys
import types

# camera_grid_navigate imports rclpy at module scope; stub it so this stays a
# pure unit test with no ROS on the path.
for _n in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["std_msgs.msg"].String = object

from camera_grid_navigate import settle_floor_update  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


# Production values: settle_rpm_step 4.0, settle_moved_eps_deg 0.3.
STEP, EPS, MAX = 4.0, 0.3, 24.0


def upd(prev, err, floor=0.0, last=4.0):
    return settle_floor_update(prev, err, floor, last, STEP, MAX, EPS)


print("the exact live sequence that drove Alvik1 off the grid")
# Pass 1 -> 2: -2.86 to +10.54. Crossed the target and landed FURTHER out.
floor, action = upd(-2.86, 10.54, floor=8.0)
check("overshoot is recognised, not called stiction", action == "overshoot",
      f"(was 'stuck' -> raised RPM)")
check("and the floor comes DOWN", floor < 8.0, f"8.0 -> {floor}")

# Pass 2 -> 3: +10.54 to -23.37. Same shape, worse.
floor, action = upd(10.54, -23.37, floor=12.0)
check("second overshoot also lowers the floor",
      action == "overshoot" and floor < 12.0, f"12.0 -> {floor}")

print("\ngenuine stiction still raises the crawl")
floor, action = upd(5.00, 4.95, floor=0.0, last=4.0)
check("a pass that barely moved is stuck", action == "stuck",
      "turned 0.05deg")
check("and the floor goes UP", floor > 4.0, f"-> {floor}")
floor, action = upd(-5.00, -5.00, floor=0.0, last=6.0)
check("exactly zero movement is stuck", action == "stuck")

print("\nmovement below/above the epsilon is the dividing line")
_, a_below = upd(5.0, 5.0 - (EPS / 2))
_, a_above = upd(5.0, 5.0 - (EPS * 2))
check("just under moved_eps counts as stuck", a_below == "stuck")
check("just over moved_eps does not", a_above == "ok")

print("\na converging bounce is left alone")
# Crossed the target but landed CLOSER -- this is the settle working.
floor, action = upd(-4.00, 1.50, floor=6.0)
check("crossing the target while improving is not an overshoot",
      action == "ok", "-4.00 -> +1.50")
check("floor unchanged", floor == 6.0)

print("\nnormal same-side convergence is left alone")
floor, action = upd(8.0, 3.0, floor=6.0)
check("moving toward the target is 'ok'", action == "ok")
check("floor unchanged", floor == 6.0)

print("\nbounds")
floor, _ = upd(5.0, 5.0, floor=MAX, last=MAX)
check("the floor never exceeds rpm_max", floor <= MAX, f"-> {floor}")
floor, _ = upd(-1.0, 3.0, floor=1.0)
check("the floor never goes negative", floor >= 0.0, f"-> {floor}")

print("\nfirst pass has nothing to compare against")
floor, action = upd(None, 7.0, floor=5.0)
check("prev_err None is a no-op", action == "ok" and floor == 5.0)

print("\nthe old magnitude test would have failed these")
# Regression guard, stated as the property that actually matters: crossing
# the target and landing further out must never RAISE the floor.
bad = []
for prev, err in ((-2.86, 10.54), (10.54, -23.37), (-1.0, 5.0), (3.0, -9.0)):
    f, a = upd(prev, err, floor=8.0)
    if f > 8.0:
        bad.append((prev, err, f))
check("no overshoot anywhere raises the crawl floor", not bad, str(bad))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
