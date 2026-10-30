#!/usr/bin/env python3
"""A turn must not start if it would sweep into another robot.

Until 2026-09-30 a turn was checked against NOTHING. `decide()` dispatched
every non-move immediately with "movement gate deferred": a turn is not a
move, so it took no reservation, and a stationary robot has no closing speed
for the emergency brake to act on. A robot could rotate into a neighbour
with every guard in the system watching.

Two gates now cover it:

  SYMBOLIC  advisor.entry_rotation_footprint -- reasons about reservations,
            so it can hold a robot before it reaches a contentious node.
            Tested in test_entry_rotation.py.
  POSITIONAL  this one -- measured camera distance, so a robot that drifted
            off its node is judged on where it actually IS.

The threshold is the robot's own turning circle: 5.625in corner to corner,
measured on the hardware.

    python3 test_rotation_clearance.py
"""
import math
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

import fleetSupervisor as F           # noqa: E402
from camera_grid_navigate import DEPOT_SLOT_WORLD_IN as D   # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


class _Item:
    def __init__(self, kind="turn"): self.kind = kind


class _St:
    def __init__(self, kind="turn"):
        self.next_index = 0
        self.plan = [_Item(kind)]


class _Adv:
    def __init__(self, names, kind="turn"):
        self.robots = {n: _St(kind) for n in names}
        # The real advisor's wait-for registration. The guard calls this when
        # it holds a turn, so the EXISTING deadlock detector can see a
        # standoff that is not a reservation -- see the 2026-10-15 run-5
        # writeup in _rotation_clearance_hold(). Recorded here so the tests
        # below can assert the registration actually happens.
        self.vision_blocks = []

    def note_vision_block(self, robot, blocker):
        self.vision_blocks.append((robot, blocker))


class _Vis:
    def __init__(self, poses): self._p = poses
    def snapshot_in(self): return self._p


def hold(poses, who="Alvik1", kind="turn", vision=True, adv_out=None):
    sup = object.__new__(F.FleetSupervisor)
    sup.vision = _Vis(poses) if vision else None
    adv = _Adv(list(poses), kind)
    if adv_out is not None:
        adv_out.append(adv)
    return F.FleetSupervisor._rotation_clearance_hold(sup, adv, who)


parked = {f"Alvik{i}": (D[f"D{i}"][0], D[f"D{i}"][1], 0.0, 0.0)
          for i in range(1, 7)}

print(f"threshold is the measured turning circle: {F.ROTATION_SWEEP_IN}in")
check("it equals the corner-to-corner measurement",
      abs(F.ROTATION_SWEEP_IN - 5.625) < 1e-9)

print("\nthe depot row -- what the operator asked for")
check("six robots parked ON their slots may all rotate at once",
      hold(parked) is None,
      "slot pitch 6.30-6.50in vs 5.625in sweep")

p = dict(parked); p["Alvik1"] = (D["D1"][0] + 0.5, D["D1"][1], 0.0, 0.0)
check("one robot 0.5in off toward a neighbour still clears",
      hold(p) is None, "6.30 - 0.5 = 5.80in, +0.175in spare")

p = dict(parked)
p["Alvik5"] = (D["D5"][0] + 0.5, D["D5"][1], 0.0, 0.0)
p["Alvik6"] = (D["D6"][0] - 0.5, D["D6"][1], 0.0, 0.0)
r = hold(p, "Alvik5")
check("BOTH 0.5in off toward each other is refused", r is not None,
      "5.30in -- overlaps by 0.325in")
check("and the message says by how much", r and "short by" in r, r or "")

# The real budget, stated as a property rather than a magic number, at
# whichever pair of slots is tightest in testbed_coordinates.json (D5-D6 at
# 6.30in until 2026-10-22; D1-D2 at 6.19in after the meter-stick re-set).
pairs = [(f"D{k}", f"D{k+1}") for k in range(1, 6)]
lo, hi = min(pairs, key=lambda ab: math.hypot(D[ab[1]][0] - D[ab[0]][0],
                                              D[ab[1]][1] - D[ab[0]][1]))
ra, rb = f"Alvik{lo[1:]}", f"Alvik{hi[1:]}"
pitch = math.hypot(D[hi][0] - D[lo][0], D[hi][1] - D[lo][1])
budget = pitch - 5.625
for frac in (0.90, 0.97):
    off = budget * frac / 2
    p = dict(parked)
    p[ra] = (D[lo][0] + off, D[lo][1], 0.0, 0.0)
    p[rb] = (D[hi][0] - off, D[hi][1], 0.0, 0.0)
    check(f"{lo}-{hi}: each {off:.2f}in off (total {2*off:.2f}in) still clears",
          hold(p, ra) is None, f"budget is {budget:.3f}in at the tightest pair")
off = budget * 1.06 / 2
p = dict(parked)
p[ra] = (D[lo][0] + off, D[lo][1], 0.0, 0.0)
p[rb] = (D[hi][0] - off, D[hi][1], 0.0, 0.0)
check(f"{lo}-{hi}: each {off:.2f}in off (total {2*off:.2f}in) is refused",
      hold(p, ra) is not None)

print("\nthe 2026-09-30 collision geometry")
r = hold({"Alvik3": (0.0, 0.0, 0.0, 0.0), "Alvik4": (5.00, 0.0, 0.0, 0.0)},
         "Alvik3")
check("entry-to-lattice at 5.00in is refused", r is not None, r or "")
r = hold({"Alvik3": (0.0, 0.0, 0.0, 0.0), "Alvik4": (4.99, 0.0, 0.0, 0.0)},
         "Alvik3")
check("entry-to-workstation at 4.99in is refused", r is not None)
r = hold({"Alvik3": (0.0, 0.0, 0.0, 0.0), "Alvik4": (10.00, 0.0, 0.0, 0.0)},
         "Alvik3")
check("lattice-to-lattice at 10.00in is allowed", r is None,
      "+4.38in clear -- must not gate ordinary turns")

print("\nit only gates TURNS")
check("a move is not gated by this",
      hold({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (1.0, 0.0, 0.0, 0.0)},
           kind="move") is None,
      "moves have their own corridor and reservation gates")

print("\nit never guesses from missing data")
check("no vision at all -> defer",
      hold({"Alvik1": (0.0, 0.0, 0.0, 0.0)}, vision=False) is None)
check("my own pose stale -> defer",
      hold({"Alvik1": (0.0, 0.0, 0.0, 99.0), "Alvik2": (1.0, 0.0, 0.0, 0.0)})
      is None)
check("the OTHER robot's pose stale -> defer on that robot",
      hold({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (1.0, 0.0, 0.0, 99.0)})
      is None,
      "a stale fix is not evidence of a clear lane, but it is not evidence "
      "of a blocked one either")
check("alone on the table -> allowed",
      hold({"Alvik1": (0.0, 0.0, 0.0, 0.0)}) is None)

print("\nit reports the CLOSEST offender when several are near")
r = hold({"Alvik1": (0.0, 0.0, 0.0, 0.0),
          "Alvik2": (5.5, 0.0, 0.0, 0.0),
          "Alvik3": (4.0, 0.0, 0.0, 0.0)})
check("names Alvik3, the nearer one", r is not None and "Alvik3" in r, r or "")

print("\na hold must be VISIBLE to the deadlock detector")
# 2026-10-15 run 5: Alvik1 stalled and stopped dead on node 18. This guard
# correctly refused to let Alvik3 rotate into it -- about every 3 s for
# 115 s, while four more robots queued up behind. The run ended in a
# five-robot standstill with deadlocks_detected = 0, because a positional
# hold is not a reservation and the detector saw an empty graph. The
# corridor gate had the identical bug fixed on 2026-09-29 by recording the
# hold; this guard, added a day later, did not get it.
seen = []
r = hold({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (4.0, 0.0, 0.0, 0.0)},
         who="Alvik1", adv_out=seen)
check("the turn is still refused", r is not None, r or "")
check("and the standoff is registered in the wait-for graph",
      seen and seen[0].vision_blocks == [("Alvik1", "Alvik2")],
      str(seen[0].vision_blocks if seen else "no advisor captured"))

seen = []
hold({"Alvik1": (0.0, 0.0, 0.0, 0.0)}, who="Alvik1", adv_out=seen)
check("a clear lane registers nothing",
      seen and seen[0].vision_blocks == [],
      str(seen[0].vision_blocks if seen else "no advisor captured"))

seen = []
hold({"Alvik1": (0.0, 0.0, 0.0, 0.0), "Alvik2": (4.0, 0.0, 0.0, 0.0)},
     who="Alvik1", kind="move", adv_out=seen)
check("a MOVE item is not a turn, so nothing is registered",
      seen and seen[0].vision_blocks == [],
      str(seen[0].vision_blocks if seen else "no advisor captured"))

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
