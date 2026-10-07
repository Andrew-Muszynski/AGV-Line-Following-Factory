#!/usr/bin/env python3
"""The collision rule, tested against the geometry of the real table.

Run directly (no ROS needed -- fleetSupervisor's rclpy imports are stubbed):

    python3 fleet/test_collision_guard.py

WHY THIS FILE EXISTS
--------------------
On 2026-09-29 Alvik5 drove into Alvik3 in the depot lane and the supervisor
reported it as a generic ABORT. Nothing detected it, because the only
proximity check worked in grid cells and filtered to the lattice, which
excludes the depot lane entirely. The rule that replaces it has to hold two
things at once that pull in opposite directions:

  * catch a pair that is about to touch, anywhere on the table, and
  * never fire on two robots correctly parked in adjacent depot slots, which
    are only 6.16in apart -- closer than any fixed "safe" margin would be.

That is why the avoid threshold is a function of closing speed, and why it is
worth testing rather than eyeballing.
"""
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _m in ("rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos",
           "rclpy.callback_groups", "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
sys.modules["rclpy"].ok = lambda: True
sys.modules["rclpy.node"].Node = type("Node", (), {"__init__": lambda s, *a, **k: None})
sys.modules["rclpy.executors"].SingleThreadedExecutor = object
sys.modules["rclpy.qos"].QoSProfile = object
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(
    BEST_EFFORT=0, RELIABLE=1)
sys.modules["rclpy.callback_groups"].ReentrantCallbackGroup = object
sys.modules["std_msgs.msg"].String = type("String", (), {"data": ""})

import fleetSupervisor as F  # noqa: E402

FAILS = []


def check(cond, msg, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {msg}" + (f"   {detail}" if detail else ""))
    if not cond:
        FAILS.append(msg)


print("1. contact is detected wherever it happens")
check(F.collision_verdict(5.04, 0.0) == "collision",
      "exactly at contact counts as a collision")
check(F.collision_verdict(3.0, 0.0) == "collision",
      "overlapping counts as a collision even at a standstill",
      "a robot shoved into another is not moving")
check(F.collision_verdict(4.9, -2.0) == "collision",
      "already touching while separating is still a collision")

print("\n2. correctly parked robots must NEVER trigger")
# Measured depot slot spacing, camera_grid_navigate.DEPOT_SLOT_WORLD_IN:
# D1..D6 at x = 19.64, 25.88, 32.20, 38.58, 44.96, 51.20 -> 6.16in tightest.
for sep in (6.16, 6.24, 6.32, 6.38):
    check(F.collision_verdict(sep, 0.0) is None,
          f"parked pair {sep}in apart, not closing", "no trigger")
check(F.collision_verdict(6.16, 0.2) is None,
      "parked pair with vision jitter (0.2in/s) does not trigger",
      "jitter is below COLLISION_MIN_CLOSING_IN_S")

print("\n3. a closing pair is stopped before contact")
# Cruise measured from the logs: a 13.8in leg took ~3.9s -> ~3.2in/s, so two
# robots driving at each other close at up to ~6.4in/s.
# The separations here are DERIVED from the constants rather than written in.
# They used to be literals pinned to COLLISION_CONTACT_IN = 5.04, and when
# that was raised to the measured 5.625 on 2026-09-30 three of them failed --
# not because the guard broke, but because the band they pinned had moved by
# exactly the amount intended. Pinning the shape of the rule survives a
# re-measurement; pinning its output does not.
check(abs(F.COLLISION_CONTACT_IN - 5.625) < 1e-9,
      "contact distance is the MEASURED 5.625in corner-to-corner",
      f"got {F.COLLISION_CONTACT_IN} -- if this changed on purpose, update it "
      f"here too; if not, something moved it silently")
for closing in (6.4, 3.2, 1.0):
    trigger = F.COLLISION_CONTACT_IN + closing * F.COLLISION_REACTION_SEC
    inside, outside = trigger - 0.1, trigger + 0.1
    # Inside the band is "collision" when the pair is already within contact,
    # otherwise "avoid" -- at 1.0in/s the whole band sits inside contact.
    want_in = "collision" if inside <= F.COLLISION_CONTACT_IN else "avoid"
    check(F.collision_verdict(inside, closing) == want_in,
          f"closing {closing}in/s at {inside:.2f}in -> {want_in}",
          f"trigger {trigger:.2f}in")
    check(F.collision_verdict(outside, closing) is None,
          f"closing {closing}in/s at {outside:.2f}in -> no action",
          f"trigger {trigger:.2f}in")

print("\n4. the threshold really does scale with speed")
slow = F.COLLISION_CONTACT_IN + 1.0 * F.COLLISION_REACTION_SEC
fast = F.COLLISION_CONTACT_IN + 8.0 * F.COLLISION_REACTION_SEC
check(fast > slow, "faster closing triggers earlier",
      f"{slow:.2f}in at 1in/s vs {fast:.2f}in at 8in/s")
check(slow < 6.16, "even the slow threshold stays under the depot slot pitch",
      f"{slow:.2f}in < 6.16in, so parked neighbours are never caught")

print("\n5. the 2026-09-29 depot collision, replayed")
# Alvik3 standing at DE3 (32.09, 3.15); Alvik5 driving DE2->DE3 at ~3.2in/s.
a3 = (32.09, 3.15)
caught = None
for step in range(40):
    a5 = (25.93 + 0.16 * step, 3.07)          # ~3.2in/s at the 20Hz tick
    sep = math.hypot(a5[0] - a3[0], a5[1] - a3[1])
    verdict = F.collision_verdict(sep, 3.2)
    if verdict is not None:
        caught = (round(sep, 2), verdict)
        break
check(caught is not None and caught[1] == "avoid",
      "Alvik5 is stopped BEFORE reaching Alvik3",
      f"caught at {caught[0]}in with verdict '{caught[1]}'" if caught else "never caught")
check(caught is not None and caught[0] > F.COLLISION_CONTACT_IN,
      "and it is caught while still short of contact",
      f"{caught[0]}in > {F.COLLISION_CONTACT_IN}in" if caught else "")

print("\n6. closing_pairs_in() measures separation and closing from poses")
vm = object.__new__(F.VisionMonitor)
import threading
vm._lock = threading.Lock()
vm._poses = {}
vm._poses_in = {}
vm._pose_in_t = {}
vm._prev_sep = {}
import time as _t
now = _t.time()
vm._poses_in = {"AlvikA": (0.0, 0.0, 0.0), "AlvikB": (10.0, 0.0, 0.0)}
vm._pose_in_t = {"AlvikA": now, "AlvikB": now}
first = vm.closing_pairs_in()
check(len(first) == 1 and abs(first[0][2] - 10.0) < 1e-6,
      "separation measured in inches", f"{first[0][2]:.2f}in")
# A sub-threshold interval must yield NO rate: dividing a real change in
# separation by an almost-zero dt is what produced "closing at 61.1in/s".
vm._poses_in["AlvikB"] = (9.5, 0.0, 0.0)
vm._pose_in_t["AlvikB"] = now + 0.02
quick = vm.closing_pairs_in()
check(quick[0][3] == 0.0,
      "a 20ms interval produces NO closing rate (the 61in/s bug)",
      f"{quick[0][3]:.2f}in/s")
# ...and the baseline must NOT have advanced, so the next sample still
# has something real to measure against.
vm._poses_in["AlvikB"] = (9.0, 0.0, 0.0)
vm._pose_in_t["AlvikB"] = now + 0.2
second = vm.closing_pairs_in()
check(abs(second[0][3] - 5.0) < 0.3,
      "over a real 0.2s interval the rate IS measured", 
      f"{second[0][3]:.2f}in/s from 10.0in -> 9.0in")
# a stale pose must not be used for a safety decision
vm._pose_in_t["AlvikB"] = now - 10.0
check(vm.closing_pairs_in() == [], "stale poses are excluded entirely",
      "a safety call on an old fix is worse than none")

print("\n7. in_bay_spur(): measured bay occupancy, the 2026-09-29 deadlock")
# Cell geometry, from dispatch_advisor.label_to_cell():
#   workstation  gx=c+0.5  gy=row_up+0.5
#   bay ENTRY    gx=c+0.5  gy=row_up+0.85   <- ON the corridor, must NOT count
#   lattice node gx=col    gy=row
from dispatch_advisor import label_to_cell as _cell
ws = _cell(69, 8, 8)      # workstation bay 5 -- where Alvik1 was parked
en = _cell(118, 8, 8)     # its entry
check(F.in_bay_spur(*ws), "a robot AT workstation 69 is in the spur", str(ws))
check(not F.in_bay_spur(*en),
      "a robot at its ENTRY 118 is NOT (an entry blocks the lane)", str(en))
for _n in (6, 14, 13, 5):
    _c = _cell(_n, 8, 8)
    check(not F.in_bay_spur(*_c), f"lattice node {_n} is not a spur", str(_c))
check(not F.in_bay_spur(4.5, 1.0),
      "driving along a row (mid-column) is not a spur")
check(not F.in_bay_spur(5.0, 0.5),
      "driving along a column (mid-row) is not a spur",
      "this is edge 6->14 itself, which Alvik1 wrongly blocked")
_bad_ws = _bad_en = 0
for _b in range(1, 50):
    _w = _cell(64 + _b, 8, 8); _e = _cell(113 + _b, 8, 8)
    if not F.in_bay_spur(*_w): _bad_ws += 1
    if F.in_bay_spur(*_e): _bad_en += 1
    for _dx, _dy in ((0.15, 0), (-0.15, 0), (0, 0.15), (0, -0.15)):
        if not F.in_bay_spur(_w[0] + _dx, _w[1] + _dy): _bad_ws += 1
check(_bad_ws == 0,
      "all 49 workstations detected, incl. +/-1.5in placement error",
      f"{_bad_ws} misses")
check(_bad_en == 0, "no bay ENTRY mistaken for a spur",
      f"{_bad_en} false positives")

print("\n9. a PARKED neighbour must not stop traffic (2026-09-29 stalls)")
# The four emergency stops from that run, verbatim from the supervisor log.
# Each one dropped a robot out of wheel-follow mid-leg and failed its leg.
for _sep, _cl in ((5.23, 0.6), (5.36, 0.9), (5.38, 1.2), (5.48, 2.1)):
    check(F.collision_verdict(_sep, _cl, parked_pair=True) is None,
          f"parked neighbour at {_sep}in closing {_cl}in/s -> no stop")
    # The parked exemption must not change the moving-pair path at all, so
    # assert against the two-tier rule itself rather than a blanket "avoid" --
    # at 0.9in/s the trigger is 5.355in, so 5.36in is correctly NOT a stop.
    # Mirror the real two-tier rule, INCLUDING the contact tier. This used to
    # read "avoid or None" only, which was accidentally right while contact
    # was 5.04 -- every separation here sat above it. At the measured 5.625
    # these pairs are inside contact, so the honest expectation is
    # "collision", and the old formula reported a false failure.
    _want = ("collision" if _sep <= F.COLLISION_CONTACT_IN
             else "avoid"
             if (_cl >= F.COLLISION_MIN_CLOSING_IN_S
                 and _sep <= F.COLLISION_CONTACT_IN
                             + _cl * F.COLLISION_REACTION_SEC)
             else None)
    check(F.collision_verdict(_sep, _cl) == _want,
          f"  two MOVING robots at {_sep}in / {_cl}in/s -> {_want}",
          f"trigger is {F.COLLISION_CONTACT_IN + _cl * F.COLLISION_REACTION_SEC:.2f}in")
# clearances the table is deliberately built with
check(F.collision_verdict(5.71, 0.0, parked_pair=True) is None,
      "bottom lane passing a depot slot (5.71in) is fine")
check(F.collision_verdict(5.22, 3.2, parked_pair=True) is None,
      "passing a bay entry beside a parked robot (5.22in) is fine")
# but genuine overlap is still a collision
check(F.collision_verdict(3.9, 0.0, parked_pair=True) == "collision",
      "actually overlapping a parked robot IS still a collision", "3.9in")
check(F.collision_verdict(4.5, 0.0, parked_pair=True) is None,
      "4.5in from a parked robot is clearance, not contact")

print("\n8. impossible closing rates are rejected")
check(F.COLLISION_MAX_CLOSING_IN_S < 27.0,
      "the 27.0in/s reading that stopped Alvik1+Alvik4 is rejected",
      f"cap {F.COLLISION_MAX_CLOSING_IN_S}in/s")
check(F.COLLISION_MAX_CLOSING_IN_S < 61.1,
      "so is the 61.1in/s that stopped Alvik5+Alvik6 at 24.3in apart")
check(F.COLLISION_MAX_CLOSING_IN_S > 6.4,
      "but a real head-on pair at cruise (~6.4in/s) still registers")
check(F.COLLISION_MIN_DT_SEC >= 0.1,
      "and the rate spans a real interval, not one frame",
      f"{F.COLLISION_MIN_DT_SEC}s")

print("\n" + ("ALL PASS" if not FAILS else f"{len(FAILS)} FAILURES: {FAILS}"))
sys.exit(1 if FAILS else 0)
