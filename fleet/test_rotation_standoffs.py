#!/usr/bin/env python3
"""The 2026-10-16 standoffs: what held robots, and what now releases them.

    python3 test_rotation_standoffs.py

Run 208a81ec (6 robots, rotate_rel) lost ~60s to two standoffs and was then
aborted by hand. Each section below pins one cause, measured from that run's
log and pose telemetry:

  1. Alvik3 FINISHED its turn at entry 122 (yaw +88.3, pointing away from
     node 18), then the advisor held the no-op turn item behind it for 60s:
     "rotation at entry 122 would sweep into Alvik4 on node 18".
  2. Alvik4 ARRIVED at 18 (95.1deg against a 90deg target), was held 20s at
     its square-up, failed, was retried from 17 -- which it had left -- read
     as off-route 100 times, and was excluded.
  3. Alvik2 backed out of entry 137 to node 36, 5.00in away, so that Alvik3
     could turn at 137 into bay 88: a yield that recreated the standoff.
  4. The same Alvik2/Alvik3 cycle was "detected" again 1.2s into Alvik2's
     yield, so Alvik3 was sent aside too; each saw the other gone, decided
     it had passed, and both went home into the same deadlock -- twice.
  5. Four of the run's five DEADLOCKs were one-tick artefacts at the depot
     lane exit ("deadlock cleared after 0.0s"), each counted in metrics.
"""
import os
import sys
import threading
import time
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

import camera_grid_navigate as N  # noqa: E402
import dispatch_advisor as da  # noqa: E402
import fleetSupervisor as F  # noqa: E402

PASS, FAIL = [], []


def check(label, cond, note=""):
    (PASS if cond else FAIL).append(label)
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'   ' + note if note else ''}")


R = C = 8
SRC = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fleetSupervisor.py"), encoding="utf-8").read()

# --------------------------------------------------------------------------
print("1. the advisor no longer holds a turn item that has already happened")


class _St:
    def __init__(self, name, node, plan=()):
        self.name, self.current_node = name, node
        self.plan, self.next_index = list(plan), 0
        self.last_wait_sig = self.last_selected_candidate_ms = None


def advisor_at_entry(gated_at_execution):
    a = object.__new__(da.DispatchAdvisor)
    a.rows, a.cols = R, C
    turn = da.PlanItem(command="RIGHT_UNTIL_COLOR", kind="turn")
    a.robots = {"Alvik3": _St("Alvik3", 122, [turn]),
                "Alvik4": _St("Alvik4", 18)}
    a.reservations, a.wait_for = {}, {}
    a.expire_standing_linger = lambda now: None
    a._clear_request = lambda st: None
    a._wait_decision = lambda st, why, conflicts=None: ("WAIT", why)
    a._send_planned = lambda st, why, now_sec=None: ("SEND", why)
    if gated_at_execution:
        a.rotation_gated_at_execution = True
    return a


col = advisor_at_entry(False)
got = col.decide("Alvik3", 60.2)
check("colour mode: the turn item IS the rotation, so it is still held",
      got[0] == "WAIT" and "would sweep into Alvik4" in got[1], str(got))
check("...and registers the standoff for the deadlock resolver",
      col.wait_for.get("Alvik3") == "Alvik4")
vis = advisor_at_entry(True)
got = vis.decide("Alvik3", 60.2)
check("vision mode: the no-op turn item is sent, Alvik3 drives on",
      got[0] == "SEND", str(got))
check("...and no phantom standoff edge is registered",
      "Alvik3" not in vis.wait_for)
check("the class default is colour-mode behaviour",
      da.DispatchAdvisor.rotation_gated_at_execution is False)
check("the supervisor sets it from the mission's drive mode",
      'advisor.rotation_gated_at_execution = (node.drive_mode == "vision")'
      in SRC)

# --------------------------------------------------------------------------
print("\n2. a gate only guards a turn that will actually happen")


class _Sup:
    def __init__(self, result=(True, "clear")):
        self.calls, self.result, self.warnings = [], result, []

    def await_rotation_clearance(self, name, label, timeout_sec=None,
                                 cancelled=None, at_node=None, idle=None,
                                 nudge=None, drive_to=None):
        self.calls.append((label, timeout_sec, cancelled))
        self.at_nodes = getattr(self, "at_nodes", []) + [at_node]
        r = self.result(timeout_sec) if callable(self.result) else self.result
        return r

    def get_logger(self):
        return types.SimpleNamespace(
            warning=lambda m: self.warnings.append(m),
            info=lambda m: None, error=lambda m: None)


class _Nav:
    def __init__(self, yaw):
        self.yaw = yaw
        self.args = types.SimpleNamespace(
            rows=R, cols=C, turn_tol_deg=4.0, final_yaw_tol_deg=2.0)
        self.localization_mode = "camera_assist"
        self.turns, self.drives, self.dwells = [], [], []
        self.last_failure = None

    def control_pose(self):
        return None if self.yaw is None else types.SimpleNamespace(yaw=self.yaw)

    def fresh_pose(self):
        return None          # no camera: the on-line check is skipped

    def stop_only(self, *a):
        pass

    def _spin_once(self, t):
        pass

    def ensure_odom_anchor(self, *a):
        return True

    def drive_leg(self, x, y, label):
        self.drives.append(label)
        return True

    def dwell(self, ms, label):
        self.dwells.append(label)
        return True

    def turn_to_heading_rotate_rel(self, heading, label, tol_deg=None):
        self.turns.append((label, round(heading, 1), tol_deg))
        return True


def worker(nav, sup):
    w = object.__new__(F.VisionLegWorker)
    w.name, w.nav, w.supervisor = "Alvik4", nav, sup
    w._cancel = threading.Event()
    return w


s = _Sup()
w = worker(_Nav(yaw=91.5), s)
ok, why = w._clear_to_rotate(90.0, "turn at 18", timeout_sec=None)
check("within turn_tol_deg: no gate, the navigator will skip the turn",
      ok and why == "no rotation needed" and s.calls == [], why)
w = worker(_Nav(yaw=95.1), s)
w._clear_to_rotate(90.0, "square up at 18", timeout_sec=20.0)
check("Alvik4's measured 5.1deg error IS a turn, so it is gated",
      len(s.calls) == 1 and s.calls[0][1] == 20.0, str(s.calls))
check("the gate is handed the worker's cancel flag",
      s.calls and s.calls[0][2] == w._cancel.is_set)
s2 = _Sup()
worker(_Nav(yaw=None), s2)._clear_to_rotate(90.0, "x", timeout_sec=None)
check("no pose to judge by -> assume it turns, and gate it", len(s2.calls) == 1)
s3 = _Sup()
worker(_Nav(yaw=-179.0), s3)._clear_to_rotate(180.0, "x", timeout_sec=None)
check("the +-180 wrap is handled (-179 vs 180 is 1deg)", s3.calls == [])

# --------------------------------------------------------------------------
print("\n3. an ARRIVED robot is never failed back to its start node")


def run_move(frm, to, next_move, yaw, sup):
    nav = _Nav(yaw=yaw)
    w = worker(nav, sup)
    item = da.PlanItem(command="FWD", kind="move", from_node=frm, to_node=to)
    nxt = (da.PlanItem(command="FWD", kind="move",
                       from_node=next_move[0], to_node=next_move[1])
           if next_move else None)
    res = w._execute(item, nxt, False, da.movement_segments, N.heading_between)
    return res, nav


x17, y17 = F.nav_node_to_world(17, R, C)
x18, y18 = F.nav_node_to_world(18, R, C)
h_1718 = N.heading_between(x17, y17, x18, y18)

blocked = _Sup(result=lambda t: (False, "rotation blocked for 20s by Alvik3")
               if t is not None else (True, "clear"))
(ok, note, _), nav = run_move(17, 18, None, h_1718 + 5.1, blocked)
check("Alvik4's case: square-up at 18 stays blocked -> leg still SUCCEEDS",
      ok, note)
check("...the square-up is skipped, not performed into Alvik3",
      nav.turns == [], str(nav.turns))
check("...and the skip is logged", any("skipped" in m for m in blocked.warnings),
      str(blocked.warnings))

aborted = _Sup(result=(False, "aborted while holding for rotation clearance"))
(ok, note, _), nav = run_move(17, 18, None, h_1718 + 5.1, aborted)
check("an operator abort during the hold still fails the leg", not ok, note)

ws = _Sup()
x137, y137 = F.nav_node_to_world(137, R, C)
x88, y88 = F.nav_node_to_world(88, R, C)
(ok, note, _), nav = run_move(137, 88, None, 0.0, ws)
check("workstation arrival: the required heading is held with NO deadline",
      ok and ws.calls and ws.calls[0][1] is None, str(ws.calls[:1]))
check("...and squared to the tight 2deg tolerance",
      nav.turns and nav.turns[0][2] == 2.0, str(nav.turns))

nxt = _Sup()
x26, y26 = F.nav_node_to_world(26, R, C)
h_1826 = N.heading_between(x18, y18, x26, y26)
(ok, note, _), nav = run_move(17, 18, (18, 26), h_1718, nxt)
check("turn toward the next leg: held with NO deadline",
      ok and nxt.calls and nxt.calls[-1][1] is None
      and "toward 18->26" in nxt.calls[-1][0], str(nxt.calls))

# --------------------------------------------------------------------------
print("\n4. the turn after a dwell is gated (it was not, until today)")
# Alvik2 after its dwell at entry 137: facing ~180, next leg 137->35 (270).
dw = _Sup()
nav = _Nav(yaw=177.9)
w = worker(nav, dw)
item = da.PlanItem(command="DWELL 100", kind="dwell")
nm = da.PlanItem(command="FWD", kind="move", from_node=137, to_node=35)
ok, note, _ = w._execute(item, nm, False, da.movement_segments,
                         N.heading_between)
check("Alvik2's 92deg turn at entry 137 after a dwell consults the gate",
      ok and dw.calls and "turn after dwell" in dw.calls[0][0], str(dw.calls))
check("...with no deadline, so a blocked turn does not re-run the dwell",
      dw.calls and dw.calls[0][1] is None)
quiet = _Sup()
w = worker(_Nav(yaw=180.0), quiet)
w._execute(item, da.PlanItem(command="FWD", kind="move", from_node=88,
                             to_node=137),
           False, da.movement_segments, N.heading_between)
check("leaving a workstation needs no turn (already squared to 180)",
      quiet.calls == [], str(quiet.calls))
check("every rotation site goes through _clear_to_rotate",
      SRC.count("self._clear_to_rotate(") >= 4,
      f"{SRC.count('self._clear_to_rotate(')} call sites")

# --------------------------------------------------------------------------
print("\n5. await_rotation_clearance: no deadline, but cancellable")


class _Vis:
    def __init__(self, poses):
        self.poses = poses

    def snapshot_in(self):
        return self.poses


def sup_with(poses):
    s = object.__new__(F.FleetSupervisor)
    s.vision = _Vis(poses)
    s.advisor = None
    s._logged = []
    s.get_logger = lambda: types.SimpleNamespace(
        info=lambda m: s._logged.append(m), error=lambda m: None)
    s._event = lambda *a, **k: None
    return s


tight = {"Alvik4": (22.9, 36.9, 95.1, 0.0), "Alvik3": (28.2, 36.8, 88.3, 0.0)}
s = sup_with(dict(tight))
done = {}


def hold():
    done["r"] = s.await_rotation_clearance("Alvik4", "square up at 18",
                                           timeout_sec=None)


t = threading.Thread(target=hold, daemon=True)
t.start()
time.sleep(0.35)
check("no deadline: still holding well past several polls", t.is_alive())
s.vision.poses = {"Alvik4": tight["Alvik4"], "Alvik3": (38.2, 36.8, 88.3, 0.0)}
t.join(timeout=2.0)
check("...and released once the neighbour moves away",
      not t.is_alive() and done.get("r", (False,))[0], str(done))

s = sup_with(dict(tight))
flag = threading.Event()
flag.set()
t0 = time.monotonic()
ok, why = s.await_rotation_clearance("Alvik4", "x", timeout_sec=None,
                                     cancelled=flag.is_set)
check("a cancelled worker stops waiting at once (no late turn)",
      not ok and "abort" in why and time.monotonic() - t0 < 0.5, why)

# --------------------------------------------------------------------------
print("\n6. yields: one per standoff, real passing, and not into the turn")
a = object.__new__(da.DispatchAdvisor)
a.rows, a.cols = R, C
st = _St("Alvik3", 27, [
    da.PlanItem(command="F", kind="move", from_node=27, to_node=35),
    da.PlanItem(command="T", kind="turn"),
    da.PlanItem(command="F", kind="move", from_node=35, to_node=137),
    da.PlanItem(command="F", kind="move", from_node=137, to_node=88)])
a.robots = {"Alvik3": st}
check("Alvik3 still needs 137 (Alvik2's home) -> Alvik2 must not go back",
      a.needs_node_soon("Alvik3", 137))
check("a node not on its way is not needed", not a.needs_node_soon("Alvik3", 36))
check("a cycle a running yield is already resolving is left alone",
      'key == frozenset({yname, y["blocked"]})' in SRC)
check("'has passed' requires planned progress, not just a move",
      'bst.next_index > y.get("blocked_index", -1)' in SRC)
check("...and that the robot let through is not itself yielding",
      'y["blocked"] not in yielding' in SRC)
check("...and no longer needs the yielder's home",
      'advisor.needs_node_soon(\n                                    y["blocked"], n) for n in way_home)' in SRC
      and 'way_home = [str(y["home"])]' in SRC)
check("a cycle must persist DEADLOCK_CONFIRM_SEC before it is declared",
      F.DEADLOCK_CONFIRM_SEC >= 0.5 and "pending_cycles" in SRC)

print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("FAILED: " + "; ".join(FAIL))
sys.exit(1 if FAIL else 0)
