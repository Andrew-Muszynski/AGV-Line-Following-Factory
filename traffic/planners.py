"""The four traffic-planning methods the DOE compares.

    astar  Uncoordinated A*: every robot's fastest route on its own, as if
           the others did not exist. No waits are planned; every conflict is
           left to the supervisor's runtime layer. The no-coordination
           baseline: what planning traffic in advance buys at all.
    wait   Iterative delay insertion (the incumbent, solver.js
           edgeExclusiveSchedule): fixed L-shaped routes, find the earliest
           conflict, delay the later robot just past it, repeat.
    pp     Prioritized planning: robots planned one at a time, each avoiding
           every robot planned before it (Erdmann & Lozano-Perez 1987;
           Silver 2005). Longest uncoordinated route first. Incomplete, so
           it restarts with shuffled orders when a robot cannot be placed.
    cbs    Conflict-Based Search (Sharon et al. 2015): plan robots alone,
           find the earliest conflict, branch on which robot must avoid the
           other's interval, best-first on (makespan, sum of completion
           times). Interval constraints in continuous time, in the spirit
           of CCBS (Andreychuk et al. 2019); like it, not complete with
           interval constraints, so a budget ends the search.

All four use the same testbed model, the same fixed launch prefix, the same
job constraints and the same conflict model (conflicts.py) -- the only thing
that differs is how conflicts are resolved, which is the treatment.
"""
from __future__ import annotations

import heapq
import random
import time
from dataclasses import dataclass, field

from .conflicts import (POST_SEC, Constraints, Hold, detect_conflicts,
                        holds)
from .instance import Instance, readiness_violations, drop_times
from .plan import TimedPath, retime, time_path
from .sipp import Budget, plan_agent

METHODS = ("astar", "wait", "pp", "cbs")


@dataclass
class Result:
    method: str
    paths: dict
    success: bool
    planning_sec: float
    stats: dict = field(default_factory=dict)
    note: str = ""


# ---------------------------------------------------------------------------
# shared pieces
# ---------------------------------------------------------------------------
def prefix_holds(inst: Instance, task) -> list[Hold]:
    """The fixed launch prefix as holds, for robots not yet planned. Its
    last step (arrival at node 1) is held only briefly: how long the robot
    then stays there is part of its PLANNED route, not the prefix."""
    out = []
    p = task.prefix
    last = len(p.steps) - 1
    for h in holds(inst.tb, p):
        if h.step == last and h.kind == "node":
            h = Hold(h.agent, h.kind, h.resource, h.start,
                     p.steps[last].arrive + POST_SEC, h.step, h.via)
        out.append(h)
    return out


def _base_constraints(inst: Instance, name: str) -> Constraints:
    c = Constraints()
    for t in inst.tasks:
        if t.name != name:
            for h in prefix_holds(inst, t):
                c.add_against(inst.tb, h)
    return c


def _ready_times(inst: Instance, paths: dict) -> dict:
    return {b: t + inst.process_sec for b, t in drop_times(inst, paths).items()}


def _with_readiness(inst: Instance, plan_fn, rounds: int = 6):
    """Plan, then raise pick-up release times to the planned drop + process
    time and plan again, until no pick leaves its bay early. Release times
    only ever increase, so this settles; the same loop wraps every method."""
    ready: dict = {}
    paths, stats = None, {}
    for r in range(rounds):
        paths, stats = plan_fn(ready)
        if paths is None or not inst.rpd:
            return paths, stats
        if not readiness_violations(inst, paths):
            stats["readiness_rounds"] = r + 1
            return paths, stats
        for b, t in _ready_times(inst, paths).items():
            ready[b] = max(ready.get(b, 0.0), t)
    stats["readiness_rounds"] = rounds
    return paths, stats


# ---------------------------------------------------------------------------
# A* -- uncoordinated
# ---------------------------------------------------------------------------
def solve_astar(inst: Instance, time_limit: float = 30.0, **_) -> Result:
    t0 = time.perf_counter()
    budget = Budget(time_limit)

    def plan(ready):
        out = {}
        for task in inst.tasks:
            p = plan_agent(inst, task, Constraints(), ready, budget)
            if p is None:
                return None, {}
            out[task.name] = p
        return out, {}

    paths, stats = _with_readiness(inst, plan)
    return Result("astar", paths or {}, paths is not None,
                  time.perf_counter() - t0, stats)


# ---------------------------------------------------------------------------
# PP -- prioritized planning
# ---------------------------------------------------------------------------
def solve_pp(inst: Instance, time_limit: float = 30.0, restarts: int = 20,
             seed: int = 0, **_) -> Result:
    t0 = time.perf_counter()
    budget = Budget(time_limit)
    solo = {t.name: plan_agent(inst, t, Constraints(), None, budget)
            for t in inst.tasks}
    base_order = sorted(inst.tasks, key=lambda t: (
        -(solo[t.name].end if solo[t.name] else 0), t.slot))
    tried = {"attempts": 0}

    def plan(ready):
        rng = random.Random(seed)
        for attempt in range(restarts + 1):
            tried["attempts"] += 1
            order = list(base_order)
            if attempt:
                rng.shuffle(order)
            planned: dict = {}
            for task in order:
                cons = _base_constraints(inst, task.name)
                for p in planned.values():
                    cons.add_path(inst.tb, p)
                p = plan_agent(inst, task, cons, ready, budget)
                if p is None:
                    break
                planned[task.name] = p
            else:
                return planned, {"order": [t.name for t in order],
                                 "attempts": tried["attempts"]}
            if budget.deadline and time.monotonic() > budget.deadline:
                break
        return None, {"attempts": tried["attempts"]}

    paths, stats = _with_readiness(inst, plan)
    return Result("pp", paths or {}, paths is not None,
                  time.perf_counter() - t0, stats,
                  "" if paths else "no priority order placed every robot")


# ---------------------------------------------------------------------------
# CBS -- conflict-based search
# ---------------------------------------------------------------------------
def solve_cbs(inst: Instance, time_limit: float = 30.0,
              max_nodes: int = 5000, **_) -> Result:
    t0 = time.perf_counter()
    budget = Budget(time_limit)
    tb = inst.tb
    by_name = inst.by_name
    base = {t.name: _base_constraints(inst, t.name) for t in inst.tasks}
    counters = {"nodes": 0, "expanded": 0}

    def key(paths, n_conflicts):
        """Makespan first (the DOE's primary response, and what CBS is
        optimal for here), then fewest remaining conflicts, then the sum of
        completion times. Ranking soc above the conflict count made CBS
        breadth-first along shared routes: two robots homing along one lane
        conflict once per hop, and every hop's 'other' branch costs a little
        less soc, so it was explored first -- 2^hops nodes, budget gone
        (2026-10-17, Alvik2/Alvik4 along row 0)."""
        ends = [p.end for p in paths.values()]
        return (round(max(ends), 6), n_conflicts, sum(ends))

    def plan(ready):
        paths = {}
        for t in inst.tasks:
            p = plan_agent(inst, t, base[t.name].copy(), ready, budget)
            if p is None:
                return None, dict(counters)
            paths[t.name] = p
        # Best-first on (makespan, sum of completion times), ties broken by
        # how many conflicts a node still has. Without that tie-break CBS
        # wanders a plateau: on these instances most children cost exactly
        # the uncoordinated lower bound, and 2026-10-17's BRKGA route spent
        # its whole 5000-node budget among them.
        seq = 0
        open_ = [(key(paths, len(detect_conflicts(tb, paths))), seq, {},
                  paths)]
        while open_:
            if budget.deadline and time.monotonic() > budget.deadline:
                break
            if counters["expanded"] >= max_nodes:
                break
            _k, _s, adds, cur = heapq.heappop(open_)
            counters["expanded"] += 1
            found = detect_conflicts(tb, cur, first_only=True)
            if not found:
                return cur, dict(counters)
            c = found[0]
            for mine, theirs in ((c.a, c.b), (c.b, c.a)):
                who = mine.agent
                if mine.step < cur[who].prefix:
                    continue               # its fixed launch cannot change
                new_adds = dict(adds)
                new_adds[who] = adds.get(who, ()) + (theirs,)
                cons = base[who].copy()
                for h in new_adds[who]:
                    cons.add_against(tb, h)
                p = plan_agent(inst, by_name[who], cons, ready, budget)
                if p is None:
                    continue
                child = dict(cur)
                child[who] = p
                counters["nodes"] += 1
                seq += 1
                heapq.heappush(open_, (key(child,
                                           len(detect_conflicts(tb, child))),
                                       seq, new_adds, child))
        return None, dict(counters)

    paths, stats = _with_readiness(inst, plan)
    return Result("cbs", paths or {}, paths is not None,
                  time.perf_counter() - t0, stats,
                  "" if paths else "budget exhausted before a conflict-free "
                                   "node was found")


# ---------------------------------------------------------------------------
# WAIT -- iterative delay insertion on fixed L-shaped routes
# ---------------------------------------------------------------------------
def wait_route_nodes(inst: Instance, task) -> list:
    """solver.js buildAgentRoute(pathMode="vh") after the launch prefix:
    row-first L-shaped legs; each bay entered from the flank that needs no
    about-face, else the nearer; left toward the next bay (or home)."""
    tb = inst.tb

    def leg(cur, end):
        r, c = cur
        out = []
        while r != end[0]:
            r += 1 if end[0] > r else -1
            out.append(tb.lattice(r, c))
        while c != end[1]:
            c += 1 if end[1] > c else -1
            out.append(tb.lattice(r, c))
        return out

    full = [1]
    cur = (0, 0)
    bays = [v["bay"] for v in task.visits]
    for b_i, bay in enumerate(bays):
        br, bc = tb.bay_cell(bay)
        entry, ws = tb.entry_of(bay), tb.ws_of(bay)
        sides = [(br + 1, bc), (br + 1, bc + 1)]
        scored = []
        for order, side in enumerate(sides):
            seg = leg(cur, side)
            tail = full + seg
            arrival = tb.heading(tail[-2], tail[-1]) if len(tail) >= 2 else None
            entering = tb.heading(tb.lattice(*side), entry)
            flip = 1 if tb.reverses(arrival, entering) else 0
            dist = abs(side[0] - cur[0]) + abs(side[1] - cur[1])
            scored.append((flip, dist, order, seg, side))
        scored.sort(key=lambda s: s[:3])
        full += scored[0][3]
        if b_i + 1 < len(bays):
            nr, nc = tb.bay_cell(bays[b_i + 1])
            ref = (nr + 1, nc + 0.5)
        else:
            ref = (0, 0)
        west, east = sides

        def toward(node):
            return abs(node[0] - ref[0]) + abs(node[1] - ref[1])
        exit_ = east if toward(east) < toward(west) else west
        full += [entry, ws, entry, tb.lattice(*exit_)]
        cur = exit_
    full += leg(cur, (0, 0))
    full += [0] + [f"DE{j}" for j in range(1, task.slot + 1)] + [f"D{task.slot}"]
    return full


def solve_wait(inst: Instance, time_limit: float = 30.0,
               max_iterations: int = 400, step_sec: float = 0.1, **_) -> Result:
    t0 = time.perf_counter()
    tb = inst.tb
    deadline = time.monotonic() + time_limit
    paths: dict = {}
    for task in inst.tasks:
        pre = task.prefix
        nodes = pre.nodes[:-1] + wait_route_nodes(inst, task)
        waits = pre.waits[:-1] + [0.0] * (len(nodes) - len(pre.nodes) + 1)
        p = time_path(tb, task.name, nodes, waits, t0=pre.steps[0].arrive,
                      heading0=pre.steps[0].heading_in,
                      prefix=len(pre.steps) - 1, allow_reversal=True)
        # service dwell: the processing wait spent in the bay
        if inst.rpd:
            p = _apply_service_dwell(inst, task, p)
        paths[task.name] = p
    rank = {t.name: t.slot for t in inst.tasks}
    iterations = 0
    capped = False
    why = ""
    start_span = max(p.end for p in paths.values())
    # WAIT IS INCOMPLETE: delays alone cannot separate every pair of fixed
    # routes -- two robots that must pass head-on along one lane, or swap
    # bays, cannot be untangled by waiting, because a robot made to wait is
    # itself an obstacle. The first version kept adding waits until the
    # makespan doubled and then shipped its "best" attempt: on the 6-robot
    # DOE plan that left 15 conflicts AND waits of 204 s inside bay 9, 166 s
    # at bay 10's entry and 149 s on lattice node 18, which on hardware
    # parked Alvik2 on node 18 for 2.5 minutes in everyone's way
    # (2026-10-21). That is not delay insertion; it is a broken plan.
    #
    # OPTION A (operator, 2026-10-21): resolve every conflict waiting CAN
    # resolve, and leave the pairs it cannot to the runtime layer, exactly as
    # uncoordinated A* leaves all of them. When the waits for one pair of
    # robots fail (no delay helps, the makespan runs away, or the pair eats
    # more than PAIR_STEP_CAP delay steps), every wait added since work on
    # that pair began is rolled back and the pair's conflicts are marked as
    # left for the supervisor.
    def pair(c):
        return frozenset((c.a.agent, c.b.agent))

    def snapshot():
        return {n: list(p.waits) for n, p in paths.items()}

    def restore(snap):
        for n, w in snap.items():
            paths[n] = retime(tb, paths[n], list(w))

    left: set = set()          # robot pairs handed to the runtime
    started: dict = {}         # pair -> waits before work on it began
    steps: dict = {}           # pair -> delay steps spent on it
    reasons: list = []
    while True:
        if iterations >= max_iterations or time.monotonic() > deadline:
            capped, why = True, "iteration cap"
            break
        viol = readiness_violations(inst, paths)
        if viol:
            iterations += 1
            name, idx, short = viol[0]
            # Wait at the robot's PREVIOUS stop, not inside the pick-up bay:
            # the part is delivered into that bay by another robot, which
            # cannot get in while this one sits there (2026-10-19).
            _add_wait(tb, paths, name, _wait_point(inst, paths[name], name, idx),
                      short + 1e-3)
            continue
        found = [c for c in detect_conflicts(tb, paths) if pair(c) not in left]
        if not found:
            break
        iterations += 1
        c = found[0]
        k = pair(c)
        started.setdefault(k, snapshot())
        steps[k] = steps.get(k, 0) + 1
        ok = _delay_for(tb, paths, c, rank, step_sec)
        span = max(p.end for p in paths.values())
        failure = None
        if not ok:
            failure = f"no delay resolves {c.describe()}"
        elif span > RUNAWAY_FACTOR * start_span:
            failure = f"delays running away on {c.describe()}"
        elif steps[k] > PAIR_STEP_CAP:
            failure = f"{PAIR_STEP_CAP} delay steps on {c.describe()}"
        if failure:
            restore(started.pop(k))
            left.add(k)
            reasons.append(failure)
    remaining = [c for c in detect_conflicts(tb, paths)]
    n_left = len(remaining)
    clean = n_left == 0 and not readiness_violations(inst, paths)
    pairs_txt = ", ".join(" & ".join(sorted(p)) for p in sorted(left, key=sorted))
    note = ""
    if not clean:
        note = (f"waiting cannot separate {pairs_txt}; their {n_left} "
                "conflicts are left for the runtime" if left
                else f"{why}; {n_left} conflicts left for the runtime")
    return Result("wait", paths, clean, time.perf_counter() - t0,
                  {"iterations": iterations, "capped": capped,
                   "conflicts_left": n_left,
                   "pairs_left_to_runtime": sorted(sorted(p) for p in left),
                   "why": reasons}, note)


#: WAIT gives up once delays have more than doubled the makespan: past that
#: the robots are waiting on each other, not on traffic.
RUNAWAY_FACTOR = 2.0
#: Delay steps WAIT may spend on one pair of robots before deciding waiting
#: cannot separate them.
PAIR_STEP_CAP = 60


def _apply_service_dwell(inst, task, p: TimedPath) -> TimedPath:
    waits = p.waits
    gi = 0
    for i in range(p.prefix, len(p.steps)):
        if gi < len(task.goals) - 1 and p.steps[i].node == task.goals[gi].node:
            waits[i] = max(waits[i], task.goals[gi].min_dwell)
            gi += 1
    return retime(inst.tb, p, waits)


def _wait_point(inst, p, name, idx) -> int:
    """Step index where a robot early for a pick-up should wait: its last
    goal before step idx (the previous workstation it served), or idx itself
    if the pick-up is its first goal on the grid."""
    task = next(t for t in inst.tasks if t.name == name)
    gi, last = 0, None
    for i in range(p.prefix, min(idx, len(p.steps))):
        if gi < len(task.goals) and p.steps[i].node == task.goals[gi].node:
            last = i
            gi += 1
    return last if last is not None else idx


def _add_wait(tb, paths, name, idx, seconds):
    p = paths[name]
    w = p.waits
    w[idx] += seconds
    paths[name] = retime(tb, p, w)


def _delay_for(tb, paths, c, rank, step) -> bool:
    """WAIT's rule: delay the robot that gets there later (ties: the later
    launch), by just enough to clear the blocker."""
    a, b = c.a, c.b
    if c.kind == "rotation" or (c.kind == "node"
                                and tb.squeeze_partners(a.resource)):
        return _delay_around_entry(tb, paths, c, rank, step)
    late, early = (b, a) if (b.start, rank[b.agent]) >= (a.start, rank[a.agent]) \
        else (a, b)
    for delayed, blocker in ((late, early), (early, late)):
        need = blocker.end - delayed.start
        if need == float("inf"):
            continue
        idx = delayed.step if delayed.via == "drive" else delayed.step - 1
        if _try(tb, paths, delayed.agent, idx, need, step):
            return True
    return False


def _delay_around_entry(tb, paths, c, rank, step) -> bool:
    """Around a bay entry, "later" has to mean the same thing for every
    conflict between the same two robots, or the fixes chase each other.

    2026-10-17, first run of this port on the 09-30 RPD scenario: Alvik3 had
    to turn at entry 130 and then drive to 27, which Alvik4 also needed. The
    rotation rule delayed Alvik4 until the turn ended; the node rule at 27
    then delayed Alvik3, which moved its turn; which re-delayed Alvik4 --
    400 alternations of ~1.5s each until the cap. solver.js never meets this
    because it has no rotation rule.

    So: the robot that started its pass through the contested spots FIRST
    keeps them; the other waits until that pass is over -- every presence and
    turn of it -- then enters. "Pass" is the contiguous run of the robot's
    steps on those spots, NOT its whole visit: a robot that turns in at an
    entry and then spends 30s in the bay has finished with the lattice node
    beside the entry when it drives into the bay (workstation-lattice is
    7.07in, room to spare). The holds waited on do not move, so the loop
    converges."""
    spots = set()
    for h in (c.a, c.b):
        spots |= {h.resource} | tb.squeeze_partners(h.resource)
    spots = {n for n in spots if tb.kind(n) != "workstation"} | \
        {h.resource for h in (c.a, c.b)}

    def run(name, step_i):
        """Steps of `name` forming one contiguous pass over `spots` that
        includes step_i; and that pass's holds."""
        p = paths[name]
        lo = hi = step_i
        while lo - 1 >= 0 and p.steps[lo - 1].node in spots:
            lo -= 1
        while hi + 1 < len(p.steps) and p.steps[hi + 1].node in spots:
            hi += 1
        hs = [h for h in holds(tb, p)
              if lo <= h.step <= hi and h.kind in ("node", "turn")]
        return lo, hs

    lo_a, ha = run(c.a.agent, c.a.step)
    lo_b, hb = run(c.b.agent, c.b.step)
    if not ha or not hb:
        return False
    start_a = min(h.start for h in ha)
    start_b = min(h.start for h in hb)
    order = sorted([(start_a, rank[c.a.agent], c.a.agent, ha, c.b.agent, lo_b,
                     start_b),
                    (start_b, rank[c.b.agent], c.b.agent, hb, c.a.agent, lo_a,
                     start_a)])
    for _s, _r, _keeper, keep_h, waiter, w_lo, w_start in order:
        clear = max(h.end for h in keep_h)
        if clear == float("inf"):
            continue
        if _try(tb, paths, waiter, w_lo - 1, clear - w_start, step):
            return True
        # the waiter cannot be held back (launch prefix): swap roles
    return False


def _try(tb, paths, name, idx, need, step) -> bool:
    p = paths[name]
    if idx < p.prefix or idx >= len(p.steps) - 1 or need == float("inf"):
        return False
    secs = max(step, round((need + step) / step) * step)
    _add_wait(tb, paths, name, idx, secs)
    return True


SOLVERS = {"astar": solve_astar, "wait": solve_wait, "pp": solve_pp,
           "cbs": solve_cbs}


def solve(inst: Instance, method: str, **opts) -> Result:
    if method not in SOLVERS:
        raise ValueError(f"unknown method {method!r}; choose from {METHODS}")
    return SOLVERS[method](inst, **opts)
