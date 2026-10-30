"""ALNS job allocation, sequential vs joint with traffic planning.

The experiment this serves: does planning the job allocation WITH traffic in
the loop (joint) beat planning it without and routing afterwards
(sequential)? Both arms are the same ALNS -- same start, same destroy and
repair operators, same seed, same wall-clock budget. They differ in ONE
place, the objective that judges a candidate allocation:

  sequential  each robot's route planned alone (traffic.planners astar):
              turns, the launch cascade, processing and pick-up readiness
              are all in, other robots are not. Afterwards the winning
              allocation is routed with PP, as the DOE's plans were.
  joint       the allocation is routed with PP inside the evaluation, and
              that conflict-free makespan is what ALNS minimises.

So any difference between the arms' final PP plans is what putting traffic
in the loop is worth.

A solution is, per robot, an ordered list of events (bay, 'd'rop | 'p'ick).
Every bay has exactly one drop and one pick (VRP-RPD): a robot starts with
`capacity` parts, a drop uses one, a pick returns one, and a pick may not
start until the drop + processing time. A drop immediately followed by the
same robot's pick of that bay is one 'service' stay (instance.merge_service_
visits), which is what every DOE plan used. With swaps off (default), drop
and pick of a bay go to the same robot; with --split, they may be separated
in that robot's sequence (drop A, drop B, pick A, pick B).

Repair uses a cheap static model (proxy_makespan) to choose insertion
points; the arm's objective then accepts or rejects the whole candidate.
Using the same proxy in both arms keeps the operators identical.
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

from .instance import Goal, Instance, Task, cascade_prefixes, merge_service_visits
from .model import ENTRY_WS_SEC, TURN_180_SEC, Testbed
from .planners import solve_astar, solve_pp

DROP, PICK = "d", "p"
ARMS = ("seq", "joint")
TIE_WEIGHT = 0.001


@dataclass
class Workload:
    bays: list
    robots: int
    capacity: int = 2
    process_sec: float = 30.0
    rows: int = 8
    cols: int = 8
    swaps: bool = False          # drop and pick of a bay by different robots
    split: bool = False          # same robot, drop and pick not adjacent
    service: bool = True         # a robot may wait in the bay for its own part
    drop_sec: float = 0.0        # the page's DWELL at every workstation arrival
    name: str = "workload"

    @property
    def ready_after_arrival(self) -> float:
        """Arrival of the drop -> part READY. Operator rule 2026-10-29: the
        drop is the page's DWELL (default 2 s) after the robot squares up
        with ROTATE_180; then the robot may leave and the workstation runs
        its processing clock. Without service stays: turn + drop + process."""
        if self.service:
            return self.process_sec
        return TURN_180_SEC + self.drop_sec + self.process_sec

    @property
    def slots(self) -> list:
        return list(range(1, self.robots + 1))


@dataclass
class Evaluation:
    makespan: float
    ends: dict                    # robot name -> route end, s
    paths: dict | None = None

    @property
    def cost(self) -> float:
        """What the search minimises: makespan, ties broken by the sum of
        robot end times (many allocations share the bottleneck robot)."""
        return self.makespan + TIE_WEIGHT * sum(self.ends.values())


INFEASIBLE = Evaluation(math.inf, {})


class Problem:
    """Everything fixed for one workload: graph, launch prefixes, timings.
    Built once; candidate allocations then only build Tasks."""

    def __init__(self, wl: Workload):
        self.wl = wl
        self.tb = Testbed(wl.rows, wl.cols, slots=max(6, wl.robots))
        self.prefixes = cascade_prefixes(self.tb, wl.slots)
        self.names = [f"Alvik{k}" for k in wl.slots]
        self.start_t = {n: self.prefixes[k].end for n, k in zip(self.names, wl.slots)}
        self._tt: dict = {}

    # -- static travel times (proxy) ---------------------------------------
    def tt(self, a, b) -> float:
        if a == b:
            return 0.0
        d = self._tt.get(b)
        if d is None:
            d = self._tt[b] = self.tb.time_to(b)
        return d.get(a, math.inf)

    # -- instances ------------------------------------------------------------
    def instance(self, sol: dict, name: str = "alns") -> Instance:
        tb, wl = self.tb, self.wl
        tasks = []
        for k, rname in zip(wl.slots, self.names):
            visits = [{"bay": b, "mode": "drop" if e == DROP else "pick"}
                      for b, e in sol[rname]]
            merged = merge_service_visits(visits)
            # The page DWELLs on every workstation arrival, picks included.
            dwell = {"service": wl.process_sec, "drop": wl.drop_sec,
                     "pick": wl.drop_sec}
            goals = [Goal(tb.ws_of(v["bay"]), v["bay"], v["mode"], dwell[v["mode"]])
                     for v in merged]
            goals.append(Goal(f"D{k}", None, "home"))
            tasks.append(Task(rname, k, merged, goals, self.prefixes[k]))
        # The planners' readiness rule is "drop ARRIVAL + process_sec", so
        # the drop stay is folded into the processing time they see.
        return Instance(tb, tasks, True, wl.ready_after_arrival, name=name,
                        source=page_header(self))


def page_header(pb: Problem) -> dict:
    """The fields the page needs beside the routes to import a plan."""
    tb, wl = pb.tb, pb.wl
    ws = []
    for b in wl.bays:
        r, c = tb.bay_cell(b)
        ws.append({"bay": b, "row": r + 1, "col": c + 1,
                   "north_entry_nodes": list(tb.flanks(tb.entry_of(b)))})
    return {
        "grid": {"rows": wl.rows, "cols": wl.cols},
        "capacity": wl.capacity,
        "rpd": {"enabled": True, "process_sec": wl.process_sec,
                "drop_dwell_sec": wl.drop_sec,
                "processing_starts": "after the drop dwell"},
        "depot": {"junction_node": 0, "grid_connection_node": 1,
                  "layout": "south of grid, single line, all slots enter "
                            "through node 0 then node 1"},
        "workstations": ws,
        "collision_window_sec": 5,
        "edge_collision_window_sec": 7,
    }


# ---------------------------------------------------------------------------
# feasibility and the static proxy
# ---------------------------------------------------------------------------
def capacity_ok(seq: list, capacity: int) -> bool:
    load = capacity
    for _b, e in seq:
        load += -1 if e == DROP else 1
        if load < 0 or load > capacity:
            return False
    return True


def proxy_makespan(pb: Problem, sol: dict) -> tuple[float, float]:
    """(makespan, sum of robot ends) under static travel times: each robot
    drives its events in order, a pick waits for drop + processing. Returns
    inf if the events wait on each other in a cycle (no robot can move)."""
    wl = pb.wl
    names = pb.names
    pos = {n: 0 for n in names}
    t = dict(pb.start_t)
    at = {n: 1 for n in names}
    dropped: dict = {}
    remaining = sum(len(sol[n]) for n in names)
    while remaining:
        moved = False
        for n in names:
            seq = sol[n]
            while pos[n] < len(seq):
                b, e = seq[pos[n]]
                if e == PICK and b not in dropped:
                    break
                ws = pb.tb.ws_of(b)
                arrive = t[n] + pb.tt(at[n], ws)
                if e == PICK:
                    arrive = max(arrive, dropped[b] + wl.ready_after_arrival)
                else:
                    dropped[b] = arrive
                arrive += wl.drop_sec
                t[n] = arrive + ENTRY_WS_SEC     # back out of the spur
                at[n] = ws
                pos[n] += 1
                remaining -= 1
                moved = True
        if not moved:
            return math.inf, math.inf
    ends = [t[n] + pb.tt(at[n], f"D{k}") for n, k in zip(names, wl.slots)]
    return max(ends), sum(ends)


def has_service(seq: list) -> bool:
    return any(a[1] == DROP and b == (a[0], PICK) for a, b in zip(seq, seq[1:]))


def feasible(pb: Problem, sol: dict) -> bool:
    if any(not sol[n] for n in pb.names):
        return False                       # every robot serves at least one bay
    if not pb.wl.service and any(has_service(sol[n]) for n in pb.names):
        return False                       # no robot waits out its own part
    if not all(capacity_ok(sol[n], pb.wl.capacity) for n in pb.names):
        return False
    return proxy_makespan(pb, sol)[0] < math.inf


# ---------------------------------------------------------------------------
# the two objectives
# ---------------------------------------------------------------------------
def evaluate(pb: Problem, sol: dict, arm: str, keep_paths: bool = False,
             pp_restarts: int = 3, pp_limit: float = 2.0) -> Evaluation:
    inst = pb.instance(sol)
    if arm == "seq":
        res = solve_astar(inst, time_limit=pp_limit)
    elif arm == "joint":
        res = solve_pp(inst, time_limit=pp_limit, restarts=pp_restarts)
    else:
        raise ValueError(f"arm must be one of {ARMS}")
    if not res.success or not res.paths:
        return INFEASIBLE
    ends = {n: p.end for n, p in res.paths.items()}
    return Evaluation(max(ends.values()), ends, res.paths if keep_paths else None)


# ---------------------------------------------------------------------------
# moves
# ---------------------------------------------------------------------------
def copy_sol(sol: dict) -> dict:
    return {n: list(s) for n, s in sol.items()}


def key_of(sol: dict) -> tuple:
    return tuple(tuple(sol[n]) for n in sorted(sol))


def remove_bays(sol: dict, bays) -> dict:
    out = {n: [ev for ev in s if ev[0] not in bays] for n, s in sol.items()}
    return out


def insertion_options(pb: Problem, sol: dict, bay: int, rng: random.Random,
                      drop_keep: int = 3):
    """Every way to put `bay` back: (robot, position) for a service stay,
    and with --split / --swaps the separated drop and pick positions.

    Hand-offs (drop by one robot, pick by another) would be N^2 L^2 options;
    instead the `drop_keep` best drop placements (by proxy, pick not yet
    placed) are each paired with every pick placement on the other robots."""
    wl = pb.wl
    for n in pb.names:
        s = sol[n]
        for i in range(len(s) + 1):
            if wl.service:
                yield {n: s[:i] + [(bay, DROP), (bay, PICK)] + s[i:]}
            if wl.split or not wl.service:
                for j in range(i + 1, len(s) + 1):
                    yield {n: s[:i] + [(bay, DROP)] + s[i:j] + [(bay, PICK)] + s[j:]}
    if wl.swaps:
        drops = []
        for a in pb.names:
            for i in range(len(sol[a]) + 1):
                seq = sol[a][:i] + [(bay, DROP)] + sol[a][i:]
                if not capacity_ok(seq, wl.capacity + 1):   # its pick comes later
                    continue
                part = dict(sol)
                part[a] = seq
                mk, tot = proxy_makespan(pb, part)
                if mk < math.inf:
                    drops.append((mk + 0.01 * tot, rng.random(), a, seq))
        drops.sort()
        for _c, _r, a, seq in drops[:drop_keep]:
            for b in pb.names:
                if b == a:
                    continue
                for j in range(len(sol[b]) + 1):
                    yield {a: seq, b: sol[b][:j] + [(bay, PICK)] + sol[b][j:]}


def best_insertions(pb: Problem, sol: dict, bay: int, rng: random.Random,
                    noise: float = 0.0, keep: int = 2) -> list:
    """The `keep` cheapest insertions of `bay`, by proxy makespan (ties by
    sum of ends), each as (cost, new solution)."""
    found = []
    for change in insertion_options(pb, sol, bay, rng):
        cand = dict(sol)
        cand.update(change)
        if not all(capacity_ok(cand[n], pb.wl.capacity) for n in change):
            continue
        mk, tot = proxy_makespan(pb, cand)
        if mk == math.inf:
            continue
        cost = mk + 0.01 * tot
        if noise:
            cost *= 1.0 + rng.uniform(-noise, noise)
        found.append((cost, cand))
    found.sort(key=lambda x: x[0])
    return found[:keep]


# destroy operators: return the bays to take out ------------------------------
def d_random(pb, sol, ev, q, rng):
    return set(rng.sample(pb.wl.bays, q))


def d_related(pb, sol, ev, q, rng):
    """Shaw removal: a seed bay and its nearest neighbours on the table,
    so the repair can re-split a crowded area between robots."""
    seed = rng.choice(pb.wl.bays)
    cell = pb.tb.bay_cell
    r0, c0 = cell(seed)
    near = sorted(pb.wl.bays, key=lambda b: (abs(cell(b)[0] - r0) + abs(cell(b)[1] - c0),
                                             rng.random()))
    return set(near[:q])


def d_critical(pb, sol, ev, q, rng):
    """Bays of the robot that finishes last under the ARM'S OWN objective
    (in the joint arm that includes its traffic waits), topped up at random."""
    if not ev.ends:
        return d_random(pb, sol, ev, q, rng)
    last = max(ev.ends, key=ev.ends.get)
    mine = list({b for b, _ in sol[last]})
    rng.shuffle(mine)
    out = set(mine[:q])
    rest = [b for b in pb.wl.bays if b not in out]
    out |= set(rng.sample(rest, max(0, q - len(out))))
    return out


DESTROY = {"random": d_random, "related": d_related, "critical": d_critical}


def pick_exchange(pb: Problem, sol: dict, rng: random.Random,
                  samples: int = 80) -> dict | None:
    """Hand-off move (--swaps): two robots trade pick-ups -- A collects the
    part at B's bay and B the one at A's. Every robot must finish with as
    many picks as drops (it can never hold more than `capacity`), so a lone
    hand-off is never feasible; trading keeps each robot's load profile
    exactly as it was. Best of `samples` random trades, by proxy."""
    picks = [(n, i, b) for n in pb.names for i, (b, e) in enumerate(sol[n]) if e == PICK]
    best, best_cost = None, math.inf
    for _ in range(samples):
        (a, i, x), (b, j, y) = rng.sample(picks, 2)
        if a == b:
            continue
        cand = dict(sol)
        cand[a] = sol[a][:i] + [(y, PICK)] + sol[a][i + 1:]
        cand[b] = sol[b][:j] + [(x, PICK)] + sol[b][j + 1:]
        if not pb.wl.service and (has_service(cand[a]) or has_service(cand[b])):
            continue
        mk, tot = proxy_makespan(pb, cand)
        if mk + 0.01 * tot < best_cost:
            best, best_cost = cand, mk + 0.01 * tot
    return best


# repair operators ---------------------------------------------------------------
def r_greedy(pb, sol, bays, rng, noise=0.0):
    order = list(bays)
    rng.shuffle(order)
    for b in order:
        opts = best_insertions(pb, sol, b, rng, noise, keep=1)
        if not opts:
            return None
        sol = opts[0][1]
    return sol


def r_noise(pb, sol, bays, rng):
    return r_greedy(pb, sol, bays, rng, noise=0.10)


def r_regret(pb, sol, bays, rng):
    """Regret-2: insert first the bay that loses most if its best slot goes."""
    left = set(bays)
    while left:
        pick, pick_regret, pick_sol = None, -1.0, None
        for b in left:
            opts = best_insertions(pb, sol, b, rng, keep=2)
            if not opts:
                return None
            regret = (opts[1][0] - opts[0][0]) if len(opts) > 1 else math.inf
            if regret > pick_regret:
                pick, pick_regret, pick_sol = b, regret, opts[0][1]
        sol = pick_sol
        left.discard(pick)
    return sol


REPAIR = {"greedy": r_greedy, "noise": r_noise, "regret": r_regret}


# ---------------------------------------------------------------------------
# construction and the search
# ---------------------------------------------------------------------------
def construct(pb: Problem, rng: random.Random) -> dict:
    """Seed every robot with one bay (two without service stays), nearest-
    first from the launch, then regret-insert the rest. Deterministic for a given seed, so both arms
    start from the same allocation."""
    bays = sorted(pb.wl.bays, key=lambda b: pb.tt(1, pb.tb.ws_of(b)))
    sol = {n: [] for n in pb.names}
    if pb.wl.service:
        for n, b in zip(pb.names, bays):
            sol[n] = [(b, DROP), (b, PICK)]
        rest = bays[len(pb.names):]
    else:
        # No service stays: each robot starts on two bays, drop both, then
        # collect both, so every robot has somewhere to insert a drop/pick.
        if len(bays) < 2 * len(pb.names):
            raise ValueError("without service stays every robot needs two bays: "
                             f"{len(bays)} bays < 2 x {len(pb.names)} robots")
        for k, n in enumerate(pb.names):
            a, b = bays[2 * k], bays[2 * k + 1]
            sol[n] = [(a, DROP), (b, DROP), (a, PICK), (b, PICK)]
        rest = bays[2 * len(pb.names):]
    out = r_regret(pb, sol, rest, rng)
    if out is None:
        raise RuntimeError("could not build a feasible starting allocation")
    return out


@dataclass
class SearchResult:
    arm: str
    best: dict
    best_eval: Evaluation
    start_eval: Evaluation
    iterations: int
    evaluations: int
    seconds: float
    history: list = field(default_factory=list)       # (t, best makespan)
    op_stats: dict = field(default_factory=dict)


def search(pb: Problem, arm: str, time_limit: float = 60.0, seed: int = 0,
           max_iterations: int | None = None, q_min: int = 2,
           q_frac: float = 0.35, t_start: float = 0.05, t_end: float = 0.002,
           segment: int = 50, reaction: float = 0.2,
           init: dict | None = None, log=None) -> SearchResult:
    """ALNS with simulated-annealing acceptance and adaptive operator weights
    (Ropke and Pisinger 2006). Temperature falls geometrically with ELAPSED
    TIME from t_start to t_end (fractions of the start makespan), so both
    arms cool on the same schedule whatever their evaluation speed."""
    rng = random.Random(seed)
    sol = copy_sol(init) if init else construct(pb, random.Random(seed))
    cache: dict = {}

    bounds: dict = {}

    def judge(s):
        k = key_of(s)
        ev = cache.get(k)
        if ev is None:
            ev = cache[k] = evaluate(pb, s, arm)
        return ev

    def bound(s):
        """Traffic-free (A*) cost: a lower bound on the joint objective."""
        k = key_of(s)
        ev = bounds.get(k)
        if ev is None:
            ev = bounds[k] = evaluate(pb, s, "seq")
        return ev

    t0 = time.monotonic()
    cur, cur_ev = sol, judge(sol)
    start_ev = cur_ev
    best, best_ev = cur, cur_ev
    history = [(0.0, best_ev.makespan)]
    T0, T1 = t_start * cur_ev.cost, t_end * cur_ev.cost
    destroy_ops = list(DESTROY) + (["handoff"] if pb.wl.swaps else [])
    w_d = {k: 1.0 for k in destroy_ops}
    w_r = {k: 1.0 for k in REPAIR}
    score_d = {k: 0.0 for k in destroy_ops}
    score_r = {k: 0.0 for k in REPAIR}
    used_d = {k: 0 for k in destroy_ops}
    used_r = {k: 0 for k in REPAIR}
    stats = {"new_best": 0, "improved": 0, "accepted": 0, "rejected": 0,
             "infeasible": 0, "duplicate": 0, "screened": 0}
    n_bays = len(pb.wl.bays)
    q_max = max(q_min, int(round(q_frac * n_bays)))
    it = 0
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= time_limit or (max_iterations and it >= max_iterations):
            break
        it += 1
        dname = rng.choices(list(w_d), weights=list(w_d.values()))[0]
        rname = rng.choices(list(w_r), weights=list(w_r.values()))[0]
        used_d[dname] += 1
        q = rng.randint(q_min, min(q_max, n_bays))
        if dname == "handoff":             # a complete move, no repair step
            rname = None
            cand = pick_exchange(pb, cur, rng)
        else:
            used_r[rname] += 1
            out = DESTROY[dname](pb, cur, cur_ev, q, rng)
            cand = REPAIR[rname](pb, remove_bays(cur, out), out, rng)
        reward = 0.0
        if cand is None or not feasible(pb, cand):
            stats["infeasible"] += 1
        elif key_of(cand) == key_of(cur):
            stats["duplicate"] += 1
        else:
            # Acceptance is decided by a draw made BEFORE the evaluation:
            # accept iff cost < cur + T*ln(1/u) (simulated annealing; covers
            # improvements). PP can only add waits to the robots' solo routes,
            # so in the joint arm a candidate whose A* cost already misses
            # that bar is rejected without running PP -- the search is
            # exactly the same, only faster.
            frac = min(1.0, elapsed / time_limit) if time_limit else 0.0
            temp = T0 * (T1 / T0) ** frac
            bar = cur_ev.cost - max(temp, 1e-9) * math.log(max(rng.random(), 1e-300))
            ev = None
            if arm == "joint":
                lb = bound(cand)
                if lb.makespan == math.inf:
                    stats["infeasible"] += 1
                elif lb.cost >= bar:
                    stats["screened"] += 1
                else:
                    ev = judge(cand)
            else:
                ev = judge(cand)
            if ev is None:
                pass
            elif ev.makespan == math.inf:
                stats["infeasible"] += 1
            else:
                delta = ev.cost - cur_ev.cost
                if ev.cost < best_ev.cost - 1e-6:
                    best, best_ev = cand, ev
                    history.append((round(elapsed, 2), ev.makespan))
                    stats["new_best"] += 1
                    reward = 33.0
                    if log:
                        log(f"  [{arm}] {elapsed:6.1f}s it {it:5d} best {ev.makespan:7.2f}"
                            f"  ({dname}/{rname}, q={q})")
                if ev.cost < bar:
                    if delta < -1e-6:
                        stats["improved"] += 1
                        reward = max(reward, 9.0)
                    else:
                        stats["accepted"] += 1
                        reward = max(reward, 13.0 if delta > 1e-6 else 0.0)
                    cur, cur_ev = cand, ev
                else:
                    stats["rejected"] += 1
        score_d[dname] += reward
        if rname:
            score_r[rname] += reward
        if it % segment == 0:
            for w, sc, used in ((w_d, score_d, used_d), (w_r, score_r, used_r)):
                for k in w:
                    if used[k]:
                        w[k] = max(0.05, (1 - reaction) * w[k]
                                   + reaction * sc[k] / used[k])
                    sc[k], used[k] = 0.0, 0
    stats.update({"weights_destroy": {k: round(v, 3) for k, v in w_d.items()},
                  "weights_repair": {k: round(v, 3) for k, v in w_r.items()},
                  "cache_size": len(cache)})
    return SearchResult(arm, best, best_ev, start_ev, it, len(cache),
                        time.monotonic() - t0, history, stats)
