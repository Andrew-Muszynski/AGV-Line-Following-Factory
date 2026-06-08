"""
ALNS-lite solver for VRP-RPD.

This is a CPU-only, single-instance adaptation of the ALNS framework from
Sodhi, Prasad & Saseendran (2026). It implements:
  - Sweep + nearest-neighbor initial construction
  - 3 destroy operators: Random, Worst, Critical-Path
  - 2 repair operators: Greedy, Regret-2
  - Simulated annealing acceptance with reheating
  - Roulette-wheel adaptive operator selection

Simplifications relative to the paper:
  - No GPU parallelism (32 concurrent instances -> 1)
  - No BRKGA stage (ALNS incumbent is the final answer)
  - No Shaw removal (less impactful at n <= 64)

For a 64-node instance this converges in seconds to minutes on a laptop.
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Callable

from .instance import Instance
from .schedule import Schedule, Stop, Op


# ---------------------------------------------------------------------------
# Schedule evaluation (simulates a schedule to compute makespan)
# ---------------------------------------------------------------------------

def evaluate_makespan(schedule: Schedule, instance: Instance) -> float:
    """
    Simulate the schedule deterministically and return makespan.

    Uses a multi-pass approach: we iterate over all stops, advancing each
    AGV's clock, and defer pickups whose dropoff+processing isn't complete.
    Returns +inf if the schedule is infeasible.
    """
    n_agvs = instance.num_agvs
    depot = instance.depot_node

    # Per-AGV state
    agv_time = [0.0] * n_agvs
    agv_pos = [depot] * n_agvs
    agv_idx = [0] * n_agvs  # pointer into routes[v]

    drop_complete: dict[int, float] = {}  # customer -> earliest pickup time (T_drop + p_c)

    # We iterate in rounds, each round each AGV tries to execute its next stop.
    # A stop is deferred if it's a pickup and the resource isn't ready yet.
    max_rounds = sum(len(r) for r in schedule.routes) * 3 + 10
    for _round in range(max_rounds):
        progress = False
        all_done = True
        for v in range(n_agvs):
            route = schedule.routes[v]
            if agv_idx[v] >= len(route):
                continue
            all_done = False
            stop = route[agv_idx[v]]

            # Travel to the stop
            travel = instance.distance(agv_pos[v], stop.node)
            arrival = agv_time[v] + travel

            if stop.op == Op.DROPOFF:
                # Dropoff happens on arrival
                agv_time[v] = arrival
                agv_pos[v] = stop.node
                drop_complete[stop.node] = arrival + instance.processing_times[stop.node]
                agv_idx[v] += 1
                progress = True
            else:  # PICKUP
                ready = drop_complete.get(stop.node)
                if ready is None:
                    # Dropoff hasn't happened yet — defer this AGV this round
                    continue
                # AGV must wait until processing is complete
                agv_time[v] = max(arrival, ready)
                agv_pos[v] = stop.node
                agv_idx[v] += 1
                progress = True

        if all_done:
            break
        if not progress:
            # Deadlock: some pickup is waiting for a dropoff that never comes
            return math.inf

    # Return to depot
    return_times = []
    for v in range(n_agvs):
        rt = agv_time[v] + instance.distance(agv_pos[v], depot)
        return_times.append(rt)
    return max(return_times) if return_times else 0.0


# ---------------------------------------------------------------------------
# Initial construction: sweep + nearest-neighbor
# ---------------------------------------------------------------------------

def _sweep_assign(instance: Instance) -> list[list[int]]:
    """Partition customers into num_agvs sectors by pseudo-angle from depot."""
    depot_r, depot_c = instance.depot_row, instance.depot_col
    angled = []
    for c in instance.demand:
        r, col = instance.node_to_rc(c)
        dr, dc = r - depot_r, col - depot_c
        theta = math.atan2(dr, dc)
        angled.append((theta, c))
    angled.sort()

    buckets: list[list[int]] = [[] for _ in range(instance.num_agvs)]
    per = max(1, len(angled) // instance.num_agvs)
    for i, (_, c) in enumerate(angled):
        v = min(i // per, instance.num_agvs - 1)
        buckets[v].append(c)
    return buckets


def build_initial(instance: Instance, rng: random.Random) -> Schedule:
    """Greedy initial schedule: each AGV does all its drops then all its picks."""
    buckets = _sweep_assign(instance)
    # Balance within capacity
    for b in buckets:
        while len(b) > instance.capacity:
            # Move excess to the smallest bucket
            target = min(range(instance.num_agvs), key=lambda i: len(buckets[i]))
            if len(buckets[target]) < instance.capacity:
                buckets[target].append(b.pop())
            else:
                break  # all full

    routes: list[list[Stop]] = []
    for v, bucket in enumerate(buckets):
        # Nearest-neighbor ordering from depot
        ordered = []
        remaining = list(bucket)
        cur = instance.depot_node
        while remaining:
            nxt = min(remaining, key=lambda c: instance.distance(cur, c))
            ordered.append(nxt)
            remaining.remove(nxt)
            cur = nxt
        # Drop all, then pick all in the same order (same AGV round-trip)
        route = [Stop(c, Op.DROPOFF) for c in ordered] + [
            Stop(c, Op.PICKUP) for c in ordered
        ]
        routes.append(route)

    return Schedule(routes=routes)


# ---------------------------------------------------------------------------
# Destroy operators (remove customers entirely — both D and P)
# ---------------------------------------------------------------------------

def _remove_customers(schedule: Schedule, customers: set[int]) -> Schedule:
    new_routes = []
    for route in schedule.routes:
        new_routes.append([s for s in route if s.node not in customers])
    return Schedule(routes=new_routes)


def destroy_random(
    schedule: Schedule, instance: Instance, q: int, rng: random.Random
) -> tuple[Schedule, list[int]]:
    picked = rng.sample(instance.demand, min(q, len(instance.demand)))
    return _remove_customers(schedule, set(picked)), picked


def destroy_worst(
    schedule: Schedule, instance: Instance, q: int, rng: random.Random
) -> tuple[Schedule, list[int]]:
    """Remove customers whose removal most decreases makespan."""
    base = evaluate_makespan(schedule, instance)
    gains = []
    for c in instance.demand:
        trial = _remove_customers(schedule, {c})
        mk = evaluate_makespan(trial, instance)
        if mk < math.inf:
            gains.append((base - mk, c))
    gains.sort(reverse=True)
    # Power-biased selection (p=6 per the paper)
    picked = []
    remaining = gains[:]
    while remaining and len(picked) < q:
        # Sample rank r with probability proportional to r^-6
        weights = [1.0 / ((i + 1) ** 6) for i in range(len(remaining))]
        total = sum(weights)
        x = rng.uniform(0, total)
        acc = 0.0
        idx = 0
        for i, w in enumerate(weights):
            acc += w
            if acc >= x:
                idx = i
                break
        picked.append(remaining.pop(idx)[1])
    return _remove_customers(schedule, set(picked)), picked


def destroy_critical_path(
    schedule: Schedule, instance: Instance, q: int, rng: random.Random
) -> tuple[Schedule, list[int]]:
    """Remove customers from the bottleneck AGV's route."""
    # Identify bottleneck: AGV with latest return time
    n_agvs = instance.num_agvs
    depot = instance.depot_node

    # Per-AGV return time in this schedule
    agv_time = [0.0] * n_agvs
    agv_pos = [depot] * n_agvs
    drop_complete: dict[int, float] = {}
    # Quick forward simulation
    max_rounds = sum(len(r) for r in schedule.routes) * 3 + 10
    idx = [0] * n_agvs
    for _ in range(max_rounds):
        progress = False
        done = True
        for v in range(n_agvs):
            route = schedule.routes[v]
            if idx[v] >= len(route):
                continue
            done = False
            stop = route[idx[v]]
            arr = agv_time[v] + instance.distance(agv_pos[v], stop.node)
            if stop.op == Op.DROPOFF:
                agv_time[v] = arr
                agv_pos[v] = stop.node
                drop_complete[stop.node] = arr + instance.processing_times[stop.node]
                idx[v] += 1
                progress = True
            else:
                ready = drop_complete.get(stop.node)
                if ready is None:
                    continue
                agv_time[v] = max(arr, ready)
                agv_pos[v] = stop.node
                idx[v] += 1
                progress = True
        if done or not progress:
            break
    returns = [agv_time[v] + instance.distance(agv_pos[v], depot) for v in range(n_agvs)]
    bottleneck = max(range(n_agvs), key=lambda v: returns[v])

    # Customers touched by the bottleneck route
    candidates = list({s.node for s in schedule.routes[bottleneck]})
    rng.shuffle(candidates)
    picked = candidates[:q]
    # Top up from other routes if needed
    if len(picked) < q:
        extras = [c for c in instance.demand if c not in picked]
        rng.shuffle(extras)
        picked.extend(extras[: q - len(picked)])
    return _remove_customers(schedule, set(picked)), picked


# ---------------------------------------------------------------------------
# Repair operators (reinsert customers as D+P pairs)
# ---------------------------------------------------------------------------

def _try_insert(
    schedule: Schedule,
    instance: Instance,
    customer: int,
    drop_agv: int,
    drop_pos: int,
    pick_agv: int,
    pick_pos: int,
) -> Schedule | None:
    """Try inserting D at (drop_agv, drop_pos) and P at (pick_agv, pick_pos).
    Returns the new schedule, or None if the insertion is locally infeasible
    (e.g. same-AGV with pickup before dropoff, or capacity violation)."""
    # If same AGV, pickup must come strictly after dropoff (after the drop is inserted)
    if drop_agv == pick_agv and pick_pos <= drop_pos:
        return None

    new_routes = [list(r) for r in schedule.routes]
    new_routes[drop_agv].insert(drop_pos, Stop(customer, Op.DROPOFF))
    # If same route, pick_pos shifted because we inserted before it (when pick_pos > drop_pos)
    if drop_agv == pick_agv and pick_pos > drop_pos:
        new_routes[pick_agv].insert(pick_pos + 1, Stop(customer, Op.PICKUP))
    else:
        new_routes[pick_agv].insert(pick_pos, Stop(customer, Op.PICKUP))

    # Quick capacity check per route. AGV starts carrying one resource per
    # dropoff it will perform; load decreases on drops, increases on picks.
    # Load must stay in [0, k] throughout.
    for route in new_routes:
        num_drops = sum(1 for s in route if s.op == Op.DROPOFF)
        load = num_drops
        if load > instance.capacity:
            return None  # carrying too many to start with
        for s in route:
            if s.op == Op.DROPOFF:
                load -= 1
            else:
                load += 1
            if load > instance.capacity or load < 0:
                return None
    return Schedule(routes=new_routes)


def _best_insertion(
    schedule: Schedule, instance: Instance, customer: int
) -> tuple[float, Schedule] | None:
    """Find the best (lowest-makespan) insertion for one customer."""
    best: tuple[float, Schedule] | None = None
    n_agvs = instance.num_agvs
    for da in range(n_agvs):
        for dp in range(len(schedule.routes[da]) + 1):
            for pa in range(n_agvs):
                for pp in range(len(schedule.routes[pa]) + 1):
                    cand = _try_insert(schedule, instance, customer, da, dp, pa, pp)
                    if cand is None:
                        continue
                    mk = evaluate_makespan(cand, instance)
                    if mk == math.inf:
                        continue
                    if best is None or mk < best[0]:
                        best = (mk, cand)
    return best


def repair_greedy(
    schedule: Schedule, instance: Instance, removed: list[int], rng: random.Random
) -> Schedule:
    """Greedy: at each step, insert the customer with the cheapest best-insertion."""
    remaining = list(removed)
    rng.shuffle(remaining)
    current = schedule
    while remaining:
        # Evaluate each remaining customer's best insertion
        best_choice: tuple[float, int, Schedule] | None = None
        for c in remaining:
            result = _best_insertion(current, instance, c)
            if result is None:
                continue
            mk, cand = result
            if best_choice is None or mk < best_choice[0]:
                best_choice = (mk, c, cand)
        if best_choice is None:
            # No feasible insertion — abort (shouldn't happen for reasonable instances)
            break
        _, chosen, current = best_choice
        remaining.remove(chosen)
    return current


def repair_regret2(
    schedule: Schedule, instance: Instance, removed: list[int], rng: random.Random
) -> Schedule:
    """Regret-2: insert the customer that most 'regrets' not getting its best spot."""
    remaining = list(removed)
    rng.shuffle(remaining)
    current = schedule
    while remaining:
        # For each remaining customer, find best and second-best insertions
        best_choice = None  # (regret, customer, best_schedule, best_mk)
        for c in remaining:
            # Collect top 2 insertion costs
            costs: list[tuple[float, Schedule]] = []
            for da in range(instance.num_agvs):
                for dp in range(len(current.routes[da]) + 1):
                    for pa in range(instance.num_agvs):
                        for pp in range(len(current.routes[pa]) + 1):
                            cand = _try_insert(current, instance, c, da, dp, pa, pp)
                            if cand is None:
                                continue
                            mk = evaluate_makespan(cand, instance)
                            if mk == math.inf:
                                continue
                            costs.append((mk, cand))
            if not costs:
                continue
            costs.sort(key=lambda x: x[0])
            best_mk, best_sched = costs[0]
            second_mk = costs[1][0] if len(costs) > 1 else best_mk
            regret = second_mk - best_mk
            if best_choice is None or regret > best_choice[0]:
                best_choice = (regret, c, best_sched, best_mk)
        if best_choice is None:
            break
        _, chosen, new_sched, _ = best_choice
        current = new_sched
        remaining.remove(chosen)
    return current


# ---------------------------------------------------------------------------
# Local search: pickup repositioning (per paper Section 4.2.6)
# ---------------------------------------------------------------------------

def _capacity_valid(routes: list[list[Stop]], k: int) -> bool:
    """Check that every route's load trajectory stays in [0, k]."""
    for route in routes:
        num_drops = sum(1 for s in route if s.op == Op.DROPOFF)
        load = num_drops
        if load > k:
            return False
        for s in route:
            if s.op == Op.DROPOFF:
                load -= 1
            else:
                load += 1
            if load < 0 or load > k:
                return False
    return True


def pickup_reposition(
    schedule: Schedule, instance: Instance, max_passes: int = 3
) -> Schedule:
    """
    Try moving each pickup to every feasible (agv, position) and apply the
    best improvement. Iterates up to max_passes times or until no improvement.

    This is the critical operator for cross-agent coordination: it decouples
    a pickup from its dropoff's AGV without requiring the destroy-repair cycle.
    """
    current = schedule
    current_mk = evaluate_makespan(current, instance)

    for _pass in range(max_passes):
        improved = False
        # Collect all pickup locations
        pick_locs: list[tuple[int, int, int]] = []  # (agv, pos, customer)
        for v, route in enumerate(current.routes):
            for i, s in enumerate(route):
                if s.op == Op.PICKUP:
                    pick_locs.append((v, i, s.node))

        for src_agv, src_pos, customer in pick_locs:
            best_schedule = current
            best_mk = current_mk
            # Try every (target_agv, target_pos)
            for tgt_agv in range(instance.num_agvs):
                route = current.routes[tgt_agv]
                # Possible insertion positions in target (after removal from source)
                max_pos = len(route) + (1 if tgt_agv != src_agv else 0)
                for tgt_pos in range(max_pos):
                    if tgt_agv == src_agv and tgt_pos == src_pos:
                        continue  # no-op
                    # Build candidate: remove P from src, insert at tgt
                    new_routes = [list(r) for r in current.routes]
                    new_routes[src_agv].pop(src_pos)
                    # If same AGV and tgt_pos > src_pos, adjust for removal
                    adj_pos = tgt_pos
                    if tgt_agv == src_agv and tgt_pos > src_pos:
                        adj_pos -= 1
                    new_routes[tgt_agv].insert(adj_pos, Stop(customer, Op.PICKUP))

                    if not _capacity_valid(new_routes, instance.capacity):
                        continue
                    cand = Schedule(routes=new_routes)
                    mk = evaluate_makespan(cand, instance)
                    if mk < best_mk:
                        best_mk = mk
                        best_schedule = cand

            if best_mk < current_mk:
                current = best_schedule
                current_mk = best_mk
                improved = True
                # After modification, pick_locs indices may be stale;
                # break and restart the pass
                break

        if not improved:
            break

    return current


def pickup_swap(
    schedule: Schedule, instance: Instance, max_passes: int = 3
) -> Schedule:
    """
    Swap two pickups between different AGVs. Unlike pickup_reposition, this
    preserves each AGV's drop/pick count, so it works even when all AGVs are
    at full capacity utilization (demand == num_agvs * capacity).

    This is the move that unlocks cross-agent coordination when the fleet
    is fully loaded, which is the default in our setup.
    """
    current = schedule
    current_mk = evaluate_makespan(current, instance)

    for _pass in range(max_passes):
        improved = False
        # Collect pickups grouped by AGV
        picks_by_agv: list[list[tuple[int, int]]] = [
            [(i, s.node) for i, s in enumerate(route) if s.op == Op.PICKUP]
            for route in current.routes
        ]

        best_schedule = current
        best_mk = current_mk
        for a in range(instance.num_agvs):
            for b in range(a + 1, instance.num_agvs):
                for pa_idx, cust_a in picks_by_agv[a]:
                    for pb_idx, cust_b in picks_by_agv[b]:
                        new_routes = [list(r) for r in current.routes]
                        # Swap the two pickup stops (preserving positions)
                        new_routes[a][pa_idx] = Stop(cust_b, Op.PICKUP)
                        new_routes[b][pb_idx] = Stop(cust_a, Op.PICKUP)
                        if not _capacity_valid(new_routes, instance.capacity):
                            continue
                        cand = Schedule(routes=new_routes)
                        mk = evaluate_makespan(cand, instance)
                        if mk < best_mk:
                            best_mk = mk
                            best_schedule = cand

        if best_mk < current_mk:
            current = best_schedule
            current_mk = best_mk
            improved = True

        if not improved:
            break

    return current



def dropoff_reposition(
    schedule: Schedule, instance: Instance, max_passes: int = 2
) -> Schedule:
    """
    Mirror of pickup_reposition for dropoffs. Important because moving a
    dropoff changes the initial load of both source and target AGVs.
    """
    current = schedule
    current_mk = evaluate_makespan(current, instance)

    for _pass in range(max_passes):
        improved = False
        drop_locs: list[tuple[int, int, int]] = []
        for v, route in enumerate(current.routes):
            for i, s in enumerate(route):
                if s.op == Op.DROPOFF:
                    drop_locs.append((v, i, s.node))

        for src_agv, src_pos, customer in drop_locs:
            best_schedule = current
            best_mk = current_mk
            for tgt_agv in range(instance.num_agvs):
                route = current.routes[tgt_agv]
                max_pos = len(route) + (1 if tgt_agv != src_agv else 0)
                for tgt_pos in range(max_pos):
                    if tgt_agv == src_agv and tgt_pos == src_pos:
                        continue
                    new_routes = [list(r) for r in current.routes]
                    new_routes[src_agv].pop(src_pos)
                    adj_pos = tgt_pos
                    if tgt_agv == src_agv and tgt_pos > src_pos:
                        adj_pos -= 1
                    new_routes[tgt_agv].insert(adj_pos, Stop(customer, Op.DROPOFF))
                    if not _capacity_valid(new_routes, instance.capacity):
                        continue
                    cand = Schedule(routes=new_routes)
                    mk = evaluate_makespan(cand, instance)
                    if mk < best_mk:
                        best_mk = mk
                        best_schedule = cand

            if best_mk < current_mk:
                current = best_schedule
                current_mk = best_mk
                improved = True
                break

        if not improved:
            break

    return current




@dataclass
class ALNSConfig:
    max_iter: int = 2000
    T0_coef: float = 0.30
    alpha: float = 0.9995
    reheat_after: int = 500
    sigma1: float = 33
    sigma2: float = 9
    sigma3: float = 13
    reaction: float = 0.1
    min_weight: float = 0.1
    weight_update_every: int = 50
    time_limit_sec: float = 60.0
    verbose: bool = True


@dataclass
class ALNSResult:
    schedule: Schedule
    makespan: float
    iterations: int
    elapsed_sec: float
    history: list[tuple[int, float]] = field(default_factory=list)


def solve_alns(
    instance: Instance,
    cfg: ALNSConfig | None = None,
    rng: random.Random | None = None,
) -> ALNSResult:
    cfg = cfg or ALNSConfig()
    rng = rng or random.Random(instance.seed)

    # Build initial solution
    current = build_initial(instance, rng)
    current_mk = evaluate_makespan(current, instance)
    if current_mk == math.inf:
        raise RuntimeError("Initial construction produced infeasible schedule")

    best = current
    best_mk = current_mk

    T = cfg.T0_coef * current_mk  # initial temperature

    destroy_ops: list[tuple[str, Callable]] = [
        ("random", destroy_random),
        ("worst", destroy_worst),
        ("critical", destroy_critical_path),
    ]
    repair_ops: list[tuple[str, Callable]] = [
        ("greedy", repair_greedy),
        ("regret2", repair_regret2),
    ]

    d_weights = [1.0] * len(destroy_ops)
    r_weights = [1.0] * len(repair_ops)
    d_scores = [0.0] * len(destroy_ops)
    r_scores = [0.0] * len(repair_ops)
    d_attempts = [0] * len(destroy_ops)
    r_attempts = [0] * len(repair_ops)

    def select(weights: list[float]) -> int:
        total = sum(weights)
        x = rng.uniform(0, total)
        acc = 0.0
        for i, w in enumerate(weights):
            acc += w
            if acc >= x:
                return i
        return len(weights) - 1

    history: list[tuple[int, float]] = [(0, best_mk)]
    stagnation = 0
    start = time.time()
    n = instance.num_customers
    qmax = max(2, n // 20)
    qmin = max(2, qmax // 2)

    for it in range(1, cfg.max_iter + 1):
        if time.time() - start > cfg.time_limit_sec:
            if cfg.verbose:
                print(f"  [time limit reached at iter {it}]")
            break

        di = select(d_weights)
        ri = select(r_weights)
        d_attempts[di] += 1
        r_attempts[ri] += 1

        q = rng.randint(qmin, qmax)
        partial, removed = destroy_ops[di][1](current, instance, q, rng)
        candidate = repair_ops[ri][1](partial, instance, removed, rng)
        cand_mk = evaluate_makespan(candidate, instance)

        if cand_mk == math.inf:
            pass  # reject
        elif cand_mk < current_mk:
            current, current_mk = candidate, cand_mk
            d_scores[di] += cfg.sigma2
            r_scores[ri] += cfg.sigma2
            if cand_mk < best_mk:
                best, best_mk = candidate, cand_mk
                d_scores[di] += cfg.sigma1 - cfg.sigma2
                r_scores[ri] += cfg.sigma1 - cfg.sigma2
                stagnation = 0
                history.append((it, best_mk))
                if cfg.verbose and it % 1 == 0:
                    print(
                        f"  iter {it:4d}: new best = {best_mk:8.2f} "
                        f"(destroy={destroy_ops[di][0]}, repair={repair_ops[ri][0]})"
                    )
                # Polish with pickup repositioning + swap — these find
                # the cross-agent coordination opportunities.
                polished = pickup_reposition(best, instance, max_passes=3)
                polished = pickup_swap(polished, instance, max_passes=2)
                polished_mk = evaluate_makespan(polished, instance)
                if polished_mk < best_mk:
                    best, best_mk = polished, polished_mk
                    current, current_mk = polished, polished_mk
                    history.append((it, best_mk))
                    if cfg.verbose:
                        print(f"  iter {it:4d}:   + polish -> {best_mk:8.2f}")
        elif rng.random() < math.exp(-(cand_mk - current_mk) / max(T, 1e-9)):
            current, current_mk = candidate, cand_mk
            d_scores[di] += cfg.sigma3
            r_scores[ri] += cfg.sigma3

        T *= cfg.alpha
        stagnation += 1

        if stagnation >= cfg.reheat_after:
            T = 0.5 * cfg.T0_coef * best_mk
            stagnation = 0
            if cfg.verbose:
                print(f"  iter {it:4d}: reheat, T -> {T:.2f}")

        if it % cfg.weight_update_every == 0:
            for i in range(len(d_weights)):
                if d_attempts[i] > 0:
                    d_weights[i] = max(
                        cfg.min_weight,
                        (1 - cfg.reaction) * d_weights[i]
                        + cfg.reaction * d_scores[i] / d_attempts[i],
                    )
                d_scores[i] = 0
                d_attempts[i] = 0
            for i in range(len(r_weights)):
                if r_attempts[i] > 0:
                    r_weights[i] = max(
                        cfg.min_weight,
                        (1 - cfg.reaction) * r_weights[i]
                        + cfg.reaction * r_scores[i] / r_attempts[i],
                    )
                r_scores[i] = 0
                r_attempts[i] = 0

    # Final polish: repeatedly apply pickup swap, pickup reposition, and
    # dropoff reposition until no improvement. pickup_swap is critical when
    # AGVs run at full capacity (demand == num_agvs * capacity).
    if cfg.verbose:
        print(f"  [polish] running final swap + reposition...")
    for _ in range(8):
        before = best_mk
        best = pickup_swap(best, instance, max_passes=3)
        best_mk = evaluate_makespan(best, instance)
        best = pickup_reposition(best, instance, max_passes=3)
        best_mk = evaluate_makespan(best, instance)
        best = dropoff_reposition(best, instance, max_passes=2)
        best_mk = evaluate_makespan(best, instance)
        if best_mk >= before - 1e-6:
            break
        if cfg.verbose:
            print(f"  [polish] makespan -> {best_mk:.2f}")
    history.append((it, best_mk))

    elapsed = time.time() - start
    return ALNSResult(
        schedule=best,
        makespan=best_mk,
        iterations=it,
        elapsed_sec=elapsed,
        history=history,
    )
