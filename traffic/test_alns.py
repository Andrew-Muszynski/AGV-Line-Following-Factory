"""Tests for the joint-vs-sequential ALNS (traffic/alns.py).

    python3 -m pytest -q traffic/test_alns.py

Grouped by what each guarantee protects:
  * a solution is a legal VRP-RPD allocation: capacity holds, every bay is
    dropped and picked exactly once, waits that form a cycle are rejected;
  * both arms start from the same allocation and differ only in objective;
  * the joint arm's objective IS the PP makespan its plan is dispatched with;
  * the plan written for the page reads back valid under the validator.
"""
from __future__ import annotations

import math
import random
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from traffic import instance as I  # noqa: E402
from traffic.alns import (DROP, PICK, Problem, Workload, capacity_ok,  # noqa: E402
                          construct, evaluate, feasible, proxy_makespan, search)
from traffic.export import load_page_paths, to_page_plan  # noqa: E402
from traffic.planners import solve_pp  # noqa: E402
from traffic.validate import metrics, validate  # noqa: E402

DOE_BAYS = [1, 5, 9, 10, 14, 17, 22, 24, 27, 30, 31, 42]


def problem(**kw) -> Problem:
    opts = dict(bays=DOE_BAYS, robots=6, capacity=2, process_sec=30.0)
    opts.update(kw)
    return Problem(Workload(**opts))


def every_bay_once(pb, sol):
    c = Counter(ev for s in sol.values() for ev in s)
    return all(c[(b, DROP)] == 1 and c[(b, PICK)] == 1 for b in pb.wl.bays) \
        and sum(c.values()) == 2 * len(pb.wl.bays)


# -- legality -----------------------------------------------------------------
def test_capacity():
    assert capacity_ok([(1, DROP), (2, DROP), (1, PICK), (2, PICK)], 2)
    assert not capacity_ok([(1, DROP), (2, DROP), (3, DROP)], 2)
    assert not capacity_ok([(1, PICK)], 2)            # would carry 3


def test_cyclic_waits_are_infeasible():
    pb = problem(bays=[9, 10], robots=2, swaps=True)
    # Alvik1 picks 10 before dropping 9; Alvik2 picks 9 before dropping 10.
    sol = {"Alvik1": [(10, PICK), (9, DROP)], "Alvik2": [(9, PICK), (10, DROP)]}
    assert proxy_makespan(pb, sol)[0] == math.inf
    assert not feasible(pb, sol)


def test_construct_is_legal_and_deterministic():
    pb = problem()
    a = construct(pb, random.Random(3))
    b = construct(pb, random.Random(3))
    assert a == b
    assert every_bay_once(pb, a) and feasible(pb, a)
    assert all(a[n] for n in pb.names)


def test_search_keeps_allocation_legal():
    pb = problem(split=True)
    for arm in ("seq", "joint"):
        res = search(pb, arm, time_limit=30, max_iterations=25, seed=1)
        assert every_bay_once(pb, res.best) and feasible(pb, res.best)
        assert res.best_eval.makespan <= res.start_eval.makespan + 1e-6


# -- the two arms -------------------------------------------------------------
def test_arms_share_start_and_differ_only_in_objective():
    pb = problem()
    s = search(pb, "seq", time_limit=30, max_iterations=1, seed=5)
    j = search(pb, "joint", time_limit=30, max_iterations=1, seed=5)
    start = construct(pb, random.Random(5))
    assert evaluate(pb, start, "seq").makespan == s.start_eval.makespan
    assert evaluate(pb, start, "joint").makespan == j.start_eval.makespan
    # traffic can only delay: the joint objective is never below the solo one
    assert j.start_eval.makespan >= s.start_eval.makespan - 1e-6


def test_joint_objective_is_the_dispatched_pp_makespan():
    pb = problem()
    res = search(pb, "joint", time_limit=30, max_iterations=20, seed=2)
    inst = pb.instance(res.best)
    pp = solve_pp(inst, time_limit=30, restarts=3)
    assert abs(max(p.end for p in pp.paths.values()) - res.best_eval.makespan) < 1e-6


# -- page plan -----------------------------------------------------------------
def test_page_plan_round_trip():
    pb = problem()
    res = search(pb, "joint", time_limit=30, max_iterations=10, seed=4)
    inst = pb.instance(res.best, name="t")
    pp = solve_pp(inst, time_limit=30, restarts=20)
    rep = validate(inst, pp.paths)
    assert rep["valid"]
    plan = to_page_plan(inst, pp.paths, "pp", rep, metrics(inst, pp.paths, rep))
    assert sorted(w["bay"] for w in plan["workstations"]) == DOE_BAYS
    back = I.load_data(plan, "t")
    paths = load_page_paths(back, plan)
    assert validate(back, paths)["valid"]
    for name, p in pp.paths.items():
        assert paths[name].nodes == p.nodes
        assert abs(paths[name].end - p.end) < 1e-6


def test_redeployment_with_handoffs():
    """24 bays, 6 robots x 2 parts: half the bays can only be served with a
    part someone picked up again, possibly from another robot's bay."""
    extra = [6, 7, 11, 12, 13, 19, 23, 25, 32, 37, 40, 44]
    pb = problem(bays=sorted(DOE_BAYS + extra), swaps=True, split=True,
                 service=False, drop_sec=5.0)
    res = search(pb, "joint", time_limit=60, max_iterations=15, seed=1)
    assert every_bay_once(pb, res.best) and feasible(pb, res.best)
    for n in pb.names:
        assert capacity_ok(res.best[n], 2)
    inst = pb.instance(res.best)
    pp = solve_pp(inst, time_limit=30, restarts=20)
    assert pp.success and validate(inst, pp.paths)["valid"]


def test_astar_cost_bounds_pp_cost():
    """The joint arm skips PP when the A* cost already fails the acceptance
    bar; that is exact only if PP never comes out cheaper than A*."""
    pb = problem(bays=sorted(DOE_BAYS + [6, 7, 11, 12, 13, 19]), split=True)
    rng = random.Random(0)
    sol = construct(pb, rng)
    from traffic.alns import DESTROY, REPAIR, remove_bays
    for k in range(25):
        out = DESTROY["random"](pb, sol, None, 4, rng)
        cand = REPAIR["noise"](pb, remove_bays(sol, out), out, rng)
        if cand is None:
            continue
        a, j = evaluate(pb, cand, "seq"), evaluate(pb, cand, "joint")
        if j.makespan < math.inf:
            assert j.cost >= a.cost - 1e-6
        sol = cand


def test_no_service_rule_and_ready_after_drop():
    """Operator rule 2026-10-29: the drop is the page's 2 s DWELL after the
    robot squares up (ROTATE_180, 5 s); the part is READY 30 s after the
    drop ends, for any robot; no robot waits in a bay for its own part."""
    from traffic.alns import has_service
    from traffic.instance import drop_times
    extra = [6, 7, 11, 12, 13, 19, 23, 25, 32, 37, 40, 44]
    pb = problem(bays=sorted(DOE_BAYS + extra), swaps=True, split=True,
                 service=False, drop_sec=2.0)
    res = search(pb, "joint", time_limit=60, max_iterations=40, seed=2)
    assert not any(has_service(res.best[n]) for n in pb.names)
    inst = pb.instance(res.best)
    pp = solve_pp(inst, time_limit=30, restarts=20)
    assert validate(inst, pp.paths)["valid"]
    drops = drop_times(inst, pp.paths)
    for task in inst.tasks:
        p = pp.paths[task.name]
        gi = 0
        for i in range(p.prefix, len(p.steps)):
            st = p.steps[i]
            if gi < len(task.goals) - 1 and st.node == task.goals[gi].node:
                g = task.goals[gi]
                if g.mode == "drop":
                    assert st.wait >= 2.0 - 1e-6          # the drop DWELL
                if g.mode == "pick":                       # leaves only when READY
                    assert st.drive_start >= drops[g.bay] + 5.0 + 2.0 + 30.0 - 1e-6
                gi += 1
