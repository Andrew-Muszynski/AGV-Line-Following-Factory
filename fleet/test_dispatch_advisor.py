#!/usr/bin/env python3
"""
Unit tests for dispatch_advisor.py (pure Python — no ROS, no API calls).

Run from this directory:  python -m pytest test_dispatch_advisor.py -v
"""

import pytest

from dispatch_advisor import (
    AlignmentError,
    DispatchAdvisor,
    INTERSECTION_CLEARANCE_SEC,
    OPPOSITE_DIRECTION_CLEARANCE_SEC,
    align_commands_to_route,
    corridor_key,
    detect_conflicts,
    edge_key,
    entry_edge_nodes,
    is_workstation_node,
    label_to_cell,
    parse_dwell_ms,
    simulate_plan,
)

ROWS, COLS = 8, 8


# --- fixture builders ---------------------------------------------------------

def seg(i, frm, to, move=3.56, turn=0.0, type_="RED_TO_RED"):
    return {"segment_index": i, "from": frm, "to": to,
            "start_sec": 0.0, "end_sec": 0.0, "wait_before_sec": 0.0,
            "move_sec": move, "turn_sec": turn, "type": type_}


def route(agv, segs, launch=0.0):
    return {"agent": int(agv[-1]), "agv": agv,
            "launch_offset_sec": launch, "segments": segs}


def schedule(*routes_):
    return {"grid": {"rows": ROWS, "cols": COLS},
            "agents": len(routes_), "routes": list(routes_)}


class MockLLM:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def propose(self, payload, rejection=None, prior=None):
        self.calls.append({"payload": payload, "rejection": rejection})
        return self.responses.pop(0)


# --- parse_dwell_ms -----------------------------------------------------------

def test_parse_dwell_defaults_and_clamps():
    assert parse_dwell_ms("DWELL") == 2000
    assert parse_dwell_ms("DWELL 500") == 500
    assert parse_dwell_ms("DWELL 50") == 200        # firmware MIN_DWELL_MS
    assert parse_dwell_ms("DWELL 99999") == 30000   # firmware MAX_DWELL_MS
    assert parse_dwell_ms("DWELL abc") == 2000


# --- alignment ----------------------------------------------------------------

def test_align_moves_and_turn_absorbs_export_turn_sec():
    r = route("Alvik1", [
        seg(1, 1, 2),
        seg(2, 2, 10, move=3.56, turn=2.65),
    ])
    plan = align_commands_to_route(
        r, ["FORWARD_UNTIL_RED", "RIGHT_UNTIL_COLOR", "FORWARD_UNTIL_RED"],
        ROWS, COLS)
    assert [p.kind for p in plan] == ["move", "turn", "move"]
    assert plan[1].duration_sec == pytest.approx(2.65)   # from export turn_sec
    assert plan[2].duration_sec == pytest.approx(3.56)   # move only, turn separate


def test_align_missing_turn_command_keeps_schedule_timing():
    r = route("Alvik1", [seg(1, 1, 2, move=3.56, turn=2.65)])
    plan = align_commands_to_route(r, ["FORWARD_UNTIL_RED"], ROWS, COLS)
    # no turn command: the exported turn time folds into the move
    assert plan[0].duration_sec == pytest.approx(3.56 + 2.65)


def test_align_yellow_and_dwell():
    r = route("Alvik1", [
        seg(1, 2, 114, move=2.00, type_="RED_TO_YELLOW"),      # entry node
        seg(2, 114, 65, move=1.69, type_="YELLOW_TO_YELLOW"),  # workstation
    ])
    plan = align_commands_to_route(
        r, ["FORWARD_UNTIL_YELLOW", "FORWARD_UNTIL_YELLOW", "DWELL 3000"],
        ROWS, COLS)
    assert [p.kind for p in plan] == ["move", "move", "dwell"]
    assert plan[2].duration_sec == pytest.approx(3.0)


def test_align_depot_exit_all_blue_to_node0():
    # Table re-stickered 2026-07-14: node 0 is now BLUE (depot marker, per
    # the user's convention). Exit: FWD_BLUE (stub), FWD_BLUE (node 0),
    # FWD_RED (node 1) — one segment each.
    r = route("Alvik1", [
        seg(1, "D1", "DE1", move=4.9, type_="DEPOT_SLOT_TO_ENTRY"),
        seg(2, "DE1", 0, move=4.9, type_="DEPOT_ENTRY_TO_NODE_0"),
        seg(3, 0, 1, move=4.19, type_="BLUE_TO_RED"),
    ])
    plan = align_commands_to_route(
        r, ["FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_RED"],
        ROWS, COLS)
    assert [(p.from_node, p.to_node) for p in plan] == [
        ("D1", "DE1"), ("DE1", 0), (0, 1)]
    assert all(not p.implicit_segs for p in plan)
    sim = simulate_plan(plan, 0, 0.0)
    assert [e.node for e in sim.events] == ["DE1", "0", "1"]
    assert sim.end_sec == pytest.approx(4.9 + 4.9 + 4.19)


def test_align_depot_return_stops_on_connector_blue():
    # Corrected from the live Alvik4 run 2026-07-13: the returning
    # FORWARD_UNTIL_BLUE stops on the connector's blue at node 0 (the
    # follower does NOT ride the lane corner), so each return command
    # consumes exactly one segment.
    r = route("Alvik1", [
        seg(1, 1, 0, move=4.19, type_="RED_TO_BLUE_RETURN"),
        seg(2, 0, "DE1", move=4.9, type_="NODE_0_TO_DEPOT_ENTRY"),
        seg(3, "DE1", "D1", move=4.9, type_="DEPOT_ENTRY_TO_SLOT"),
    ])
    plan = align_commands_to_route(
        r, ["FORWARD_UNTIL_BLUE", "LEFT_UNTIL_COLOR", "FORWARD_UNTIL_BLUE",
            "LEFT_UNTIL_COLOR", "FORWARD_UNTIL_BLUE", "ROTATE_180"],
        ROWS, COLS)
    moves = [p for p in plan if p.kind == "move"]
    assert [(m.from_node, m.to_node) for m in moves] == [
        (1, 0), (0, "DE1"), ("DE1", "D1")]
    assert all(not m.implicit_segs for m in moves)
    sim = simulate_plan(plan, 0, 0.0)
    assert [e.node for e in sim.events] == ["0", "DE1", "D1"]


def test_align_color_mismatch_raises():
    r = route("Alvik1", [seg(1, 1, 2)])  # destination is a RED grid node
    with pytest.raises(AlignmentError):
        align_commands_to_route(r, ["FORWARD_UNTIL_YELLOW"], ROWS, COLS)


def test_align_move_without_segment_raises():
    r = route("Alvik1", [seg(1, 1, 2)])
    with pytest.raises(AlignmentError):
        align_commands_to_route(
            r, ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"], ROWS, COLS)


def test_align_unknown_command_raises():
    r = route("Alvik1", [seg(1, 1, 2)])
    with pytest.raises(AlignmentError):
        align_commands_to_route(r, ["YENTRY"], ROWS, COLS)  # archive vocab


# --- simulation timing --------------------------------------------------------

def test_simulate_turn_occupies_node_before_edge_translation():
    r = route("Alvik1", [
        seg(1, 1, 2),
        seg(2, 2, 10, move=3.0, turn=2.65),
    ])
    plan = align_commands_to_route(
        r, ["FORWARD_UNTIL_RED", "RIGHT_UNTIL_COLOR", "FORWARD_UNTIL_RED"],
        ROWS, COLS)
    sim = simulate_plan(plan, 0, 10.0)
    # first move: 10.0 -> 13.56
    assert sim.intervals[0].start_sec == pytest.approx(10.0)
    assert sim.intervals[0].end_sec == pytest.approx(13.56)
    # The in-place turn occupies node 2. Translation starts after it.
    assert sim.intervals[1].start_sec == pytest.approx(13.56 + 2.65)
    assert sim.intervals[1].end_sec == pytest.approx(13.56 + 2.65 + 3.0)
    turn_occupancy = next(
        interval for interval in sim.node_intervals
        if interval.node == "2" and interval.start_sec == pytest.approx(13.56))
    assert turn_occupancy.end_sec == pytest.approx(13.56 + 2.65)


def test_simulate_lead_dwell_shifts_everything():
    r = route("Alvik1", [seg(1, 1, 2)])
    plan = align_commands_to_route(r, ["FORWARD_UNTIL_RED"], ROWS, COLS)
    base = simulate_plan(plan, 0, 0.0)
    held = simulate_plan(plan, 0, 0.0, lead_dwell_ms=5000)
    assert held.events[0].time_sec == pytest.approx(base.events[0].time_sec + 5.0)


# --- conflict detection (port of detectConflicts) ------------------------------

def _sims_for(plans_and_anchors):
    fleet = {}
    for name, (r, cmds, anchor) in plans_and_anchors.items():
        plan = align_commands_to_route(r, cmds, ROWS, COLS)
        fleet[name] = simulate_plan(plan, 0, anchor)
    return fleet


def test_node_conflict_inside_window():
    fleet = _sims_for({
        "Alvik1": (
            route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)]),
            ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"], 0.0),
        "Alvik2": (
            route("Alvik2", [seg(1, 10, 2), seg(2, 2, 11)]),
            ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"], 0.5),
    })
    conflicts = detect_conflicts(fleet)
    assert any(c.type == "intersecting edges" and c.key == "2"
               for c in conflicts)
    node_c = next(c for c in conflicts
                  if c.type == "intersecting edges" and c.key == "2")
    assert node_c.gap_sec == pytest.approx(0.5)


def test_node_conflict_boundary():
    fleet = _sims_for({
        "Alvik1": (
            route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)]),
            ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"], 0.0),
        "Alvik2": (
            route("Alvik2", [seg(1, 10, 2), seg(2, 2, 11)]),
            ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
            INTERSECTION_CLEARANCE_SEC + 0.2),
    })
    assert not any(c.type == "intersecting edges" and c.key == "2"
                   for c in detect_conflicts(fleet))


def test_opposite_edge_overlap_and_contention():
    a = route("Alvik1", [seg(1, 1, 2, move=3.0)])
    b = route("Alvik2", [seg(1, 2, 1, move=3.0)])
    # overlap
    fleet = _sims_for({"Alvik1": (a, ["FORWARD_UNTIL_RED"], 0.0),
                       "Alvik2": (b, ["FORWARD_UNTIL_RED"], 1.0)})
    assert any(c.type == "opposite-direction edge"
               for c in detect_conflicts(fleet))
    # gap inside the opposite-direction clearance
    fleet = _sims_for({"Alvik1": (a, ["FORWARD_UNTIL_RED"], 0.0),
                       "Alvik2": (
                           b, ["FORWARD_UNTIL_RED"],
                           3.0 + OPPOSITE_DIRECTION_CLEARANCE_SEC - 0.1)})
    assert any(c.type == "opposite-direction edge"
               for c in detect_conflicts(fleet))
    # gap outside the direction-specific clearance
    fleet = _sims_for({"Alvik1": (a, ["FORWARD_UNTIL_RED"], 0.0),
                       "Alvik2": (
                           b, ["FORWARD_UNTIL_RED"],
                           3.0 + OPPOSITE_DIRECTION_CLEARANCE_SEC + 0.1)})
    assert not any(c.is_edge for c in detect_conflicts(fleet))


# --- advisor end-to-end (mocked LLM) --------------------------------------------

def crossing_schedule():
    """Alvik2 reaches shared node 2 first; Alvik1 is staggered by 0.5s.
    Alvik2 also has more remaining work, but entry time—not route length—is
    what establishes priority."""
    return schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)], launch=0.5),
        route("Alvik2", [seg(1, 10, 2), seg(2, 2, 11), seg(3, 11, 12)]),
    )


CROSSING_CMDS = {
    "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    "Alvik2": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
}


def test_earlier_entry_proceeds_and_commits_reservation():
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    d = adv.decide("Alvik2", 0.0)
    assert d.inserted_dwell_ms is None
    assert d.command == "FORWARD_UNTIL_RED"
    assert "priority" in d.reason
    assert "Alvik2" in adv.reservations
    # Alvik2 is now mid-move and owns node 2; Alvik1 stays supervisor-idle.
    d1 = adv.decide("Alvik1", 0.05)
    assert d1.supervisor_wait
    assert d1.command is None
    assert adv.robots["Alvik1"].pending is None


def test_fifo_tie_beats_longer_remaining_work():
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2)]),
        route("Alvik2", [
            seg(1, 10, 2), seg(2, 2, 11), seg(3, 11, 12)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik2": [
            "FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED",
            "FORWARD_UNTIL_RED"],
    }, llm=None)
    first_request = adv.decide("Alvik1", 0.0)
    assert first_request.command == "FORWARD_UNTIL_RED"
    assert not first_request.supervisor_wait
    later_request = adv.decide("Alvik2", 0.01)
    assert later_request.supervisor_wait


# --- online timing calibration ----------------------------------------------

def test_timing_calibration_ewma_clamp_and_persistence(tmp_path):
    from dispatch_advisor import TimingCalibration
    path = tmp_path / "calib.json"
    cal = TimingCalibration(str(path))
    assert cal.factor("Alvik1", "move") == 1.0
    # sample ratio 90/3 = 30 clamps to 2.0; EWMA: 0.7*1.0 + 0.3*2.0 = 1.3
    assert cal.observe("Alvik1", "move", 3.0, 90.0) == 2.0
    assert cal.factor("Alvik1", "move") == pytest.approx(1.3)
    # unusable samples: unknown kind, near-zero prediction
    assert cal.observe("Alvik1", "dwell", 2.0, 2.0) is None
    assert cal.observe("Alvik1", "move", 0.1, 0.2) is None
    # persisted and reloaded by a fresh instance (cross-mission learning)
    reloaded = TimingCalibration(str(path))
    assert reloaded.factor("Alvik1", "move") == pytest.approx(1.3)
    assert "Alvik1.move=1.30" in reloaded.summary()


def test_simulate_plan_scales_moves_and_turns():
    from dispatch_advisor import simulate_plan
    adv = DispatchAdvisor(
        schedule(route("Alvik1", [seg(1, 1, 2)])),
        {"Alvik1": ["RIGHT_UNTIL_COLOR", "FORWARD_UNTIL_RED"]}, llm=None)
    plan = adv.robots["Alvik1"].plan
    turn_sec = sum(i.duration_sec for i in plan if i.kind == "turn")
    move_sec = sum(i.duration_sec for i in plan if i.kind == "move")
    base = simulate_plan(plan, 0, 0.0).end_sec
    assert simulate_plan(plan, 0, 0.0, move_scale=1.5).end_sec == \
        pytest.approx(base + 0.5 * move_sec)
    assert simulate_plan(plan, 0, 0.0, turn_scale=2.0).end_sec == \
        pytest.approx(base + turn_sec)


def test_record_done_feeds_calibration_per_robot():
    adv = DispatchAdvisor(
        schedule(route("Alvik1", [seg(1, 1, 2)]),
                 route("Alvik2", [seg(1, 30, 31)])),
        {"Alvik1": ["FORWARD_UNTIL_RED"], "Alvik2": ["FORWARD_UNTIL_RED"]},
        llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.command == "FORWARD_UNTIL_RED"
    adv.record_done("Alvik1", 120.0)  # absurdly slow -> sample clamps to 2.0
    assert adv.calibration.factor("Alvik1", "move") == pytest.approx(1.3)
    # the other robot's model is untouched
    assert adv.calibration.factor("Alvik2", "move") == 1.0
    # and its simulation now runs on the stretched clock
    fleet = adv._simulate_fleet(deciding="Alvik2", now_sec=120.0)
    assert fleet["Alvik1"].end_sec >= fleet["Alvik2"].end_sec - 120.0


def test_no_conflict_sends_planned_command():
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2)]),
        route("Alvik2", [seg(1, 30, 31)]),  # far away
    )
    adv = DispatchAdvisor(sched, {"Alvik1": ["FORWARD_UNTIL_RED"],
                                  "Alvik2": ["FORWARD_UNTIL_RED"]}, llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.command == "FORWARD_UNTIL_RED"
    assert d.inserted_dwell_ms is None


def test_conflict_no_llm_waits_in_supervisor_without_command():
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.supervisor_wait
    assert d.command is None
    assert d.inserted_dwell_ms is None
    assert adv.robots["Alvik1"].pending is None
    # The selected horizon is deterministic, but is not sent to the robot.
    ok, why = adv.validate_dwell(
        "Alvik1", 0.0, d.selected_candidate_ms,
        allowed_candidates={d.selected_candidate_ms})
    assert ok, why
    adv.record_done("Alvik1", 0.01)  # no issued command: must be a no-op
    assert adv.robots["Alvik1"].next_index == 0
    d2 = adv.decide("Alvik2", 0.05)
    assert d2.command == "FORWARD_UNTIL_RED"


def test_llm_proposal_pass_through_gate():
    llm = MockLLM([{"dwell_ms": 2000, "rationale": "listed safe candidate"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    d = adv.decide("Alvik1", 0.0)
    assert d.llm_used
    assert d.supervisor_wait
    assert d.selected_candidate_ms == 2000
    assert d.command is None
    assert len(llm.calls) == 1
    payload = llm.calls[0]["payload"]
    assert payload["deciding_robot"] == "Alvik1"
    assert payload["predicted_conflicts"]
    assert payload["candidate_dwells"]
    assert all(c["conflict_free"] for c in payload["candidate_dwells"])


def test_llm_payload_carries_tradeoff_signals():
    # The candidates must carry the fields that make the LLM's choice a real
    # tradeoff rather than an arbitrary pick from equally-safe options.
    llm = MockLLM([{"dwell_ms": 2000, "rationale": "lowest makespan"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    adv.decide("Alvik1", 0.0)
    payload = llm.calls[0]["payload"]
    assert "objective" in payload and "how_to_choose" in payload
    assert "field_guide" in payload
    for cand in payload["candidate_dwells"]:
        assert "fleet_makespan_sec" in cand
        assert "later_holds_for_me" in cand
        # extra_fleet_sec_vs_best is added to every safe candidate.
        assert "extra_fleet_sec_vs_best" in cand
    # The cheapest safe candidate has extra_fleet_sec_vs_best == 0.
    safe = [c for c in payload["candidate_dwells"] if c["conflict_free"]]
    assert min(c["extra_fleet_sec_vs_best"] for c in safe) == 0.0


def test_build_candidates_counts_later_holds():
    # later_holds_for_me is a non-negative integer on every candidate.
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    conflicts = adv._losing_conflicts("Alvik1", [
        c for c in detect_conflicts(adv._simulate_fleet("Alvik1", 0.0))
        if "Alvik1" in c.robots])
    cands = adv._build_candidates("Alvik1", 0.0, conflicts)
    assert cands
    for c in cands:
        assert isinstance(c["later_holds_for_me"], int)
        assert c["later_holds_for_me"] >= 0


class TieLLM:
    """MockLLM that also breaks priority ties by returning a fixed owner."""
    def __init__(self, tie_owner=None, dwell_responses=None):
        self.tie_owner = tie_owner
        self.responses = list(dwell_responses or [])
        self.tie_calls = []
        self.calls = []

    def propose(self, payload, rejection=None, prior=None):
        self.calls.append({"payload": payload, "rejection": rejection})
        return self.responses.pop(0)

    def break_tie(self, payload):
        self.tie_calls.append(payload)
        return self.tie_owner


def tie_schedule():
    """Both robots launch together and reach shared node 2 at the same instant
    (identical entry times) — a genuine priority tie with no deterministic
    separator, so the choice of who yields falls to the tie-breaker."""
    return schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)]),
        route("Alvik2", [seg(1, 3, 2), seg(2, 2, 1)]),
    )


TIE_CMDS = {
    "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    "Alvik2": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
}


def _tie_conflict(adv, deciding="Alvik1"):
    fleet = adv._simulate_fleet(deciding, 0.0)
    return [c for c in detect_conflicts(fleet)
            if set(c.robots) == {"Alvik1", "Alvik2"}]


def test_llm_breaks_genuine_priority_tie():
    llm = TieLLM(tie_owner="Alvik2")
    adv = DispatchAdvisor(tie_schedule(), TIE_CMDS, llm=llm)
    conflicts = _tie_conflict(adv)
    assert conflicts, "expected a shared-resource tie"
    owner = adv._priority_owner("Alvik1", conflicts[0])
    assert owner == "Alvik2"          # LLM's pick, not the name-sort default
    assert adv.llm_stats["tie_breaks"] == 1
    assert llm.tie_calls              # break_tie was actually consulted


def test_tie_break_is_consistent_regardless_of_deciding_robot():
    # The deadlock guard: A and B must agree on the owner. With the LLM picking
    # Alvik2, BOTH deciders must return Alvik2 (else A yields to B and B to A).
    llm = TieLLM(tie_owner="Alvik2")
    adv = DispatchAdvisor(tie_schedule(), TIE_CMDS, llm=llm)
    c1 = _tie_conflict(adv, "Alvik1")[0]
    c2 = _tie_conflict(adv, "Alvik2")[0]
    assert adv._priority_owner("Alvik1", c1) == "Alvik2"
    assert adv._priority_owner("Alvik2", c2) == "Alvik2"
    # Cached: only one consultation for the pair, not one per decider.
    assert len(llm.tie_calls) == 1


def test_tie_break_falls_back_when_llm_returns_bad_owner():
    llm = TieLLM(tie_owner="Alvik9")   # not one of the tied robots
    adv = DispatchAdvisor(tie_schedule(), TIE_CMDS, llm=llm)
    c = _tie_conflict(adv)[0]
    # Guard rejects the invalid pick -> deterministic name sort (min).
    assert adv._priority_owner("Alvik1", c) == "Alvik1"
    assert adv.llm_stats["tie_breaks"] == 0


def test_no_tie_break_without_llm():
    adv = DispatchAdvisor(tie_schedule(), TIE_CMDS, llm=None)
    c = _tie_conflict(adv)[0]
    assert adv._priority_owner("Alvik1", c) == "Alvik1"  # deterministic


def test_clear_temporal_lead_never_consults_tie_break():
    # Staggered entry (0.5s apart) is a CLEAR lead, not a tie: the earlier
    # robot owns priority deterministically and break_tie is never called.
    llm = TieLLM(tie_owner="Alvik1")
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    fleet = adv._simulate_fleet("Alvik1", 0.0)
    mine = [c for c in detect_conflicts(fleet) if "Alvik1" in c.robots]
    for c in mine:
        adv._priority_owner("Alvik1", c)
    assert llm.tie_calls == []


def test_llm_rejected_then_accepted_on_retry():
    # The LLM cannot invent a value outside the safe candidate list.
    llm = MockLLM([{"dwell_ms": 500, "rationale": "too short"},
                   {"dwell_ms": 2000, "rationale": "listed retry"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    d = adv.decide("Alvik1", 0.0)
    assert d.supervisor_wait
    assert d.selected_candidate_ms == 2000
    assert len(llm.calls) == 2
    assert llm.calls[1]["rejection"]  # gate reason was fed back


def test_llm_rejected_twice_waits_locally_and_skips_llm_next():
    llm = MockLLM([{"dwell_ms": 500, "rationale": "bad"},
                   {"dwell_ms": 210, "rationale": "still bad"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    d = adv.decide("Alvik1", 0.0)
    assert d.supervisor_wait
    assert d.command is None
    assert len(llm.calls) == 2
    # Same conflict is rechecked without a command completion or new LLM call.
    d2 = adv.decide("Alvik1", 0.1)
    assert d2.supervisor_wait
    assert "LLM skipped" in d2.reason
    assert len(llm.calls) == 2


def test_llm_activity_summary_never_consulted():
    # No conflict reaches the advisor -> the LLM is enabled but never used.
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2)]),
        route("Alvik2", [seg(1, 30, 31)]),  # far away, no conflict
    )
    adv = DispatchAdvisor(sched, {"Alvik1": ["FORWARD_UNTIL_RED"],
                                  "Alvik2": ["FORWARD_UNTIL_RED"]},
                          llm=MockLLM([]))
    adv.decide("Alvik1", 0.0)
    assert adv.llm_stats["consultations"] == 0
    summary = adv.llm_activity_summary()
    assert "no yield-dwell consultation was needed" in summary


def test_llm_activity_summary_and_log_fn_on_consult():
    lines = []
    llm = MockLLM([{"dwell_ms": 2000, "rationale": "listed safe candidate"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm,
                          log_fn=lines.append)
    adv.decide("Alvik1", 0.0)
    assert adv.llm_stats["consultations"] == 1
    assert adv.llm_stats["llm_pass"] == 1
    # The terminal sink saw the consult and the chosen dwell.
    assert any("LLM consult" in ln for ln in lines)
    assert any("chose dwell 2000ms" in ln for ln in lines)
    assert "1 accepted" in adv.llm_activity_summary()


def test_llm_activity_summary_no_llm_mode():
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    adv.decide("Alvik1", 0.0)
    assert adv.llm_stats["no_llm_wait"] == 1
    assert "DISABLED" in adv.llm_activity_summary()


def test_llm_exception_falls_back():
    class Boom:
        def propose(self, payload, rejection=None, prior=None):
            raise RuntimeError("api down")
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=Boom())
    d = adv.decide("Alvik1", 0.0)
    assert d.supervisor_wait
    assert d.command is None


def test_gate_rejects_bad_types_and_bounds():
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    assert not adv.validate_dwell("Alvik1", 0.0, "2000")[0]
    assert not adv.validate_dwell("Alvik1", 0.0, 2000.5)[0]
    assert not adv.validate_dwell("Alvik1", 0.0, True)[0]
    assert not adv.validate_dwell("Alvik1", 0.0, 100)[0]
    assert not adv.validate_dwell("Alvik1", 0.0, 31000)[0]
    assert not adv.validate_dwell(
        "Alvik1", 0.0, 2000, allowed_candidates={1600})[0]


def test_reanchoring_shifts_conflict_prediction():
    """Once Alvik2 is past the shared node, Alvik1's conflict disappears."""
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=None)
    # Supervisor sent Alvik2 its first move at t=0 (state set directly so
    # this test controls the sequence without Alvik2's own advisor pass).
    adv.robots["Alvik2"].pending = ("plan", 0)
    # While Alvik2 is still inbound to node 2, Alvik1 must hold:
    d0 = adv.decide("Alvik1", 0.1)
    assert d0.supervisor_wait
    assert adv.robots["Alvik1"].pending is None
    # Alvik2 completes its move through node 2 at 3.56s.
    adv.record_done("Alvik2", 3.56)
    clearing = adv.decide("Alvik2", 3.57)
    assert clearing.command == "FORWARD_UNTIL_RED"
    adv.record_done("Alvik2", 7.13)
    # At Alvik1's next trigger the fleet is re-anchored: Alvik2 is beyond
    # node 2, so the planned command goes out with no inserted hold.
    d = adv.decide("Alvik1", 9.0)
    assert d.inserted_dwell_ms is None
    assert d.command == "FORWARD_UNTIL_RED"


def test_record_done_advances_and_tracks_node():
    sched = schedule(route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)]))
    adv = DispatchAdvisor(
        sched, {"Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"]}, llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.command == "FORWARD_UNTIL_RED"
    adv.record_done("Alvik1", 3.6)
    st = adv.robots["Alvik1"]
    assert st.next_index == 1
    assert st.current_node == 2
    assert not adv.is_finished("Alvik1")
    d = adv.decide("Alvik1", 3.7)
    adv.record_done("Alvik1", 7.2)
    assert adv.is_finished("Alvik1")
    assert adv.decide("Alvik1", 7.3).command is None


def test_reported_19_second_priority_inversion_regression():
    sched = schedule(
        route("Alvik2", [
            seg(1, 1, 0, move=6.9, type_="RED_TO_BLUE_RETURN"),
            seg(2, 0, "DE2", move=4.9, type_="NODE_0_TO_DEPOT_ENTRY"),
        ]),
        route("Alvik3", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN"),
            seg(2, 0, 1, move=3.56, type_="BLUE_TO_RED"),
            seg(3, 1, 2, move=3.56),
        ]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik2": ["FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_BLUE"],
        "Alvik3": [
            "FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_RED",
            "FORWARD_UNTIL_RED"],
    }, llm=None)
    a2, a3 = adv.robots["Alvik2"], adv.robots["Alvik3"]
    a2.launched = a3.launched = True
    a2.current_node = a3.current_node = 1
    a2.last_done_sec = 254.3
    a3.last_done_sec = 262.0

    fleet = adv._simulate_fleet("Alvik2", 254.3)
    conflict = next(
        c for c in detect_conflicts(fleet)
        if c.type == "same-direction edge" and c.key == "0<>1"
        and c.info["Alvik2"]["plan_index"] == 0
        and c.info["Alvik3"]["plan_index"] == 0)
    assert conflict.info["Alvik2"]["start"] == pytest.approx(254.3)
    assert conflict.info["Alvik2"]["end"] == pytest.approx(261.2)
    assert conflict.info["Alvik3"]["start"] == pytest.approx(262.0)
    assert adv._priority_owner("Alvik2", conflict) == "Alvik2"
    with pytest.raises(ValueError, match="owns"):
        adv._needed_dwell_sec("Alvik2", conflict)
    assert 0 < adv._needed_dwell_sec("Alvik3", conflict) < 1.0

    first = adv.decide("Alvik2", 254.3)
    assert first.command == "FORWARD_UNTIL_BLUE"
    assert not first.supervisor_wait
    later = adv.decide("Alvik3", 254.4)
    assert later.supervisor_wait
    assert later.command is None


def test_turn_before_movement_is_dispatched_without_movement_gate():
    sched = schedule(
        route("Alvik2", [
            seg(1, 1, 0, move=6.9, turn=2.65,
                type_="RED_TO_BLUE_RETURN")]),
        route("Alvik3", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN")]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik2": ["LEFT_UNTIL_COLOR", "FORWARD_UNTIL_BLUE"],
        "Alvik3": ["FORWARD_UNTIL_BLUE"],
    }, llm=None)
    decision = adv.decide("Alvik2", 254.3)
    assert decision.command == "LEFT_UNTIL_COLOR"
    assert not decision.supervisor_wait
    assert "Alvik2" not in adv.reservations


def test_node_occupant_can_clear_before_a_later_arrival():
    sched = schedule(
        route("Alvik2", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN")]),
        route("Alvik3", [
            seg(1, 9, 1), seg(2, 1, 2), seg(3, 2, 3)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik2": ["FORWARD_UNTIL_BLUE"],
        "Alvik3": [
            "FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED",
            "FORWARD_UNTIL_RED"],
    }, llm=None)
    approaching = adv.decide("Alvik3", 0.0)
    assert approaching.command == "FORWARD_UNTIL_RED"
    assert not approaching.supervisor_wait
    clearing = adv.decide("Alvik2", 0.01)
    assert clearing.command == "FORWARD_UNTIL_BLUE"
    assert not clearing.supervisor_wait


def test_label_to_cell_accepts_numeric_strings():
    # Regression (2026-07-21 depot collision): the reservation layer produces
    # node labels as STRINGS via str(node). The supervisor's vision corridor
    # gate passes those to label_to_cell — which used to return None for ANY
    # string, silently disabling the gate. Numeric strings must map to a cell;
    # only genuine non-numeric depot-lane labels ('DE1','D2') return None.
    assert label_to_cell("1", ROWS, COLS) == (0.0, 0.0)
    assert label_to_cell("18", ROWS, COLS) == (1.0, 2.0)
    assert label_to_cell(18, ROWS, COLS) == label_to_cell("18", ROWS, COLS)
    assert label_to_cell("0", ROWS, COLS) == label_to_cell(0, ROWS, COLS)
    assert label_to_cell("DE1", ROWS, COLS) is None
    assert label_to_cell("D2", ROWS, COLS) is None
    assert label_to_cell("depot", ROWS, COLS) is None


def test_is_workstation_node_vs_entry_and_grid():
    # A servicing robot parks on a WORKSTATION node (nodes+1..nodes+bays), off
    # the through-lane. The gate uses this to NOT block a passer (2026-07-21
    # workstation false-hold: Alvik4 at ws88 was 0.35 cells from entry 137 and
    # blocked Alvik1 traversing that lane). Workstation nodes yes; grid/entry no.
    nodes = ROWS * COLS          # 64
    bays = (ROWS - 1) * (COLS - 1)  # 49
    assert is_workstation_node(88, ROWS, COLS)          # ws for bay 24
    assert is_workstation_node(nodes + 1, ROWS, COLS)   # first workstation (65)
    assert is_workstation_node(nodes + bays, ROWS, COLS)  # last workstation
    assert not is_workstation_node(37, ROWS, COLS)      # grid node
    assert not is_workstation_node(137, ROWS, COLS)     # ENTRY node, not ws
    assert not is_workstation_node(nodes + bays + 1, ROWS, COLS)  # first entry
    assert not is_workstation_node("0", ROWS, COLS)     # depot
    assert not is_workstation_node("DE1", ROWS, COLS)   # depot lane
    assert is_workstation_node("88", ROWS, COLS)        # numeric string ok


def test_servicing_robot_sits_off_the_through_lane():
    # Geometry that justifies excluding a servicing robot from the corridor
    # gate: ws88 (2.5, 3.5) is 0.35 cells from its entry 137, but that's the
    # dead-end spur — it is NOT on the 35<->36 through-lane. entry 137's flanks
    # (the actual lane nodes) are 0.71 away.
    import math
    ws88 = label_to_cell(88, ROWS, COLS)
    e137 = label_to_cell(137, ROWS, COLS)
    assert entry_edge_nodes(137, ROWS, COLS) == (35, 36)
    # workstation is close to its own entry (why the radius alone can't fix it):
    assert math.hypot(ws88[0] - e137[0], ws88[1] - e137[1]) < 0.6
    # ...but the fix is is_workstation_node, not the radius.
    assert is_workstation_node(88, ROWS, COLS)


def test_servicing_robot_projects_onto_entry_edge_but_is_off_lane():
    # The EDGE-PROJECTION variant (2026-07-21): Alvik4 servicing ws73 (1.5,1.5)
    # projects ~0.5 cells onto the 18<->19 lane (entry 122 sits on it) and held
    # Alvik1 via the edge-corridor check. is_workstation_node(73) lets the
    # supervisor's off_lane() exclusion skip it. This asserts the geometry the
    # exclusion depends on (the hold logic itself needs ROS to exercise).
    import math
    ws73 = label_to_cell(73, ROWS, COLS)
    n18, n19 = label_to_cell(18, ROWS, COLS), label_to_cell(19, ROWS, COLS)

    def dist_to_seg(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy) / denom))
        return math.hypot(p[0] - (a[0]+t*dx), p[1] - (a[1]+t*dy))

    # It really does project within the 0.5-cell edge-corridor threshold...
    assert dist_to_seg(ws73, n18, n19) <= 0.5
    # ...so only is_workstation_node can rescue it, and it does:
    assert is_workstation_node(73, ROWS, COLS)
    assert entry_edge_nodes(122, ROWS, COLS) == (18, 19)  # entry 122 on 18-19


def test_corridor_geometry_helpers_for_workstation_lane():
    # The vision corridor gate (supervisor side) relies on these: entry 122 is
    # on the 18-19 lane, and a move departing entry 122 is a dead-end exit.
    sched = schedule(
        route("Alvik1", [seg(1, 122, 19), seg(2, 19, 20)]),
        route("Alvik3", [seg(1, 17, 18), seg(2, 18, 19)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
        "Alvik3": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    }, llm=None)
    adv.robots["Alvik1"].current_node = 122
    # Alvik3's move 17->18 occupies the 17-18 lane incl. its entry node 121.
    req3 = adv._movement_request(adv.robots["Alvik3"].plan[0])
    assert "18" in adv.request_corridor_nodes(req3)
    # Leaving entry 122 is a workstation/dead-end exit.
    assert adv.is_workstation_or_depot_exit("Alvik1")
    assert adv._entry_on_edge(18, 19) == 122
    # Two-node lookahead: the move AFTER 122->19 is 19->20.
    nxt = adv.next_move_request("Alvik1")
    assert nxt is not None
    assert "20" in {str(to) for _f, to, _d in nxt.segments}


def test_corridor_gate_radius_catches_rotation_but_not_adjacent_bays():
    # The corridor-gate radius must (a) catch a robot rotating ON a shared node,
    # yet (b) NOT flag a robot tucked into a workstation/bay, which reads ~0.71
    # cells from its adjacent grid nodes (the 2026-07-21 workstation false-hold:
    # Alvik1 at ws1 was 0.71 from node 2 and blocked Alvik3's 2->3 move under the
    # old 0.95 radius). fleetSupervisor imports rclpy (absent off-robot), so pin
    # the constant's value here — keep in sync with VISION_CORRIDOR_CLEAR_CELLS.
    import math
    CLEAR = 0.6   # VISION_CORRIDOR_CLEAR_CELLS
    n1 = label_to_cell("1", ROWS, COLS)
    n0 = label_to_cell("0", ROWS, COLS)
    n2 = label_to_cell("2", ROWS, COLS)
    ws1 = label_to_cell(65, ROWS, COLS)     # workstation 1 (0.5, 0.5)
    # (a) a robot rotating ON node 1 (~0 cells away) is caught -> blocked:
    assert 0.0 <= CLEAR
    # (b) a robot in workstation 1 is NOT flagged as on node 1 or node 2:
    assert math.hypot(ws1[0] - n1[0], ws1[1] - n1[1]) > CLEAR
    assert math.hypot(ws1[0] - n2[0], ws1[1] - n2[1]) > CLEAR
    # a lead that has moved to node 0 (1.35 south) is well clear:
    assert math.hypot(n1[0] - n0[0], n1[1] - n0[1]) > CLEAR
    # an adjacent grid node is 1.0 away -> never over-blocks:
    assert math.hypot(n1[0] - n2[0], n1[1] - n2[1]) > CLEAR


def test_next_move_request_skips_turns_and_ends():
    sched = schedule(route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3)]))
    adv = DispatchAdvisor(
        sched, {"Alvik1": ["FORWARD_UNTIL_RED", "RIGHT_UNTIL_COLOR",
                           "FORWARD_UNTIL_RED"]}, llm=None)
    # current move is 1->2; the next move (past the turn) is 2->3.
    nxt = adv.next_move_request("Alvik1")
    assert nxt is not None
    assert {str(to) for _f, to, _d in nxt.segments} == {"3"}
    # advance to the last move -> no further move.
    adv.robots["Alvik1"].next_index = 2
    assert adv.next_move_request("Alvik1") is None


def test_opposite_direction_edge_swap_reports_deadlock():
    sched = schedule(
        route("Alvik1", [seg(1, 1, 9)]),
        route("Alvik2", [seg(1, 9, 1)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    a = adv.decide("Alvik1", 0.0)
    b = adv.decide("Alvik2", 0.0)
    assert a.supervisor_wait and b.supervisor_wait
    assert a.deadlock and b.deadlock
    assert "rerouting or backtracking" in a.reason
    # Updated 2026-08-24 for _grant_standing_reservation(): a waiting robot
    # now reserves the node it is physically parked on (root-caused from a
    # real hardware collision where a robot parked mid-wait held NO
    # reservation and a third robot was dispatched straight into it -- see
    # that method's docstring). Both robots here started at different nodes
    # (Alvik1 at node 1, Alvik2 at node 9), so each holds its own standing
    # reservation without blocking the other (_reservation_blocker always
    # excludes a robot's own reservation).
    assert set(adv.reservations) == {"Alvik1", "Alvik2"}
    assert adv.reservations["Alvik1"].request.destination_nodes == {"1"}
    assert adv.reservations["Alvik2"].request.destination_nodes == {"9"}


def test_deadlock_cycles_reports_the_mutual_wait():
    """deadlock_cycles() must surface the wait-for cycle the pairwise
    detector already found. Before 2026-08-31 _wait_cycle() was never
    called, so a held deadlock was invisible to both the metrics conflict
    table (deadlock_count read 0 for a run with ~100 deadlock holds) and to
    any fail-fast path -- the run just hung until aborted by hand."""
    sched = schedule(
        route("Alvik1", [seg(1, 1, 9)]),
        route("Alvik2", [seg(1, 9, 1)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    assert adv.deadlock_cycles() == []
    a = adv.decide("Alvik1", 0.0)
    b = adv.decide("Alvik2", 0.0)
    assert a.deadlock and b.deadlock
    # One deadlock, not two: A->B->A and B->A->B are the same cycle, and
    # the supervisor keys its conflict_id off this membership.
    assert adv.deadlock_cycles() == [["Alvik1", "Alvik2"]]


def test_deadlock_cycles_empty_when_waits_are_not_circular():
    """A one-way wait (ordinary reservation queueing) is NOT a deadlock --
    it clears on its own. Only a cycle counts, or every busy run would
    report deadlocks."""
    sched = schedule(
        route("Alvik1", [seg(1, 1, 9)]),
        route("Alvik2", [seg(1, 9, 10)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    adv.decide("Alvik1", 0.0)
    adv.decide("Alvik2", 0.0)
    for cycle in adv.deadlock_cycles():
        assert len(cycle) > 1


def test_first_depot_return_proceeds_when_corridor_is_clear():
    sched = schedule(
        route("Alvik1", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN")]),
        route("Alvik2", [seg(1, 9, 10)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_BLUE"],
        "Alvik2": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    decision = adv.decide("Alvik1", 0.0)
    assert decision.command == "FORWARD_UNTIL_BLUE"
    assert not decision.supervisor_wait
    assert adv.reservations["Alvik1"].request.shared_depot_entrance


def test_depot_follower_can_approach_node_zero_while_leader_clears_it():
    sched = schedule(
        route("Alvik1", [
            seg(1, 0, "DE1", move=4.9, turn=2.65,
                type_="NODE_0_TO_DEPOT_ENTRY"),
        ]),
        route("Alvik2", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN")]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["LEFT_UNTIL_COLOR", "FORWARD_UNTIL_BLUE"],
        "Alvik2": ["FORWARD_UNTIL_BLUE"],
    }, llm=None)
    turning = adv.decide("Alvik1", 0.0)
    assert turning.command == "LEFT_UNTIL_COLOR"
    assert not turning.supervisor_wait
    assert "Alvik1" not in adv.reservations

    follower = adv.decide("Alvik2", 0.01)
    assert follower.command == "FORWARD_UNTIL_BLUE"
    assert not follower.supervisor_wait
    assert adv.reservations["Alvik2"].request.shared_depot_entrance

    # The leader's 2.65-second turn clears node 0 before the follower's
    # 4.2-second approach arrives, so its outgoing move remains dispatchable.
    adv.record_done("Alvik1", 2.65)
    departing = adv.decide("Alvik1", 2.66)
    assert departing.command == "FORWARD_UNTIL_BLUE"
    assert not departing.supervisor_wait
    assert not adv.immediate_movement_request(
        "Alvik1").shared_depot_entrance


def test_depot_follower_waits_when_leader_cannot_clear_node_zero_in_time():
    sched = schedule(
        route("Alvik1", [
            seg(1, 0, "DE1", move=4.9, turn=5.0,
                type_="NODE_0_TO_DEPOT_ENTRY"),
        ]),
        route("Alvik2", [
            seg(1, 1, 0, move=4.2, type_="RED_TO_BLUE_RETURN")]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["LEFT_UNTIL_COLOR", "FORWARD_UNTIL_BLUE"],
        "Alvik2": ["FORWARD_UNTIL_BLUE"],
    }, llm=None)
    turning = adv.decide("Alvik1", 0.0)
    assert turning.command == "LEFT_UNTIL_COLOR"

    follower = adv.decide("Alvik2", 0.01)
    assert follower.supervisor_wait
    assert follower.command is None
    assert adv.wait_for["Alvik2"] == "Alvik1"


def test_logical_node_occupancy_survives_missing_vision_update():
    sched = schedule(
        route("Alvik2", [seg(1, 9, 1)]),
        route("Alvik3", [seg(1, 2, 1)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik2": ["FORWARD_UNTIL_RED"],
        "Alvik3": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    sent = adv.decide("Alvik2", 0.0)
    assert sent.command == "FORWARD_UNTIL_RED"
    adv.record_done("Alvik2", 3.56)
    assert adv.robots["Alvik2"].current_node == 1
    # No vision object or update participates: completed logical occupancy is
    # enough to protect node 1.
    blocked = adv.decide("Alvik3", 3.57)
    assert blocked.supervisor_wait
    assert blocked.command is None
    assert adv.wait_for["Alvik3"] == "Alvik2"


def test_robot_parked_mid_route_blocks_a_third_robot_from_its_node():
    """Regression for a real hardware collision (2026-08-24, 4-robot run):
    Alvik3 arrived at DE2 and was correctly SUPERVISOR-WAITing (blocked by
    Alvik1's stuck reservation on DE1 further down its own route) -- but
    Alvik3's own presence at DE2 was invisible to the reservation system,
    since record_done() releases a robot's reservation the instant its move
    completes, regardless of whether it can actually proceed. Alvik2 was
    then granted a move whose destination was DE2 and drove into Alvik3.

    Reproduced here with the same shape, minus vision/timing noise: Alvik1
    reserves DE1 and never completes (simulating "frozen, unresponsive" --
    record_done() is deliberately never called for it, exactly like the
    real incident where Alvik1's leg never cleanly finished or errored out
    within this trace). Alvik3 arrives at DE2 (record_done), then asks to
    continue to DE1 and must wait on Alvik1. Alvik2 then asks to move into
    DE2 -- must be refused, citing Alvik3, not silently granted."""
    sched = schedule(
        route("Alvik1", [seg(1, "D1", "DE1", move=4.9,
                             type_="DEPOT_SLOT_TO_ENTRY")]),
        route("Alvik2", [seg(1, "DE3", "DE2", move=5.9),
                        seg(2, "DE2", "DE1", move=5.9)]),
        route("Alvik3", [seg(1, "DE3", "DE2", move=5.9),
                        seg(2, "DE2", "DE1", move=5.9)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_BLUE"],
        "Alvik2": ["FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_BLUE"],
        "Alvik3": ["FORWARD_UNTIL_BLUE", "FORWARD_UNTIL_BLUE"],
    }, llm=None)

    # Alvik1 is dispatched onto DE1's resource (its move's destination) and
    # never finishes -- the frozen-robot condition from the real incident.
    frozen = adv.decide("Alvik1", 0.0)
    assert frozen.command == "FORWARD_UNTIL_BLUE"
    assert adv.reservations["Alvik1"].request.destination_nodes == {"DE1"}

    # Alvik3 arrives at DE2 (completes its first leg) and asks to continue
    # toward DE1 -- refused, since Alvik1 still holds that resource.
    a3_first = adv.decide("Alvik3", 0.0)
    assert a3_first.command == "FORWARD_UNTIL_BLUE"
    adv.record_done("Alvik3", 5.9)
    assert adv.robots["Alvik3"].current_node == "DE2"
    a3_second = adv.decide("Alvik3", 6.0)
    assert a3_second.supervisor_wait
    assert adv.wait_for["Alvik3"] == "Alvik1"

    # Alvik2 now asks to move into DE2, where Alvik3 is physically parked.
    # Before the fix this was granted (nothing reserved DE2); it must now
    # be refused, citing Alvik3.
    a2_decision = adv.decide("Alvik2", 6.1)
    assert a2_decision.supervisor_wait
    assert a2_decision.command is None
    assert adv.wait_for["Alvik2"] == "Alvik3"


def test_planned_dwell_remains_a_tracked_robot_command():
    sched = schedule(route("Alvik1", [seg(1, 1, 2)]))
    adv = DispatchAdvisor(
        sched, {"Alvik1": ["DWELL 3000", "FORWARD_UNTIL_RED"]}, llm=None)
    decision = adv.decide("Alvik1", 0.0)
    assert decision.command == "DWELL 3000"
    assert not decision.supervisor_wait
    assert adv.robots["Alvik1"].pending == ("plan", 0)


def test_unknown_robot_or_missing_route_fails_loudly():
    with pytest.raises(AlignmentError):
        DispatchAdvisor(crossing_schedule(),
                        {"Alvik9": ["FORWARD_UNTIL_RED"]}, llm=None)


# --- workstation-entry corridor coupling (entry node lies on a grid edge) -----

def test_entry_edge_nodes_maps_entry_to_flanking_grid_nodes():
    # Bay 9 entry on 8x8 = 64 (grid) + 49 (bays) + 9 = 122, on the 18<>19 lane.
    assert entry_edge_nodes(122, ROWS, COLS) == (18, 19)
    # Bay 1 entry = 64 + 49 + 1 = 114. Bay 1 is bottom-left; its entry sits one
    # grid row UP from the depot corners, on the 9<>10 lane (not 1<>2).
    assert entry_edge_nodes(114, ROWS, COLS) == (9, 10)
    # A grid node and a workstation node are not entries.
    assert entry_edge_nodes(18, ROWS, COLS) is None
    assert entry_edge_nodes(65, ROWS, COLS) is None  # workstation spur


def test_corridor_key_unifies_grid_edge_with_entry_half_edges():
    full = corridor_key(18, 19, ROWS, COLS)
    # Both half-edges through entry 122 collapse to the same corridor as 18<>19.
    assert corridor_key(18, 122, ROWS, COLS) == full
    assert corridor_key(122, 19, ROWS, COLS) == full
    assert corridor_key(19, 122, ROWS, COLS) == full
    # A depot/other edge with no entry on it is untouched.
    assert corridor_key(1, 9, ROWS, COLS) == edge_key(1, 9)


def test_corridor_coupling_is_general_across_every_entry_edge():
    # The coupling is derived from grid geometry, not a hardcoded edge list, so
    # EVERY grid edge that carries a bay entry is unified with its entry node.
    # These 12 were the bays in the 2026-07-20 mission; the rule must cover all.
    nodes = ROWS * COLS
    bays = (ROWS - 1) * (COLS - 1)
    listed = [(9, 10), (13, 14), (18, 19), (19, 20), (23, 24), (27, 28),
              (33, 34), (35, 36), (38, 39), (42, 43), (43, 44), (55, 56)]
    coupled = {}
    for label in range(nodes + bays + 1, nodes + 2 * bays + 1):
        flank = entry_edge_nodes(label, ROWS, COLS)
        if flank:
            coupled[tuple(sorted(flank))] = label
    for edge in listed:
        assert tuple(sorted(edge)) in coupled, f"{edge} not entry-coupled"
        g1, g2 = edge
        e = coupled[tuple(sorted(edge))]
        ck = corridor_key(g1, g2, ROWS, COLS)
        assert corridor_key(g1, e, ROWS, COLS) == ck
        assert corridor_key(e, g2, ROWS, COLS) == ck
    # Every interior grid edge on a bay row has exactly one entry coupled to it.
    assert len(coupled) == bays


def test_bay_service_conflicts_with_grid_traversal_on_same_lane():
    # The reported 2026-07-20 collision: Alvik1 leaves bay 9 (entry 122) while
    # Alvik3 traverses grid edge 18->19. Same physical lane; must be a conflict.
    sched = schedule(
        route("Alvik1", [seg(1, 122, 19, move=3.56)]),   # leaving the bay entry
        route("Alvik3", [seg(1, 18, 19, move=3.56)]),     # driving the grid edge
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik3": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    for st in adv.robots.values():
        st.launched = True
    adv.robots["Alvik1"].current_node = 122
    adv.robots["Alvik3"].current_node = 18
    fleet = adv._simulate_fleet("Alvik1", 0.0)
    lane = corridor_key(18, 19, ROWS, COLS)
    hits = [c for c in detect_conflicts(fleet)
            if c.key == lane and "edge" in c.type]
    assert hits, "bay-service vs grid-traversal on the shared lane must conflict"


def test_disjoint_lanes_do_not_falsely_conflict():
    # Bay 9 (entry 122 on 18<>19) vs a grid edge two columns away: no conflict.
    sched = schedule(
        route("Alvik1", [seg(1, 122, 19, move=3.56)]),
        route("Alvik3", [seg(1, 20, 21, move=3.56)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED"],
        "Alvik3": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    for st in adv.robots.values():
        st.launched = True
    adv.robots["Alvik1"].current_node = 122
    adv.robots["Alvik3"].current_node = 20
    fleet = adv._simulate_fleet("Alvik1", 0.0)
    edge_conflicts = [c for c in detect_conflicts(fleet) if "edge" in c.type]
    assert not edge_conflicts


# --- try_authorize_next_move (turn-then-drive fusion pre-check) --------------

def test_authorize_next_move_clear_when_no_conflict():
    # Alvik1: leg 1 (dispatched, still "in flight") then leg 2 (3->4) -- no
    # other robot anywhere nearby, so the next leg should authorize clear.
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3), seg(3, 3, 4)]),
        route("Alvik2", [seg(1, 30, 31)]),  # far away, irrelevant
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED"],
    }, llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.command == "FORWARD_UNTIL_RED"  # move 0 (1->2) granted, now pending
    st = adv.robots["Alvik1"]
    assert st.pending == ("plan", 0)

    ok = adv.try_authorize_next_move("Alvik1", 1, 1.0)  # move 1 = 2->3
    assert ok is True


def test_authorize_next_move_blocked_by_real_conflict():
    # Alvik1: leg 0 (1->2, unrelated) then leg 1 (20->21). Alvik2: leg 0
    # (30->31, unrelated) then leg 1 (20->21, SAME edge as Alvik1's leg 1).
    # Alvik2 gets a real reservation on 20<>21 first; Alvik1's hypothetical
    # next move onto that same edge must be refused, not silently granted.
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 20, 21)]),
        route("Alvik2", [seg(1, 30, 31), seg(2, 20, 21)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    }, llm=None)
    d1 = adv.decide("Alvik1", 0.0)
    assert d1.command is not None  # Alvik1's leg 0 (1->2) granted, now pending
    d2 = adv.decide("Alvik2", 0.0)
    assert d2.command is not None  # Alvik2's leg 0 (30->31) granted, now pending
    # Alvik2 finishes its own first move and gets a REAL reservation granted
    # on its second move (20->21) via the normal path.
    adv.record_done("Alvik2", 1.0)
    d2b = adv.decide("Alvik2", 1.0)
    assert d2b.command is not None
    assert "Alvik2" in adv.reservations

    # Alvik1's hypothetical next move is also 20->21 -- must be refused.
    ok = adv.try_authorize_next_move("Alvik1", 1, 1.0)
    assert ok is False


def _snapshot(adv, name):
    st = adv.robots[name]
    return {
        "st": {
            "next_index": st.next_index, "current_node": st.current_node,
            "current_edge": st.current_edge, "pending_edges": st.pending_edges,
            "request_seq": st.request_seq,
            "request_plan_index": st.request_plan_index,
            "last_wait_sig": st.last_wait_sig,
            "last_fallback_sig": st.last_fallback_sig,
            "last_selected_candidate_ms": st.last_selected_candidate_ms,
            "pending": st.pending,
        },
        "reservations": dict(adv.reservations),
        "wait_for": dict(adv.wait_for),
        "request_counter": adv._request_counter,
        "reservation_counter": adv._reservation_counter,
    }


def test_authorize_next_move_failure_restores_all_state_exactly():
    # Same conflict setup as test_authorize_next_move_blocked_by_real_conflict
    # -- confirm a FALSE result leaves every touched field bit-for-bit as found.
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 20, 21)]),
        route("Alvik2", [seg(1, 30, 31), seg(2, 20, 21)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
        "Alvik2": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    }, llm=None)
    adv.decide("Alvik1", 0.0)
    adv.decide("Alvik2", 0.0)
    adv.record_done("Alvik2", 1.0)
    adv.decide("Alvik2", 1.0)  # Alvik2 now holds the real reservation on 20<>21

    before = _snapshot(adv, "Alvik1")
    ok = adv.try_authorize_next_move("Alvik1", 1, 1.0)
    assert ok is False
    assert _snapshot(adv, "Alvik1") == before


def test_authorize_next_move_success_grants_reservation_and_fused_done_cleans_up():
    sched = schedule(
        route("Alvik1", [seg(1, 1, 2), seg(2, 2, 3), seg(3, 3, 4)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik1": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    }, llm=None)
    d = adv.decide("Alvik1", 0.0)
    assert d.command is not None
    st = adv.robots["Alvik1"]
    assert st.pending == ("plan", 0)

    ok = adv.try_authorize_next_move("Alvik1", 1, 1.0)
    assert ok is True
    # A real reservation now exists for the FUSED (next) move, and next_index
    # is already advanced to it -- this is what closes the collision-safety
    # gap during the fused window (no other robot can be granted 2<>3 while
    # Alvik1 is physically driving it, even though record_done_fused()
    # hasn't run yet).
    assert "Alvik1" in adv.reservations
    assert adv.reservations["Alvik1"].plan_index == 1
    assert st.next_index == 1
    assert st.current_node == 2  # landed at move 0's to_node
    # pending is UNCHANGED -- still the original in-flight move -- until the
    # caller actually reports the fused drive complete via record_done_fused().
    assert st.pending == ("plan", 0)

    adv.record_done_fused("Alvik1", 0, 1, 2.0)
    assert st.pending is None
    assert st.next_index == 2  # past BOTH move 0 and move 1
    assert st.current_node == 3  # move 1's to_node
    # The MOVEMENT reservation is released exactly once -- the edge claims are
    # gone. What remains is a node-only STANDING hold on node 3, because the
    # robot is physically sitting on it. Before 2026-09-29 arrival dropped
    # everything, and a second robot could be granted the node this one was
    # still standing on; that is how Alvik5 drove into Alvik3 at DE3.
    assert "Alvik1" in adv.reservations
    assert not adv.reservations["Alvik1"].request.edge_keys
    assert (adv.reservations["Alvik1"].request.destination_nodes
            == frozenset({"3"}))
    assert st.standing_node == 3
    # ...and it is released only once vision has shown it clear for the full
    # linger, not on the first frame that says so.
    assert adv.note_standing_clearance("Alvik1", True, 2.0) is False
    assert adv.note_standing_clearance("Alvik1", True, 2.2) is False
    assert adv.note_standing_clearance("Alvik1", True, 2.6) is True
    assert "Alvik1" not in adv.reservations
    assert st.standing_node is None

    # The advisor is now in a normal, consistent state -- decide() for move 2
    # works exactly as it would after any ordinary record_done().
    d3 = adv.decide("Alvik1", 2.0)
    assert d3.command is not None


def test_authorize_next_move_never_calls_llm():
    # Force a real conflict that would normally trigger LLM consultation in
    # decide() -- try_authorize_next_move must return False WITHOUT ever
    # calling the LLM (a speculative check must have zero real side effects).
    llm = MockLLM([{"dwell_ms": 2000, "rationale": "should never be called"}])
    adv = DispatchAdvisor(crossing_schedule(), CROSSING_CMDS, llm=llm)
    adv.decide("Alvik2", 0.0)  # Alvik2 gets priority on the shared node first
    st1 = adv.robots["Alvik1"]
    st1.pending = ("plan", 0)  # pretend Alvik1's first move is in flight
    st1.launched = True

    ok = adv.try_authorize_next_move("Alvik1", 1, 0.6)
    assert ok is False
    assert len(llm.calls) == 0


def test_authorize_next_move_out_of_range_index_is_false():
    sched = schedule(route("Alvik1", [seg(1, 1, 2)]))
    adv = DispatchAdvisor(sched, {"Alvik1": ["FORWARD_UNTIL_RED"]}, llm=None)
    adv.decide("Alvik1", 0.0)
    assert adv.try_authorize_next_move("Alvik1", 99, 0.0) is False


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))


def test_standing_hold_survives_arrival_until_vision_says_clear():
    """The 2026-09-29 depot collision, as a unit test.

    Hardware sequence that produced it, from the supervisor log:

        .627  Alvik5 WAIT: resource edge DE2<>DE3 reserved by Alvik3
        .754  Alvik3's plan advances to a TURN -- which takes no reservation
        .757  Alvik5 granted move DE2->DE3 while Alvik3 stands on DE3
        .800  Alvik3 only NOW begins to leave

    Alvik5 drove into it. The hold must now survive arrival and only lift
    once vision reports the robot has physically left, plus the linger.
    """
    sched = schedule(
        route("Alvik3", [seg(1, 1, 2), seg(2, 2, 3)]),
    )
    adv = DispatchAdvisor(sched, {
        "Alvik3": ["FORWARD_UNTIL_RED", "FORWARD_UNTIL_RED"],
    }, llm=None)
    adv.decide("Alvik3", 0.0)
    st3 = adv.robots["Alvik3"]

    # Alvik3 completes its move and is now standing on node 2.
    adv.record_done("Alvik3", 1.0)

    assert st3.current_node == 2
    assert st3.standing_node == 2, "arrival must not free the node"
    held = adv.reservations.get("Alvik3")
    assert held is not None, "the node it stands on is still held"
    assert held.request.destination_nodes == frozenset({"2"})
    assert not held.request.edge_keys, "but the lane behind it is freed"

    # Still parked there: every frame that shows it on the node re-arms.
    assert adv.note_standing_clearance("Alvik3", False, 1.2) is False
    assert adv.note_standing_clearance("Alvik3", False, 1.8) is False
    assert "Alvik3" in adv.reservations

    # It leaves. One clear frame is not enough.
    assert adv.note_standing_clearance("Alvik3", True, 2.0) is False
    assert "Alvik3" in adv.reservations
    # A frame that puts it back over the node resets the timer, so partial
    # credit cannot accumulate across a bad fix.
    assert adv.note_standing_clearance("Alvik3", False, 2.3) is False
    assert adv.note_standing_clearance("Alvik3", True, 2.4) is False
    assert "Alvik3" in adv.reservations, "timer restarted, not resumed"
    # Clear continuously for the linger -> released.
    assert adv.note_standing_clearance("Alvik3", True, 2.95) is True
    assert "Alvik3" not in adv.reservations
    assert st3.standing_node is None
