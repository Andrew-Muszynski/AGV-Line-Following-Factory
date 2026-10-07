# LLM Dispatch Advisor — Design

Status: IMPLEMENTED 2026-07-02; HARDWARE-RUN 2026-07-15 (first 4-robot
mission: gate + fallback-hold behavior worked live). BACKEND SWAPPED
2026-07-15: the advisor now calls a local Ollama model (`ollama_advisor.py`,
default `llama3.2:3b`, schema-constrained JSON, no API account) —
`haiku_advisor.py` (Anthropic API, bills separately from a Claude
subscription) is retired but kept for reference. The design below was the
input; the build is:

- `dispatch_advisor.py` — deterministic core: loads the solver HTML's JSON
  export as ground truth, aligns each robot's command list to its route at
  startup (fails loudly on mismatch), re-anchors remaining schedules to live
  completion times, ports `detectConflicts()` (5.0s node / 7.0s edge), and
  gates every dwell value deterministically before it can be emitted.
- `haiku_advisor.py` — Claude Haiku 4.5 client, structured output only
  (`output_config.format`), schema `{dwell_ms, rationale}`.
- `Juan_Fleet_Supervisor.py --schedule plan.json` — advisor mode: autonomous,
  per-robot advancement (each robot re-evaluated the moment IT finishes a
  command), staggered launch offsets honored, JSONL audit log. Lockstep mode
  unchanged and still the default without `--schedule`.
- `test_dispatch_advisor.py` — 23 unit tests (alignment, timing, conflict
  port, re-anchoring, gate, fallback, mocked LLM), all passing.

Gate-rejection fallback (decided this session): send the plain `DWELL 2000`
and re-evaluate when the hold ends — repeat holds on the same conflict skip
the LLM call (cost guard). Rerouting via alternate command sequences remains
an open question (see below), deliberately not built.

## What exists today (built this session)

- `Juan.ino` — reactive one-command-at-a-time executor. No grid/node model.
  - `<ROBOT_NAME>_pose` topic: `{"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis}`,
    published every 200ms from `alvik.get_pose()` / `alvik.get_battery_charge()`.
  - `DWELL` now accepts an optional override: `DWELL <ms>` (clamped to
    `[MIN_DWELL_MS=200, MAX_DWELL_MS=30000]`), falling back to the fixed
    `WORKSTATION_WAIT_MS=2000` when no argument is given. This is the only
    lever a supervisor has today to change a robot's timing at a stop.
  - Axis note for pose→grid conversion: `reset_pose(0,0,0)` happens at setup,
    while the robot faces **south** (before the depot-exit maneuver). So
    heading north into the grid afterward is **-x**, not +x, in that fixed
    odometry frame. `alvikPoseToGridPoint()` in `agv_grid_workstation_solver.html`
    already applies this.
- `Juan_Fleet_Supervisor.py` — one process, holds a flat command list per
  robot, advances the whole fleet one command per step, gated on a terminal
  prompt (Enter / `auto` / `quit`). No collision awareness yet — that's what
  this design adds.
- `agv_grid_workstation_solver.html` — has the actual planning model Juan.ino
  lacks: an 8×8 grid, `route.path` (node/entry/workstation/special points),
  per-segment timing (`MOVEMENT_SEC`, `TURN_SEC`, `DEPOT_ENTRY_CLEARANCE_SEC`),
  and working collision detection (`detectConflicts()`) with two windows:
  `SAFETY_WINDOW_SEC=5.0` for node conflicts, `EDGE_SAFETY_WINDOW_SEC=7.0` for
  edge conflicts. This is the schedule referred to below as "the precomputed
  schedule" — it's the ground truth, not something re-derived from scratch.

## The gap

Juan.ino itself has no grid concept — all routing intelligence has to live
off-robot. `Juan_Fleet_Supervisor.py` currently just replays a flat command
list per robot with no cross-robot awareness beyond the manual step gate.
There is no live link between "the precomputed schedule" (solver HTML) and
"what the physical robots are actually doing right now."

## What the user wants (their words, lightly organized)

1. Treat the solver's precomputed schedule as ground truth / starting point —
   don't recompute a plan from nothing on the robot side.
2. Re-derive the future path **at each update from the robot** — i.e. every
   time a robot finishes a command and reports back to the supervisor
   (BUSY→done→ready-for-next), that's the trigger point to re-evaluate, not a
   fixed timer.
3. At each such update, simulate forward in time (time-driven simulation, same
   rough magnitude as the HTML's playback) to predict whether continuing on
   the current plan risks a node/edge collision with another robot.
4. If a risk is found, compare **makespan** (single-robot and fleet-wide)
   across alternatives — e.g. take a right turn instead of driving straight
   to avoid another robot's path, vs. holding at a workstation for a few
   seconds and letting the other robot pass. Pick whichever is faster /
   avoids the collision.
5. The only concrete mechanism confirmed as buildable *today* on Juan.ino is
   variable DWELL duration (see above) — turn/route substitution would need
   pre-authored alternate command sequences per robot, since Juan.ino has no
   pathfinding of its own (see "Reroute meaning" decision earlier in the
   session: holds only, unless alternates are explicitly authored).
6. Eventually: fuse in AprilTag-based computer vision for real robot
   pose/trajectory (tag on each robot, 4 corner tags for the table frame),
   as a replacement or supplement for the odometry-based `_pose` topic —
   explicitly because odometry drifts and "other attempts have fallen short
   of simulating the exact movement the robot takes."
7. The actual collision *decision* should be proposed by an LLM (Claude Haiku
   4.5 — cheapest tier, chosen explicitly for cost, event-driven so it's cheap
   in practice) but a **deterministic check must gate anything sent to a
   robot** — this was a hard requirement the user confirmed explicitly earlier
   in the session (no LLM output reaches `_cmd` unchecked).

## Proposed architecture (for next session to refine, not final)

```
                    ┌─────────────────────────────┐
                    │  agv_grid_workstation_solver  │
                    │  .html — precomputed schedule │
                    │  (exported JSON: path_nodes,  │
                    │   segments, node_events, ...) │
                    └───────────────┬───────────────┘
                                    │ (export once per plan)
                                    ▼
                    ┌─────────────────────────────┐
                    │  Juan_Fleet_Supervisor.py    │
                    │  (existing: per-robot cmd    │
                    │   queue, step gate)          │
                    └───────────────┬───────────────┘
                     robot reports done            │ next command to send
                                    ▼               │
                    ┌─────────────────────────────┐│
                    │  Dispatch Advisor (NEW)      ││
                    │  - re-simulate remaining      ││
                    │    schedule from live pose    ││
                    │  - deterministic conflict     ││
                    │    check (ported from         ││
                    │    detectConflicts())         ││
                    │  - IF risk: ask Haiku for a   ││
                    │    DWELL-duration proposal    ││
                    │  - re-validate proposal        ││
                    │    deterministically before   ││
                    │    ever emitting it            ││
                    └───────────────┬───────────────┘│
                                    └─────────────────┘
                                    ▼
                          Alvik#_cmd (only after
                          deterministic pass/fail)
```

## Open questions for next session

- How to represent "the remaining schedule" for a robot once it's live and
  drifting from the plan — re-anchor the plan's remaining segments to the
  robot's actual current position/time, or treat the plan as advisory only
  and re-derive purely from live pose deltas?
- Where does makespan comparison actually run — in Python (fast, easy to
  test) or does it need to reuse the HTML's JS functions (`movementProfile`,
  `applyRouteTiming`) directly to avoid a second implementation drifting from
  the first? (Likely: port the relevant pure functions to Python once, single
  source of truth, since they're already pure/timing-table-driven.)
- Turn/route substitution: needs a decision on where "alternate command
  sequences" get authored — hand-authored per workstation pair, or generated
  by extending `route_planner.py`'s token model (which belongs to the
  *other*, unrelated `AGV_MULTI_WS_DISPATCH.ino` sketch) into something
  Juan.ino-compatible.
- AprilTag CV fusion timing — build after or alongside the re-plan loop?
  Camera calibration (focal length/optical center) is still unmeasured
  (see `apriltag_detect.py`, currently pixel-only, no metric pose yet).
- Confirm the Haiku prompt/response schema (structured output via
  `output_config.format`) once the simulation/makespan model above is
  settled — the LLM's job is still "propose a DWELL duration," but the
  *comparison it's reasoning over* (makespan A vs B) needs the simulator
  first.

## Explicitly deferred from this session (do not re-litigate without reason)

- Any LLM output reaching `Alvik#_cmd` without passing a deterministic
  collision/makespan check first — user confirmed this requirement twice.
- Reroute meaning is holds (DWELL duration) only, unless/until alternate
  command sequences are explicitly authored — no automatic pathfinding on
  Juan.ino, because it has no grid model.
- Model choice: Claude Haiku 4.5, for cost reasons, explicitly chosen after
  discussing local/free alternatives.
