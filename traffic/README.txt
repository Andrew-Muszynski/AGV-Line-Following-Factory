TRAFFIC PLANNING FOR THE DOE
traffic/ - built 2026-10-17


WHAT IT IS
  Four traffic-planning methods on one model of the testbed, judged by one
  validator. The job plan (which robot visits which bays, in what order) is
  frozen; only the traffic planner changes.

    astar   Uncoordinated A*: each robot's fastest route alone, no waits.
            Conflicts are left to the supervisor's runtime layer.
    wait    Iterative delay insertion (the incumbent, ported from the page).
    pp      Prioritized planning, longest route first, restarts on failure.
    cbs     Conflict-Based Search, makespan first, budget-limited.


RUN THE SIMULATION TIER   (Windows, PowerShell, in C:\Users\muszy\VRP)
  python3 -m traffic.run_doe solved_plans\plan_brkga_ga_20261016_135559.json

  Options:
    --methods astar,wait,pp,cbs   which methods (default: all four)
    --time-limit 30               planning budget per scenario and method, s
    --out traffic_results.csv     results, one row per scenario x method
    --plans traffic_plans         dispatchable plan files ('' = none)
    --check-page FILE.json        judge a page export (incumbent) the same way

  Scenario files: any saved plan - plan_server's solved_plans\*.json or the
  page's own exports (generated_plans\, runs\). Rows are APPENDED to the CSV.
  The CSV also has variant/experiment/dataset/instance/makespan columns, so
  external\vrp_rpd\statistical_analysis.py reads it as it stands.


RUN A PLAN ON THE ROBOTS   (hardware tier)
  1. Run the tier above; plans land in traffic_plans\<scenario>__<method>.json
  2. In the page (Run & Monitor), Import plan -> pick that file. It is taken
     exactly as planned: no re-routing, no WAIT polish.
  3. Generate commands, Send mission, Start - as usual.
  Do not press Solve after importing: that rebuilds the routes.
  Only the Flask page (app.py, :8000) has this import, not the standalone
  agv_grid_workstation_solver.html.


THE MODEL (what all four share)
  Graph, move times and turn times are the page's own (static/js/solver.js);
  verified to 0.001 s against a page export. Launch: the rolling cascade,
  ported exactly and fixed for every method. Conflicts: a robot holds its
  node from arrival until it drives away (plus 1.0 s each side), holds a
  lane while driving it, and a robot on a bay entry also holds that lane.
  A turn at an entry (or beside one) while another robot is on a squeeze
  partner is a conflict - the runtime rotation gate's rule. No about-face
  mid-grid. Pick-ups wait for drop + processing time.


FILES
  model.py      testbed graph, timings, lanes, squeeze partners
  plan.py       timed paths (nodes + waits) and the timing rule
  instance.py   scenario loading, service merge, launch cascade, readiness
  conflicts.py  the conflict model and planning constraints
  sipp.py       single-robot planner (safe-interval search with heading)
  planners.py   astar, wait, pp, cbs
  validate.py   validator and per-plan metrics
  export.py     page-format plans in and out
  run_doe.py    the runner
  test_traffic.py   python3 -m pytest -q traffic\test_traffic.py


KNOWN LIMITS
  - WAIT is incomplete: delays alone cannot separate some routes (a waiting
    robot is an obstacle). It then keeps its best plan and reports the
    conflicts left, like A*. The page's WAIT never sees these because it
    has no turn-at-entry rule.
  - CBS with interval constraints is not complete in continuous time and can
    run out of budget on large scenarios (e.g. generated_plans
    nearest_2026-08-12, 84 A* conflicts). A timeout is a result, not a bug.
  - PP is incomplete by construction; restarts with shuffled orders.
  - Plans are open-loop: on the floor the supervisor still gates every move.
