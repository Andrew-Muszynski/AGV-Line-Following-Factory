# Archive — superseded AGV code

Everything in this directory predates the current stack (`Juan.ino` +
`Juan_Fleet_Supervisor.py` + `agv_grid_workstation_solver.html`, described in
the repo root [README.md](../README.md)). None of it is compatible with the
current command vocabulary or topic names — don't mix them.

| Directory | What it was |
|---|---|
| `BaseAGV_v4/` | Original factory-node protocol (`/agv_factory/command`, `agv_factory_node.py`) |
| `hardware_gui/` | Flask+SocketIO GUI wired to the `BaseAGV_v4` protocol |
| `AGV_MULTI_WS_DISPATCH/` | A later sketch + `dispatch_node.py`/`route_planner.py`, using a *different* token vocabulary (`RED YENTRY YWORK DOCK EXIT BLUE R L YAW0 CLEAR`) tied to a grid/workstation model — not the same as `Juan.ino`'s command set |
| `ORACLE_VM/` | Near-duplicate of `AGV_MULTI_WS_DISPATCH`'s dispatch node, run from a separate VM checkout |
| `AGV_CELL_ROS_TEST/`, `AGV_LINE_FOLLOWING_CELL/`, `AGV_LINE_FOLLOWING_PERIMETER/`, `AGV_LINE_FOLLOWING_TEST/`, `AGV_MULTI_WS_ROS/`, `Line_Following_WORKSTATION_ROUTER_WS012_v2/`, `Line_Follow_Simple/`, `PD_TUNING/`, `color_calibration/` | Earlier line-following / PID-tuning / ROS-wiring experiments |
| `ros_dispatch.py` (top-level file) | Bridge script targeting `/agv_factory/command` (the `BaseAGV_v4` protocol) |

If you need something from here, check it still makes sense against
`Juan.ino`'s actual command set before reusing it — several of these use
incompatible topic names or token vocabularies.
