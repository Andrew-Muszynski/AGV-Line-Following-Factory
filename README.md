# AGV Testbed — Setup and Architecture Guide

This system runs a fleet of Arduino Alvik AGVs on an 8×8 grid using ROS 2 and
micro-ROS over WiFi. Each Alvik runs `AGV_Factory_color_pose.ino` — a reactive
one-command-at-a-time executor with no onboard grid knowledge. Routing and
collision planning live off-robot in the grid solver.

> Older sketches and dispatch scripts (`AGV_MULTI_WS_DISPATCH`, `BaseAGV_v4`,
> line-following experiments, etc.) have been moved to [`archive/`](archive/)
> — they predate `AGV_Factory_color_pose.ino` and use a different, incompatible command
> vocabulary and topic protocol. Don't mix them with the current stack below.

---

## Current stack

```
agv_grid_workstation_solver.html   ← grid model, timing constants, collision
                                      detection (detectConflicts), route/
                                      schedule planning + JSON export, live
                                      pose visualization via rosbridge

AGV_Factory_color_pose/
  └── AGV_Factory_color_pose.ino   ← Arduino sketch flashed to every Alvik

apriltag_detect.py                 ← webcam AprilTag ID + pixel-location
                                      viewer (tag36h11); pixels only

apriltag_localize.py               ← metric (inches) robot localization from
                                      the table-corner tags 20-23 via a
                                      plane-to-plane homography; works with
                                      any >=2 corner tags visible
test_apriltag_localize.py          ← synthetic-camera tests for the fitter

solver.py, instance.py, schedule.py,
cli.py, gui.py, simulator.py,
visualizer.py, main.py             ← VRP-RPD solver core (protocol-agnostic,
                                      unaffected by which sketch is live)
```

The sketch is in
[`AGV_Factory_color_pose/`](AGV_Factory_color_pose/). Arduino requires the
`.ino` file to remain in a same-named folder.

---

## AGV_Factory_color_pose.ino — command protocol

Reactive executor: accepts **one atomic command at a time** on `<ROBOT_NAME>_cmd`
and reports back on `<ROBOT_NAME>_status` when done. It has no concept of a
route, a grid, or a workstation — the PC side (a supervisor script) holds the
plan and sends the next command only after the current one finishes.

### Topics (per robot, name picked automatically from WiFi MAC — see `getAlvikID()`)

| Topic | Direction | Format |
|---|---|---|
| `<ROBOT_NAME>_cmd` | subscribe | plain string command |
| `<ROBOT_NAME>_status` | publish, ~1/sec when idle, on every state change | plain string, e.g. `BUSY FORWARD_UNTIL_RED`, `IDLE`, `DETECTED RED ...` |
| `<ROBOT_NAME>_pose` | publish, every 200ms | JSON: `{"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis}` |

### Commands

| Command | Effect |
|---|---|
| `FORWARD_UNTIL_RED` / `FORWARD_UNTIL_COLOR` / `FORWARD_UNTIL_YELLOW` / `FORWARD_UNTIL_BLUE` | Drive forward on the line until the named marker color is detected |
| `BACKWARD_UNTIL_COLOR` / `BACKWARD_UNTIL_YELLOW` / `BACKWARD_UNTIL_BLUE` | Same, reversing |
| `RIGHT_UNTIL_COLOR` / `LEFT_UNTIL_COLOR` | Turn in place ~83° until a marker is (re)detected |
| `ROTATE_180` | Turn in place 180° |
| `DWELL` | Wait `WORKSTATION_WAIT_MS` (2000ms, fixed default) |
| `DWELL <ms>` | Wait the given duration instead, clamped to `[200, 30000]` ms — this is the lever a fleet supervisor uses to avoid a collision by holding a robot longer or shorter than the default |
| `STOP` | Brake, return to IDLE |
| `RESET_POSE` | Brake, zero the odometry origin (`x=y=yaw=0`) |
| `GET_STATUS` | Publish current busy/idle + last detected color |

### Axis convention (for anything converting `_pose` x/y into grid coordinates)

`reset_pose(0,0,0)` is called once in `setup()`, while the robot is still
facing **south** — before the depot-exit maneuver (right turn → drive west →
node0 → right turn → face north into the grid). So once the robot is
driving north into the grid, that's **-x** in the fixed odometry frame set
at `reset_pose`, not +x. See `alvikPoseToGridPoint()` in
`agv_grid_workstation_solver.html` for the working conversion (node spacing:
10in uniform, except the node0→node1 hop which is 13.5in).

### Network config (in `AGV_Factory_color_pose.ino`, top of file)

```cpp
char WIFI_SSID[]     = "YOUR_WIFI_SSID";
char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
char AGENT_IP[]      = "192.0.2.10";  // replace with the micro-ROS agent IP
const uint32_t AGENT_PORT = 8888;
```

### MAC → robot name mapping (`getAlvikID()` in `AGV_Factory_color_pose.ino`)

```cpp
if (mac == "02:00:00:00:00:01") return 1;   // Alvik1
if (mac == "02:00:00:00:00:02") return 2;   // Alvik2
if (mac == "02:00:00:00:00:03") return 3;   // Alvik3
if (mac == "02:00:00:00:00:04") return 4;   // Alvik4
```

To add a robot: flash it, open Serial Monitor to read its MAC, add a line
here and reflash.

---

## Running the fleet

### 1. Start the micro-ROS agent

```bash
micro-ros-agent udp4 --port 8888
```

### 2. Flash `AGV_Factory_color_pose.ino` to each Alvik

Arduino IDE, board = Arduino Nano ESP32 (or your Alvik's ESP32 variant). No
per-unit changes needed — MAC lookup handles identity.

### 3. Send and monitor a command

```bash
ros2 topic pub --once /Alvik1_cmd std_msgs/msg/String \
  "{data: 'FORWARD_UNTIL_RED'}"
ros2 topic echo /Alvik1_status
```

Wait for the robot to return to `IDLE` before sending its next command.

### 4. Plan routes / check for collisions

Open `agv_grid_workstation_solver.html` in a browser. Build routes, check the
Route Report for node/edge collision risk (`SAFETY_WINDOW_SEC=5.0s` for
nodes, `EDGE_SAFETY_WINDOW_SEC=7.0s` for edges).

**VRP-RPD mode** (checkbox next to Solve): each delivered part is processed
for a configurable time (default 15 s) and must then be picked up and
returned to the depot. Pickups are auctioned across the whole fleet
(greedy earliest-completion, capacity-limited) rather than tied to the
delivering robot; tours run drop-block then pick-block so the capacity
constraint always holds; too-early arrivals wait inside the bay via
scheduled DWELLs, enforced in the same fixed-point loop as collision
deconfliction. Then either:

- Export the plan as JSON for downstream dispatch tooling, or
- Switch to **Real mode** (top toggle) to watch live robot positions overlaid
  on the plan, via rosbridge subscribed to each `Alvik#_pose` topic. Set
  "Number of agents" to 1 to track a single robot first.

rosbridge must be running at `ws://localhost:9090` for Real mode to connect.

---

## Localization and live dispatch

AprilTag-based localization: `apriltag_localize.py` maps every detected robot
tag to metric table coordinates (inches) plus yaw, using the table-corner
reference tags 20-23 (20=bottom-left origin corner, 21=bottom-right,
22=top-left, 23=top-right; tag 20 center is 2.75 in from both edges, 20->23
center offset is 91.5 in in x and y). Because the corner tags share the robot
tags' physical plane, a single image->table homography gives metric positions.
Reference tags on the table below elevated robot tags violate that assumption
and introduce location-dependent parallax error; raise references to robot-tag
height. Optional measured lens correction is available through
`--camera-calibration`; it is disabled by default. It runs with just the
two diagonal tags 20+23 (weakest accuracy near the empty 21/22 corners) and
automatically tightens up when all four are placed. With
`--rosbridge <ros2-laptop-ip>` it publishes each robot's pose into the ROS2
graph through the rosbridge websocket (the same route the solver HTML's Real
mode uses; needs `roslibpy` on this machine, `rosbridge_suite` on the ROS2
laptop): one `std_msgs/String` topic per robot, `/<Name>_vision_pose`, JSON
`{"x_in", "y_in", "yaw_deg", "grid_x", "grid_y", "tag_id", "ms"}` plus
additive raw-yaw/filter metadata, and
`/vision_calib` health. Grid coordinates are continuous cells relative to
node 1 (tape-measured at 13.5, 16.75 in from the table edges), 10 in pitch,
axes parallel to the table.
It's meant to supplement or replace the odometry-based `_pose` topic, since
odometry drifts over a run. `test_apriltag_localize.py` validates the fitter
against a synthetic camera.

The solver HTML also has a **Dispatch mode** (no route files): after solving,
"Generate commands" translates each route's segments into AGV_Factory_color_pose.ino commands
(editable before launch — verify the depot exit against a proven route file),
and Start executes the fleet directly over rosbridge: publishes
`<Name>_cmd`, tracks `<Name>_status` done markers, and gates every command
behind a deterministic conflict check anchored to live completion times,
plus two vision checks (robot off-route;
another robot occupying the next node). Conflicts are resolved by inserted
`DWELL`s, longest-remaining-makespan robot gets priority; Hold pauses
issuing and Abort broadcasts STOP. Rerouting mid-run is not built yet.

---

## Archived pipeline

`archive/` holds every sketch and script that predates `AGV_Factory_color_pose.ino`:
`AGV_MULTI_WS_DISPATCH` (+ its `ORACLE_VM` Python dispatch node and
`route_planner.py`, a *different* token vocabulary: `RED YENTRY YWORK DOCK
EXIT BLUE R L YAW0 CLEAR`), `BaseAGV_v4` (the original factory-node
protocol, `/agv_factory/command`), `hardware_gui` (Flask GUI wired to that
protocol), and several line-following/tuning experiments
(`AGV_LINE_FOLLOWING_*`, `AGV_CELL_ROS_TEST`,
`Line_Following_WORKSTATION_ROUTER_WS012_v2`, `Line_Follow_Simple`,
`PD_TUNING`, `color_calibration`). None of it is compatible with `AGV_Factory_color_pose.ino`
— don't mix command vocabularies or topic names across the two.
