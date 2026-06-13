# AGV Testbed — Setup and Handoff Guide

This system runs a fleet of Arduino Alvik AGVs on an 8×8 grid using ROS 2 and micro-ROS over WiFi. The PC (or VM) runs the dispatch node; each Alvik runs a micro-ROS sketch that receives token sequences and executes them blindly.

---

## Repository layout

```
AGV_MULTI_WS_DISPATCH/
    AGV_MULTI_WS_DISPATCH.ino   ← Arduino sketch flashed to every Alvik
ORACLE_VM/
    dispatch_node.py            ← ROS 2 node: mission commands → token sequences → Alviks
    route_planner.py            ← Routing engine (no ROS dependency, pure Python)
    agv_robots.yaml             ← Robot names and MAC addresses
    workstations.json           ← Workstation positions on the 8×8 grid
    INSTRUCTIONS.txt            ← Quick-reference launch cheat sheet
```

---

## Prerequisites

### On the PC / VM running the dispatch node

| Requirement | Version tested |
|---|---|
| Ubuntu 22.04 (or WSL2 on Windows) | 22.04 LTS |
| ROS 2 Humble | humble |
| Python 3.10+ | 3.10 |
| Python packages | `pyyaml` (`pip install pyyaml`) |
| micro-ROS agent | see below |

Install micro-ROS agent (first time only):

```bash
sudo snap install micro-ros-agent
# OR build from source:
# https://micro.ros.arduino.io/
```

### On each Alvik

- Arduino IDE 2.x
- Arduino Alvik library (install via Arduino Library Manager → search "Arduino Alvik")
- micro_ros_arduino library — download the `.zip` for ESP32 from:
  https://github.com/micro-ROS/micro_ros_arduino/releases
  Install via Arduino IDE → Sketch → Include Library → Add .ZIP Library

---

## Network setup

All Alviks and the dispatch PC must be on the **same WiFi network**.

The sketch hard-codes the network credentials. Open `AGV_MULTI_WS_DISPATCH.ino` and change these three lines near the top:

```cpp
char WIFI_SSID[]     = "AGV_SWARM";      // ← your network SSID
char WIFI_PASSWORD[] = "ISECap123";      // ← your network password
char AGENT_IP[]      = "192.168.1.141";  // ← IP of the PC running the micro-ROS agent
const uint32_t AGENT_PORT = 8888;        // leave as 8888 unless you changed it
```

To find the PC's IP on Linux/WSL:

```bash
ip addr show | grep "inet " | grep -v 127
```

---

## Alvik MAC addresses and IDs

Each Alvik is identified by its WiFi MAC address. The mapping is in two places and must match:

**`agv_robots.yaml`** — controls which ROS topics the dispatch node creates:

```yaml
agvs:
  agv_1:
    name: "Alvik1"
    mac_address: "3C:84:27:C2:87:50"
  agv_2:
    name: "Alvik2"
    mac_address: "3C:84:27:C3:E7:DC"
```

**`AGV_MULTI_WS_DISPATCH.ino`** — `getAlvikID()` function (around line 338):

```cpp
int getAlvikID() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "3C:84:27:C2:87:50") return 1;   // Alvik1
  if (mac == "3C:84:27:C3:E7:DC") return 2;   // Alvik2
  if (mac == "74:4D:BD:A2:1B:70") return 3;   // Alvik3
  if (mac == "48:CA:43:2E:1D:CC") return 4;   // Alvik4
  return 1;                                    // fallback
}
```

To add a new Alvik:
1. Flash the sketch to it, connect it to WiFi, open Serial Monitor — it will print its MAC address.
2. Add the MAC → ID mapping to `getAlvikID()`.
3. Add the corresponding entry to `agv_robots.yaml` with the matching name `AlvikN`.

---

## Launch order

### 1. Start the micro-ROS agent on the PC

```bash
micro-ros-agent udp4 --port 8888
```

Leave this running in its own terminal. Every Alvik connects to this agent over UDP.

### 2. Flash the Alviks

Open `AGV_MULTI_WS_DISPATCH/AGV_MULTI_WS_DISPATCH.ino` in Arduino IDE.

- Select board: **Arduino Nano ESP32** (or the ESP32 variant your Alvik uses)
- Select the correct COM port
- Flash each Alvik (no changes needed per-unit — the MAC lookup handles identity automatically)

Place each Alvik on its blue start sticker facing **north** (toward the grid). The LED will blink red/off while searching for the blue sticker, then go solid blue once confirmed.

### 3. Source ROS 2 and start the dispatch node

```bash
source /opt/ros/humble/setup.bash
cd ~/VRP/ORACLE_VM          # or wherever you cloned the repo
python dispatch_node.py
```

You should see:

```
[INFO] AGV DISPATCH NODE starting
[INFO] Loaded 28 workstations
[INFO] AGVs: ['agv_1', 'agv_2', 'agv_3', 'agv_4']
[INFO] AGV DISPATCH NODE ready
```

### 4. Send a mission

From a second terminal (with ROS 2 sourced):

```bash
# Grid traversal test — serpentine 8×8, counts all 64 red stickers
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'GRID_TEST agv_1'}"

# Specific workstation route
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'DISPATCH {\"routes\":[{\"agv\":\"agv_1\",\"workstations\":[\"WS01\",\"WS04\",\"WS07\"]}]}'}"

# Random 3-stop route
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'RANDOM {\"agv\":\"agv_1\",\"n_stops\":3,\"seed\":42}'}"

# Emergency stop all
ros2 topic pub --once /agv_dispatch/command std_msgs/msg/String \
  "{data: 'STOP_ALL'}"
```

---

## Monitor telemetry

```bash
# Global dispatch status (1 Hz JSON summary of all AGVs)
ros2 topic echo /agv_dispatch/status

# Single Alvik raw telemetry (200 ms — includes yaw, line sensors, color, red_count)
ros2 topic echo /Alvik1_status
ros2 topic echo /Alvik1_color
```

The color topic JSON format:

```json
{"color":"RED","h":0.0,"s":1.000,"v":0.060,"L":784,"C":314,"R":598,
 "red_count":21,"yaw":-287.5,"tgt":102.8,"bot":9.8}
```

- `yaw` — actual IMU heading (degrees, raw -180..+180 from IMU)
- `tgt` — `leg_target_yaw`, the heading reference for the current leg
- `bot` — bottom ToF distance in cm (6–7 cm = over sticker, 9–11 cm = over black tape)
- `red_count` — cumulative red stickers detected this run

---

## Workstation layout (`workstations.json`)

Workstations are defined as pairs of adjacent grid nodes. The grid is 8×8, numbered row-major from the bottom-left:

```
Row 8:  57 58 59 60 61 62 63 64
Row 7:  49 50 51 52 53 54 55 56
...
Row 1:   1  2  3  4  5  6  7  8
```

Depot is node 1 (bottom-left). The AGV starts facing north on the blue sticker at node 1.

To add or change workstations, edit `workstations.json`. Each entry needs:

```json
{"id": "WS01", "between_nodes": [10, 11]}
```

`between_nodes` is the pair of grid nodes the workstation spur sits between.

---

## Tuning constants (in the Arduino sketch)

All tuning is at the top of `AGV_MULTI_WS_DISPATCH.ino`. The values that are currently working best:

| Constant | Value | Purpose |
|---|---|---|
| `BASE_SPEED` | 40.0 | Forward drive speed (RPM) |
| `KP` | 120.0 | Line-following proportional gain |
| `KD` | 8.0 | Line-following derivative gain |
| `MAX_CORRECTION` | 30.0 | Max RPM differential from line PD |
| `DRIVE_TRIM` | 0.0 | Constant RPM offset to cancel motor asymmetry (negative = slow right wheel) |
| `KP_YAW_BLEND` | 1.0 | Heading correction strength on clean tape |
| `KP_YAW_CROSS` | 2.0 | Heading hold strength during sticker blind window |
| `MARKER_BLIND_MS` | 350 | Duration (ms) to suppress line sensors over a sticker |
| `YAW_TOLERANCE` | 3.0 | Degrees within which a turn is considered settled |

**If the robot drifts right** (yaw goes increasingly negative on a straight north leg): make `DRIVE_TRIM` more negative (try -2.0, -3.0, -4.0).

**If the robot oscillates** side-to-side on the tape: reduce `KP` or `KP_YAW_BLEND`.

**If the robot loses the tape after a turn**: `KP_YAW_CROSS` is too low, or the turn is not settling — check that `YAW_TOLERANCE` is reachable (3° is usually fine).

---

## Debugging a route without a robot

```bash
cd ~/VRP/ORACLE_VM
python route_planner.py route WS01 WS04 WS07 --format explain
python route_planner.py random 4 --seed 42 --format json
python route_planner.py route WS02 WS06 --format tokens
```

---

## Common problems

**Alvik LED stays blinking red/off after placing on blue sticker**
→ The color sensor is not seeing the blue sticker. Adjust position — the sensor needs to be roughly centered on the sticker. The robot must stay still for 1.5 seconds for confirmation.

**dispatch_node prints "AGV considered offline"**
→ The Alvik is not publishing status. Check: WiFi connected? micro-ROS agent running? Correct `AGENT_IP` in sketch?

**`loadScript` returns false / robot stays IDLE after `run` command**
→ The token string contains an unknown token. Check the command for typos. Valid tokens: `RED YENTRY YWORK DOCK DWELL EXIT BLUE R L YAW0 CLEAR`.

**Robot spins endlessly during a turn**
→ The target yaw is numerically degenerate (e.g. computed from an un-normalized `leg_target_yaw` that wraps past ±180°). Use `GRID_TEST` to confirm turns work before running full routes.

**`red_count` stops incrementing mid-run**
→ Robot has drifted off the tape. Check `DRIVE_TRIM` — set it so yaw stays flat with `KP_YAW_BLEND=0` on a straight run, then re-enable blend.
