// =============================================================================
// AGV_Factory_camera_correction.ino
// Arduino Alvik — micro-ROS over WiFi
// Fork of AGV_Factory_color_pose.ino (do not edit that file from here) that
// adds WHEEL_FOLLOW_MODE: a no-ack streaming input that lets an off-board
// controller (a camera/AprilTag-based yaw PID, see fleet/camera_line_follow.py)
// drive the wheels directly at high rate, e.g. to hold a robot straight on a
// piece of tape using ground-truth vision yaw instead of the onboard IR line
// sensors. Everything else — vocabulary, state machine, turn logic — is
// identical to AGV_Factory_color_pose.ino as of 2026-07-22.
//
// Topics (per-robot, named via MAC-based identity — see getAlvikID()):
// <ROBOT_NAME>_status    (publisher), e.g. Alvik1_status
// <ROBOT_NAME>_cmd       (subscriber), e.g. Alvik1_cmd — discrete commands,
//   BUSY-acked, same vocabulary as AGV_Factory_color_pose.ino.
// <ROBOT_NAME>_pose      (publisher), e.g. Alvik1_pose
//   JSON string: {"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis}
//   Published every POSE_PERIOD_MS when idle/color-driving and every
//   POSE_CONTROL_PERIOD_MS in WHEEL_FOLLOW_MODE.
// <ROBOT_NAME>_color     (publisher), e.g. Alvik1_color
//   JSON string: {"r":0..1,"g":0..1,"b":0..1,"h":deg,"s":0..1,"v":0..1,
//                 "color_label":"RED|YELLOW|BLUE|NONE","ms":millis}
//   Published every COLOR_PERIOD_MS from alvik.get_color().
// <ROBOT_NAME>_wheel_cmd (subscriber), e.g. Alvik1_wheel_cmd — NEW.
//   Plain-text "<left_rpm> <right_rpm>", applied immediately, no ack. Only
//   has effect in STATE_WHEEL_FOLLOW (entered via the WHEEL_FOLLOW_MODE
//   command on <ROBOT_NAME>_cmd); ignored otherwise. A watchdog brakes after
//   WHEEL_CMD_BRAKE_MS and exits the mode after WHEEL_CMD_ABORT_MS if fresh
//   setpoints stop arriving, so a stalled/crashed off-board controller or a
//   dropped WiFi link can't leave the robot spinning at its last speed.
//
// "DWELL" waits WORKSTATION_WAIT_MS (2000ms). "DWELL <ms>" overrides the wait
// (clamped to [MIN_DWELL_MS, MAX_DWELL_MS]) — lets a supervisor lengthen or
// shorten the stop to avoid a collision with another AGV.
//
// ROTATE_TO <deg>: turn to an exact absolute heading (no rounding to a
// 90/180 multiple), for a caller with ground-truth vision yaw correcting
// drift the robot's own IMU didn't see.
//
// CHANGE v1.1: on detecting a color/marker, the robot advances/reverses an
// extra ADVANCE_AFTER_DETECT_MS before braking and publishing DETECTED.
//
// CHANGE v2.2: lost-line failsafe 1500 -> 400 ms (tuned on-table: stops the
// robot before the south edge after a wrong depot-lane command; less blind
// travel before braking) + LINE_GAP telemetry: whenever the line
// reappears after a gap >150 ms, "LINE_GAP <ms>" is published to measure how
// long real sticker crossings last and tune the threshold from data.
// GET_STATUS now includes MAX_GAP.
//
// CHANGE v2.1: yellow saturation threshold 0.60 -> 0.45 (new vinyl reads
// s=0.60 flat / s~0.51 with glare; see STICKER_READING_TEST 2026-07-13).
//
// CHANGE v2: FORWARD_UNTIL_BLUE can start ON TOP OF a blue sticker (depot).
// To avoid confusing the starting sticker with the destination, blue
// detection "arms" in two phases: ignore blue for 1000 ms, then require
// consecutive non-blue readings (confirming the robot has left the starting
// sticker) before accepting the next stable blue.
// =============================================================================

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include "Arduino_Alvik.h"
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

Arduino_Alvik alvik;

// =============================================================================
// WiFi / agent config
// =============================================================================
char WIFI_SSID[] = "AGV_Testbed";
char WIFI_PASSWORD[] = "62110204";
char AGENT_IP[] = "192.168.0.212";  // reverted 2026-08-03: back to the Linux laptop (was 192.168.0.162 / WSL2) -- see memory: wsl2_microros_migration
const uint32_t AGENT_PORT = 8888;

// Per-robot identity / topic names, filled by getAlvikID() in setup()
char ROBOT_NAME[16] = "";
char T_STATUS[32];
char T_CMD[32];
char T_POSE[32];
char T_COLOR[32];
char T_WHEEL_CMD[32];

// =============================================================================
// micro-ROS objects
// =============================================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t pub_status;
rcl_publisher_t pub_pose;
rcl_publisher_t pub_color;
rcl_subscription_t sub_cmd;
// No-ack streaming input for WHEEL_FOLLOW_MODE: an off-board PID controller
// publishes "<left_rpm> <right_rpm>" continuously (e.g. 30-60 Hz) and this
// robot applies it directly, the same trust level as the _pose/_color
// telemetry it already streams out -- unlike sub_cmd/cmdCallback, there is no
// BUSY/IDLE ack, because a continuous setpoint stream has no "done".
rcl_subscription_t sub_wheel_cmd;

static char status_buf[256];
static char pose_buf[128];
static char color_buf[192];
static char cmd_buf[128];
static char wheel_cmd_buf[32];
std_msgs__msg__String msg_status;
std_msgs__msg__String msg_pose;
std_msgs__msg__String msg_color;
std_msgs__msg__String msg_cmd;
std_msgs__msg__String msg_wheel_cmd;

bool ros_ready = false;
unsigned long last_status_ms = 0;
unsigned long last_pose_ms = 0;
unsigned long last_color_ms = 0;

// Stale-session recovery, added 2026-07-31: ros_ready was previously set
// ONCE at boot from initGraph()'s result and never re-checked. If the
// micro-ROS AGENT later loses and silently re-establishes the underlying
// transport session (confirmed on hardware: agent log showed a fresh
// "session established" after an agent restart, with no robot power-cycle),
// this robot's own rcl entities (publishers/executor/node/support) stayed
// bound to the OLD session -- rclc_executor_spin_some() kept running
// without erroring, the LED stayed green (set once at boot, never revisited
// either), but every publish silently went nowhere: confirmed on hardware
// via `ros2 topic echo /Alvik1_status` showing zero messages for 15+
// seconds while /Alvik1_cmd's own subscription still showed as matched.
// rcl_publish()'s return value was discarded everywhere (publish_status/
// publish_pose/publish_color) -- this is the direct, un-proxied signal that
// something is wrong, tracked here instead of an indirect transport ping
// (rmw_uros_ping_agent() would likely still report the transport reachable,
// since the AGENT side reconnected fine -- it's specifically OUR entities
// that are stale, not the network path).
int consecutive_publish_failures = 0;
const int PUBLISH_FAILURE_REINIT_THRESHOLD = 5;
const unsigned long POSE_PERIOD_MS = 300;
// Encoder/camera-assisted position-target control needs fresh odometry while
// wheel setpoints are streaming. 300 ms is acceptable idle telemetry but can
// move the robot several inches between control observations at cruise speed.
// Raise only the active WHEEL_FOLLOW rate so ordinary color/tape missions keep
// their existing network load and timing.
const unsigned long POSE_CONTROL_PERIOD_MS = 50;
const unsigned long COLOR_PERIOD_MS = 150;

// =============================================================================
// Tuning constants
// =============================================================================
const int TAPE_THRESHOLD = 275;
const float BASE_SPEED = 50.0f; // 50 works the best
const float BACKWARD_SPEED = 20.0f;
const float KP = 25.0f;
const float MAX_CORRECTION = 20.0f;
const float STICKER_CROSS_SPEED = 60.0f;
const float YAW_TOLERANCE = 1.0f; // was 2.0 -- tighter stop shrinks the residual
                                   // sensor-bar offset from the tape after a turn
                                   // (the pivot axis isn't at the line sensors, so
                                   // yaw-correct doesn't guarantee sensor-centered)
const float TURN_MIN_SPEED = 18.0f;  //possibly lower this
const float TURN_MAX_SPEED = 60.0f; // was 20.0
const unsigned long TURN_CONTROL_MS = 5;
// ROTATE_REL completion: is_target_reached() polling previously caused full
// firmware hangs, while a blind duration could declare completion before a
// long rotation physically finished. Keep alvik.rotate(..., false), but use
// the independently refreshed onboard IMU to require a stable target before
// braking and acknowledging completion. ROTATE_DEG_PER_SEC is now used only
// to set a hard onboard-verification deadline; it never declares success.
const float ROTATE_DEG_PER_SEC = 100.0f;
// ROTATE_REL now completes on an IMU-confirmed target, not this estimate.
// The estimate is retained only as a hard onboard-verification deadline if
// the motor controller never reaches/stabilizes near the requested heading.
const unsigned long ROTATE_REL_TARGET_TIMEOUT_MS = 2500;
const unsigned long ROTATE_REL_MIN_BRAKE_DELAY_MS = 250;
const unsigned long ROTATE_REL_TARGET_STABLE_MS = 120;
const float ROTATE_REL_MAX_BRAKE_TOLERANCE_DEG = 8.0f;
const float ROTATE_REL_STABLE_HEADING_TOLERANCE_DEG = 1.0f;
const int MARKER_STABLE_SAMPLES = 3;
// v2.2: 1500 -> 400 ms, tuned on the real table (2026-07-14): 400 stops the
// robot (~4.3 cm blind travel) before the table edge at the one place a bad
// command can reach it — the depot lane, ~7.6 cm from the south edge — while
// real sticker crossings stayed below it. If LINE_GAP reports ever approach
// 400 ms (low battery, skewed crossings), raise this before it false-trips.
const unsigned long LOST_LINE_FAILSAFE_MS = 400;
// Report line gaps longer than this once the line reappears ("LINE_GAP <ms>"
// status; supervisors ignore it). Real sticker-crossing durations from these
// reports justify (or veto) lowering LOST_LINE_FAILSAFE_MS further.
const unsigned long LINE_GAP_REPORT_MS = 175; // was 200  try to update this
unsigned long max_line_gap_ms = 0;
// LOOP_DELAY_MS removed 2026-07-30 along with the delay(LOOP_DELAY_MS) call
// at the end of loop() it fed -- see loop()'s comment for why.

// Extra forward travel after detecting a color/marker, before braking
const unsigned long ADVANCE_AFTER_DETECT_MS = 150;

// Wait (dwell) time at a workstation
// "DWELL" uses WORKSTATION_WAIT_MS; "DWELL <ms>" overrides it (clamped) so a
// fleet supervisor can lengthen/shorten the wait to avoid a collision.
const unsigned long WORKSTATION_WAIT_MS = 2000;
const unsigned long MIN_DWELL_MS = 200;
const unsigned long MAX_DWELL_MS = 30000;
const unsigned long WORKSTATION_BLINK_MS = 250;

// WHEEL_FOLLOW_MODE watchdog has two stages. The first missing-command
// threshold still applies an immediate physical brake, preserving the
// original fail-safe response. A short best-effort DDS/WiFi interruption is
// allowed to recover while the robot remains stopped; only a sustained gap
// exits the mode and aborts the route. Real log evidence (run_1527.log): the
// robot measured a 302 ms receive gap even though Python had published only
// 20 ms earlier, proving the 300 ms terminal error could false-trip while the
// controller itself was healthy.
const unsigned long WHEEL_CMD_BRAKE_MS = 300;
const unsigned long WHEEL_CMD_ABORT_MS = 1000;
// Hard speed cap applied to whatever the off-board controller requests, so a
// runaway PID (bad gains, bad yaw reading) can't command an unbounded speed.
// Set to the robot's true mechanical max (~70 RPM per bench testing) --
// camera_line_follow.py is responsible for keeping base_speed +
// max_correction under this on its own (it's meant to run near the top of
// the speed range with only small corrections expected), this constant is
// the last-resort backstop, not the normal-operation ceiling.
const float WHEEL_FOLLOW_MAX_RPM = 70.0f;

// =============================================================================
// State machine
// =============================================================================
enum AGVState {
 IDLE,
 STATE_FORWARD_UNTIL_COLOR,
 STATE_FORWARD_UNTIL_RED,
 STATE_FORWARD_UNTIL_YELLOW,
 STATE_FORWARD_UNTIL_BLUE,
 STATE_BACKWARD_UNTIL_COLOR,
 STATE_BACKWARD_UNTIL_YELLOW,
 STATE_BACKWARD_UNTIL_BLUE,
 STATE_TURNING_RIGHT,
 STATE_TURNING_LEFT,
 STATE_ROTATE_180,
 STATE_ROTATE_TO,
 STATE_ADVANCE_AFTER_FORWARD_COLOR,
 STATE_ADVANCE_AFTER_FORWARD_RED,
 STATE_ADVANCE_AFTER_FORWARD_YELLOW,
 STATE_ADVANCE_AFTER_FORWARD_BLUE,
 STATE_ADVANCE_AFTER_BACKWARD_COLOR,
 STATE_ADVANCE_AFTER_BACKWARD_YELLOW,
 STATE_ADVANCE_AFTER_BACKWARD_BLUE,
 STATE_DWELL,
 STATE_WHEEL_FOLLOW,  // accepting live left/right speeds from sub_wheel_cmd
 // ROTATE_REL <deg> (added 2026-07-30): closed-loop in-place rotation using
 // alvik.rotate(), NOT the wheel-speed-scaling law ROTATE_TO/updateTurn()
 // uses. See the ROTATE_REL cmd handler below and its long comment for why
 // this exists -- short version: camera_grid_navigate.py's Python-side
 // turn_to_heading() (streaming wheel_cmd setpoints, WHEEL_FOLLOW_MODE)
 // was measured on hardware (--turn-test, 2026-07-30) to overshoot by
 // 4-54deg with NO consistent direction or magnitude at ANY tested RPM
 // (10/15/20/60) -- not a tunable brake-lead problem, a fundamentally
 // unreliable control loop over WiFi+DDS. alvik.rotate() runs closed-loop
 // on Alvik's own motor-control MCU (separate UART protocol) with zero
 // network round-trip in the
 // rotation itself, which is what ROTATE_TO/updateTurn() and
 // WHEEL_FOLLOW_MODE's turn_to_heading() both lack. NOTE: completion is
 // detected from a stable onboard IMU target, NOT by polling
 // is_target_reached() -- that was tried first and caused real hardware
 // hangs. The timer is now only a hard onboard-verification deadline.
 STATE_ROTATE_REL,
 STATE_ERROR
};

AGVState current_state = IDLE;

// =============================================================================
// Runtime variables
// =============================================================================
int marker_stable_count = 0;
char last_marker_color[8] = "NONE"; // color type of the current stable count (RED/YELLOW/NONE)
char last_detected_color[16] = "NONE";
char pending_color_name[16] = "NONE"; // detected color, published after the extra advance
float last_h = 0, last_s = 0, last_v = 0;
bool is_busy = false;
float turn_target_yaw = 0.0f;
float turn_start_yaw = 0.0f;
// IMU-sourced heading (alvik.get_orientation, packet 'q'), refreshed every
// loop() iteration regardless of state -- independent of get_pose()'s
// wheel-odometry theta (packet 'z'), which drifts under wheel slip and is
// what publish_pose() still sends for x/y/theta tracking. Turn logic reads
// this global instead of calling get_orientation() itself, so "current
// heading" is always fresh and comes from one place.
float robot_heading_deg = 0.0f;
// WHEEL_FOLLOW_MODE: last speeds received on sub_wheel_cmd, and when. No ack
// protocol backs this stream, so the two-stage watchdog checked in the state
// machine brakes first and terminates the mode if fresh setpoints stay absent
// -- a stalled/crashed off-board controller or a dropped WiFi link must not
// leave the robot spinning at its last commanded speed indefinitely.
float wheel_cmd_left_rpm = 0.0f;
float wheel_cmd_right_rpm = 0.0f;
unsigned long wheel_cmd_last_ms = 0;
bool wheel_cmd_watchdog_braked = false;
unsigned long wheel_cmd_watchdog_max_gap_ms = 0;
unsigned long line_lost_since_ms = 0;
bool line_was_lost = false;
unsigned long last_turn_control_ms = 0;
unsigned long marker_ignore_until_ms = 0; // ignore color right after a command starts
unsigned long advance_until_ms = 0; // keep advancing until this time after detecting a color
unsigned long dwell_until_ms = 0; // wait at the workstation until this time
unsigned long rotate_rel_done_ms = 0; // onboard-verification deadline, never success
unsigned long rotate_rel_min_brake_ms = 0;
unsigned long rotate_rel_in_tolerance_since_ms = 0;
float rotate_rel_target_yaw = 0.0f;
float rotate_rel_brake_tolerance_deg = 1.0f;
float rotate_rel_stable_heading = 0.0f;

const unsigned long CMD_MARKER_IGNORE_MS = 700; // ms to ignore color after receiving a command

// v2: FORWARD_UNTIL_BLUE can start ON TOP OF a blue sticker (depot: the
// parking sticker or a lane junction). To avoid confusing the starting
// sticker with the destination: 1) ignore blue for BLUE_START_IGNORE_MS;
// 2) require BLUE_ARM_NONBLUE_SAMPLES consecutive non-blue readings
// (confirming the robot has left the starting sticker); only then is blue
// detection armed.
const unsigned long BLUE_START_IGNORE_MS = 1000;
const int BLUE_ARM_NONBLUE_SAMPLES = 3;
unsigned long blue_ignore_until_ms = 0;
bool blue_detection_armed = false;
int blue_nonblue_count = 0;

// If the last detected color was RED, scale the ignore window
// (multiplier is currently 1.0, i.e. no change)
unsigned long getMarkerIgnoreMs() {
 if (strcmp(last_detected_color, "RED") == 0) {
 return (unsigned long)(CMD_MARKER_IGNORE_MS * 1.0f);
 }
 return CMD_MARKER_IGNORE_MS;
}

// =============================================================================
// LED helpers
// =============================================================================
void setLEDGreen() { alvik.left_led.set_color(0,1,0); alvik.right_led.set_color(0,1,0); }
void setLEDRed() { alvik.left_led.set_color(1,0,0); alvik.right_led.set_color(1,0,0); }
void setLEDBlue() { alvik.left_led.set_color(0,0,1); alvik.right_led.set_color(0,0,1); }
void setLEDYellow() { alvik.left_led.set_color(1,1,0); alvik.right_led.set_color(1,1,0); }
void setLEDOff() { alvik.left_led.set_color(0,0,0); alvik.right_led.set_color(0,0,0); }

// =============================================================================
// notePublishResult — shared stale-session detector for every rcl_publish()
// call (see consecutive_publish_failures' own comment above for the full
// story). A single non-OK result doesn't necessarily mean the session is
// dead (a single dropped/best-effort write is normal and expected on this
// transport) -- only PUBLISH_FAILURE_REINIT_THRESHOLD in a row, with no
// successful publish in between, is treated as "the session is stale,
// reconnect." Any success resets the counter to 0 immediately.
// =============================================================================
void notePublishResult(rcl_ret_t ret) {
 if (ret == RCL_RET_OK) {
 consecutive_publish_failures = 0;
 return;
 }
 consecutive_publish_failures++;
}

// =============================================================================
// publish_status
// =============================================================================
void publish_status(const char* txt) {
 if (!ros_ready) return;
 msg_status.data.data = status_buf;
 msg_status.data.size = snprintf(status_buf, sizeof(status_buf), "%s", txt);
 msg_status.data.capacity = sizeof(status_buf);
 notePublishResult(rcl_publish(&pub_status, &msg_status, NULL));
}

// =============================================================================
// publish_pose — x, y, yaw (alvik.get_pose) + battery% (alvik.get_battery_charge)
// =============================================================================
void publish_pose() {
 if (!ros_ready) return;
 unsigned long pose_period = (current_state == STATE_WHEEL_FOLLOW)
   ? POSE_CONTROL_PERIOD_MS : POSE_PERIOD_MS;
 if (millis() - last_pose_ms < pose_period) return;
 last_pose_ms = millis();

 float x, y, yaw;
 alvik.get_pose(x, y, yaw, CM, DEG);
 int battery = alvik.get_battery_charge();

 msg_pose.data.data = pose_buf;
 msg_pose.data.size = snprintf(pose_buf, sizeof(pose_buf),
 "{\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,\"battery\":%d,\"ms\":%lu}",
 x, y, yaw, battery, millis());
 msg_pose.data.capacity = sizeof(pose_buf);
 notePublishResult(rcl_publish(&pub_pose, &msg_pose, NULL));
}

// =============================================================================
// Color classification — RED, YELLOW, BLUE
// Taken from Andrew's code; includes false-yellow rejection on black tape
// =============================================================================
const int RED_STABLE_SAMPLES = 4;
const int YELLOW_STABLE_SAMPLES = 1;
const int BLUE_STABLE_SAMPLES = 5;

bool isRed(float h, float s, float v) {
 bool hue_red = (h > 340.0f || h < 20.0f);
 bool saturated = s > 0.40f;
 bool bright_enough = v > 0.04f;
 return hue_red && saturated && bright_enough;
}

float colorChroma(float a, float b, float c) {
 float max_val = a; if (b > max_val) max_val = b; if (c > max_val) max_val = c;
 float min_val = a; if (b < min_val) min_val = b; if (c < min_val) min_val = c;
 return max_val - min_val;
}

bool isYellow(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right) {
 float chroma = colorChroma(nr, ng, nb);
 bool tape_now = (left > TAPE_THRESHOLD || center > TAPE_THRESHOLD || right > TAPE_THRESHOLD);

 // Rejects the repeatable false-yellow reading on shiny black tape
 bool black_tape_false_yellow =
 tape_now &&
 h > 65.0f && h < 95.0f &&
 s < 0.25f &&
 v < 0.30f &&
 chroma < 0.05f;

 if (black_tape_false_yellow) return false;

 // bool hue_yellow = h > 32.0f && h < 50.0f; // works for Alvik1, Alvik2, Alvik4
 bool hue_yellow = h > 25.0f && h < 50.0f;
 // v2.1: 0.60 -> 0.45. New yellow VINYL reads s=0.637-0.682 centered (only
 // ~6% margin over 0.60; old paint read ~0.70) and s~0.51 at partial sticker
 // coverage / edges (sticker test 2026-07-13). Lower cutoff = insurance for
 // sun, wear, and per-robot sensor spread (Alvik3 yellow reads low); worst
 // case is triggering ~1-2 cm early at the sticker edge, absorbed by the
 // post-detect advance. No cross-talk risk: no non-yellow surface in the
 // test read h 25-50 with s > 0.40.
 bool strongly_saturated = s > 0.45f;
 bool bright_enough = v > 0.08f;
 bool colorful_enough = chroma > 0.075f;

 return hue_yellow && strongly_saturated && bright_enough && colorful_enough;
}

bool isBlue(float h, float s, float v) {
 return h > 190.0f && h < 260.0f && s > 0.60f && v > 0.05f;
}

const char* classifyColor(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right) {
 if (isRed(h, s, v)) return "RED";
 if (isYellow(h, s, v, nr, ng, nb, left, center, right)) return "YELLOW";
 if (isBlue(h, s, v)) return "BLUE";
 return nullptr;
}

// =============================================================================
// publish_color — RGB + HSV + classifier label for continuous color logging
// =============================================================================
void publish_color() {
 if (!ros_ready) return;
 if (millis() - last_color_ms < COLOR_PERIOD_MS) return;
 last_color_ms = millis();

 float h, s, v;
 float r, g, b;
 int left, center, right;
 alvik.get_color(h, s, v, HSV);
 alvik.get_color(r, g, b, RGB);
 alvik.get_line_sensors(left, center, right);

 const char* label = classifyColor(h, s, v, r, g, b, left, center, right);
 if (label == nullptr) label = "NONE";

 msg_color.data.data = color_buf;
 msg_color.data.size = snprintf(color_buf, sizeof(color_buf),
 "{\"r\":%.3f,\"g\":%.3f,\"b\":%.3f,\"h\":%.1f,\"s\":%.3f,\"v\":%.3f,\"color_label\":\"%s\",\"ms\":%lu}",
 r, g, b, h, s, v, label, millis());
 msg_color.data.capacity = sizeof(color_buf);
 notePublishResult(rcl_publish(&pub_color, &msg_color, NULL));
}

void reset_marker_stability() { marker_stable_count = 0; }

bool checkStableRed() {
 if (millis() < marker_ignore_until_ms) { marker_stable_count = 0; return false; }
 float h, s, v;
 alvik.get_color(h, s, v, HSV);
 if (isRed(h, s, v)) {
 marker_stable_count++;
 last_h = h; last_s = s; last_v = v;
 strncpy(last_detected_color, "RED", sizeof(last_detected_color) - 1);
 } else {
 marker_stable_count = 0;
 }
 if (marker_stable_count >= RED_STABLE_SAMPLES) {
 marker_stable_count = 0;
 return true;
 }
 return false;
}

bool checkStableYellow() {
 if (millis() < marker_ignore_until_ms) { marker_stable_count = 0; return false; }

 float h, s, v, nr, ng, nb;
 int left, center, right;
 alvik.get_color(h, s, v, HSV);
 alvik.get_color(nr, ng, nb, RGB);
 alvik.get_line_sensors(left, center, right);

 bool yellow_now = isYellow(h, s, v, nr, ng, nb, left, center, right);

 if (yellow_now) {
 marker_stable_count++;
 last_h = h; last_s = s; last_v = v;
 strncpy(last_detected_color, "YELLOW", sizeof(last_detected_color) - 1);
 } else {
 marker_stable_count = 0;
 }

 if (marker_stable_count >= YELLOW_STABLE_SAMPLES) {
 marker_stable_count = 0;
 return true;
 }
 return false;
}

bool checkStableBlue() {
 if (millis() < marker_ignore_until_ms) { marker_stable_count = 0; return false; }

 float h, s, v;
 alvik.get_color(h, s, v, HSV);

 if (isBlue(h, s, v)) {
 marker_stable_count++;
 last_h = h; last_s = s; last_v = v;
 strncpy(last_detected_color, "BLUE", sizeof(last_detected_color) - 1);
 } else {
 marker_stable_count = 0;
 }

 if (marker_stable_count >= BLUE_STABLE_SAMPLES) {
 marker_stable_count = 0;
 return true;
 }
 return false;
}

bool checkStableColor(const char** color_name) {
 if (millis() < marker_ignore_until_ms) { marker_stable_count = 0; return false; }

 float h, s, v, nr, ng, nb;
 int left, center, right;
 alvik.get_color(h, s, v, HSV);
 alvik.get_color(nr, ng, nb, RGB);
 alvik.get_line_sensors(left, center, right);

 const char* c = classifyColor(h, s, v, nr, ng, nb, left, center, right);

 // If the detected color type changes (RED <-> YELLOW <-> NONE), reset the
 // counter -- same as last_marker_target in Andrew's code. Prevents samples
 // of different colors from mixing into the same stable count.
 const char* c_key = c ? c : "NONE";
 if (strcmp(c_key, last_marker_color) != 0) {
 marker_stable_count = 0;
 strncpy(last_marker_color, c_key, sizeof(last_marker_color) - 1);
 }

 if (c) {
 marker_stable_count++;
 last_h = h; last_s = s; last_v = v;
 strncpy(last_detected_color, c, sizeof(last_detected_color) - 1);
 } else {
 marker_stable_count = 0;
 }

 int needed = (c != nullptr && strcmp(c, "YELLOW") == 0) ? YELLOW_STABLE_SAMPLES : RED_STABLE_SAMPLES;

 if (c && marker_stable_count >= needed) {
 marker_stable_count = 0;
 if (color_name) *color_name = last_detected_color;
 return true;
 }
 return false;
}

// =============================================================================
// Line following (onboard IR sensors -- unused while STATE_WHEEL_FOLLOW is
// active; camera_line_follow.py drives the wheels directly in that mode)
// =============================================================================
void followLine() {
 int left_s, center_s, right_s;
 alvik.get_line_sensors(left_s, center_s, right_s);
 bool tape_now = (left_s > TAPE_THRESHOLD || center_s > TAPE_THRESHOLD || right_s > TAPE_THRESHOLD);

 if (!tape_now) {
 alvik.set_wheels_speed(STICKER_CROSS_SPEED, STICKER_CROSS_SPEED, RPM);
 } else {
 float sum = left_s + center_s + right_s;
 float centroid = (left_s + center_s * 2.0f + right_s * 3.0f) / sum;
 float error = -(centroid - 2.0f);
 float correction = constrain(error * KP, -MAX_CORRECTION, MAX_CORRECTION);
 alvik.set_wheels_speed(BASE_SPEED - correction, BASE_SPEED + correction, RPM);
 }

 if (!tape_now) {
 if (!line_was_lost) { line_lost_since_ms = millis(); line_was_lost = true; }
 else if (millis() - line_lost_since_ms > LOST_LINE_FAILSAFE_MS) {
 alvik.brake();
 publish_status("ERROR LINE_LOST");
 current_state = STATE_ERROR;
 }
 } else {
 // v2.2: the line reappeared — measure how long the gap (sticker
 // crossing) lasted and publish LINE_GAP to calibrate the failsafe.
 if (line_was_lost) {
 unsigned long gap = millis() - line_lost_since_ms;
 if (gap > max_line_gap_ms) max_line_gap_ms = gap;
 if (gap >= LINE_GAP_REPORT_MS) {
 char gap_buf[32];
 snprintf(gap_buf, sizeof(gap_buf), "LINE_GAP %lums", gap);
 publish_status(gap_buf);
 }
 }
 line_was_lost = false;
 }
}

// =============================================================================
// Yaw helpers
// =============================================================================
float get_yaw() {
 float x, y, yaw;
 alvik.get_pose(x, y, yaw, CM, DEG);
 return yaw;
}

float normalizeYaw(float a) {
 a = fmod(a + 360.0f, 360.0f);
 if (a < 0) a += 360.0f;
 return a;
}

float yawError(float target, float current) {
 return fmod((normalizeYaw(target) - normalizeYaw(current) + 540.0f), 360.0f) - 180.0f;
}

// Refresh robot_heading_deg from the IMU (alvik.get_orientation) -- called
// once per loop() iteration unconditionally, so it stays live whether the
// robot is turning, driving straight, or idle, and any code that needs
// "which way am I facing right now" always has a fresh drift-free answer.
void update_robot_heading() {
 float roll, pitch, yaw;
 alvik.get_orientation(roll, pitch, yaw);
 robot_heading_deg = normalizeYaw(yaw);
}

// Snap the target to the nearest absolute 90 deg grid heading in the
// commanded direction, instead of "current yaw + 90", so each turn corrects
// any drift left over from previous turns/line-following instead of
// compounding it. A relative +-83 deg offset (the old constant) undershoots
// true-grid by 7 deg every time and stacks that bias turn after turn, which
// is what left robots visibly off-angle at workstations.
// yaw_increases matches this robot's sign convention, not compass
// clockwise/counterclockwise: physical RIGHT decreases yaw (old
// RIGHT_TURN_DEG was -83), physical LEFT increases it (old was +83) -- see
// the wheel-speed signs in updateTurn(). Call with false for RIGHT, true for
// LEFT.
void startTurnToNearest90(bool yaw_increases) {
 update_robot_heading();
 turn_start_yaw = robot_heading_deg;
 float nearest90 = roundf(turn_start_yaw / 90.0f) * 90.0f;
 turn_target_yaw = normalizeYaw(yaw_increases ? nearest90 + 90.0f : nearest90 - 90.0f);
 last_turn_control_ms = 0;
}

// Snap to the absolute heading 180 deg from the nearest grid heading, rather
// than "current yaw + 180" -- same drift-correction reasoning as above.
void startTurnTo180() {
 update_robot_heading();
 turn_start_yaw = robot_heading_deg;
 float nearest90 = roundf(turn_start_yaw / 90.0f) * 90.0f;
 turn_target_yaw = normalizeYaw(nearest90 + 180.0f);
 last_turn_control_ms = 0;
}

// Turn to an EXACT absolute heading, no rounding to a 90/180 multiple --
// for ROTATE_TO <deg>, where a supervisor with ground-truth vision yaw
// supplies the target directly (e.g. correcting drift the robot's own IMU
// didn't see, or holding it to something other than a grid-aligned heading).
void startTurnToAbsolute(float target_deg) {
 update_robot_heading();
 turn_start_yaw = robot_heading_deg;
 turn_target_yaw = normalizeYaw(target_deg);
 last_turn_control_ms = 0;
}

bool updateTurn() {
 unsigned long now = millis();
 if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) return false;
 last_turn_control_ms = now;

 update_robot_heading();
 float error = yawError(turn_target_yaw, robot_heading_deg);
 if (fabsf(error) <= YAW_TOLERANCE) { alvik.brake(); return true; }

 float scale = constrain(fabsf(error) / 90.0f, 0.0f, 1.0f);
 float spd = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * scale;

 if (error > 0.0f) alvik.set_wheels_speed(-spd, spd, RPM);
 else alvik.set_wheels_speed( spd, -spd, RPM);
 return false;
}

// =============================================================================
// /agv/cmd callback
// =============================================================================
void cmdCallback(const void* msgin) {
 const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
 String cmd = String(msg->data.data);
 cmd.trim();
 char pub_buf[256];

 if (cmd == "FORWARD_UNTIL_RED") {
 alvik.brake(); reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY FORWARD_UNTIL_RED");
 current_state = STATE_FORWARD_UNTIL_RED; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_COLOR") {
 alvik.brake(); reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY FORWARD_UNTIL_COLOR");
 current_state = STATE_FORWARD_UNTIL_COLOR; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_YELLOW") {
 alvik.brake(); reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY FORWARD_UNTIL_YELLOW");
 current_state = STATE_FORWARD_UNTIL_YELLOW; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_BLUE") {
 alvik.brake(); reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 blue_ignore_until_ms = millis() + BLUE_START_IGNORE_MS;
 blue_detection_armed = false;
 blue_nonblue_count = 0;
 publish_status("BUSY FORWARD_UNTIL_BLUE");
 current_state = STATE_FORWARD_UNTIL_BLUE; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_COLOR") {
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_COLOR");
 current_state = STATE_BACKWARD_UNTIL_COLOR; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_YELLOW") {
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_YELLOW");
 current_state = STATE_BACKWARD_UNTIL_YELLOW; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_BLUE") {
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_BLUE");
 current_state = STATE_BACKWARD_UNTIL_BLUE; is_busy = true;

 } else if (cmd == "RIGHT_UNTIL_COLOR") {
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY RIGHT_UNTIL_COLOR");
 publish_status("TURNING RIGHT");
 startTurnToNearest90(false);  // RIGHT: yaw decreases (matches old RIGHT_TURN_DEG=-83)
 current_state = STATE_TURNING_RIGHT; is_busy = true;

 } else if (cmd == "LEFT_UNTIL_COLOR") {
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY LEFT_UNTIL_COLOR");
 publish_status("TURNING LEFT");
 startTurnToNearest90(true);  // LEFT: yaw increases (matches old LEFT_TURN_DEG=+83)
 current_state = STATE_TURNING_LEFT; is_busy = true;

 } else if (cmd == "ROTATE_180") {
 alvik.brake(); reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY ROTATE_180");
 startTurnTo180();
 current_state = STATE_ROTATE_180; is_busy = true;

 } else if (cmd.startsWith("ROTATE_TO ")) {
 // ROTATE_TO <deg>: turn to an exact absolute heading supplied by the
 // caller (e.g. the fleet supervisor correcting drift using vision yaw),
 // instead of snapping to the nearest 90/180. No marker/line handling --
 // this is a pure in-place reorientation, same as ROTATE_180.
 //
 // NOTE 2026-07-30: this command's underlying control law (updateTurn(),
 // below -- alvik.set_wheels_speed() scaled proportionally to yaw error,
 // using this robot's OWN onboard robot_heading_deg) is the SAME KIND of
 // control loop as camera_grid_navigate.py's turn_to_heading() (which
 // streams wheel_cmd setpoints computed from VISION yaw instead). Both
 // are open-loop-per-tick wheel-speed laws with no hardware-level
 // stopping-distance compensation. camera_grid_navigate.py's version was
 // measured (--turn-test) to overshoot 4-54deg unpredictably at every
 // tested RPM -- this command was NOT re-tested after that finding, but
 // shares the same architecture, so treat it with the same suspicion
 // until it's specifically verified. For camera-corrected turns, prefer
 // ROTATE_REL (below), which uses alvik.rotate() -- a genuinely different,
 // closed-loop-on-hardware primitive -- instead of either of these
 // proportional wheel-speed laws.
 float target_deg = cmd.substring(10).toFloat();
 alvik.brake(); reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 char rt_buf[32];
 snprintf(rt_buf, sizeof(rt_buf), "BUSY ROTATE_TO %.1f", target_deg);
 publish_status(rt_buf);
 startTurnToAbsolute(target_deg);
 current_state = STATE_ROTATE_TO; is_busy = true;

 } else if (cmd.startsWith("ROTATE_REL ")) {
 // ROTATE_REL <deg> (added 2026-07-30): relative in-place rotation using
 // alvik.rotate(deg, DEG, false) -- Alvik's own closed-loop primitive,
 // executed on the separate motor-control MCU via its UART packet
 // protocol (packetC1F('R', ...), see Arduino_Alvik::rotate() /
 // is_target_reached() in the library source), NOT a wheel-speed law
 // computed here on the ESP32 side. +deg = CCW, -deg = CW (matches
 // alvik.rotate()'s own convention directly, no remapping).
 //
 // WHY THIS EXISTS: WHEEL_FOLLOW_MODE-based turning (the Python-side
 // turn_to_heading() in camera_grid_navigate.py, streaming live
 // left/right RPM setpoints computed from vision yaw every ~20ms) was
 // measured on real hardware 2026-07-30 via --turn-test at turn_rpm =
 // 10, 15, 20, and 60: EVERY run overshot the target after the brake
 // fired, by anywhere from 4deg to 54deg, with NO consistent direction
 // (sometimes past the target, sometimes short, alternating turn to
 // turn) and no consistent relationship to RPM (10 RPM overshot 4-8deg;
 // 60 RPM overshot 22-54deg; a "no progress for 5s" stall was also
 // observed at 60 RPM mid-turn). This is NOT a brake-lead-distance
 // tuning problem (the kind --stop-test/--turn-test's brake_lead_deg
 // exists to fix) -- the actual real-world route run this was diagnosing
 // (Alvik1, D1->DE1->0->1->2->10->9->1->0->DE1->D1) burned all 5
 // re-approach attempts oscillating between roughly +2.6deg and -2.3deg
 // of a single target heading and never converged inside the 2deg
 // tolerance. Streaming wheel setpoints over WiFi -> micro-ROS agent ->
 // rclpy, closing the loop against CAMERA vision at ~20-100ms cadence,
 // has no hardware-level stopping-distance compensation and no tight
 // real-time guarantee -- alvik.rotate() moves that entire loop onto
 // the robot's own motor controller instead.
 //
 // NO alvik.brake() here (unlike ROTATE_180/ROTATE_TO/every other
 // command) -- FOUND 2026-07-30 after two hardware hangs (LED frozen,
 // ZERO ROS traffic -- status AND pose both dead, robot unresponsive to
 // GET_STATUS -- required power-cycle to recover both times).
 // Arduino_Alvik::parse_message() (library source) DISCARDS any ack
 // byte that arrives while waiting_ack == NO_ACK (case 'x': sets
 // last_ack then immediately zeroes it right back out). rotate() does
 // not set waiting_ack = 'R' until AFTER its own internal delay(200) +
 // UART write. brake()'s own UART write (via drive(0,0)) immediately
 // before rotate() adds UART traffic in that same narrow window,
 // increasing the odds a fast ack from the motor-control MCU lands
 // before waiting_ack is armed and gets silently thrown away --
 // is_target_reached() then polls forever for an ack that will never
 // come again, and the caller's own poll loop (this sketch's loop(),
 // and camera_grid_navigate.py's ROTATE_REL-COMPLETE wait) hangs with
 // it. The robot is ALREADY STATIONARY between commands (every command
 // handler ends by transitioning to a terminal/IDLE-reachable state
 // with the wheels stopped), so this brake() was always redundant here
 // -- it existed only by copy-paste consistency with ROTATE_180/
 // ROTATE_TO, which use the wheel-speed law (updateTurn()) and
 // genuinely need a fresh brake() to zero any residual wheel speed
 // before starting their own control loop. rotate() has no such need.
 // NEVER add alvik.brake() (or any other UART-writing alvik.* call)
 // back in immediately before alvik.rotate() without re-verifying this
 // race is actually closed.
 // COMPLETION DETECTION UPDATE 2026-07-30 (same day, after two hardware
 // hangs even with brake() removed above): is_target_reached() polling
 // was abandoned entirely, not just the brake() race fixed. See
 // ROTATE_DEG_PER_SEC's comment near the top of this file for why --
 // short version: it never reliably acked on real hardware, and the only
 // proven-working alvik.rotate() usage in this codebase (driveTo.ino)
 // never polls it either, always just waits a fixed duration instead.
 // Completion below is IMU-target-gated and non-blocking; its timer is only
 // a hard onboard-verification deadline.
 float rel_deg = cmd.substring(11).toFloat();
 reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 char rr_buf[32];
 snprintf(rr_buf, sizeof(rr_buf), "BUSY ROTATE_REL %.1f", rel_deg);
 publish_status(rr_buf);
 update_robot_heading();
 rotate_rel_target_yaw = normalizeYaw(robot_heading_deg + rel_deg);
 // Small camera corrections need a correspondingly tight gate; a fixed
 // broad tolerance would otherwise declare a 2-4 degree command complete
 // before it moved. Full grid turns may use up to 8 degrees, leaving only a
 // minor residual for the deliberate camera correction phase.
 rotate_rel_brake_tolerance_deg = constrain(
   0.25f * fabsf(rel_deg), 1.0f, ROTATE_REL_MAX_BRAKE_TOLERANCE_DEG);
 rotate_rel_in_tolerance_since_ms = 0;
 rotate_rel_stable_heading = robot_heading_deg;
 alvik.rotate(rel_deg, DEG, false);  // non-blocking; completion is IMU-target-gated below
 rotate_rel_min_brake_ms = millis() + ROTATE_REL_MIN_BRAKE_DELAY_MS;
 rotate_rel_done_ms = millis()
     + (unsigned long)(1000.0f * fabsf(rel_deg) / ROTATE_DEG_PER_SEC)
     + ROTATE_REL_TARGET_TIMEOUT_MS;
 current_state = STATE_ROTATE_REL; is_busy = true;

 } else if (cmd == "DWELL" || cmd.startsWith("DWELL ")) {
 unsigned long dwell_ms = WORKSTATION_WAIT_MS;
 if (cmd.length() > 6) {
 long requested_ms = cmd.substring(6).toInt();
 if (requested_ms > 0) {
 dwell_ms = (unsigned long)constrain(requested_ms, (long)MIN_DWELL_MS, (long)MAX_DWELL_MS);
 }
 }
 alvik.brake();
 publish_status("BUSY DWELL");
 dwell_until_ms = millis() + dwell_ms;
 current_state = STATE_DWELL; is_busy = true;

 } else if (cmd == "WHEEL_FOLLOW_MODE") {
 // Enter live wheel-speed-follower mode: sub_wheel_cmd now drives the
 // motors directly (see wheelCmdCallback/STATE_WHEEL_FOLLOW) until any
 // other command is received (STOP included) or the watchdog trips.
 // Busy/never-idle by design -- there is no "done" for a streamed mode,
 // the caller ends it explicitly.
 alvik.brake();
 wheel_cmd_left_rpm = 0.0f; wheel_cmd_right_rpm = 0.0f;
 wheel_cmd_last_ms = millis();
 wheel_cmd_watchdog_braked = false;
 wheel_cmd_watchdog_max_gap_ms = 0;
 publish_status("BUSY WHEEL_FOLLOW_MODE");
 current_state = STATE_WHEEL_FOLLOW; is_busy = true;

 } else if (cmd == "STOP") {
 // DO NOT try to cancel an in-flight rotate from here with
 // alvik.rotate(0, DEG, false). That was tried 2026-08-25 (incident #7)
 // and BENCH-MEASURED AS A CLEAR REGRESSION: --stop-interrupt-test went
 // from 2/9 stopped cleanly (4-21deg overrun) to 0/9 (8-61deg overrun),
 // Alvik1, same 9 trials. Two reasons, both now confirmed:
 //
 // 1. Arduino_Alvik::rotate() contains a blocking delay(200) BEFORE its
 //    UART write (library source: semaphore -> delay(200) -> write ->
 //    waiting_ack='R'). Calling it first therefore DELAYS THE BRAKE BY
 //    200ms -- the exact opposite of what a stop needs. The two trials
 //    that passed at baseline were the early ones (STOP ~0.23s in); with
 //    the cancel they drifted 8.3 and 9.1deg, and ~200ms at ~100deg/s is
 //    ~20deg, the right order of magnitude.
 // 2. Mid/late trials degraded far MORE than that 200ms alone explains
 //    (4-21deg -> 46-61deg), which says a zero-angle 'R' packet does not
 //    cancel the co-processor's active maneuver at all -- it appears to
 //    re-target or restart it.
 //
 // Any future cancel attempt must NOT route through alvik.rotate() (the
 // delay(200) is unconditional and disqualifying for a stop path). See
 // ROTATE_REL_HAZARD_2026-08-20.md for the full table and the remaining
 // candidates.
 alvik.brake(); publish_status("STOPPED");
 wheel_cmd_watchdog_braked = false;
 wheel_cmd_watchdog_max_gap_ms = 0;
 blue_detection_armed = false; blue_ignore_until_ms = 0; blue_nonblue_count = 0;
 current_state = IDLE; is_busy = false;

 } else if (cmd == "RESET_POSE") {
 alvik.brake(); alvik.reset_pose(0, 0, 0, CM, DEG);
 publish_status("POSE_RESET");
 current_state = IDLE; is_busy = false;

 } else if (cmd == "GET_STATUS") {
 snprintf(pub_buf, sizeof(pub_buf), "%s LAST_COLOR=%s MAX_GAP=%lums",
 is_busy ? "BUSY" : "IDLE", last_detected_color, max_line_gap_ms);
 publish_status(pub_buf);

 } else {
 publish_status("ERROR UNKNOWN_COMMAND");
 }
}

// =============================================================================
// /agv/wheel_cmd callback -- no-ack streaming setpoints for WHEEL_FOLLOW_MODE
// =============================================================================
void wheelCmdCallback(const void* msgin) {
 const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
 // Expected payload: "<left_rpm> <right_rpm>", e.g. "42.5 38.1" or "-10 10".
 // Ignored outside STATE_WHEEL_FOLLOW so a stray/late message can't move the
 // robot while it's doing something else.
 if (current_state != STATE_WHEEL_FOLLOW) return;
 float left = 0.0f, right = 0.0f;
 if (sscanf(msg->data.data, "%f %f", &left, &right) != 2) return;
 wheel_cmd_left_rpm = constrain(left, -WHEEL_FOLLOW_MAX_RPM, WHEEL_FOLLOW_MAX_RPM);
 wheel_cmd_right_rpm = constrain(right, -WHEEL_FOLLOW_MAX_RPM, WHEEL_FOLLOW_MAX_RPM);
 wheel_cmd_last_ms = millis();
}

// =============================================================================
// State machine
// =============================================================================
void update_state_machine() {
 const char* color_name = nullptr;
 char pub_buf[256];

 switch (current_state) {
 case IDLE: break;

 case STATE_FORWARD_UNTIL_COLOR:
 if (checkStableColor(&color_name)) {
 // Color detected: instead of braking now, advance a bit further first
 strncpy(pending_color_name, color_name, sizeof(pending_color_name) - 1);
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_COLOR");
 current_state = STATE_ADVANCE_AFTER_FORWARD_COLOR;
 } else { followLine(); }
 break;

 case STATE_ADVANCE_AFTER_FORWARD_COLOR:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED %s", pending_color_name);
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { followLine(); }
 break;

 case STATE_FORWARD_UNTIL_YELLOW:
 if (checkStableYellow()) {
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_YELLOW");
 current_state = STATE_ADVANCE_AFTER_FORWARD_YELLOW;
 } else { followLine(); }
 break;

 case STATE_ADVANCE_AFTER_FORWARD_YELLOW:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED YELLOW");
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { followLine(); }
 break;

 case STATE_FORWARD_UNTIL_BLUE:
 if (!blue_detection_armed) {
 // v2 — Arming phase: the robot may have started ON TOP OF its starting
 // blue sticker. Ignore blue for 1000 ms, then require consecutive
 // non-blue readings before arming detection. Line-follow throughout.
 if (millis() >= blue_ignore_until_ms) {
 float h, s, v;
 alvik.get_color(h, s, v, HSV);
 if (!isBlue(h, s, v)) {
 blue_nonblue_count++;
 if (blue_nonblue_count >= BLUE_ARM_NONBLUE_SAMPLES) {
 blue_detection_armed = true;
 reset_marker_stability(); // blue stable count starts clean
 }
 } else {
 blue_nonblue_count = 0;
 }
 }
 followLine();
 } else if (checkStableBlue()) {
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_BLUE");
 current_state = STATE_ADVANCE_AFTER_FORWARD_BLUE;
 } else { followLine(); }
 break;

 case STATE_ADVANCE_AFTER_FORWARD_BLUE:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED BLUE");
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { followLine(); }
 break;

 case STATE_BACKWARD_UNTIL_COLOR:
 if (checkStableColor(&color_name)) {
 // Color detected: instead of braking now, reverse a bit further first
 strncpy(pending_color_name, color_name, sizeof(pending_color_name) - 1);
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_COLOR");
 current_state = STATE_ADVANCE_AFTER_BACKWARD_COLOR;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_ADVANCE_AFTER_BACKWARD_COLOR:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED %s", pending_color_name);
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_BACKWARD_UNTIL_YELLOW:
 if (checkStableYellow()) {
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_YELLOW");
 current_state = STATE_ADVANCE_AFTER_BACKWARD_YELLOW;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_ADVANCE_AFTER_BACKWARD_YELLOW:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED YELLOW");
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_BACKWARD_UNTIL_BLUE:
 if (checkStableBlue()) {
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_BLUE");
 current_state = STATE_ADVANCE_AFTER_BACKWARD_BLUE;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_ADVANCE_AFTER_BACKWARD_BLUE:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED BLUE");
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { alvik.set_wheels_speed(-BACKWARD_SPEED, -BACKWARD_SPEED, RPM); }
 break;

 case STATE_TURNING_RIGHT:
 if (updateTurn()) {
 publish_status("TURN COMPLETE");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
 break;

 case STATE_TURNING_LEFT:
 if (updateTurn()) {
 publish_status("TURN COMPLETE");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
 break;

 case STATE_FORWARD_UNTIL_RED:
 if (checkStableRed()) {
 // Red detected: advance a bit further before braking
 advance_until_ms = millis() + ADVANCE_AFTER_DETECT_MS;
 publish_status("BUSY ADVANCE_AFTER_RED");
 current_state = STATE_ADVANCE_AFTER_FORWARD_RED;
 } else { followLine(); }
 break;

 case STATE_ADVANCE_AFTER_FORWARD_RED:
 if (millis() >= advance_until_ms) {
 alvik.brake();
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED RED");
 publish_status(pub_buf); publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else { followLine(); }
 break;

 case STATE_ROTATE_180:
 if (updateTurn()) {
 publish_status("ROTATE_180 COMPLETE");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
 break;

 case STATE_ROTATE_TO:
 if (updateTurn()) {
 publish_status("ROTATE_TO COMPLETE");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
 break;

 case STATE_ROTATE_REL:
 // Do not poll is_target_reached(): that ack path caused full firmware
 // hangs on this hardware. Instead use the already-refreshed onboard IMU.
 // Crucially, this is NOT the old blind timer brake that could fire while
 // the maneuver was still active. Brake only after yaw has stayed within
 // the angle-scaled target tolerance for ROTATE_REL_TARGET_STABLE_MS.
 {
   unsigned long now = millis();
   float error = fabsf(yawError(rotate_rel_target_yaw, robot_heading_deg));
   if (now >= rotate_rel_min_brake_ms
       && error <= rotate_rel_brake_tolerance_deg) {
     if (rotate_rel_in_tolerance_since_ms == 0) {
       rotate_rel_in_tolerance_since_ms = now;
       rotate_rel_stable_heading = robot_heading_deg;
     } else if (fabsf(yawError(
                  robot_heading_deg, rotate_rel_stable_heading))
                > ROTATE_REL_STABLE_HEADING_TOLERANCE_DEG) {
       // Inside the target band but still moving: restart the stationarity
       // window instead of braking on a drive-through sample.
       rotate_rel_in_tolerance_since_ms = now;
       rotate_rel_stable_heading = robot_heading_deg;
     } else if (now - rotate_rel_in_tolerance_since_ms
                >= ROTATE_REL_TARGET_STABLE_MS) {
       alvik.brake();
       publish_status("ROTATE_REL COMPLETE");
       publish_status("IDLE");
       current_state = IDLE; is_busy = false;
     }
   } else {
     rotate_rel_in_tolerance_since_ms = 0;
   }

  if (current_state == STATE_ROTATE_REL && now >= rotate_rel_done_ms) {
    // Never report COMPLETE merely because a timer elapsed. Brake first,
    // then report a provisional terminal state. Camera-capable controllers
    // may accept it only after fresh, settled vision verifies their yaw
    // tolerance; encoder-only control must abort because its sole onboard
    // localization source did not confirm the target. This is deliberately
    // not an ERROR: the fleet supervisor would exclude the robot before its
    // navigation worker could perform that independent camera verification.
    alvik.brake();
    publish_status("ROTATE_REL BRAKED_UNCONFIRMED");
     publish_status("IDLE");
     current_state = IDLE; is_busy = false;
   }
 }
 break;

 case STATE_DWELL:
 alvik.brake();
 if (((millis() / WORKSTATION_BLINK_MS) % 2) == 0) setLEDYellow(); else setLEDOff();
 if (millis() >= dwell_until_ms) {
 setLEDGreen();
 publish_status("DWELL COMPLETE");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
 break;

 case STATE_WHEEL_FOLLOW:
 {
 unsigned long wheel_cmd_gap_ms = millis() - wheel_cmd_last_ms;
 if (wheel_cmd_gap_ms > WHEEL_CMD_BRAKE_MS) {
 // Stage 1 always brakes at 300 ms, exactly as before. Remain in
 // WHEEL_FOLLOW_MODE so a transient best-effort delivery stall can recover
 // from a NEW command instead of permanently excluding a healthy robot.
 if (!wheel_cmd_watchdog_braked) {
   alvik.brake();
   wheel_cmd_watchdog_braked = true;
 }
 if (wheel_cmd_gap_ms > wheel_cmd_watchdog_max_gap_ms) {
   wheel_cmd_watchdog_max_gap_ms = wheel_cmd_gap_ms;
 }

 // Stage 2: a full second without a received setpoint is a sustained
 // controller/link loss. The robot has already been physically braked since
 // 300 ms; now terminate the mode and make the route abort explicitly.
 if (wheel_cmd_gap_ms > WHEEL_CMD_ABORT_MS) {
   snprintf(pub_buf, sizeof(pub_buf),
            "ERROR WHEEL_CMD_TIMEOUT gap=%lums", wheel_cmd_gap_ms);
   publish_status(pub_buf);
   publish_status("IDLE");
   current_state = IDLE; is_busy = false;
 }
 } else {
 if (wheel_cmd_watchdog_braked) {
   snprintf(pub_buf, sizeof(pub_buf),
            "WHEEL_CMD_STALL_RECOVERED gap=%lums",
            wheel_cmd_watchdog_max_gap_ms);
   publish_status(pub_buf);
   wheel_cmd_watchdog_braked = false;
   wheel_cmd_watchdog_max_gap_ms = 0;
 }
 alvik.set_wheels_speed(wheel_cmd_left_rpm, wheel_cmd_right_rpm, RPM);
 }
 }
 break;

 case STATE_ERROR:
 alvik.brake();
 break;
 }
}

// =============================================================================
// MAC-based identity — picks ROBOT_NAME (Alvik1..Alvik4) from the WiFi MAC,
// so each robot gets its own <ROBOT_NAME>_cmd / <ROBOT_NAME>_status topics.
// =============================================================================
// Old mac address if we have 6 workstation
// int getAlvikID() {
//  String mac = WiFi.macAddress();
//  mac.toUpperCase();
//  if (mac == "02:00:00:00:00:01") return 1;   // Alvik1
//  if (mac == "02:00:00:00:00:04") return 2;   // Alvik2
//  if (mac == "02:00:00:00:00:09") return 3;   // Alvik3 Fault in yellow detection. Low H value
//  if (mac == "02:00:00:00:00:07") return 4;   // Alvik4
//  if (mac == "02:00:00:00:00:06") return 5;  //  Alvik5 Fault in yellow detection. Low H value
//  if (mac == "02:00:00:00:00:08") return 6;  // Alvik6
//  return 1;
// }

int getAlvikID() {
 String mac = WiFi.macAddress();
 mac.toUpperCase();
 if (mac == "3C:84:27:C2:87:50") return 1;    // Alvik1
 if (mac == "3C:84:27:C3:E8:4C") return 2;    // Alvik2
 if (mac == "48:CA:43:2E:32:FC") return 3;    // Alvik3
 if (mac == "48:CA:43:2E:1D:CC") return 4;    // Alvik4
 if (mac == "80:65:99:C5:E5:70") return 5;    // Alvik5
 if (mac == "3C:84:27:C3:EA:EC") return 6;    // Alvik6
 return 1;
}

// =============================================================================
// micro-ROS init (same pattern as Andrew's code)
// =============================================================================
void initTransport() {
 set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
 unsigned long start_ms = millis();
 while (WiFi.status() != WL_CONNECTED && millis() - start_ms < 15000) {
 setLEDYellow(); delay(250);
 setLEDOff(); delay(250);
 }
}

// =============================================================================
// finalizeGraph — teardown for reconnectGraph() below, added 2026-07-31.
// Reverse of initGraph()'s creation order: executor first (it holds
// references to the subscriptions, must go before they're finalized), then
// subscriptions, then publishers, then node, then support. Best-effort: a
// stale/already-broken session may fail some of these finalizers (the whole
// POINT of this path is that our local handles may no longer correspond to
// anything real on the agent side) -- every call result is ignored on
// purpose, since the actual recovery is initGraph() creating BRAND NEW
// entities afterward, not these old ones succeeding at cleanup.
// =============================================================================
void finalizeGraph() {
 rclc_executor_fini(&executor);
 rcl_subscription_fini(&sub_wheel_cmd, &node);
 rcl_subscription_fini(&sub_cmd, &node);
 rcl_publisher_fini(&pub_color, &node);
 rcl_publisher_fini(&pub_pose, &node);
 rcl_publisher_fini(&pub_status, &node);
 rcl_node_fini(&node);
 rclc_support_fini(&support);
}

// =============================================================================
// reconnectGraph — called from loop() once consecutive_publish_failures
// crosses PUBLISH_FAILURE_REINIT_THRESHOLD (see that constant's own comment
// for the full story: the agent can silently re-establish a fresh session
// after a restart without this robot ever power-cycling, leaving our old
// entities bound to nothing). Tears down whatever we currently hold and
// re-runs initGraph() to bind fresh entities to the CURRENT session.
// millis()-only timing throughout (no delay()), matching the rest of this
// firmware's control loop -- a blocking reconnect here would freeze
// followLine()/update_state_machine() for its duration, same reasoning
// already applied everywhere else in this file.
// =============================================================================
void reconnectGraph() {
 setLEDYellow();  // visible "reconnecting" indicator, matches initTransport()'s use of yellow while WiFi associates
 finalizeGraph();
 ros_ready = initGraph();
 consecutive_publish_failures = 0;
 if (ros_ready) {
 setLEDGreen();
 } else {
 setLEDRed();
 }
}

bool initGraph() {
 allocator = rcl_get_default_allocator();
 if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
 if (rclc_node_init_default(&node, ROBOT_NAME, "", &support) != RCL_RET_OK) return false;

 if (rclc_publisher_init_default(
 &pub_status, &node,
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
 T_STATUS) != RCL_RET_OK) return false;

 if (rclc_publisher_init_best_effort(
 &pub_pose, &node,
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
 T_POSE) != RCL_RET_OK) return false;

 if (rclc_publisher_init_best_effort(
 &pub_color, &node,
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
 T_COLOR) != RCL_RET_OK) return false;

 if (rclc_subscription_init_best_effort(
 &sub_cmd, &node,
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
 T_CMD) != RCL_RET_OK) return false;

 if (rclc_subscription_init_best_effort(
 &sub_wheel_cmd, &node,
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
 T_WHEEL_CMD) != RCL_RET_OK) return false;

 msg_cmd.data.data = cmd_buf;
 msg_cmd.data.size = 0;
 msg_cmd.data.capacity = sizeof(cmd_buf);

 msg_wheel_cmd.data.data = wheel_cmd_buf;
 msg_wheel_cmd.data.size = 0;
 msg_wheel_cmd.data.capacity = sizeof(wheel_cmd_buf);

 // 2 handles now: sub_cmd (discrete, BUSY-acked) + sub_wheel_cmd (streamed,
 // no ack). Both must be registered below or spin_some() will never
 // deliver one of them.
 if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
 if (rclc_executor_add_subscription(
 &executor, &sub_cmd, &msg_cmd,
 &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
 if (rclc_executor_add_subscription(
 &executor, &sub_wheel_cmd, &msg_wheel_cmd,
 &wheelCmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

 // Initialize msg_status (no buffer yet — assigned in publish_status)
 msg_status.data.data = status_buf;
 msg_status.data.size = 0;
 msg_status.data.capacity = sizeof(status_buf);

 msg_pose.data.data = pose_buf;
 msg_pose.data.size = 0;
 msg_pose.data.capacity = sizeof(pose_buf);

 msg_color.data.data = color_buf;
 msg_color.data.size = 0;
 msg_color.data.capacity = sizeof(color_buf);

 return true;
}

// =============================================================================
// setup() — Alvik first, then micro-ROS (same as Andrew's code)
// =============================================================================
void setup() {
 Serial.begin(115200);

 // 1. Alvik first
 alvik.begin();
 alvik.reset_pose(0, 0, 0, CM, DEG);
 alvik.set_illuminator(true);
 setLEDBlue();
 Serial.println("Alvik OK");

 // 2. micro-ROS
 initTransport();

 Serial.print("MAC: "); Serial.println(WiFi.macAddress());

 snprintf(ROBOT_NAME, sizeof(ROBOT_NAME), "Alvik%d", getAlvikID());
 snprintf(T_STATUS,    sizeof(T_STATUS),    "%s_status",    ROBOT_NAME);
 snprintf(T_CMD,       sizeof(T_CMD),       "%s_cmd",       ROBOT_NAME);
 snprintf(T_POSE,      sizeof(T_POSE),      "%s_pose",      ROBOT_NAME);
 snprintf(T_COLOR,     sizeof(T_COLOR),     "%s_color",     ROBOT_NAME);
 snprintf(T_WHEEL_CMD, sizeof(T_WHEEL_CMD), "%s_wheel_cmd", ROBOT_NAME);

 ros_ready = initGraph();

 if (ros_ready) {
 setLEDGreen();
 Serial.println("micro-ROS OK");
 publish_status("IDLE");
 } else {
 setLEDRed();
 Serial.println("micro-ROS FAIL — continuando sin ROS");
 }
}

// =============================================================================
// loop()
// =============================================================================
// delay(LOOP_DELAY_MS) REMOVED 2026-07-30 -- standing rule: never use
// delay() anywhere it could run during normal operation, it blocks
// rclc_executor_spin_some()/state-machine servicing for its full duration
// (exactly the kind of stall the ROTATE_REL hang investigation was
// chasing, even though this specific 5ms delay() was not itself the
// confirmed cause -- removing it costs nothing and rules it out entirely).
// delay(5) existed only to avoid spinning the ESP32 at 100% CPU, not to
// rate-limit anything semantically -- rclc_executor_spin_some() and the
// state machine should run every pass, as fast as possible, not be gated
// behind a timer. No replacement pacing added: loop() just runs
// unthrottled now.
void loop() {
 // Stale-session recovery -- checked BEFORE spin_some()/anything else uses
 // the (possibly stale) executor this tick. See
 // consecutive_publish_failures' own comment for the full story. Only
 // fires once ros_ready was true at least once (initTransport()'s own WiFi
 // retry loop in setup() handles never having connected at all -- a
 // different problem from a connection that was good and went stale).
 if (ros_ready && consecutive_publish_failures >= PUBLISH_FAILURE_REINIT_THRESHOLD) {
 reconnectGraph();
 }

 if (ros_ready) {
 rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
 }

 update_robot_heading();

 if (alvik.get_touch_cancel()) {
 alvik.brake();
 setLEDRed();
 publish_status("ERROR EMERGENCY_STOP");
 blue_detection_armed = false; blue_ignore_until_ms = 0; blue_nonblue_count = 0;
 current_state = STATE_ERROR;
 is_busy = false;
 }

 // WHEEL_FOLLOW safety hardening (2026-08-26, single-robot route failure):
 // the first spin_some() above was followed by Alvik orientation + touch
 // semaphore reads before the 300 ms wheel watchdog ran. If either read
 // stalled temporarily, fresh best-effort wheel packets could already be
 // queued while wheel_cmd_last_ms still looked expired; the state machine
 // would brake before the next loop got a chance to consume them. Service
 // the executor once more immediately before the watchdog. This does not
 // weaken a real controller/link-loss stop: with no queued command the
 // timestamp remains old and the same 300 ms check still fires.
 if (ros_ready && current_state == STATE_WHEEL_FOLLOW) {
 rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
 }

 update_state_machine();

 if (ros_ready) {
 // Publish IDLE periodically while at rest
 if (!is_busy && millis() - last_status_ms > 1000) {
 publish_status("IDLE");
 last_status_ms = millis();
 }

 publish_pose();
 publish_color();
 }
}
