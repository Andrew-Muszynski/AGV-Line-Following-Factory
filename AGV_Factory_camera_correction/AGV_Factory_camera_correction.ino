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
//   Published every POSE_PERIOD_MS from alvik.get_pose() / get_battery_charge().
// <ROBOT_NAME>_color     (publisher), e.g. Alvik1_color
//   JSON string: {"r":0..1,"g":0..1,"b":0..1,"h":deg,"s":0..1,"v":0..1,
//                 "color_label":"RED|YELLOW|BLUE|NONE","ms":millis}
//   Published every COLOR_PERIOD_MS from alvik.get_color().
// <ROBOT_NAME>_wheel_cmd (subscriber), e.g. Alvik1_wheel_cmd — NEW.
//   Plain-text "<left_rpm> <right_rpm>", applied immediately, no ack. Only
//   has effect in STATE_WHEEL_FOLLOW (entered via the WHEEL_FOLLOW_MODE
//   command on <ROBOT_NAME>_cmd); ignored otherwise. A watchdog
//   (WHEEL_CMD_TIMEOUT_MS) brakes and exits the mode if fresh setpoints stop
//   arriving, so a stalled/crashed off-board controller or a dropped WiFi
//   link can't leave the robot spinning at its last commanded speed.
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
char WIFI_SSID[] = "YOUR_WIFI_SSID";
char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
char AGENT_IP[] = "192.0.2.14";
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
const unsigned long POSE_PERIOD_MS = 300;
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
const unsigned long LOOP_DELAY_MS = 5;

// Extra forward travel after detecting a color/marker, before braking
const unsigned long ADVANCE_AFTER_DETECT_MS = 150;

// Wait (dwell) time at a workstation
// "DWELL" uses WORKSTATION_WAIT_MS; "DWELL <ms>" overrides it (clamped) so a
// fleet supervisor can lengthen/shorten the wait to avoid a collision.
const unsigned long WORKSTATION_WAIT_MS = 2000;
const unsigned long MIN_DWELL_MS = 200;
const unsigned long MAX_DWELL_MS = 30000;
const unsigned long WORKSTATION_BLINK_MS = 250;

// WHEEL_FOLLOW_MODE watchdog: brake if no fresh sub_wheel_cmd setpoint
// arrives within this window. Well above one publish period even at 60 Hz
// (~17 ms) so ordinary jitter never false-trips it, but still short enough
// to catch a genuinely stalled controller or dropped link quickly.
const unsigned long WHEEL_CMD_TIMEOUT_MS = 300;
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
// protocol backs this stream, so a watchdog (WHEEL_CMD_TIMEOUT_MS, checked in
// the state-machine case) brakes the robot if fresh setpoints stop arriving
// -- a stalled/crashed off-board controller or a dropped WiFi link must not
// leave the robot spinning at its last commanded speed indefinitely.
float wheel_cmd_left_rpm = 0.0f;
float wheel_cmd_right_rpm = 0.0f;
unsigned long wheel_cmd_last_ms = 0;
unsigned long line_lost_since_ms = 0;
bool line_was_lost = false;
unsigned long last_turn_control_ms = 0;
unsigned long marker_ignore_until_ms = 0; // ignore color right after a command starts
unsigned long advance_until_ms = 0; // keep advancing until this time after detecting a color
unsigned long dwell_until_ms = 0; // wait at the workstation until this time

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
// publish_status
// =============================================================================
void publish_status(const char* txt) {
 if (!ros_ready) return;
 msg_status.data.data = status_buf;
 msg_status.data.size = snprintf(status_buf, sizeof(status_buf), "%s", txt);
 msg_status.data.capacity = sizeof(status_buf);
 rcl_publish(&pub_status, &msg_status, NULL);
}

// =============================================================================
// publish_pose — x, y, yaw (alvik.get_pose) + battery% (alvik.get_battery_charge)
// =============================================================================
void publish_pose() {
 if (!ros_ready) return;
 if (millis() - last_pose_ms < POSE_PERIOD_MS) return;
 last_pose_ms = millis();

 float x, y, yaw;
 alvik.get_pose(x, y, yaw, CM, DEG);
 int battery = alvik.get_battery_charge();

 msg_pose.data.data = pose_buf;
 msg_pose.data.size = snprintf(pose_buf, sizeof(pose_buf),
 "{\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,\"battery\":%d,\"ms\":%lu}",
 x, y, yaw, battery, millis());
 msg_pose.data.capacity = sizeof(pose_buf);
 rcl_publish(&pub_pose, &msg_pose, NULL);
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
 rcl_publish(&pub_color, &msg_color, NULL);
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
 float target_deg = cmd.substring(10).toFloat();
 alvik.brake(); reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 char rt_buf[32];
 snprintf(rt_buf, sizeof(rt_buf), "BUSY ROTATE_TO %.1f", target_deg);
 publish_status(rt_buf);
 startTurnToAbsolute(target_deg);
 current_state = STATE_ROTATE_TO; is_busy = true;

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
 publish_status("BUSY WHEEL_FOLLOW_MODE");
 current_state = STATE_WHEEL_FOLLOW; is_busy = true;

 } else if (cmd == "STOP") {
 alvik.brake(); publish_status("STOPPED");
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
 if (millis() - wheel_cmd_last_ms > WHEEL_CMD_TIMEOUT_MS) {
 // Watchdog: no fresh setpoint recently -- the off-board controller
 // stalled or the link dropped. Brake and drop out of the mode rather
 // than keep running the last speed indefinitely.
 alvik.brake();
 publish_status("ERROR WHEEL_CMD_TIMEOUT");
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 } else {
 alvik.set_wheels_speed(wheel_cmd_left_rpm, wheel_cmd_right_rpm, RPM);
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
 if (mac == "02:00:00:00:00:01") return 1;    // Alvik1
 if (mac == "02:00:00:00:00:04") return 2;    // Alvik2
 if (mac == "02:00:00:00:00:08") return 3;    // Alvik3
 if (mac == "02:00:00:00:00:07") return 4;    // Alvik4
 if (mac == "02:00:00:00:00:0B") return 5;    // Alvik5
 if (mac == "02:00:00:00:00:05") return 6;    // Alvik6
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
void loop() {
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

 delay(LOOP_DELAY_MS);
}
