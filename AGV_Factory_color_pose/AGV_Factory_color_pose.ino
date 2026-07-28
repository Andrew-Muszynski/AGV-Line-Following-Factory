// =============================================================================
// agv_alvik_microros.ino
// Arduino Alvik — micro-ROS over WiFi
// Low-level motion primitive executor (v2)
//
// Topics (per-robot, named via MAC-based identity — see getAlvikID()):
// <ROBOT_NAME>_status (publisher), e.g. Alvik1_status
// <ROBOT_NAME>_cmd    (subscriber), e.g. Alvik1_cmd
// <ROBOT_NAME>_pose   (publisher), e.g. Alvik1_pose
//   JSON string: {"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis}
//   Published every POSE_PERIOD_MS from alvik.get_pose() / get_battery_charge().
// <ROBOT_NAME>_color  (publisher), e.g. Alvik1_color
//   JSON string: {"r":0..1,"g":0..1,"b":0..1,"h":deg,"s":0..1,"v":0..1,
//                 "color_label":"RED|YELLOW|BLUE|NONE","ms":millis}
//   Published every COLOR_PERIOD_MS from alvik.get_color().
//
// "DWELL" waits WORKSTATION_WAIT_MS (2000ms). "DWELL <ms>" overrides the wait
// (clamped to [MIN_DWELL_MS, MAX_DWELL_MS]) — lets a supervisor lengthen or
// shorten the stop to avoid a collision with another AGV.
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

static char status_buf[256];
static char pose_buf[128];
static char color_buf[192];
static char cmd_buf[128];
std_msgs__msg__String msg_status;
std_msgs__msg__String msg_pose;
std_msgs__msg__String msg_color;
std_msgs__msg__String msg_cmd;

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
const float STRAIGHT_SPEED = 60.0f; // cruise speed when centered on tape
const float MAX_WHEEL_RPM = 70.0f;  // absolute hardware ceiling, both wheels
const float BACKWARD_SPEED = 20.0f;
const float KP = 25.0f;
const float MAX_CORRECTION = 20.0f;
// Matches STRAIGHT_SPEED at zero correction (was 60 when BASE_SPEED was 50 --
// deliberately faster than cruise; that meant a momentary sensor dropout sped
// the robot up right when it had no steering info, and ate into the ~4.3cm
// blind-travel margin LOST_LINE_FAILSAFE_MS was tuned around on 2026-07-14).
const float STICKER_CROSS_SPEED = 50.0f;
const float RIGHT_TURN_DEG = -83.0f;
const float LEFT_TURN_DEG = 83.0f;
const float YAW_TOLERANCE = 2.0f;
const float TURN_MIN_SPEED = 18.0f;
const float TURN_MAX_SPEED = 28.0f; // was 20 -- modest speedup; TURN_MIN_SPEED
                                     // (the gentle final-approach floor near
                                     // YAW_TOLERANCE) is untouched on purpose
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
const unsigned long LINE_GAP_REPORT_MS = 200; // was 200  try to update this
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
unsigned long line_lost_since_ms = 0;
bool line_was_lost = false;
unsigned long last_turn_control_ms = 0;
unsigned long marker_ignore_until_ms = 0; // ignore color right after a command starts
unsigned long advance_until_ms = 0; // keep advancing until this time after detecting a color
unsigned long dwell_until_ms = 0; // wait at the workstation until this time

// FORWARD_THROUGH_<color>: like FORWARD_UNTIL_<color> but does not brake or
// report IDLE on detection -- publishes DETECTED only and keeps line-
// following, so a pre-verified next FORWARD_UNTIL_*/FORWARD_THROUGH_* leg can
// start without the robot ever reaching zero velocity. The supervisor only
// uses this for command pairs it already knows are a straight chain (no turn/
// dwell between them) -- the off-robot dispatcher must pre-scan this case.
// Both ends of the chain must skip the brake: the outgoing detection here,
// AND the incoming next command (see the pass_through_continue check in
// cmdCallback) -- braking on just one side still stops the robot.
bool pass_through = false;
bool pass_through_continue = false; // next cmd arrived while still pass-through-driving

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
// Line following
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
 // Cruise faster when centered, automatically backing off toward the
 // current (proven) curve behavior as correction grows -- at max
 // correction (20) this reduces to 30/70, identical to the old flat
 // BASE_SPEED=50 controller's sharpest curves.
 float cruise = min(STRAIGHT_SPEED, MAX_WHEEL_RPM - fabsf(correction));
 alvik.set_wheels_speed(cruise - correction, cruise + correction, RPM);
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

void startTurn(float delta_deg) {
 turn_start_yaw = get_yaw();
 turn_target_yaw = normalizeYaw(turn_start_yaw + delta_deg);
 last_turn_control_ms = 0;
}

// ROTATE_TO <deg>: turn to an exact absolute heading supplied by the caller
// (e.g. the fleet supervisor correcting drift using vision yaw), instead of
// a relative delta.
void startTurnToAbsolute(float target_deg) {
 turn_start_yaw = get_yaw();
 turn_target_yaw = normalizeYaw(target_deg);
 last_turn_control_ms = 0;
}

bool updateTurn() {
 unsigned long now = millis();
 if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) return false;
 last_turn_control_ms = now;

 float error = yawError(turn_target_yaw, get_yaw());
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

 // FORWARD_THROUGH_<color> arrives while the robot may already be
 // pass-through-driving (no brake since the last detection) -- if so, the
 // NEXT command must also skip the brake, or one-sided braking still stops
 // the robot. pass_through_continue is consumed (cleared) by whichever
 // branch below uses it; only forward-family commands honor it.
 bool skip_brake = pass_through_continue;
 pass_through_continue = false;

 if (cmd == "FORWARD_UNTIL_RED" || cmd == "FORWARD_THROUGH_RED") {
 pass_through = (cmd == "FORWARD_THROUGH_RED");
 if (!skip_brake) alvik.brake();
 reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status(pass_through ? "BUSY FORWARD_THROUGH_RED" : "BUSY FORWARD_UNTIL_RED");
 current_state = STATE_FORWARD_UNTIL_RED; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_COLOR" || cmd == "FORWARD_THROUGH_COLOR") {
 pass_through = (cmd == "FORWARD_THROUGH_COLOR");
 if (!skip_brake) alvik.brake();
 reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status(pass_through ? "BUSY FORWARD_THROUGH_COLOR" : "BUSY FORWARD_UNTIL_COLOR");
 current_state = STATE_FORWARD_UNTIL_COLOR; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_YELLOW" || cmd == "FORWARD_THROUGH_YELLOW") {
 pass_through = (cmd == "FORWARD_THROUGH_YELLOW");
 if (!skip_brake) alvik.brake();
 reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status(pass_through ? "BUSY FORWARD_THROUGH_YELLOW" : "BUSY FORWARD_UNTIL_YELLOW");
 current_state = STATE_FORWARD_UNTIL_YELLOW; is_busy = true;

 } else if (cmd == "FORWARD_UNTIL_BLUE") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); line_was_lost = false; marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 blue_ignore_until_ms = millis() + BLUE_START_IGNORE_MS;
 blue_detection_armed = false;
 blue_nonblue_count = 0;
 publish_status("BUSY FORWARD_UNTIL_BLUE");
 current_state = STATE_FORWARD_UNTIL_BLUE; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_COLOR") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_COLOR");
 current_state = STATE_BACKWARD_UNTIL_COLOR; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_YELLOW") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_YELLOW");
 current_state = STATE_BACKWARD_UNTIL_YELLOW; is_busy = true;

 } else if (cmd == "BACKWARD_UNTIL_BLUE") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY BACKWARD_UNTIL_BLUE");
 current_state = STATE_BACKWARD_UNTIL_BLUE; is_busy = true;

 } else if (cmd == "RIGHT_UNTIL_COLOR") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY RIGHT_UNTIL_COLOR");
 publish_status("TURNING RIGHT");
 startTurn(RIGHT_TURN_DEG);
 current_state = STATE_TURNING_RIGHT; is_busy = true;

 } else if (cmd == "LEFT_UNTIL_COLOR") {
 pass_through = false;
 alvik.brake(); reset_marker_stability(); marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY LEFT_UNTIL_COLOR");
 publish_status("TURNING LEFT");
 startTurn(LEFT_TURN_DEG);
 current_state = STATE_TURNING_LEFT; is_busy = true;

 } else if (cmd == "ROTATE_180") {
 pass_through = false;
 alvik.brake(); reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 publish_status("BUSY ROTATE_180");
 startTurn(180.0f);
 current_state = STATE_ROTATE_180; is_busy = true;

 } else if (cmd.startsWith("ROTATE_TO ")) {
 // ROTATE_TO <deg>: turn to an exact absolute heading supplied by the
 // caller (e.g. the fleet supervisor correcting drift using vision yaw),
 // instead of snapping to the nearest 90/180. No marker/line handling --
 // this is a pure in-place reorientation, same as ROTATE_180.
 pass_through = false;
 float target_deg = cmd.substring(10).toFloat();
 alvik.brake(); reset_marker_stability();
 marker_ignore_until_ms = millis() + getMarkerIgnoreMs();
 char rt_buf[32];
 snprintf(rt_buf, sizeof(rt_buf), "BUSY ROTATE_TO %.1f", target_deg);
 publish_status(rt_buf);
 startTurnToAbsolute(target_deg);
 current_state = STATE_ROTATE_TO; is_busy = true;

 } else if (cmd == "DWELL" || cmd.startsWith("DWELL ")) {
 pass_through = false;
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

 } else if (cmd == "STOP") {
 // BUSY ack added 2026-07-24: every other command acks with BUSY before
 // its done-marker, but STOP went straight to STOPPED -- a caller using
 // the same ack-then-done tracking as every other command (e.g.
 // send_route.py) would see no BUSY and resend forever, even though the
 // robot had already stopped correctly on the first STOP.
 publish_status("BUSY STOP");
 pass_through = false;
 alvik.brake(); publish_status("STOPPED");
 blue_detection_armed = false; blue_ignore_until_ms = 0; blue_nonblue_count = 0;
 current_state = IDLE; is_busy = false;

 } else if (cmd == "RESET_POSE") {
 pass_through = false;
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
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED %s", pending_color_name);
 publish_status(pub_buf);
 if (pass_through) {
 // Keep driving -- no brake, no IDLE. The next command (expected to
 // be a forward continuation, pre-verified by the supervisor) will
 // arrive and pick up current_state from here without ever stopping.
 pass_through_continue = true;
 followLine();
 } else {
 alvik.brake();
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
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
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED YELLOW");
 publish_status(pub_buf);
 if (pass_through) {
 pass_through_continue = true;
 followLine();
 } else {
 alvik.brake();
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
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
 snprintf(pub_buf, sizeof(pub_buf), "DETECTED RED");
 publish_status(pub_buf);
 if (pass_through) {
 pass_through_continue = true;
 followLine();
 } else {
 alvik.brake();
 publish_status("IDLE");
 current_state = IDLE; is_busy = false;
 }
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

 msg_cmd.data.data = cmd_buf;
 msg_cmd.data.size = 0;
 msg_cmd.data.capacity = sizeof(cmd_buf);

 if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
 if (rclc_executor_add_subscription(
 &executor, &sub_cmd, &msg_cmd,
 &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

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
 snprintf(T_STATUS, sizeof(T_STATUS), "%s_status", ROBOT_NAME);
 snprintf(T_CMD,    sizeof(T_CMD),    "%s_cmd",    ROBOT_NAME);
 snprintf(T_POSE,   sizeof(T_POSE),   "%s_pose",   ROBOT_NAME);
 snprintf(T_COLOR,  sizeof(T_COLOR),  "%s_color",  ROBOT_NAME);

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
 }

 delay(LOOP_DELAY_MS);
}
