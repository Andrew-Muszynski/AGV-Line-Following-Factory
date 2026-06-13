/*
 * Line_Follow_Simple
 *
 * Same ROS 2 / token / color-detection infrastructure as AGV_MULTI_WS_DISPATCH,
 * but line following is reduced to the simplest form that works well — matching
 * the Arduino "Line_follower" example:
 *
 *   error   = centroid(L, C, R)   weighted position, zero when centered
 *   control = error * KP
 *   left    = BASE_SPEED - control
 *   right   = BASE_SPEED + control
 *
 * No D term. No yaw blend. No blind window on intersections.
 * The color sensor still stops the robot at RED / YELLOW / BLUE targets.
 * Turns still use the IMU-based rotate-to-heading sequence.
 *
 * Tune KP and BASE_SPEED only. If the robot oscillates, lower KP.
 * If it understeers on curves, raise KP.
 */

#include "Arduino_Alvik.h"
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

Arduino_Alvik alvik;

// =====================================================
// MICRO-ROS CONFIGURATION
// =====================================================

char WIFI_SSID[]     = "AGV_SWARM";
char WIFI_PASSWORD[] = "ISECap123";
char AGENT_IP[]      = "192.168.1.141";
const uint32_t AGENT_PORT = 8888;

char ROBOT_NAME[16] = "";
char T_STATUS[32];
char T_CMD[32];
char T_COLOR[32];

rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;
rcl_publisher_t pub_status;
rcl_publisher_t pub_color;
rcl_subscription_t sub_cmd;
std_msgs__msg__String msg_status;
std_msgs__msg__String msg_color;
std_msgs__msg__String msg_cmd_in;
char cmd_buf[512];

bool ros_ready = false;
unsigned long last_status_ms = 0;
unsigned long last_color_ms  = 0;
const unsigned long STATUS_PERIOD_MS = 200;

// =====================================================
// TUNING  — only these two matter for line following
// =====================================================

const float BASE_SPEED = 60.0f;   // RPM, both wheels when centered
const float KP         = 50.0f;   // proportional gain on centroid error

// Speeds for special manoeuvres — adjust if needed
const float YELLOW_SEARCH_SPEED        = 20.0f;
const float WORKSTATION_APPROACH_SPEED = 30.0f;
const float REVERSE_DOCK_SPEED         = 20.0f;
const float CLEAR_MARKER_SPEED         = 28.0f;
const float POST_TURN_EXIT_SPEED       = 25.0f;

// Turn controller
const float YAW_TOLERANCE  = 3.0f;
const float TURN_MIN_SPEED = 5.0f;
const float TURN_MAX_SPEED = 60.0f;
const float REVERSE_DOCK_ABSOLUTE_YAW = 0.0f;

// Tape / marker thresholds
const int   TAPE_THRESHOLD  = 250;
const int   RED_STABLE_SAMPLES    = 1;
const int   YELLOW_STABLE_SAMPLES = 1;
const int   BLUE_STABLE_SAMPLES   = 5;

// Timing guards
const unsigned long TURN_CONTROL_MS               = 2;
const unsigned long TURN_CENTERING_MS             = 250;
const unsigned long PRE_CENTER_BRAKE_MS           = 100;
const unsigned long PRE_ROTATE_BRAKE_MS           = 200;
const unsigned long POST_ROTATE_BRAKE_MS          = 200;
const unsigned long POST_TURN_EXIT_MS             = 350;
const unsigned long MARKER_IGNORE_AFTER_TURN_MS   = 600;
const unsigned long MARKER_IGNORE_AFTER_RED_MS    = 1000;
const unsigned long YELLOW_ARM_AFTER_RED_MS       = 750;
const unsigned long YELLOW_IGNORE_AFTER_YAW_MS    = 300;
const unsigned long EXIT_WORKSTATION_IGNORE_MS    = 1000;
const unsigned long WORKSTATION_WAIT_MS           = 2000;
const unsigned long WORKSTATION_BLINK_MS          = 250;
const unsigned long CLEAR_MARKER_MS               = 500;
const unsigned long LOST_LINE_FAILSAFE_MS         = 1500;
const unsigned long LOOP_DELAY_MS                 = 10;   // 100 Hz — matches example's delay(100) character
const unsigned long START_CONFIRM_MS              = 1500;

// =====================================================
// TOKEN VOCABULARY
// =====================================================

enum TokenOp : uint8_t {
  TOK_RED    = 0,
  TOK_YENTRY = 1,
  TOK_YWORK  = 2,
  TOK_DOCK   = 3,
  TOK_DWELL  = 4,
  TOK_EXIT   = 5,
  TOK_BLUE   = 6,
  TOK_R      = 7,
  TOK_L      = 8,
  TOK_YAW0   = 9,
  TOK_CLEAR  = 10,
  TOK_UNKNOWN = 0xFF
};

#define MAX_TOKENS 256
TokenOp  token_script[MAX_TOKENS];
uint16_t token_count = 0;
uint16_t token_index = 0;

// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState : uint8_t {
  WAIT_FOR_START,
  EXEC_TOKEN,
  DRIVE_TO_RED,
  DRIVE_TO_YENTRY,
  DRIVE_TO_YWORK,
  REVERSE_TO_DOCK,
  WORKSTATION_WAIT,
  DRIVE_OUT_TO_YENTRY,
  DRIVE_TO_BLUE,
  DO_CLEAR,
  TURN_GENERIC,
  PAUSED,
  DONE,
  EMERGENCY_STOP
};

RobotState robot_state      = WAIT_FOR_START;
RobotState after_turn_state = DONE;

enum TurnPhase : uint8_t {
  TURN_IDLE,
  TURN_PRE_CENTER_BRAKE,
  TURN_CENTER_FORWARD,
  TURN_PRE_ROTATE_BRAKE,
  TURN_ROTATING,
  TURN_POST_ROTATE_BRAKE,
  TURN_POST_EXIT
};

TurnPhase turn_phase = TURN_IDLE;

enum TargetColor : uint8_t {
  TARGET_RED,
  TARGET_YELLOW,
  TARGET_BLUE
};

// =====================================================
// GLOBALS
// =====================================================

float x = 0, y = 0, yaw = 0;
float leg_target_yaw = 0.0f;
uint32_t red_sticker_count = 0;

int   last_ll = 0, last_lc = 0, last_lr = 0;
float last_h = 0, last_s = 0, last_v = 0;
float last_nr = 0, last_ng = 0, last_nb = 0;
float last_rgb_chroma = 0, last_hsv_chroma = 0;
bool  last_red_now    = false;
bool  last_yellow_now = false;
bool  last_blue_now   = false;

unsigned long lost_line_start_ms     = 0;
unsigned long marker_ignore_until_ms = 0;
TargetColor   last_marker_target     = TARGET_RED;
int           marker_stable_count    = 0;

unsigned long turn_phase_start_ms    = 0;
unsigned long turn_start_ms          = 0;
unsigned long last_turn_control_ms   = 0;
float         turn_start_yaw         = 0;
float         turn_target_yaw        = 0;
int           turn_settled_count     = 0;
float         pending_turn_angle     = 0;
bool          pending_turn_absolute  = false;
float         pending_absolute_yaw   = 0;
unsigned long pending_center_ms         = 0;
unsigned long pending_post_exit_ms      = 0;
unsigned long pending_marker_ignore_ms  = MARKER_IGNORE_AFTER_TURN_MS;

unsigned long workstation_wait_start_ms = 0;
unsigned long clear_start_ms            = 0;

bool ready_confirmed        = false;
unsigned long blue_confirm_start_ms = 0;
bool script_active          = false;
bool pause_requested        = false;
bool emergency_printed      = false;
bool mission_active         = false;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

void waitForStartState();
void execTokenState();
void driveToRedState();
void driveToYEntryState();
void driveToYWorkState();
void reverseToDockState();
void workstationWaitState();
void driveOutToYEntryState();
void driveToBlueState();
void doClearState();
void turnGenericState();

bool  driveForwardUntilColor(TargetColor t, float speed);
bool  reverseStraightUntilColor(TargetColor t);
void  followLine(int l, int c, int r, float base_speed);
float calculateCenterError(int l, int c, int r);
bool  isOnTape(int l, int c, int r);
bool  isIntersection(int l, int c, int r);

void  beginTurnToYaw(float yaw_target, RobotState next,
                     unsigned long ctr, unsigned long post, unsigned long ign);
void  finishTurn();
float normalizeYaw(float a);
float yawError(float tgt, float cur);
void  startRotateAbsolute(float tgt);
bool  updateRotateTo();

bool  targetColorDetectedStable(TargetColor t, bool r, bool y, bool b);
void  resetMarkerStable();
bool  isRed(float h, float s, float v);
bool  isYellow(float h, float s, float v, float nr, float ng, float nb, int l, int c, int r);
bool  isBlue(float h, float s, float v);
float colorChroma(float a, float b, float cc);
void  checkLineFailsafe(bool tape, bool r, bool y, bool b);

void  advanceToNextToken();
bool  loadScript(const String& seq);
bool  parseToken(const String& tok, TokenOp& out);
const char* tokenName(TokenOp op);
const char* stateName(RobotState s);
void  processStartConfirmation();
void  publishStatus(unsigned long now);
void  publishColor(unsigned long now);
void  cmdCallback(const void* msgin);
void  initTransport();
bool  initGraph();

void setLEDOff();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDYellow();

// =====================================================
// MAC-BASED IDENTITY
// =====================================================

int getAlvikID() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "3C:84:27:C2:87:50") return 1;
  if (mac == "3C:84:27:C3:E7:DC") return 2;
  if (mac == "74:4D:BD:A2:1B:70") return 3;
  if (mac == "48:CA:43:2E:1D:CC") return 4;
  return 1;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  alvik.begin();
  alvik.set_illuminator(true);
  alvik.reset_pose(0, 0, 0, CM, DEG);

  initTransport();
  snprintf(ROBOT_NAME, sizeof(ROBOT_NAME), "Alvik%d", getAlvikID());
  snprintf(T_STATUS, sizeof(T_STATUS), "%s_status", ROBOT_NAME);
  snprintf(T_CMD,    sizeof(T_CMD),    "%s_cmd",    ROBOT_NAME);
  snprintf(T_COLOR,  sizeof(T_COLOR),  "%s_color",  ROBOT_NAME);

  ros_ready = initGraph();
  setLEDBlue();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  if (ros_ready) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    publishStatus(now);
    publishColor(now);
  }

  if (alvik.get_touch_cancel() && robot_state != EMERGENCY_STOP) {
    alvik.brake();
    robot_state    = EMERGENCY_STOP;
    mission_active = false;
  }

  switch (robot_state) {
    case WAIT_FOR_START:      waitForStartState();        break;
    case EXEC_TOKEN:          execTokenState();           break;
    case DRIVE_TO_RED:        driveToRedState();          break;
    case DRIVE_TO_YENTRY:     driveToYEntryState();       break;
    case DRIVE_TO_YWORK:      driveToYWorkState();        break;
    case REVERSE_TO_DOCK:     reverseToDockState();       break;
    case WORKSTATION_WAIT:    workstationWaitState();     break;
    case DRIVE_OUT_TO_YENTRY: driveOutToYEntryState();    break;
    case DRIVE_TO_BLUE:       driveToBlueState();         break;
    case DO_CLEAR:            doClearState();             break;
    case TURN_GENERIC:        turnGenericState();         break;

    case PAUSED:
      alvik.brake();
      setLEDBlue();
      break;

    case DONE:
      alvik.brake();
      mission_active = false;
      setLEDGreen();
      break;

    case EMERGENCY_STOP:
      alvik.brake();
      setLEDRed();
      if (!emergency_printed) emergency_printed = true;
      break;
  }

  delay(LOOP_DELAY_MS);
}

// =====================================================
// WAIT FOR START
// =====================================================

void waitForStartState() {
  alvik.brake();
  processStartConfirmation();
}

void processStartConfirmation() {
  int l, c, r;
  float h, s, v;
  unsigned long now = millis();

  alvik.get_line_sensors(l, c, r);
  alvik.get_color(h, s, v, HSV);
  last_ll = l; last_lc = c; last_lr = r;
  last_h = h; last_s = s; last_v = v;
  last_blue_now = isBlue(h, s, v);

  bool on_tape = c > TAPE_THRESHOLD;

  if (on_tape && last_blue_now) {
    if (blue_confirm_start_ms == 0) blue_confirm_start_ms = now;
    if (!ready_confirmed && now - blue_confirm_start_ms >= START_CONFIRM_MS) {
      ready_confirmed = true;
      alvik.reset_pose(0, 0, 0, CM, DEG);
      float r0, p0, y0;
      alvik.get_orientation(r0, p0, y0);
      leg_target_yaw = y0;
      setLEDBlue();
    }
  } else {
    ready_confirmed       = false;
    blue_confirm_start_ms = 0;
    if (((now / 300) % 2) == 0) setLEDRed(); else setLEDOff();
  }
}

// =====================================================
// TOKEN EXECUTION
// =====================================================

void execTokenState() {
  if (!script_active || token_index >= token_count) {
    script_active  = false;
    mission_active = false;
    robot_state    = DONE;
    return;
  }

  if (pause_requested) {
    pause_requested = false;
    robot_state     = PAUSED;
    return;
  }

  TokenOp op = token_script[token_index++];

  switch (op) {
    case TOK_RED:
      robot_state = DRIVE_TO_RED;
      break;
    case TOK_YENTRY:
      marker_ignore_until_ms = millis() + YELLOW_ARM_AFTER_RED_MS;
      resetMarkerStable();
      robot_state = DRIVE_TO_YENTRY;
      break;
    case TOK_YWORK:
      robot_state = DRIVE_TO_YWORK;
      break;
    case TOK_DOCK:
      robot_state = REVERSE_TO_DOCK;
      break;
    case TOK_DWELL:
      workstation_wait_start_ms = millis();
      robot_state = WORKSTATION_WAIT;
      break;
    case TOK_EXIT:
      marker_ignore_until_ms = millis() + EXIT_WORKSTATION_IGNORE_MS;
      resetMarkerStable();
      robot_state = DRIVE_OUT_TO_YENTRY;
      break;
    case TOK_BLUE:
      robot_state = DRIVE_TO_BLUE;
      break;
    case TOK_R:
      beginTurnToYaw(leg_target_yaw - 90.0f,
                     EXEC_TOKEN, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                     MARKER_IGNORE_AFTER_TURN_MS);
      break;
    case TOK_L:
      beginTurnToYaw(leg_target_yaw + 90.0f,
                     EXEC_TOKEN, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                     MARKER_IGNORE_AFTER_TURN_MS);
      break;
    case TOK_YAW0:
      beginTurnToYaw(REVERSE_DOCK_ABSOLUTE_YAW,
                     EXEC_TOKEN, 0, 0,
                     YELLOW_IGNORE_AFTER_YAW_MS);
      break;
    case TOK_CLEAR:
      clear_start_ms         = millis();
      marker_ignore_until_ms = millis() + CLEAR_MARKER_MS + 150;
      resetMarkerStable();
      robot_state = DO_CLEAR;
      break;
    default:
      robot_state = EXEC_TOKEN;
      break;
  }
}

void advanceToNextToken() { robot_state = EXEC_TOKEN; }

// =====================================================
// DRIVE STATES
// =====================================================

void driveToRedState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_MS;
    resetMarkerStable();
    red_sticker_count++;
    advanceToNextToken();
  }
}

void driveToYEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) advanceToNextToken();
}

void driveToYWorkState() {
  if (driveForwardUntilColor(TARGET_YELLOW, WORKSTATION_APPROACH_SPEED)) advanceToNextToken();
}

void reverseToDockState() {
  if (reverseStraightUntilColor(TARGET_YELLOW)) advanceToNextToken();
}

void workstationWaitState() {
  alvik.brake();
  unsigned long elapsed = millis() - workstation_wait_start_ms;
  if (((elapsed / WORKSTATION_BLINK_MS) % 2) == 0) setLEDYellow(); else setLEDOff();
  if (elapsed >= WORKSTATION_WAIT_MS) {
    resetMarkerStable();
    setLEDGreen();
    advanceToNextToken();
  }
}

void driveOutToYEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) advanceToNextToken();
}

void driveToBlueState() {
  if (driveForwardUntilColor(TARGET_BLUE, BASE_SPEED)) advanceToNextToken();
}

void doClearState() {
  if (millis() - clear_start_ms >= CLEAR_MARKER_MS) {
    alvik.brake();
    advanceToNextToken();
    return;
  }
  setLEDGreen();
  alvik.set_wheels_speed(CLEAR_MARKER_SPEED, CLEAR_MARKER_SPEED, RPM);
}

// =====================================================
// DRIVE HELPERS
// =====================================================

bool driveForwardUntilColor(TargetColor target, float drive_speed) {
  int l, c, r;
  float h, s, v, nr, ng, nb;

  alvik.get_line_sensors(l, c, r);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);
  last_ll = l; last_lc = c; last_lr = r;
  last_h = h; last_s = s; last_v = v;
  last_nr = nr; last_ng = ng; last_nb = nb;
  last_rgb_chroma = colorChroma(nr, ng, nb);
  last_hsv_chroma = s * v;

  bool tape_now   = isOnTape(l, c, r);
  bool red_now    = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, l, c, r);
  bool blue_now   = isBlue(h, s, v);
  last_red_now    = red_now;
  last_yellow_now = yellow_now;
  last_blue_now   = blue_now;

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  followLine(l, c, r, drive_speed);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);
  return false;
}

bool reverseStraightUntilColor(TargetColor target) {
  int l, c, r;
  float h, s, v, nr, ng, nb;

  alvik.get_line_sensors(l, c, r);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);
  last_ll = l; last_lc = c; last_lr = r;
  last_h = h; last_s = s; last_v = v;
  last_nr = nr; last_ng = ng; last_nb = nb;
  last_rgb_chroma = colorChroma(nr, ng, nb);
  last_hsv_chroma = s * v;

  bool tape_now   = isOnTape(l, c, r);
  bool red_now    = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, l, c, r);
  bool blue_now   = isBlue(h, s, v);
  last_red_now    = red_now;
  last_yellow_now = yellow_now;
  last_blue_now   = blue_now;

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  alvik.set_wheels_speed(-REVERSE_DOCK_SPEED, -REVERSE_DOCK_SPEED, RPM);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);
  return false;
}

// =====================================================
// LINE FOLLOWING  — Arduino example style, pure P only
// =====================================================

void followLine(int l, int c, int r, float base_speed) {
  float error   = calculateCenterError(l, c, r);
  float control = error * KP;
  alvik.set_wheels_speed(base_speed - control, base_speed + control, RPM);
}

float calculateCenterError(int l, int c, int r) {
  float sum_weight = l + c + r;
  if (sum_weight <= 0.0f) return 0.0f;
  float centroid = (l + c * 2.0f + r * 3.0f) / sum_weight;
  return -(centroid - 2.0f);  // zero when centered; +ve = drift left, -ve = drift right
}

bool isOnTape(int l, int c, int r) {
  return (l > TAPE_THRESHOLD || c > TAPE_THRESHOLD || r > TAPE_THRESHOLD);
}

bool isIntersection(int l, int c, int r) {
  (void)c;
  return (l > TAPE_THRESHOLD && r > TAPE_THRESHOLD);
}

// =====================================================
// TURN CONTROL  — unchanged from AGV_MULTI_WS_DISPATCH
// =====================================================

void beginTurnToYaw(float yaw_target, RobotState next,
                    unsigned long ctr, unsigned long post, unsigned long ign) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute   = true;
  pending_absolute_yaw    = yaw_target;
  pending_turn_angle      = 0.0f;
  after_turn_state        = next;
  pending_center_ms       = ctr;
  pending_post_exit_ms    = post;
  pending_marker_ignore_ms = ign;

  turn_phase          = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state         = TURN_GENERIC;
  resetMarkerStable();
}

void turnGenericState() {
  unsigned long now = millis();

  switch (turn_phase) {
    case TURN_IDLE:
      turn_phase          = TURN_PRE_CENTER_BRAKE;
      turn_phase_start_ms = now;
      break;

    case TURN_PRE_CENTER_BRAKE:
      alvik.brake();
      setLEDYellow();
      if (now - turn_phase_start_ms >= PRE_CENTER_BRAKE_MS) {
        if (pending_center_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase          = TURN_CENTER_FORWARD;
          turn_phase_start_ms = now;
        } else {
          turn_phase          = TURN_PRE_ROTATE_BRAKE;
          turn_phase_start_ms = now;
        }
      }
      break;

    case TURN_CENTER_FORWARD:
      setLEDYellow();
      if (now - turn_phase_start_ms >= pending_center_ms) {
        alvik.brake();
        turn_phase          = TURN_PRE_ROTATE_BRAKE;
        turn_phase_start_ms = now;
      }
      break;

    case TURN_PRE_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();
      if (now - turn_phase_start_ms >= PRE_ROTATE_BRAKE_MS) {
        startRotateAbsolute(pending_absolute_yaw);
        turn_phase = TURN_ROTATING;
      }
      break;

    case TURN_ROTATING:
      setLEDYellow();
      if (updateRotateTo()) {
        turn_phase          = TURN_POST_ROTATE_BRAKE;
        turn_phase_start_ms = millis();
      }
      break;

    case TURN_POST_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();
      if (now - turn_phase_start_ms >= POST_ROTATE_BRAKE_MS) {
        if (pending_post_exit_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase          = TURN_POST_EXIT;
          turn_phase_start_ms = now;
        } else {
          finishTurn();
        }
      }
      break;

    case TURN_POST_EXIT: {
      setLEDYellow();
      int l, c, r;
      alvik.get_line_sensors(l, c, r);
      if (isOnTape(l, c, r) && !isIntersection(l, c, r)) {
        followLine(l, c, r, POST_TURN_EXIT_SPEED);
      } else {
        alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
      }
      if (now - turn_phase_start_ms >= pending_post_exit_ms) {
        alvik.brake();
        finishTurn();
      }
      break;
    }
  }
}

void finishTurn() {
  turn_phase             = TURN_IDLE;
  marker_ignore_until_ms = millis() + pending_marker_ignore_ms;
  float roll, pitch, imu_yaw;
  alvik.get_orientation(roll, pitch, imu_yaw);
  leg_target_yaw = imu_yaw;
  resetMarkerStable();
  robot_state = after_turn_state;
}

float normalizeYaw(float a) {
  a = fmod(a + 360.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

float yawError(float tgt, float cur) {
  float diff = normalizeYaw(tgt) - normalizeYaw(cur);
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  return diff;
}

void startRotateAbsolute(float tgt) {
  float roll, pitch;
  alvik.get_orientation(roll, pitch, yaw);
  turn_start_yaw       = yaw;
  turn_target_yaw      = normalizeYaw(tgt);
  turn_start_ms        = millis();
  last_turn_control_ms = 0;
  turn_settled_count   = 0;
}

bool updateRotateTo() {
  unsigned long now = millis();
  if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) return false;
  last_turn_control_ms = now;

  float roll, pitch;
  alvik.get_orientation(roll, pitch, yaw);
  float err = yawError(turn_target_yaw, normalizeYaw(yaw));

  if (fabsf(err) <= YAW_TOLERANCE) {
    turn_settled_count++;
    alvik.brake();
    if (turn_settled_count >= 3) { turn_settled_count = 0; return true; }
    return false;
  }
  turn_settled_count = 0;

  float scale = constrain(fabsf(err) / 90.0f, 0.0f, 1.0f);
  float spd   = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * scale;

  if (err > 0.0f) alvik.set_wheels_speed(-spd,  spd, RPM);
  else            alvik.set_wheels_speed( spd,  -spd, RPM);

  return false;
}

// =====================================================
// MARKER / COLOR DETECTION
// =====================================================

bool targetColorDetectedStable(TargetColor target, bool r, bool y, bool b) {
  if (millis() < marker_ignore_until_ms) { resetMarkerStable(); return false; }

  bool now_det = (target == TARGET_RED)    ? r :
                 (target == TARGET_YELLOW) ? y : b;
  int needed   = (target == TARGET_YELLOW) ? YELLOW_STABLE_SAMPLES :
                 (target == TARGET_BLUE)   ? BLUE_STABLE_SAMPLES   : RED_STABLE_SAMPLES;

  if (now_det) {
    if (last_marker_target != target) { marker_stable_count = 0; last_marker_target = target; }
    marker_stable_count++;
  } else {
    marker_stable_count = 0;
    last_marker_target  = target;
  }

  if (marker_stable_count >= needed) { resetMarkerStable(); return true; }
  return false;
}

void resetMarkerStable() { marker_stable_count = 0; }

bool isRed(float h, float s, float v) {
  return (h > 340.0f || h < 20.0f) && s > 0.40f && v > 0.03f;
}

float colorChroma(float a, float b, float cc) {
  float mx = a > b ? a : b; mx = mx > cc ? mx : cc;
  float mn = a < b ? a : b; mn = mn < cc ? mn : cc;
  return mx - mn;
}

bool isYellow(float h, float s, float v, float nr, float ng, float nb, int l, int c, int r) {
  float rgb_chroma = colorChroma(nr, ng, nb);
  float hsv_chroma = s * v;
  bool  tape_now   = isOnTape(l, c, r);

  bool false_positive =
    tape_now && h > 65.0f && h < 95.0f && s < 0.25f && v < 0.30f && rgb_chroma < 0.05f;
  if (false_positive) return false;

  return h > 28.0f && h < 55.0f && s > 0.42f && v > 0.07f && hsv_chroma > 0.045f;
}

bool isBlue(float h, float s, float v) {
  return h > 190.0f && h < 260.0f && s > 0.60f && v > 0.05f;
}

void checkLineFailsafe(bool tape, bool r, bool y, bool b) {
  if (!tape && !r && !y && !b) {
    if (lost_line_start_ms == 0) lost_line_start_ms = millis();
    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS) {
      alvik.brake();
      robot_state = EMERGENCY_STOP;
    }
  } else {
    lost_line_start_ms = 0;
  }
}

// =====================================================
// SCRIPT LOADING
// =====================================================

bool parseToken(const String& tok, TokenOp& out) {
  String t = tok;
  t.trim(); t.toUpperCase();
  if (t == "RED")    { out = TOK_RED;    return true; }
  if (t == "YENTRY") { out = TOK_YENTRY; return true; }
  if (t == "YWORK")  { out = TOK_YWORK;  return true; }
  if (t == "DOCK")   { out = TOK_DOCK;   return true; }
  if (t == "DWELL" || t == "DWELL5") { out = TOK_DWELL; return true; }
  if (t == "EXIT")   { out = TOK_EXIT;   return true; }
  if (t == "BLUE")   { out = TOK_BLUE;   return true; }
  if (t == "R")      { out = TOK_R;      return true; }
  if (t == "L")      { out = TOK_L;      return true; }
  if (t == "YAW0")   { out = TOK_YAW0;   return true; }
  if (t == "CLEAR")  { out = TOK_CLEAR;  return true; }
  return false;
}

bool loadScript(const String& seq) {
  token_count = 0;
  token_index = 0;

  String s = seq;
  s.replace(';', ',');
  s.replace(' ', ',');

  int start = 0;
  while (start <= (int)s.length()) {
    int comma = s.indexOf(',', start);
    String tok;
    if (comma < 0) { tok = s.substring(start); start = s.length() + 1; }
    else           { tok = s.substring(start, comma); start = comma + 1; }
    tok.trim();
    if (tok.length() == 0) continue;
    if (token_count >= MAX_TOKENS) break;
    TokenOp op;
    if (!parseToken(tok, op)) return false;
    token_script[token_count++] = op;
  }

  return token_count > 0;
}

// =====================================================
// MICRO-ROS CALLBACKS
// =====================================================

void cmdCallback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  String cmd = "";
  for (size_t i = 0; i < msg->data.size; i++) cmd += msg->data.data[i];
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("reset")) {
    alvik.brake();
    script_active         = false;
    mission_active        = false;
    pause_requested       = false;
    turn_phase            = TURN_IDLE;
    robot_state           = WAIT_FOR_START;
    ready_confirmed       = false;
    blue_confirm_start_ms = 0;
    token_count           = 0;
    token_index           = 0;
    emergency_printed     = false;
    return;
  }

  if (cmd.equalsIgnoreCase("pause")) { pause_requested = true; return; }

  if (cmd.equalsIgnoreCase("resume")) {
    if (robot_state == PAUSED) { pause_requested = false; robot_state = EXEC_TOKEN; }
    return;
  }

  if (cmd.length() > 7 && cmd.substring(0, 7).equalsIgnoreCase("append ")) {
    String seq = cmd.substring(7);
    seq.trim(); seq.replace(';', ','); seq.replace(' ', ',');
    int start = 0;
    while (start <= (int)seq.length()) {
      int comma = seq.indexOf(',', start);
      String tok;
      if (comma < 0) { tok = seq.substring(start); start = seq.length() + 1; }
      else           { tok = seq.substring(start, comma); start = comma + 1; }
      tok.trim();
      if (tok.length() == 0) continue;
      if (token_count >= MAX_TOKENS) break;
      TokenOp op;
      if (!parseToken(tok, op)) return;
      token_script[token_count++] = op;
    }
    return;
  }

  bool idle = (robot_state == WAIT_FOR_START || robot_state == DONE || robot_state == PAUSED);

  if (cmd.length() > 4 && cmd.substring(0, 4).equalsIgnoreCase("run ")) {
    if (!idle) return;
    String seq = cmd.substring(4);
    if (!loadScript(seq)) return;
    if (!ready_confirmed) return;

    script_active           = true;
    mission_active          = true;
    pause_requested         = false;
    emergency_printed       = false;
    lost_line_start_ms      = 0;
    red_sticker_count       = 0;
    marker_ignore_until_ms  = millis() + 500;
    resetMarkerStable();
    turn_phase     = TURN_IDLE;
    alvik.reset_pose(0, 0, 0, CM, DEG);
    leg_target_yaw = 0.0f;

    robot_state = EXEC_TOKEN;
    return;
  }
}

// =====================================================
// TELEMETRY
// =====================================================

void publishStatus(unsigned long now) {
  if (now - last_status_ms < STATUS_PERIOD_MS) return;
  last_status_ms = now;

  alvik.get_pose(x, y, yaw, CM, DEG);

  const char* cur_tok = (script_active && token_index > 0 && token_index <= token_count)
    ? tokenName(token_script[token_index - 1]) : "";

  static char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"step\":%u,\"total\":%u,\"token\":\"%s\","
    "\"ready\":%d,\"active\":%d,"
    "\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,"
    "\"red_now\":%d,\"yellow_now\":%d,\"blue_now\":%d,"
    "\"ros\":%d,\"ms\":%lu}",
    stateName(robot_state), token_index, token_count, cur_tok,
    ready_confirmed ? 1 : 0, mission_active ? 1 : 0,
    x, y, yaw,
    last_red_now ? 1 : 0, last_yellow_now ? 1 : 0, last_blue_now ? 1 : 0,
    ros_ready ? 1 : 0, now);

  msg_status.data.data     = buf;
  msg_status.data.size     = strlen(buf);
  msg_status.data.capacity = sizeof(buf);
  rcl_publish(&pub_status, &msg_status, NULL);
}

void publishColor(unsigned long now) {
  if (now - last_color_ms < STATUS_PERIOD_MS) return;
  last_color_ms = now;

  const char* label = last_red_now    ? "RED"
                    : last_yellow_now ? "YELLOW"
                    : last_blue_now   ? "BLUE"
                    : "---";

  float bottom_dist = alvik.get_distance_bottom(CM);

  static char cbuf[256];
  snprintf(cbuf, sizeof(cbuf),
    "{\"color\":\"%s\",\"h\":%.1f,\"s\":%.3f,\"v\":%.3f,"
    "\"L\":%d,\"C\":%d,\"R\":%d,\"red_count\":%lu,"
    "\"yaw\":%.1f,\"tgt\":%.1f,\"bot\":%.1f}",
    label, last_h, last_s, last_v,
    last_ll, last_lc, last_lr,
    (unsigned long)red_sticker_count,
    yaw, leg_target_yaw, bottom_dist);

  msg_color.data.data     = cbuf;
  msg_color.data.size     = strlen(cbuf);
  msg_color.data.capacity = sizeof(cbuf);
  rcl_publish(&pub_color, &msg_color, NULL);
}

const char* tokenName(TokenOp op) {
  switch (op) {
    case TOK_RED:    return "RED";
    case TOK_YENTRY: return "YENTRY";
    case TOK_YWORK:  return "YWORK";
    case TOK_DOCK:   return "DOCK";
    case TOK_DWELL:  return "DWELL";
    case TOK_EXIT:   return "EXIT";
    case TOK_BLUE:   return "BLUE";
    case TOK_R:      return "R";
    case TOK_L:      return "L";
    case TOK_YAW0:   return "YAW0";
    case TOK_CLEAR:  return "CLEAR";
    default:         return "UNKNOWN";
  }
}

const char* stateName(RobotState s) {
  switch (s) {
    case WAIT_FOR_START:      return ready_confirmed ? "IDLE" : "NOT_READY";
    case EXEC_TOKEN:          return "MOVING";
    case DRIVE_TO_RED:        return "MOVING";
    case DRIVE_TO_YENTRY:     return "MOVING";
    case DRIVE_TO_YWORK:      return "MOVING";
    case REVERSE_TO_DOCK:     return "MOVING";
    case WORKSTATION_WAIT:    return "DWELL";
    case DRIVE_OUT_TO_YENTRY: return "MOVING";
    case DRIVE_TO_BLUE:       return "MOVING";
    case DO_CLEAR:            return "MOVING";
    case TURN_GENERIC:        return "MOVING";
    case PAUSED:              return "PAUSED";
    case DONE:                return "ARRIVED";
    case EMERGENCY_STOP:      return "EMERGENCY_STOP";
    default:                  return "UNKNOWN";
  }
}

// =====================================================
// MICRO-ROS SETUP
// =====================================================

void initTransport() {
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    setLEDYellow(); delay(250); setLEDOff(); delay(250);
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

  if (rclc_publisher_init_default(
        &pub_color, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_COLOR) != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_CMD) != RCL_RET_OK) return false;

  msg_cmd_in.data.data     = cmd_buf;
  msg_cmd_in.data.size     = 0;
  msg_cmd_in.data.capacity = sizeof(cmd_buf);

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(
        &executor, &sub_cmd, &msg_cmd_in,
        &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  msg_status.data.data     = NULL;
  msg_status.data.size     = 0;
  msg_status.data.capacity = 0;

  msg_color.data.data      = NULL;
  msg_color.data.size      = 0;
  msg_color.data.capacity  = 0;
  return true;
}

// =====================================================
// LED HELPERS
// =====================================================

void setLEDOff()    { alvik.left_led.set_color(0,0,0); alvik.right_led.set_color(0,0,0); }
void setLEDRed()    { alvik.left_led.set_color(1,0,0); alvik.right_led.set_color(1,0,0); }
void setLEDGreen()  { alvik.left_led.set_color(0,1,0); alvik.right_led.set_color(0,1,0); }
void setLEDBlue()   { alvik.left_led.set_color(0,0,1); alvik.right_led.set_color(0,0,1); }
void setLEDYellow() { alvik.left_led.set_color(1,1,0); alvik.right_led.set_color(1,1,0); }
