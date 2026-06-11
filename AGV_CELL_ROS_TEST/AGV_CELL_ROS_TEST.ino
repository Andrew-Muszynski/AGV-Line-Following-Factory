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

// =====================================================
// MICRO-ROS CONFIGURATION
// =====================================================

char WIFI_SSID[]     = "AGV_SWARM";
char WIFI_PASSWORD[] = "ISECap123";
char AGENT_IP[]      = "192.168.1.141";
const uint32_t AGENT_PORT = 8888;

char ROBOT_NAME[16] = "Alvik1";
char T_STATUS[32];
char T_CMD[32];

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t pub_status;
rcl_subscription_t sub_cmd;
std_msgs__msg__String msg_status;
char cmd_buf[512];
std_msgs__msg__String msg_cmd_in;

bool ros_ready = false;

// =====================================================
// TUNING VALUES
// =====================================================

// Black tape threshold
const int TAPE_THRESHOLD = 250;

// Line following. These are the same values that worked on your perimeter test.
const float BASE_SPEED = 40.0;                  // Normal wheel speed in RPM
const float YELLOW_SEARCH_SPEED = 40.0;         // Slower speed when the next target is yellow
const float WORKSTATION_APPROACH_SPEED = 28.0;  // Slower speed from entry yellow to workstation yellow
const float KP = 28.0;                          // Line correction gain
const float MAX_CORRECTION = 18.0;              // Prevents excessive wheel speed difference

// If tape is temporarily hidden by a sticker, drive straight at this speed
const float STICKER_CROSS_SPEED = 24.0;
const float LINE_RECOVERY_FORWARD_SPEED = 18.0;
const float LINE_RECOVERY_TURN_SPEED = 30.0;
const unsigned long LINE_RECOVERY_GRACE_MS = 900;

// Slow reverse used to back into the workstation marker after yaw correction
const float REVERSE_DOCK_SPEED = 16.0;

// Turn commands. If right turns go the wrong way, change RIGHT_TURN_DEG to +90.0.
const float RIGHT_TURN_DEG = -90.0;

// Absolute yaw used before reverse docking.
// The robot starts facing NORTH at yaw = 0.0.
const float REVERSE_DOCK_ABSOLUTE_YAW = 0.0;

// RotateTo-style yaw control
const float YAW_TOLERANCE = 2.0;            // degrees
const float TURN_MIN_SPEED = 18.0;          // RPM
const float TURN_MAX_SPEED = 40.0;          // RPM
const unsigned long TURN_CONTROL_MS = 5;    // update period
const float ADAPTIVE_TURN_MIN_DEG = 70.0;   // start trusting line reacquisition after this much turn
const float ADAPTIVE_TURN_EXTRA_DEG = 12.0; // search this far past nominal yaw if line is not found
const float ADAPTIVE_TURN_SEARCH_SPEED = 16.0;
const int ADAPTIVE_TURN_SIDE_THRESHOLD = 200;
const int ADAPTIVE_TURN_CENTER_THRESHOLD = 500;
const int ADAPTIVE_TURN_LINE_STABLE_SAMPLES = 4;
const float ARC_TURN_INNER_SPEED = 0.0;
const float ARC_TURN_OUTER_SPEED = 70.0;
const float ARC_TURN_SEARCH_INNER_SPEED = 0.0;
const float ARC_TURN_SEARCH_OUTER_SPEED = 45.0;

// Turn centering and exit motions
const unsigned long TURN_CENTERING_MS = 120;
const unsigned long PRE_CENTER_BRAKE_MS = 80;
const unsigned long PRE_ROTATE_BRAKE_MS = 100;
const unsigned long POST_ROTATE_BRAKE_MS = 100;

// General post-turn exit
const unsigned long POST_TURN_EXIT_MS = 120;

// Shorter post-turn exit when the next target is yellow.
const unsigned long YELLOW_POST_TURN_EXIT_MS = 60;

const float POST_TURN_EXIT_SPEED = 45.0;

// Marker gating
const int MARKER_STABLE_SAMPLES = 2;
const int YELLOW_STABLE_SAMPLES = 1;

const unsigned long MARKER_IGNORE_AFTER_TURN_MS = 600;
const unsigned long NO_MARKER_IGNORE_MS = 0;
const unsigned long YELLOW_ENTRY_DEPART_IGNORE_MS = 250;
const unsigned long YELLOW_IGNORE_AFTER_YAW_ALIGN_MS = 300;
const unsigned long EXIT_WORKSTATION_IGNORE_MS = 1000;

// Workstation wait
const unsigned long WORKSTATION_WAIT_MS = 5000;
const unsigned long WORKSTATION_BLINK_MS = 250;
const unsigned long PROCESSING_WAIT_MS = 30000;

// Line-loss failsafe
const unsigned long LOST_LINE_FAILSAFE_MS = 1500;

// Keep the same effective loop pacing that worked well on the tape
const unsigned long LOOP_DELAY_MS = 10;

// Debug print interval while searching for yellow
const unsigned long YELLOW_DEBUG_PRINT_MS = 200;

// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState {
  WAIT_FOR_START,
  PROCESSING_WAIT,
  SCRIPT_ADVANCE,
  SCRIPT_CLEAR_MARKER,

  DRIVE_TO_RED_1,
  DRIVE_TO_YELLOW_ENTRY,
  DRIVE_TO_YELLOW_WORKSTATION,
  REVERSE_TO_WORKSTATION_YELLOW,
  WORKSTATION_WAIT,
  DRIVE_OUT_TO_YELLOW_ENTRY,
  DRIVE_TO_RED_2,
  DRIVE_TO_RED_3,
  DRIVE_TO_BLUE,

  TURN_GENERIC,
  DONE,
  EMERGENCY_STOP
};

RobotState robot_state = WAIT_FOR_START;
RobotState after_turn_state = DONE;

enum MissionMode {
  MISSION_NONE,
  MISSION_SINGLE_PASS,
  MISSION_STATION_A_CYCLE
};

enum MissionPhase {
  PHASE_IDLE,
  PHASE_DELIVERING,
  PHASE_PROCESSING,
  PHASE_PICKING_UP,
  PHASE_COMPLETE
};

enum ScriptOp {
  OP_RED,
  OP_YENTRY,
  OP_YWORK,
  OP_EXIT,
  OP_BLUE,
  OP_DOCK,
  OP_DWELL5,
  OP_WAIT30,
  OP_CLEAR,
  OP_R,
  OP_L,
  OP_R_YSEARCH,
  OP_R_SPUR,
  OP_R_FINAL,
  OP_YAW0
};

MissionMode mission_mode = MISSION_NONE;
MissionPhase mission_phase = PHASE_IDLE;
unsigned long processing_wait_start_ms = 0;
unsigned long last_status_ms = 0;
bool ready_confirmed = false;
unsigned long blue_confirm_start_ms = 0;
const unsigned long START_CONFIRM_MS = 1500;
const unsigned long CLEAR_MARKER_MS = 500;
const float CLEAR_MARKER_SPEED = 28.0;

#define MAX_SCRIPT_OPS 80
ScriptOp script_ops[MAX_SCRIPT_OPS];
int script_len = 0;
int script_idx = 0;
bool script_active = false;
unsigned long script_wait_duration_ms = 0;
unsigned long script_clear_start_ms = 0;

enum TurnPhase {
  TURN_IDLE,
  TURN_PRE_CENTER_BRAKE,
  TURN_CENTER_FORWARD,
  TURN_PRE_ROTATE_BRAKE,
  TURN_ROTATING,
  TURN_POST_ROTATE_BRAKE,
  TURN_POST_EXIT
};

TurnPhase turn_phase = TURN_IDLE;

enum TargetColor {
  TARGET_RED,
  TARGET_YELLOW,
  TARGET_BLUE
};

// =====================================================
// GLOBAL VARIABLES
// =====================================================

float x, y, yaw;
float last_line_error = 0.0;

int red_count = 0;
bool emergency_printed = false;

unsigned long lost_line_start_ms = 0;
unsigned long marker_ignore_until_ms = 0;

// Marker stability
TargetColor last_marker_target = TARGET_RED;
int marker_stable_count = 0;

// Turn timing/control
unsigned long turn_phase_start_ms = 0;
unsigned long turn_start_ms = 0;
unsigned long last_turn_control_ms = 0;
int adaptive_turn_line_stable_count = 0;
bool adaptive_turn_searching = false;
bool adaptive_turn_saw_gap = false;
bool adaptive_turn_failed = false;

float turn_start_yaw = 0.0;
float turn_target_yaw = 0.0;
float pending_turn_angle = 0.0;

// Absolute-turn support
bool pending_turn_absolute = false;
float pending_absolute_yaw = 0.0;

unsigned long pending_center_ms = 0;
unsigned long pending_post_exit_ms = 0;
unsigned long pending_marker_ignore_ms = MARKER_IGNORE_AFTER_TURN_MS;
const char* pending_turn_label = "turn";

// Workstation wait/blink
unsigned long workstation_wait_start_ms = 0;

// Yellow debug
unsigned long last_yellow_debug_ms = 0;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

void waitForStartState();
void processingWaitState();
void driveToRed1State();
void driveToYellowEntryState();
void driveToYellowWorkstationState();
void reverseToWorkstationYellowState();
void workstationWaitState();
void driveOutToYellowEntryState();
void driveToRed2State();
void driveToRed3State();
void driveToBlueState();

bool driveForwardUntilColor(TargetColor target, const char* label, float drive_speed);
bool reverseStraightUntilColor(TargetColor target, const char* label);
void followLineOrDriveStraight(int left, int center, int right, float base_speed);
float calculateCenterError(int left, int center, int right);
bool isOnTape(int left, int center, int right);

void beginTurn(float angle, RobotState next_state, unsigned long center_ms,
               unsigned long post_exit_ms, unsigned long marker_ignore_ms,
               const char* label);
void beginTurnToYaw(float target_yaw, RobotState next_state, unsigned long center_ms,
                    unsigned long post_exit_ms, unsigned long marker_ignore_ms,
                    const char* label);
void turnGenericState();
void finishTurn();
float normalizeYaw(float angle);
float yawError(float target, float current);
void startRotateRelative(float relativeAngle);
void startRotateAbsolute(float targetYaw);
bool updateRotateTo();
void startArcTurnRelative(float relativeAngle);
bool updateArcTurnToLine();

bool targetColorDetectedStable(TargetColor target, bool red_now, bool yellow_now, bool blue_now);
void resetMarkerStable();
void printDetectedColor(const char* label, float h, float s, float v);
void printYellowDebug(float h, float s, float v, bool yellow_now);
bool isRed(float h, float s, float v);
bool isYellow(float h, float s, float v);
bool isBlue(float h, float s, float v);

void checkLineFailsafe(bool tape_now, bool red_now, bool yellow_now, bool blue_now);
void setLEDOff();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDYellow();
void printStateName(RobotState state);
void startMission(MissionMode mode);
void resetRouteRun();
void handleMissionDone();
void advanceScript();
void scriptClearMarkerState();
bool loadStationAPassScript();
bool loadStationACycleScript();
bool loadScriptFromText(String seq);
bool appendScriptOp(ScriptOp op);
bool tokenToScriptOp(String token, ScriptOp& op);
void startLoadedScript(MissionMode mode);
const char* scriptOpName(ScriptOp op);
const char* stateName(RobotState state);
const char* missionPhaseName(MissionPhase phase);

void initTransport();
bool initGraph();
void cmdCallback(const void* msgin);
void publishStatus(unsigned long now);
void processStartConfirmation();

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  alvik.begin();
  alvik.reset_pose(0, 0, 0, CM, DEG);
  alvik.set_illuminator(true);

  snprintf(T_STATUS, sizeof(T_STATUS), "%s_status", ROBOT_NAME);
  snprintf(T_CMD, sizeof(T_CMD), "%s_cmd", ROBOT_NAME);

  initTransport();
  ros_ready = initGraph();
  ready_confirmed = false;
  blue_confirm_start_ms = 0;

  Serial.println("Alvik workstation route with ROS2 command start");
  Serial.println("Start on the blue sticker facing NORTH.");
  Serial.println("ROS command: station_a_cycle");
  Serial.println("Manual fallback: press OK for one station pass.");
  Serial.println();

  setLEDBlue();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  if (ros_ready) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    publishStatus(now);
  }

  if (alvik.get_touch_cancel() && robot_state != EMERGENCY_STOP) {
    alvik.brake();
    robot_state = EMERGENCY_STOP;
    mission_phase = PHASE_IDLE;
  }

  switch (robot_state) {
    case WAIT_FOR_START:
      waitForStartState();
      break;

    case PROCESSING_WAIT:
      processingWaitState();
      break;

    case SCRIPT_ADVANCE:
      advanceScript();
      break;

    case SCRIPT_CLEAR_MARKER:
      scriptClearMarkerState();
      break;

    case DRIVE_TO_RED_1:
      driveToRed1State();
      break;

    case DRIVE_TO_YELLOW_ENTRY:
      driveToYellowEntryState();
      break;

    case DRIVE_TO_YELLOW_WORKSTATION:
      driveToYellowWorkstationState();
      break;

    case REVERSE_TO_WORKSTATION_YELLOW:
      reverseToWorkstationYellowState();
      break;

    case WORKSTATION_WAIT:
      workstationWaitState();
      break;

    case DRIVE_OUT_TO_YELLOW_ENTRY:
      driveOutToYellowEntryState();
      break;

    case DRIVE_TO_RED_2:
      driveToRed2State();
      break;

    case DRIVE_TO_RED_3:
      driveToRed3State();
      break;

    case DRIVE_TO_BLUE:
      driveToBlueState();
      break;

    case TURN_GENERIC:
      turnGenericState();
      break;

    case DONE:
      alvik.brake();
      handleMissionDone();
      break;

    case EMERGENCY_STOP:
      alvik.brake();
      setLEDRed();

      if (!emergency_printed) {
        Serial.println("Emergency stop.");
        emergency_printed = true;
      }
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

  if (!ready_confirmed) {
    return;
  }

  if (alvik.get_touch_ok()) {
    startMission(MISSION_SINGLE_PASS);
  }
}

void processStartConfirmation() {
  int left, center, right;
  float h, s, v;
  unsigned long now = millis();

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);

  bool on_center_tape = center > TAPE_THRESHOLD;
  bool blue_now = isBlue(h, s, v);

  if (on_center_tape && blue_now) {
    if (blue_confirm_start_ms == 0) {
      blue_confirm_start_ms = now;
    }

    if (!ready_confirmed && now - blue_confirm_start_ms >= START_CONFIRM_MS) {
      ready_confirmed = true;
      alvik.reset_pose(0, 0, 0, CM, DEG);
      setLEDBlue();
      Serial.println("Blue start marker confirmed. AGV is IDLE and ready for ROS command.");
    }
  } else {
    ready_confirmed = false;
    blue_confirm_start_ms = 0;
    if (((now / 300) % 2) == 0) {
      setLEDRed();
    } else {
      setLEDOff();
    }
  }
}

void startMission(MissionMode mode) {
  bool loaded = (mode == MISSION_STATION_A_CYCLE) ? loadStationACycleScript() : loadStationAPassScript();
  if (!loaded) {
    Serial.println("Could not load mission script.");
    return;
  }

  startLoadedScript(mode);
}

void resetRouteRun() {
  red_count = 0;
  emergency_printed = false;
  lost_line_start_ms = 0;
  marker_ignore_until_ms = 0;
  processing_wait_start_ms = 0;
  turn_phase = TURN_IDLE;
  after_turn_state = DONE;
  resetMarkerStable();

  alvik.reset_pose(0, 0, 0, CM, DEG);
  setLEDGreen();
}

void processingWaitState() {
  alvik.brake();

  unsigned long now = millis();
  unsigned long elapsed = now - processing_wait_start_ms;

  if (((elapsed / 500) % 2) == 0) {
    setLEDBlue();
  } else {
    setLEDYellow();
  }

  if (elapsed >= script_wait_duration_ms) {
    mission_phase = PHASE_PICKING_UP;
    Serial.println("Processing wait complete. Advancing script.");
    advanceScript();
  }
}

void handleMissionDone() {
  script_active = false;
  mission_phase = PHASE_COMPLETE;
  mission_mode = MISSION_NONE;
  setLEDGreen();
}

void scriptClearMarkerState() {
  if (millis() - script_clear_start_ms >= CLEAR_MARKER_MS) {
    alvik.brake();
    advanceScript();
    return;
  }

  setLEDGreen();
  alvik.set_wheels_speed(CLEAR_MARKER_SPEED, CLEAR_MARKER_SPEED, RPM);
}

bool appendScriptOp(ScriptOp op) {
  if (script_len >= MAX_SCRIPT_OPS) {
    return false;
  }

  script_ops[script_len++] = op;
  return true;
}

bool loadStationAPassScript() {
  script_len = 0;
  return appendScriptOp(OP_RED) &&
         appendScriptOp(OP_R_YSEARCH) &&
         appendScriptOp(OP_YENTRY) &&
         appendScriptOp(OP_R_SPUR) &&
         appendScriptOp(OP_YWORK) &&
         appendScriptOp(OP_YAW0) &&
         appendScriptOp(OP_DOCK) &&
         appendScriptOp(OP_DWELL5) &&
         appendScriptOp(OP_EXIT) &&
         appendScriptOp(OP_R) &&
         appendScriptOp(OP_RED) &&
         appendScriptOp(OP_R) &&
         appendScriptOp(OP_RED) &&
         appendScriptOp(OP_R) &&
         appendScriptOp(OP_BLUE) &&
         appendScriptOp(OP_R_FINAL);
}

bool loadStationACycleScript() {
  script_len = 0;
  if (!loadStationAPassScript()) return false;
  if (!appendScriptOp(OP_WAIT30)) return false;

  ScriptOp pass_ops[] = {
    OP_RED, OP_R_YSEARCH, OP_YENTRY, OP_R_SPUR, OP_YWORK, OP_YAW0, OP_DOCK, OP_DWELL5,
    OP_EXIT, OP_R, OP_RED, OP_R, OP_RED, OP_R, OP_BLUE, OP_R_FINAL
  };

  for (int i = 0; i < 16; i++) {
    if (!appendScriptOp(pass_ops[i])) return false;
  }

  return true;
}

bool tokenToScriptOp(String token, ScriptOp& op) {
  token.trim();
  token.toUpperCase();

  if (token == "RED") op = OP_RED;
  else if (token == "YENTRY") op = OP_YENTRY;
  else if (token == "YWORK") op = OP_YWORK;
  else if (token == "EXIT") op = OP_EXIT;
  else if (token == "BLUE") op = OP_BLUE;
  else if (token == "DOCK") op = OP_DOCK;
  else if (token == "DWELL5") op = OP_DWELL5;
  else if (token == "WAIT30") op = OP_WAIT30;
  else if (token == "CLEAR") op = OP_CLEAR;
  else if (token == "R") op = OP_R;
  else if (token == "L") op = OP_L;
  else if (token == "R_YSEARCH") op = OP_R_YSEARCH;
  else if (token == "R_SPUR") op = OP_R_SPUR;
  else if (token == "R_FINAL") op = OP_R_FINAL;
  else if (token == "YAW0") op = OP_YAW0;
  else return false;

  return true;
}

bool loadScriptFromText(String seq) {
  script_len = 0;
  int start = 0;

  while (start < seq.length()) {
    int comma = seq.indexOf(',', start);
    int space = seq.indexOf(' ', start);
    int end = -1;

    if (comma < 0) end = space;
    else if (space < 0) end = comma;
    else end = min(comma, space);

    if (end < 0) end = seq.length();

    String token = seq.substring(start, end);
    token.trim();
    if (token.length() > 0) {
      ScriptOp op;
      if (!tokenToScriptOp(token, op) || !appendScriptOp(op)) {
        script_len = 0;
        return false;
      }
    }

    start = end + 1;
  }

  return script_len > 0;
}

void startLoadedScript(MissionMode mode) {
  mission_mode = mode;
  mission_phase = (mode == MISSION_STATION_A_CYCLE) ? PHASE_DELIVERING : PHASE_PICKING_UP;
  script_idx = 0;
  script_active = true;
  resetRouteRun();

  Serial.print("Starting loaded primitive script with ");
  Serial.print(script_len);
  Serial.println(" steps.");

  advanceScript();
}

void advanceScript() {
  if (!script_active || script_idx >= script_len) {
    robot_state = DONE;
    return;
  }

  ScriptOp op = script_ops[script_idx++];
  Serial.print("Script step ");
  Serial.print(script_idx);
  Serial.print("/");
  Serial.print(script_len);
  Serial.print(": ");
  Serial.println(scriptOpName(op));

  switch (op) {
    case OP_RED:
      robot_state = DRIVE_TO_RED_1;
      break;

    case OP_YENTRY:
      robot_state = DRIVE_TO_YELLOW_ENTRY;
      break;

    case OP_YWORK:
      robot_state = DRIVE_TO_YELLOW_WORKSTATION;
      break;

    case OP_EXIT:
      marker_ignore_until_ms = millis() + EXIT_WORKSTATION_IGNORE_MS;
      resetMarkerStable();
      robot_state = DRIVE_OUT_TO_YELLOW_ENTRY;
      break;

    case OP_BLUE:
      robot_state = DRIVE_TO_BLUE;
      break;

    case OP_DOCK:
      robot_state = REVERSE_TO_WORKSTATION_YELLOW;
      break;

    case OP_DWELL5:
      workstation_wait_start_ms = millis();
      robot_state = WORKSTATION_WAIT;
      break;

    case OP_WAIT30:
      mission_phase = PHASE_PROCESSING;
      processing_wait_start_ms = millis();
      script_wait_duration_ms = PROCESSING_WAIT_MS;
      robot_state = PROCESSING_WAIT;
      break;

    case OP_CLEAR:
      script_clear_start_ms = millis();
      marker_ignore_until_ms = millis() + CLEAR_MARKER_MS + 150;
      resetMarkerStable();
      alvik.set_wheels_speed(CLEAR_MARKER_SPEED, CLEAR_MARKER_SPEED, RPM);
      robot_state = SCRIPT_CLEAR_MARKER;
      break;

    case OP_R:
      beginTurn(RIGHT_TURN_DEG, SCRIPT_ADVANCE, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                MARKER_IGNORE_AFTER_TURN_MS, "script right turn");
      break;

    case OP_L:
      beginTurn(-RIGHT_TURN_DEG, SCRIPT_ADVANCE, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                MARKER_IGNORE_AFTER_TURN_MS, "script left turn");
      break;

    case OP_R_YSEARCH:
      beginTurn(RIGHT_TURN_DEG, SCRIPT_ADVANCE, TURN_CENTERING_MS, YELLOW_POST_TURN_EXIT_MS,
                NO_MARKER_IGNORE_MS, "script right turn toward yellow");
      break;

    case OP_R_SPUR:
      beginTurn(RIGHT_TURN_DEG, SCRIPT_ADVANCE, TURN_CENTERING_MS, YELLOW_POST_TURN_EXIT_MS,
                YELLOW_ENTRY_DEPART_IGNORE_MS, "script right turn into workstation spur");
      break;

    case OP_R_FINAL:
      beginTurn(RIGHT_TURN_DEG, SCRIPT_ADVANCE, 0, 0,
                MARKER_IGNORE_AFTER_TURN_MS, "script final right turn");
      break;

    case OP_YAW0:
      beginTurnToYaw(REVERSE_DOCK_ABSOLUTE_YAW, SCRIPT_ADVANCE, 0, 0,
                     YELLOW_IGNORE_AFTER_YAW_ALIGN_MS, "script align yaw 0");
      break;
  }
}

// =====================================================
// ROUTE STATES
// =====================================================

void driveToRed1State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 1: first red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED marker. Primitive complete.");
    advanceScript();
  }
}

void driveToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW ENTRY: first yellow marker", YELLOW_SEARCH_SPEED)) {
    Serial.println("At entry YELLOW. Primitive complete.");
    advanceScript();
  }
}

void driveToYellowWorkstationState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW WORKSTATION: second yellow marker",
                             WORKSTATION_APPROACH_SPEED)) {
    Serial.println("At workstation YELLOW. Primitive complete.");
    advanceScript();
  }
}

void reverseToWorkstationYellowState() {
  if (reverseStraightUntilColor(TARGET_YELLOW, "workstation yellow docking marker")) {
    Serial.println("Docked at workstation yellow. Primitive complete.");
    advanceScript();
  }
}

void workstationWaitState() {
  alvik.brake();

  unsigned long now = millis();
  unsigned long elapsed = now - workstation_wait_start_ms;

  if (((elapsed / WORKSTATION_BLINK_MS) % 2) == 0) {
    setLEDYellow();
  } else {
    setLEDOff();
  }

  if (elapsed >= WORKSTATION_WAIT_MS) {
    Serial.println("Workstation dwell complete. Primitive complete.");
    advanceScript();
  }
}

void driveOutToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW ENTRY: returning from workstation", YELLOW_SEARCH_SPEED)) {
    Serial.println("Back at entry YELLOW. Primitive complete.");
    advanceScript();
  }
}

void driveToRed2State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 2: top-right red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED marker. Primitive complete.");
    advanceScript();
  }
}

void driveToRed3State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 3: bottom-right red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED marker. Primitive complete.");
    advanceScript();
  }
}

void driveToBlueState() {
  if (driveForwardUntilColor(TARGET_BLUE, "BLUE: starting marker", BASE_SPEED)) {
    Serial.println("Blue start marker reached. Primitive complete.");
    advanceScript();
  }
}

// =====================================================
// DRIVE HELPERS
// =====================================================

bool driveForwardUntilColor(TargetColor target, const char* label, float drive_speed) {
  int left, center, right;
  float h, s, v;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v);
  bool blue_now = isBlue(h, s, v);

  if (target == TARGET_YELLOW) {
    printYellowDebug(h, s, v, yellow_now);
  }

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    if (!script_active) {
      alvik.brake();
    }
    printDetectedColor(label, h, s, v);
    return true;
  }

  followLineOrDriveStraight(left, center, right, drive_speed);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

bool reverseStraightUntilColor(TargetColor target, const char* label) {
  int left, center, right;
  float h, s, v;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v);
  bool blue_now = isBlue(h, s, v);

  if (target == TARGET_YELLOW) {
    printYellowDebug(h, s, v, yellow_now);
  }

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    printDetectedColor(label, h, s, v);
    return true;
  }

  alvik.set_wheels_speed(-REVERSE_DOCK_SPEED, -REVERSE_DOCK_SPEED, RPM);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

void followLineOrDriveStraight(int left, int center, int right, float base_speed) {
  bool tape_now = isOnTape(left, center, right);

  if (!tape_now) {
    if (lost_line_start_ms != 0 && millis() - lost_line_start_ms > 120) {
      float turn = (last_line_error >= 0.0) ? LINE_RECOVERY_TURN_SPEED : -LINE_RECOVERY_TURN_SPEED;
      alvik.set_wheels_speed(LINE_RECOVERY_FORWARD_SPEED - turn,
                             LINE_RECOVERY_FORWARD_SPEED + turn,
                             RPM);
    } else {
      alvik.set_wheels_speed(STICKER_CROSS_SPEED, STICKER_CROSS_SPEED, RPM);
    }
    return;
  }

  float error = calculateCenterError(left, center, right);
  last_line_error = error;
  float correction = error * KP;

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  float left_speed = base_speed - correction;
  float right_speed = base_speed + correction;

  alvik.set_wheels_speed(left_speed, right_speed, RPM);
}

float calculateCenterError(int left, int center, int right) {
  float sum_weight = left + center + right;

  if (sum_weight <= 0.0) {
    return 0.0;
  }

  float centroid = (left + center * 2.0 + right * 3.0) / sum_weight;
  float error = -centroid + 2.0;

  return error;
}

bool isOnTape(int left, int center, int right) {
  return (left > TAPE_THRESHOLD || center > TAPE_THRESHOLD || right > TAPE_THRESHOLD);
}

// =====================================================
// TURN CONTROL
// =====================================================

void beginTurn(float angle,
               RobotState next_state,
               unsigned long center_ms,
               unsigned long post_exit_ms,
               unsigned long marker_ignore_ms,
               const char* label) {
  setLEDYellow();

  pending_turn_absolute = false;

  pending_turn_angle = angle;
  after_turn_state = next_state;
  pending_center_ms = center_ms;
  pending_post_exit_ms = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;
  pending_turn_label = label;

  startArcTurnRelative(angle);
  turn_phase = TURN_ROTATING;
  turn_phase_start_ms = millis();
  robot_state = TURN_GENERIC;

  resetMarkerStable();

  Serial.print("Begin ");
  Serial.println(pending_turn_label);
}

void beginTurnToYaw(float target_yaw,
                    RobotState next_state,
                    unsigned long center_ms,
                    unsigned long post_exit_ms,
                    unsigned long marker_ignore_ms,
                    const char* label) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute = true;
  pending_absolute_yaw = normalizeYaw(target_yaw);

  pending_turn_angle = 0.0;
  after_turn_state = next_state;
  pending_center_ms = center_ms;
  pending_post_exit_ms = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;
  pending_turn_label = label;

  turn_phase = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state = TURN_GENERIC;

  resetMarkerStable();

  Serial.print("Begin absolute yaw turn: ");
  Serial.println(pending_turn_label);
}

void turnGenericState() {
  unsigned long now = millis();

  switch (turn_phase) {
    case TURN_IDLE:
      turn_phase = TURN_PRE_CENTER_BRAKE;
      turn_phase_start_ms = now;
      break;

    case TURN_PRE_CENTER_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= PRE_CENTER_BRAKE_MS) {
        if (pending_center_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase = TURN_CENTER_FORWARD;
          turn_phase_start_ms = now;
        } else {
          turn_phase = TURN_PRE_ROTATE_BRAKE;
          turn_phase_start_ms = now;
        }
      }
      break;

    case TURN_CENTER_FORWARD:
      setLEDYellow();

      if (now - turn_phase_start_ms >= pending_center_ms) {
        alvik.brake();
        turn_phase = TURN_PRE_ROTATE_BRAKE;
        turn_phase_start_ms = now;
      }
      break;

    case TURN_PRE_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= PRE_ROTATE_BRAKE_MS) {
        if (pending_turn_absolute) {
          startRotateAbsolute(pending_absolute_yaw);
        } else {
          startArcTurnRelative(pending_turn_angle);
        }

        turn_phase = TURN_ROTATING;
      }
      break;

    case TURN_ROTATING:
      setLEDYellow();

      if (pending_turn_absolute ? updateRotateTo() : updateArcTurnToLine()) {
        unsigned long elapsed_ms = millis() - turn_start_ms;
        alvik.get_pose(x, y, yaw, CM, DEG);

        Serial.print("Turn complete: ");
        Serial.print(pending_turn_label);

        if (pending_turn_absolute) {
          Serial.print(" | Absolute target: ");
          Serial.print(pending_absolute_yaw);
        } else {
          Serial.print(" | Commanded: ");
          Serial.print(pending_turn_angle);
        }

        Serial.print(" deg | Time: ");
        Serial.print(elapsed_ms);
        Serial.print(" ms | Start yaw: ");
        Serial.print(turn_start_yaw, 2);
        Serial.print(" | End yaw: ");
        Serial.print(yaw, 2);
        Serial.print(" | Change: ");
        Serial.println(yaw - turn_start_yaw, 2);

        if (!pending_turn_absolute) {
          if (pending_post_exit_ms > 0) {
            alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
            turn_phase = TURN_POST_EXIT;
            turn_phase_start_ms = millis();
          } else {
            finishTurn();
          }
        } else {
          turn_phase = TURN_POST_ROTATE_BRAKE;
          turn_phase_start_ms = millis();
        }
      }
      break;

    case TURN_POST_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= POST_ROTATE_BRAKE_MS) {
        if (pending_post_exit_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase = TURN_POST_EXIT;
          turn_phase_start_ms = now;
        } else {
          finishTurn();
        }
      }
      break;

    case TURN_POST_EXIT:
      setLEDYellow();

      if (now - turn_phase_start_ms >= pending_post_exit_ms) {
        alvik.brake();
        finishTurn();
      }
      break;
  }
}

void finishTurn() {
  turn_phase = TURN_IDLE;

  if (adaptive_turn_failed) {
    alvik.brake();
    Serial.println("Turn failed to reacquire tape. Emergency stop.");
    robot_state = EMERGENCY_STOP;
    return;
  }

  marker_ignore_until_ms = millis() + pending_marker_ignore_ms;
  resetMarkerStable();

  Serial.print("Next state: ");
  printStateName(after_turn_state);

  robot_state = after_turn_state;
}

float normalizeYaw(float angle) {
  angle = fmod(angle + 360.0, 360.0);

  if (angle < 0.0) {
    angle += 360.0;
  }

  return angle;
}

float yawError(float target, float current) {
  target = normalizeYaw(target);
  current = normalizeYaw(current);

  return fmod((target - current + 540.0), 360.0) - 180.0;
}

void startRotateRelative(float relativeAngle) {
  alvik.get_pose(x, y, yaw, CM, DEG);

  turn_start_yaw = yaw;
  turn_target_yaw = normalizeYaw(yaw + relativeAngle);

  turn_start_ms = millis();
  last_turn_control_ms = 0;
  adaptive_turn_line_stable_count = 0;
  adaptive_turn_searching = false;
  adaptive_turn_saw_gap = false;
  adaptive_turn_failed = false;
  adaptive_turn_failed = false;
  adaptive_turn_saw_gap = false;

  Serial.print("Start yaw: ");
  Serial.print(turn_start_yaw, 2);
  Serial.print(" deg | Relative target yaw: ");
  Serial.print(turn_target_yaw, 2);
  Serial.println(" deg");
}

void startRotateAbsolute(float targetYaw) {
  alvik.get_pose(x, y, yaw, CM, DEG);

  turn_start_yaw = yaw;
  turn_target_yaw = normalizeYaw(targetYaw);

  turn_start_ms = millis();
  last_turn_control_ms = 0;
  adaptive_turn_line_stable_count = 0;
  adaptive_turn_searching = false;

  Serial.print("Start yaw: ");
  Serial.print(turn_start_yaw, 2);
  Serial.print(" deg | Absolute target yaw: ");
  Serial.print(turn_target_yaw, 2);
  Serial.println(" deg");
}

void startArcTurnRelative(float relativeAngle) {
  alvik.get_pose(x, y, yaw, CM, DEG);

  turn_start_yaw = yaw;
  turn_target_yaw = normalizeYaw(yaw + relativeAngle);

  turn_start_ms = millis();
  last_turn_control_ms = 0;
  adaptive_turn_line_stable_count = 0;
  adaptive_turn_searching = false;
  adaptive_turn_saw_gap = false;
  adaptive_turn_failed = false;

  Serial.print("Start arc turn | Start yaw: ");
  Serial.print(turn_start_yaw, 2);
  Serial.print(" deg | Target yaw: ");
  Serial.print(turn_target_yaw, 2);
  Serial.println(" deg");
}

bool updateArcTurnToLine() {
  unsigned long now = millis();

  if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) {
    return false;
  }

  last_turn_control_ms = now;
  alvik.get_pose(x, y, yaw, CM, DEG);

  float current_yaw = normalizeYaw(yaw);
  float turned_deg = fabs(yawError(current_yaw, turn_start_yaw));
  float max_search_deg = fabs(pending_turn_angle) + ADAPTIVE_TURN_EXTRA_DEG;

  int left, center, right;
  alvik.get_line_sensors(left, center, right);

  bool gap_seen_now = (left < 120 && center < 120 && right < 120);
  if (turned_deg >= 45.0 && gap_seen_now) {
    adaptive_turn_saw_gap = true;
    adaptive_turn_line_stable_count = 0;
  }

  bool center_on_tape = center > ADAPTIVE_TURN_CENTER_THRESHOLD;
  bool side_on_tape = (left > ADAPTIVE_TURN_SIDE_THRESHOLD ||
                       right > ADAPTIVE_TURN_SIDE_THRESHOLD);
  bool line_reacquired = (turned_deg >= ADAPTIVE_TURN_MIN_DEG &&
                          adaptive_turn_saw_gap &&
                          center_on_tape &&
                          side_on_tape);

  if (line_reacquired) {
    adaptive_turn_line_stable_count++;
  } else if (!gap_seen_now) {
    adaptive_turn_line_stable_count = 0;
  }

  if (adaptive_turn_line_stable_count >= ADAPTIVE_TURN_LINE_STABLE_SAMPLES) {
    alvik.brake();
    Serial.print("Arc turn reacquired tape at ");
    Serial.print(turned_deg, 1);
    Serial.println(" deg.");
    return true;
  }

  if (turned_deg >= max_search_deg) {
    alvik.brake();
    adaptive_turn_failed = true;
    Serial.println("Arc turn search limit reached.");
    return true;
  }

  bool right_turn = pending_turn_angle < 0.0;
  float inner_speed = (turned_deg >= fabs(pending_turn_angle)) ? ARC_TURN_SEARCH_INNER_SPEED : ARC_TURN_INNER_SPEED;
  float outer_speed = (turned_deg >= fabs(pending_turn_angle)) ? ARC_TURN_SEARCH_OUTER_SPEED : ARC_TURN_OUTER_SPEED;

  if (right_turn) {
    alvik.set_wheels_speed(outer_speed, inner_speed, RPM);
  } else {
    alvik.set_wheels_speed(inner_speed, outer_speed, RPM);
  }

  return false;
}

bool updateRotateTo() {
  unsigned long now = millis();

  if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) {
    return false;
  }

  last_turn_control_ms = now;

  alvik.get_pose(x, y, yaw, CM, DEG);

  float current_yaw = normalizeYaw(yaw);
  float error = yawError(turn_target_yaw, current_yaw);
  bool relative_turn = !pending_turn_absolute && fabs(pending_turn_angle) > 1.0;
  float turned_deg = fabs(yawError(current_yaw, turn_start_yaw));
  bool line_reacquired = false;

  if (relative_turn && turned_deg >= ADAPTIVE_TURN_MIN_DEG) {
    int left, center, right;
    alvik.get_line_sensors(left, center, right);

    bool gap_seen_now = (left < 120 && center < 120 && right < 120);
    if (gap_seen_now) {
      adaptive_turn_saw_gap = true;
      adaptive_turn_line_stable_count = 0;
    }

    bool center_on_tape = center > ADAPTIVE_TURN_CENTER_THRESHOLD;
    bool side_on_tape = (left > ADAPTIVE_TURN_SIDE_THRESHOLD ||
                         right > ADAPTIVE_TURN_SIDE_THRESHOLD);
    line_reacquired = adaptive_turn_saw_gap && center_on_tape && side_on_tape;

    if (line_reacquired) {
      adaptive_turn_line_stable_count++;
    } else {
      adaptive_turn_line_stable_count = 0;
    }

    if (adaptive_turn_line_stable_count >= ADAPTIVE_TURN_LINE_STABLE_SAMPLES) {
      alvik.brake();
      Serial.print("Adaptive turn stopped on tape at ");
      Serial.print(turned_deg, 1);
      Serial.println(" deg.");
      return true;
    }
  }

  if (relative_turn && adaptive_turn_searching) {
    float max_search_deg = fabs(pending_turn_angle) + ADAPTIVE_TURN_EXTRA_DEG;
    if (turned_deg >= max_search_deg) {
      alvik.brake();
      adaptive_turn_failed = true;
      Serial.println("Adaptive turn search limit reached.");
      return true;
    }

    if (pending_turn_angle > 0.0) {
      alvik.set_wheels_speed(-ADAPTIVE_TURN_SEARCH_SPEED, ADAPTIVE_TURN_SEARCH_SPEED, RPM);
    } else {
      alvik.set_wheels_speed(ADAPTIVE_TURN_SEARCH_SPEED, -ADAPTIVE_TURN_SEARCH_SPEED, RPM);
    }

    return false;
  }

  if (fabs(error) <= YAW_TOLERANCE) {
    if (relative_turn) {
      adaptive_turn_searching = true;
      return false;
    }

    alvik.brake();
    return true;
  }

  float scale = fabs(error) / 90.0;

  if (scale > 1.0) {
    scale = 1.0;
  }

  float speed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * scale;

  if (error > 0.0) {
    alvik.set_wheels_speed(-speed, speed, RPM);
  } else {
    alvik.set_wheels_speed(speed, -speed, RPM);
  }

  return false;
}

// =====================================================
// MARKER DETECTION
// =====================================================

bool targetColorDetectedStable(TargetColor target, bool red_now, bool yellow_now, bool blue_now) {
  if (millis() < marker_ignore_until_ms) {
    resetMarkerStable();
    return false;
  }

  bool target_now = false;

  if (target == TARGET_RED) {
    target_now = red_now;
  } else if (target == TARGET_YELLOW) {
    target_now = yellow_now;
  } else if (target == TARGET_BLUE) {
    target_now = blue_now;
  }

  int needed_samples = MARKER_STABLE_SAMPLES;

  if (target == TARGET_YELLOW) {
    needed_samples = YELLOW_STABLE_SAMPLES;
  }

  if (target_now) {
    if (last_marker_target != target) {
      marker_stable_count = 0;
      last_marker_target = target;
    }

    marker_stable_count++;
  } else {
    marker_stable_count = 0;
    last_marker_target = target;
  }

  if (marker_stable_count >= needed_samples) {
    resetMarkerStable();
    return true;
  }

  return false;
}

void resetMarkerStable() {
  marker_stable_count = 0;
}

void printDetectedColor(const char* label, float h, float s, float v) {
  Serial.print("Detected ");
  Serial.print(label);
  Serial.print(" | H=");
  Serial.print(h, 1);
  Serial.print(" S=");
  Serial.print(s, 3);
  Serial.print(" V=");
  Serial.println(v, 3);
}

void printYellowDebug(float h, float s, float v, bool yellow_now) {
  if (millis() - last_yellow_debug_ms < YELLOW_DEBUG_PRINT_MS) {
    return;
  }

  last_yellow_debug_ms = millis();

  Serial.print("Searching for yellow | H=");
  Serial.print(h, 1);
  Serial.print(" S=");
  Serial.print(s, 3);
  Serial.print(" V=");
  Serial.print(v, 3);
  Serial.print(" | yellow_now=");
  Serial.println(yellow_now);
}

bool isRed(float h, float s, float v) {
  bool hue_red = (h > 340.0 || h < 20.0);
  bool saturated = s > 0.40;
  bool bright_enough = v > 0.04;

  return hue_red && saturated && bright_enough;
}

bool isYellow(float h, float s, float v) {
  bool hue_yellow = (h > 30.0 && h < 95.0);
  bool saturated = s > 0.20;
  bool bright_enough = v > 0.03;

  return hue_yellow && saturated && bright_enough;
}

bool isBlue(float h, float s, float v) {
  bool hue_blue = (h > 190.0 && h < 230.0);
  bool saturated = s > 0.40;
  bool bright_enough = v > 0.04;

  return hue_blue && saturated && bright_enough;
}

// =====================================================
// LINE FAILSAFE
// =====================================================

void checkLineFailsafe(bool tape_now, bool red_now, bool yellow_now, bool blue_now) {
  if (!tape_now && !red_now && !yellow_now && !blue_now) {
    if (lost_line_start_ms == 0) {
      lost_line_start_ms = millis();
    }

    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS + LINE_RECOVERY_GRACE_MS) {
      alvik.brake();
      Serial.println("Failsafe: line lost.");
      robot_state = EMERGENCY_STOP;
    }
  } else {
    lost_line_start_ms = 0;
  }
}

// =====================================================
// LED HELPERS
// =====================================================

void setLEDOff() {
  alvik.left_led.set_color(0, 0, 0);
  alvik.right_led.set_color(0, 0, 0);
}

void setLEDRed() {
  alvik.left_led.set_color(1, 0, 0);
  alvik.right_led.set_color(1, 0, 0);
}

void setLEDGreen() {
  alvik.left_led.set_color(0, 1, 0);
  alvik.right_led.set_color(0, 1, 0);
}

void setLEDBlue() {
  alvik.left_led.set_color(0, 0, 1);
  alvik.right_led.set_color(0, 0, 1);
}

void setLEDYellow() {
  alvik.left_led.set_color(1, 1, 0);
  alvik.right_led.set_color(1, 1, 0);
}

// =====================================================
// MICRO-ROS HELPERS
// =====================================================

void initTransport() {
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  unsigned long start_ms = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_ms < 15000) {
    setLEDYellow();
    delay(250);
    setLEDOff();
    delay(250);
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

  if (rclc_subscription_init_best_effort(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_CMD) != RCL_RET_OK) return false;

  msg_cmd_in.data.data = cmd_buf;
  msg_cmd_in.data.size = 0;
  msg_cmd_in.data.capacity = sizeof(cmd_buf);

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(
        &executor, &sub_cmd, &msg_cmd_in,
        &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  msg_status.data.data = NULL;
  msg_status.data.size = 0;
  msg_status.data.capacity = 0;
  return true;
}

void cmdCallback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  String cmd = String(msg->data.data);
  cmd.trim();

  if (cmd.equalsIgnoreCase("stop")) {
    alvik.brake();
    mission_mode = MISSION_NONE;
    mission_phase = PHASE_IDLE;
    script_active = false;
    script_idx = 0;
    script_len = 0;
    turn_phase = TURN_IDLE;
    robot_state = WAIT_FOR_START;
    setLEDBlue();
    return;
  }

  if (!ready_confirmed) return;

  if (cmd.length() > 4 && cmd.substring(0, 4).equalsIgnoreCase("run ")) {
    if (!(robot_state == WAIT_FOR_START || robot_state == DONE)) return;
    String seq = cmd.substring(4);
    if (loadScriptFromText(seq)) {
      startLoadedScript(MISSION_STATION_A_CYCLE);
    } else {
      Serial.println("Bad primitive script in run command.");
    }
    return;
  }

  if (cmd.length() > 7 && cmd.substring(0, 7).equalsIgnoreCase("script ")) {
    if (!(robot_state == WAIT_FOR_START || robot_state == DONE)) return;
    String seq = cmd.substring(7);
    if (loadScriptFromText(seq)) {
      startLoadedScript(MISSION_STATION_A_CYCLE);
    } else {
      Serial.println("Bad primitive script in script command.");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("station_a_cycle") ||
      cmd.equalsIgnoreCase("workstation_a_cycle") ||
      cmd.equalsIgnoreCase("cell_cycle")) {
    if (robot_state == WAIT_FOR_START || robot_state == DONE) {
      startMission(MISSION_STATION_A_CYCLE);
    }
    return;
  }

  if (cmd.equalsIgnoreCase("station_a_once") ||
      cmd.equalsIgnoreCase("cell_once")) {
    if (robot_state == WAIT_FOR_START || robot_state == DONE) {
      startMission(MISSION_SINGLE_PASS);
    }
    return;
  }
}

void publishStatus(unsigned long now) {
  if (now - last_status_ms < 200) return;
  last_status_ms = now;

  int left, center, right;
  float h, s, v;
  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  alvik.get_pose(x, y, yaw, CM, DEG);

  unsigned long remaining_ms = 0;
  if (robot_state == PROCESSING_WAIT) {
    unsigned long elapsed = now - processing_wait_start_ms;
    remaining_ms = (elapsed >= PROCESSING_WAIT_MS) ? 0 : (PROCESSING_WAIT_MS - elapsed);
  }

  static char status_buf[480];
  snprintf(status_buf, sizeof(status_buf),
           "{\"state\":\"%s\",\"phase\":\"%s\",\"station\":\"A\","
           "\"cmd_topic\":\"%s\",\"status_topic\":\"%s\","
           "\"step\":%d,\"total\":%d,\"processing_remaining_ms\":%lu,"
           "\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,"
           "\"L\":%d,\"C\":%d,\"R\":%d,\"h\":%.1f,\"s\":%.3f,\"v\":%.3f,"
           "\"ros\":%d,\"ms\":%lu}",
           stateName(robot_state), missionPhaseName(mission_phase), T_CMD, T_STATUS, script_idx, script_len,
           remaining_ms, x, y, yaw, left, center, right, h, s, v,
           ros_ready ? 1 : 0, now);

  msg_status.data.data = status_buf;
  msg_status.data.size = strlen(status_buf);
  msg_status.data.capacity = sizeof(status_buf);
  rcl_publish(&pub_status, &msg_status, NULL);
}

// =====================================================
// DEBUG HELPERS
// =====================================================

const char* stateName(RobotState state) {
  switch (state) {
    case WAIT_FOR_START: return ready_confirmed ? "IDLE" : "NOT_READY";
    case PROCESSING_WAIT: return "PROCESSING_WAIT";
    case SCRIPT_ADVANCE: return "MOVING";
    case SCRIPT_CLEAR_MARKER: return "MOVING";
    case DRIVE_TO_RED_1:
    case DRIVE_TO_YELLOW_ENTRY:
    case DRIVE_TO_YELLOW_WORKSTATION:
    case REVERSE_TO_WORKSTATION_YELLOW:
    case WORKSTATION_WAIT:
    case DRIVE_OUT_TO_YELLOW_ENTRY:
    case DRIVE_TO_RED_2:
    case DRIVE_TO_RED_3:
    case DRIVE_TO_BLUE:
    case TURN_GENERIC:
      return "MOVING";
    case DONE: return "ARRIVED";
    case EMERGENCY_STOP: return "EMERGENCY_STOP";
  }

  return "UNKNOWN";
}

const char* scriptOpName(ScriptOp op) {
  switch (op) {
    case OP_RED: return "RED";
    case OP_YENTRY: return "YENTRY";
    case OP_YWORK: return "YWORK";
    case OP_EXIT: return "EXIT";
    case OP_BLUE: return "BLUE";
    case OP_DOCK: return "DOCK";
    case OP_DWELL5: return "DWELL5";
    case OP_WAIT30: return "WAIT30";
    case OP_CLEAR: return "CLEAR";
    case OP_R: return "R";
    case OP_L: return "L";
    case OP_R_YSEARCH: return "R_YSEARCH";
    case OP_R_SPUR: return "R_SPUR";
    case OP_R_FINAL: return "R_FINAL";
    case OP_YAW0: return "YAW0";
  }

  return "UNKNOWN";
}

const char* missionPhaseName(MissionPhase phase) {
  switch (phase) {
    case PHASE_IDLE: return "IDLE";
    case PHASE_DELIVERING: return "DELIVERING";
    case PHASE_PROCESSING: return "PROCESSING";
    case PHASE_PICKING_UP: return "PICKING_UP";
    case PHASE_COMPLETE: return "COMPLETE";
  }

  return "UNKNOWN";
}

void printStateName(RobotState state) {
  switch (state) {
    case WAIT_FOR_START:
      Serial.println("WAIT_FOR_START");
      break;
    case PROCESSING_WAIT:
      Serial.println("PROCESSING_WAIT");
      break;
    case SCRIPT_ADVANCE:
      Serial.println("SCRIPT_ADVANCE");
      break;
    case SCRIPT_CLEAR_MARKER:
      Serial.println("SCRIPT_CLEAR_MARKER");
      break;
    case DRIVE_TO_RED_1:
      Serial.println("DRIVE_TO_RED_1");
      break;
    case DRIVE_TO_YELLOW_ENTRY:
      Serial.println("DRIVE_TO_YELLOW_ENTRY");
      break;
    case DRIVE_TO_YELLOW_WORKSTATION:
      Serial.println("DRIVE_TO_YELLOW_WORKSTATION");
      break;
    case REVERSE_TO_WORKSTATION_YELLOW:
      Serial.println("REVERSE_TO_WORKSTATION_YELLOW");
      break;
    case WORKSTATION_WAIT:
      Serial.println("WORKSTATION_WAIT");
      break;
    case DRIVE_OUT_TO_YELLOW_ENTRY:
      Serial.println("DRIVE_OUT_TO_YELLOW_ENTRY");
      break;
    case DRIVE_TO_RED_2:
      Serial.println("DRIVE_TO_RED_2");
      break;
    case DRIVE_TO_RED_3:
      Serial.println("DRIVE_TO_RED_3");
      break;
    case DRIVE_TO_BLUE:
      Serial.println("DRIVE_TO_BLUE");
      break;
    case TURN_GENERIC:
      Serial.println("TURN_GENERIC");
      break;
    case DONE:
      Serial.println("DONE");
      break;
    case EMERGENCY_STOP:
      Serial.println("EMERGENCY_STOP");
      break;
  }
}
