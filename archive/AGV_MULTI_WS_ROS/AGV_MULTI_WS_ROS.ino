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

char WIFI_SSID[]     = "YOUR_WIFI_SSID";
char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
char AGENT_IP[]      = "192.0.2.13";
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
std_msgs__msg__String msg_cmd_in;
char cmd_buf[256];

bool ros_ready = false;
unsigned long last_status_ms = 0;
const unsigned long STATUS_PERIOD_MS = 200;

// =====================================================
// TARGET SELECTION / MULTI-WORKSTATION MISSION
// =====================================================
// Default mission order. ROS commands can replace this at runtime.
const uint8_t MAX_MISSION_STOPS = 12;
uint8_t WORKSTATION_SEQUENCE[MAX_MISSION_STOPS] = { 1, 2, 3, 4, 5, 6 };
uint8_t NUM_MISSION_STOPS = 6;

// true  = after the final workstation, return to the blue depot marker.
// false = after the final workstation, remain docked there.
bool RETURN_TO_DEPOT_AFTER_FINAL_WORKSTATION = true;

// =====================================================
// MAP / NODE-BASED WORKSTATION DICTIONARY
// =====================================================
// Node convention:
// - 8 columns x 8 rows.
// - Nodes are numbered row-major from the bottom-left.
// - Row 1: nodes 1-8
// - Row 2: nodes 9-16
// - Row 3: nodes 17-24
// - ...
// - Row 8: nodes 57-64
//
// Workstations are located BETWEEN two neighboring red nodes.
//
// Current approach:
// - Each workstation can be approached from either neighboring red node.
// - APPROACH_EAST enters from the left node and drives EAST through the segment.
// - APPROACH_WEST enters from the right node and drives WEST through the segment.
// - Yellow entry marker triggers a turn SOUTH into the workstation spur.

const uint8_t GRID_COLS = 8;

enum ApproachDirection {
  APPROACH_EAST,
  APPROACH_WEST
};

struct Workstation {
  uint8_t id;

  float map_x_in;
  float map_y_in;

  float agv_x_in;
  float agv_y_in;

  uint8_t left_node;
  uint8_t right_node;

  ApproachDirection approach_direction;

  const char* name;
};

const Workstation WORKSTATIONS[] = {
  // id, map_x, map_y, agv_x, agv_y, left_node, right_node, approach_direction, name
  { 1,   4.5, 10.0, 10.0,  4.5,  9, 10, APPROACH_EAST, "WS01" },
  { 2,  44.5, 10.0, 10.0, 44.5, 13, 14, APPROACH_EAST, "WS02" },
  { 3,  15.0, 19.75, 19.75, 15.0, 18, 19, APPROACH_EAST, "WS03" },
  { 4,  24.5, 19.75, 19.75, 24.5, 19, 20, APPROACH_EAST, "WS04" },
  { 5,  64.0, 19.75, 19.75, 64.0, 23, 24, APPROACH_EAST, "WS05" },
  { 6,  25.0, 29.75, 29.75, 25.0, 27, 28, APPROACH_EAST, "WS06" },
  { 7,   5.0, 40.0, 40.0,  5.0, 33, 34, APPROACH_EAST, "WS07" },
  { 8,  24.5, 40.0, 40.0, 24.5, 35, 36, APPROACH_EAST, "WS08" },
  { 9,  54.5, 40.0, 40.0, 54.5, 38, 39, APPROACH_EAST, "WS09" },
  {10,  15.0, 49.5, 49.5, 15.0, 42, 43, APPROACH_EAST, "WS10" },
  {11,  25.0, 49.5, 49.5, 25.0, 43, 44, APPROACH_EAST, "WS11" },
  {12,  64.0, 59.5, 59.5, 64.0, 55, 56, APPROACH_EAST, "WS12" }
};

const uint8_t NUM_WORKSTATIONS = sizeof(WORKSTATIONS) / sizeof(WORKSTATIONS[0]);
const Workstation* target_ws = nullptr;
const Workstation* next_ws = nullptr;
ApproachDirection active_approach_direction = APPROACH_EAST;
ApproachDirection next_approach_direction = APPROACH_EAST;

uint8_t mission_index = 0;

// =====================================================
// GRID HEADINGS (Alvik yaw frame, CCW positive)
// =====================================================
// Matches the convention already used in this sketch: 90 = WEST, 270 = EAST.
const float YAW_N = 0.0;
const float YAW_E = 270.0;
const float YAW_S = 180.0;
const float YAW_W = 90.0;

// The grid heading the current leg should hold. Updated by finishTurn() and
// used as the steering reference whenever the tape is absent or ambiguous.
float leg_target_yaw = YAW_N;

// =====================================================
// TUNING VALUES
// =====================================================

const int TAPE_THRESHOLD = 275;

const float BASE_SPEED = 60.0;
const float YELLOW_SEARCH_SPEED = 30.0;
const float WORKSTATION_APPROACH_SPEED = 30.0;
const float KP = 45.0;
const float MAX_CORRECTION = 40.0;

const float STICKER_CROSS_SPEED = 30.0;

// Heading hold: used on stickers, at intersections, and while reversing.
const float KP_HEADING = 1.5;             // RPM per degree of heading error
const float MAX_HEADING_CORRECTION = 25.0;

const float REVERSE_DOCK_SPEED = 20.0;

// The robot starts facing NORTH at yaw = 0.0. To back into a station while its
// travel direction is SOUTH, it should face NORTH before reversing.
const float REVERSE_DOCK_ABSOLUTE_YAW = 0.0;
const float FINAL_DEPOT_YAW = 0.0;

// Turn controller. 0.5 deg tolerance with 5 RPM minimum stalls near the target:
// the wheels cannot overcome static friction at that speed, so the turn never
// satisfies the tolerance. 2 deg / 15 RPM completes reliably, and the heading
// hold plus line PD remove the residual error during the leg.
const float YAW_TOLERANCE = 1.0;
const float TURN_MIN_SPEED = 15.0;
const float TURN_MAX_SPEED = 60.0;
const unsigned long TURN_CONTROL_MS = 5;

const unsigned long TURN_CENTERING_MS = 120;
const unsigned long PRE_CENTER_BRAKE_MS = 100;
const unsigned long PRE_ROTATE_BRAKE_MS = 200;
const unsigned long POST_ROTATE_BRAKE_MS = 200;

const unsigned long POST_TURN_EXIT_MS = 350;
const unsigned long YELLOW_POST_TURN_EXIT_MS = 350;
const float POST_TURN_EXIT_SPEED = 30.0;

// Marker gating
const int RED_STABLE_SAMPLES = 1;
// The stricter HSV/RGB yellow filter handles false positives; keep this low
// enough that the AGV does not pass over a short yellow sticker before latching it.
const int YELLOW_STABLE_SAMPLES = 1;
const int BLUE_STABLE_SAMPLES = 5;

const unsigned long MARKER_IGNORE_AFTER_TURN_MS = 600;
const unsigned long MARKER_IGNORE_AFTER_RED_PASS_MS = 750;

// After the AGV reaches the red node at the start of a workstation segment,
// this prevents immediate yellow acceptance while it is still physically on
// the red node. It forces the AGV to continue into the segment first.
const unsigned long YELLOW_ARM_AFTER_ENTRY_RED_MS = 750;

const unsigned long YELLOW_ENTRY_DEPART_IGNORE_MS = 350;
const unsigned long YELLOW_IGNORE_AFTER_YAW_ALIGN_MS = 300;
const unsigned long EXIT_WORKSTATION_IGNORE_MS = 1000;

const unsigned long WORKSTATION_WAIT_MS = 1000;
const unsigned long WORKSTATION_BLINK_MS = 250;

const unsigned long LOST_LINE_FAILSAFE_MS = 1500;
const unsigned long LOOP_DELAY_MS = 10;

// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState {
  WAIT_FOR_START,

  DRIVE_NORTH_TO_ENTRY_ROW,
  DRIVE_EAST_TO_ENTRY_COL,
  DRIVE_TO_YELLOW_ENTRY,
  DRIVE_TO_YELLOW_WORKSTATION,
  REVERSE_TO_WORKSTATION_YELLOW,
  WORKSTATION_WAIT,
  DRIVE_OUT_TO_YELLOW_ENTRY,

  DRIVE_EAST_TO_NEXT_ENTRY_COL,
  DRIVE_WEST_TO_NEXT_ENTRY_COL,
  DRIVE_NORTH_TO_NEXT_ENTRY_ROW,
  DRIVE_SOUTH_TO_NEXT_ENTRY_ROW,

  DRIVE_WEST_TO_RETURN_COL,
  DRIVE_SOUTH_TO_BLUE,

  TURN_GENERIC,
  DONE,
  EMERGENCY_STOP
};

RobotState robot_state = WAIT_FOR_START;
RobotState after_turn_state = DONE;

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
int last_line_left = 0;
int last_line_center = 0;
int last_line_right = 0;
float last_color_h = 0.0;
float last_color_s = 0.0;
float last_color_v = 0.0;
float last_color_nr = 0.0;
float last_color_ng = 0.0;
float last_color_nb = 0.0;
float last_color_chroma = 0.0;
float last_hsv_chroma = 0.0;
bool last_red_now = false;
bool last_yellow_now = false;
bool last_blue_now = false;

unsigned long lost_line_start_ms = 0;
unsigned long marker_ignore_until_ms = 0;

TargetColor last_marker_target = TARGET_RED;
int marker_stable_count = 0;

int northbound_red_count = 0;
int eastbound_red_count = 0;
int westbound_return_red_count = 0;

int transfer_red_count = 0;
int transfer_needed_reds = 0;

unsigned long turn_phase_start_ms = 0;
unsigned long turn_start_ms = 0;
unsigned long last_turn_control_ms = 0;

float turn_start_yaw = 0.0;
float turn_target_yaw = 0.0;
float pending_turn_angle = 0.0;

bool pending_turn_absolute = false;
float pending_absolute_yaw = 0.0;

unsigned long pending_center_ms = 0;
unsigned long pending_post_exit_ms = 0;
unsigned long pending_marker_ignore_ms = MARKER_IGNORE_AFTER_TURN_MS;

unsigned long workstation_wait_start_ms = 0;

bool ready_confirmed = false;
unsigned long blue_confirm_start_ms = 0;
const unsigned long START_CONFIRM_MS = 1500;
bool ros_start_requested = false;
bool mission_active = false;
bool emergency_printed = false;
char last_command[96] = "";

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

void waitForStartState();
void driveNorthToEntryRowState();
void driveEastToEntryColState();
void driveToYellowEntryState();
void driveToYellowWorkstationState();
void reverseToWorkstationYellowState();
void workstationWaitState();
void driveOutToYellowEntryState();

void driveEastToNextEntryColState();
void driveWestToNextEntryColState();
void driveNorthToNextEntryRowState();
void driveSouthToNextEntryRowState();

void driveWestToReturnColState();
void driveSouthToBlueState();

bool driveForwardUntilColor(TargetColor target, float drive_speed);
bool reverseStraightUntilColor(TargetColor target);
void followLineOrDriveStraight(int left, int center, int right, float base_speed);
float calculateCenterError(int left, int center, int right);
bool isOnTape(int left, int center, int right);
bool isIntersection(int left, int center, int right);
float headingCorrection();

void beginTurn(float angle, RobotState next_state, unsigned long center_ms,
               unsigned long post_exit_ms, unsigned long marker_ignore_ms);
void beginTurnToYaw(float target_yaw, RobotState next_state, unsigned long center_ms,
                    unsigned long post_exit_ms, unsigned long marker_ignore_ms);
void turnGenericState();
void finishTurn();
float normalizeYaw(float angle);
float yawError(float target, float current);
void startRotateRelative(float relativeAngle);
void startRotateAbsolute(float targetYaw);
bool updateRotateTo();

bool targetColorDetectedStable(TargetColor target, bool red_now, bool yellow_now, bool blue_now);
void resetMarkerStable();
bool isRed(float h, float s, float v);
float colorChroma(float a, float b, float c);
bool isYellow(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right);
bool isBlue(float h, float s, float v);

void checkLineFailsafe(bool tape_now, bool red_now, bool yellow_now, bool blue_now);
void setLEDOff();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDYellow();

const Workstation* findWorkstationById(uint8_t id);
int nodeRow(uint8_t node);
int nodeCol(uint8_t node);
uint8_t entryNodeFor(const Workstation* ws, ApproachDirection approach);
uint8_t exitNodeFor(const Workstation* ws, ApproachDirection approach);
uint8_t activeEntryNode();
uint8_t activeExitNode();
float approachYaw(ApproachDirection approach);
const char* approachName(ApproachDirection approach);
ApproachDirection chooseInitialApproach(const Workstation* ws);
ApproachDirection chooseNextApproach(const Workstation* from_ws, const Workstation* to_ws);
int horizontalRedCountFromCurrentSegmentToNode(const Workstation* from_ws, uint8_t to_node);
int entryRowRedCount(uint8_t entry_node);
int eastRedCountBeforeYellowEntry(uint8_t entry_node);
int westRedCountToDepotColumn(const Workstation* ws);

bool hasNextWorkstation();
void prepareNextWorkstationTransfer();
void startYellowEntryApproach(bool arrived_facing_east);
void finishHorizontalTransferToNextColumn(bool arrived_facing_east);
void finishVerticalTransferToNextRow(bool arrived_facing_north);
void activateNextWorkstationTarget();

void initTransport();
bool initGraph();
void cmdCallback(const void* msgin);
void publishStatus(unsigned long now);
void processStartConfirmation();
bool loadMissionSequence(String seq);
bool appendMissionStop(uint8_t id);
void clearMissionSequence();
bool startLoadedMission();
void requestMissionStart();
void resetMissionRuntime();
const char* stateName(RobotState state);
const char* activeTargetName();
void formatMissionSequence(char* out, size_t out_size);

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  alvik.begin();
  alvik.set_illuminator(true);
  alvik.reset_pose(0, 0, 0, CM, DEG);

  snprintf(T_STATUS, sizeof(T_STATUS), "%s_status", ROBOT_NAME);
  snprintf(T_CMD, sizeof(T_CMD), "%s_cmd", ROBOT_NAME);

  initTransport();
  ros_ready = initGraph();
  ready_confirmed = false;
  blue_confirm_start_ms = 0;

  if (NUM_MISSION_STOPS == 0) {
    robot_state = EMERGENCY_STOP;
    setLEDRed();
    return;
  }

  mission_index = 0;
  target_ws = findWorkstationById(WORKSTATION_SEQUENCE[mission_index]);

  if (target_ws == nullptr) {
    robot_state = EMERGENCY_STOP;
    setLEDRed();
    return;
  }

  active_approach_direction = chooseInitialApproach(target_ws);
  next_approach_direction = APPROACH_EAST;

  Serial.println("Alvik multi-workstation mission router with ROS2 commands");
  Serial.println("Start on the blue sticker facing NORTH.");
  Serial.println("ROS examples: mission 1,2,3,4,5,6 | start | stop | reset");
  Serial.println("Manual fallback: press OK after blue is confirmed.");
  Serial.println();

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
  }

  if (alvik.get_touch_cancel() && robot_state != EMERGENCY_STOP) {
    alvik.brake();
    robot_state = EMERGENCY_STOP;
    mission_active = false;
  }

  switch (robot_state) {
    case WAIT_FOR_START:
      waitForStartState();
      break;

    case DRIVE_NORTH_TO_ENTRY_ROW:
      driveNorthToEntryRowState();
      break;

    case DRIVE_EAST_TO_ENTRY_COL:
      driveEastToEntryColState();
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

    case DRIVE_EAST_TO_NEXT_ENTRY_COL:
      driveEastToNextEntryColState();
      break;

    case DRIVE_WEST_TO_NEXT_ENTRY_COL:
      driveWestToNextEntryColState();
      break;

    case DRIVE_NORTH_TO_NEXT_ENTRY_ROW:
      driveNorthToNextEntryRowState();
      break;

    case DRIVE_SOUTH_TO_NEXT_ENTRY_ROW:
      driveSouthToNextEntryRowState();
      break;

    case DRIVE_WEST_TO_RETURN_COL:
      driveWestToReturnColState();
      break;

    case DRIVE_SOUTH_TO_BLUE:
      driveSouthToBlueState();
      break;

    case TURN_GENERIC:
      turnGenericState();
      break;

    case DONE:
      alvik.brake();
      mission_active = false;
      setLEDGreen();
      break;

    case EMERGENCY_STOP:
      alvik.brake();
      setLEDRed();
      mission_active = false;
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

  if (NUM_MISSION_STOPS == 0) {
    robot_state = EMERGENCY_STOP;
    return;
  }

  if (!ready_confirmed) {
    return;
  }

  if (ros_start_requested || alvik.get_touch_ok()) {
    ros_start_requested = false;
    startLoadedMission();
  }
}

void processStartConfirmation() {
  int left, center, right;
  float h, s, v;
  unsigned long now = millis();

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  last_line_left = left;
  last_line_center = center;
  last_line_right = right;
  last_color_h = h;
  last_color_s = s;
  last_color_v = v;
  last_color_nr = 0.0;
  last_color_ng = 0.0;
  last_color_nb = 0.0;
  last_color_chroma = 0.0;
  last_hsv_chroma = s * v;

  bool on_center_tape = center > TAPE_THRESHOLD;
  bool blue_now = isBlue(h, s, v);
  last_red_now = false;
  last_yellow_now = false;
  last_blue_now = blue_now;

  if (on_center_tape && blue_now) {
    if (blue_confirm_start_ms == 0) {
      blue_confirm_start_ms = now;
    }

    if (!ready_confirmed && now - blue_confirm_start_ms >= START_CONFIRM_MS) {
      ready_confirmed = true;
      alvik.reset_pose(0, 0, 0, CM, DEG);
      leg_target_yaw = YAW_N;
      setLEDBlue();
      Serial.println("Blue start marker confirmed. AGV is IDLE and ready for ROS mission.");
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

void resetMissionRuntime() {
  mission_index = 0;
  if (NUM_MISSION_STOPS == 0) {
    target_ws = nullptr;
    next_ws = nullptr;
    return;
  }

  target_ws = findWorkstationById(WORKSTATION_SEQUENCE[mission_index]);
  next_ws = nullptr;
  active_approach_direction = chooseInitialApproach(target_ws);
  next_approach_direction = APPROACH_EAST;

  northbound_red_count = 0;
  eastbound_red_count = 0;
  westbound_return_red_count = 0;
  transfer_red_count = 0;
  transfer_needed_reds = 0;

  lost_line_start_ms = 0;
  marker_ignore_until_ms = millis() + 500;
  resetMarkerStable();

  turn_phase = TURN_IDLE;
  emergency_printed = false;
  alvik.reset_pose(0, 0, 0, CM, DEG);
  leg_target_yaw = YAW_N;
}

bool startLoadedMission() {
  if (!ready_confirmed || NUM_MISSION_STOPS == 0) {
    return false;
  }

  resetMissionRuntime();

  if (target_ws == nullptr) {
    robot_state = EMERGENCY_STOP;
    return false;
  }

  mission_active = true;
  robot_state = DRIVE_NORTH_TO_ENTRY_ROW;

  Serial.print("Starting ROS mission: ");
  Serial.println(last_command);
  return true;
}

void requestMissionStart() {
  if (ready_confirmed) {
    startLoadedMission();
  } else {
    robot_state = WAIT_FOR_START;
    ros_start_requested = true;
  }
}

void clearMissionSequence() {
  NUM_MISSION_STOPS = 0;
}

bool appendMissionStop(uint8_t id) {
  if (NUM_MISSION_STOPS >= MAX_MISSION_STOPS) {
    return false;
  }

  if (findWorkstationById(id) == nullptr) {
    return false;
  }

  WORKSTATION_SEQUENCE[NUM_MISSION_STOPS++] = id;
  return true;
}

bool loadMissionSequence(String seq) {
  seq.trim();
  seq.replace(';', ',');
  seq.replace(' ', ',');

  uint8_t old_sequence[MAX_MISSION_STOPS];
  uint8_t old_count = NUM_MISSION_STOPS;
  for (uint8_t i = 0; i < old_count; i++) {
    old_sequence[i] = WORKSTATION_SEQUENCE[i];
  }

  clearMissionSequence();

  int start = 0;
  while (start < seq.length()) {
    int comma = seq.indexOf(',', start);
    String token;

    if (comma < 0) {
      token = seq.substring(start);
      start = seq.length();
    } else {
      token = seq.substring(start, comma);
      start = comma + 1;
    }

    token.trim();
    token.toUpperCase();
    if (token.length() == 0) {
      continue;
    }

    if (token.startsWith("WS")) {
      token = token.substring(2);
    }

    int id = token.toInt();
    if (id < 1 || id > 12 || !appendMissionStop((uint8_t)id)) {
      NUM_MISSION_STOPS = old_count;
      for (uint8_t i = 0; i < old_count; i++) {
        WORKSTATION_SEQUENCE[i] = old_sequence[i];
      }
      return false;
    }
  }

  if (NUM_MISSION_STOPS == 0) {
    NUM_MISSION_STOPS = old_count;
    for (uint8_t i = 0; i < old_count; i++) {
      WORKSTATION_SEQUENCE[i] = old_sequence[i];
    }
    return false;
  }

  target_ws = findWorkstationById(WORKSTATION_SEQUENCE[0]);
  active_approach_direction = chooseInitialApproach(target_ws);
  next_approach_direction = APPROACH_EAST;
  formatMissionSequence(last_command, sizeof(last_command));
  return true;
}

// =====================================================
// ROUTE STATES: DEPOT TO FIRST WORKSTATION
// =====================================================

void driveNorthToEntryRowState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    northbound_red_count++;

    if (northbound_red_count >= entryRowRedCount(activeEntryNode())) {
      beginTurnToYaw(YAW_E, DRIVE_EAST_TO_ENTRY_COL, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                     MARKER_IGNORE_AFTER_TURN_MS);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void driveEastToEntryColState() {
  int needed_reds = eastRedCountBeforeYellowEntry(activeEntryNode());

  if (needed_reds <= 0) {
    startYellowEntryApproach(true);
    return;
  }

  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    eastbound_red_count++;

    if (eastbound_red_count >= needed_reds) {
      startYellowEntryApproach(true);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

// =====================================================
// WORKSTATION ENTRY / DOCKING STATES
// =====================================================

void driveToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) {
    beginTurnToYaw(YAW_S, DRIVE_TO_YELLOW_WORKSTATION, TURN_CENTERING_MS, YELLOW_POST_TURN_EXIT_MS,
                   YELLOW_ENTRY_DEPART_IGNORE_MS);
  }
}

void driveToYellowWorkstationState() {
  if (driveForwardUntilColor(TARGET_YELLOW, WORKSTATION_APPROACH_SPEED)) {
    beginTurnToYaw(REVERSE_DOCK_ABSOLUTE_YAW, REVERSE_TO_WORKSTATION_YELLOW, 0, 0,
                   YELLOW_IGNORE_AFTER_YAW_ALIGN_MS);
  }
}

void reverseToWorkstationYellowState() {
  if (reverseStraightUntilColor(TARGET_YELLOW)) {
    workstation_wait_start_ms = millis();
    robot_state = WORKSTATION_WAIT;
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
    resetMarkerStable();
    setLEDGreen();

    if (hasNextWorkstation()) {
      next_ws = findWorkstationById(WORKSTATION_SEQUENCE[mission_index + 1]);

      if (next_ws == nullptr) {
        robot_state = EMERGENCY_STOP;
        return;
      }

      beginTurnToYaw(YAW_N,
                     DRIVE_OUT_TO_YELLOW_ENTRY,
                     0,
                     0,
                     EXIT_WORKSTATION_IGNORE_MS);
    } else {
      if (RETURN_TO_DEPOT_AFTER_FINAL_WORKSTATION) {
        next_ws = nullptr;

        beginTurnToYaw(YAW_N,
                       DRIVE_OUT_TO_YELLOW_ENTRY,
                       0,
                       0,
                       EXIT_WORKSTATION_IGNORE_MS);
      } else {
        robot_state = DONE;
      }
    }
  }
}

void driveOutToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) {
    if (next_ws != nullptr) {
      prepareNextWorkstationTransfer();
    } else {
      westbound_return_red_count = 0;

      // At the final workstation yellow entry sticker, stop and face WEST before returning home.
      beginTurnToYaw(YAW_W,
                     DRIVE_WEST_TO_RETURN_COL,
                     0,
                     POST_TURN_EXIT_MS,
                     MARKER_IGNORE_AFTER_TURN_MS);
    }
  }
}

// =====================================================
// ROUTE STATES: WORKSTATION TO NEXT WORKSTATION
// =====================================================

void driveEastToNextEntryColState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    transfer_red_count++;

    if (transfer_red_count >= transfer_needed_reds) {
      finishHorizontalTransferToNextColumn(true);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void driveWestToNextEntryColState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    transfer_red_count++;

    if (transfer_red_count >= transfer_needed_reds) {
      finishHorizontalTransferToNextColumn(false);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void driveNorthToNextEntryRowState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    transfer_red_count++;

    if (transfer_red_count >= transfer_needed_reds) {
      finishVerticalTransferToNextRow(true);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void driveSouthToNextEntryRowState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    transfer_red_count++;

    if (transfer_red_count >= transfer_needed_reds) {
      finishVerticalTransferToNextRow(false);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

// =====================================================
// ROUTE STATES: FINAL WORKSTATION TO DEPOT
// =====================================================

void driveWestToReturnColState() {
  int needed_reds = westRedCountToDepotColumn(target_ws);

  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    westbound_return_red_count++;

    if (westbound_return_red_count >= needed_reds) {
      beginTurnToYaw(YAW_S, DRIVE_SOUTH_TO_BLUE, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
                     MARKER_IGNORE_AFTER_TURN_MS);
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void driveSouthToBlueState() {
  if (driveForwardUntilColor(TARGET_BLUE, BASE_SPEED)) {
    beginTurnToYaw(FINAL_DEPOT_YAW, DONE, 0, 0,
                   MARKER_IGNORE_AFTER_TURN_MS);
  }
}

// =====================================================
// TRANSFER HELPERS
// =====================================================

bool hasNextWorkstation() {
  return (mission_index + 1 < NUM_MISSION_STOPS);
}

void prepareNextWorkstationTransfer() {
  if (target_ws == nullptr || next_ws == nullptr) {
    robot_state = EMERGENCY_STOP;
    return;
  }

  next_approach_direction = chooseNextApproach(target_ws, next_ws);

  transfer_red_count = 0;
  transfer_needed_reds = 0;

  int current_left_col = nodeCol(target_ws->left_node);
  int current_right_col = nodeCol(target_ws->right_node);
  int next_col = nodeCol(entryNodeFor(next_ws, next_approach_direction));

  // The AGV is sitting at the yellow entry sticker, physically facing NORTH.
  // From there:
  // - If the chosen entry node is east of this segment, face EAST and count red nodes.
  // - If the chosen entry node is west of this segment, face WEST and count red nodes.
  // - If the chosen entry node is one of this segment's red nodes, one red detection
  //   puts the AGV at that node.

  if (next_col >= current_right_col) {
    transfer_needed_reds = next_col - current_left_col;

    beginTurnToYaw(YAW_E,
                   DRIVE_EAST_TO_NEXT_ENTRY_COL,
                   TURN_CENTERING_MS,
                   POST_TURN_EXIT_MS,
                   MARKER_IGNORE_AFTER_TURN_MS);
    return;
  }

  if (next_col <= current_left_col) {
    transfer_needed_reds = current_right_col - next_col;

    beginTurnToYaw(YAW_W,
                   DRIVE_WEST_TO_NEXT_ENTRY_COL,
                   TURN_CENTERING_MS,
                   POST_TURN_EXIT_MS,
                   MARKER_IGNORE_AFTER_TURN_MS);
    return;
  }

  robot_state = EMERGENCY_STOP;
}

void startYellowEntryApproach(bool arrived_facing_east) {
  bool should_face_east = (active_approach_direction == APPROACH_EAST);
  bool already_facing_entry =
    (should_face_east && arrived_facing_east) ||
    (!should_face_east && !arrived_facing_east);

  if (already_facing_entry) {
    marker_ignore_until_ms = millis() + YELLOW_ARM_AFTER_ENTRY_RED_MS;
    resetMarkerStable();
    robot_state = DRIVE_TO_YELLOW_ENTRY;
    return;
  }

  beginTurnToYaw(approachYaw(active_approach_direction),
                 DRIVE_TO_YELLOW_ENTRY,
                 TURN_CENTERING_MS,
                 YELLOW_POST_TURN_EXIT_MS,
                 YELLOW_ARM_AFTER_ENTRY_RED_MS);
}

void finishHorizontalTransferToNextColumn(bool arrived_facing_east) {
  int current_row = nodeRow(target_ws->left_node);
  int next_row = nodeRow(entryNodeFor(next_ws, next_approach_direction));

  if (current_row == next_row) {
    // We are now at the next workstation's entry node row and column.
    activateNextWorkstationTarget();
    startYellowEntryApproach(arrived_facing_east);
    return;
  }

  transfer_red_count = 0;
  transfer_needed_reds = abs(next_row - current_row);

  // Absolute headings make this independent of which way the AGV arrived;
  // the closed-loop turn takes the shortest path from either facing.
  if (next_row > current_row) {
    beginTurnToYaw(YAW_N,
                   DRIVE_NORTH_TO_NEXT_ENTRY_ROW,
                   TURN_CENTERING_MS,
                   POST_TURN_EXIT_MS,
                   MARKER_IGNORE_AFTER_TURN_MS);
  } else {
    beginTurnToYaw(YAW_S,
                   DRIVE_SOUTH_TO_NEXT_ENTRY_ROW,
                   TURN_CENTERING_MS,
                   POST_TURN_EXIT_MS,
                   MARKER_IGNORE_AFTER_TURN_MS);
  }
}

void finishVerticalTransferToNextRow(bool arrived_facing_north) {
  (void)arrived_facing_north;  // absolute heading turn handles both facings

  activateNextWorkstationTarget();

  // After reaching the next workstation's entry node, face the chosen approach
  // direction and search for the yellow entry marker.
  beginTurnToYaw(approachYaw(active_approach_direction),
                 DRIVE_TO_YELLOW_ENTRY,
                 TURN_CENTERING_MS,
                 YELLOW_POST_TURN_EXIT_MS,
                 YELLOW_ARM_AFTER_ENTRY_RED_MS);
}

void activateNextWorkstationTarget() {
  if (next_ws == nullptr) {
    robot_state = EMERGENCY_STOP;
    return;
  }

  mission_index++;
  target_ws = next_ws;
  active_approach_direction = next_approach_direction;
  next_ws = nullptr;
  next_approach_direction = APPROACH_EAST;

  northbound_red_count = 0;
  eastbound_red_count = 0;
  westbound_return_red_count = 0;
  transfer_red_count = 0;
  transfer_needed_reds = 0;

  marker_ignore_until_ms = millis() + YELLOW_ARM_AFTER_ENTRY_RED_MS;
  resetMarkerStable();
}

// =====================================================
// DRIVE HELPERS
// =====================================================

bool driveForwardUntilColor(TargetColor target, float drive_speed) {
  int left, center, right;
  float h, s, v;
  float nr, ng, nb;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);
  last_line_left = left;
  last_line_center = center;
  last_line_right = right;
  last_color_h = h;
  last_color_s = s;
  last_color_v = v;

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, left, center, right);
  bool blue_now = isBlue(h, s, v);
  last_color_nr = nr;
  last_color_ng = ng;
  last_color_nb = nb;
  last_color_chroma = colorChroma(nr, ng, nb);
  last_hsv_chroma = s * v;
  last_red_now = red_now;
  last_yellow_now = yellow_now;
  last_blue_now = blue_now;

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  followLineOrDriveStraight(left, center, right, drive_speed);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

bool reverseStraightUntilColor(TargetColor target) {
  int left, center, right;
  float h, s, v;
  float nr, ng, nb;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);
  last_line_left = left;
  last_line_center = center;
  last_line_right = right;
  last_color_h = h;
  last_color_s = s;
  last_color_v = v;

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, left, center, right);
  bool blue_now = isBlue(h, s, v);
  last_color_nr = nr;
  last_color_ng = ng;
  last_color_nb = nb;
  last_color_chroma = colorChroma(nr, ng, nb);
  last_hsv_chroma = s * v;
  last_red_now = red_now;
  last_yellow_now = yellow_now;
  last_blue_now = blue_now;

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  // Heading hold while reversing keeps the dock approach straight.
  // leg_target_yaw is YAW_N here (set by the align turn before reversing).
  float h_corr = headingCorrection();
  alvik.set_wheels_speed(-REVERSE_DOCK_SPEED - h_corr, -REVERSE_DOCK_SPEED + h_corr, RPM);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

// Heading error converted to a wheel-speed correction with the same sign
// convention as the line PD: positive correction steers CCW (left).
float headingCorrection() {
  alvik.get_pose(x, y, yaw, CM, DEG);
  float err = yawError(leg_target_yaw, yaw);
  return constrain(KP_HEADING * err, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
}

// An intersection puts the crossing line under both outer sensors at once,
// which makes the line centroid meaningless.
bool isIntersection(int left, int center, int right) {
  (void)center;
  return (left > TAPE_THRESHOLD && right > TAPE_THRESHOLD);
}

void followLineOrDriveStraight(int left, int center, int right, float base_speed) {
  bool tape_now = isOnTape(left, center, right);
  bool intersection_now = isIntersection(left, center, right);

  // Sticker (no tape) or intersection (cross line under the sensors):
  // the line data is absent or garbage. Hold the grid heading instead of
  // driving blind along whatever skew the robot currently has. This is what
  // straightens the robot WHILE it crosses markers and intersections.
  if (!tape_now || intersection_now) {
    float speed = tape_now ? base_speed : STICKER_CROSS_SPEED;
    float h_corr = headingCorrection();
    alvik.set_wheels_speed(speed - h_corr, speed + h_corr, RPM);
    return;
  }

  // Clean single line under the sensors: the tape is ground truth laterally.
  float error = calculateCenterError(left, center, right);
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
               unsigned long marker_ignore_ms) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute = false;

  pending_turn_angle = angle;
  after_turn_state = next_state;
  pending_center_ms = center_ms;
  pending_post_exit_ms = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;

  turn_phase = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state = TURN_GENERIC;

  resetMarkerStable();
}

void beginTurnToYaw(float target_yaw,
                    RobotState next_state,
                    unsigned long center_ms,
                    unsigned long post_exit_ms,
                    unsigned long marker_ignore_ms) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute = true;
  pending_absolute_yaw = normalizeYaw(target_yaw);

  pending_turn_angle = 0.0;
  after_turn_state = next_state;
  pending_center_ms = center_ms;
  pending_post_exit_ms = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;

  turn_phase = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state = TURN_GENERIC;

  resetMarkerStable();
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
          startRotateRelative(pending_turn_angle);
        }

        turn_phase = TURN_ROTATING;
      }
      break;

    case TURN_ROTATING:
      setLEDYellow();

      if (updateRotateTo()) {
        turn_phase = TURN_POST_ROTATE_BRAKE;
        turn_phase_start_ms = millis();
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

    case TURN_POST_EXIT: {
      setLEDYellow();

      // Stabilize on the tape at low speed before entering the next full-speed state.
      // leg_target_yaw is not yet updated here, so use the pending turn target
      // as the heading reference for the exit creep.
      int left, center, right;
      alvik.get_line_sensors(left, center, right);

      bool tape_now = isOnTape(left, center, right);
      bool intersection_now = isIntersection(left, center, right);

      if (tape_now && !intersection_now) {
        followLineOrDriveStraight(left, center, right, POST_TURN_EXIT_SPEED);
      } else {
        alvik.get_pose(x, y, yaw, CM, DEG);
        float err = yawError(pending_turn_absolute ? pending_absolute_yaw : turn_target_yaw, yaw);
        float h_corr = constrain(KP_HEADING * err, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
        alvik.set_wheels_speed(POST_TURN_EXIT_SPEED - h_corr, POST_TURN_EXIT_SPEED + h_corr, RPM);
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
  turn_phase = TURN_IDLE;
  marker_ignore_until_ms = millis() + pending_marker_ignore_ms;
  resetMarkerStable();

  // Record the grid heading for the new leg. The heading hold uses this as
  // its steering reference on stickers and at intersections.
  if (pending_turn_absolute) {
    leg_target_yaw = pending_absolute_yaw;
  } else {
    leg_target_yaw = turn_target_yaw;
  }

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
}

void startRotateAbsolute(float targetYaw) {
  alvik.get_pose(x, y, yaw, CM, DEG);

  turn_start_yaw = yaw;
  turn_target_yaw = normalizeYaw(targetYaw);

  turn_start_ms = millis();
  last_turn_control_ms = 0;
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

  if (fabs(error) <= YAW_TOLERANCE) {
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

  int needed_samples = RED_STABLE_SAMPLES;

  if (target == TARGET_YELLOW) {
    needed_samples = YELLOW_STABLE_SAMPLES;
  } else if (target == TARGET_BLUE) {
    needed_samples = BLUE_STABLE_SAMPLES;
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

bool isRed(float h, float s, float v) {
  bool hue_red = (h > 340.0 || h < 20.0);
  bool saturated = s > 0.40;
  bool bright_enough = v > 0.04;

  return hue_red && saturated && bright_enough;
}

float colorChroma(float a, float b, float c) {
  float max_val = a;
  if (b > max_val) {
    max_val = b;
  }
  if (c > max_val) {
    max_val = c;
  }

  float min_val = a;
  if (b < min_val) {
    min_val = b;
  }
  if (c < min_val) {
    min_val = c;
  }

  return max_val - min_val;
}

bool isYellow(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right) {
  // Tuned from the logged data:
  // - False yellow on black tape: on_tape=1, H about 76-85, S about 0.10-0.13, V about 0.18-0.20, norm_chroma about 0.02
  // - Actual yellow at driving position: on_tape=1, H about 39-40, S about 0.83-0.85, V about 0.13, norm_chroma about 0.107-0.112
  //
  // Important: do NOT require the line sensors to be off tape here.
  // At the real yellow stop point, the color sensor can see yellow while the line sensors still see tape.

  float rgb_chroma = colorChroma(nr, ng, nb);
  float hsv_chroma = s * v;
  bool tape_now = isOnTape(left, center, right);

  // Reject the repeatable false-positive signature measured on glossy black duct tape.
  bool black_tape_false_yellow =
    tape_now &&
    h > 65.0 && h < 95.0 &&
    s < 0.25 &&
    v < 0.30 &&
    rgb_chroma < 0.05;

  if (black_tape_false_yellow) {
    return false;
  }

  // Accept the actual yellow sticker using hue, saturation, and HSV chroma.
  // Earlier real yellow samples included H about 33, S about 0.55 and H about 39,
  // V about 0.13. At the sticker edge, the RGB shape can be weak even while
  // HSV is consistently yellow, so do not require a fixed R/G/B ordering.
  bool hue_yellow =
    h > 28.0 && h < 55.0;

  bool saturated_enough =
    s > 0.42;

  bool bright_enough =
    v > 0.07;

  bool colorful_enough =
    hsv_chroma > 0.045;

  return hue_yellow &&
         saturated_enough &&
         bright_enough &&
         colorful_enough;
}

bool isBlue(float h, float s, float v) {
  bool hue_blue = (h > 200.0 && h < 220.0);
  bool saturated = s > 0.60;
  bool bright_enough = v > 0.08;

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

    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS) {
      alvik.brake();
      robot_state = EMERGENCY_STOP;
    }
  } else {
    lost_line_start_ms = 0;
  }
}

// =====================================================
// MAP / NODE ROUTE HELPERS
// =====================================================

const Workstation* findWorkstationById(uint8_t id) {
  for (uint8_t i = 0; i < NUM_WORKSTATIONS; i++) {
    if (WORKSTATIONS[i].id == id) {
      return &WORKSTATIONS[i];
    }
  }

  return nullptr;
}

int nodeRow(uint8_t node) {
  return ((int)node - 1) / GRID_COLS + 1;
}

int nodeCol(uint8_t node) {
  return ((int)node - 1) % GRID_COLS + 1;
}

uint8_t entryNode(const Workstation* ws) {
  if (ws->approach_direction == APPROACH_EAST) {
    return ws->left_node;
  }

  return ws->right_node;
}

uint8_t exitNode(const Workstation* ws) {
  if (ws->approach_direction == APPROACH_EAST) {
    return ws->right_node;
  }

  return ws->left_node;
}

int entryRowRedCount(const Workstation* ws) {
  int needed = nodeRow(entryNode(ws)) - 1;

  if (needed < 1) {
    needed = 1;
  }

  return needed;
}

int eastRedCountBeforeYellowEntry(const Workstation* ws) {
  return nodeCol(entryNode(ws)) - 1;
}

int westRedCountToDepotColumn(const Workstation* ws) {
  return nodeCol(entryNode(ws));
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
  String cmd = "";

  for (size_t i = 0; i < msg->data.size; i++) {
    cmd += msg->data.data[i];
  }

  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  Serial.print("ROS command: ");
  Serial.println(cmd);

  if (cmd.equalsIgnoreCase("stop")) {
    alvik.brake();
    mission_active = false;
    ros_start_requested = false;
    turn_phase = TURN_IDLE;
    robot_state = WAIT_FOR_START;
    ready_confirmed = false;
    blue_confirm_start_ms = 0;
    return;
  }

  if (cmd.equalsIgnoreCase("reset")) {
    alvik.brake();
    mission_active = false;
    ros_start_requested = false;
    turn_phase = TURN_IDLE;
    robot_state = WAIT_FOR_START;
    ready_confirmed = false;
    blue_confirm_start_ms = 0;
    resetMissionRuntime();
    return;
  }

  if (cmd.equalsIgnoreCase("return_on")) {
    RETURN_TO_DEPOT_AFTER_FINAL_WORKSTATION = true;
    return;
  }

  if (cmd.equalsIgnoreCase("return_off")) {
    RETURN_TO_DEPOT_AFTER_FINAL_WORKSTATION = false;
    return;
  }

  bool idle_or_done = (robot_state == WAIT_FOR_START || robot_state == DONE);

  if (cmd.length() > 5 && cmd.substring(0, 5).equalsIgnoreCase("load ")) {
    if (!idle_or_done) return;
    String seq = cmd.substring(5);
    if (loadMissionSequence(seq)) {
      Serial.println("Mission loaded. Send start when ready.");
    } else {
      Serial.println("Bad mission sequence.");
    }
    return;
  }

  if (cmd.length() > 8 && cmd.substring(0, 8).equalsIgnoreCase("mission ")) {
    if (!idle_or_done) return;
    String seq = cmd.substring(8);
    if (loadMissionSequence(seq)) {
      requestMissionStart();
      Serial.println("Mission loaded and queued to start.");
    } else {
      Serial.println("Bad mission sequence.");
    }
    return;
  }

  if (cmd.length() > 4 && cmd.substring(0, 4).equalsIgnoreCase("run ")) {
    if (!idle_or_done) return;
    String seq = cmd.substring(4);
    if (loadMissionSequence(seq)) {
      requestMissionStart();
      Serial.println("Mission loaded and queued to start.");
    } else {
      Serial.println("Bad mission sequence.");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("start")) {
    if (idle_or_done) {
      requestMissionStart();
    }
    return;
  }
}

void publishStatus(unsigned long now) {
  if (now - last_status_ms < STATUS_PERIOD_MS) return;
  last_status_ms = now;

  char sequence_buf[64];

  alvik.get_pose(x, y, yaw, CM, DEG);
  formatMissionSequence(sequence_buf, sizeof(sequence_buf));

  static char status_buf[1024];
  snprintf(status_buf, sizeof(status_buf),
           "{\"state\":\"%s\",\"target\":\"%s\",\"mission_index\":%u,"
           "\"mission_total\":%u,\"sequence\":\"%s\","
           "\"cmd_topic\":\"%s\",\"status_topic\":\"%s\","
           "\"ready\":%d,\"active\":%d,\"return_to_depot\":%d,"
           "\"north_red\":%d,\"east_red\":%d,\"west_red\":%d,"
           "\"transfer_red\":%d,\"transfer_needed\":%d,"
           "\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,"
           "\"L\":%d,\"C\":%d,\"R\":%d,\"h\":%.1f,\"s\":%.3f,\"v\":%.3f,"
           "\"nr\":%.3f,\"ng\":%.3f,\"nb\":%.3f,\"rgb_chroma\":%.3f,"
           "\"hsv_chroma\":%.3f,"
           "\"red_now\":%d,\"yellow_now\":%d,\"blue_now\":%d,"
           "\"ros\":%d,\"ms\":%lu}",
           stateName(robot_state), activeTargetName(), mission_index,
           NUM_MISSION_STOPS, sequence_buf,
           T_CMD, T_STATUS,
           ready_confirmed ? 1 : 0, mission_active ? 1 : 0,
           RETURN_TO_DEPOT_AFTER_FINAL_WORKSTATION ? 1 : 0,
           northbound_red_count, eastbound_red_count, westbound_return_red_count,
           transfer_red_count, transfer_needed_reds,
           x, y, yaw,
           last_line_left, last_line_center, last_line_right,
           last_color_h, last_color_s, last_color_v,
           last_color_nr, last_color_ng, last_color_nb, last_color_chroma,
           last_hsv_chroma,
           last_red_now ? 1 : 0, last_yellow_now ? 1 : 0, last_blue_now ? 1 : 0,
           ros_ready ? 1 : 0, now);

  msg_status.data.data = status_buf;
  msg_status.data.size = strlen(status_buf);
  msg_status.data.capacity = sizeof(status_buf);
  rcl_publish(&pub_status, &msg_status, NULL);
}

const char* stateName(RobotState state) {
  switch (state) {
    case WAIT_FOR_START:
      return ready_confirmed ? "IDLE" : "NOT_READY";
    case DRIVE_NORTH_TO_ENTRY_ROW:
    case DRIVE_EAST_TO_ENTRY_COL:
    case DRIVE_TO_YELLOW_ENTRY:
    case DRIVE_TO_YELLOW_WORKSTATION:
    case REVERSE_TO_WORKSTATION_YELLOW:
    case DRIVE_OUT_TO_YELLOW_ENTRY:
    case DRIVE_EAST_TO_NEXT_ENTRY_COL:
    case DRIVE_WEST_TO_NEXT_ENTRY_COL:
    case DRIVE_NORTH_TO_NEXT_ENTRY_ROW:
    case DRIVE_SOUTH_TO_NEXT_ENTRY_ROW:
    case DRIVE_WEST_TO_RETURN_COL:
    case DRIVE_SOUTH_TO_BLUE:
    case TURN_GENERIC:
      return "MOVING";
    case WORKSTATION_WAIT:
      return "DWELL";
    case DONE:
      return "ARRIVED";
    case EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }

  return "UNKNOWN";
}

const char* activeTargetName() {
  if (target_ws == nullptr) {
    return "";
  }

  return target_ws->name;
}

void formatMissionSequence(char* out, size_t out_size) {
  if (out_size == 0) {
    return;
  }

  out[0] = '\0';

  for (uint8_t i = 0; i < NUM_MISSION_STOPS; i++) {
    char item[6];
    snprintf(item, sizeof(item), "%s%u", (i == 0) ? "" : ",", WORKSTATION_SEQUENCE[i]);
    size_t used = strlen(out);
    if (used >= out_size - 1) {
      break;
    }
    strncat(out, item, out_size - used - 1);
  }
}
