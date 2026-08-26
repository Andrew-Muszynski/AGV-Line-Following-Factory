#include "Arduino_Alvik.h"
#include <math.h>

Arduino_Alvik alvik;

// =====================================================
// TUNING VALUES
// =====================================================

// Black tape threshold
const int TAPE_THRESHOLD = 200;

// Line following. These are the same values that worked on your perimeter test.
const float BASE_SPEED = 70.0;                  // Normal wheel speed in RPM
const float YELLOW_SEARCH_SPEED = 40.0;         // Slower speed when the next target is yellow
const float WORKSTATION_APPROACH_SPEED = 28.0;  // Slower speed from entry yellow to workstation yellow
const float KP = 28.0;                          // Line correction gain
const float MAX_CORRECTION = 18.0;              // Prevents excessive wheel speed difference

// If tape is temporarily hidden by a sticker, drive straight at this speed
const float STICKER_CROSS_SPEED = 24.0;

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
const float TURN_MAX_SPEED = 70.0;          // RPM
const unsigned long TURN_CONTROL_MS = 5;    // update period

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

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  alvik.begin();
  alvik.reset_pose(0, 0, 0, CM, DEG);

  Serial.println("Alvik workstation route");
  Serial.println("Start on the blue sticker facing NORTH.");
  Serial.println("Press OK to start.");
  Serial.println();

  setLEDBlue();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  if (alvik.get_touch_cancel() && robot_state != EMERGENCY_STOP) {
    alvik.brake();
    robot_state = EMERGENCY_STOP;
  }

  switch (robot_state) {
    case WAIT_FOR_START:
      waitForStartState();
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
      setLEDGreen();
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
  setLEDBlue();

  if (alvik.get_touch_ok()) {
    red_count = 0;
    emergency_printed = false;
    lost_line_start_ms = 0;
    marker_ignore_until_ms = millis() + 500;
    resetMarkerStable();

    alvik.reset_pose(0, 0, 0, CM, DEG);

    Serial.println("Starting workstation route.");
    Serial.println("Step 1: Drive NORTH to the first red sticker.");

    robot_state = DRIVE_TO_RED_1;
  }
}

// =====================================================
// ROUTE STATES
// =====================================================

void driveToRed1State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 1: first red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED 1. Rotate right toward the entry yellow marker.");

    beginTurn(RIGHT_TURN_DEG, DRIVE_TO_YELLOW_ENTRY, TURN_CENTERING_MS, YELLOW_POST_TURN_EXIT_MS,
              NO_MARKER_IGNORE_MS, "right turn after RED 1");
  }
}

void driveToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW ENTRY: first yellow marker", YELLOW_SEARCH_SPEED)) {
    Serial.println("At first YELLOW. Rotate right into the workstation spur.");

    beginTurn(RIGHT_TURN_DEG, DRIVE_TO_YELLOW_WORKSTATION, TURN_CENTERING_MS, YELLOW_POST_TURN_EXIT_MS,
              YELLOW_ENTRY_DEPART_IGNORE_MS, "right turn into workstation spur");
  }
}

void driveToYellowWorkstationState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW WORKSTATION: second yellow marker",
                             WORKSTATION_APPROACH_SPEED)) {
    Serial.println("At workstation YELLOW. Correcting to absolute yaw 0.0 before reverse docking.");

    beginTurnToYaw(REVERSE_DOCK_ABSOLUTE_YAW, REVERSE_TO_WORKSTATION_YELLOW, 0, 0,
                   YELLOW_IGNORE_AFTER_YAW_ALIGN_MS, "align to yaw 0 before reverse docking");
  }
}

void reverseToWorkstationYellowState() {
  if (reverseStraightUntilColor(TARGET_YELLOW, "workstation yellow docking marker")) {
    Serial.println("Docked at workstation yellow. Waiting 5 seconds.");
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
    Serial.println("Workstation wait complete. Drive forward back to the first yellow marker.");

    marker_ignore_until_ms = millis() + EXIT_WORKSTATION_IGNORE_MS;
    resetMarkerStable();
    setLEDGreen();

    robot_state = DRIVE_OUT_TO_YELLOW_ENTRY;
  }
}

void driveOutToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, "YELLOW ENTRY: returning from workstation", YELLOW_SEARCH_SPEED)) {
    Serial.println("Back at first YELLOW. Rotate right toward RED 2.");

    beginTurn(RIGHT_TURN_DEG, DRIVE_TO_RED_2, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
              MARKER_IGNORE_AFTER_TURN_MS, "right turn toward RED 2");
  }
}

void driveToRed2State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 2: top-right red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED 2. Rotate right toward RED 3.");

    beginTurn(RIGHT_TURN_DEG, DRIVE_TO_RED_3, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
              MARKER_IGNORE_AFTER_TURN_MS, "right turn after RED 2");
  }
}

void driveToRed3State() {
  if (driveForwardUntilColor(TARGET_RED, "RED 3: bottom-right red intersection", BASE_SPEED)) {
    red_count++;
    Serial.println("At RED 3. Rotate right toward the blue start marker.");

    beginTurn(RIGHT_TURN_DEG, DRIVE_TO_BLUE, TURN_CENTERING_MS, POST_TURN_EXIT_MS,
              MARKER_IGNORE_AFTER_TURN_MS, "right turn after RED 3");
  }
}

void driveToBlueState() {
  if (driveForwardUntilColor(TARGET_BLUE, "BLUE: starting marker", BASE_SPEED)) {
    Serial.println("Blue start marker reached. Rotate right to face NORTH again.");

    beginTurn(RIGHT_TURN_DEG, DONE, 0, 0,
              MARKER_IGNORE_AFTER_TURN_MS, "final right turn at blue");
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
    alvik.brake();
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
    alvik.set_wheels_speed(STICKER_CROSS_SPEED, STICKER_CROSS_SPEED, RPM);
    return;
  }

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
               unsigned long marker_ignore_ms,
               const char* label) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute = false;

  pending_turn_angle = angle;
  after_turn_state = next_state;
  pending_center_ms = center_ms;
  pending_post_exit_ms = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;
  pending_turn_label = label;

  turn_phase = TURN_PRE_CENTER_BRAKE;
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
          startRotateRelative(pending_turn_angle);
        }

        turn_phase = TURN_ROTATING;
      }
      break;

    case TURN_ROTATING:
      setLEDYellow();

      if (updateRotateTo()) {
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

  Serial.print("Start yaw: ");
  Serial.print(turn_start_yaw, 2);
  Serial.print(" deg | Absolute target yaw: ");
  Serial.print(turn_target_yaw, 2);
  Serial.println(" deg");
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

    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS) {
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
// DEBUG HELPERS
// =====================================================

void printStateName(RobotState state) {
  switch (state) {
    case WAIT_FOR_START:
      Serial.println("WAIT_FOR_START");
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