#include "Arduino_Alvik.h"

Arduino_Alvik alvik;

// =====================================================
// TUNING VALUES
// =====================================================

// Black tape threshold
const int TAPE_THRESHOLD = 200;

// Line following
const float BASE_SPEED = 70.0;       // wheel speed in RPM
const float KP = 28.0;               // line correction gain
const float MAX_CORRECTION = 18.0;   // prevents insane wheel speed difference

// If tape is temporarily hidden by a sticker, drive straight at this speed
const float STICKER_CROSS_SPEED = 24.0;

// Red sticker counting
const unsigned long RED_COOLDOWN_MS = 700;

// Right turn behavior
// If the robot turns left instead of right, change this to +90.0
const float RIGHT_TURN_DEG = -90.0;

// After detecting a corner red sticker, drive forward briefly before turning.
// This helps center the robot near the grid intersection.
const unsigned long TURN_CENTERING_MS = 120;

// After turning, move forward briefly to exit the cross intersection.
const unsigned long POST_TURN_EXIT_MS = 250;
const float POST_TURN_EXIT_SPEED = 45.0;

// Ignore red briefly after turning so the same corner sticker is not counted twice.
const unsigned long RED_IGNORE_AFTER_TURN_MS = 600;

// Ignore blue briefly after the third turn, so it does not accidentally stop immediately.
const unsigned long BLUE_IGNORE_AFTER_TURN_MS = 800;

// Line-loss failsafe
const unsigned long LOST_LINE_FAILSAFE_MS = 1500;

// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState {
  WAIT_FOR_START,
  DRIVE_ROUTE,
  TURN_RIGHT,
  DRIVE_TO_BLUE,
  BLINK_RESULT,
  DONE
};

RobotState robot_state = WAIT_FOR_START;

// =====================================================
// ROUTE LEG TRACKING
// =====================================================
//
// Corrected coordinate convention:
//
// The AGV starts at row 1, col 1.
// The AGV starts facing +Y.
// Therefore, the first movement increases COLUMN.
//
// Route:
//
// Start: row 1, col 1
//
// Leg 1: row 1, col 1 -> row 1, col 8
// Turn right
//
// Leg 2: row 1, col 8 -> row 8, col 8
// Turn right
//
// Leg 3: row 8, col 8 -> row 8, col 1
// Turn right
//
// Leg 4: row 8, col 1 -> row 1, col 1 blue sticker
//

enum RouteLeg {
  LEG_PLUS_Y,
  LEG_PLUS_X,
  LEG_MINUS_Y,
  LEG_MINUS_X_TO_BLUE
};

RouteLeg route_leg = LEG_PLUS_Y;

int current_row = 1;
int current_col = 1;

// =====================================================
// GLOBAL VARIABLES
// =====================================================

int red_count = 0;

bool red_latched = false;
bool blink_done = false;

unsigned long last_red_count_ms = 0;
unsigned long last_turn_finished_ms = 0;
unsigned long lost_line_start_ms = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  alvik.begin();

  Serial.println("Alvik 8x8 grid perimeter route");
  Serial.println("Start robot on blue sticker at row 1, col 1.");
  Serial.println("Robot should start facing +Y direction.");
  Serial.println("Press OK button to start.");

  setLEDBlue();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // Emergency stop with Cancel button
  if (alvik.get_touch_cancel()) {
    alvik.brake();
    setLEDRed();
    Serial.println("Emergency stop.");
    while (true) {
      delay(100);
    }
  }

  switch (robot_state) {

    case WAIT_FOR_START:
      waitForStartState();
      break;

    case DRIVE_ROUTE:
      driveRouteState();
      break;

    case TURN_RIGHT:
      turnRightState();
      break;

    case DRIVE_TO_BLUE:
      driveToBlueState();
      break;

    case BLINK_RESULT:
      alvik.brake();

      if (!blink_done) {
        Serial.print("Finished. Final red sticker count: ");
        Serial.println(red_count);

        blinkRedCount(red_count);

        blink_done = true;
        robot_state = DONE;
      }
      break;

    case DONE:
      alvik.brake();
      setLEDGreen();
      break;
  }

  delay(10);
}

// =====================================================
// WAIT FOR START STATE
// =====================================================

void waitForStartState() {
  alvik.brake();
  setLEDBlue();

  if (alvik.get_touch_ok()) {
    red_count = 0;
    red_latched = false;
    blink_done = false;

    current_row = 1;
    current_col = 1;
    route_leg = LEG_PLUS_Y;

    last_red_count_ms = 0;
    last_turn_finished_ms = 0;
    lost_line_start_ms = 0;

    Serial.println("Starting 8x8 perimeter route...");
    Serial.println("Current position: row 1, col 1");
    Serial.println("Current leg: +Y, row 1 col 1 -> row 1 col 8");

    delay(500);

    robot_state = DRIVE_ROUTE;
  }
}

// =====================================================
// DRIVE ROUTE STATE
// =====================================================

void driveRouteState() {
  int left, center, right;
  float h, s, v;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool blue_now = isBlue(h, s, v);

  followLineOrDriveStraight(left, center, right);

  // Count red stickers and update estimated grid position
  if (countRedIfNeeded(red_now)) {
    updateGridPosition();
    printCurrentGridPosition();

    if (shouldTurnRightHere()) {
      Serial.println("Corner reached. Preparing to turn right.");

      alvik.brake();
      delay(80);

      // Move forward slightly so the robot is closer to the intersection center.
      alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED);
      delay(TURN_CENTERING_MS);

      alvik.brake();
      delay(100);

      robot_state = TURN_RIGHT;
      return;
    }
  }

  checkLineFailsafe(tape_now, red_now, blue_now);
}

// =====================================================
// TURN RIGHT STATE
// =====================================================

void turnRightState() {
  alvik.brake();
  setLEDYellow();
  delay(250);

  Serial.println("Turning right...");

  alvik.rotate(RIGHT_TURN_DEG, DEG, false);
  delay(500);

  alvik.brake();
  delay(150);

  advanceRouteLegAfterTurn();

  // Drive forward briefly to exit the cross intersection.
  alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED);
  delay(POST_TURN_EXIT_MS);

  alvik.brake();
  delay(100);

  red_latched = false;
  lost_line_start_ms = 0;
  last_turn_finished_ms = millis();

  printCurrentLeg();

  if (route_leg == LEG_MINUS_X_TO_BLUE) {
    Serial.println("Final leg active. Drive until blue sticker is detected.");
    robot_state = DRIVE_TO_BLUE;
  } else {
    robot_state = DRIVE_ROUTE;
  }
}

// =====================================================
// FINAL LEG: DRIVE UNTIL BLUE
// =====================================================

void driveToBlueState() {
  int left, center, right;
  float h, s, v;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);

  bool tape_now = isOnTape(left, center, right);
  bool red_now = isRed(h, s, v);
  bool blue_now = isBlue(h, s, v);

  followLineOrDriveStraight(left, center, right);

  // Keep counting red stickers on the final leg.
  // This should count row 7 col 1, row 6 col 1, etc.,
  // until blue is reached at row 1 col 1.
  if (countRedIfNeeded(red_now)) {
    updateGridPosition();
    printCurrentGridPosition();
  }

  // Stop when blue start sticker is detected
  if (millis() - last_turn_finished_ms > BLUE_IGNORE_AFTER_TURN_MS) {
    if (blue_now) {
      alvik.brake();

      Serial.println("Blue start sticker detected. Route complete.");
      Serial.print("Final red sticker count: ");
      Serial.println(red_count);

      delay(300);
      robot_state = BLINK_RESULT;
      return;
    }
  }

  checkLineFailsafe(tape_now, red_now, blue_now);
}

// =====================================================
// RED COUNTING
// =====================================================

bool countRedIfNeeded(bool red_now) {
  // Ignore red briefly after a turn so the same corner sticker does not double-count.
  if (last_turn_finished_ms != 0 && millis() - last_turn_finished_ms < RED_IGNORE_AFTER_TURN_MS) {
    return false;
  }

  if (red_now && !red_latched && millis() - last_red_count_ms > RED_COOLDOWN_MS) {
    red_count++;
    red_latched = true;
    last_red_count_ms = millis();

    Serial.print("Red sticker counted. Total = ");
    Serial.println(red_count);

    flashBothLEDsRedOnce();

    return true;
  }

  if (!red_now) {
    red_latched = false;
  }

  return false;
}

// =====================================================
// GRID POSITION LOGIC
// =====================================================

void updateGridPosition() {
  if (route_leg == LEG_PLUS_Y) {
    current_col++;
  } 
  else if (route_leg == LEG_PLUS_X) {
    current_row++;
  } 
  else if (route_leg == LEG_MINUS_Y) {
    current_col--;
  } 
  else if (route_leg == LEG_MINUS_X_TO_BLUE) {
    current_row--;
  }

  // Keep values inside the 8x8 grid just in case
  current_row = constrain(current_row, 1, 8);
  current_col = constrain(current_col, 1, 8);
}

bool shouldTurnRightHere() {
  // First corner: row 1, col 8
  if (route_leg == LEG_PLUS_Y && current_row == 1 && current_col == 8) {
    return true;
  }

  // Second corner: row 8, col 8
  if (route_leg == LEG_PLUS_X && current_row == 8 && current_col == 8) {
    return true;
  }

  // Third corner: row 8, col 1
  if (route_leg == LEG_MINUS_Y && current_row == 8 && current_col == 1) {
    return true;
  }

  return false;
}

void advanceRouteLegAfterTurn() {
  if (route_leg == LEG_PLUS_Y) {
    route_leg = LEG_PLUS_X;
  } 
  else if (route_leg == LEG_PLUS_X) {
    route_leg = LEG_MINUS_Y;
  } 
  else if (route_leg == LEG_MINUS_Y) {
    route_leg = LEG_MINUS_X_TO_BLUE;
  }
}

void printCurrentGridPosition() {
  Serial.print("Grid position: row ");
  Serial.print(current_row);
  Serial.print(", col ");
  Serial.println(current_col);
}

void printCurrentLeg() {
  if (route_leg == LEG_PLUS_Y) {
    Serial.println("Current leg: +Y, row 1 col 1 -> row 1 col 8");
  } 
  else if (route_leg == LEG_PLUS_X) {
    Serial.println("Current leg: +X, row 1 col 8 -> row 8 col 8");
  } 
  else if (route_leg == LEG_MINUS_Y) {
    Serial.println("Current leg: -Y, row 8 col 8 -> row 8 col 1");
  } 
  else if (route_leg == LEG_MINUS_X_TO_BLUE) {
    Serial.println("Current leg: -X, row 8 col 1 -> blue start sticker at row 1 col 1");
  }
}

// =====================================================
// LINE FOLLOWING
// =====================================================

void followLineOrDriveStraight(int left, int center, int right) {
  bool tape_now = isOnTape(left, center, right);

  // If the tape is temporarily covered by a sticker,
  // keep driving straight slowly to cross the sticker.
  if (!tape_now) {
    alvik.set_wheels_speed(STICKER_CROSS_SPEED, STICKER_CROSS_SPEED);
    return;
  }

  float error = calculateCenterError(left, center, right);
  float correction = error * KP;

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  float left_speed = BASE_SPEED - correction;
  float right_speed = BASE_SPEED + correction;

  alvik.set_wheels_speed(left_speed, right_speed);
}

float calculateCenterError(int left, int center, int right) {
  float sum_weight = left + center + right;

  if (sum_weight <= 0.0) {
    return 0.0;
  }

  // Centroid line following:
  // left = 1, center = 2, right = 3
  float centroid = (left + center * 2.0 + right * 3.0) / sum_weight;

  // Center target is 2.0
  float error = -centroid + 2.0;

  return error;
}

bool isOnTape(int left, int center, int right) {
  return (left > TAPE_THRESHOLD || center > TAPE_THRESHOLD || right > TAPE_THRESHOLD);
}

// =====================================================
// LINE FAILSAFE
// =====================================================

void checkLineFailsafe(bool tape_now, bool red_now, bool blue_now) {
  // Do not treat red/blue stickers as line loss.
  if (!tape_now && !red_now && !blue_now) {
    if (lost_line_start_ms == 0) {
      lost_line_start_ms = millis();
    }

    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS) {
      alvik.brake();
      Serial.println("Failsafe: line lost.");
      robot_state = BLINK_RESULT;
    }
  } else {
    lost_line_start_ms = 0;
  }
}

// =====================================================
// COLOR DETECTION
// =====================================================

bool isRed(float h, float s, float v) {
  // Red wraps around 0/360 degrees.
  // Your measured red was roughly H=359.5, S=0.746, V=0.156.
  bool hue_red = (h > 340.0 || h < 20.0);
  bool saturated = s > 0.40;
  bool bright_enough = v > 0.04;

  return hue_red && saturated && bright_enough;
}

bool isBlue(float h, float s, float v) {
  // Your measured blue was roughly H=208.9, S=0.849, V=0.179.
  bool hue_blue = (h > 190.0 && h < 230.0);
  bool saturated = s > 0.40;
  bool bright_enough = v > 0.04;

  return hue_blue && saturated && bright_enough;
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

void flashBothLEDsRedOnce() {
  setLEDRed();
  delay(80);

  setLEDOff();
  delay(40);
}

void blinkRedCount(int count) {
  delay(500);

  for (int i = 0; i < count; i++) {
    setLEDRed();
    delay(350);

    setLEDOff();
    delay(350);
  }

  setLEDGreen();
}