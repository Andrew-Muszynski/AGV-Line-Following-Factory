/*
 * PD_TUNING
 *
 * Standalone tuning sketch for the Alvik line follower.
 * No ROS, no workstations, no state machine.
 *
 * At intersections (red) and workstation entries (yellow) the line sensors
 * are unreliable. When a marker is detected the controller freezes the last
 * good line error and suppresses the D term for MARKER_BLIND_MS. The P term
 * alone steers through the crossing using the pre-marker error as reference.
 * The color sensor provides early detection; the line sensor threshold catches
 * any gaps the color sensor misses.
 *
 * SERIAL OUTPUT
 * -------------
 *   t(ms)  L  C  R  yaw  err  corr  mode
 *   mode = PD | BLIND
 */

#include "Arduino_Alvik.h"
#include <math.h>

Arduino_Alvik alvik;

// =====================================================
// TUNING CONSTANTS  (match AGV_MULTI_WS_DISPATCH)
// =====================================================

const float KP              = 360.0f;
const float KD              = 4.0f;
const float KP_YAW_BLEND    = 2.0f;   // heading correction on clean tape (fixed reference)
const float KP_YAW_CROSS    = 2.0f;   // heading hold during sticker crossing
const float MAX_CORRECTION  = 20.0f;
const float MAX_WHEEL_RPM   = 70.0f;
const float BASE_SPEED      = 50.0f;

const int   TAPE_THRESHOLD     = 250;
const int   STICKER_LOW_THRESH = 400;

const unsigned long MARKER_BLIND_MS = 350;
const unsigned long BLIND_REARM_MS  = 400;

const unsigned long LOOP_DELAY_MS     = 1;
const unsigned long PRINT_INTERVAL_MS = 50;

// =====================================================
// STATE
// =====================================================

bool  running             = false;
float last_line_error     = 0.0f;
float cross_yaw           = 0.0f;   // heading locked at the moment sticker is detected
float target_yaw          = 0.0f;   // running heading target on clean tape
bool  target_yaw_set      = false;
unsigned long blind_until_ms  = 0;
unsigned long rearm_after_ms  = 0;
unsigned long last_print_ms   = 0;

// =====================================================
// HELPERS
// =====================================================

bool isOnTape(int l, int c, int r) {
  return (l > TAPE_THRESHOLD || c > TAPE_THRESHOLD || r > TAPE_THRESHOLD);
}

float calculateCenterError(int l, int c, int r) {
  float sum = l + c + r;
  if (sum <= 0.0f) return 0.0f;
  return (float)(l - r) / sum;
}

float normalizeYaw(float a) {
  a = fmod(a + 360.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

float yawError(float target, float current) {
  float diff = normalizeYaw(target) - normalizeYaw(current);
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  return diff;
}

bool isRed(float h, float s, float v) {
  return (h > 340.0f || h < 20.0f) && s > 0.40f && v > 0.04f;
}

bool isYellow(float h, float s, float v) {
  return h > 28.0f && h < 55.0f && s > 0.42f && v > 0.07f;
}

bool isBlue(float h, float s, float v) {
  return h > 190.0f && h < 260.0f && s > 0.60f && v > 0.05f;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  alvik.begin();
  alvik.set_illuminator(true);

  Serial.println("=== PD TUNING SKETCH ===");
  Serial.print("KP="); Serial.print(KP);
  Serial.print("  KD="); Serial.println(KD);
  Serial.println("Place Alvik on tape, centered. Press OK to start, CANCEL to stop.");
  Serial.println();
  Serial.println("t(ms)\t\tL\tC\tR\tyaw\terr\tcorr\tmode\th\ts\tv\tcolor");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  if (alvik.get_touch_cancel()) {
    alvik.brake();
    running = false;
    Serial.println("--- STOPPED (CANCEL) ---");
    delay(500);
    return;
  }

  if (!running && alvik.get_touch_ok()) {
    running          = true;
    last_line_error  = 0.0f;
    blind_until_ms   = 0;
    rearm_after_ms   = 0;
    target_yaw_set   = false;
    last_print_ms    = now;
    Serial.println("--- RUNNING ---");
  }

  if (!running) {
    delay(10);
    return;
  }

  // --- Read sensors ---
  int l, c, r;
  float h, s, v;
  float roll, pitch, yaw;
  alvik.get_line_sensors(l, c, r);
  alvik.get_color(h, s, v, HSV);
  alvik.get_orientation(roll, pitch, yaw);

  if (!isOnTape(l, c, r)) {
    alvik.brake();
    running = false;
    Serial.println("--- TAPE LOST ---");
    delay(500);
    return;
  }

  // --- Marker detection ---
  // Blue is the depot sticker — suppress line-sensor threshold to avoid
  // treating it as a crossing marker while color sensor confirms it's blue.
  bool blue   = isBlue(h, s, v);
  bool marker = !blue && (isRed(h, s, v) || isYellow(h, s, v) ||
                (l < STICKER_LOW_THRESH) || (r < STICKER_LOW_THRESH));

  if (marker && now >= rearm_after_ms && now >= blind_until_ms) {
    cross_yaw      = yaw;                 // lock current heading for crossing
    blind_until_ms = now + MARKER_BLIND_MS;
    rearm_after_ms = blind_until_ms + BLIND_REARM_MS;
  }

  // --- Steering ---
  bool blinded = (now < blind_until_ms);
  float error, correction;

  if (blinded) {
    // Hold heading at the moment the sticker was detected
    float yaw_err = yawError(cross_yaw, yaw);
    error      = yaw_err;
    correction = constrain(KP_YAW_CROSS * yaw_err, -MAX_CORRECTION, MAX_CORRECTION);
  } else {
    // Lock target yaw on first clean sample and never update it —
    // on a straight run the heading should be constant.
    if (!target_yaw_set) {
      target_yaw     = yaw;
      target_yaw_set = true;
    }

    float raw_err = calculateCenterError(l, c, r);
    float dt      = LOOP_DELAY_MS / 1000.0f;
    float d_term  = (raw_err - last_line_error) / dt;
    last_line_error = raw_err;
    error = raw_err;

    float yaw_blend = KP_YAW_BLEND * yawError(target_yaw, yaw);
    correction = constrain(raw_err * KP + d_term * KD + yaw_blend,
                           -MAX_CORRECTION, MAX_CORRECTION);
  }

  float left_spd  = constrain(BASE_SPEED - correction, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
  float right_spd = constrain(BASE_SPEED + correction, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
  alvik.set_wheels_speed(left_spd, right_spd, RPM);

  // --- Serial output ---
  if (now - last_print_ms >= PRINT_INTERVAL_MS) {
    last_print_ms = now;
    const char* colorLabel = isRed(h, s, v)    ? "RED"
                           : isYellow(h, s, v) ? "YELLOW"
                           : isBlue(h, s, v)   ? "BLUE"
                           : "---";
    Serial.print(now);          Serial.print("\t\t");
    Serial.print(l);            Serial.print("\t");
    Serial.print(c);            Serial.print("\t");
    Serial.print(r);            Serial.print("\t");
    Serial.print(yaw, 1);       Serial.print("\t");
    Serial.print(error, 3);     Serial.print("\t");
    Serial.print(correction, 3); Serial.print("\t");
    Serial.print(blinded ? "BLIND" : "PD"); Serial.print("\t");
    Serial.print(h, 0);         Serial.print("\t");
    Serial.print(s, 2);         Serial.print("\t");
    Serial.print(v, 2);         Serial.print("\t");
    Serial.println(colorLabel);
  }

  delay(LOOP_DELAY_MS);
}
