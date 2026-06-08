/*
  BaseAGV_v4.ino  —  Dumb Step-Following AGV

  The AGV has no knowledge of the warehouse layout.  It sits at the depot
  (confirmed by the blue start sticker) and waits for the factory node to
  send a pre-computed sequence of intersection moves.

  ── Command protocol ──────────────────────────────────────────────────────
  Commands arrive on topic  <RobotName>_cmd  as plain strings.

    route F,R,F,L,F   Load move buffer.  Each letter = one intersection:
                         F = go straight   L = turn left   R = turn right
                       Can be sent while IDLE or ARRIVED.

    go                Start executing the loaded move buffer.
                      Transitions IDLE/ARRIVED → MOVING.

    stop              Halt immediately → IDLE. Clears move buffer.

  ── Status reporting ──────────────────────────────────────────────────────
  Published every 200 ms on  <RobotName>_status  as JSON:
    {"state":"IDLE","step":2,"total":6,"x":0.00,"y":0.00,"yaw":0.00,
     "L":120,"C":950,"R":115,"ms":12345}

  state values: NOT_READY | IDLE | MOVING | ARRIVED

  ── Intersection detection ────────────────────────────────────────────────
  When all three line sensors simultaneously read above INTER_THRESH the
  robot is at a grid intersection.  It stops, executes the next buffered
  move (turn or straight), then resumes line-following.  A cooldown timer
  prevents double-triggering on the same intersection.

  ── Startup ───────────────────────────────────────────────────────────────
  The robot must sit on the blue depot sticker for START_CONFIRM_MS ms
  before it becomes ready.  This ensures it knows its true start pose.
*/

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <Arduino_Alvik.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// ── State machine ────────────────────────────────────────────────────────
typedef enum {
  STATE_NOT_READY,   // waiting for blue-sticker startup confirmation
  STATE_IDLE,        // at depot/position, no active route
  STATE_MOVING,      // executing move buffer
  STATE_ARRIVED,     // move buffer exhausted, stopped at destination
} RobotState;

// ── WiFi / micro-ROS agent ───────────────────────────────────────────────
char WIFI_SSID[]     = "ISECapstone";
char WIFI_PASSWORD[] = "j0shf1sh";
char AGENT_IP[]      = "192.168.0.116";
const uint32_t AGENT_PORT  = 8888;

// ── Robot identity ───────────────────────────────────────────────────────
char ROBOT_NAME[16] = "Alvik1";
char T_STATUS[32];
char T_CMD[32];

// ── micro-ROS handles ────────────────────────────────────────────────────
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;
rclc_executor_t    executor;
rcl_publisher_t    pub_status;
rcl_subscription_t sub_cmd;
std_msgs__msg__String msg_status;
char cmd_buf[256];
std_msgs__msg__String msg_cmd_in;

// ── Alvik ────────────────────────────────────────────────────────────────
Arduino_Alvik alvik;

// ── Line-following parameters ─────────────────────────────────────────────
const int   TAPE_MIN     = 100;    // any tape detection threshold
const int   INTER_THRESH = 250;    // all-3 threshold for intersection
const float BASE_SPEED   = 35.0f;
const float TURN_SPEED   = 34.0f;
const float SAFE_STOP_CM = 7.0f;
const float KP           = 0.035f;
const float KD           = 0.20f;

int   L = 0, C = 0, R = 0;
float last_error = 0.0f;
bool  safety_hold = false;

// ── Intersection cooldown ─────────────────────────────────────────────────
const unsigned long INTER_COOLDOWN_MS = 1200;
unsigned long last_inter_ms = 0;

// ── Turn control ──────────────────────────────────────────────────────────
bool  forced_turn_active   = false;
int   forced_turn_dir      = 0;       // +1 = right (CW), -1 = left (CCW)
unsigned long forced_turn_start_ms = 0;
const unsigned long TURN_MIN_MS    = 220;
const unsigned long TURN_MAX_MS    = 2200;
static unsigned long center_reacq_ms = 0;
const unsigned long CENTER_REACQ_MS  = 110;

// ── Move buffer ───────────────────────────────────────────────────────────
#define MAX_MOVES 128
char move_buf[MAX_MOVES];
int  move_count = 0;
int  move_idx   = 0;

// ── Cached sensor readings (updated each loop) ───────────────────────────
float g_tof_fl = 0.0f, g_tof_fcl = 0.0f, g_tof_fc = 0.0f;
float g_tof_fcr = 0.0f, g_tof_fr = 0.0f;
bool  g_obj_detected = false;
float g_battery_pct  = 100.0f;

// ── Global state ──────────────────────────────────────────────────────────
RobotState robot_state = STATE_NOT_READY;
int my_alvik_id = 1;

// ── Startup confirmation ──────────────────────────────────────────────────
bool ready_confirmed = false;
unsigned long blue_confirm_start_ms = 0;
const unsigned long START_CONFIRM_MS = 2000;
const float START_S_MIN = 0.45f;
const float START_V_MIN = 0.05f;
const float START_V_MAX = 0.35f;
float start_yaw_deg = 0.0f;

// ── Robot ID from MAC ─────────────────────────────────────────────────────
int getAlvikID() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "3C:84:27:C3:EA:EC") return 1;
  if (mac == "3C:84:27:C2:BC:40") return 2;
  if (mac == "74:4D:BD:A2:1D:C0") return 3;
  return 1;
}

// ── Utilities ─────────────────────────────────────────────────────────────
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void setWheels(float l, float r) {
  const float MAX_CMD = 60.0f;
  const float MIN_CMD = 8.0f;
  if (fabsf(l) > 1e-3f && fabsf(l) < MIN_CMD) l = (l > 0) ? MIN_CMD : -MIN_CMD;
  if (fabsf(r) > 1e-3f && fabsf(r) < MIN_CMD) r = (r > 0) ? MIN_CMD : -MIN_CMD;
  alvik.set_wheels_speed(clampf(l, -MAX_CMD, MAX_CMD),
                         clampf(r, -MAX_CMD, MAX_CMD));
}

void stopRobot() { alvik.brake(); }

float normDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ── LED state indicators ──────────────────────────────────────────────────
void setLEDState(RobotState s) {
  switch (s) {
    case STATE_NOT_READY:
      alvik.left_led.set_color(1, 0, 1); alvik.right_led.set_color(1, 0, 1); break; // magenta
    case STATE_IDLE:
      alvik.left_led.set_color(0, 0, 1); alvik.right_led.set_color(0, 0, 1); break; // blue
    case STATE_MOVING:
      alvik.left_led.set_color(0, 1, 0); alvik.right_led.set_color(0, 1, 0); break; // green
    case STATE_ARRIVED:
      alvik.left_led.set_color(1, 1, 0); alvik.right_led.set_color(1, 1, 0); break; // yellow
  }
}

// ── HSV helpers ───────────────────────────────────────────────────────────
void readHSV(float &h, float &s, float &v) { alvik.get_color(h, s, v, HSV); }

static inline bool hueInBlue(float h) { return (h >= 190.0f && h <= 240.0f); }

const char* classifyColor(float h, float s, float v) {
  if (v < 0.05f)                         return "BLACK";
  if (s < 0.15f)                         return "WHITE";
  if (hueInBlue(h))                      return "BLUE";
  if (h < 15.0f || h >= 345.0f)          return "RED";
  if (h < 45.0f)                         return "ORANGE";
  if (h < 75.0f)                         return "YELLOW";
  if (h < 165.0f)                        return "GREEN";
  return "PURPLE";
}

// ── Sensor cache update (call once per loop) ──────────────────────────────
void update_sensors() {
  alvik.get_distance(g_tof_fl, g_tof_fcl, g_tof_fc, g_tof_fcr, g_tof_fr, CM);
  g_obj_detected = (g_tof_fl  > 0.0f && g_tof_fl  < SAFE_STOP_CM) ||
                   (g_tof_fcl > 0.0f && g_tof_fcl < SAFE_STOP_CM) ||
                   (g_tof_fc  > 0.0f && g_tof_fc  < SAFE_STOP_CM) ||
                   (g_tof_fcr > 0.0f && g_tof_fcr < SAFE_STOP_CM) ||
                   (g_tof_fr  > 0.0f && g_tof_fr  < SAFE_STOP_CM);
  static unsigned long last_bat_ms = 0;
  if (millis() - last_bat_ms >= 2000) {
    g_battery_pct = alvik.get_battery_charge();
    last_bat_ms   = millis();
  }
}

// ── Startup: wait for robot to sit on blue depot sticker ──────────────────
void processStartConfirmation() {
  stopRobot();
  unsigned long now = millis();
  alvik.get_line_sensors(L, C, R);
  if (C < INTER_THRESH) {
    blue_confirm_start_ms = 0;
    setLEDState(STATE_NOT_READY);
    return;
  }
  float h, s, v;
  readHSV(h, s, v);
  bool looks_blue = hueInBlue(h) &&
                    (s >= START_S_MIN) &&
                    (v >= START_V_MIN) &&
                    (v <= START_V_MAX);
  if (looks_blue) {
    if (blue_confirm_start_ms == 0) blue_confirm_start_ms = now;
    if ((now - blue_confirm_start_ms) >= START_CONFIRM_MS) {
      ready_confirmed = true;
      alvik.reset_pose(0, 0, start_yaw_deg, CM, DEG);
      robot_state = STATE_IDLE;
      setLEDState(STATE_IDLE);
      last_inter_ms = now;   // seed cooldown from depot
    }
  } else {
    blue_confirm_start_ms = 0;
    setLEDState(STATE_NOT_READY);
  }
}

// ── Turn execution ────────────────────────────────────────────────────────
void beginTurn(int dir) {
  forced_turn_active   = true;
  forced_turn_dir      = dir;
  forced_turn_start_ms = millis();
  center_reacq_ms      = 0;
  setWheels(dir > 0 ?  TURN_SPEED : -TURN_SPEED,
            dir > 0 ? -TURN_SPEED :  TURN_SPEED);
}

// Returns true when the turn has completed and line is reacquired.
bool processTurn() {
  if (!forced_turn_active) return true;
  unsigned long elapsed = millis() - forced_turn_start_ms;

  if (elapsed > TURN_MAX_MS) {           // safety timeout
    forced_turn_active = false;
    center_reacq_ms = 0;
    return true;
  }
  if (elapsed < TURN_MIN_MS) {           // enforce minimum rotation
    setWheels(forced_turn_dir > 0 ?  TURN_SPEED : -TURN_SPEED,
              forced_turn_dir > 0 ? -TURN_SPEED :  TURN_SPEED);
    return false;
  }
  // After min duration: wait for centre sensor to re-acquire the new tape line
  alvik.get_line_sensors(L, C, R);
  if (C > INTER_THRESH) {
    if (center_reacq_ms == 0) center_reacq_ms = millis();
    if (millis() - center_reacq_ms >= CENTER_REACQ_MS) {
      forced_turn_active = false;
      center_reacq_ms    = 0;
      last_error         = 0.0f;
      return true;
    }
  } else {
    center_reacq_ms = 0;
  }
  setWheels(forced_turn_dir > 0 ?  TURN_SPEED : -TURN_SPEED,
            forced_turn_dir > 0 ? -TURN_SPEED :  TURN_SPEED);
  return false;
}

// ── Intersection handler ──────────────────────────────────────────────────
// Called when all 3 sensors go high.  Dequeues the next move and acts on it.
void onIntersection() {
  last_inter_ms = millis();
  char move = move_buf[move_idx++];

  if (move == 'L') {
    beginTurn(-1);
  } else if (move == 'R') {
    beginTurn(+1);
  }
  // 'F': no turn; robot continues straight through the intersection.
  // The cooldown prevents re-triggering while still crossing.

  // If this was the last move AND it is 'F', stop here.
  if (move_idx >= move_count && move == 'F') {
    stopRobot();
    robot_state = STATE_ARRIVED;
    setLEDState(STATE_ARRIVED);
  }
}

// ── Main movement loop ────────────────────────────────────────────────────
void followStep() {
  // Obstacle safety — g_obj_detected is refreshed by update_sensors() each loop
  if (g_obj_detected) {
    if (!safety_hold) { stopRobot(); safety_hold = true; }
    return;
  }
  safety_hold = false;

  // Let the turn FSM run if active
  if (forced_turn_active) {
    bool done = processTurn();
    if (done && move_idx >= move_count) {
      stopRobot();
      robot_state = STATE_ARRIVED;
      setLEDState(STATE_ARRIVED);
    }
    return;
  }

  alvik.get_line_sensors(L, C, R);

  // Tape-lost safety: all sensors below minimum means completely off tape — stop
  if (L < TAPE_MIN && C < TAPE_MIN && R < TAPE_MIN) {
    stopRobot();
    robot_state = STATE_IDLE;
    alvik.left_led.set_color(1, 0.3f, 0);   // orange = tape lost
    alvik.right_led.set_color(1, 0.3f, 0);
    return;
  }

  // Intersection detection: all 3 sensors above threshold + cooldown elapsed
  bool inter = (L > INTER_THRESH) && (C > INTER_THRESH) && (R > INTER_THRESH);
  bool cool  = (millis() - last_inter_ms) > INTER_COOLDOWN_MS;

  if (inter && cool && move_idx < move_count) {
    onIntersection();
    return;
  }

  // PD line-following
  float weighted = 0.0f * (float)L + 1.0f * (float)C + 2.0f * (float)R;
  float total    = (float)(L + C + R);
  float avg      = (total > 50.0f) ? (weighted / total) : 1.0f;
  float error    = avg - 1.0f;
  float deriv    = error - last_error;
  float corr     = KP * error + KD * deriv;
  last_error     = error;
  setWheels(BASE_SPEED + corr, BASE_SPEED - corr);
}

// ── ROS2 command callback ─────────────────────────────────────────────────
void cmd_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  cmd.trim();

  // "stop" always works regardless of ready state
  if (cmd.equalsIgnoreCase("stop")) {
    stopRobot();
    forced_turn_active = false;
    move_count = 0;
    move_idx   = 0;
    last_error = 0.0f;
    robot_state = STATE_IDLE;
    setLEDState(STATE_IDLE);
    return;
  }

  if (!ready_confirmed) return;

  // "route F,R,F,L,..."  — load move buffer
  if (cmd.length() > 6 && cmd.substring(0, 6).equalsIgnoreCase("route ")) {
    String moves_str = cmd.substring(6);
    moves_str.trim();
    move_count = 0;
    move_idx   = 0;
    for (int i = 0; i < (int)moves_str.length() && move_count < MAX_MOVES; i++) {
      char ch = (char)toupper((unsigned char)moves_str.charAt(i));
      if (ch == 'F' || ch == 'L' || ch == 'R') {
        move_buf[move_count++] = ch;
      }
    }
    return;
  }

  // "go"  — begin executing loaded buffer
  if (cmd.equalsIgnoreCase("go")) {
    if (move_count > 0 &&
        (robot_state == STATE_IDLE || robot_state == STATE_ARRIVED)) {
      move_idx           = 0;
      forced_turn_active = false;
      last_error         = 0.0f;
      last_inter_ms      = millis();  // suppress detection at current position
      robot_state = STATE_MOVING;
      setLEDState(STATE_MOVING);
    }
    return;
  }
}

// ── Status publisher ──────────────────────────────────────────────────────
unsigned long last_status_ms = 0;

void publish_status(unsigned long now) {
  if (now - last_status_ms < 200) return;
  last_status_ms = now;

  float x, y, th;
  alvik.get_pose(x, y, th, CM, DEG);
  float roll, pitch, yaw;
  alvik.get_orientation(roll, pitch, yaw);
  float yaw_deg = normDeg((fabsf(yaw) <= 3.2f) ? yaw * 57.2957795f : yaw);
  alvik.get_line_sensors(L, C, R);

  float h_c, s_c, v_c;
  readHSV(h_c, s_c, v_c);
  const char* col_str = classifyColor(h_c, s_c, v_c);

  const char *state_str =
      (robot_state == STATE_NOT_READY) ? "NOT_READY" :
      (robot_state == STATE_IDLE)      ? "IDLE"      :
      (robot_state == STATE_MOVING)    ? "MOVING"    : "ARRIVED";

  static char status_buf[400];
  snprintf(status_buf, sizeof(status_buf),
    "{\"state\":\"%s\",\"step\":%d,\"total\":%d,"
    "\"x\":%.2f,\"y\":%.2f,\"yaw\":%.1f,"
    "\"L\":%d,\"C\":%d,\"R\":%d,"
    "\"tof\":[%.1f,%.1f,%.1f,%.1f,%.1f],"
    "\"obj\":%d,\"bat\":%.0f,\"color\":\"%s\",\"ms\":%lu}",
    state_str, move_idx, move_count,
    x, y, yaw_deg, L, C, R,
    g_tof_fl, g_tof_fcl, g_tof_fc, g_tof_fcr, g_tof_fr,
    g_obj_detected ? 1 : 0, g_battery_pct, col_str, now);

  msg_status.data.data     = status_buf;
  msg_status.data.size     = strlen(status_buf);
  msg_status.data.capacity = msg_status.data.size + 1;
  rcl_publish(&pub_status, &msg_status, NULL);
}

// ── Transport + graph init ────────────────────────────────────────────────
void init_transport() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    static bool blink = false;
    alvik.left_led.set_color(0, blink ? 0.5f : 0, blink ? 0.5f : 0);
    alvik.right_led.set_color(0, blink ? 0.5f : 0, blink ? 0.5f : 0);
    blink = !blink;
    delay(300);
  }
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
}

bool init_graph() {
  allocator = rcl_get_default_allocator();
  if (rmw_uros_ping_agent(1000, 5)                          != RMW_RET_OK) return false;
  if (rclc_support_init(&support, 0, NULL, &allocator)      != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, ROBOT_NAME, "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &pub_status, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_STATUS) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        T_CMD) != RCL_RET_OK) return false;

  msg_cmd_in.data.data     = cmd_buf;
  msg_cmd_in.data.size     = 0;
  msg_cmd_in.data.capacity = sizeof(cmd_buf);

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(
        &executor, &sub_cmd, &msg_cmd_in,
        &cmd_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  msg_status.data.data     = NULL;
  msg_status.data.size     = 0;
  msg_status.data.capacity = 0;
  return true;
}

// ── Arduino entry points ──────────────────────────────────────────────────
void setup() {
  alvik.begin();
  stopRobot();
  alvik.set_illuminator(true);

  my_alvik_id = getAlvikID();
  snprintf(ROBOT_NAME, sizeof(ROBOT_NAME), "Alvik%d", my_alvik_id);
  snprintf(T_STATUS,   sizeof(T_STATUS),   "%s_status", ROBOT_NAME);
  snprintf(T_CMD,      sizeof(T_CMD),      "%s_cmd",    ROBOT_NAME);

  float roll, pitch, yaw;
  alvik.get_orientation(roll, pitch, yaw);
  start_yaw_deg = normDeg((fabsf(yaw) <= 3.2f) ? yaw * 57.2957795f : yaw);

  init_transport();

  if (!init_graph()) {
    // Flash red on failure — spin forever
    while (true) {
      alvik.left_led.set_color(1, 0, 0); alvik.right_led.set_color(1, 0, 0);
      delay(400);
      alvik.left_led.set_color(0, 0, 0); alvik.right_led.set_color(0, 0, 0);
      delay(400);
    }
  }

  robot_state = STATE_NOT_READY;
  setLEDState(STATE_NOT_READY);
  ready_confirmed     = false;
  blue_confirm_start_ms = 0;
}

void loop() {
  unsigned long now = millis();

  if (!ready_confirmed) {
    processStartConfirmation();
    publish_status(now);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    delay(5);
    return;
  }

  update_sensors();

  switch (robot_state) {
    case STATE_IDLE:
    case STATE_ARRIVED:
      stopRobot();
      break;
    case STATE_MOVING:
      followStep();
      break;
    default:
      break;
  }

  publish_status(now);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(5);
}
