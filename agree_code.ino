// --- Includes ---------------------------------------------------------------
#include <ModbusMaster.h>
#include "motor_control.h"  // writeRegister, readEncoder, pr0AbsoluteMove, writeSteeringPosition, setTractionVelocity, etc.

// ========================= USER TUNABLE MACROS ==============================
// Pins
#define RELAY_PIN           6
#define STEERING_PWM_PIN    8   // RC CH0 (ω)
#define TRACTION_PWM_PIN    7   // RC CH1 (Vx)
#define MODE_TOGGLE_PIN     5   // RC CH2 (mode select)
#define EMERGENCY           4   // Active LOW with INPUT_PULLUP
#define LED_PIN            13
#define RELAY1              9
#define RELAY2             10
#define RELAY3             11
#define RELAY4             12

// Geometry / conversions
#define WHEELBASE_L       0.990f
#define TRACK_W           1.256f
#define MAX_WHEEL_RPM       450

// *** UPDATED: scale so 45° == 500000 pulses (full rotation = 4,000,000 pulses) ***
#define PULSES_PER_DEG   (500000.0f/90.0f)   // = 11111.111... pulses per degree

// Deadbands / thresholds
#define DEADBAND_V        0.15f
#define DEADBAND_W        0.05f
#define OMEGA_PRESENT     0.08f
#define VX_PRESENT        0.05f

// RC pulse read
#define RC_PULSE_TIMEOUT   30000UL
#define RC_LINK_MIN          300

// CH0 (steering/ω) mapping window
#define CH0_LOW_MIN        1069
#define CH0_LOW_MAX        1475
#define CH0_HIGH_MIN       1505
#define CH0_HIGH_MAX       1960

// CH1 (traction/Vx) mapping window
#define CH1_LOW_MIN         994
#define CH1_LOW_MAX        1480
#define CH1_HIGH_MIN       1505
#define CH1_HIGH_MAX       1988

// CH2 (mode toggle)
#define CH2_SWERVE_MIN     1600
#define CH2_DIAG_MAX       1400

#define READ_INTERVAL_MS     500
#define ENFORCED_ARC_ANGLE   45.0f

// *** keep clamp at ±500000 pulses (unchanged) ***
#define MAX_ENCODER_OFFSET_PULSES 500000L

// ========================= STATE / MAPPING ==================================
enum HomingState { HOMING_WAIT, HOMING_TRIM, HOMING_READY };
HomingState homingState = HOMING_WAIT;

// ZERO positions [FL, FR, RL, RR]
long ZERO_ABS_POS[4] = { 0, 0, 0, 0 };

ModbusMaster nodes[8];
unsigned long lastReadTime = 0;

// steering node indices -> nodes[] indices (zero-based)
const uint8_t steering_ids[4] = { 0, 1, 4, 5 };
const char*   steering_tag[4] = { "FL", "FR", "RL", "RR" };

// TRACTION slave ids requested: Modbus IDs 3,4,7,8 -> nodes[] indices 2,3,6,7
// traction_ids mapped as [FL, FR, RL, RR]
const uint8_t traction_ids[4] = { 2, 3, 6, 7 };

// SWERVE math order = [FR, FL, RR, RL]
// steering node_map (FR, FL, RR, RL) -> nodes[] indices for steering
const uint8_t motor_map[4]        = { 1, 0, 5, 4 };
// Map swerve index -> ZERO_ABS_POS index ([FR,FL,RR,RL] -> [1,0,3,2])
const uint8_t swerve_zero_index[4]= { 1, 0, 3, 2 };

// For SWERVE traction mapping (FR, FL, RR, RL) -> traction nodes[] indices
// Using traction_ids mapping [FL,FR,RL,RR] -> nodes indices [2,3,6,7]
// So FR -> traction_ids[1]=3, FL->traction_ids[0]=2, RR->traction_ids[3]=7, RL->traction_ids[2]=6
const uint8_t swerve_traction_node_index[4] = { 3, 2, 7, 6 };

enum ControlMode { DIAGONAL, SWERVE };
ControlMode controlMode = DIAGONAL;

bool eStopActive = false;
bool rcFailActive = false;

void preTransmission() {}
void postTransmission() {}

inline float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }
inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Setup ------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(EMERGENCY, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STEERING_PWM_PIN, INPUT);
  pinMode(TRACTION_PWM_PIN, INPUT);
  pinMode(MODE_TOGGLE_PIN, INPUT);

  pinMode(RELAY1, OUTPUT); pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT); pinMode(RELAY4, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  delay(5000);
  digitalWrite(RELAY1, HIGH); digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH); digitalWrite(RELAY4, HIGH);
  digitalWrite(RELAY_PIN, LOW);

  Serial2.begin(115200);
  Serial3.begin(9600);

  for (uint8_t i = 0; i < 8; i++) {
    nodes[i].begin(i + 1, Serial3);
    nodes[i].preTransmission(preTransmission);
    nodes[i].postTransmission(postTransmission);
  }

  Serial2.println("⚙️ Initializing steering motors...");

  setupSteeringMotor(nodes[0], "Steering FL", "PR0");
  setupSteeringMotor(nodes[1], "Steering FR", "PR0");
  setupSteeringMotor(nodes[4], "Steering RL", "PR0");
  setupSteeringMotor(nodes[5], "Steering RR", "PR0");

  // Traction setup (optional) - initializing traction drives
  setupTractionMotor(nodes[2], "Traction FL", "PR0");  // Modbus ID 3
  setupTractionMotor(nodes[3], "Traction FR", "PR0");  // Modbus ID 4
  setupTractionMotor(nodes[6], "Traction RL", "PR0");  // Modbus ID 7
  setupTractionMotor(nodes[7], "Traction RR", "PR0");  // Modbus ID 8

  for (uint8_t i = 0; i < 4; i++) {
    pr0AbsoluteMove(nodes[steering_ids[i]], ZERO_ABS_POS[i], steering_tag[i]);
  }
}

// --- Main loop --------------------------------------------------------------
void loop() {

  bool emergencyActive = (digitalRead(EMERGENCY) == HIGH);
  if (emergencyActive) {
    if (!eStopActive) {
      eStopActive = true;
      Serial2.println("🚨 EMERGENCY STOP!");
      digitalWrite(LED_PIN, HIGH);
      // halt traction motors immediately
      for (int t = 0; t < 4; t++) setTractionVelocity(nodes[traction_ids[t]], 0);
    }
    delay(10);
    return;
  } else if (eStopActive) {
    eStopActive = false;
    digitalWrite(LED_PIN, LOW);
    Serial2.println("✅ EMERGENCY RELEASED. Homing...");
    for (uint8_t i = 0; i < 4; i++)
      pr0AbsoluteMove(nodes[steering_ids[i]], ZERO_ABS_POS[i], steering_tag[i]);
    homingState = HOMING_WAIT;
  }

  // ------------ RC READ ----------------
  uint16_t pwmSteering = pulseIn(STEERING_PWM_PIN, HIGH, RC_PULSE_TIMEOUT);
  delayMicroseconds(300);
  uint16_t pwmTraction = pulseIn(TRACTION_PWM_PIN, HIGH, RC_PULSE_TIMEOUT);
  delayMicroseconds(300);
  uint16_t pwmToggle   = pulseIn(MODE_TOGGLE_PIN, HIGH, RC_PULSE_TIMEOUT);

  // <-- FIX: declare angle here so it's available later in DIAGONAL block
  int16_t angle = 0;

  if (pwmSteering < RC_LINK_MIN || pwmTraction < RC_LINK_MIN) {
    if (!rcFailActive) {
      rcFailActive = true;
      Serial2.println("🔴 RC LINK LOST.");
      // stop traction on link loss
      for (int t = 0; t < 4; t++) setTractionVelocity(nodes[traction_ids[t]], 0);
    }
    delay(20);
    return;
  } else if (rcFailActive) {
    rcFailActive = false;
    Serial2.println("🟢 RC LINK RESTORED.");
  }

  if (pwmToggle > CH2_SWERVE_MIN) controlMode = SWERVE;
  else if (pwmToggle < CH2_DIAG_MAX) controlMode = DIAGONAL;

  float Vx = 0.0f, omega = 0.0f;

  if (pwmTraction > CH1_HIGH_MIN)
    Vx = mapf(pwmTraction, CH1_HIGH_MIN, CH1_HIGH_MAX, 0, 1.0f);
  else if (pwmTraction < CH1_LOW_MAX)
    Vx = mapf(pwmTraction, CH1_LOW_MIN, CH1_LOW_MAX, -1.0f, 0);

  if (controlMode == SWERVE) {
    if (pwmSteering > CH0_HIGH_MIN)
      omega = mapf(pwmSteering, CH0_HIGH_MIN, CH0_HIGH_MAX, 0, 1.0f);
    else if (pwmSteering < CH0_LOW_MAX)
      omega = mapf(pwmSteering, CH0_LOW_MIN, CH0_LOW_MAX, -1.0f, 0);
  }

  if (fabsf(Vx) < DEADBAND_V) Vx = 0.0f;
  if (fabsf(omega) < DEADBAND_W) omega = 0.0f;

  // ====================== DIAGONAL MODE =====================================
  if (controlMode == DIAGONAL) {

    // use angle (mapped earlier or default 0)
    if (pwmSteering > CH0_HIGH_MIN)
      angle = (int16_t)mapf(pwmSteering, CH0_HIGH_MIN, CH0_HIGH_MAX, 0, 90);
    else if (pwmSteering < CH0_LOW_MAX)
      angle = (int16_t)mapf(pwmSteering, CH0_LOW_MIN, CH0_LOW_MAX, -90, 0);

    int32_t encoderOffset = (int32_t)(PULSES_PER_DEG * (float)angle);

    if (encoderOffset > MAX_ENCODER_OFFSET_PULSES) encoderOffset = MAX_ENCODER_OFFSET_PULSES;
    if (encoderOffset < -MAX_ENCODER_OFFSET_PULSES) encoderOffset = -MAX_ENCODER_OFFSET_PULSES;

    long targetAbs[4];
    for (int i = 0; i < 4; i++) {
      targetAbs[i] = ZERO_ABS_POS[i] + encoderOffset;
      writeSteeringPosition(nodes[steering_ids[i]], targetAbs[i]);
      Serial2.print("➡️  DIAG Steering "); Serial2.print(steering_tag[i]);
      Serial2.print(" -> "); Serial2.println(targetAbs[i]);
      delay(5);
    }
    // Traction: set all wheels the same rpm in diagonal mode
    int rpmAll = (int)(Vx * (float)MAX_WHEEL_RPM);
    for (int i = 0; i < 4; i++) {
      setTractionVelocity(nodes[traction_ids[i]], rpmAll);
      Serial2.print("   DIAG Traction "); Serial2.print(steering_tag[i]);
      Serial2.print(" rpm="); Serial2.println(rpmAll);
    }
  }

  // ======================= SWERVE MODE ======================================
  if (controlMode == SWERVE) {
    // Base IK (A/B/C/D)
    float A = Vx - omega * (WHEELBASE_L * 0.5f);
    float B = Vx + omega * (WHEELBASE_L * 0.5f);
    float C = -omega * (TRACK_W * 0.5f);
    float D =  omega * (TRACK_W * 0.5f);

    // order: [FR, FL, RR, RL]
    float speed[4] = {
      sqrtf(B*B + D*D), sqrtf(B*B + C*C),
      sqrtf(A*A + D*D), sqrtf(A*A + C*C)
    };
    float angleDeg[4] = {
      atan2f(D, B) * 180.0f / PI, atan2f(C, B) * 180.0f / PI,
      atan2f(D, A) * 180.0f / PI, atan2f(C, A) * 180.0f / PI
    };

    // Fold to [-90, +90] and carry speed sign on 180° changes
    for (int i = 0; i < 4; i++) {
      if (angleDeg[i] >  90.0f) { angleDeg[i] -= 180.0f; speed[i] = -speed[i]; }
      if (angleDeg[i] < -90.0f) { angleDeg[i] += 180.0f; speed[i] = -speed[i]; }
    }

    const bool haveOmega = (fabsf(omega) >= OMEGA_PRESENT);
    const bool haveVx    = (fabsf(Vx)    >= VX_PRESENT);

    // --------- ENFORCERS (your exact patterns) ------------------------------
    if (!haveOmega && haveVx) {
      // Pure forward/back ⇒ all 0°
      for (int i = 0; i < 4; i++) angleDeg[i] = 0.0f;

    } else if (haveOmega && haveVx) {
      // ARC SWERVE: fronts ±45, rears ∓45
      const float frontAng = (omega > 0.0f) ? -ENFORCED_ARC_ANGLE : +ENFORCED_ARC_ANGLE;
      const float rearAng  = -frontAng;
      // angles: [FR, FL, RR, RL]
      angleDeg[0] = frontAng;  // FR
      angleDeg[1] = frontAng;  // FL
      angleDeg[2] = rearAng;   // RR
      angleDeg[3] = rearAng;   // RL
      // Traction: fronts follow Vx, rears opposite Vx
      const float sgn = (Vx >= 0.0f) ? +1.0f : -1.0f;
      speed[0] = sgn * fabsf(speed[0]);
      speed[1] = sgn * fabsf(speed[1]);
      speed[2] = sgn * fabsf(speed[2]);
      speed[3] = sgn * fabsf(speed[3]);

    } else if (haveOmega && !haveVx) {
      // IN-PLACE ROTATION (NOT "X"):
      angleDeg[0] = -1 * ENFORCED_ARC_ANGLE;  // FR
      angleDeg[1] = +1 * ENFORCED_ARC_ANGLE;  // FL
      angleDeg[2] = -1 * ENFORCED_ARC_ANGLE;  // RR
      angleDeg[3] = +1 * ENFORCED_ARC_ANGLE;  // RL
      
      // Traction by side: CCW ⇒ right forward, left reverse (CW flips)
      const int sRight = (omega > 0.0f) ? +1 : -1;
      const int sLeft  = sRight;
      speed[0] = sRight * fabsf(speed[0]);  // FR (right)
      speed[2] = -sRight * fabsf(speed[2]);  // RR (right)
      speed[1] = -sLeft  * fabsf(speed[1]);  // FL (left)
      speed[3] = sLeft  * fabsf(speed[3]);  // RL (left)
    }
    // else: no motion ⇒ keep zeros

    // -------------------- Phase 1: STEERING (all first) ---------------------
    long targetAbs[4];
    for (int i = 0; i < 4; i++) {
      int32_t encoderOffset = (int32_t)(PULSES_PER_DEG * angleDeg[i]);

      // clamp offset to safe limit (±500000 pulses for ±90°)
      if (encoderOffset > (int32_t)MAX_ENCODER_OFFSET_PULSES) encoderOffset = (int32_t)MAX_ENCODER_OFFSET_PULSES;
      if (encoderOffset < (int32_t)-MAX_ENCODER_OFFSET_PULSES) encoderOffset = (int32_t)-MAX_ENCODER_OFFSET_PULSES;

      uint8_t zeroIdx       = swerve_zero_index[i]; // FR/FL/RR/RL -> [FL,FR,RL,RR]
      targetAbs[i] = ZERO_ABS_POS[zeroIdx] + encoderOffset;

      writeSteeringPosition(nodes[motor_map[i]], targetAbs[i]);
      Serial2.print("➡️  SWERVE Steering idx "); Serial2.print(i);
      Serial2.print(" -> "); Serial2.println(targetAbs[i]);
      delay(5);
    }

    // -------------------- Phase 2: TRACTION (after steering) ----------------
    for (int i = 0; i < 4; i++) {
      int rpm = (int)roundf(clampf(speed[i], -1.0f, 1.0f) * (float)MAX_WHEEL_RPM);
      rpm = constrain(rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);

      // use swerve_traction_node_index to address correct traction nodes
      setTractionVelocity(nodes[ swerve_traction_node_index[i] ], rpm);
      Serial2.print("   SWERVE Traction idx "); Serial2.print(i);
      Serial2.print(" rpm="); Serial2.println(rpm);
    }
  }

  delay(5);
}
