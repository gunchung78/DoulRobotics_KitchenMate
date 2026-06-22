#include "pd_control.h"
#include "axis.h"
#include "motion.h"

static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;

static float degToRad(float deg) {
  return deg * DEG_TO_RAD_F;
}

static float radToDeg(float rad) {
  return rad * RAD_TO_DEG_F;
}

static float clampFloat(float value, float minValue, float maxValue) {
  if (value > maxValue) return maxValue;
  if (value < minValue) return minValue;
  return value;
}

// Index [0] is unused.
// offset meaning:
//   M1/M2: degree [deg]
//   M3   : millimeter [mm]
//
// Default design:
//   M1/M2 = internal MIT PD
//   M3    = external PD + software unwrap
//
// For safety, default offset is 0 for every motor.
// It holds the current position at boot instead of moving immediately.
static PDParam pd[4] = {
  // offset, kp,    kd,    baseTorque, torqueLimit
  { 0.000f,  0.0f,  0.00f, 0.00f,      0.0f }, // unused
  { 0.000f, 30.0f,  1.5f,  0.00f,      15.0f }, // M1 internal MIT PD default
  { 0.000f, 15.0f,  1.5f,  0.00f,      15.0f }, // M2 internal MIT PD default
  { 0.000f, 7.5f,   1.5f,  0.00f,      17.0f }  // M3 external PD default
};

static PDControlMode controlMode[4] = {
  PD_MODE_EXTERNAL, // unused
  PD_MODE_INTERNAL, // M1
  PD_MODE_INTERNAL, // M2
  PD_MODE_EXTERNAL  // M3
};

static bool initialized = false;
static float goalPosRad[4] = {0, 0, 0, 0};

static const char* modeName(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return "INVALID";
  return (controlMode[motorId] == PD_MODE_INTERNAL) ? "INTERNAL" : "EXTERNAL";
}

static float computeExternalTorqueRadInternal(uint8_t motorId) {
  float currentPosRad = (motorId == 3) ? m3AxisGetContinuousRad() : fb[motorId].pos;
  float posErrorRad = goalPosRad[motorId] - currentPosRad;

  // Target velocity is currently 0. This term acts as damping/brake.
  float velErrorRadps = 0.0f - fb[motorId].vel;

  float torque =
      pd[motorId].kp * posErrorRad +
      pd[motorId].kd * velErrorRadps +
      pd[motorId].baseTorque;

  torque = clampFloat(torque, -pd[motorId].torqueLimit, pd[motorId].torqueLimit);

  return torque;
}

void pdControlReset() {
  trajectoryClear();
  initialized = false;
  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    goalPosRad[i] = 0.0f;
  }
  Serial.println("[PD] reset goal only. M3 unwrap is kept.");
}

void pdControlSetExternalMotor(uint8_t motorId, float offset, float kp, float kd) {
  if (motorId < 1 || motorId > MOTOR_COUNT) {
    Serial.println("[PD-EXT] invalid motor id. Use 1~3");
    return;
  }

  controlMode[motorId] = PD_MODE_EXTERNAL;
  pd[motorId].offset = offset;
  pd[motorId].kp = kp;
  pd[motorId].kd = kd;

  pdControlReset();
  arrivalReset();

  if (motorId == 3) {
    Serial.printf("[PD-EXT] M3 offset=%.3f mm kp=%.3f kd=%.3f (external PD, unwrap ON)\n",
                  offset, kp, kd);
  } else {
    Serial.printf("[PD-EXT] M%d offset=%.3f deg kp=%.3f kd=%.3f\n",
                  motorId, offset, kp, kd);
  }
}

void pdControlSetInternalMotor(uint8_t motorId, float offset, float kp, float kd) {
  if (motorId < 1 || motorId > MOTOR_COUNT) {
    Serial.println("[IPD] invalid motor id. Use 1~3");
    return;
  }

  // if (motorId == 3) {
  //   Serial.println("[IPD] M3 internal PD is blocked.");
  //   Serial.println("[IPD] Reason: M3 needs software unwrap for multi-turn linear motion.");
  //   Serial.println("[IPD] Use external PD for M3: pd 3 offset_mm kp kd");
  //   return;
  // }

  controlMode[motorId] = PD_MODE_INTERNAL;
  pd[motorId].offset = offset;
  pd[motorId].kp = kp;
  pd[motorId].kd = kd;

  pdControlReset();
  arrivalReset();

  Serial.printf("[IPD] M%d offset=%.3f deg kp=%.3f kd=%.3f (motor internal MIT PD)\n",
                motorId, offset, kp, kd);
}

// Backward compatibility: "pd" means external PD.
void pdControlSetMotor(uint8_t motorId, float offset, float kp, float kd) {
  pdControlSetExternalMotor(motorId, offset, kp, kd);
}

void pdControlSetBaseTorque(uint8_t motorId, float torque) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return;
  pd[motorId].baseTorque = torque;
}

void pdControlClearOffsets() {
  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    pd[i].offset = 0.0f;
  }
  Serial.println("[PD] all offsets cleared");
}

bool pdControlIsInternal(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return false;
  return controlMode[motorId] == PD_MODE_INTERNAL;
}

PDControlMode pdControlGetMode(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return PD_MODE_EXTERNAL;
  return controlMode[motorId];
}

float pdControlGetMitKp(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return pdControlIsInternal(motorId) ? pd[motorId].kp : 0.0f;
}

float pdControlGetMitKd(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return pdControlIsInternal(motorId) ? pd[motorId].kd : 0.0f;
}

void pdControlPrintParams() {
  Serial.println("[PD] params:");
  Serial.println("  pd  id offset kp kd  = external PD torque control");
  Serial.println("  ipd id offset kp kd  = motor internal MIT PD control");
  Serial.println("  M1/M2 offset unit = deg, M3 offset unit = mm");
  Serial.println("  M3 should use external PD because it uses software unwrap.");

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    if (i == 3) {
      Serial.printf("  M3 mode=%s offset=%.3fmm kp=%.3f kd=%.3f base=%.3f extLimit=%.3f\n",
                    modeName(i),
                    pd[i].offset, pd[i].kp, pd[i].kd,
                    pd[i].baseTorque, pd[i].torqueLimit);
    } else {
      Serial.printf("  M%d mode=%s offset=%.3fdeg kp=%.3f kd=%.3f base=%.3f extLimit=%.3f\n",
                    i,
                    modeName(i),
                    pd[i].offset, pd[i].kp, pd[i].kd,
                    pd[i].baseTorque, pd[i].torqueLimit);
    }
  }
}

void pdControlPrintGoal() {
  Serial.printf("[PD] goal M1=%.3fdeg M2=%.3fdeg M3=%.3fmm\n",
                radToDeg(goalPosRad[1]),
                radToDeg(goalPosRad[2]),
                m3AxisRadToMillimeter(goalPosRad[3]));

  Serial.printf("[M3] raw=%.3frad continuous=%.3frad current=%.3fmm goal=%.3fmm\n",
                fb[3].pos,
                m3AxisGetContinuousRad(),
                m3AxisGetContinuousMillimeter(),
                m3AxisRadToMillimeter(goalPosRad[3]));
}


void pdControlUpdate(float targetPos[4], float targetVel[4], float targetTrq[4]) {
  // M3 continuous position must be updated every control cycle after recvFeedback().
  m3AxisUpdate();

  float refPosRad[4] = {0, 0, 0, 0};
  float refVelRadps[4] = {0, 0, 0, 0};

  if (trajectoryHasReference()) {
    trajectoryUpdate();

    for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
      goalPosRad[i] = trajectoryGetGoalRad(i);
      refPosRad[i] = trajectoryGetRefRad(i);
      refVelRadps[i] = trajectoryGetRefVelRadps(i);
    }
  } else {
    if (!initialized) {
      goalPosRad[1] = fb[1].pos + degToRad(pd[1].offset);
      goalPosRad[2] = fb[2].pos + degToRad(pd[2].offset);

      // M3: current continuous rad + mm offset converted to rad.
      goalPosRad[3] = m3AxisGetContinuousRad() + m3AxisMillimeterToRad(pd[3].offset);

      // Soft limit.
      goalPosRad[1] = axisLimitClampGoalRad(1, goalPosRad[1]);
      goalPosRad[2] = axisLimitClampGoalRad(2, goalPosRad[2]);
      goalPosRad[3] = axisLimitClampGoalRad(3, goalPosRad[3]);

      initialized = true;

      Serial.println("[PD] control goal initialized");
      pdControlPrintGoal();
    }

    for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
      refPosRad[i] = goalPosRad[i];
      refVelRadps[i] = 0.0f;
    }
  }

  targetPos[1] = refPosRad[1];
  targetPos[2] = refPosRad[2];
  targetPos[3] = m3AxisWrapRadToMitRange(refPosRad[3]);

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    targetVel[i] = refVelRadps[i];

    if (controlMode[i] == PD_MODE_INTERNAL) {
      targetTrq[i] = pd[i].baseTorque;
    } else {
      float currentPosRad = (i == 3) ? m3AxisGetContinuousRad() : fb[i].pos;
      float posErrorRad = refPosRad[i] - currentPosRad;
      float velErrorRadps = refVelRadps[i] - fb[i].vel;

      float torque =
          pd[i].kp * posErrorRad +
          pd[i].kd * velErrorRadps +
          pd[i].baseTorque;

      targetTrq[i] = clampFloat(torque, -pd[i].torqueLimit, pd[i].torqueLimit);
    }
  }
}


float pdControlGetGoalRad(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return goalPosRad[motorId];
}

float pdControlGetGoalDegree(uint8_t motorId) {
  return radToDeg(pdControlGetGoalRad(motorId));
}

float pdControlGetM3GoalMillimeter() {
  return m3AxisRadToMillimeter(goalPosRad[3]);
}

// Backward-compatible M3 wrappers.
void pdControlM3ResetUnwrap() {
  m3AxisResetUnwrap();
}

float pdControlM3GetContinuousRad() {
  return m3AxisGetContinuousRad();
}

float pdControlM3GetContinuousMillimeter() {
  return m3AxisGetContinuousMillimeter();
}

float pdControlM3GetGoalRad() {
  return goalPosRad[3];
}

float pdControlM3GetGoalMillimeter() {
  return m3AxisRadToMillimeter(goalPosRad[3]);
}

float pdControlM3RadToMeter(float rad) {
  return m3AxisRadToMeter(rad);
}

float pdControlM3MeterToRad(float meter) {
  return m3AxisMeterToRad(meter);
}

float pdControlM3RadpsToMps(float radps) {
  return m3AxisRadpsToMps(radps);
}

float pdControlM3RadToMillimeter(float rad) {
  return m3AxisRadToMillimeter(rad);
}

float pdControlM3MillimeterToRad(float mm) {
  return m3AxisMillimeterToRad(mm);
}

float pdControlM3RadpsToMmps(float radps) {
  return m3AxisRadpsToMmps(radps);
}
