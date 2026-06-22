#include "axis.h"

// ============================================================
//  Common constants
// ============================================================

static constexpr float TWO_PI_F     = 6.2831853071795864769f;
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

// ============================================================
//  M3 mechanical conversion
// ============================================================
//
// IMPORTANT:
//   Current mechanism:
//     1 revolution = 360deg = 2*pi rad = 5mm linear movement.
//
// Change only these two values if the mechanism changes again.
// Do not duplicate these constants in pd_control.cpp or motion.cpp.

static constexpr float AXIS_M3_LINEAR_MM_PER_REV = 5.0f;
static constexpr float AXIS_M3_LINEAR_M_PER_REV  = 0.005f;

static constexpr float AXIS_M3_MM_PER_RAD = AXIS_M3_LINEAR_MM_PER_REV / TWO_PI_F;
static constexpr float AXIS_M3_M_PER_RAD  = AXIS_M3_LINEAR_M_PER_REV  / TWO_PI_F;

// ============================================================
//  M3 unwrap state
// ============================================================

static constexpr float MIT_POS_RANGE_RAD = P_MAX - P_MIN;

// If raw position jumps more than about half of the MIT range in one sample,
// treat it as a wrap from +720deg to -720deg, or vice versa.
static constexpr float M3_WRAP_THRESHOLD_RAD = MIT_POS_RANGE_RAD * 0.45f;

static bool  m3Initialized   = false;
static float m3PrevRawRad    = 0.0f;
static float m3WrapOffsetRad = 0.0f;
static float m3ContinuousRad = 0.0f;

// ============================================================
//  Utility
// ============================================================

static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

// ============================================================
//  M3 conversion - new names
// ============================================================

float axisM3RadToMm(float rad) {
  return rad * AXIS_M3_MM_PER_RAD;
}

float axisM3MmToRad(float mm) {
  return mm / AXIS_M3_MM_PER_RAD;
}

float axisM3RadpsToMmps(float radps) {
  return radps * AXIS_M3_MM_PER_RAD;
}

float axisM3RadToMeter(float rad) {
  return rad * AXIS_M3_M_PER_RAD;
}

float axisM3MeterToRad(float meter) {
  return meter / AXIS_M3_M_PER_RAD;
}

float axisM3RadpsToMps(float radps) {
  return radps * AXIS_M3_M_PER_RAD;
}

// ============================================================
//  MIT position wrap - new names
// ============================================================

float axisWrapRadToMitRange(float rad) {
  while (rad > P_MAX) rad -= MIT_POS_RANGE_RAD;
  while (rad < P_MIN) rad += MIT_POS_RANGE_RAD;

  if (rad > P_MAX) rad = P_MAX;
  if (rad < P_MIN) rad = P_MIN;
  return rad;
}

// ============================================================
//  M3 unwrap - new names
// ============================================================

void axisM3Update() {
  float rawRad = fb[3].pos;

  if (!m3Initialized) {
    m3PrevRawRad = rawRad;
    m3WrapOffsetRad = 0.0f;
    m3ContinuousRad = rawRad;
    m3Initialized = true;
    return;
  }

  float delta = rawRad - m3PrevRawRad;

  // raw: +12.5 -> -12.5. Actual continuous position should keep increasing.
  if (delta < -M3_WRAP_THRESHOLD_RAD) {
    m3WrapOffsetRad += MIT_POS_RANGE_RAD;
    Serial.printf("[M3] unwrap forward. raw jump=%.3f, offset=%.3f rad\n",
                  delta, m3WrapOffsetRad);
  }
  // raw: -12.5 -> +12.5. Actual continuous position should keep decreasing.
  else if (delta > M3_WRAP_THRESHOLD_RAD) {
    m3WrapOffsetRad -= MIT_POS_RANGE_RAD;
    Serial.printf("[M3] unwrap backward. raw jump=%.3f, offset=%.3f rad\n",
                  delta, m3WrapOffsetRad);
  }

  m3ContinuousRad = rawRad + m3WrapOffsetRad;
  m3PrevRawRad = rawRad;
}

void axisM3ResetUnwrap() {
  m3Initialized = false;
  m3PrevRawRad = 0.0f;
  m3WrapOffsetRad = 0.0f;
  m3ContinuousRad = fb[3].pos;
  Serial.println("[M3] unwrap reset. Current raw position becomes the new continuous reference.");
}

float axisM3GetContinuousRad() {
  return m3ContinuousRad;
}

float axisM3GetContinuousMm() {
  return axisM3RadToMm(m3ContinuousRad);
}

// ============================================================
//  Soft limits - new names
// ============================================================

float axisClampDeg(uint8_t motorId, float targetDeg) {
  if (motorId == 1) {
    return clampFloat(targetDeg, LIMIT_M1.minDeg, LIMIT_M1.maxDeg);
  }

  if (motorId == 2) {
    return clampFloat(targetDeg, LIMIT_M2.minDeg, LIMIT_M2.maxDeg);
  }

  return targetDeg;
}

float axisClampM3Mm(float targetMm) {
  return clampFloat(targetMm, LIMIT_M3.minMm, LIMIT_M3.maxMm);
}

float axisClampGoalRad(uint8_t motorId, float targetRad) {
  if (motorId == 1 || motorId == 2) {
    float targetDeg = targetRad * RAD_TO_DEG_F;
    float limitedDeg = axisClampDeg(motorId, targetDeg);

    if (limitedDeg != targetDeg) {
      Serial.printf("[LIMIT] M%d target limited: %.2fdeg -> %.2fdeg\n",
                    motorId, targetDeg, limitedDeg);
    }

    return limitedDeg * DEG_TO_RAD_F;
  }

  if (motorId == 3) {
    float targetMm = axisM3RadToMm(targetRad);
    float limitedMm = axisClampM3Mm(targetMm);

    if (limitedMm != targetMm) {
      Serial.printf("[LIMIT] M3 target limited: %.2fmm -> %.2fmm\n",
                    targetMm, limitedMm);
    }

    return axisM3MmToRad(limitedMm);
  }

  return targetRad;
}

void axisPrintLimits() {
  Serial.println("[LIMIT] Soft limits:");
  Serial.printf("  M1: %.2fdeg ~ %.2fdeg\n", LIMIT_M1.minDeg, LIMIT_M1.maxDeg);
  Serial.printf("  M2: %.2fdeg ~ %.2fdeg\n", LIMIT_M2.minDeg, LIMIT_M2.maxDeg);
  Serial.printf("  M3: %.2fmm  ~ %.2fmm\n",  LIMIT_M3.minMm,  LIMIT_M3.maxMm);
}

// ============================================================
//  Backward-compatible old m3_axis names
// ============================================================

void m3AxisUpdate() {
  axisM3Update();
}

void m3AxisResetUnwrap() {
  axisM3ResetUnwrap();
}

float m3AxisGetContinuousRad() {
  return axisM3GetContinuousRad();
}

float m3AxisGetContinuousMillimeter() {
  return axisM3GetContinuousMm();
}

float m3AxisRadToMillimeter(float rad) {
  return axisM3RadToMm(rad);
}

float m3AxisMillimeterToRad(float mm) {
  return axisM3MmToRad(mm);
}

float m3AxisRadpsToMmps(float radps) {
  return axisM3RadpsToMmps(radps);
}

float m3AxisRadToMeter(float rad) {
  return axisM3RadToMeter(rad);
}

float m3AxisMeterToRad(float meter) {
  return axisM3MeterToRad(meter);
}

float m3AxisRadpsToMps(float radps) {
  return axisM3RadpsToMps(radps);
}

float m3AxisWrapRadToMitRange(float rad) {
  return axisWrapRadToMitRange(rad);
}

// ============================================================
//  Backward-compatible old axis_limit names
// ============================================================

float axisLimitClampDeg(uint8_t motorId, float targetDeg) {
  return axisClampDeg(motorId, targetDeg);
}

float axisLimitClampM3Mm(float targetMm) {
  return axisClampM3Mm(targetMm);
}

float axisLimitClampGoalRad(uint8_t motorId, float targetRad) {
  return axisClampGoalRad(motorId, targetRad);
}

void axisLimitPrint() {
  axisPrintLimits();
}
