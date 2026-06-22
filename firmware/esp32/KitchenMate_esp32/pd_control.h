#pragma once
#include "config.h"

/*
 * External/Internal PD control module
 *
 * Unit policy:
 *   - M1/M2 user input and print unit: degree [deg]
 *   - M3    user input and print unit: millimeter [mm]
 *   - Internal calculation unit for ALL motors: rad / rad/s
 *
 * Commands:
 *   pd  id offset kp kd   -> external PD torque control
 *   ipd id offset kp kd   -> motor internal MIT PD control
 *
 * Recommended mode:
 *   - M1/M2: internal MIT PD
 *   - M3   : external PD + M3 software unwrap
 */

enum PDControlMode : uint8_t {
  PD_MODE_EXTERNAL = 0,
  PD_MODE_INTERNAL = 1
};

struct PDParam {
  float offset;       // M1/M2: [deg], M3: [mm]
  float kp;           // internal mode: MIT kp, external mode: external kp [Nm/rad]
  float kd;           // internal mode: MIT kd, external mode: external kd [Nm/(rad/s)]
  float baseTorque;   // [Nm] feedforward/base torque
  float torqueLimit;  // [Nm] external PD torque limit
};

void pdControlReset();
void pdControlUpdate(float targetPos[4], float targetVel[4], float targetTrq[4]);

// Existing "pd" command uses external PD.
void pdControlSetMotor(uint8_t motorId, float offset, float kp, float kd);
void pdControlSetExternalMotor(uint8_t motorId, float offset, float kp, float kd);
void pdControlSetInternalMotor(uint8_t motorId, float offset, float kp, float kd);

void pdControlSetBaseTorque(uint8_t motorId, float torque);
void pdControlClearOffsets();

bool pdControlIsInternal(uint8_t motorId);
PDControlMode pdControlGetMode(uint8_t motorId);
float pdControlGetMitKp(uint8_t motorId);
float pdControlGetMitKd(uint8_t motorId);

void pdControlPrintGoal();
void pdControlPrintParams();

float pdControlGetGoalRad(uint8_t motorId);
float pdControlGetGoalDegree(uint8_t motorId);
float pdControlGetM3GoalMillimeter();

// Backward-compatible M3 helper wrappers.
void pdControlM3ResetUnwrap();
float pdControlM3GetContinuousRad();
float pdControlM3GetContinuousMillimeter();
float pdControlM3GetGoalRad();
float pdControlM3GetGoalMillimeter();

float pdControlM3RadToMeter(float rad);
float pdControlM3MeterToRad(float meter);
float pdControlM3RadpsToMps(float radps);

float pdControlM3RadToMillimeter(float rad);
float pdControlM3MillimeterToRad(float mm);
float pdControlM3RadpsToMmps(float radps);
