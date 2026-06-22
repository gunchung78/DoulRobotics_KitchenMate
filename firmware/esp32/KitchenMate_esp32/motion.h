#pragma once
#include "config.h"

// ============================================================
//  Unified motion module
// ============================================================
//
// This file replaces:
//   - trajectory.h
//   - trajectory.cpp
//   - arrival_check.h
//   - arrival_check.cpp
//   - motion_sequence.h
//   - motion_sequence.cpp
//
// Main responsibility:
//   1) 5th-order trajectory generation
//   2) arrival check
//   3) demo motion sequence
//
// Unit policy:
//   - M1/M2 external user unit: degree
//   - M3 external user unit: millimeter
//   - internal motor/MIT unit: rad, rad/s

// ============================================================
//  Motion step
// ============================================================

enum MotionStepType : uint8_t {
  STEP_MOVE_ABS = 0,
  STEP_WAIT,
  STEP_BRAKE_ENGAGE,
  STEP_BRAKE_RELEASE,
  STEP_SOL_LOCK,
  STEP_SOL_UNLOCK
};

struct MotionStep {
  MotionStepType type;

  float m1Deg;           // STEP_MOVE_ABS: absolute M1 target [deg]
  float m2Deg;           // STEP_MOVE_ABS: absolute M2 target [deg]
  float m3Mm;            // STEP_MOVE_ABS: absolute M3 target [mm]
  float durationSec;     // STEP_MOVE_ABS: trajectory duration [sec]

  uint32_t waitAfterMs;  // wait time after move/action [ms]
  const char* name;
};

extern const MotionStep DEMO_SEQ[];
extern const uint8_t DEMO_SEQ_COUNT;

// ============================================================
//  New recommended names - trajectory
// ============================================================

bool motionStartAbsolute(float m1Deg, float m2Deg, float m3Mm, float durationSec);
bool motionStartRelative(float m1DeltaDeg, float m2DeltaDeg, float m3DeltaMm, float durationSec);

void motionTrajectoryUpdate();
void motionTrajectoryClear();

bool motionTrajectoryHasReference();
bool motionTrajectoryIsActive();
bool motionTrajectoryIsDone();

float motionGetRefRad(uint8_t motorId);
float motionGetRefVelRadps(uint8_t motorId);
float motionGetGoalRad(uint8_t motorId);

void motionTrajectoryPrint();

// ============================================================
//  New recommended names - arrival
// ============================================================

void motionArrivalReset();
bool motionArrivalAllReached();
void motionArrivalPrint();

// ============================================================
//  New recommended names - sequence
// ============================================================

void motionSequenceStartDemo();
void motionSequenceStop();
void motionSequenceUpdate();
void motionSequencePrintStatus();

bool motionSequenceIsRunning();


// ============================================================
//  New recommended names - M3 torque seek
// ============================================================
//
// M3 moves in one direction and stops when command torque rises.
// This keeps the existing seek3/seekstop/seekstat commands working
// without a separate torque_seek.h/cpp file.

void torqueSeekStartM3(float deltaMm, float durationSec, float torqueThresholdNm, uint32_t holdMs = 100);
void torqueSeekStop();
void torqueSeekUpdate();
void torqueSeekPrintStatus();

bool torqueSeekIsActive();
bool torqueSeekIsHit();

// ============================================================
//  Backward-compatible old names
//  These allow existing code to keep using old function names
//  after changing includes to #include "motion.h".
// ============================================================

// Old trajectory names
bool trajectoryStartAbsoluteDegMm(float m1Deg, float m2Deg, float m3Mm, float durationSec);
bool trajectoryStartRelativeDegMm(float m1DeltaDeg, float m2DeltaDeg, float m3DeltaMm, float durationSec);

void trajectoryUpdate();
void trajectoryClear();

bool trajectoryHasReference();
bool trajectoryIsActive();
bool trajectoryIsDone();

float trajectoryGetRefRad(uint8_t motorId);
float trajectoryGetRefVelRadps(uint8_t motorId);
float trajectoryGetGoalRad(uint8_t motorId);

void trajectoryPrint();

// Old arrival names
void arrivalReset();
bool arrivalAllReached();
void arrivalPrint();

// Old sequence names
void sequenceStartDemo();
void motionSequenceStartDemoRepeat(uint16_t count);
void sequenceStartDemoRepeat(uint16_t count);
void sequenceStop();
void sequenceUpdate();
void sequencePrintStatus();

bool sequenceIsRunning();
