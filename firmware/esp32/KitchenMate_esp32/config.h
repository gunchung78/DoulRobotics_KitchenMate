#pragma once
#include <Arduino.h>

// ============================================================
//  Pin settings
// ============================================================

#define PIN_CAN_TX    GPIO_NUM_21
#define PIN_CAN_RX    GPIO_NUM_22

#define PIN_BRAKE1    25
#define PIN_BRAKE2    26
#define PIN_BRAKE_SOL 27

// ============================================================
//  Motor IDs
// ============================================================

#define MOTOR_ID1  1   // RS-03
#define MOTOR_ID2  2   // RS-03
#define MOTOR_ID3  3   // RS-02
#define MOTOR_COUNT 3

// ============================================================
//  Brake PWM
// ============================================================

#define BRAKE_PWM_LOCK  0
#define BRAKE_PWM_FULL  255
#define BRAKE_PWM_HOLD  102
#define BRAKE_FULL_DURATION_MS 1000

#define SOL_PWM_LOCK    255
#define SOL_PWM_UNLOCK  0
#define SOL_UNLOCK_PULSE_MS 1500
#define SOL_UNLOCK_MAX_MS   1700 
extern uint32_t solPulseStart;
extern uint32_t solPulseDurationMs;

// ============================================================
//  MIT range settings
// ============================================================

#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -44.0f
#define V_MAX    44.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -17.0f
#define T_MAX    17.0f

// ============================================================
//  Control settings
// ============================================================

#define CTRL_PERIOD_MS     5
#define STATUS_PERIOD_MS   500
#define FB_TIMEOUT_MS      100


// ============================================================
//  Soft limit settings
// ============================================================

struct AxisLimitDeg {
  float minDeg;
  float maxDeg;
};

struct AxisLimitMm {
  float minMm;
  float maxMm;
};

extern const AxisLimitDeg LIMIT_M1;
extern const AxisLimitDeg LIMIT_M2;
extern const AxisLimitMm  LIMIT_M3;

// ============================================================
//  Arrival check settings
// ============================================================

#define ARRIVE_ROT_POS_DEG_THRESH   4.0f

#define ARRIVE_M3_POS_MM_THRESH     3.0f

#define ARRIVE_HOLD_MS              200


// Motor internal gains. These are kept for reference/manual MIT control.
// The external PD torque loop sends MIT kp/kd as 0 in safety.cpp.
extern const float MOTOR_KP[4];
extern const float MOTOR_KD[4];

// ============================================================
//  Runtime types
// ============================================================

enum BrakePhase { BRAKE_LOCKED, BRAKE_FULL, BRAKE_HOLD };
enum SolPhase {
  SOL_LOCKED,
  SOL_UNLOCK_PULSE
};

struct MotorFeedback {
  float    pos;       // raw feedback position [rad], wraps near +/-720deg
  float    vel;       // feedback velocity [rad/s]
  float    trq;       // feedback torque [Nm]
  float    temp;      // temperature [deg C], decoded from MIT feedback data[6:7]
  uint8_t  err;       // reserved/fault flag. Normal MIT feedback does not use data[6] as fault here.
  uint32_t ts;        // last feedback timestamp [ms]
};

struct MotorCommand {
  float p;
  float v;
  float kp;
  float kd;
  float t;
};

// ============================================================
//  Shared globals defined in robot_arm_pd_mm_split.ino
// ============================================================

extern MotorFeedback fb[4];
extern MotorCommand  lastCmd[4];

extern BrakePhase brakePhase;
extern SolPhase   solPhase;

extern bool estop;
extern bool rawDebug;

extern uint32_t brakeFullStart;
