#pragma once
#include "config.h"

// Main control runtime variables.
// Defined in robot_arm_pd_mm_split.ino, used by command/status/safety modules.

extern bool controlEnabled;

extern uint32_t ctrlTimer;
extern uint32_t printTimer;

extern float targetPos[4];
extern float targetVel[4];
extern float targetTrq[4];

extern uint32_t solPulseStart;
extern uint32_t solPulseDurationMs;