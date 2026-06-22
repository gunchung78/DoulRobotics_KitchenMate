#pragma once
#include "config.h"

bool feedbackAlive(uint8_t motorIndex, uint32_t now);
bool waitInitialFeedback(uint32_t timeoutMs);

void captureCurrentPositionAsTarget();
void sendTargetCommand();
void safeStop();
