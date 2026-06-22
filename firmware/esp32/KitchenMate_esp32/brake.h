#pragma once
#include "config.h"

void setupBrake();

void brakeEngage();
void brakeRelease();
void updateBrake();

void solLock();
void solUnlock();

void solUnlock();
void solPulseLow(uint32_t durationMs);
void updateSol();