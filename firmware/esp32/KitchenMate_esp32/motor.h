#pragma once
#include "config.h"

bool initTWAI();

bool sendFrame(uint32_t id, uint8_t* data, uint8_t len);

void sendMIT(uint8_t id, float pos, float vel,
             float kp, float kd, float torque);

void enableAll();
void disableAll();

void repeatLastCommands();

void recvFeedback();

void setZeroPosition(uint8_t id);
