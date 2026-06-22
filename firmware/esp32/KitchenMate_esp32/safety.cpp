#include "safety.h"
#include "runtime.h"
#include "motor.h"
#include "brake.h"
#include "pd_control.h"

bool feedbackAlive(uint8_t motorIndex, uint32_t now) {
  return fb[motorIndex].ts && (now - fb[motorIndex].ts <= FB_TIMEOUT_MS);
}

bool waitInitialFeedback(uint32_t timeoutMs) {
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    recvFeedback();

    bool ok = true;
    uint32_t now = millis();

    for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
      if (!feedbackAlive(i, now)) {
        ok = false;
        break;
      }
    }

    if (ok) return true;
    delay(2);
  }

  return false;
}

void captureCurrentPositionAsTarget() {
  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    targetPos[i] = fb[i].pos;
    targetVel[i] = 0.0f;
    targetTrq[i] = 0.0f;
    lastCmd[i] = { targetPos[i], targetVel[i], 0.0f, 0.0f, targetTrq[i] };
  }
}

void sendTargetCommand() {
  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    float mitKp = pdControlGetMitKp(i);
    float mitKd = pdControlGetMitKd(i);

    // Hybrid mode:
    //   INTERNAL motor -> MIT kp/kd are sent to the motor.
    //   EXTERNAL motor -> MIT kp/kd are 0, targetTrq contains ESP32-calculated torque.
    sendMIT(i, targetPos[i], targetVel[i], mitKp, mitKd, targetTrq[i]);
  }
}

void safeStop() {
  Serial.println("[STOP] Disable motors and engage brake");
  brakeEngage();
  delay(100);
  disableAll();
  controlEnabled = false;
}
