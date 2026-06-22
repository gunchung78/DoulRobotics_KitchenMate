#include "brake.h"

static bool solLocked = false;
static uint32_t solLockStartMs = 0;

void setupBrake() {
  analogWriteFrequency(PIN_BRAKE1, 20000);
  analogWriteFrequency(PIN_BRAKE2, 20000);
  analogWriteFrequency(PIN_BRAKE_SOL, 20000);

  for (int i = 0; i < 10; i++) {
    analogWrite(PIN_BRAKE1, BRAKE_PWM_FULL);
    analogWrite(PIN_BRAKE2, BRAKE_PWM_FULL);
    analogWrite(PIN_BRAKE_SOL, 255);
    delay(100);
  }

  analogWrite(PIN_BRAKE1, BRAKE_PWM_HOLD);
  analogWrite(PIN_BRAKE2, BRAKE_PWM_HOLD);

  brakePhase = BRAKE_HOLD;
}

void brakeEngage() {
  brakePhase = BRAKE_LOCKED;
  analogWrite(PIN_BRAKE1, BRAKE_PWM_LOCK);
  analogWrite(PIN_BRAKE2, BRAKE_PWM_LOCK);
}

void brakeRelease() {
  brakePhase = BRAKE_FULL;
  brakeFullStart = millis();
  analogWrite(PIN_BRAKE1, BRAKE_PWM_FULL);
  analogWrite(PIN_BRAKE2, BRAKE_PWM_FULL);
}

void updateBrake() {
  if (brakePhase == BRAKE_FULL &&
      millis() - brakeFullStart >= BRAKE_FULL_DURATION_MS) {
    analogWrite(PIN_BRAKE1, BRAKE_PWM_HOLD);
    analogWrite(PIN_BRAKE2, BRAKE_PWM_HOLD);
    brakePhase = BRAKE_HOLD;
  }
}

void solLock() {
  solPhase = SOL_LOCKED;
  solPulseStart = 0;
  solPulseDurationMs = 0;

  analogWrite(PIN_BRAKE_SOL, SOL_PWM_LOCK);  // 255
}

void solPulseLow(uint32_t durationMs) {
  // 0.5초 이상 유지 방지
  if (durationMs >= SOL_UNLOCK_MAX_MS) {
    durationMs = SOL_UNLOCK_MAX_MS - 50;  // 예: 450ms
  }

  solPhase = SOL_UNLOCK_PULSE;
  solPulseStart = millis();
  solPulseDurationMs = durationMs;

  analogWrite(PIN_BRAKE_SOL, SOL_PWM_UNLOCK);  // 0
}

void solUnlock() {
  solPulseLow(SOL_UNLOCK_PULSE_MS);
}

void updateSol() {
  if (solPhase != SOL_UNLOCK_PULSE) {
    return;
  }

  uint32_t elapsed = millis() - solPulseStart;

  // durationMs가 지나거나, 혹시 몰라 MAX 시간을 넘으면 무조건 255 복귀
  if (elapsed >= solPulseDurationMs || elapsed >= SOL_UNLOCK_MAX_MS) {
    analogWrite(PIN_BRAKE_SOL, SOL_PWM_LOCK);  // 255로 자동 복귀

    solPhase = SOL_LOCKED;
    solPulseStart = 0;
    solPulseDurationMs = 0;

    Serial.println("[SOL] auto return to 255");
  }
}


