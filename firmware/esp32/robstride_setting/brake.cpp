/*
 * brake.cpp
 * ─────────────────────────────────────────────
 * 브레이크 / 솔레노이드 제어 구현
 * ─────────────────────────────────────────────
 */

#include "brake.h"

// ============================================================
// 전역 상태 변수 정의
// ============================================================
BrakePhase    brakePhase     = BRAKE_LOCKED;
unsigned long brakeFullStart = 0;
SolPhase      solPhase       = SOL_LOCKED;

// ============================================================
// 브레이크 초기화
// ============================================================
void setupBrake() {
    // PWM 주파수 20kHz로 설정 (각 핀별로)
    analogWriteFrequency(PIN_BRAKE1,    20000);
    analogWriteFrequency(PIN_BRAKE2,    20000);
    analogWriteFrequency(PIN_BRAKE_SOL, 20000);

    // 초기 브레이크 동작
    for (int i = 0; i < 10; i++) {
        analogWrite(PIN_BRAKE1, BRAKE_PWM_FULL);
        analogWrite(PIN_BRAKE2, BRAKE_PWM_FULL);
        analogWrite(PIN_BRAKE_SOL, 255);
        delay(100);
    }
    analogWrite(PIN_BRAKE1, BRAKE_PWM_HOLD);
    analogWrite(PIN_BRAKE2, BRAKE_PWM_HOLD);
}

// ============================================================
// 브레이크 잠금 — PWM 0% (스프링 작동)
// ============================================================
void brakeEngage() {
    brakePhase = BRAKE_LOCKED;
    analogWrite(PIN_BRAKE1, BRAKE_PWM_LOCK);
    analogWrite(PIN_BRAKE2, BRAKE_PWM_LOCK);
}

// ============================================================
// 브레이크 해제 — PWM 100% 시작, updateBrake()가 HOLD로 전환
// ============================================================
void brakeRelease() {
    brakePhase     = BRAKE_FULL;
    brakeFullStart = millis();
    analogWrite(PIN_BRAKE1, BRAKE_PWM_FULL);
    analogWrite(PIN_BRAKE2, BRAKE_PWM_FULL);
}

// ============================================================
// 솔레노이드 잠금 — PWM 0%
// ============================================================
void solLock() {
    solPhase = SOL_LOCKED;
    analogWrite(PIN_BRAKE_SOL, 0);
}

// ============================================================
// 솔레노이드 해제 — PWM 100%
// ============================================================
void solUnlock() {
    solPhase = SOL_UNLOCKED;
    analogWrite(PIN_BRAKE_SOL, 255);
}

// ============================================================
// FULL→HOLD 자동 전환 — loop() 최상단에서 호출
// ============================================================
void updateBrake() {
    if (brakePhase == BRAKE_FULL &&
        millis() - brakeFullStart >= BRAKE_FULL_DURATION_MS) {
        brakePhase = BRAKE_HOLD;
        analogWrite(PIN_BRAKE1, BRAKE_PWM_HOLD);
        analogWrite(PIN_BRAKE2, BRAKE_PWM_HOLD);
    }
}
