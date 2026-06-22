/*
 * motor.cpp
 * ─────────────────────────────────────────────
 * 전역 변수 정의, ID 유효성 검사 및 값 변환 구현
 * ─────────────────────────────────────────────
 */

#include "motor.h"

// ============================================================
// 전역 상태 변수 정의
// ============================================================
uint8_t       g_motorID  = 0x01;
bool          g_mitMode  = false;
bool          g_mitReady = false;
bool          g_motorEnabled[MOTOR_ID_MAX] = {false};
MotorFeedback g_feedback[MOTOR_ID_MAX];

// ============================================================
// ID 유효성 검사
// ============================================================
bool isValidDeviceID(int id) {
    // 29bit 설정/스캔에서는 0번도 사용할 수 있어서 0x00 허용
    return (id >= 0x00 && id <= 0x7F);
}

bool isValidMotorID(int id) {
    // MIT 제어용 모터 ID는 0x01~0x7F 권장
    return (id >= 0x01 && id <= 0x7F);
}

bool anyMotorEnabled() {
    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        if (g_motorEnabled[i]) return true;
    }
    return false;
}

void updateMitReadyFlag() {
    g_mitReady = anyMotorEnabled();
}

// ============================================================
// uint → float 변환, MIT 피드백 디코딩용
// ============================================================
float u2f(uint16_t x, float mn, float mx, uint8_t bits) {
    float span = mx - mn;
    float maxInt = (float)((1UL << bits) - 1UL);
    return ((float)x) * span / maxInt + mn;
}

// ============================================================
// float → uint 변환, MIT 제어 인코딩용
// ============================================================
uint16_t f2u(float x, float mn, float mx, uint8_t bits) {
    x = constrain(x, mn, mx);
    return (uint16_t)((x - mn) / (mx - mn) * (float)((1UL << bits) - 1UL));
}
