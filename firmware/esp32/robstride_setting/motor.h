/*
 * motor.h
 * ─────────────────────────────────────────────
 * MIT 피드백 구조체, 전역 상태 변수 선언,
 * ID 유효성 검사 및 값 변환 함수 선언
 * ─────────────────────────────────────────────
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// ============================================================
// MIT 피드백 저장 구조체
// ============================================================
struct MotorFeedback {
    bool     valid = false;
    uint8_t  motorID = 0;
    float    position = 0.0f;      // rad
    float    velocity = 0.0f;      // rad/s
    float    torque = 0.0f;        // Nm
    float    temperature = 0.0f;   // Celsius
    uint8_t  raw[8] = {0};
    uint8_t  dlc = 0;
    uint32_t rxCanID = 0;
    bool     extd = false;
    uint32_t lastUpdateMs = 0;
};

// ============================================================
// 전역 상태 변수 (정의는 motor.cpp)
// ============================================================
extern uint8_t       g_motorID;          // 마지막으로 Enable/제어/조회한 모터 ID
extern bool          g_mitMode;          // MIT 프로토콜 전환 여부 표시용
extern bool          g_mitReady;         // 하나 이상의 모터가 Enable 되었는지 표시용
extern bool          g_motorEnabled[];   // ID별 Enable 상태 추적
extern MotorFeedback g_feedback[];       // ID별 피드백 저장

// ============================================================
// ID 유효성 검사
// ============================================================
bool isValidDeviceID(int id);   // 29bit 설정/스캔용 (0x00~0x7F)
bool isValidMotorID(int id);    // MIT 제어용 (0x01~0x7F)
bool anyMotorEnabled();
void updateMitReadyFlag();

// ============================================================
// 값 변환 함수
// ============================================================
float    u2f(uint16_t x, float mn, float mx, uint8_t bits);  // uint → float
uint16_t f2u(float x, float mn, float mx, uint8_t bits);     // float → uint

#endif // MOTOR_H
