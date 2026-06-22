/*
 * brake.h
 * ─────────────────────────────────────────────
 * 브레이크 / 솔레노이드 제어
 *   - 브레이크: PWM 기반 FULL→HOLD 자동 전환
 *   - 솔레노이드: Lock / Unlock
 * ─────────────────────────────────────────────
 */

#pragma once
#include "config.h"

// ============================================================
// 브레이크 상태 enum
// ============================================================
enum BrakePhase {
    BRAKE_LOCKED,   // 잠금 (PWM 0%, 스프링 작동)
    BRAKE_FULL,     // 해제 직후 100% 구동
    BRAKE_HOLD      // 유지 전류 (~40%)
};

enum SolPhase {
    SOL_LOCKED,     // 솔레노이드 잠금 (PWM 0%)
    SOL_UNLOCKED    // 솔레노이드 해제 (PWM 100%)
};

// ============================================================
// 전역 상태 변수 (정의는 brake.cpp)
// ============================================================
extern BrakePhase    brakePhase;
extern unsigned long brakeFullStart;
extern SolPhase      solPhase;

// ============================================================
// 함수 선언
// ============================================================

// 브레이크 초기화 — setup()에서 호출
void setupBrake();

// 브레이크 잠금 — PWM 0% (스프링 작동)
void brakeEngage();

// 브레이크 해제 — PWM 100% 시작, 이후 updateBrake()가 HOLD로 전환
void brakeRelease();

// loop() 최상단에서 반드시 호출 — FULL→HOLD 전환 타이밍 처리
void updateBrake();

// 솔레노이드 잠금 — PWM 0%
void solLock();

// 솔레노이드 해제 — PWM 100%
void solUnlock();
