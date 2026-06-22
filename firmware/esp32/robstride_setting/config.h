/*
 * config.h
 * ─────────────────────────────────────────────
 * 하드웨어 핀 설정 및 RS02 MIT 물리량 범위 정의
 *
 * 하드웨어:
 *   ESP32 TWAI CAN 1Mbps
 *   TX: GPIO21 / RX: GPIO22
 *   CAN Transceiver: SN65HVD230
 *   CANH-CANL 종단저항 120Ω 권장
 * ─────────────────────────────────────────────
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// CAN 핀 설정
// ============================================================
#define CAN_TX_PIN  GPIO_NUM_21
#define CAN_RX_PIN  GPIO_NUM_22

// ============================================================
// RS02 MIT 물리량 범위
// 주의: 기존 요약본에는 속도 범위 -33~33 rad/s도 있었음.
// 현재 코드는 기존 사용 코드 기준 -44~44 rad/s 사용.
// 실제 모터 매뉴얼 기준으로 필요 시 수정.
// ============================================================
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -44.0f
#define V_MAX    44.0f
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define T_MIN   -17.0f
#define T_MAX    17.0f

// ============================================================
// 모터 ID 최대 개수 (0x00 ~ 0x7F)
// ============================================================
#define MOTOR_ID_MAX  128

// ============================================================
// 브레이크 / 솔레노이드 핀 설정
// ※ 실제 하드웨어에 맞춰 핀 번호를 수정하세요
// ============================================================
#define PIN_BRAKE1      16
#define PIN_BRAKE2      17
#define PIN_BRAKE_SOL   18

// ============================================================
// 브레이크 PWM 파라미터
//   FULL : 해제 직후 100% 구동 (초기 인가)
//   HOLD : FULL 이후 유지 전류 (~40%)
//   LOCK : 잠금 시 PWM 0% (스프링 작동)
// ============================================================
#define BRAKE_PWM_FULL           255
#define BRAKE_PWM_HOLD           102   // ~40% of 255
#define BRAKE_PWM_LOCK             0

// FULL → HOLD 자동 전환까지의 시간 (ms)
#define BRAKE_FULL_DURATION_MS  1000

#endif // CONFIG_H
