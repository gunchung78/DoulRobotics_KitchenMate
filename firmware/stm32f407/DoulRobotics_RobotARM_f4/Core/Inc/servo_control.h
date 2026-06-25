/**
 * servo_control.h
 * RS232 텍스트 명령 → TIM4 PWM 서보 제어
 * Target: STM32F407G-DISC1
 *
 * 명령 형식:
 *   SERVO <angle>\r\n       — 서보 각도 설정  (예: SERVO 45\r\n)
 *   SERVO MODE <90|180>\r\n — 서보 범위 모드  (예: SERVO MODE 180\r\n)
 *   STATUS\r\n              — 현재 상태 출력
 *   HELP\r\n                — 명령어 목록 출력
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── 설정값 ────────────────────────────────────────────── */
#define SERVO_TIMER         htim4
#define SERVO_CHANNEL       TIM_CHANNEL_1
#define UART_RS232          huart2

/* ARR = 999, 1tick = 20μs (TIM4 @ 50Hz) */
#define SERVO_ARR           999

/* 90° 모드: 1000~2000μs */
#define SERVO_90_MIN_US     1000
#define SERVO_90_MAX_US     2000
#define SERVO_90_CCR_MIN    50      /* 1000/20 */
#define SERVO_90_CCR_MAX    100     /* 2000/20 */

/* 180° 모드: 500~2500μs */
#define SERVO_180_MIN_US    500
#define SERVO_180_MAX_US    2500
#define SERVO_180_CCR_MIN   25      /* 500/20  */
#define SERVO_180_CCR_MAX   125     /* 2500/20 */

/* RX 버퍼 */
#define RX_BUF_SIZE         64

/* ── 타입 ──────────────────────────────────────────────── */
typedef enum {
    SERVO_MODE_90  = 90,
    SERVO_MODE_180 = 180
} ServoMode_t;

typedef struct {
    ServoMode_t mode;       /* 현재 범위 모드 */
    uint8_t     angle;      /* 현재 각도 */
    uint32_t    ccr;        /* 현재 CCR 값 */
} ServoState_t;

/* ── extern 선언 (main.c에서 생성된 핸들) ─────────────── */
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern uint8_t s_rx_byte;

/* ── 함수 선언 ─────────────────────────────────────────── */
void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);
void Servo_SetMode(ServoMode_t mode);
void RS232_ProcessByte(uint8_t byte);
void RS232_SendString(const char *str);
ServoState_t Servo_GetState(void);

#endif /* SERVO_CONTROL_H */
