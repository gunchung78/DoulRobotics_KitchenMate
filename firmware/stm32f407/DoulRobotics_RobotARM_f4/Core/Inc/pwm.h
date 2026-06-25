/**
 * @file    pwm.h
 * @brief   브레이크(x2) + 솔레노이드(x1) PWM 제어 모듈
 *
 * 타이머  : TIM3 (APB1 Timer Clock = 84 MHz)
 * 주파수  : 15 kHz  (Prescaler=167, ARR=332 → 84M÷168÷333 ≈ 15.015 kHz)
 * CCR 기준: ARR+1 = 333  → 100% = 333, 40% = 133, 0% = 0
 *
 * 채널 매핑:
 *   CH1 (PC6) – 브레이크 1
 *   CH2 (PC7) – 브레이크 2
 *   CH3 (PC8) – 솔레노이드
 *
 * 브레이크 동작:
 *   ON  → Duty 100%  즉시 출력
 *         TIM2 5ms 틱 카운트로 1초(200틱) 경과 후 → Duty 40% 홀딩
 *   OFF → Duty 0%    즉시 출력, 타이머 카운트 리셋
 *
 * 솔레노이드 동작:
 *   ON  → Duty 100%
 *   OFF → Duty 0%
 *
 * USB CDC 명령 (usbd_cdc_if.c 의 CDC_Receive_FS 에서 호출):
 *   "BK1 ON\r\n"   "BK1 OFF\r\n"
 *   "BK2 ON\r\n"   "BK2 OFF\r\n"
 *   "SOL ON\r\n"   "SOL OFF\r\n"
 *   "STATUS\r\n"
 *
 * 호출 구조:
 *   main.c  : PWM_Init()
 *   TIM2 ISR: PWM_Tick5ms()           ← 5ms 마다 호출 (1초 홀딩 타이머)
 *   CDC RX  : PWM_ProcessCommand()    ← 수신 문자열 전달
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  공개 API                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief  PWM 모듈 초기화 — TIM3 CH1/CH2/CH3 PWM 시작, 전체 OFF 상태.
 *         main.c 의 USER CODE BEGIN 2 에서 1회 호출.
 */
void PWM_Init(void);

/**
 * @brief  5ms 주기 틱 함수 — TIM2 PeriodElapsedCallback 에서 호출.
 *         내부적으로 1초(200틱) 경과 시 브레이크를 40% 홀딩으로 전환.
 */
void PWM_Tick5ms(void);

/**
 * @brief  USB CDC 수신 문자열 처리.
 *         usbd_cdc_if.c 의 CDC_Receive_FS 에서 호출.
 *
 * @param  buf  수신 데이터 포인터 (NULL 종료 불필요, len 사용)
 * @param  len  수신 바이트 수
 */
void PWM_ProcessCommand(const uint8_t *buf, uint32_t len);

#endif /* PWM_H */