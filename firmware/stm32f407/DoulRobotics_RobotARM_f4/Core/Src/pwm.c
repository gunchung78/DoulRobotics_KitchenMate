/**
 * @file    pwm.c
 * @brief   브레이크(x2) + 솔레노이드(x1) PWM 제어 구현
 *
 * 설계 방향
 *   - command 파싱은 cmd.c에서만 처리한다.
 *   - pwm.c는 PWM 하드웨어 제어 함수만 제공한다.
 *   - USB CDC 수신 버퍼/라인 파싱/PWM_ProcessCommand()는 사용하지 않는다.
 */

#include "stm32f4xx_hal.h"   /* HAL 전체 포함 — TIM_HandleTypeDef, TIM_CHANNEL_x 등 */
#include "pwm.h"
#include <string.h>
#include <stdio.h>

/* ================================================================== */
/*  상수 정의                                                          */
/* ================================================================== */

/*
 * TIM3 주파수 계산
 *   APB1 Timer Clock = 84 MHz
 *   Prescaler = 167  → 84,000,000 / 168 = 500,000 Hz
 *   ARR       =  27  → 500,000 /  28   ≈ 17,857 Hz (≈ 18kHz)
 *   CCR 범위  = 0 ~ 28 (= ARR + 1)
 */
#define TIM3_ARR_PLUS1    28u

#define DUTY_100_CCR     (TIM3_ARR_PLUS1)                    /*  28 : 100% */
#define DUTY_40_CCR      (TIM3_ARR_PLUS1 * 40u / 100u)       /*  11 :  40% */
#define DUTY_0_CCR       (0u)                                /*   0 :   0% */

/* 1초 홀딩: 5ms 틱 × 200 = 1000ms */
#define HOLD_TICKS        1000u

/* ================================================================== */
/*  내부 타입                                                          */
/* ================================================================== */

typedef enum {
    BK_OFF  = 0,
    BK_ON   = 1,
    BK_HOLD = 2
} BrakeState_t;

typedef struct {
    BrakeState_t  state;
    uint32_t      tick_cnt;
} BrakeCtx_t;

/* ================================================================== */
/*  내부 변수                                                          */
/* ================================================================== */

static BrakeCtx_t  s_bk[2];       /* [0]=브레이크1, [1]=브레이크2 */
static uint8_t     s_sol_on = 0u;

/* ================================================================== */
/*  HAL 핸들 (CubeMX가 tim.c/main.c에서 생성)                         */
/* ================================================================== */

extern TIM_HandleTypeDef htim3;

/* ================================================================== */
/*  CDC 송신                                                           */
/*  주의: PWM_PrintStatus()는 cmd.c의 메인 루프 명령 처리에서만 호출   */
/* ================================================================== */

extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

static void cdc_print(const char *str)
{
    CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}

/* ================================================================== */
/*  내부 헬퍼                                                          */
/* ================================================================== */

static void set_ccr(uint32_t ch, uint32_t ccr)
{
    __HAL_TIM_SET_COMPARE(&htim3, ch, ccr);
}

static uint32_t brake_ch(uint8_t idx)
{
    return (idx == 0u) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static void brake_on(uint8_t idx)
{
    if (idx >= 2u) return;

    s_bk[idx].state    = BK_ON;
    s_bk[idx].tick_cnt = 0u;
    set_ccr(brake_ch(idx), DUTY_100_CCR);
}

static void brake_off(uint8_t idx)
{
    if (idx >= 2u) return;

    s_bk[idx].state    = BK_OFF;
    s_bk[idx].tick_cnt = 0u;
    set_ccr(brake_ch(idx), DUTY_0_CCR);
}

/* ================================================================== */
/*  공개 API                                                           */
/* ================================================================== */

void PWM_Init(void)
{
    memset(s_bk, 0, sizeof(s_bk));
    s_sol_on = 0u;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* BK1 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* BK2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* SOL */

    set_ccr(TIM_CHANNEL_1, DUTY_0_CCR);
    set_ccr(TIM_CHANNEL_2, DUTY_0_CCR);
    set_ccr(TIM_CHANNEL_3, DUTY_0_CCR);

    /* 여기서는 출력하지 않는다.
     * USB CDC가 아직 준비 전일 수 있고, cmd.c 설계 원칙상 출력은 메인 루프에서 처리한다.
     */
}

/* ------------------------------------------------------------------ */
/*  5ms마다 호출                                                       */
/*  주의: 타이머 인터럽트에서 호출될 수 있으므로 CDC 출력 금지          */
/* ------------------------------------------------------------------ */

void PWM_Tick5ms(void)
{
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (s_bk[i].state == BK_ON)
        {
            s_bk[i].tick_cnt++;

            if (s_bk[i].tick_cnt >= HOLD_TICKS)
            {
                s_bk[i].state = BK_HOLD;
                set_ccr(brake_ch(i), DUTY_40_CCR);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  브레이크 1                                                         */
/* ------------------------------------------------------------------ */

void PWM_BK1_On(void)
{
    brake_on(0u);      /* 100%로 켠 뒤 PWM_Tick5ms()에서 1초 후 40% HOLD */
}

void PWM_BK1_Off(void)
{
    brake_off(0u);
}

/* ------------------------------------------------------------------ */
/*  브레이크 2                                                         */
/* ------------------------------------------------------------------ */

void PWM_BK2_On(void)
{
    brake_on(1u);      /* 100%로 켠 뒤 PWM_Tick5ms()에서 1초 후 40% HOLD */
}

void PWM_BK2_Off(void)
{
    brake_off(1u);
}

/* ------------------------------------------------------------------ */
/*  솔레노이드                                                         */
/* ------------------------------------------------------------------ */

void PWM_SOL_On(void)
{
    s_sol_on = 1u;
    set_ccr(TIM_CHANNEL_3, DUTY_100_CCR);
}

void PWM_SOL_Off(void)
{
    s_sol_on = 0u;
    set_ccr(TIM_CHANNEL_3, DUTY_0_CCR);
}

/* ------------------------------------------------------------------ */
/*  상태 출력                                                          */
/*  cmd.c에서 "PWM STATUS" 명령을 받았을 때 호출한다.                 */
/* ------------------------------------------------------------------ */

void PWM_PrintStatus(void)
{
    char resp[160];
    const char *bk_str[3] = {"OFF", "ON(100%)", "HOLD(40%)"};

    snprintf(resp, sizeof(resp),
        "[PWM STATUS] BK1=%s BK2=%s SOL=%s | CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
        bk_str[s_bk[0].state],
        bk_str[s_bk[1].state],
        s_sol_on ? "ON" : "OFF",
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3));

    cdc_print(resp);
}

/* ------------------------------------------------------------------ */
/*  PWM_ProcessCommand() 제거됨                                        */
/* ------------------------------------------------------------------ */
