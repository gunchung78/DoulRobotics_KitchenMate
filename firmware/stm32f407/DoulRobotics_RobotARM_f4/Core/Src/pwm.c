/**
 * @file    pwm.c
 * @brief   브레이크(x2) + 솔레노이드(x1) PWM 제어 구현
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
 *   ARR       = 332  → 500,000 / 333   ≈ 15,015 Hz
 *   CCR 범위  = 0 ~ 333 (= ARR + 1)
 */
#define TIM3_ARR_PLUS1    333u

#define DUTY_100_CCR     (TIM3_ARR_PLUS1)                    /* 333 : 100% */
#define DUTY_40_CCR      (TIM3_ARR_PLUS1 * 40u / 100u)      /* 133 :  40% */
#define DUTY_0_CCR       (0u)                                /*   0 :   0% */

/* 1초 홀딩: TIM2 5ms 틱 × 200 = 1000ms */
#define HOLD_TICKS        200u

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
/*  HAL 핸들 (CubeMX 가 tim.c / main.c 에서 생성)                    */
/* ================================================================== */

extern TIM_HandleTypeDef htim3;

/* ================================================================== */
/*  CDC 송신 (usbd_cdc_if.c 에 정의됨)                               */
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
    s_bk[idx].state    = BK_ON;
    s_bk[idx].tick_cnt = 0u;
    set_ccr(brake_ch(idx), DUTY_100_CCR);
}

static void brake_off(uint8_t idx)
{
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

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    set_ccr(TIM_CHANNEL_1, DUTY_0_CCR);
    set_ccr(TIM_CHANNEL_2, DUTY_0_CCR);
    set_ccr(TIM_CHANNEL_3, DUTY_0_CCR);

    cdc_print("[PWM] Init OK. BK1=OFF BK2=OFF SOL=OFF\r\n");
}

/* ------------------------------------------------------------------ */

void PWM_Tick5ms(void)
{
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (s_bk[i].state == BK_ON)
        {
            s_bk[i].tick_cnt++;

            if (s_bk[i].tick_cnt >= HOLD_TICKS)
            {
                s_bk[i].state = BK_HOLD;
                set_ccr(brake_ch(i), DUTY_40_CCR);

                if (i == 0u)
                    cdc_print("[PWM] BK1 -> HOLD 40%\r\n");
                else
                    cdc_print("[PWM] BK2 -> HOLD 40%\r\n");
            }
        }
    }
}

/* ------------------------------------------------------------------ */

void PWM_ProcessCommand(const uint8_t *buf, uint32_t len)
{
    static char    line[64];
    static uint8_t li = 0u;

    char resp[128];

    uint32_t i;
    for (i = 0u; i < len; i++)
    {
        uint8_t c = buf[i];

        if (c == '\r' || c == '\n')
        {
            if (li == 0u) { continue; }

            line[li] = '\0';
            li = 0u;

            /* ---- 명령 분기 ---- */

            if (strcmp(line, "BK1 ON") == 0)
            {
                brake_on(0u);
                cdc_print("[PWM] BK1 ON (100%) -> HOLD 40% after 1s\r\n");
            }
            else if (strcmp(line, "BK1 OFF") == 0)
            {
                brake_off(0u);
                cdc_print("[PWM] BK1 OFF\r\n");
            }
            else if (strcmp(line, "BK2 ON") == 0)
            {
                brake_on(1u);
                cdc_print("[PWM] BK2 ON (100%) -> HOLD 40% after 1s\r\n");
            }
            else if (strcmp(line, "BK2 OFF") == 0)
            {
                brake_off(1u);
                cdc_print("[PWM] BK2 OFF\r\n");
            }
            else if (strcmp(line, "SOL ON") == 0)
            {
                s_sol_on = 1u;
                set_ccr(TIM_CHANNEL_3, DUTY_100_CCR);
                cdc_print("[PWM] SOL ON\r\n");
            }
            else if (strcmp(line, "SOL OFF") == 0)
            {
                s_sol_on = 0u;
                set_ccr(TIM_CHANNEL_3, DUTY_0_CCR);
                cdc_print("[PWM] SOL OFF\r\n");
            }
            else if (strcmp(line, "STATUS") == 0)
            {
                const char *bk_str[3] = {"OFF", "ON(100%)", "HOLD(40%)"};
                snprintf(resp, sizeof(resp),
                    "[STATUS] BK1=%s BK2=%s SOL=%s | CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
                    bk_str[s_bk[0].state],
                    bk_str[s_bk[1].state],
                    s_sol_on ? "ON" : "OFF",
                    (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
                    (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
                    (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3));
                cdc_print(resp);
            }
            else if (strcmp(line, "HELP") == 0)
            {
                cdc_print(
                    "[HELP] BK1 ON/OFF | BK2 ON/OFF | SOL ON/OFF | STATUS\r\n");
            }
            else
            {
                snprintf(resp, sizeof(resp), "[ERR] Unknown: '%s'\r\n", line);
                cdc_print(resp);
            }
        }
        else
        {
            if (li < (uint8_t)(sizeof(line) - 1u))
            {
                line[li++] = (char)c;
            }
        }
    }
}
