//###########################################################################
// FILE:    config.c
// TITLE:   튜닝/설정 파라미터 기본값 정의
//###########################################################################

#include "config.h"

/* =========================================================
 * 축별 기본 설정  [0]=J0 [1]=J1 [2]=J2
 *
 *  J0,J1 : 회전 관절 → 내부 PD (모터 펌웨어가 위치제어)
 *  J2    : 슬라이드  → 외부 PD (STM32가 토크 계산)
 *
 *  ESP32 pd_control 기본값 참고:
 *    M1: kp=30 kd=1.5   M2: kp=15 kd=1.5   M3(외부): kp=7.5 kd=1.5 limit=17
 * ========================================================= */
AxisConfig_t g_axis_cfg[MOTOR_COUNT] = {
    /* J0 (내부PD) */
    { .kp = 30.0f, .kd = 1.5f, .q_min = -3.14f, .q_max = 3.14f,
      .mode = PD_MODE_INTERNAL, .base_torque = 0.0f, .torque_limit = 10.0f },

    /* J1 (내부PD) */
    { .kp = 15.0f, .kd = 1.5f, .q_min = -1.57f, .q_max = 1.57f,
      .mode = PD_MODE_INTERNAL, .base_torque = 0.0f, .torque_limit = 10.0f },

    /* J2 (외부PD, 슬라이드) */
    { .kp = 7.5f,  .kd = 1.5f, .q_min = -2.0f,  .q_max = 2.0f,
      .mode = PD_MODE_EXTERNAL, .base_torque = 0.0f, .torque_limit = 17.0f },
};

void config_Init(void)
{
    /* 정적 초기값 사용. 추후 Flash 로드 시 여기서 처리 */
}
