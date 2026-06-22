//###########################################################################
// FILE:    config.h
// TITLE:   튜닝/설정 파라미터 중앙 관리
//###########################################################################

#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================================================
 * 모터 설정
 * ========================================================= */
#define MOTOR_COUNT         3

#define TWO_PI_F   6.2831853071795864769f
 
/* ★ 메커니즘: 1회전 = 5mm */
#define AXIS_J2_MM_PER_REV   75.0f
#define AXIS_J2_MM_PER_RAD   (AXIS_J2_MM_PER_REV / TWO_PI_F)
 
/* MIT 위치 범위 */
#define MIT_POS_MIN   (-12.57f)
#define MIT_POS_MAX   ( 12.57f)
#define MIT_POS_RANGE (MIT_POS_MAX - MIT_POS_MIN)
 
/* wrap 판정 임계 (한 샘플에 범위의 45% 이상 점프하면 wrap) */
#define WRAP_THRESHOLD (MIT_POS_RANGE * 0.45f)

/* =========================================================
 * 제어 주기
 * ========================================================= */
#define CTRL_PERIOD_MS      5                          /* TIM2 주기 (ms) */
#define CTRL_DT             (CTRL_PERIOD_MS * 0.001f)  /* 초 단위 자동계산 */
#define PRINT_PERIOD_MS     100                        /* 출력 주기 (ms) */

/* =========================================================
 * 통신 버퍼 크기
 * ========================================================= */
#define RX_RING_SIZE        512
#define CMD_LINE_SIZE       128

/* =========================================================
 * 궤적 기본값
 * ========================================================= */
#define TRAJ_DEFAULT_TIME   2.0f
#define TRAJ_MIN_TIME       0.3f

/* =========================================================
 * PD 제어 모드
 *   INTERNAL : 모터 펌웨어가 PD (위치+Kp/Kd 송신) — 회전 관절 J0,J1
 *   EXTERNAL : STM32가 PD 계산해 토크만 송신 — 슬라이드 관절 J2
 * ========================================================= */
typedef enum {
    PD_MODE_INTERNAL = 0,
    PD_MODE_EXTERNAL = 1
} PDControlMode;

/* =========================================================
 * 축별 런타임 설정 구조체
 *   - kp/kd       : 내부PD면 MIT 게인, 외부PD면 토크 PD 게인
 *   - q_min/q_max : 각도 제한 (rad)
 *   - mode        : 내부/외부 PD
 *   - base_torque : 외부PD feedforward 토크 (Nm)
 *   - torque_limit: 외부PD 토크 제한 (Nm)
 * ========================================================= */
typedef struct {
    float          kp;
    float          kd;
    float          q_min;
    float          q_max;
    PDControlMode  mode;
    float          base_torque;
    float          torque_limit;
} AxisConfig_t;

extern AxisConfig_t g_axis_cfg[MOTOR_COUNT];

/* =========================================================
 * 초기화
 * ========================================================= */
void config_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H__ */
