#ifndef __PD_CONTROL_H__
#define __PD_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/* =========================================================
 * PD 제어 출력 (한 축의 모터 명령)
 * ========================================================= */
typedef struct {
    float pos;      /* 목표 위치 rad */
    float vel;      /* 목표 속도 rad/s */
    float kp;       /* MIT Kp (외부PD면 0) */
    float kd;       /* MIT Kd (외부PD면 0) */
    float torque;   /* 토크 Nm (외부PD면 계산값) */
} PDOutput_t;

/* =========================================================
 * 초기화
 * ========================================================= */
void pd_Init(void);

/* =========================================================
 * 순수 계산: 한 축의 모터 명령
 *   ref_pos/ref_vel : 목표 (궤적 진행 중이면 궤적값, 끝났으면 홀드값+0)
 *   cur_pos/cur_vel : 현재 (J2는 연속각, 호출측이 변환해서 전달)
 *   하드웨어/궤적 의존 없음. 입력 → 출력만.
 * ========================================================= */
void pd_Compute(int axis,
                float ref_pos, float ref_vel,
                float cur_pos, float cur_vel,
                PDOutput_t* out);

/* =========================================================
 * 홀드 목표 관리 (궤적 끝난 뒤 유지할 위치)
 *   ISR이 궤적 상태를 보고 갱신/조회
 * ========================================================= */
void  pd_SetGoal(int axis, float pos);   /* 궤적 진행 중 호출: 홀드 목표 갱신 */
float pd_GetGoal(int axis);              /* 궤적 끝난 후 호출: 홀드 목표 */
uint8_t pd_HasGoal(int axis);            /* 한 번이라도 목표 받았나 */
void pd_ResetGoal(int axis, float pos);

/* =========================================================
 * 런타임 설정 변경
 * ========================================================= */
void pd_SetMode(int axis, PDControlMode mode);
void pd_SetGains(int axis, float kp, float kd);
void pd_SetTorqueLimit(int axis, float limit);
void pd_SetBaseTorque(int axis, float base);

PDControlMode pd_GetMode(int axis);

#ifdef __cplusplus
}
#endif

#endif /* __PD_CONTROL_H__ */