//###########################################################################
// FILE:    pd_control.c
// TITLE:   외부/내부 PD 제어 (STM32F407, HAL)
//
//   의존: config.h 만. (trajectory/RobStride/axis 모름)
//   순수 계산 모듈 — 입력(목표·현재) → 출력(모터명령).
//   궤적/모터/연속각은 전부 ISR(호출측)이 넘겨준다.
//
//   내부 PD (J0,J1): 위치+게인 전달 (모터가 위치제어)
//   외부 PD (J2)   : 토크 = kp*(목표-현재) + kd*(목표속도-현재속도) + base
//                    홀드: 궤적 끝나면 ISR이 마지막 목표+속도0을 넘김
//###########################################################################

#include "pd_control.h"

/* =========================================================
 * 홀드 목표 (궤적 끝난 뒤 유지할 위치)
 *   값만 저장. 갱신/조회는 ISR이 궤적 상태 보고 호출.
 * ========================================================= */
static float   s_goal_pos[MOTOR_COUNT]   = {0};
static uint8_t s_goal_valid[MOTOR_COUNT] = {0};

/* =========================================================
 * 내부 유틸: clamp
 * ========================================================= */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pd_Init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        s_goal_pos[i]   = 0.0f;
        s_goal_valid[i] = 0;
    }
}

/* =========================================================
 * 홀드 목표 관리
 * ========================================================= */
void pd_SetGoal(int axis, float pos)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    s_goal_pos[axis]   = pos;
    s_goal_valid[axis] = 1;
}

float pd_GetGoal(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return s_goal_pos[axis];
}

uint8_t pd_HasGoal(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0;
    return s_goal_valid[axis];
}

/* =========================================================
 * 순수 계산: 한 축의 모터 명령
 * ========================================================= */
void pd_Compute(int axis,
                float ref_pos, float ref_vel,
                float cur_pos, float cur_vel,
                PDOutput_t* out)
{
    if (axis < 0 || axis >= MOTOR_COUNT)
    {
        out->pos = out->vel = out->kp = out->kd = out->torque = 0.0f;
        return;
    }

    AxisConfig_t* cfg = &g_axis_cfg[axis];

    if (cfg->mode == PD_MODE_INTERNAL)
    {
        out->pos    = ref_pos;
        out->vel    = ref_vel;
        out->kp     = cfg->kp;
        out->kd     = cfg->kd;
        out->torque = cfg->base_torque;
    }
    else  /* PD_MODE_EXTERNAL */
    {
        float pos_err = ref_pos - cur_pos;
        float vel_err = ref_vel - cur_vel;

        float torque = cfg->kp * pos_err
                     + cfg->kd * vel_err
                     + cfg->base_torque;

        torque = clampf(torque, -cfg->torque_limit, cfg->torque_limit);

        out->pos    = 0.0f;
        out->vel    = 0.0f;
        out->kp     = 0.0f;
        out->kd     = 0.0f;
        out->torque = torque;
    }
}

/* =========================================================
 * 런타임 설정 변경
 * ========================================================= */
void pd_SetMode(int axis, PDControlMode mode)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_axis_cfg[axis].mode = mode;
}

void pd_SetGains(int axis, float kp, float kd)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_axis_cfg[axis].kp = kp;
    g_axis_cfg[axis].kd = kd;
}

void pd_SetTorqueLimit(int axis, float limit)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_axis_cfg[axis].torque_limit = limit;
}

void pd_SetBaseTorque(int axis, float base)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_axis_cfg[axis].base_torque = base;
}

PDControlMode pd_GetMode(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return PD_MODE_INTERNAL;
    return g_axis_cfg[axis].mode;
}