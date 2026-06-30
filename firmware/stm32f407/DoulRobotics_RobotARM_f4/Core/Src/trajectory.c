/**
 * @file trajectory.c
 * @author geon lee (sweng.geon@gmail.com)
 * @brief 5차 다항식(Quintic) 궤적 생성 구현부
 * @version 0.3
 * @date 2026-06-29
 *
 * @details
 * 관절 궤적(g_traj)과 직교 궤적(g_cart)을 모두 생성한다.
 * TIM2 ISR(1ms)에서 traj_Update()/cart_Update()를 호출해 목표값을
 * 계산·저장만 하며, 모터 송신은 호출측(ISR)이 getter로 결과를 읽어
 * 직접 수행한다. 본 모듈은 CAN/모터/IK를 모른다.
 */

#include "trajectory.h"

/* =========================================================
 * 전역 궤적 상태
 * ========================================================= */
TrajAxis_t g_traj[MOTOR_COUNT] = {0};
CartTraj_t g_cart              = {0};

/* =========================================================
 * 공용 보간기: 5차 다항식
 *   s(τ)   = 10τ^3 - 15τ^4 + 6τ^5
 *   s'(τ)  = (30τ^2 - 60τ^3 + 30τ^4) / T
 *   s''(τ) = (60τ - 180τ^2 + 120τ^3) / T^2
 * ========================================================= */
void quintic_eval(float q0, float qf, float T, float t, TrajPoint_t* out)
{
    /* T가 0 이하면 즉시 끝값 반환 (0 나누기 방지) */
    if (T <= 0.0f) {
        out->q = qf; out->qd = 0.0f; out->qdd = 0.0f;
        return;
    }

    /* 범위 밖이면 clamp (끝나면 끝값 유지) */
    if (t < 0.0f) t = 0.0f;
    if (t > T)    t = T;

    float dq   = qf - q0;
    float tau  = t / T;
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;

    /* 시작/끝에서 속도·가속도 0 보장 */
    float s   = 10.0f*tau3 - 15.0f*tau4 + 6.0f*tau5;
    float sd  = (30.0f*tau2 - 60.0f*tau3 + 30.0f*tau4) / T;
    float sdd = (60.0f*tau - 180.0f*tau2 + 120.0f*tau3) / (T*T);

    out->q   = q0 + dq * s;
    out->qd  = dq * sd;
    out->qdd = dq * sdd;
}

/* =========================================================
 * 관절(Joint) 궤적 — 시작 / 정지
 * ========================================================= */
void traj_Start(int axis, float q0, float qf, float T)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;

    g_traj[axis].q0     = q0;
    g_traj[axis].qf     = qf;
    g_traj[axis].T      = T;
    g_traj[axis].t      = 0.0f;
    g_traj[axis].q_cur  = q0;
    g_traj[axis].qd_cur = 0.0f;
    g_traj[axis].kp     = g_axis_cfg[axis].kp;
    g_traj[axis].kd     = g_axis_cfg[axis].kd;
    g_traj[axis].active = 1;
}

void traj_StartAll(const float q0[MOTOR_COUNT], const float qf[MOTOR_COUNT], float T)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        traj_Start(i, q0[i], qf[i], T);
}

void traj_Stop(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_traj[axis].active = 0;
}

void traj_StopAll(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        g_traj[i].active = 0;
}

/* =========================================================
 * 관절 궤적 — 1스텝 진행 (계산/저장만)
 *   t가 T를 넘어도 quintic_eval 내부에서 clamp되어 끝값을 반복 출력.
 *   종료 판정/정지는 송신측(ISR)이 traj_IsFinished 보고 처리.
 * ========================================================= */
void traj_Update(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!g_traj[i].active) continue;

        /* 경과 시간 진행 */
        g_traj[i].t += CTRL_DT;

        /* 목표 계산 (t > T 면 내부에서 clamp) */
        TrajPoint_t pt;
        quintic_eval(g_traj[i].q0, g_traj[i].qf, g_traj[i].T, g_traj[i].t, &pt);

        g_traj[i].q_cur  = pt.q;
        g_traj[i].qd_cur = pt.qd;
    }
}

/* =========================================================
 * 관절 궤적 — 결과 읽기 (송신측에서 사용)
 * ========================================================= */
uint8_t traj_IsActive(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0;
    return g_traj[axis].active;
}

float traj_GetTargetPos(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].q_cur;
}

float traj_GetTargetVel(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].qd_cur;
}

float traj_GetKp(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].kp;
}

float traj_GetKd(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].kd;
}

uint8_t traj_IsFinished(int axis)
{
    if (axis < 0 || axis >= MOTOR_COUNT) return 0;
    if (!g_traj[axis].active) return 0;
    return (g_traj[axis].t >= g_traj[axis].T) ? 1 : 0;
}

/* =========================================================
 * 직교(Cartesian) 궤적 — 시작 / 정지
 *   x,y,z 3축이 공통 시간 T를 공유하며 직선 경로를 생성.
 * ========================================================= */
void cart_Start(const float p0[3], const float pf[3], float T)
{
    for (int i = 0; i < 3; i++) {
        g_cart.p0[i]    = p0[i];
        g_cart.pf[i]    = pf[i];
        g_cart.p_cur[i] = p0[i];
        g_cart.v_cur[i] = 0.0f;
    }
    g_cart.T      = T;
    g_cart.t      = 0.0f;
    g_cart.active = 1;
}

void cart_Stop(void)
{
    g_cart.active = 0;
}

/* =========================================================
 * 직교 궤적 — 1스텝 진행 (좌표만 계산, IK 없음)
 *   각 축을 quintic으로 보간. t>T면 끝값 유지.
 * ========================================================= */
void cart_Update(void)
{
    if (!g_cart.active) return;

    /* 경과 시간 진행 */
    g_cart.t += CTRL_DT;

    for (int i = 0; i < 3; i++) {
        TrajPoint_t pt;
        quintic_eval(g_cart.p0[i], g_cart.pf[i], g_cart.T, g_cart.t, &pt);
        g_cart.p_cur[i] = pt.q;
        g_cart.v_cur[i] = pt.qd;
    }
}

/* =========================================================
 * 직교 궤적 — 결과 읽기 (IK/송신측에서 사용)
 * ========================================================= */
uint8_t cart_IsActive(void)
{
    return g_cart.active;
}

void cart_GetTargetPos(float xyz[3])
{
    xyz[0] = g_cart.p_cur[0];
    xyz[1] = g_cart.p_cur[1];
    xyz[2] = g_cart.p_cur[2];
}

void cart_GetTargetVel(float vxyz[3])
{
    vxyz[0] = g_cart.v_cur[0];
    vxyz[1] = g_cart.v_cur[1];
    vxyz[2] = g_cart.v_cur[2];
}

uint8_t cart_IsFinished(void)
{
    if (!g_cart.active) return 0;
    return (g_cart.t >= g_cart.T) ? 1 : 0;
}
