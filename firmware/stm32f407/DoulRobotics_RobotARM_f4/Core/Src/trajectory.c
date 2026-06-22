//###########################################################################
// FILE:    trajectory.c
// TITLE:   5차 다항식(Quintic) 다축 동시 궤적 생성 (순수 계산)
//          STM32F407G-DISC1 / CubeIDE
//
// 구조: TIM2 ISR(5ms)에서 traj_Update() 호출 → 목표값 계산/저장만
//   - CAN 송신은 하지 않음 (RobStride_MIT 의존 제거)
//   - 송신은 ISR이 getter로 결과를 읽어서 직접 처리
//
// 의존: config.h (MOTOR_COUNT, CTRL_DT, g_axis_cfg) 만.
//       RobStride_MIT 는 모름 → 다른 모터/보드/시뮬에 재사용 가능
//###########################################################################

#include "trajectory.h"

/* =========================================================
 * 다축 궤적 상태
 * ========================================================= */
TrajAxis_t g_traj[MOTOR_COUNT] = {0};

/* =========================================================
 * 순수 계산: 5차 다항식
 *   s(tau)   = 10tau^3 - 15tau^4 + 6tau^5
 *   s'(tau)  = (30tau^2 - 60tau^3 + 30tau^4) / T
 *   s''(tau) = (60tau - 180tau^2 + 120tau^3) / T^2
 * ========================================================= */
void quintic_eval(float q0, float qf, float T, float t, TrajPoint_t* out) {
    /* T가 0 이하면 즉시 끝값 반환 (0 나누기 방지) */
    if (T <= 0.0f) {
        out->q = qf; out->qd = 0.0f; out->qdd = 0.0f;
        return;
    }
    /* 범위 밖이면 clamp (끝나면 끝값 유지) */
    if (t < 0.0f)  t = 0.0f;
    if (t > T)     t = T;

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
 * 궤적 시작 / 정지
 * ========================================================= */
void traj_Start(int axis, float q0, float qf, float T) {
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

void traj_StartAll(const float q0[MOTOR_COUNT], const float qf[MOTOR_COUNT], float T) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        traj_Start(i, q0[i], qf[i], T);
}

void traj_Stop(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return;
    g_traj[axis].active = 0;
}

void traj_StopAll(void) {
    for (int i = 0; i < MOTOR_COUNT; i++)
        g_traj[i].active = 0;
}

/* =========================================================
 * 궤적 스텝 진행 (TIM2 ISR 5ms) — 계산/저장만
 *   종료 판정은 송신 이후에 해야 끝점이 송신됨.
 *   하지만 송신이 ISR로 빠졌으므로, 여기서 active를 끄지 않고
 *   "끝시간 도달" 상태만 유지 → 송신측이 끝점 보낸 뒤 traj_Stop.
 *   단순화를 위해: 끝점까지 계산은 계속하되, t가 T를 넘어도
 *   clamp 되어 끝값을 반복 출력 → 송신측이 도달 판단해 Stop 호출.
 * ========================================================= */
void traj_Update(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!g_traj[i].active) continue;

        /* 경과 시간 진행 */
        g_traj[i].t += CTRL_DT;

        /* 목표 계산 (t > T 면 quintic_eval 내부에서 clamp) */
        TrajPoint_t pt;
        quintic_eval(g_traj[i].q0, g_traj[i].qf, g_traj[i].T, g_traj[i].t, &pt);

        /* 결과 저장만 (CAN 송신 안 함) */
        g_traj[i].q_cur  = pt.q;
        g_traj[i].qd_cur = pt.qd;
    }
}

/* =========================================================
 * 결과 읽기 (송신측에서 사용)
 * ========================================================= */
uint8_t traj_IsActive(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0;
    return g_traj[axis].active;
}

float traj_GetTargetPos(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].q_cur;
}

float traj_GetTargetVel(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].qd_cur;
}

float traj_GetKp(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].kp;
}

float traj_GetKd(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0.0f;
    return g_traj[axis].kd;
}

/* =========================================================
 * 끝시간 도달 여부 (송신측이 끝점 송신 후 Stop 판단용)
 *   t >= T 이면 1 반환. active와 별개 (끝점 송신 보장).
 * ========================================================= */
uint8_t traj_IsFinished(int axis) {
    if (axis < 0 || axis >= MOTOR_COUNT) return 0;
    if (!g_traj[axis].active) return 0;
    return (g_traj[axis].t >= g_traj[axis].T) ? 1 : 0;
}