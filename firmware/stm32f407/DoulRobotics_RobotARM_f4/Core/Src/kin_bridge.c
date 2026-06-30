/**
 * @file kin_bridge.c
 * @author geon lee (sweng.geon@gmail.com)
 * @brief 운동학 연동 레이어 + Cartesian 추종 러너 구현부
 * @version 0.3
 * @date 2026-06-29
 *
 * @details
 * IK(robot_ik) ↔ 우리 축(axis) 변환과 Cartesian 추종 러너를 구현한다.
 * 러너는 비대칭 주기(ISR 1ms 고정 / 메인루프 IK 가변)를 안전하게 잇기 위해
 * 관절목표 공유 버퍼를 임계구역(__disable_irq)으로 보호한다.
 *
 * 역할:
 *   - 시간 진행 (cart_Update)       : ISR (trajectory 모듈에서 수행)
 *   - IK 풀이 (kin_RunnerUpdate)    : 메인루프 (무거움)
 *   - 관절목표 읽기 (kin_GetJointGoal): ISR (가벼움, 임계구역 복사)
 */

#include "kin_bridge.h"

#include "robot_ik.h"
#include "trajectory.h"
#include "axis.h"
#include "config.h"

#ifdef STM32F407xx
#include "stm32f4xx_hal.h"  /* __disable_irq / __enable_irq */
#define KIN_ENTER_CRITICAL() __disable_irq()
#define KIN_EXIT_CRITICAL()  __enable_irq()
#else
/* PC 시뮬레이션: 임계구역 무동작 (단일스레드) */
#define KIN_ENTER_CRITICAL() ((void)0)
#define KIN_EXIT_CRITICAL()  ((void)0)
#endif

/* warm-start step IK 파라미터 (1ms 주기 기준) */
#define KIN_STEP_MAX_ITER       4       ///< 한 호출 최대 반복
#define KIN_STEP_MAX_SLIDER_M   0.003f  ///< 한 호출 slider 변화 상한 (3mm/주기)
#define KIN_STEP_MAX_RAD        0.05f   ///< 한 호출 회전축 변화 상한 (~2.9deg/주기)

/* =========================================================
 * 러너 내부 상태 (메인루프/ISR 공유)
 * ========================================================= */

/* ISR이 읽는 공유 관절목표 버퍼 (임계구역 보호 대상) */
static volatile float   s_joint_goal[3] = {0};  ///< J0,J1,J2 목표 (J2=연속각 rad)
static volatile uint8_t s_goal_valid    = 0;    ///< 1이면 유효 목표 존재

/* 러너 상태 (메인루프가 갱신, ISR은 읽기만) */
static volatile KinRunState s_run_state = KIN_RUN_IDLE;

/* warm-start 시드: 직전 IK 해 (메인루프 전용) */
static RobotIKJoint s_prev_q  = {0};
static float        s_track_err_mm = 0.0f;

/* =========================================================
 * 내부 유틸: IK 해(slider m, l1, l2) → 우리 축 목표(J0,J1,J2 rad)
 *   매핑: l2→J0, l1→J1, slider(m)→J2 연속각(rad)
 * ========================================================= */
static void ik_to_joint_goal(const RobotIKJoint* q, float goal[3])
{
    float j2_mm = q->slider_m * 1000.0f;       /* m → mm */
    goal[0] = q->l2_rad;                       /* J0 */
    goal[1] = q->l1_rad;                       /* J1 */
    goal[2] = axis_J2_MmToRad(j2_mm);          /* J2 연속각 rad */
}

/* 우리 축 시드(J0,J1 rad / J2 mm) → IK 시드(slider m, l1, l2 rad) */
static void joint_to_ik_seed(float cur_j0_rad, float cur_j1_rad,
                             float cur_j2_mm, RobotIKJoint* seed)
{
    seed->slider_m = cur_j2_mm / 1000.0f;      /* mm → m */
    seed->l1_rad   = cur_j1_rad;               /* J1 → l1 */
    seed->l2_rad   = cur_j0_rad;               /* J0 → l2 */
}

/* 공유 버퍼에 관절목표 저장 (임계구역 보호) */
static void store_joint_goal(const float goal[3])
{
    KIN_ENTER_CRITICAL();
    s_joint_goal[0] = goal[0];
    s_joint_goal[1] = goal[1];
    s_joint_goal[2] = goal[2];
    s_goal_valid    = 1;
    KIN_EXIT_CRITICAL();
}

/* =========================================================
 * 단발 IK (GOTO) — 멀티스타트
 * ========================================================= */
void kin_SolveGoto(float x_mm, float y_mm, float z_mm,
                   float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                   KinTarget_t* out)
{
    /* 현재 관절을 IK 시드로 (연속성 위해) */
    RobotIKJoint seed;
    joint_to_ik_seed(cur_j0_rad, cur_j1_rad, cur_j2_mm, &seed);

    /* IK 풀기 */
    RobotIKResult res;
    RobotIKStatus st = RobotIK_SolveXYZ(x_mm, y_mm, z_mm, &seed, &res);

    /* 결과를 우리 축으로 매핑 */
    out->ok       = (st == ROBOT_IK_OK) ? 1 : 0;
    out->error_mm = res.error_mm;

    float j2_mm   = res.q.slider_m * 1000.0f;
    out->j2_rad   = axis_J2_MmToRad(j2_mm);
    out->j1_rad   = res.q.l1_rad;   /* l1 → J1 */
    out->j0_rad   = res.q.l2_rad;   /* l2 → J0 */
}

/* 진단용: warm-start step IK를 수렴까지 반복 (도달성 점검)
 *   STEPTEST는 IKTEST처럼 "이 점이 warm-start로 도달 가능한가"를 본다.
 *   현재 자세 시드에서 출발해 한 스텝씩 추종하며 최대 N회 반복,
 *   잔차가 허용 이내로 떨어지면 ok=1. 실제 모터는 움직이지 않는다. */
void kin_SolveStepOnce(float x_mm, float y_mm, float z_mm,
                       float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                       KinTarget_t* out)
{
    RobotIKJoint q;
    joint_to_ik_seed(cur_j0_rad, cur_j1_rad, cur_j2_mm, &q);
    RobotIK_ClampJoint(&q);

    RobotIKResult res;
    res.error_mm = 1.0e30f;

    /* 한 스텝 변화량이 clamp되므로, 먼 목표는 여러 스텝 필요.
     * 200회면 slider 3mm×200=600mm, 회전 0.05rad×200=10rad까지 커버.
     * 수렴 판정은 st 플래그가 아니라 실제 잔차로 한다:
     * SolveStep은 내부 수렴 후 prev_q 기준 max_step clamp를 적용하므로,
     * st=OK라도 clamp로 되돌려진 q의 잔차는 클 수 있다(반복 추종 시). */
    for (int n = 0; n < 200; n++)
    {
        RobotIK_SolveStep(x_mm, y_mm, z_mm, &q,
                          KIN_STEP_MAX_ITER,
                          KIN_STEP_MAX_SLIDER_M,
                          KIN_STEP_MAX_RAD,
                          &res);
        q = res.q;
        if (res.error_mm <= 1.0f) break;   /* 실제 잔차 기준 */
    }

    out->ok       = (res.error_mm <= 1.0f) ? 1 : 0;
    out->error_mm = res.error_mm;

    float j2_mm   = res.q.slider_m * 1000.0f;
    out->j2_rad   = axis_J2_MmToRad(j2_mm);
    out->j1_rad   = res.q.l1_rad;
    out->j0_rad   = res.q.l2_rad;
}

/* =========================================================
 * Cartesian 추종 러너
 * ========================================================= */
void kin_StartCartesian(float x_mm, float y_mm, float z_mm, float T,
                        float cur_j0_rad, float cur_j1_rad, float cur_j2_mm)
{
    /* 1. warm-start 시드를 현재 관절로 초기화 */
    joint_to_ik_seed(cur_j0_rad, cur_j1_rad, cur_j2_mm, &s_prev_q);
    RobotIK_ClampJoint(&s_prev_q);

    /* 2. 현재 TCP(FK) → 목표 TCP 직선 궤적 시작 */
    RobotIKPoint p_start;
    RobotIK_FK(&s_prev_q, &p_start);

    float p0[3] = { p_start.x_mm, p_start.y_mm, p_start.z_mm };
    float pf[3] = { x_mm, y_mm, z_mm };
    cart_Start(p0, pf, T);

    /* 3. 시작 관절목표를 즉시 공유버퍼에 채움 (ISR 끊김 방지) */
    float goal[3];
    ik_to_joint_goal(&s_prev_q, goal);
    store_joint_goal(goal);

    s_track_err_mm = 0.0f;
    s_run_state    = KIN_RUN_ACTIVE;
}

void kin_StopCartesian(void)
{
    cart_Stop();
    s_run_state = KIN_RUN_IDLE;
    /* s_goal_valid는 유지: 마지막 목표를 홀드하도록 ISR에 남겨둠 */
}

/* [메인루프] 무거운 IK 풀이 */
void kin_RunnerUpdate(void)
{
    if (s_run_state == KIN_RUN_IDLE) return;

    /* 1. ISR이 진행시킨 최신 목표 XYZ 읽기 */
    float xyz[3];
    cart_GetTargetPos(xyz);

    /* 2. warm-start IK (직전 해 시드, 반복 제한, 변화량 clamp) */
    RobotIKResult res;
    RobotIK_SolveStep(xyz[0], xyz[1], xyz[2],
                      &s_prev_q,
                      KIN_STEP_MAX_ITER,
                      KIN_STEP_MAX_SLIDER_M,
                      KIN_STEP_MAX_RAD,
                      &res);

    /* 3. 결과를 시드로 갱신 + 우리 축 목표로 매핑해 공유버퍼 저장 */
    s_prev_q       = res.q;
    s_track_err_mm = res.error_mm;

    float goal[3];
    ik_to_joint_goal(&res.q, goal);
    store_joint_goal(goal);

    /* 4. 궤적 종료 판정 → HOLD 전환 (목표는 마지막 값 유지) */
    if (cart_IsFinished())
    {
        cart_Stop();
        s_run_state = KIN_RUN_HOLD;
    }
}

/* [ISR] 관절목표 복사 (가벼움) */
uint8_t kin_GetJointGoal(float joint_goal_rad[3])
{
    uint8_t valid;

    KIN_ENTER_CRITICAL();
    valid = s_goal_valid;
    joint_goal_rad[0] = s_joint_goal[0];
    joint_goal_rad[1] = s_joint_goal[1];
    joint_goal_rad[2] = s_joint_goal[2];
    KIN_EXIT_CRITICAL();

    return valid;
}

KinRunState kin_GetRunState(void)
{
    return s_run_state;
}

float kin_GetTrackError(void)
{
    return s_track_err_mm;
}
