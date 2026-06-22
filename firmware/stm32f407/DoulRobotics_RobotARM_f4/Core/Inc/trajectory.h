//###########################################################################
// FILE:    trajectory.h
// TITLE:   5차 다항식(Quintic) 3축 궤적 생성 모듈
//###########################################################################

#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/* =========================================================
 * 자료형
 * ========================================================= */

/** @brief 궤적 한 점 */
typedef struct {
    float q;    /**< 위치 (rad)*/
    float qd;   /**< 속도 (rad/s) */
    float qdd;  /**< 가속도 (rad/s^2) */
} TrajPoint_t;

typedef struct {
    float q0;       /**< 시작각 () */
    float qf;       /**< 끝각 () */
    float T;        /**< 총 이동시간 (s) */
    float t;        /**< 현재 경과시간 (s)*/
    uint8_t active; /**< true면 궤적 진행 중*/
    float q_cur;    /**< 현재 목표*/
    float qd_cur;
    float kp;
    float kd;
} TrajAxis_t;

/* =========================================================
 * 전역 변수
 * ========================================================= */
extern TrajAxis_t g_traj[MOTOR_COUNT];

/* =========================================================
 * 함수
 * ========================================================= */

/**
 * @brief   5차 다항식 궤적을 평가한다.
 * @param[in]  q0   시작 위치 (rad)
 * @param[in]  qf   끝 위치 (rad)
 * @param[in]  T    총 이동시간 (s), 0 이하면 끝값 반환
 * @param[in]  t    현재 시간 (s), [0,T]로 clamp됨
 * @param[out] out  결과 위치/속도/가속도
 * @note    시작·끝에서 속도와 가속도가 0이 되도록 보장한다.
 */
void quintic_eval(float q0, float qf, float T, float t, TrajPoint_t* out);

/**
 * @brief   단일 축 궤적을 시작한다.
 * @param[in]  axis 축 번호
 * @param[in]  q0   시작
 * @param[in]  qf   목표
 * @param[in]  T    이동시간 (s)
 */
void traj_Start(int axis, float q0, float qf, float T);

/**
 * @brief   3축을 동시에 시작한다.
 * @param[in] q0 3축 시작 배열
 * @param[in] qf 3축 목표 배열
 * @param[in] T  공통 이동시간 (s)   
 */
void traj_StartAll(const float q0[MOTOR_COUNT], const float qf[MOTOR_COUNT], float T);

/**
 * @brief   특정 축의 궤적을 정지한다.
 * @param[in] axis  축 번호 (0~2)
 */
void traj_StopAll(void);

void traj_Stop(int axis);

/**
 * @brief   궤적을 1스텝 진행한다.
 * @note    활성 축의 경과시간 진행 → 목표각 계산 → CAN 송신. 
 *          ISR 컨텍스트에서 실행되므로 무거운 작업 X.
 */
void traj_Update(void);

/* =========================================================
 * 결과 읽기
 * ========================================================= */
uint8_t traj_IsActive(int axis);     /* 활성 여부 */
float   traj_GetTargetPos(int axis); /* 목표 위치 */
float   traj_GetTargetVel(int axis); /* 목표 속도 */
float   traj_GetKp(int axis);        /* 게인 Kp */
float   traj_GetKd(int axis);        /* 게인 Kd */
uint8_t traj_IsFinished(int axis);   /* 끝시간 도달 여부 */

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_H__ */
