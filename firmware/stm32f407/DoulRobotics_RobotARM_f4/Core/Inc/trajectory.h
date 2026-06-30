/**
 * @file trajectory.h
 * @author geon lee (sweng.geon@gmail.com)
 * @brief 5차 다항식(Quintic) 궤적 생성 모듈 인터페이스
 * @version 0.3
 * @date 2026-06-29
 *
 * @details
 * 두 종류의 궤적을 제공한다.
 *   - 관절 궤적(Joint) : 축별 rad 목표를 quintic 보간. MOVE/GOTO(joint-space)용.
 *   - 직교 궤적(Cartesian) : TCP 좌표(x,y,z mm)를 quintic 보간. 매 제어주기
 *                            IK로 풀어 추종하는 Cartesian-space 모션용.
 *
 * 두 궤적 모두 시작·끝에서 속도/가속도가 0이 되는 부드러운 프로파일이며,
 * 본 모듈은 config.h 외 의존성이 없는 순수 계산 모듈이다.
 * (CAN/모터/IK를 모름 → 다른 보드·시뮬에 재사용 가능)
 */

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

/** @brief 궤적 한 점의 위치/속도/가속도 */
typedef struct {
    float q;    ///< 위치 (rad 또는 mm)
    float qd;   ///< 속도 (rad/s 또는 mm/s)
    float qdd;  ///< 가속도 (rad/s^2 또는 mm/s^2)
} TrajPoint_t;

/** @brief 단일 관절 축의 궤적 상태 */
typedef struct {
    float   q0;     ///< 시작 위치 (rad)
    float   qf;     ///< 끝 위치 (rad)
    float   T;      ///< 총 이동시간 (s)
    float   t;      ///< 현재 경과시간 (s)
    uint8_t active; ///< 1이면 궤적 진행 중
    float   q_cur;  ///< 현재 목표 위치 (rad)
    float   qd_cur; ///< 현재 목표 속도 (rad/s)
    float   kp;     ///< 시작 시점에 캡처한 Kp
    float   kd;     ///< 시작 시점에 캡처한 Kd
} TrajAxis_t;

/** @brief 직교(Cartesian) 궤적 상태 (x,y,z 공통 시간) */
typedef struct {
    float   p0[3];    ///< 시작 좌표 x,y,z (mm)
    float   pf[3];    ///< 끝 좌표 x,y,z (mm)
    float   T;        ///< 총 이동시간 (s)
    float   t;        ///< 현재 경과시간 (s)
    uint8_t active;   ///< 1이면 궤적 진행 중
    float   p_cur[3]; ///< 현재 목표 좌표 x,y,z (mm)
    float   v_cur[3]; ///< 현재 목표 속도 x,y,z (mm/s)
} CartTraj_t;

/* =========================================================
 * 전역 변수
 * ========================================================= */
extern TrajAxis_t g_traj[MOTOR_COUNT]; ///< 관절 궤적 상태 [0]=J0 [1]=J1 [2]=J2
extern CartTraj_t g_cart;              ///< 직교 궤적 상태 (단일)

/* =========================================================
 * 공용 보간기
 * ========================================================= */

/**
 * @brief 5차 다항식 궤적을 평가한다.
 * @param[in]  q0  시작 위치
 * @param[in]  qf  끝 위치
 * @param[in]  T   총 이동시간 (s), 0 이하면 끝값 반환
 * @param[in]  t   현재 시간 (s), [0,T]로 clamp됨
 * @param[out] out 결과 위치/속도/가속도
 * @note 시작·끝에서 속도와 가속도가 0이 되도록 보장한다.
 */
void quintic_eval(float q0, float qf, float T, float t, TrajPoint_t* out);

/* =========================================================
 * 관절(Joint) 궤적 API
 * ========================================================= */

/**
 * @brief 단일 관절 축 궤적을 시작한다.
 * @param[in] axis 축 번호 (0~2)
 * @param[in] q0   시작 각도 (rad)
 * @param[in] qf   끝 각도 (rad)
 * @param[in] T    이동시간 (s)
 */
void traj_Start(int axis, float q0, float qf, float T);

/**
 * @brief 3축 관절 궤적을 동시에 시작한다.
 * @param[in] q0 3축 시작 각도 배열 (rad)
 * @param[in] qf 3축 끝 각도 배열 (rad)
 * @param[in] T  공통 이동시간 (s)
 */
void traj_StartAll(const float q0[MOTOR_COUNT], const float qf[MOTOR_COUNT], float T);

/**
 * @brief 특정 관절 축 궤적을 정지한다.
 * @param[in] axis 축 번호 (0~2)
 */
void traj_Stop(int axis);

/**
 * @brief 모든 관절 축 궤적을 정지한다.
 */
void traj_StopAll(void);

/**
 * @brief 관절 궤적을 1스텝 진행한다 (경과시간 진행 + 목표 계산).
 * @note ISR 컨텍스트에서 실행되므로 무거운 작업을 하지 않는다.
 */
void traj_Update(void);

/**
 * @brief 관절 축의 궤적 활성 여부를 반환한다.
 * @param[in] axis 축 번호 (0~2)
 * @return 1이면 활성
 */
uint8_t traj_IsActive(int axis);

/**
 * @brief 관절 축의 현재 목표 위치를 반환한다 (rad).
 * @param[in] axis 축 번호 (0~2)
 * @return 목표 위치
 */
float traj_GetTargetPos(int axis);

/**
 * @brief 관절 축의 현재 목표 속도를 반환한다 (rad/s).
 * @param[in] axis 축 번호 (0~2)
 * @return 목표 속도
 */
float traj_GetTargetVel(int axis);

/**
 * @brief 관절 축의 캡처된 Kp를 반환한다.
 * @param[in] axis 축 번호 (0~2)
 * @return Kp
 */
float traj_GetKp(int axis);

/**
 * @brief 관절 축의 캡처된 Kd를 반환한다.
 * @param[in] axis 축 번호 (0~2)
 * @return Kd
 */
float traj_GetKd(int axis);

/**
 * @brief 관절 축이 끝시간(t >= T)에 도달했는지 반환한다.
 * @param[in] axis 축 번호 (0~2)
 * @return 1이면 끝시간 도달
 */
uint8_t traj_IsFinished(int axis);

/* =========================================================
 * 직교(Cartesian) 궤적 API
 * ========================================================= */

/**
 * @brief 직교 궤적을 시작한다 (TCP 좌표 직선 보간).
 * @param[in] p0 시작 좌표 x,y,z (mm)
 * @param[in] pf 끝 좌표 x,y,z (mm)
 * @param[in] T  이동시간 (s)
 */
void cart_Start(const float p0[3], const float pf[3], float T);

/**
 * @brief 직교 궤적을 정지한다.
 */
void cart_Stop(void);

/**
 * @brief 직교 궤적을 1스텝 진행한다 (경과시간 진행 + 목표 좌표 계산).
 * @note ISR 컨텍스트에서 실행되며 IK는 풀지 않는다 (좌표만 계산).
 */
void cart_Update(void);

/**
 * @brief 직교 궤적 활성 여부를 반환한다.
 * @return 1이면 활성
 */
uint8_t cart_IsActive(void);

/**
 * @brief 현재 목표 TCP 좌표를 반환한다 (mm).
 * @param[out] xyz 목표 좌표 x,y,z를 받을 배열 [3]
 */
void cart_GetTargetPos(float xyz[3]);

/**
 * @brief 현재 목표 TCP 속도를 반환한다 (mm/s).
 * @param[out] vxyz 목표 속도 vx,vy,vz를 받을 배열 [3]
 */
void cart_GetTargetVel(float vxyz[3]);

/**
 * @brief 직교 궤적이 끝시간(t >= T)에 도달했는지 반환한다.
 * @return 1이면 끝시간 도달
 */
uint8_t cart_IsFinished(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_H__ */
