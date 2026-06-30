/**
 * @file kin_bridge.h
 * @author geon lee (sweng.geon@gmail.com)
 * @brief 운동학 연동 레이어 + Cartesian 추종 러너 인터페이스
 * @version 0.3
 * @date 2026-06-29
 *
 * @details
 * robot_ik(URDF 기반)와 우리 모터 축(J0/J1/J2) 사이의 변환을 한 곳에 모은다.
 * 축 매핑/단위 변환이 틀어지면 이 파일만 보면 된다.
 *
 *  축 매핑:  IK slider_m → J2 (외부PD 슬라이드, mm),
 *            IK l1_rad   → J1,
 *            IK l2_rad   → J0
 *  단위:     IK slider = meter (우리 J2 = mm),  IK l1/l2 = rad (우리 J0/J1 = rad)
 *
 * 제공 기능:
 *   - kin_SolveGoto      : 단발 멀티스타트 IK (GOTO 명령용)
 *   - Cartesian 추종 러너 : 좌표 궤적을 매 주기 warm-start IK로 풀어 추종.
 *       · cart_Update 시간 진행은 ISR이 수행 (trajectory 모듈)
 *       · kin_RunnerUpdate IK 풀이는 메인루프가 수행 (무거움)
 *       · kin_GetJointGoal 관절목표 복사는 ISR이 수행 (가벼움, 임계구역 보호)
 */

#ifndef __KIN_BRIDGE_H__
#define __KIN_BRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================================================
 * 자료형
 * ========================================================= */

/** @brief 단발 IK(GOTO) 결과: 각 축 목표값 (우리 축 기준) */
typedef struct {
    float   j0_rad;   ///< = IK l2
    float   j1_rad;   ///< = IK l1
    float   j2_rad;   ///< = IK slider → 연속각 rad
    uint8_t ok;       ///< 1이면 IK 해를 찾음
    float   error_mm; ///< IK 잔차 (mm)
} KinTarget_t;

/** @brief Cartesian 추종 러너 상태 */
typedef enum {
    KIN_RUN_IDLE = 0, ///< 정지 (추종 안 함)
    KIN_RUN_ACTIVE,   ///< 궤적 진행 + 매 주기 IK 추종
    KIN_RUN_HOLD      ///< 궤적 종료, 마지막 관절목표 유지
} KinRunState;

/* =========================================================
 * 단발 IK (GOTO)
 * ========================================================= */

/**
 * @brief 목표점(x,y,z mm)을 멀티스타트 IK로 풀어 각 축 목표를 계산한다.
 * @param[in]  x_mm      목표 X (mm)
 * @param[in]  y_mm      목표 Y (mm)
 * @param[in]  z_mm      목표 Z (mm)
 * @param[in]  cur_j0_rad 현재 J0 각도 (시드용, rad)
 * @param[in]  cur_j1_rad 현재 J1 각도 (시드용, rad)
 * @param[in]  cur_j2_mm  현재 J2 슬라이드 위치 (시드용, mm)
 * @param[out] out        우리 축 목표값
 */
void kin_SolveGoto(float x_mm, float y_mm, float z_mm,
                   float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                   KinTarget_t* out);

/**
 * @brief warm-start step IK를 수렴까지 반복해 도달성을 점검한다 (진단용).
 *
 * Cartesian 러너를 시작하지 않고, 현재 자세 시드에서 한 스텝씩 추종하며
 * 최대 200회 반복한다. 잔차가 허용 이내로 떨어지면 out->ok=1.
 * STEPTEST 명령에서 "이 점이 warm-start로 도달 가능한가"를 보는 용도이며,
 * 공유 버퍼나 러너 상태를 건드리지 않고 실제 모터도 움직이지 않는다.
 *
 * @param[in]  x_mm       목표 X (mm)
 * @param[in]  y_mm       목표 Y (mm)
 * @param[in]  z_mm       목표 Z (mm)
 * @param[in]  cur_j0_rad 현재 J0 각도 (시드용, rad)
 * @param[in]  cur_j1_rad 현재 J1 각도 (시드용, rad)
 * @param[in]  cur_j2_mm  현재 J2 슬라이드 위치 (시드용, mm)
 * @param[out] out        우리 축 목표값 (ok = 잔차 허용 이내 여부)
 */
void kin_SolveStepOnce(float x_mm, float y_mm, float z_mm,
                       float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                       KinTarget_t* out);

/* =========================================================
 * Cartesian 추종 러너
 * ========================================================= */

/**
 * @brief Cartesian 추종을 시작한다 (현재 자세 → 목표점 직선 궤적).
 *
 * 현재 관절값을 warm-start 시드와 궤적 시작점으로 잡고, 목표 TCP까지의
 * 직선 Cartesian 궤적(trajectory 모듈)을 시작한다. 시작 시점의 관절 목표를
 * 공유 버퍼에 즉시 채워 ISR이 끊김 없이 읽도록 한다.
 *
 * @param[in] x_mm       목표 X (mm)
 * @param[in] y_mm       목표 Y (mm)
 * @param[in] z_mm       목표 Z (mm)
 * @param[in] T          이동시간 (s)
 * @param[in] cur_j0_rad 현재 J0 각도 (rad)
 * @param[in] cur_j1_rad 현재 J1 각도 (rad)
 * @param[in] cur_j2_mm  현재 J2 슬라이드 위치 (mm)
 */
void kin_StartCartesian(float x_mm, float y_mm, float z_mm, float T,
                        float cur_j0_rad, float cur_j1_rad, float cur_j2_mm);

/**
 * @brief Cartesian 추종을 정지한다 (러너 IDLE).
 */
void kin_StopCartesian(void);

/**
 * @brief [메인루프] 최신 목표 XYZ를 읽어 warm-start IK로 풀고 관절목표를 갱신한다.
 *
 * 무거운 IK 풀이를 담당한다. ISR이 cart_Update로 진행시킨 최신 목표 좌표를
 * 읽어 RobotIK_SolveStep으로 풀고, 결과 관절목표를 임계구역 보호하에
 * 공유 버퍼에 저장한다. 궤적 종료 시 러너를 HOLD로 전환한다.
 * 러너가 IDLE이면 아무 일도 하지 않는다.
 */
void kin_RunnerUpdate(void);

/**
 * @brief [ISR] 최신 관절목표를 복사한다 (IK 없음, 임계구역 보호).
 * @param[out] joint_goal_rad J0,J1,J2 관절목표를 받을 배열 [3] (J2는 연속각 rad)
 * @return 1이면 유효한 목표(추종/홀드 중), 0이면 목표 없음(IDLE)
 */
uint8_t kin_GetJointGoal(float joint_goal_rad[3]);

/**
 * @brief 현재 러너 상태를 반환한다.
 * @return KinRunState
 */
KinRunState kin_GetRunState(void);

/**
 * @brief 마지막 IK 추종 잔차를 반환한다 (mm, 진단용).
 * @return 잔차 (mm)
 */
float kin_GetTrackError(void);

#ifdef __cplusplus
}
#endif

#endif /* __KIN_BRIDGE_H__ */
