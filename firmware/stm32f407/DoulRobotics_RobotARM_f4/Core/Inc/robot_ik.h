/**
 * @file robot_ik.h
 * @author geon lee (sweng.geon@gmail.com)
 * @brief URDF 기반 3축 로봇 순/역기구학 API 선언부
 * @version 0.2
 * @date 2026-06-29
 * 
 * @details
 * kitchenmate_v2 urdf의 순기구학과 역기구학은 제공한다.
 * 
 * IK는 두가지 제공한다.
 *  - RobotIK_SolveXYZ  : 멀티스타드 방식, 단발성 명령 사용한다.
 * 
 *  - RobotIK_SolveStep : warm-start 방식, 매 제어주기 호출용 경량 IK
 * 
 */

#ifndef ROBOT_IK_H
#define ROBOT_IK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IK 풀이 결과 상태 코드
 */
typedef enum
{
    ROBOT_IK_OK = 0,
    ROBOT_IK_MAX_ITER = 1,
    ROBOT_IK_UNREACHABLE = 2,
    ROBOT_IK_BAD_ARG = 3
} RobotIKStatus;

/**
 * @brief TCP 위치 [mm]
 */
typedef struct
{
    float slider_m; ///< 슬라이드 축 위치 (m) 범위 [0, 0.5]
    float l1_rad;   ///< l1 회전 관절 각도 (rad) 범위 [-1.8326, 1.8326]
    float l2_rad;   ///< l2 회전 관절 각도 (rad) 범위 [-π, π] 
} RobotIKJoint;

/**
 * @brief tcp 위치
 */
typedef struct
{
    float x_mm; ///< x 좌표 (mm)
    float y_mm; ///< y 좌표 (mm)
    float z_mm; ///< z 좌표 (mm)
} RobotIKPoint;

/**
 * @brief IK 풀이결과
 */
typedef struct 
{
    RobotIKJoint    q;          ///< 풀린 관절 벡터
    RobotIKPoint    tcp_mm;     ///< q로 계산한 fk 결과 tcp (mm)
    float           error_mm;   ///< tcp 잔차 (mm)
    int             iterations; ///< 실제 반복 횟수
} RobotIKResult;


/* 유틸리티 */
/**
 * @brief 영점 자세 반환한다.
 * @param q 초기화할 관절 벡터
 */
void RobotIK_GetZeroPose(RobotIKJoint *q);

/**
 * @brief 관절값 urdf 범위로 clamp한다.
 * @param q clamp 관절 각도
 */
void RobotIK_ClampJoint(RobotIKJoint *q);

/**
 * @brief rad를 deg로 변환한다.
 * @param rad 
 * @return float 
 */
float RobotIK_RadToDeg(float rad);

/**
 * @brief deg를 rad로 변환한다.
 * @param deg 
 * @return float 
 */
float RobotIK_DegToRad(float deg);


/* 순기구학 */

/**
 * @brief 순기구학 : 관절값 -> tcp 위치
 * @param q 관절 벡터
 * @param tcp_mm tcp 위치
 */
void RobotIK_FK(const RobotIKJoint *q, RobotIKPoint *tcp_mm);


/* 역기구학*/

/**
 * @brief 멀티스타트 IK 임의 목표점으로 강인하게 푼다.
 * 
 * 최대 13개 초기 시드에서 damped least squares 반복(80회) 수행
 * 단발성 GOTO 명령에 적합하다.
 * 
 * @param[in]  target_x_mm 목표 X [mm]
 * @param[in]  target_y_mm 목표 Y [mm]
 * @param[in]  target_z_mm 목표 Z [mm]
 * @param[in]  seed_q      초기 시드 (NULL이면 내부 기본 시드만 사용)
 * @param[out] out         풀이 결과
 * @return RobotIKStatus 
 */
RobotIKStatus RobotIK_SolveXYZ(float target_x,
                               float tartget_y,
                               float tartget_z,
                               const RobotIKJoint *seed_q,
                               RobotIKResult *out);

/**
 * @brief warm-start step IK 매 제어주기 cartestian 추종용 경량 IK
 * 
 * prev_q 1개만 시드로 사용하고 반복을 max_iter로 제한한다.
 * 직전 해에서 조금씩 추적하므로 보통 0~1회 반복으로 수렴한다.
 * 수렴 여부와 무관하게, 결과 관절값의 prev_q 대비 변화량을
 * 
 * @param[in]  target_x_mm      목표 X [mm]
 * @param[in]  target_y_mm      목표 Y [mm]
 * @param[in]  target_z_mm      목표 Z [mm]
 * @param[in]  prev_q           직전 주기의 관절해 (warm-start 시드)
 * @param[in]  max_iter         내부  반복 횟수 (권장 3~5)
 * @param[in]  max_step_slider_m 한 호출당 slider 변화 상한 [m]
 * @param[in]  max_step_rad     한 호출당 l1, l2 변화 상한 [rad]
 * @param[out] out              풀이 결과
 * @return ROBOT_IK_OK        잔차 허용오차 이내 수렴
 * @return ROBOT_IK_MAX_ITER  반복 내 미수렴, out은 최선 근사
 * @return ROBOT_IK_BAD_ARG   인자 오류 
 */

RobotIKStatus RobotIK_SolveStep(float target_x_mm,
                                float target_y_mm,
                                float target_z_mm,
                                const RobotIKJoint *prev_q,
                                int   max_iter,
                                float max_step_slider_m,
                                float max_step_rad,
                                RobotIKResult *out);



#ifdef __cplusplus
}
#endif

#endif
