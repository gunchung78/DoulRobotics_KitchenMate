//###########################################################################
// FILE:    kin_bridge.c
// TITLE:   운동학 연동 레이어 (IK ↔ 우리 축)
//
//   robot_ik (URDF 기반)와 우리 모터 축 사이의 변환을 한 곳에 모음.
//   축 매핑/단위 변환이 틀어지면 여기만 보면 됨.
//
//   매핑:  IK slider_m → J2 (mm),  IK l1 → J1,  IK l2 → J0
//   단위:  IK slider = meter,  우리 J2 = mm
//
//   의존: robot_ik.h, axis.h (mm↔rad 변환)
//###########################################################################

#include "kin_bridge.h"
#include "robot_ik.h"
#include "axis.h"

void kin_SolveGoto(float x_mm, float y_mm, float z_mm,
                   float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                   KinTarget_t* out)
{
    /* 1. 현재 관절을 IK 시드로 (연속성 위해) */
    RobotIKJoint seed;
    seed.slider_m = cur_j2_mm / 1000.0f;   /* mm → m */
    seed.l1_rad   = cur_j1_rad;            /* J1 → l1 */
    seed.l2_rad   = cur_j0_rad;            /* J0 → l2 */

    /* 2. IK 풀기 */
    RobotIKResult res;
    RobotIKStatus st = RobotIK_SolveXYZ(x_mm, y_mm, z_mm, &seed, &res);

    /* 3. 결과를 우리 축으로 매핑 */
    out->ok       = (st == ROBOT_IK_OK) ? 1 : 0;
    out->error_mm = res.error_mm;

    /* IK slider(m) → J2 (mm → 연속각 rad) */
    float j2_mm   = res.q.slider_m * 1000.0f;
    out->j2_rad   = axis_J2_MmToRad(j2_mm);

    /* IK 회전축 → 우리 회전축 (뒤바뀜 주의) */
    out->j1_rad   = res.q.l1_rad;   /* l1 → J1 */
    out->j0_rad   = res.q.l2_rad;   /* l2 → J0 */
}