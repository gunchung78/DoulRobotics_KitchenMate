#ifndef __KIN_BRIDGE_H__
#define __KIN_BRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================================================
 * 운동학 연동 레이어 (IK ↔ 우리 축)
 *
 *  축 매핑 (한 곳에서만 정의):
 *    IK slider_m → J2 (외부PD 슬라이드, mm)
 *    IK l1_rad   → J1
 *    IK l2_rad   → J0
 *
 *  단위:
 *    IK slider = meter, 우리 J2 = mm
 *    IK l1/l2  = rad,   우리 J0/J1 = rad
 * ========================================================= */

/* GOTO 결과: 각 축 목표값 (우리 축 기준) */
typedef struct {
    float j0_rad;   /* = IK l2 */
    float j1_rad;   /* = IK l1 */
    float j2_rad;   /* = IK slider → 연속각 rad */
    uint8_t ok;     /* 1이면 IK 해 찾음 */
    float error_mm; /* IK 잔차 */
} KinTarget_t;

/* =========================================================
 * 목표점(x,y,z mm)을 풀어 각 축 목표를 계산
 *   cur_j0/j1 : 현재 회전축 각도 (시드용, rad)
 *   cur_j2_mm : 현재 J2 슬라이드 위치 (시드용, mm)
 *   out       : 우리 축 목표값
 * ========================================================= */
void kin_SolveGoto(float x_mm, float y_mm, float z_mm,
                   float cur_j0_rad, float cur_j1_rad, float cur_j2_mm,
                   KinTarget_t* out);

#ifdef __cplusplus
}
#endif

#endif /* __KIN_BRIDGE_H__ */