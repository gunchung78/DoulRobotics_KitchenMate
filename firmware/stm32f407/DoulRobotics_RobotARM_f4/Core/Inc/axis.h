#ifndef __AXIS_H__
#define __AXIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/* =========================================================
 * J2 (슬라이드 축) 기계 변환 + Unwrap
 *
 *  메커니즘: 1 회전(2π rad) = 5mm 직선 이동
 *  메커니즘 바뀌면 axis.c 의 AXIS_J2_MM_PER_REV 만 수정
 *
 *  역할:
 *   1) rad <-> mm 변환
 *   2) MIT 피드백(±12.57 wrap)을 연속 누적각으로 unwrap
 *   3) 목표 연속각을 MIT 송신 범위로 wrap
 * ========================================================= */

/* rad <-> mm 변환 */
float axis_J2_RadToMm(float rad);
float axis_J2_MmToRad(float mm);
float axis_J2_RadpsToMmps(float radps);

/* Unwrap: 매 제어주기 호출 (피드백 갱신 후)
 *   raw_rad : 모터 피드백 각도 (RS_J2.feedback.Angle) */
void  axis_J2_Update(float raw_rad);
void  axis_J2_ResetUnwrap(float raw_rad);

/* 연속 누적 위치 */
float axis_J2_GetContinuousRad(void);
float axis_J2_GetContinuousMm(void);

/* 목표 연속각 -> MIT 송신 범위(±12.57)로 wrap */
float axis_J2_WrapRadToMitRange(float rad);

#ifdef __cplusplus
}
#endif

#endif /* __AXIS_H__ */