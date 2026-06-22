//###########################################################################
// FILE:    axis.c
// TITLE:   J2 슬라이드 축 — 회전↔직선 변환 + Unwrap (STM32F407)
//
// ESP32 axis.cpp 이식 (J2 = 원본의 M3에 해당)
//   - 회전운동을 직선운동(mm)으로 제어하기 위한 변환
//   - MIT 피드백은 ±12.57 rad로 wrap되므로, 여러 바퀴 도는
//     직선운동을 위해 누적각(continuous rad)을 별도 추적
//
// 메커니즘: 1 회전(2π rad) = 5mm
//   바뀌면 AXIS_J2_MM_PER_REV 만 수정
//###########################################################################

#include "axis.h"

/* =========================================================
 * Unwrap 상태
 * ========================================================= */
static uint8_t j2_initialized   = 0;
static float   j2_prev_raw_rad  = 0.0f;
static float   j2_wrap_offset   = 0.0f;
static float   j2_continuous    = 0.0f;

/* =========================================================
 * rad <-> mm 변환
 * ========================================================= */
float axis_J2_RadToMm(float rad)      { return rad * AXIS_J2_MM_PER_RAD; }
float axis_J2_MmToRad(float mm)       { return mm / AXIS_J2_MM_PER_RAD; }
float axis_J2_RadpsToMmps(float radps){ return radps * AXIS_J2_MM_PER_RAD; }

/* =========================================================
 * Unwrap: 매 제어주기 호출 (피드백 갱신 후)
 *   raw가 +12.5 → -12.5 로 점프하면 한 바퀴 넘어간 것
 *   → offset을 더해 연속적으로 증가하도록 보정
 * ========================================================= */
void axis_J2_Update(float raw_rad)
{
    if (!j2_initialized)
    {
        j2_prev_raw_rad = raw_rad;
        j2_wrap_offset  = 0.0f;
        j2_continuous   = raw_rad;
        j2_initialized  = 1;
        return;
    }

    float delta = raw_rad - j2_prev_raw_rad;

    /* raw: +12.5 → -12.5 (정방향으로 한 바퀴) */
    if (delta < -WRAP_THRESHOLD)
        j2_wrap_offset += MIT_POS_RANGE;
    /* raw: -12.5 → +12.5 (역방향으로 한 바퀴) */
    else if (delta > WRAP_THRESHOLD)
        j2_wrap_offset -= MIT_POS_RANGE;

    j2_continuous   = raw_rad + j2_wrap_offset;
    j2_prev_raw_rad = raw_rad;
}

void axis_J2_ResetUnwrap(float raw_rad)
{
    j2_initialized  = 0;
    j2_prev_raw_rad = 0.0f;
    j2_wrap_offset  = 0.0f;
    j2_continuous   = raw_rad;
}

float axis_J2_GetContinuousRad(void) { return j2_continuous; }
float axis_J2_GetContinuousMm(void)  { return axis_J2_RadToMm(j2_continuous); }

/* =========================================================
 * 목표 연속각 → MIT 송신 범위(±12.57)로 wrap
 *   외부 PD는 연속각으로 계산하지만, 모터로 보낼 땐
 *   MIT 범위 안의 값이어야 함
 * ========================================================= */
float axis_J2_WrapRadToMitRange(float rad)
{
    while (rad > MIT_POS_MAX) rad -= MIT_POS_RANGE;
    while (rad < MIT_POS_MIN) rad += MIT_POS_RANGE;

    if (rad > MIT_POS_MAX) rad = MIT_POS_MAX;
    if (rad < MIT_POS_MIN) rad = MIT_POS_MIN;
    return rad;
}