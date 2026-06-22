#pragma once
#include "config.h"

// ============================================================
//  Unified axis module
// ============================================================
//
// This file replaces:
//   - m3_axis.h
//   - axis_limit.h
//
// Main responsibility:
//   1) M3 rad <-> mm conversion
//   2) M3 software unwrap
//   3) MIT position range wrapping
//   4) M1/M2/M3 soft limit clamp
//
// Unit policy:
//   - M1/M2 external user unit: degree
//   - M3 external user unit: millimeter
//   - internal motor/MIT unit: rad, rad/s

// ============================================================
//  New recommended names
// ============================================================

// M3 conversion
float axisM3RadToMm(float rad);
float axisM3MmToRad(float mm);
float axisM3RadpsToMmps(float radps);

float axisM3RadToMeter(float rad);
float axisM3MeterToRad(float meter);
float axisM3RadpsToMps(float radps);

// M3 unwrap
void axisM3Update();
void axisM3ResetUnwrap();

float axisM3GetContinuousRad();
float axisM3GetContinuousMm();

// MIT position wrap
float axisWrapRadToMitRange(float rad);

// Soft limit
float axisClampDeg(uint8_t motorId, float targetDeg);
float axisClampM3Mm(float targetMm);
float axisClampGoalRad(uint8_t motorId, float targetRad);

void axisPrintLimits();

// ============================================================
//  Backward-compatible old names
//  These allow existing code to keep using old function names
//  after changing includes to #include "axis.h".
// ============================================================

// Old m3_axis names
void m3AxisUpdate();
void m3AxisResetUnwrap();

float m3AxisGetContinuousRad();
float m3AxisGetContinuousMillimeter();

float m3AxisRadToMillimeter(float rad);
float m3AxisMillimeterToRad(float mm);
float m3AxisRadpsToMmps(float radps);

float m3AxisRadToMeter(float rad);
float m3AxisMeterToRad(float meter);
float m3AxisRadpsToMps(float radps);

float m3AxisWrapRadToMitRange(float rad);

// Old axis_limit names
float axisLimitClampDeg(uint8_t motorId, float targetDeg);
float axisLimitClampM3Mm(float targetMm);
float axisLimitClampGoalRad(uint8_t motorId, float targetRad);
void axisLimitPrint();
