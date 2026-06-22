#include "config.h"
#include "motion.h"

// Reference gains only.
// External PD torque control uses pd_control.cpp gains and sends MIT kp/kd = 0.
const float MOTOR_KP[4] = { 0.0f, 30.0f, 15.0f, 7.5f };
const float MOTOR_KD[4] = { 0.0f,  1.5f,  1.5f, 1.5f };

// ============================================================
//  Soft limits
// ============================================================

// M1/M2: degree, M3: millimeter.
// These are example ranges. Adjust them to the real robot mechanism.
const AxisLimitDeg LIMIT_M1 = { -180.0f, 360.0f };
const AxisLimitDeg LIMIT_M2 = { -105.0f, 360.0f };
const AxisLimitMm  LIMIT_M3 = { -300.0f, 300.0f };


// ============================================================
//  Demo motion sequence
// ============================================================
//
// type,                M1deg,  M2deg,  M3mm, duration, wait, name
extern const MotionStep DEMO_SEQ[] = {
  // type,                M1deg,  M2deg,  M3mm, duration, wait, name

  // ============================================================
  // 1. 튀김망에서 바스켓 잡기
  // ============================================================

  // 1-1. 튀김망 위 대기 위치로 이동
  { STEP_MOVE_ABS,      -150.0f,    5.0f,   0.0f,   5.0f, 500, "net_above" },

  // 1-2. 바스켓 접근 위치로 이동
  { STEP_MOVE_ABS,      -150.0f,    -3.0f,   0.0f,   3.0f, 500, "net_approach" },

  // 1-3. 바스켓 잡기 위치로 최종 접근
  { STEP_MOVE_ABS,      -147.0f,    -20.0f,   0.0f,   3.0f, 500, "net_grip_pos" },

  // 1-4. 솔레노이드 잠금 / 바스켓 잡기
  { STEP_MOVE_ABS,      -146.0f,    -32.0f,   0.0f,   3.0f, 500, "net_grip" },

  // 1-5. 바스켓 들기
  { STEP_MOVE_ABS,      -144.0f,    0.0f,   0.0f,   5.0f, 500, "net_lift" },


  //============================================================
  //2. 튀김기로 이동 후 바스켓 튀김기에 내려놓기
  //============================================================

  // 2-1. 안전 이동 위치로 후퇴
  { STEP_MOVE_ABS,      -144.0f,    -10.0f,   301.0f,   5.0f, 500, "move_safe_1" },

  // 2-2. 튀김기 위 위치로 이동
  { STEP_MOVE_ABS,      -130.0f,    -80.0f,   301.0f,   4.0f, 500, "fryer_above" },
  
  // 2-3. 튀김기 위 위치로 이동
  { STEP_MOVE_ABS,      -125.0f,    -85.0f,   301.0f,   4.0f, 500, "fryer_above" },

  // 2-4. 튀김기 투입 위치로 접근
  { STEP_MOVE_ABS,      -120.0f,    -92.0f,   301.0f,   4.0f, 500, "fryer_approach" },
  
  // 2-5. 솔레노이드 해제 / 바스켓 놓기
  { STEP_SOL_UNLOCK,       0.0f,      0.0f,     0.0f,   0.0f, 100, "fryer_release" },
  
  // 2-6. 튀김기 위 위치로 이동
  { STEP_MOVE_ABS,      -125.0f,    -85.0f,   301.0f,   1.0f, 500, "fryer_above" },

  // 2-7. 그리퍼 후퇴
  { STEP_MOVE_ABS,      -130.0f,    -80.0f,   301.0f,   3.0f, 500, "fryer_retract" },


  //============================================================
  //3. 튀김기에서 바스켓 잡기
  //============================================================

  // 3-1. 튀김기 위 대기 위치로 이동
  { STEP_MOVE_ABS,      -144.0f,    -10.0f,   301.0f,   3.0f, 500, "fryer_pick_above" },

  // 3-2. 바스켓 접근 위치로 이동
  { STEP_MOVE_ABS,      -130.0f,    -80.0f,   301.0f,   3.0f, 500, "fryer_pick_approach" },

  // 3-3. 바스켓 잡기 위치로 최종 접근
  { STEP_MOVE_ABS,      -125.0f,    -85.0f,   301.0f,   3.0f, 500, "fryer_pick_pos" },

  // 3-4. 솔레노이드 잠금 / 바스켓 잡기
  { STEP_MOVE_ABS,      -120.0f,    -92.0f,   301.0f,   1.0f, 300, "fryer_grip" },

  // 3-5. 바스켓 들어올리기
  { STEP_MOVE_ABS,      -125.0f,    -85.0f,   301.0f,   4.0f, 500, "fryer_lift" },

  { STEP_MOVE_ABS,      -144.0f,    -10.0f,   301.0f,   5.0f, 500, "fryer_lift" },


  // ============================================================
  // 4. 흔들기
  // ============================================================

  // // 4-1. 흔들기 시작 위치로 이동
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   2.0f, 300, "shake_ready" },

  // // 4-2. 흔들기 1회차 - 한쪽 방향
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   0.6f, 100, "shake_1_a" },

  // // 4-3. 흔들기 1회차 - 반대 방향
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   0.6f, 100, "shake_1_b" },

  // // 4-4. 흔들기 2회차 - 한쪽 방향
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   0.6f, 100, "shake_2_a" },

  // // 4-5. 흔들기 2회차 - 반대 방향
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   0.6f, 100, "shake_2_b" },

  // // 4-6. 흔들기 종료 후 안정 위치
  // { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   1.0f, 500, "shake_end" },


  //============================================================
  //5. 튀김망으로 이동 후 바스켓 내려놓기
  //============================================================

  // 5-1. 안전 이동 위치로 이동
  { STEP_MOVE_ABS,      -150.0f,    5.0f,   0.0f,   5.0f, 500, "move_safe_2" },

  // 5-2. 튀김망 위 위치로 이동
  { STEP_MOVE_ABS,      -151.0f,    -3.0f,   0.0f,   3.0f, 500, "net_return_above" },

  // 5-3. 튀김망 내려놓기 접근 위치
  { STEP_MOVE_ABS,      -149.0f,    -20.0f,   0.0f,   3.0f, 500, "net_return_approach" },

  // 5-4. 바스켓 내려놓기 위치까지 하강
  { STEP_MOVE_ABS,      -145.0f,    -30.0f,   0.0f,   2.0f, 500, "net_place" },

  // 5-5. 솔레노이드 해제 / 바스켓 놓기
  { STEP_SOL_UNLOCK,    0.0f,    0.0f,   0.0f,   0.0f, 100, "net_release" },

  // 5-6. 그리퍼 후퇴
  { STEP_MOVE_ABS,      -149.0f,    -20.0f,   0.0f,   1.5f, 500, "net_retract" },

  { STEP_MOVE_ABS,      -150.0f,    5.0f,   0.0f,   5.0f, 500, "move_safe_2" },

  // // 5-7. 홈 위치 복귀
  { STEP_MOVE_ABS,      0.0f,    0.0f,   0.0f,   4.0f, 300, "home" },
};

extern const uint8_t DEMO_SEQ_COUNT = sizeof(DEMO_SEQ) / sizeof(DEMO_SEQ[0]);