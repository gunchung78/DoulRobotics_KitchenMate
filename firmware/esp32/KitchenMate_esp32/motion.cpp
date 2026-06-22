#include "motion.h"
#include "axis.h"
#include "pd_control.h"
#include "runtime.h"
#include "brake.h"

// ============================================================
//  Common constants
// ============================================================

static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;

static float degToRad(float deg) {
  return deg * DEG_TO_RAD_F;
}

static float radToDeg(float rad) {
  return rad * RAD_TO_DEG_F;
}

static float absFloat(float v) {
  return v < 0.0f ? -v : v;
}

// ============================================================
//  Trajectory state
// ============================================================
//
// 5th-order error polynomial trajectory:
//   s(u) = 10u^3 - 15u^4 + 6u^5
//   ref = start + (goal - start) * s(u)
//   refVel = (goal - start) * ds/dt

static bool trajHasReference = false;
static bool trajActive = false;
static bool trajDone = false;

static uint32_t trajStartMs = 0;
static float trajDuration = 1.0f;

static float trajStartRad[4] = {0, 0, 0, 0};
static float trajGoalRad[4]  = {0, 0, 0, 0};
static float trajRefRad[4]   = {0, 0, 0, 0};
static float trajRefVel[4]   = {0, 0, 0, 0};

static float clampDuration(float sec) {
  if (sec < 0.05f) return 0.05f;
  if (sec > 60.0f) return 60.0f;
  return sec;
}

static void copyCurrentToTrajectoryStart() {
  trajStartRad[1] = fb[1].pos;
  trajStartRad[2] = fb[2].pos;
  trajStartRad[3] = axisM3GetContinuousRad();

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    trajRefRad[i] = trajStartRad[i];
    trajRefVel[i] = 0.0f;
  }
}

static void applyTrajectoryGoalLimits() {
  trajGoalRad[1] = axisClampGoalRad(1, trajGoalRad[1]);
  trajGoalRad[2] = axisClampGoalRad(2, trajGoalRad[2]);
  trajGoalRad[3] = axisClampGoalRad(3, trajGoalRad[3]);
}

bool motionStartAbsolute(float m1Deg, float m2Deg, float m3Mm, float durationSec) {
  copyCurrentToTrajectoryStart();

  trajGoalRad[1] = degToRad(m1Deg);
  trajGoalRad[2] = degToRad(m2Deg);
  trajGoalRad[3] = axisM3MmToRad(m3Mm);

  applyTrajectoryGoalLimits();

  trajDuration = clampDuration(durationSec);
  trajStartMs = millis();
  trajActive = true;
  trajDone = false;
  trajHasReference = true;

  Serial.printf("[TRAJ] absolute start: M1=%.2fdeg M2=%.2fdeg M3=%.2fmm T=%.2fs\n",
                radToDeg(trajGoalRad[1]),
                radToDeg(trajGoalRad[2]),
                axisM3RadToMm(trajGoalRad[3]),
                trajDuration);

  return true;
}

bool motionStartRelative(float m1DeltaDeg, float m2DeltaDeg, float m3DeltaMm, float durationSec) {
  copyCurrentToTrajectoryStart();

  trajGoalRad[1] = trajStartRad[1] + degToRad(m1DeltaDeg);
  trajGoalRad[2] = trajStartRad[2] + degToRad(m2DeltaDeg);
  trajGoalRad[3] = trajStartRad[3] + axisM3MmToRad(m3DeltaMm);

  applyTrajectoryGoalLimits();

  trajDuration = clampDuration(durationSec);
  trajStartMs = millis();
  trajActive = true;
  trajDone = false;
  trajHasReference = true;

  Serial.printf("[TRAJ] relative start: dM1=%.2fdeg dM2=%.2fdeg dM3=%.2fmm T=%.2fs\n",
                m1DeltaDeg, m2DeltaDeg, m3DeltaMm, trajDuration);

  motionTrajectoryPrint();
  return true;
}

void motionTrajectoryUpdate() {
  if (!trajHasReference) return;

  if (!trajActive) {
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
      trajRefRad[i] = trajGoalRad[i];
      trajRefVel[i] = 0.0f;
    }
    return;
  }

  float t = (millis() - trajStartMs) / 1000.0f;

  if (t >= trajDuration) {
    for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
      trajRefRad[i] = trajGoalRad[i];
      trajRefVel[i] = 0.0f;
    }

    trajActive = false;
    trajDone = true;
    Serial.println("[TRAJ] done");
    return;
  }

  float u = t / trajDuration;

  float u2 = u * u;
  float u3 = u2 * u;
  float u4 = u3 * u;
  float u5 = u4 * u;

  float s = 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
  float dsdt = (30.0f * u2 - 60.0f * u3 + 30.0f * u4) / trajDuration;

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    float delta = trajGoalRad[i] - trajStartRad[i];
    trajRefRad[i] = trajStartRad[i] + delta * s;
    trajRefVel[i] = delta * dsdt;
  }
}

void motionTrajectoryClear() {
  trajActive = false;
  trajDone = false;
  trajHasReference = false;

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    trajStartRad[i] = trajGoalRad[i] = trajRefRad[i] = 0.0f;
    trajRefVel[i] = 0.0f;
  }

  Serial.println("[TRAJ] cleared");
}

bool motionTrajectoryHasReference() {
  return trajHasReference;
}

bool motionTrajectoryIsActive() {
  return trajActive;
}

bool motionTrajectoryIsDone() {
  return trajDone;
}

float motionGetRefRad(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return trajRefRad[motorId];
}

float motionGetRefVelRadps(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return trajRefVel[motorId];
}

float motionGetGoalRad(uint8_t motorId) {
  if (motorId < 1 || motorId > MOTOR_COUNT) return 0.0f;
  return trajGoalRad[motorId];
}

void motionTrajectoryPrint() {
  Serial.printf("[TRAJ] active=%s done=%s hasRef=%s duration=%.2fs\n",
                trajActive ? "YES" : "NO",
                trajDone ? "YES" : "NO",
                trajHasReference ? "YES" : "NO",
                trajDuration);

  Serial.printf("  start: M1=%.2fdeg M2=%.2fdeg M3=%.2fmm\n",
                radToDeg(trajStartRad[1]),
                radToDeg(trajStartRad[2]),
                axisM3RadToMm(trajStartRad[3]));

  Serial.printf("  goal : M1=%.2fdeg M2=%.2fdeg M3=%.2fmm\n",
                radToDeg(trajGoalRad[1]),
                radToDeg(trajGoalRad[2]),
                axisM3RadToMm(trajGoalRad[3]));

  Serial.printf("  ref  : M1=%.2fdeg M2=%.2fdeg M3=%.2fmm\n",
                radToDeg(trajRefRad[1]),
                radToDeg(trajRefRad[2]),
                axisM3RadToMm(trajRefRad[3]));
}

// ============================================================
//  Arrival check state
// ============================================================

static uint32_t arriveStartMs = 0;
static bool holdingArrive = false;

static bool checkRotMotorReached(uint8_t motorId) {
  float currentDeg = fb[motorId].pos * RAD_TO_DEG_F;
  float targetDeg  = pdControlGetGoalDegree(motorId);

  float posErrDeg = targetDeg - currentDeg;

  bool posOk = absFloat(posErrDeg) <= ARRIVE_ROT_POS_DEG_THRESH;

  return posOk;
}

static bool checkM3Reached() {
  float currentMm = axisM3GetContinuousMm();
  float targetMm  = pdControlGetM3GoalMillimeter();

  float posErrMm = targetMm - currentMm;

  bool posOk = absFloat(posErrMm) <= ARRIVE_M3_POS_MM_THRESH;

  return posOk;
}

void motionArrivalReset() {
  arriveStartMs = 0;
  holdingArrive = false;
}

bool motionArrivalAllReached() {
  bool allOk = checkRotMotorReached(1) && checkRotMotorReached(2) && checkM3Reached();

  if (!allOk) {
    holdingArrive = false;
    arriveStartMs = 0;
    return false;
  }

  uint32_t now = millis();

  if (!holdingArrive) {
    holdingArrive = true;
    arriveStartMs = now;
    return false;
  }

  return (now - arriveStartMs >= ARRIVE_HOLD_MS);
}

void motionArrivalPrint() {
  float m1CurrentDeg = fb[1].pos * RAD_TO_DEG_F;
  float m2CurrentDeg = fb[2].pos * RAD_TO_DEG_F;

  float m1TargetDeg = pdControlGetGoalDegree(1);
  float m2TargetDeg = pdControlGetGoalDegree(2);

  float m3CurrentMm = axisM3GetContinuousMm();
  float m3TargetMm  = pdControlGetM3GoalMillimeter();

  bool reached = motionArrivalAllReached();

  Serial.println("[ARRIVE] status:");
  Serial.printf("  M1 err=%7.3fdeg vel=%7.3fdeg/s\n",
                m1TargetDeg - m1CurrentDeg,
                fb[1].vel * RAD_TO_DEG_F);

  Serial.printf("  M2 err=%7.3fdeg vel=%7.3fdeg/s\n",
                m2TargetDeg - m2CurrentDeg,
                fb[2].vel * RAD_TO_DEG_F);

  Serial.printf("  M3 err=%7.3fmm  vel=%7.3fmm/s\n",
                m3TargetMm - m3CurrentMm,
                axisM3RadpsToMmps(fb[3].vel));

  Serial.printf("  allReached=%s\n", reached ? "YES" : "NO");
}

// ============================================================
//  Motion sequence state
// ============================================================

enum SequenceState : uint8_t {
  SEQ_IDLE = 0,
  SEQ_START_STEP,
  SEQ_MOVING,
  SEQ_WAIT,
  SEQ_DONE
};

static SequenceState seqState = SEQ_IDLE;
static uint8_t seqIndex = 0;
static uint32_t waitStartMs = 0;

static uint16_t seqRepeatTarget = 1;
static uint16_t seqRepeatDone = 0;

static const char* motionStepTypeName(MotionStepType type) {
  switch (type) {
    case STEP_MOVE_ABS:      return "MOVE_ABS";
    case STEP_WAIT:          return "WAIT";
    case STEP_BRAKE_ENGAGE:  return "BRAKE_ENGAGE";
    case STEP_BRAKE_RELEASE: return "BRAKE_RELEASE";
    case STEP_SOL_LOCK:      return "SOL_LOCK";
    case STEP_SOL_UNLOCK:    return "SOL_UNLOCK";
    default:                 return "UNKNOWN";
  }
}

static const char* sequenceStateName() {
  switch (seqState) {
    case SEQ_IDLE:       return "IDLE";
    case SEQ_START_STEP: return "START_STEP";
    case SEQ_MOVING:     return "MOVING";
    case SEQ_WAIT:       return "WAIT";
    case SEQ_DONE:       return "DONE";
    default:             return "UNKNOWN";
  }
}

void motionSequenceStartDemo() {
  motionSequenceStartDemoRepeat(1);
}

void motionSequenceStartDemoRepeat(uint16_t count) {
  if (count < 1) count = 1;
  if (count > 20) count = 20;  // 안전상 최대 20 제한

  seqRepeatTarget = count;
  seqRepeatDone = 0;

  seqIndex = 0;
  waitStartMs = 0;
  seqState = SEQ_START_STEP;

  controlEnabled = true;
  motionArrivalReset();

  Serial.printf("[SEQ] demo sequence start. repeat=%u\n", (unsigned)seqRepeatTarget);
  motionSequencePrintStatus();
}

void motionSequenceStop() {
  motionTrajectoryClear();
  motionArrivalReset();

  seqState = SEQ_IDLE;
  seqIndex = 0;
  waitStartMs = 0;

  Serial.println("[SEQ] stopped");
}

bool motionSequenceIsRunning() {
  return seqState == SEQ_START_STEP || seqState == SEQ_MOVING || seqState == SEQ_WAIT;
}

void motionSequenceUpdate() {
  if (seqState == SEQ_IDLE || seqState == SEQ_DONE) {
    return;
  }

  // 전체 시퀀스 1회 끝났는지 확인
  if (seqIndex >= DEMO_SEQ_COUNT) {
    seqRepeatDone++;

    Serial.printf("[SEQ] cycle %u/%u done\n",
                  (unsigned)seqRepeatDone,
                  (unsigned)seqRepeatTarget);

    if (seqRepeatDone < seqRepeatTarget) {
      seqIndex = 0;
      waitStartMs = 0;
      motionArrivalReset();
      seqState = SEQ_START_STEP;

      Serial.printf("[SEQ] next cycle %u/%u start\n",
                    (unsigned)(seqRepeatDone + 1),
                    (unsigned)seqRepeatTarget);
      return;
    }

    motionTrajectoryClear();
    motionArrivalReset();
    seqState = SEQ_DONE;

    Serial.printf("[SEQ] done all repeats. total=%u\n",
                  (unsigned)seqRepeatTarget);
    return;
  }

  // 새 step 시작
  if (seqState == SEQ_START_STEP) {
    const MotionStep& step = DEMO_SEQ[seqIndex];

    Serial.printf("[SEQ] step %u/%u start: %s type=%s wait=%lums\n",
                  (unsigned)(seqIndex + 1),
                  (unsigned)DEMO_SEQ_COUNT,
                  step.name,
                  motionStepTypeName(step.type),
                  (unsigned long)step.waitAfterMs);

    switch (step.type) {
      case STEP_MOVE_ABS:
        Serial.printf("      target M1=%.2fdeg M2=%.2fdeg M3=%.2fmm T=%.2fs\n",
                      step.m1Deg,
                      step.m2Deg,
                      step.m3Mm,
                      step.durationSec);

        motionStartAbsolute(step.m1Deg, step.m2Deg, step.m3Mm, step.durationSec);
        motionArrivalReset();
        seqState = SEQ_MOVING;
        break;

      case STEP_BRAKE_ENGAGE:
        brakeEngage();
        waitStartMs = millis();
        seqState = SEQ_WAIT;
        break;

      case STEP_BRAKE_RELEASE:
        brakeRelease();
        waitStartMs = millis();
        seqState = SEQ_WAIT;
        break;

      case STEP_SOL_LOCK:
        solLock();
        waitStartMs = millis();
        seqState = SEQ_WAIT;
        break;

      case STEP_SOL_UNLOCK:
        solUnlock();
        waitStartMs = millis();
        seqState = SEQ_WAIT;
        break;

      case STEP_WAIT:
      default:
        waitStartMs = millis();
        seqState = SEQ_WAIT;
        break;
    }

    return;
  }

  // 이동 중
  if (seqState == SEQ_MOVING) {
    bool trajDoneNow = motionTrajectoryIsDone();
    bool arrived = motionArrivalAllReached();

    if (trajDoneNow && arrived) {
      const MotionStep& step = DEMO_SEQ[seqIndex];

      Serial.printf("[SEQ] step %u arrived: %s\n",
                    (unsigned)(seqIndex + 1),
                    step.name);

      waitStartMs = millis();
      seqState = SEQ_WAIT;
    }

    static uint32_t lastSeqDebugMs = 0;
    if (trajDoneNow && millis() - lastSeqDebugMs >= 500) {
      lastSeqDebugMs = millis();

      float m1CurrentDeg = fb[1].pos * RAD_TO_DEG_F;
      float m2CurrentDeg = fb[2].pos * RAD_TO_DEG_F;
      float m1TargetDeg = pdControlGetGoalDegree(1);
      float m2TargetDeg = pdControlGetGoalDegree(2);

      float m3CurrentMm = axisM3GetContinuousMm();
      float m3TargetMm = pdControlGetM3GoalMillimeter();

      Serial.printf("[SEQ_WAIT_ARRIVE] step=%u M1err=%.2fdeg M2err=%.2fdeg M3err=%.2fmm\n",
                    (unsigned)(seqIndex + 1),
                    m1TargetDeg - m1CurrentDeg,
                    m2TargetDeg - m2CurrentDeg,
                    m3TargetMm - m3CurrentMm);
    }

    return;
  }

  // step 후 대기
  if (seqState == SEQ_WAIT) {
    const MotionStep& step = DEMO_SEQ[seqIndex];

    if (millis() - waitStartMs >= step.waitAfterMs) {
      seqIndex++;
      seqState = SEQ_START_STEP;
    }

    return;
  }
}

void motionSequencePrintStatus() {
  uint16_t currentCycle = seqRepeatDone + 1;
  if (seqState == SEQ_DONE) currentCycle = seqRepeatDone;
  if (currentCycle > seqRepeatTarget) currentCycle = seqRepeatTarget;

  Serial.printf("[SEQ] state=%s index=%u/%u cycle=%u/%u running=%s\n",
                sequenceStateName(),
                (unsigned)seqIndex,
                (unsigned)DEMO_SEQ_COUNT,
                (unsigned)currentCycle,
                (unsigned)seqRepeatTarget,
                motionSequenceIsRunning() ? "YES" : "NO");

  if (seqIndex < DEMO_SEQ_COUNT) {
    const MotionStep& step = DEMO_SEQ[seqIndex];

    Serial.printf("  current/next step: %s type=%s M1=%.2fdeg M2=%.2fdeg M3=%.2fmm T=%.2fs wait=%lums\n",
                  step.name,
                  motionStepTypeName(step.type),
                  step.m1Deg,
                  step.m2Deg,
                  step.m3Mm,
                  step.durationSec,
                  (unsigned long)step.waitAfterMs);
  }
}



// ============================================================
//  M3 torque seek state
// ============================================================
//
// This replaces the previous separate torque_seek.h/cpp dependency.
// It uses existing trajectory + external PD command torque targetTrq[3].

enum TorqueSeekState : uint8_t {
  SEEK_IDLE = 0,
  SEEK_RUNNING,
  SEEK_HIT,
  SEEK_DONE
};

static TorqueSeekState seekState = SEEK_IDLE;

static float seekDeltaMm = 0.0f;
static float seekDurationSec = 0.0f;
static float seekThresholdNm = 0.0f;
static uint32_t seekHoldMs = 100;

static uint32_t seekOverTorqueStartMs = 0;

static const char* torqueSeekStateName() {
  switch (seekState) {
    case SEEK_IDLE:    return "IDLE";
    case SEEK_RUNNING: return "RUNNING";
    case SEEK_HIT:     return "HIT";
    case SEEK_DONE:    return "DONE";
    default:           return "UNKNOWN";
  }
}

static float absTorqueFloat(float v) {
  return v < 0.0f ? -v : v;
}

void torqueSeekStartM3(float deltaMm, float durationSec, float torqueThresholdNm, uint32_t holdMs) {
  if (durationSec <= 0.0f) {
    Serial.println("[SEEK] invalid duration");
    return;
  }

  if (torqueThresholdNm <= 0.0f) {
    Serial.println("[SEEK] invalid torque threshold");
    return;
  }

  seekDeltaMm = deltaMm;
  seekDurationSec = durationSec;
  seekThresholdNm = torqueThresholdNm;
  seekHoldMs = holdMs;
  seekOverTorqueStartMs = 0;
  seekState = SEEK_RUNNING;

  motionStartRelative(0.0f, 0.0f, seekDeltaMm, seekDurationSec);
  motionArrivalReset();

  Serial.printf("[SEEK] M3 start delta=%.3fmm duration=%.3fs threshold=%.3fNm hold=%lums\n",
                seekDeltaMm,
                seekDurationSec,
                seekThresholdNm,
                (unsigned long)seekHoldMs);
}

void torqueSeekStop() {
  motionTrajectoryClear();
  motionArrivalReset();

  seekState = SEEK_IDLE;
  seekOverTorqueStartMs = 0;

  targetTrq[3] = 0.0f;

  Serial.println("[SEEK] stopped");
}

void torqueSeekUpdate() {
  if (seekState != SEEK_RUNNING) return;

  float cmdTorqueAbs = absTorqueFloat(targetTrq[3]);
  bool over = cmdTorqueAbs >= seekThresholdNm;
  uint32_t now = millis();

  if (over) {
    if (seekOverTorqueStartMs == 0) {
      seekOverTorqueStartMs = now;
    }

    if (now - seekOverTorqueStartMs >= seekHoldMs) {
      Serial.printf("[SEEK] torque hit. cmdTrq=%.3fNm threshold=%.3fNm\n",
                    targetTrq[3],
                    seekThresholdNm);

      motionTrajectoryClear();
      motionArrivalReset();

      targetTrq[3] = 0.0f;

      seekState = SEEK_HIT;
      return;
    }
  } else {
    seekOverTorqueStartMs = 0;
  }

  if (motionTrajectoryIsDone()) {
    seekState = SEEK_DONE;
    seekOverTorqueStartMs = 0;

    Serial.println("[SEEK] finished without torque hit");
  }
}

void torqueSeekPrintStatus() {
  Serial.printf("[SEEK] state=%s delta=%.3fmm duration=%.3fs threshold=%.3fNm hold=%lums\n",
                torqueSeekStateName(),
                seekDeltaMm,
                seekDurationSec,
                seekThresholdNm,
                (unsigned long)seekHoldMs);

  Serial.printf("  M3 pos=%.3fmm target=%.3fmm cmdTrq=%.3fNm fbTrq=%.3fNm\n",
                axisM3GetContinuousMm(),
                pdControlGetM3GoalMillimeter(),
                targetTrq[3],
                fb[3].trq);
}

bool torqueSeekIsActive() {
  return seekState == SEEK_RUNNING;
}

bool torqueSeekIsHit() {
  return seekState == SEEK_HIT;
}

// ============================================================
//  Backward-compatible old trajectory names
// ============================================================

bool trajectoryStartAbsoluteDegMm(float m1Deg, float m2Deg, float m3Mm, float durationSec) {
  return motionStartAbsolute(m1Deg, m2Deg, m3Mm, durationSec);
}

bool trajectoryStartRelativeDegMm(float m1DeltaDeg, float m2DeltaDeg, float m3DeltaMm, float durationSec) {
  return motionStartRelative(m1DeltaDeg, m2DeltaDeg, m3DeltaMm, durationSec);
}

void trajectoryUpdate() {
  motionTrajectoryUpdate();
}

void trajectoryClear() {
  motionTrajectoryClear();
}

bool trajectoryHasReference() {
  return motionTrajectoryHasReference();
}

bool trajectoryIsActive() {
  return motionTrajectoryIsActive();
}

bool trajectoryIsDone() {
  return motionTrajectoryIsDone();
}

float trajectoryGetRefRad(uint8_t motorId) {
  return motionGetRefRad(motorId);
}

float trajectoryGetRefVelRadps(uint8_t motorId) {
  return motionGetRefVelRadps(motorId);
}

float trajectoryGetGoalRad(uint8_t motorId) {
  return motionGetGoalRad(motorId);
}

void trajectoryPrint() {
  motionTrajectoryPrint();
}

// ============================================================
//  Backward-compatible old arrival names
// ============================================================

void arrivalReset() {
  motionArrivalReset();
}

bool arrivalAllReached() {
  return motionArrivalAllReached();
}

void arrivalPrint() {
  motionArrivalPrint();
}

// ============================================================
//  Backward-compatible old sequence names
// ============================================================

void sequenceStartDemo() {
  motionSequenceStartDemo();
}

void sequenceStop() {
  motionSequenceStop();
}

void sequenceUpdate() {
  motionSequenceUpdate();
}

void sequencePrintStatus() {
  motionSequencePrintStatus();
}

bool sequenceIsRunning() {
  return motionSequenceIsRunning();
}

void sequenceStartDemoRepeat(uint16_t count) {
  motionSequenceStartDemoRepeat(count);
}
