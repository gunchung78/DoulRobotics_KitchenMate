#include "command.h"
#include "config.h"
#include "runtime.h"
#include "motor.h"
#include "pd_control.h"
#include "safety.h"
#include "axis.h"
#include "motion.h"
#include "brake.h"

void commandPrintHelp() {
  Serial.println("[CMD] use:");
  Serial.println("  s                          : stop");
  Serial.println("  e                          : enable control");
  Serial.println("  h                          : hold current position and reset PD goal");
  Serial.println("  r                          : reset PD goal from current position");
  Serial.println("  uw                         : reset M3 unwrap reference");
  Serial.println("  pp                         : print PD params and goal");
  Serial.println("  lim                        : print soft limits");
  Serial.println("  ar                         : print arrival status");
  Serial.println("  mt                         : print trajectory status");
  Serial.println("  mx                         : cancel trajectory and hold current position");
  Serial.println("  be                         : brake engage");
  Serial.println("  br                         : brake release, full then hold");
  Serial.println("  sl                         : solenoid lock");
  Serial.println("  su                         : solenoid unlock");
  Serial.println("  seq                          : start demo sequence");
  Serial.println("  seqstop                          : stop sequence and hold current position");
  Serial.println("  seqstat                    : print sequence status");
  Serial.println("  mv  10 -5 75 3             : absolute quintic move, M1=10deg M2=-5deg M3=75mm, 3s");
  Serial.println("  mvr 5 0 -20 2              : relative quintic move, dM1=5deg dM2=0deg dM3=-20mm, 2s");
  Serial.println("  d                          : toggle raw CAN debug");
  Serial.println("  pd  3 -75 20 0.5          : EXTERNAL PD, M3 -75mm");
  Serial.println("  pd  1 5 1.0 0.1           : EXTERNAL PD, M1 +5deg");
  Serial.println("  ipd 1 5 30 1.5            : INTERNAL MIT PD, M1 +5deg");
  Serial.println("  ipd 2 -3 10 1.5           : INTERNAL MIT PD, M2 -3deg");
}

void commandPoll() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) return;

  if (line == "s") {
    estop = true;
    return;
  }

if (line == "e") {
    enableAll();

    captureCurrentPositionAsTarget();
    pdControlReset();
    arrivalReset();

    controlEnabled = true;

    Serial.println("[CMD] motors enabled, controlEnabled = true");
    return;
}

  if (line == "h") {
    captureCurrentPositionAsTarget();
    pdControlReset();
    Serial.println("[CMD] hold current position and reset PD");
    return;
  }

  if (line == "r") {
    pdControlReset();
    Serial.println("[CMD] reset PD goal from current position");
    return;
  }

  if (line == "uw") {
    pdControlM3ResetUnwrap();
    pdControlReset();
    Serial.println("[CMD] reset M3 unwrap and PD goal");
    return;
  }

  if (line == "pp") {
    pdControlPrintParams();
    pdControlPrintGoal();
    return;
  }

  if (line == "lim") {
    axisLimitPrint();
    return;
  }

  if (line == "ar") {
    arrivalPrint();
    return;
  }

  if (line == "mt") {
    trajectoryPrint();
    return;
  }

  if (line == "mx") {
    trajectoryClear();
    captureCurrentPositionAsTarget();
    pdControlReset();
    arrivalReset();
    Serial.println("[TRAJ] cancel and hold current position");
    return;
  }

  if (line == "be") {
    brakeEngage();
    Serial.println("[BRAKE] engage");
    return;
  }

  if (line == "br") {
    brakeRelease();
    Serial.println("[BRAKE] release, full then hold");
    return;
  }

  if (line == "sl") {
    solLock();
    Serial.println("[SOL] lock");
    return;
  }

  if (line == "su") {
    solUnlock();
    Serial.println("[SOL] unlock");
    return;
  }

  int seqCount = 0;
  int matchedSeq = sscanf(line.c_str(), "seq %d", &seqCount);

  if (line == "seq" || matchedSeq == 1) {
    if (line == "seq") {
      seqCount = 1;
    }

    if (seqCount < 1) seqCount = 1;
    if (seqCount > 100) seqCount = 100;

    enableAll();

    captureCurrentPositionAsTarget();
    pdControlReset();
    arrivalReset();

    controlEnabled = true;

    sequenceStartDemoRepeat((uint16_t)seqCount);

    Serial.printf("[SEQ] start demo sequence x%d. Motors enabled, control ON.\n", seqCount);
    return;
  }

  if (line == "seqstop") {
    //seqstop
    sequenceStop();
    captureCurrentPositionAsTarget();
    pdControlReset();
    arrivalReset();
    Serial.println("[SEQ] stopped and current position hold");
    return;
  }

  if (line == "seqstat") {
    sequencePrintStatus();
    return;
  }

  int zeroId = 0;
  int matchedZero = sscanf(line.c_str(), "z %d", &zeroId);
  if (matchedZero == 1) {
    if (zeroId < 1 || zeroId > MOTOR_COUNT) {
      Serial.println("[ZERO] invalid id. Use: z 1, z 2, z 3");
      return;
    }

    controlEnabled = false;

    trajectoryClear();
    setZeroPosition((uint8_t)zeroId);

    delay(100);
    recvFeedback();

    if (zeroId == 3) {
      pdControlM3ResetUnwrap();
    }

    pdControlClearOffsets();
    captureCurrentPositionAsTarget();
    pdControlReset();
    arrivalReset();
    disableAll();
    controlEnabled = false;

    Serial.printf("[ZERO] M%d zero complete. Current position hold.\n", zeroId);
    return;
  }

  if (line == "d") {
    rawDebug = !rawDebug;
    Serial.printf("[CMD] rawDebug = %s\n", rawDebug ? "ON" : "OFF");
    return;
  }

  if (line == "a") {
  Serial.printf("[ANGLE] M1 current = %.3f deg, target = %.3f deg\n",
                fb[1].pos * 57.2957795f,
                targetPos[1] * 57.2957795f);

  Serial.printf("[ANGLE] M2 current = %.3f deg, target = %.3f deg\n",
                fb[2].pos * 57.2957795f,
                targetPos[2] * 57.2957795f);

  Serial.printf("[M3] current = %.3f mm, target = %.3f mm\n",
                m3AxisGetContinuousMillimeter(),
                pdControlGetM3GoalMillimeter());
  return;
  }

  float seekDelta = 0.0f;
  float seekTime = 0.0f;
  float seekTorque = 0.0f;
  int seekHold = 100;

  int matchedSeek = sscanf(line.c_str(), "seek3 %f %f %f %d",
                          &seekDelta,
                          &seekTime,
                          &seekTorque,
                          &seekHold);

  if (matchedSeek >= 3) {
    torqueSeekStartM3(
      seekDelta,
      seekTime,
      seekTorque,
      (uint32_t)seekHold
    );
    return;
  }

  if (line == "seekstop") {
    torqueSeekStop();
    return;
  }

  if (line == "seekstat") {
    torqueSeekPrintStatus();
    return;
  }

  float m1 = 0.0f;
  float m2 = 0.0f;
  float m3 = 0.0f;
  float sec = 0.0f;

  int matchedMoveRel = sscanf(line.c_str(), "mvr %f %f %f %f", &m1, &m2, &m3, &sec);
  if (matchedMoveRel == 4) {
    trajectoryStartRelativeDegMm(m1, m2, m3, sec);
    arrivalReset();
    return;
  }

  int matchedMoveAbs = sscanf(line.c_str(), "mv %f %f %f %f", &m1, &m2, &m3, &sec);
  if (matchedMoveAbs == 4) {
    trajectoryStartAbsoluteDegMm(m1, m2, m3, sec);
    arrivalReset();
    return;
  }

  int idValue = 0;
  float offset = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;

  // Internal MIT PD command.
  // M1/M2 offset unit: deg. M3 internal PD is intentionally blocked.
  int matchedInternal = sscanf(line.c_str(), "ipd %d %f %f %f",
                               &idValue, &offset, &kp, &kd);
  if (matchedInternal == 4) {
    pdControlSetInternalMotor((uint8_t)idValue, offset, kp, kd);
    pdControlPrintParams();
    return;
  }

  // External PD command.
  // M1/M2 offset unit: deg. M3 offset unit: mm.
  int matchedExternal = sscanf(line.c_str(), "pd %d %f %f %f",
                               &idValue, &offset, &kp, &kd);
  if (matchedExternal == 4) {
    pdControlSetExternalMotor((uint8_t)idValue, offset, kp, kd);
    pdControlPrintParams();
    return;
  }

  Serial.print("[CMD] unknown: ");
  Serial.println(line);
  commandPrintHelp();
}
