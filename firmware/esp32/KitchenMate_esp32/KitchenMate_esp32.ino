/*
 * ESP32 Robot Arm PD MM Sequence
 *
 * Role split:
 *   - robot_arm_pd_mm_hybrid_pd.ino : setup/loop and global runtime state
 *   - command.cpp               : serial command parsing
 *   - status.cpp                : status printing
 *   - safety.cpp                : feedback check, hold, send command, safe stop
 *   - pd_control.cpp            : external PD torque control
 *   - m3_axis.cpp               : M3 mm conversion + unwrap
 *   - motor.cpp                 : TWAI/CAN + MIT packet + feedback
 *   - brake.cpp                 : brake/solenoid
 */

#include "config.h"
#include "runtime.h"
#include "brake.h"
#include "motor.h"
#include "pd_control.h"
#include "command.h"
#include "status.h"
#include "safety.h"
#include "motion.h"

// ============================================================
//  Globals required by config.h / modules
// ============================================================

MotorFeedback fb[4] = {};
MotorCommand  lastCmd[4] = {};

BrakePhase brakePhase = BRAKE_LOCKED;
SolPhase   solPhase   = SOL_LOCKED;

uint32_t solPulseStart = 0;
uint32_t solPulseDurationMs = 0;

bool estop   = false;
bool rawDebug = false;

uint32_t brakeFullStart = 0;

// ============================================================
//  Runtime globals required by runtime.h
// ============================================================

bool controlEnabled = false;

uint32_t ctrlTimer = 0;
uint32_t printTimer = 0;

float targetPos[4] = {0, 0, 0, 0};
float targetVel[4] = {0, 0, 0, 0};
float targetTrq[4] = {0, 0, 0, 0};

// ============================================================
//  User control hook
// ============================================================

void userControl() {
  pdControlUpdate(targetPos, targetVel, targetTrq);
}

// ============================================================
//  Arduino setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  delay(500);

  Serial.println("\n[BOOT] ESP32 Robot Arm PD MM Sequence");
  commandPrintHelp();

  pinMode(PIN_BRAKE1, OUTPUT);
  pinMode(PIN_BRAKE2, OUTPUT);
  pinMode(PIN_BRAKE_SOL, OUTPUT);

  setupBrake();

  if (!initTWAI()) {
    Serial.println("[FATAL] CAN init failed");
    while (true) delay(1000);
  }

  enableAll();
  delay(300);

  if (!waitInitialFeedback(1500)) {
    Serial.println("[FATAL] No initial feedback. Motors disabled.");
    safeStop();
    while (true) delay(1000);
  }

  captureCurrentPositionAsTarget();
  controlEnabled = false;
  ctrlTimer = millis();

  Serial.println("[READY] Holding current position. Use seq for demo sequence, mv/mvr for trajectory, ipd for M1/M2, pd for M3.");
}

void loop() {
  uint32_t now = millis();

  if (now - ctrlTimer < CTRL_PERIOD_MS) return;
  ctrlTimer = now;

  //updateBrake();
  updateSol();
  recvFeedback();
  commandPoll();

  if (estop) {
    safeStop();
    while (true) delay(1000);
  }

  sequenceUpdate();

  if (controlEnabled) {
    userControl();
    torqueSeekUpdate();
    sendTargetCommand();
  }

  statusPrint();

}
