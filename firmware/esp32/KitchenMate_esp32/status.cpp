#include "status.h"
#include "config.h"
#include "runtime.h"
#include "pd_control.h"
#include "axis.h"

static float radToDegLocal(float rad) {
  return rad * 57.29577951308232f;
}

static const char* modeShort(uint8_t motorId) {
  return pdControlIsInternal(motorId) ? "IPD" : "EPD";
}

void statusPrint() {
  uint32_t now = millis();

  if (now - printTimer < STATUS_PERIOD_MS) return;
  printTimer = now;

  Serial.printf("[STATE] control=%s\n", controlEnabled ? "ON" : "OFF");

  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    if (i == 3) {
      Serial.printf("  M3[%s] pos=%9.3fmm target=%9.3fmm vel=%9.3fmm/s trq=%7.3f temp=%5.1fC err=0x%02X\n",
                    modeShort(3),
                    fb[3].pos,
                    m3AxisGetContinuousMillimeter(),
                    pdControlGetM3GoalMillimeter(),
                    m3AxisRadpsToMmps(fb[3].vel),
                    fb[3].trq,
                    targetTrq[3],
                    fb[3].temp,
                    fb[3].err);
    } else {
      Serial.printf("  M%d[%s] pos=%7.2fdeg target=%7.2fdeg vel=%7.3frad/s trq=%7.3f temp=%5.1fC err=0x%02X\n",
                    i,
                    modeShort(i),
                    radToDegLocal(fb[i].pos),
                    radToDegLocal(targetPos[i]),
                    fb[i].vel,
                    fb[i].trq,
                    fb[i].temp,
                    fb[i].err);
    }
  }
}

  // for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
  //   if (i == 3) {
  //     Serial.printf("  M3[%s] raw=%7.3frad pos=%9.3fmm target=%9.3fmm vel=%9.3fmm/s trq=%7.3f cmdTrq=%7.3f temp=%5.1fC err=0x%02X\n",
  //                   modeShort(3),
  //                   fb[3].pos,
  //                   m3AxisGetContinuousMillimeter(),
  //                   pdControlGetM3GoalMillimeter(),
  //                   m3AxisRadpsToMmps(fb[3].vel),
  //                   fb[3].trq,
  //                   targetTrq[3],
  //                   fb[3].temp,
  //                   fb[3].err);
  //   } else {
  //     Serial.printf("  M%d[%s] pos=%7.2fdeg target=%7.2fdeg vel=%7.3frad/s trq=%7.3f temp=%5.1fC err=0x%02X\n",
  //                   i,
  //                   modeShort(i),
  //                   radToDegLocal(fb[i].pos),
  //                   radToDegLocal(targetPos[i]),
  //                   fb[i].vel,
  //                   fb[i].trq,
  //                   fb[i].temp,
  //                   fb[i].err);
  //   }
  // }
