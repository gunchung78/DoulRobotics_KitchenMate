/*
 * ============================================================
 * RS02 모터 통합 제어 도구 - ID 지정 명령 + 피드백 출력 버전
 * Arduino IDE 2.3.x / ESP32
 *
 * 파일 구조:
 *   config.h      - 핀 설정, MIT 물리량 범위, 브레이크 상수
 *   motor.h/cpp   - 피드백 구조체, 전역 변수, ID 검증, 값 변환
 *   can_comm.h/cpp - TWAI 초기화, 프레임 송수신, 피드백 디코딩
 *   commands.h/cpp - STEP 1~7 명령, Enable/Stop/제어 함수
 *   brake.h/cpp   - 브레이크/솔레노이드 제어
 *   ID_Control.ino - setup, loop, 메뉴 출력, 명령 파싱 (이 파일)
 *
 * 핵심 명령:
 *   s                         : 29bit 모터 ID 스캔
 *   c 현재ID 새ID              : 29bit ID 변경 + EEPROM 저장
 *   m ID                      : 29bit MIT 프로토콜 전환
 *   ms                        : 11bit MIT 모드 ID 스캔
 *   ci 현재ID 새ID             : 11bit MIT 모드 ID 변경
 *   e ID                      : 해당 ID 모터 Enable
 *   t ID pos vel kp kd tff    : 해당 ID 모터 MIT 제어
 *   f                         : 마지막 Enable/제어 모터 상태값 출력
 *   f ID                      : 해당 ID 모터 상태값 요청 후 출력
 *   x ID                      : 해당 ID 모터 Stop
 *   z ID                      : 현재 위치를 Zero Position으로 설정
 *   be                        : 브레이크 잠금 (Engage)
 *   br                        : 브레이크 해제 (Release)
 *   sl                        : 솔레노이드 잠금
 *   su                        : 솔레노이드 해제
 *   bs                        : 브레이크/솔 상태 출력
 *
 * 사용 예:
 *   e 3
 *   f
 *   t 3 0 0 10 1 0
 *   f 3
 *   x 3
 *
 * 하드웨어:
 *   ESP32 TWAI CAN 1Mbps
 *   TX: GPIO21 / RX: GPIO22
 *   CAN Transceiver: SN65HVD230
 *   CANH-CANL 종단저항 120Ω 권장
 * ============================================================
 */

#include "commands.h"
#include "brake.h"

// ============================================================
// 전체 Enable 상태 출력
// ============================================================
static void printEnabledList() {
    bool any = false;
    Serial.print("║  Enabled IDs: ");

    for (int i = 0; i < MOTOR_ID_MAX; i++) {
        if (g_motorEnabled[i]) {
            if (any) Serial.print(", ");
            Serial.printf("0x%02X", i);
            any = true;
        }
    }

    if (!any) Serial.print("none");
    Serial.println();
}

// ============================================================
// 메뉴 출력
// ============================================================
static void printMenu() {
    updateMitReadyFlag();

    Serial.println("\n╔══════════════════════════════════════════════════════╗");
    Serial.printf ("║  lastID=0x%02X  MIT=%-3s  AnyEnable=%-3s              ║\n",
        g_motorID,
        g_mitMode  ? "ON" : "OFF",
        g_mitReady ? "ON" : "OFF");
    printEnabledList();
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║  s                       : STEP1   [29bit] 모터 스캔 ║");
    Serial.println("║  c 현재ID 새ID            : STEP2   [29bit] ID 변경   ║");
    Serial.println("║                            예) c 0 3                 ║");
    Serial.println("║  m ID                    : STEP3   [29bit] MIT 전환  ║");
    Serial.println("║                            예) m 3                   ║");
    Serial.println("║  ── ⚡ 전원 OFF→ON ─────────────────────────────     ║");
    Serial.println("║  ms                      : STEP5   [11bit] 스캔      ║");
    Serial.println("║  ci 현재ID 새ID           : STEP5.5 [11bit] ID 변경   ║");
    Serial.println("║                            예) ci 3 5                ║");
    Serial.println("║  e ID                    : STEP6   [11bit] Enable    ║");
    Serial.println("║                            예) e 3                   ║");
    Serial.println("║  t ID pos vel kp kd tff  : STEP7   [11bit] MIT 제어  ║");
    Serial.println("║                            예) t 3 0 0 10 1 0        ║");
    Serial.println("║                            예) t 3 0 2 0 1 0         ║");
    Serial.println("║  f                       : lastID 상태값 출력        ║");
    Serial.println("║                            예) e 3 입력 후 f         ║");
    Serial.println("║  f ID                    : 해당 ID 상태값 출력       ║");
    Serial.println("║                            예) f 3                   ║");
    Serial.println("║  x ID                    : MIT Stop                  ║");
    Serial.println("║                            예) x 3                   ║");
    Serial.println("║  z ID                    : Zero Position 설정        ║");
    Serial.println("║                            예) z 3                   ║");
    Serial.println("╠──────────────── 브레이크 / 솔레노이드 ────────────────╣");
    Serial.println("║  be                      : 브레이크 잠금 (Engage)     ║");
    Serial.println("║  br                      : 브레이크 해제 (Release)    ║");
    Serial.println("║  sl                      : 솔레노이드 잠금            ║");
    Serial.println("║  su                      : 솔레노이드 해제            ║");
    Serial.println("║  bs                      : 브레이크/솔 상태 출력      ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.print("> ");
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== RS02 통합 제어 도구 - ID 지정 명령 + 피드백 출력 버전 ===");

    if (!initTWAI()) {
        Serial.println("[FATAL] 초기화 실패!");
        while (1) delay(1000);
    }

    setupBrake();
    Serial.println("[OK] 브레이크 초기화 완료");

    printMenu();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    // 브레이크 FULL→HOLD 자동 전환 처리
    updateBrake();

    // 백그라운드 수신 처리
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        printRxAndTryDecode(msg, "수신");
    }

    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (!cmd.length()) return;

    Serial.println(cmd);

    if (cmd == "s") {
        step1_scan29bit();

    } else if (cmd.startsWith("ci")) {
        // 중요: 'c'보다 먼저 체크해야 함
        int curID = -1;
        int newID = -1;
        int n = sscanf(cmd.c_str(), "ci %i %i", &curID, &newID);

        if (n == 2 && isValidMotorID(curID) && isValidMotorID(newID)) {
            step55_mitChangeID((uint8_t)curID, (uint8_t)newID);
        } else {
            Serial.println("사용법: ci 현재ID 새ID  예) ci 3 5 또는 ci 0x03 0x05");
        }

    } else if (cmd.startsWith("c")) {
        int curID = -1;
        int newID = -1;
        int n = sscanf(cmd.c_str(), "c %i %i", &curID, &newID);

        if (n == 2 && isValidDeviceID(curID) && isValidMotorID(newID)) {
            step2_changeID((uint8_t)curID, (uint8_t)newID);
        } else {
            Serial.println("사용법: c 현재ID 새ID  예) c 0 3 또는 c 0x00 0x03");
        }

    } else if (cmd == "ms") {
        // 중요: 'm'보다 먼저 체크해야 함
        step5_scan11bit();

    } else if (cmd.startsWith("m")) {
        int id = -1;
        int n = sscanf(cmd.c_str(), "m %i", &id);

        if (n == 1 && isValidMotorID(id)) {
            step3_switchMIT((uint8_t)id);
        } else {
            Serial.println("사용법: m ID  예) m 3 또는 m 0x03");
        }

    } else if (cmd.startsWith("e")) {
        int id = -1;
        int n = sscanf(cmd.c_str(), "e %i", &id);

        if (n == 1 && isValidMotorID(id)) {
            step6_mitEnable((uint8_t)id);
        } else {
            Serial.println("사용법: e ID  예) e 3 또는 e 0x03");
        }

    } else if (cmd.startsWith("t")) {
        int id = -1;
        float pos = 0.0f;
        float vel = 0.0f;
        float kp  = 10.0f;
        float kd  = 1.0f;
        float tff = 0.0f;

        int n = sscanf(cmd.c_str(), "t %i %f %f %f %f %f",
                       &id, &pos, &vel, &kp, &kd, &tff);

        if (n == 6 && isValidMotorID(id)) {
            step7_mitControl((uint8_t)id, pos, vel, kp, kd, tff);
        } else {
            Serial.println("사용법: t ID pos vel kp kd tff");
            Serial.println("예) t 3 0 0 10 1 0");
            Serial.println("예) t 3 0 2 0 1 0");
        }

    } else if (cmd == "f") {
        // 마지막 Enable/제어/조회한 모터 상태값 출력
        requestAndPrintFeedback(g_motorID);

    } else if (cmd.startsWith("f")) {
        int id = -1;
        int n = sscanf(cmd.c_str(), "f %i", &id);

        if (n == 1 && isValidMotorID(id)) {
            requestAndPrintFeedback((uint8_t)id);
        } else {
            Serial.println("사용법: f 또는 f ID");
            Serial.println("예) e 3 입력 후 f");
            Serial.println("예) f 3 또는 f 0x03");
        }

    } else if (cmd.startsWith("x")) {
        int id = -1;
        int n = sscanf(cmd.c_str(), "x %i", &id);

        if (n == 1 && isValidMotorID(id)) {
            mitStop((uint8_t)id);
        } else {
            Serial.println("사용법: x ID  예) x 3 또는 x 0x03");
        }

    } else if (cmd.startsWith("z")) {
        int id = -1;
        int n = sscanf(cmd.c_str(), "z %i", &id);

        if (n == 1 && isValidMotorID(id)) {
            setZeroPosition((uint8_t)id);
        } else {
            Serial.println("사용법: z ID  예) z 3 또는 z 0x03");
        }

    } else if (cmd == "be") {
        brakeEngage();
        Serial.println("[OK] 브레이크 잠금 (Engage)");

    } else if (cmd == "br") {
        brakeRelease();
        Serial.println("[OK] 브레이크 해제 (Release) → FULL→HOLD 자동 전환");

    } else if (cmd == "sl") {
        solLock();
        Serial.println("[OK] 솔레노이드 잠금 (PWM 0%)");

    } else if (cmd == "su") {
        solUnlock();
        Serial.println("[OK] 솔레노이드 해제 (PWM 100%)");

    } else if (cmd == "bs") {
        const char* bStr = (brakePhase == BRAKE_LOCKED) ? "LOCKED" :
                           (brakePhase == BRAKE_FULL)   ? "FULL"   : "HOLD";
        const char* sStr = (solPhase == SOL_LOCKED) ? "LOCKED" : "UNLOCKED";
        Serial.printf("[BRAKE] phase=%s  [SOL] phase=%s\n", bStr, sStr);

    } else {
        Serial.println("[?] 명령어를 확인하세요");
    }

    //printMenu();
}
