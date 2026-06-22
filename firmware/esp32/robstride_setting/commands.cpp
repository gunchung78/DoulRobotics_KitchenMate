/*
 * commands.cpp
 * ─────────────────────────────────────────────
 * 모터 제어 명령 구현
 *   STEP 1~7, MIT Stop, 피드백 요청/출력
 * ─────────────────────────────────────────────
 */

#include "commands.h"

// ============================================================
// STEP 1: [29bit] 모터 ID 스캔 → 's'
// ============================================================
void step1_scan29bit() {
    Serial.println("\n╔══ STEP1: 모터 ID 스캔 [29bit] ══╗");
    bool found = false;

    for (uint8_t id = 0x00; id <= 0x7F; id++) {
        uint32_t extID = ((uint32_t)0x00 << 24)
                       | ((uint32_t)0x00 << 16)
                       | ((uint32_t)0x00 <<  8)
                       | (uint32_t)id;

        uint8_t zeros[8] = {0};
        sendExt(extID, zeros, 8);

        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(150)) == ESP_OK) {
            uint8_t rxMotorID = (rx.identifier >> 8) & 0xFF;
            if (rxMotorID == 0x00) rxMotorID = id;

            Serial.printf(
                "  ★ 응답! motorID=0x%02X  RX_ID=0x%08lX [%s]  Data:",
                rxMotorID, rx.identifier,
                rx.extd ? "29bit" : "11bit");

            for (int i = 0; i < rx.data_length_code; i++) {
                Serial.printf(" %02X", rx.data[i]);
            }
            Serial.println();

            g_motorID = rxMotorID;
            found = true;
            flushRX(50);
            break;
        }

        delay(5);
    }

    if (!found) Serial.println("  응답 없음 → 배선/전원 확인!");
    Serial.printf("╚══ 완료. last motorID=0x%02X ══╝\n", g_motorID);
}

// ============================================================
// STEP 2: [29bit] 모터 ID 변경 + EEPROM 저장 → 'c 0 3'
// ============================================================
void step2_changeID(uint8_t curID, uint8_t newID) {
    Serial.printf("\n╔══ STEP2: ID 변경 0x%02X → 0x%02X [29bit] ══╗\n",
                  curID, newID);

    if (!isValidDeviceID(curID)) {
        Serial.println("  [ERROR] 현재 ID 범위: 0x00~0x7F");
        return;
    }

    if (!isValidMotorID(newID)) {
        Serial.println("  [ERROR] 새 ID 범위: 0x01~0x7F");
        return;
    }

    uint8_t zeros[8] = {0};

    Serial.println("  [2-1] ID 변경 (comm_type 0x07)...");
    uint32_t chgID = ((uint32_t)0x07  << 24)
                   | ((uint32_t)newID << 16)
                   | ((uint32_t)0x00  <<  8)
                   | (uint32_t)curID;
    sendExt(chgID, zeros, 8);
    monitorRX(300);

    Serial.println("  [2-2] EEPROM 저장 (comm_type 0x16)...");
    uint32_t saveID = ((uint32_t)0x16 << 24)
                    | ((uint32_t)0x00 << 16)
                    | ((uint32_t)0x00 <<  8)
                    | (uint32_t)newID;
    uint8_t saveData[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    sendExt(saveID, saveData, 8);
    monitorRX(500);

    g_motorID = newID;
    Serial.printf("╚══ 완료. last motorID=0x%02X ══╝\n", g_motorID);
}

// ============================================================
// STEP 3: [29bit] MIT 프로토콜 전환 → 'm ID'
// ============================================================
void step3_switchMIT(uint8_t motorID) {
    Serial.printf("\n╔══ STEP3: MIT 프로토콜 전환 [29bit] (ID=0x%02X) ══╗\n",
                  motorID);

    if (!isValidMotorID(motorID)) {
        Serial.println("  [ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    Serial.println("  comm_type 0x19 - 자체저장, 재부팅 후 적용");

    uint32_t protoID = ((uint32_t)0x19 << 24)
                     | ((uint32_t)0x00 << 16)
                     | ((uint32_t)0x00 <<  8)
                     | (uint32_t)motorID;

    uint8_t protoData[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x02,0x00};
    sendExt(protoID, protoData, 8);
    monitorRX(500);

    g_motorID = motorID;
    g_mitMode = true;
    g_motorEnabled[motorID] = false;
    updateMitReadyFlag();

    Serial.println("╚══════════════════════════════════════════════╝");
    Serial.println("  ★ STEP4: 지금 모터 전원 OFF → ON 하세요!");
    Serial.println("  전원 재인가 후 'ms' 로 STEP5 실행");
}

// ============================================================
// STEP 5: [11bit] 모터 ID 스캔 (MIT 모드) → 'ms'
// 주의: 이 스캔은 Enable 프레임을 보내며 응답을 확인함.
// ============================================================
void step5_scan11bit() {
    Serial.println("\n╔══ STEP5: 모터 ID 스캔 [11bit MIT] ══╗");
    bool found = false;

    for (uint8_t id = 0x01; id <= 0x7F; id++) {
        uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
        sendStd((uint32_t)id, data, 8);

        twai_message_t rx;
        if (twai_receive(&rx, pdMS_TO_TICKS(80)) == ESP_OK) {
            Serial.printf(
                "  ★ MIT 응답! motorID=0x%02X  RX_ID=0x%08lX [%s]  Data:",
                id, rx.identifier,
                rx.extd ? "29bit" : "11bit");

            for (int i = 0; i < rx.data_length_code; i++) {
                Serial.printf(" %02X", rx.data[i]);
            }
            Serial.println();

            decodeMITFeedback(rx);

            g_motorID = id;
            g_motorEnabled[id] = true;
            updateMitReadyFlag();

            found = true;
            flushRX(50);
            break;
        }

        delay(5);
    }

    if (!found) Serial.println("  응답 없음 → MIT 전환/전원 재인가/배선 확인!");
    Serial.printf("╚══ 완료. last motorID=0x%02X ══╝\n", g_motorID);
}

// ============================================================
// STEP 5.5: [11bit] MIT 모드에서 ID 변경 → 'ci 현재ID 새ID'
// ============================================================
void step55_mitChangeID(uint8_t curID, uint8_t newID) {
    Serial.printf("\n╔══ STEP5.5: MIT ID 변경 0x%02X → 0x%02X [11bit] ══╗\n",
                  curID, newID);

    if (!isValidMotorID(curID)) {
        Serial.println("  [ERROR] 현재 ID 범위: 0x01~0x7F");
        return;
    }

    if (!isValidMotorID(newID)) {
        Serial.println("  [ERROR] 새 ID 범위: 0x01~0x7F");
        return;
    }

    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       (uint8_t)newID, 0xFA};
    sendStd((uint32_t)curID, data, 8);
    monitorRX(500);

    g_motorEnabled[curID] = false;
    g_motorEnabled[newID] = false;
    g_motorID = newID;
    updateMitReadyFlag();

    Serial.printf("╚══ 완료. last motorID=0x%02X ══╝\n", g_motorID);
    Serial.println("  권장: ID 변경 후 'e 새ID'를 다시 실행하세요.");
}

// ============================================================
// STEP 6: [11bit] MIT Enable → 'e ID'
// ============================================================
void step6_mitEnable(uint8_t motorID) {
    Serial.printf("\n╔══ STEP6: MIT Enable [11bit] (ID=0x%02X) ══╗\n",
                  motorID);

    if (!isValidMotorID(motorID)) {
        Serial.println("  [ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    sendStd((uint32_t)motorID, data, 8);

    bool got = waitFeedbackForID(motorID, 500);

    g_motorID = motorID;
    g_motorEnabled[motorID] = true;
    updateMitReadyFlag();

    Serial.printf("╚══ ID 0x%02X Enable 완료 ══╝\n", motorID);

    if (got) {
        printFeedback(motorID);
    } else {
        Serial.println("  [WARN] Enable 응답 피드백을 받지 못했습니다.");
        Serial.println("         배선/전원/CAN ID/프로토콜 상태를 확인하세요.");
    }

    Serial.println("  다음 예시: t ID pos vel kp kd tff");
    Serial.printf ("  예시    : t %d 0 0 10 1 0\n", motorID);
}

// ============================================================
// STEP 7: [11bit] MIT 제어 → 't ID pos vel kp kd tff'
// ============================================================
void step7_mitControl(uint8_t motorID,
                      float pos, float vel,
                      float kp,  float kd, float torque) {
    if (!isValidMotorID(motorID)) {
        Serial.println("[ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    if (!g_motorEnabled[motorID]) {
        Serial.printf("[ERROR] ID 0x%02X 모터가 Enable 상태가 아닙니다. 먼저 'e %d' 실행하세요!\n",
                      motorID, motorID);
        return;
    }

    Serial.printf(
        "\n[MIT CMD] ID=0x%02X pos=%.3f vel=%.3f kp=%.1f kd=%.2f tff=%.3f\n",
        motorID, pos, vel, kp, kd, torque);

    uint16_t p = f2u(pos,    P_MIN, P_MAX, 16);
    uint16_t v = f2u(vel,    V_MIN, V_MAX, 12);
    uint16_t K = f2u(kp,    KP_MIN,KP_MAX, 12);
    uint16_t D = f2u(kd,    KD_MIN,KD_MAX, 12);
    uint16_t t = f2u(torque, T_MIN, T_MAX, 12);

    uint8_t data[8];
    data[0] =  p >> 8;
    data[1] =  p & 0xFF;
    data[2] =  v >> 4;
    data[3] = ((v & 0x0F) << 4) | (K >> 8);
    data[4] =  K & 0xFF;
    data[5] =  D >> 4;
    data[6] = ((D & 0x0F) << 4) | (t >> 8);
    data[7] =  t & 0xFF;

    sendStd((uint32_t)motorID, data, 8);
    bool got = waitFeedbackForID(motorID, 200);

    g_motorID = motorID;

    if (got) {
        printFeedback(motorID);
    } else {
        Serial.printf("[WARN] ID 0x%02X 제어 후 피드백 응답 없음\n", motorID);
    }
}

// ============================================================
// 상태값 요청/출력 → 'f' 또는 'f ID'
//
// 구현 방식:
//   MIT 프로토콜에는 별도의 read-feedback 명령이 따로 없을 수 있어서,
//   현재 코드는 Enable 프레임을 한 번 보내 응답 피드백을 갱신함.
//   이미 Enable 상태인 모터에서 상태값 확인용으로 사용하는 목적.
// ============================================================
void requestAndPrintFeedback(uint8_t motorID) {
    if (!isValidMotorID(motorID)) {
        Serial.println("[ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    Serial.printf("\n[FEEDBACK REQUEST] ID=0x%02X\n", motorID);

    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    sendStd((uint32_t)motorID, data, 8);

    bool got = waitFeedbackForID(motorID, 500);

    g_motorID = motorID;
    g_motorEnabled[motorID] = true;
    updateMitReadyFlag();

    if (got) {
        printFeedback(motorID);
    } else {
        Serial.printf("[WARN] ID 0x%02X 새 피드백 응답 없음\n", motorID);
        Serial.println("       저장된 마지막 피드백을 출력합니다.");
        printFeedback(motorID);
    }
}

// ============================================================
// MIT Stop → 'x ID'
// ============================================================
void mitStop(uint8_t motorID) {
    Serial.printf("\n[MIT STOP] ID=0x%02X\n", motorID);

    if (!isValidMotorID(motorID)) {
        Serial.println("[ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    sendStd((uint32_t)motorID, data, 8);

    bool got = waitFeedbackForID(motorID, 300);

    g_motorEnabled[motorID] = false;
    g_motorID = motorID;
    updateMitReadyFlag();

    Serial.printf("[OK] ID 0x%02X Stop 완료\n", motorID);

    if (got) {
        printFeedback(motorID);
    }
}
// ============================================================
// MIT Set Zero Position → 'z ID'
// 현재 위치를 0점으로 설정
// Command 4: FF FF FF FF FF FF FF FE
// ============================================================
void setZeroPosition(uint8_t motorID) {
    Serial.printf("\n[SET ZERO] ID=0x%02X\n", motorID);

    if (!isValidMotorID(motorID)) {
        Serial.println("[ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    // 현재 위치를 0점으로 설정
    uint8_t data[8] = {
        0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFE
    };

    sendStd((uint32_t)motorID, data, 8);
    bool got = waitFeedbackForID(motorID, 500);

    g_motorID = motorID;

    if (got) {
        Serial.printf("[OK] ID 0x%02X 현재 위치를 Zero Position으로 설정 완료\n", motorID);
        Serial.println("     확인하려면 'f ID'를 입력하세요.");
    } else {
        Serial.printf("[WARN] ID 0x%02X Zero 설정 응답 없음\n", motorID);
        Serial.println("       전원 / CAN ID / MIT 모드 / 배선을 확인하세요.");
    }
}
