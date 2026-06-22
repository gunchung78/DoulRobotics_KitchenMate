/*
 * can_comm.cpp
 * ─────────────────────────────────────────────
 * TWAI 초기화, 프레임 송수신,
 * 피드백 디코딩 및 수신 유틸리티 구현
 * ─────────────────────────────────────────────
 */

#include "can_comm.h"

// ============================================================
// TWAI 초기화
// ============================================================
bool initTWAI() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("[ERROR] TWAI 설치 실패");
        return false;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("[ERROR] TWAI 시작 실패");
        return false;
    }

    Serial.println("[OK] TWAI 초기화 완료 (1Mbps)");
    return true;
}

// ============================================================
// 29bit 확장 프레임 전송
// ============================================================
bool sendExt(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.identifier       = id;
    msg.extd             = 1;    // 29bit
    msg.rtr              = 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);

    bool ok = (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);

    Serial.printf("[TX-29bit %s] ID=0x%08lX Data:",
                  ok ? "OK  " : "FAIL", id);
    for (int i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
    Serial.println();

    return ok;
}

// ============================================================
// 11bit 표준 프레임 전송
// ============================================================
bool sendStd(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.identifier       = id;
    msg.extd             = 0;    // 11bit
    msg.rtr              = 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);

    bool ok = (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);

    Serial.printf("[TX-11bit %s] ID=0x%03lX Data:",
                  ok ? "OK  " : "FAIL", id);
    for (int i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
    Serial.println();

    return ok;
}

// ============================================================
// MIT 피드백 디코딩
// ============================================================
bool decodeMITFeedback(const twai_message_t& msg) {
    if (msg.data_length_code < 8) return false;

    uint8_t motorID = msg.data[0];

    // 일부 응답은 CAN ID 자체가 motorID이고 data[0]가 다른 상태값일 수도 있음.
    // 우선 MIT 표준 피드백 형태인 data[0] = motorID 기준으로 저장.
    if (!isValidMotorID(motorID)) {
        if (!msg.extd && isValidMotorID((int)msg.identifier)) {
            motorID = (uint8_t)msg.identifier;
        } else {
            return false;
        }
    }

    uint16_t pRaw = ((uint16_t)msg.data[1] << 8) | msg.data[2];
    uint16_t vRaw = ((uint16_t)msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t tRaw = (((uint16_t)msg.data[4] & 0x0F) << 8) | msg.data[5];

    uint16_t tempRaw = ((uint16_t)msg.data[6] << 8) | msg.data[7];

    MotorFeedback &fb = g_feedback[motorID];
    fb.valid = true;
    fb.motorID = motorID;
    fb.position = u2f(pRaw, P_MIN, P_MAX, 16);
    fb.velocity = u2f(vRaw, V_MIN, V_MAX, 12);
    fb.torque = u2f(tRaw, T_MIN, T_MAX, 12);
    fb.temperature = ((float)tempRaw) / 10.0f;
    fb.dlc = msg.data_length_code;
    fb.rxCanID = msg.identifier;
    fb.extd = msg.extd;
    fb.lastUpdateMs = millis();

    for (int i = 0; i < 8; i++) fb.raw[i] = msg.data[i];

    g_motorID = motorID;
    return true;
}

// ============================================================
// 피드백 출력
// ============================================================
void printFeedback(uint8_t motorID) {
    if (!isValidMotorID(motorID)) {
        Serial.println("[ERROR] ID 범위: 0x01~0x7F");
        return;
    }

    MotorFeedback &fb = g_feedback[motorID];

    if (!fb.valid) {
        Serial.printf("[FEEDBACK] ID 0x%02X 저장된 피드백 없음\n", motorID);
        Serial.println("           먼저 'e ID', 't ID ...', 또는 'f ID'를 실행하세요.");
        return;
    }

    Serial.println("\n╔════════════ MIT FEEDBACK ════════════╗");
    Serial.printf ("║ Motor ID      : 0x%02X (%d)\n", fb.motorID, fb.motorID);
    Serial.printf ("║ Position      : %.4f rad\n", fb.position);
    Serial.printf ("║ Velocity      : %.4f rad/s\n", fb.velocity);
    Serial.printf ("║ Torque        : %.4f Nm\n", fb.torque);
    Serial.printf ("║ Temperature   : %.1f °C\n", fb.temperature);
    Serial.printf ("║ RX CAN ID     : 0x%08lX [%s]\n",
                   fb.rxCanID, fb.extd ? "29bit" : "11bit");
    Serial.printf ("║ Last Update   : %lu ms\n", (unsigned long)fb.lastUpdateMs);
    Serial.print  ("║ Raw Data      :");
    for (int i = 0; i < fb.dlc; i++) {
        Serial.printf(" %02X", fb.raw[i]);
    }
    Serial.println();
    Serial.println("╚══════════════════════════════════════╝");
}

// ============================================================
// 수신 프레임 출력 + 피드백이면 저장
// ============================================================
void printRxAndTryDecode(const twai_message_t& msg, const char* prefix) {
    Serial.printf("  [%s-%s] ID=0x%08lX Data:",
        prefix,
        msg.extd ? "29bit" : "11bit",
        msg.identifier);

    for (int i = 0; i < msg.data_length_code; i++) {
        Serial.printf(" %02X", msg.data[i]);
    }
    Serial.println();

    if (decodeMITFeedback(msg)) {
        Serial.printf("    ↳ feedback saved: ID=0x%02X\n", g_motorID);
    }
}

// ============================================================
// 수신 모니터링
// ============================================================
void monitorRX(uint32_t ms) {
    unsigned long start = millis();

    while (millis() - start < ms) {
        twai_message_t msg;

        if (twai_receive(&msg, pdMS_TO_TICKS(20)) == ESP_OK) {
            printRxAndTryDecode(msg, "RX");
        }
    }
}

// ============================================================
// 특정 ID 응답 대기
// ============================================================
bool waitFeedbackForID(uint8_t motorID, uint32_t timeoutMs) {
    unsigned long start = millis();

    while (millis() - start < timeoutMs) {
        twai_message_t msg;

        if (twai_receive(&msg, pdMS_TO_TICKS(20)) == ESP_OK) {
            printRxAndTryDecode(msg, "RX");

            if (g_feedback[motorID].valid &&
                (millis() - g_feedback[motorID].lastUpdateMs) < 200) {
                return true;
            }
        }
    }

    return false;
}

// ============================================================
// 수신 버퍼 비우기
// ============================================================
void flushRX(uint32_t waitMs) {
    twai_message_t flush;
    while (twai_receive(&flush, pdMS_TO_TICKS(waitMs)) == ESP_OK) {
        decodeMITFeedback(flush);
    }
}
