/*
 * can_comm.h
 * ─────────────────────────────────────────────
 * TWAI (CAN) 초기화, 프레임 송수신,
 * 피드백 디코딩 및 수신 유틸리티 함수 선언
 * ─────────────────────────────────────────────
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "motor.h"

// ============================================================
// TWAI 초기화
// ============================================================
bool initTWAI();

// ============================================================
// 프레임 전송
// ============================================================
bool sendExt(uint32_t id, const uint8_t* data, uint8_t len);  // 29bit 확장
bool sendStd(uint32_t id, const uint8_t* data, uint8_t len);  // 11bit 표준

// ============================================================
// MIT 피드백 디코딩
//
// 일반적인 MIT 피드백 패킹:
//   data[0] = motor ID
//   data[1:2] = position, 16bit
//   data[3:4] = velocity, 12bit
//   data[4:5] = torque, 12bit
//   data[6:7] = temperature raw, 보통 ℃ × 10
//
// 주의:
//   모터 펌웨어/프로토콜에 따라 온도/상태 비트 위치가 다를 수 있음.
//   현재 코드는 RS02 MIT 요약 기준으로 상태값 확인용 해석.
// ============================================================
bool decodeMITFeedback(const twai_message_t& msg);

// ============================================================
// 수신 유틸리티
// ============================================================
void printRxAndTryDecode(const twai_message_t& msg, const char* prefix = "RX");
void monitorRX(uint32_t ms);
bool waitFeedbackForID(uint8_t motorID, uint32_t timeoutMs);
void flushRX(uint32_t waitMs = 20);

// ============================================================
// 피드백 출력
// ============================================================
void printFeedback(uint8_t motorID);

#endif // CAN_COMM_H
