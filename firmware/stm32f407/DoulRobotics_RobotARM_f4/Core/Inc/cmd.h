#ifndef __CMD_H__
#define __CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "RobStride_MIT.h"
#include <stdint.h>

/* =========================================================
 * 모터별 목표 명령값
 * - 파싱된 명령을 저장
 * - cmd_Update() 또는 타이머가 주기적으로 CAN 송신
 * ========================================================= */
typedef struct {
    float   angle;    /* rad */
    float   speed;    /* rad/s */
    float   kp;
    float   kd;
    float   torque;   /* Nm */
    uint8_t active;   /* 1이면 주기 송신 ON */
} RS_Target_t;

extern RS_Target_t g_target[3];   /* [0]=J0 [1]=J1 [2]=J2 */

/* =========================================================
 * 버퍼/주기 설정
 * ========================================================= */
#define RX_RING_SIZE          512   /* 수신 링버퍼 크기 (2^n 권장) */
#define CMD_LINE_SIZE         128   /* 한 줄 최대 길이 */
#define CMD_TX_PERIOD_MS      5     /* 제어 명령 송신 주기 (200Hz) */
#define CMD_PRINT_PERIOD_MS   100   /* 피드백 출력 주기 (10Hz) */

/* =========================================================
 * API
 * ========================================================= */

/* 초기화 (main의 USER CODE BEGIN 2에서 1회 호출) */
void cmd_Init(void);

/* USB CDC 수신 콜백에서 호출 — 링버퍼에 복사만 (빠르게 반환)
 * usbd_cdc_if.c 의 CDC_Receive_FS 에서 호출 */
void cmd_RxPush(uint8_t* buf, uint32_t len);

/* 메인 루프에서 매번 호출
 * - 링버퍼에서 줄 조립 → 파싱
 * - CMD_TX_PERIOD_MS 마다 활성 모터에 제어 명령 송신
 * - CMD_PRINT_PERIOD_MS 마다 피드백 출력 (AUTO ON일 때) */
void cmd_Update(void);

/* 주기 출력 ON/OFF (기본 OFF) */
void cmd_SetAutoPrint(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H__ */