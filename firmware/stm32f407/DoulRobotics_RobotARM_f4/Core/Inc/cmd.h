/**
 * @file cmd.h
 * @author geon lee (sweng.geon@gmail.com)
 * @brief USB CDC 명령어 파싱 및 모터 제어 명령 처리 구현부
 * @version 0.2
 * @date 2026-06-29
 * 
 * @details
 * PC에서 USB CDC로 수신한 문자열 명령을 링버퍼에 저장하고,
 * 한 줄 단위로 파싱하여 모터 Enable/Disable, PWM, CAN 진단,
 * MOVE, GOTO, IKTEST 등의 명령을 실행한다.
 */

#ifndef __CMD_H__
#define __CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "RobStride_MIT.h"
#include <stdint.h>

/**
 * @brief 모터별 목표 명령값 구조체
 */
typedef struct {
    float   angle;    ///< 목표 위치 [rad]
    float   speed;    ///< 목표 속도 [rad/s]
    float   kp;       ///< 위치 제어 게인
    float   kd;       ///< 속도 제어 게인
    float   torque;   ///< 목표 토크 [Nm]
    uint8_t active;   ///< 1이면 주기 송신 ON
} RS_Target_t;

extern RS_Target_t g_target[3];   ///< 모터별 목표 명령값 [0]=J0, [1]=J1, [2]=J2

#define RX_RING_SIZE          512   ///< USB CDC 수신 링버퍼 크기
#define CMD_LINE_SIZE         128   ///< 명령어 한 줄 최대 길이
#define CMD_TX_PERIOD_MS      5     ///< 제어 명령 송신 주기 [ms]
#define CMD_PRINT_PERIOD_MS   100   ///< 피드백 출력 주기 [ms]

/**
 * @brief CMD 모듈을 초기화한다.
 */
void cmd_Init(void);

/**
 * @brief USB CDC 수신 데이터를 CMD 링버퍼에 저장한다.
 * @param buf 수신 데이터 버퍼
 * @param len 수신 데이터 길이
 */
void cmd_RxPush(uint8_t* buf, uint32_t len);

/**
 * @brief CMD 모듈을 주기적으로 업데이트한다.
 */
void cmd_Update(void);

/**
 * @brief 주기 피드백 출력 상태를 설정한다.
 * @param on 1이면 주기 출력 ON, 0이면 OFF
 */
void cmd_SetAutoPrint(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H__ */