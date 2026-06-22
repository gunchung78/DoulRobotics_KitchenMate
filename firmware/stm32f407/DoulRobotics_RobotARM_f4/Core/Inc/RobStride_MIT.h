#ifndef __ROBSTRIDE_MIT_H__
#define __ROBSTRIDE_MIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

/* =========================================================
 * 모터 모델 파라미터 (MIT mode 범위)
 * ========================================================= */
// RS02
#define RS02_P_MIN   -12.57f
#define RS02_P_MAX    12.57f
#define RS02_V_MIN   -44.0f
#define RS02_V_MAX    44.0f
#define RS02_KP_MIN   0.0f
#define RS02_KP_MAX   500.0f
#define RS02_KD_MIN   0.0f
#define RS02_KD_MAX   5.0f
#define RS02_T_MIN   -17.0f
#define RS02_T_MAX    17.0f

// RS03
#define RS03_P_MIN   -12.57f
#define RS03_P_MAX    12.57f
#define RS03_V_MIN   -20.0f
#define RS03_V_MAX    20.0f
#define RS03_KP_MIN   0.0f
#define RS03_KP_MAX   5000.0f
#define RS03_KD_MIN   0.0f
#define RS03_KD_MAX   100.0f
#define RS03_T_MIN   -60.0f
#define RS03_T_MAX    60.0f

// RS04
#define RS04_P_MIN   -12.57f
#define RS04_P_MAX    12.57f
#define RS04_V_MIN   -15.0f
#define RS04_V_MAX    15.0f
#define RS04_KP_MIN   0.0f
#define RS04_KP_MAX   5000.0f
#define RS04_KD_MIN   0.0f
#define RS04_KD_MAX   100.0f
#define RS04_T_MIN   -120.0f
#define RS04_T_MAX    120.0f

/* =========================================================
 * CAN ID 설정
 * ========================================================= */
#define J0_ID       0x01
#define J1_ID       0x02
#define J2_ID       0x03
#define MASTER_ID   0x7F

/* =========================================================
 * Communication Type (RobStride 프로토콜)
 * ========================================================= */
#define Communication_Type_MotorEnable              0x03
#define Communication_Type_MotorStop                0x04
#define Communication_Type_SetPosZero               0x06
#define Communication_Type_GetSingleParameter       0x11
#define Communication_Type_SetSingleParameter       0x12
#define Communication_Type_MotorDataSave            0x16
#define Communication_Type_BaudRateChange           0x17
#define Communication_Type_ProactiveEscalationSet   0x18
#define Communication_Type_MotorModeSet             0x19

/* =========================================================
 * CAN TX 타임아웃
 * ========================================================= */
#define CAN_TX_TIMEOUT_MS   5

/* =========================================================
 * 모터 모델 enum
 * ========================================================= */
typedef enum {
    RS02_MODEL = 0,
    RS03_MODEL,
    RS04_MODEL
} eMotorModel;

/* =========================================================
 * 모터 스펙 구조체
 * ========================================================= */
typedef struct {
    float P_MIN,  P_MAX;
    float V_MIN,  V_MAX;
    float KP_MIN, KP_MAX;
    float KD_MIN, KD_MAX;
    float T_MIN,  T_MAX;
} MotorSpec_t;

/* =========================================================
 * 모터 피드백 데이터 구조체
 * ========================================================= */
typedef struct {
    float    Angle;    // rad
    float    Speed;    // rad/s
    float    Torque;   // Nm
    float    Temp;     // °C
    uint8_t  pattern;  // 0:reset, 1:calibration, 2:operation
} MotorFeedback_t;

/* =========================================================
 * RobStride 모터 인스턴스 구조체
 * ========================================================= */
typedef struct {
    uint8_t         can_id;
    uint16_t        master_id;
    MotorSpec_t     spec;
    MotorFeedback_t feedback;
    CAN_HandleTypeDef* hcan;
} RS_Motor_t;

/* =========================================================
 * 모터 인스턴스 (전역)
 * ========================================================= */
extern RS_Motor_t RS_J0;   // Joint0 : RS04
extern RS_Motor_t RS_J1;   // Joint1 : RS03
extern RS_Motor_t RS_J2;   // Joint2 : RS02
extern RS_Motor_t* const RS_Motors[3];

/* =========================================================
 * 초기화
 * ========================================================= */
void RS_Motor_Init(RS_Motor_t* motor,
                   CAN_HandleTypeDef* hcan,
                   uint8_t can_id,
                   uint16_t master_id,
                   eMotorModel model);

void RS_AllMotors_Init(CAN_HandleTypeDef* hcan);

/* =========================================================
 * MIT 모드 Enable / Disable
 * ========================================================= */
uint8_t RS_MIT_Enable(RS_Motor_t* motor);
void RS_MIT_Disable(RS_Motor_t* motor);

/* =========================================================
 * MIT 제어 함수
 * ========================================================= */
// 토크/위치/속도/Kp/Kd 동시 제어 (가장 원시적 MIT 명령)
void RS_MIT_Control(RS_Motor_t* motor,
                    float angle_rad,
                    float speed_rad_s,
                    float kp,
                    float kd,
                    float torque_nm);

// MIT 위치 제어 (속도 제한 포함)
void RS_MIT_PositionControl(RS_Motor_t* motor,
                             float position_rad,
                             float speed_rad_s);

// MIT 속도 제어 (전류 제한 포함)
void RS_MIT_SpeedControl(RS_Motor_t* motor,
                          float speed_rad_s,
                          float current_limit);

/* =========================================================
 * 기타 유틸
 * ========================================================= */
void RS_MIT_SetZeroPos(RS_Motor_t* motor);

/* CAN2 수신 콜백 — stm32f4xx_it.c의 HAL_CAN_RxFifo0MsgPendingCallback에서 호출 */
void RS_CAN2_RxCallback(CAN_HandleTypeDef* hcan);



#ifdef __cplusplus
}
#endif

#endif /* __ROBSTRIDE_MIT_H__ */
