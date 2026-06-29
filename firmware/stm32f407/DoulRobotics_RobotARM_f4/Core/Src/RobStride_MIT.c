#include "RobStride_MIT.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

RS_Motor_t RS_J0;  ///< Joint0 : RS03
RS_Motor_t RS_J1;  ///< Joint1 : RS03
RS_Motor_t RS_J2;  ///< Joint2 : RS02
volatile uint32_t g_rx_callback_count = 0;
RS_Motor_t* const RS_Motors[MOTOR_COUNT] = { &RS_J0, &RS_J1, &RS_J2 };

typedef struct 
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    volatile uint8_t pending;
} RS_CanTxSlot_t;

static RS_CanTxSlot_t s_tx_slot[MOTOR_COUNT];
static volatile uint8_t s_tx_rr = 0;
static volatile uint8_t s_tx_pumping = 0;

volatile uint32_t g_can_tx_overwrite_count = 0;
volatile uint32_t g_can_tx_error_count = 0;

static int RS_CAN2_GetTxSlotByMotor(RS_Motor_t *motor)
{
    if (motor == &RS_J0) return 0;
    if (motor == &RS_J1) return 1;
    if (motor == &RS_J2) return 2;

    return -1;
}

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    x &= span;
    return (x_max - x_min) * (float)x / (float)span + x_min;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)span / (x_max - x_min));
}

static uint8_t CAN_Send(CAN_HandleTypeDef* hcan,
                        CAN_TxHeaderTypeDef* pHeader,
                        uint8_t* pData)
{
    uint32_t TxMailbox;
    uint32_t tick = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if ((HAL_GetTick() - tick) > CAN_TX_TIMEOUT_MS)
            return 1;
    }

    if (HAL_CAN_AddTxMessage(hcan, pHeader, pData, &TxMailbox) != HAL_OK)
        return 2;

    return 0;
}

void RS_CAN2_TxPump(CAN_HandleTypeDef *hcan)
{
    uint32_t txMailbox;

    /* 중복 실행 방지 */
    __disable_irq();
    if (s_tx_pumping) 
    {
        __enable_irq();
        return;
    }
    s_tx_pumping = 1;
    __enable_irq();

    /* CAN MAILBOX 비어있으면 송신 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        CAN_TxHeaderTypeDef header;
        uint8_t data[8];
        int found = -1;

        __disable_irq();

        /* 모터 인덱스 선택 */
        for (int k = 0; k < MOTOR_COUNT; k++)
        {
            int idx = (s_tx_rr + k) % MOTOR_COUNT;
            if (s_tx_slot[idx].pending)
            {
                found = idx;
                break;
            }
        }

        /* 보낼 메시지 없으면 종료 */
        if (found < 0)
        {
            s_tx_pumping = 0;
            __enable_irq();
            return;
        }

        /* slot에서 메시지 꺼내고 pending 해제 */
        header = s_tx_slot[found].header;
        memcpy(data, s_tx_slot[found].data, 8);

        /* pending 초기화 후 다음 slot 검사 */
        s_tx_slot[found].pending = 0;
        s_tx_rr = (uint8_t)((found + 1) % MOTOR_COUNT);

        __enable_irq();

        /* 실제 can 송신 요청 */
        if (HAL_CAN_AddTxMessage(hcan, &header, data, &txMailbox) != HAL_OK)
        {
            /* 송신 등록 실패 시 복구 */
            __disable_irq();

            s_tx_slot[found].header = header;
            memcpy(s_tx_slot[found].data, data, 8);
            s_tx_slot[found].pending = 1;

            g_can_tx_error_count++;
            s_tx_pumping = 0;

            __enable_irq();
            return;
        }
    }

    /* Mailbox 꽉 차면 종료 */
    __disable_irq();
    s_tx_pumping = 0;
    __enable_irq();
}

static uint8_t RS_CAN2_SendLatest(RS_Motor_t *motor,
                                  CAN_TxHeaderTypeDef *header,
                                  uint8_t *data)
{
    int slot = RS_CAN2_GetTxSlotByMotor(motor);

    if (slot < 0) return 1;

    __disable_irq();

    /* 이전 메시지(pending == 1) 있으면 overwrite 카운트 증가 */
    if(s_tx_slot[slot].pending) g_can_tx_overwrite_count++;

    /* 최신 메시지 저장 */
    s_tx_slot[slot].header = *header;
    memcpy(s_tx_slot[slot].data, data, 8);
    s_tx_slot[slot].pending = 1;

    __enable_irq();

    /* 송신 펌프 실행 */
    RS_CAN2_TxPump(motor->hcan);

    return 0;
}

static void LoadSpec(MotorSpec_t* spec, eMotorModel model)
{
    switch (model)
    {
        case RS02_MODEL:
            spec->P_MIN = RS02_P_MIN;  spec->P_MAX = RS02_P_MAX;
            spec->V_MIN = RS02_V_MIN;  spec->V_MAX = RS02_V_MAX;
            spec->KP_MIN = RS02_KP_MIN; spec->KP_MAX = RS02_KP_MAX;
            spec->KD_MIN = RS02_KD_MIN; spec->KD_MAX = RS02_KD_MAX;
            spec->T_MIN = RS02_T_MIN;  spec->T_MAX = RS02_T_MAX;
            break;
        case RS03_MODEL:
            spec->P_MIN = RS03_P_MIN;  spec->P_MAX = RS03_P_MAX;
            spec->V_MIN = RS03_V_MIN;  spec->V_MAX = RS03_V_MAX;
            spec->KP_MIN = RS03_KP_MIN; spec->KP_MAX = RS03_KP_MAX;
            spec->KD_MIN = RS03_KD_MIN; spec->KD_MAX = RS03_KD_MAX;
            spec->T_MIN = RS03_T_MIN;  spec->T_MAX = RS03_T_MAX;
            break;
        case RS04_MODEL:
        default:
            spec->P_MIN = RS04_P_MIN;  spec->P_MAX = RS04_P_MAX;
            spec->V_MIN = RS04_V_MIN;  spec->V_MAX = RS04_V_MAX;
            spec->KP_MIN = RS04_KP_MIN; spec->KP_MAX = RS04_KP_MAX;
            spec->KD_MIN = RS04_KD_MIN; spec->KD_MAX = RS04_KD_MAX;
            spec->T_MIN = RS04_T_MIN;  spec->T_MAX = RS04_T_MAX;
            break;
    }
}

void RS_Motor_Init(RS_Motor_t* motor,
                   CAN_HandleTypeDef* hcan,
                   uint8_t can_id,
                   uint16_t master_id,
                   eMotorModel model)
{
    memset(motor, 0, sizeof(RS_Motor_t));
    motor->hcan      = hcan;
    motor->can_id    = can_id;
    motor->master_id = master_id;
    LoadSpec(&motor->spec, model);
}

void RS_AllMotors_Init(CAN_HandleTypeDef* hcan)
{
    // 원본 cpp와 동일: J0=RS04, J1=RS03, J2=RS02
	RS_Motor_Init(&RS_J0, hcan, J0_ID, MASTER_ID, RS03_MODEL);
	RS_Motor_Init(&RS_J1, hcan, J1_ID, MASTER_ID, RS03_MODEL);
	RS_Motor_Init(&RS_J2, hcan, J2_ID, MASTER_ID, RS02_MODEL);
}

/* =========================================================
 * MIT Enable (StdId = CAN_ID, data = 0xFF...0xFC)
 * 원본: RobStride_Motor_MIT_Enable()
 * ========================================================= */
uint8_t RS_MIT_Enable(RS_Motor_t* motor)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    TxHeader.StdId              = motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    return CAN_Send(motor->hcan, &TxHeader, txdata);
}

/* =========================================================
 * MIT Disable (StdId = CAN_ID, data = 0xFF...0xFD)
 * 원본: RobStride_Motor_MIT_Disable()
 * ========================================================= */
void RS_MIT_Disable(RS_Motor_t* motor)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    TxHeader.StdId              = motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    CAN_Send(motor->hcan, &TxHeader, txdata);
}

/* =========================================================
 * MIT Set Zero Position (data = 0xFF...0xFE)
 * 원본: RobStride_Motor_MIT_SetZeroPos()
 * ========================================================= */
void RS_MIT_SetZeroPos(RS_Motor_t* motor)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

    TxHeader.StdId              = motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    CAN_Send(motor->hcan, &TxHeader, txdata);
}

/* =========================================================
 * MIT Control (원시 MIT 명령 — 토크/위치/속도/Kp/Kd 동시)
 * 원본: RobStride_Motor_MIT_Control()
 *
 * 프레임 구조 (8 bytes):
 * [0~1] : position (16bit)
 * [2~3] : velocity (12bit) | kp high (4bit)
 * [4]   : kp low (8bit)
 * [5]   : kd (12bit high 8bit)
 * [6]   : kd low (4bit) | torque high (4bit)
 * [7]   : torque low (8bit)
 * ========================================================= */
void RS_MIT_Control(RS_Motor_t* motor,
                    float angle_rad,
                    float speed_rad_s,
                    float kp,
                    float kd,
                    float torque_nm)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0};

    uint16_t p_uint  = float_to_uint(angle_rad,  motor->spec.P_MIN,  motor->spec.P_MAX,  16);
    uint16_t v_uint  = float_to_uint(speed_rad_s, motor->spec.V_MIN,  motor->spec.V_MAX,  12);
    uint16_t kp_uint = float_to_uint(kp,          motor->spec.KP_MIN, motor->spec.KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd,          motor->spec.KD_MIN, motor->spec.KD_MAX, 12);
    uint16_t t_uint  = float_to_uint(torque_nm,   motor->spec.T_MIN,  motor->spec.T_MAX,  12);

    txdata[0] = p_uint >> 8;
    txdata[1] = p_uint & 0xFF;
    txdata[2] = (v_uint >> 4) & 0xFF;
    txdata[3] = ((v_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF);
    txdata[4] = kp_uint & 0xFF;
    txdata[5] = (kd_uint >> 4) & 0xFF;
    txdata[6] = ((kd_uint & 0xF) << 4) | ((t_uint >> 8) & 0xF);
    txdata[7] = t_uint & 0xFF;

    TxHeader.StdId              = motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    RS_CAN2_SendLatest(motor, &TxHeader, txdata);
}

/* =========================================================
 * MIT Position Control
 * 원본: RobStride_Motor_MIT_PositionControl()
 * StdId = (1 << 8) | CAN_ID
 * data[0~3] = position_rad (float)
 * data[4~7] = speed_rad_per_s (float)
 * ========================================================= */
void RS_MIT_PositionControl(RS_Motor_t* motor,
                             float position_rad,
                             float speed_rad_s)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0};

    memcpy(&txdata[0], &position_rad, 4);
    memcpy(&txdata[4], &speed_rad_s,  4);

    TxHeader.StdId              = (1 << 8) | motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    CAN_Send(motor->hcan, &TxHeader, txdata);
}

/* =========================================================
 * MIT Speed Control
 * 원본: RobStride_Motor_MIT_SpeedControl()
 * StdId = (2 << 8) | CAN_ID
 * data[0~3] = speed_rad_per_s (float)
 * data[4~7] = current_limit   (float)
 * ========================================================= */
void RS_MIT_SpeedControl(RS_Motor_t* motor,
                          float speed_rad_s,
                          float current_limit)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txdata[8] = {0};

    memcpy(&txdata[0], &speed_rad_s,    4);
    memcpy(&txdata[4], &current_limit,  4);

    TxHeader.StdId              = (2 << 8) | motor->can_id;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    CAN_Send(motor->hcan, &TxHeader, txdata);
}

/* =========================================================
 * CAN2 수신 콜백 (매뉴얼 기준 수정본)
 *
 * MIT 피드백 프레임 (Response Command 1):
 *   StdId      = Host CAN ID (모터ID 아님!)
 *   data[0]    = Motor CAN ID        ★ 여기에 모터 ID가 있음
 *   data[1~2]  = 각도   (16bit)
 *   data[3], data[4]>>4 = 속도 (12bit)
 *   (data[4]&0xF), data[5] = 토크 (12bit)
 *   data[6~7]  = 온도 (×10)
 * ========================================================= */
void RS_CAN2_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t d[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, d) != HAL_OK)
        return;

    g_rx_callback_count++;

    /* ★ 모터 ID는 data[0]에 있다 */
    uint8_t motor_id = d[0];

    RS_Motor_t *motor = NULL;
    if      (motor_id == J0_ID) motor = &RS_J0;
    else if (motor_id == J1_ID) motor = &RS_J1;
    else if (motor_id == J2_ID) motor = &RS_J2;
    else return;

    /* ★ 데이터 오프셋 1칸씩 밀림 수정 */
    uint16_t p_raw    = ((uint16_t)d[1] << 8) | d[2];
    uint16_t v_raw    = ((uint16_t)d[3] << 4) | (d[4] >> 4);
    uint16_t t_raw    = ((uint16_t)(d[4] & 0xF) << 8) | d[5];
    uint16_t temp_raw = ((uint16_t)d[6] << 8) | d[7];

    motor->feedback.Angle   = uint16_to_float(p_raw, motor->spec.P_MIN, motor->spec.P_MAX, 16);
    motor->feedback.Speed   = uint16_to_float(v_raw, motor->spec.V_MIN, motor->spec.V_MAX, 12);
    motor->feedback.Torque  = uint16_to_float(t_raw, motor->spec.T_MIN, motor->spec.T_MAX, 12);
    motor->feedback.Temp    = (float)temp_raw / 10.0f;   /* ★ ÷10 */
    motor->feedback.pattern = 0;
}
