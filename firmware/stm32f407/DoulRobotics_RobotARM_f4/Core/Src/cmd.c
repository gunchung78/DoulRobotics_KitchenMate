/**
 * @file cmd.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmd.h"
#include "config.h"
#include "RobStride_MIT.h"
#include "usbd_cdc_if.h"
#include "trajectory.h"
#include "pd_control.h"
#include "axis.h"
#include "kin_bridge.h"
#include "pwm.h"

RS_Target_t g_target[MOTOR_COUNT] = {0};

static uint8_t s_auto_print = 0;                ///< 주기 출력 ON/OFF (기본 OFF)

extern volatile uint32_t g_rx_callback_count;   ///< CAN 진단용 외부 카운터 (RobStride_MIT.c)

static volatile uint8_t  rx_ring[RX_RING_SIZE]; ///< 링버퍼
static volatile uint16_t rx_head = 0;           ///< 콜백(인터럽트) 쓰는 위치
static volatile uint16_t rx_tail = 0;           ///< 메인 루프가 읽는 위치

static char     s_line[CMD_LINE_SIZE];          ///< 입력 배열
static uint16_t s_line_len = 0;                 ///< 입력 길이 초기화


/* 모터 인덱스 */
static int RS_GetIndex(RS_Motor_t* motor)
{
    if (motor == &RS_J0) return 0;
    if (motor == &RS_J1) return 1;
    if (motor == &RS_J2) return 2;
    return -1;
}

/* 초기화 */
void cmd_Init(void)
{
    memset(g_target, 0, sizeof(g_target));
    rx_head = 0;
    rx_tail = 0;
    s_line_len = 0;
    s_auto_print = 0;
}

void cmd_SetAutoPrint(uint8_t on)
{
    s_auto_print = on ? 1 : 0;
}

/* USB CD 수신 콜백 (링버퍼)*/
void cmd_RxPush(uint8_t* buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
        if (next != rx_tail)
        {
            /* 가득 안 찼으면*/
            rx_ring[rx_head] = buf[i];
            rx_head = next;
        }
        /* 가득 찼으면 버림*/
    }
}

/* CAN 진단 */
static void cmd_CAN_Diagnostic(void)
{
    char reply[256] = {0};
    CAN_HandleTypeDef* hcan = RS_J2.hcan;

    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan);
    uint32_t err = HAL_CAN_GetError(hcan);

    uint32_t esr = hcan->Instance->ESR;
    uint8_t tec  = (esr >> 16) & 0xFF;
    uint8_t rec  = (esr >> 24) & 0xFF;
    uint8_t boff = (esr >> 2) & 0x1;
    uint8_t epvf = (esr >> 1) & 0x1;
    uint8_t ewgf = (esr >> 0) & 0x1;

    uint32_t free_tx    = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
    uint32_t rx_pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

    snprintf(reply, sizeof(reply),
        "=== CAN DIAG ===\r\n"
        "HAL State : %d\r\n"
        "RX cb cnt : %lu\r\n"
        "Error Code: 0x%08lX\r\n"
        "TEC=%u REC=%u\r\n"
        "BusOff=%u Passive=%u Warning=%u\r\n"
        "TX free=%lu  RX pending=%lu\r\n"
        "================\r\n",
        (int)state,
        (unsigned long)g_rx_callback_count,
        (unsigned long)err,
        tec, rec, boff, epvf, ewgf,
        (unsigned long)free_tx, (unsigned long)rx_pending);

    CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
}

/*
 * 완성된 한 줄 파싱 + 실행
 *
 * 지원 명령:
 *   "J0 E"                         Enable (주기 송신 시작)
 *   "J0 D"                         Disable (주기 송신 중지)
 *   "J0 Z"                         Set Zero
 *   "E"
 *   "D"
 *   "J0 C ang spd kp kd trq"       MIT Control 목표값 설정
 *   "BK1/BK2/SOL ON"
 *   "BK1/BK2/SOL OFF" 
 *   "PWM STATUS"
 *   "STATUS"                       전체 피드백 1회 출력
 *   "AUTO 1" / "AUTO 0"            주기 출력 ON/OFF
 *   "CAN"                          CAN 진단
 *   "MOVE q0_rad q1_rad j2_mm T"
 *   "PD axis kp kd"
 *   "GOTO x_mm y_mm z_mm T"  예: GOTO -700 100 50 3 
 *   "IKTEST x y z"
*/
static void cmd_ProcessLine(char* line)
{
    char reply[256] = {0};

    /* ---- STATUS ---- */
    if (strncmp(line, "STATUS", 6) == 0)
    {
        snprintf(reply, sizeof(reply),
            "J0: A=%.2f V=%.2f T=%.2f\r\n"
            "J1: A=%.2f V=%.2f T=%.2f\r\n"
            "J2: A=%.2f V=%.2f T=%.2f\r\n"
            "J2: A=%.2fmm  V=%.2fmm/s T=%.2f\r\n",
            RS_J0.feedback.Angle, RS_J0.feedback.Speed, RS_J0.feedback.Torque,
            RS_J1.feedback.Angle, RS_J1.feedback.Speed, RS_J1.feedback.Torque,
            RS_J2.feedback.Angle, RS_J2.feedback.Speed, RS_J2.feedback.Torque,
            axis_J2_GetContinuousMm(),
            axis_J2_RadpsToMmps(RS_J2.feedback.Speed),
            RS_J2.feedback.Torque);
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "AUTO", 4) == 0)
    {
        int on = 0;
        if (sscanf(line + 4, "%d", &on) == 1)
        {
            cmd_SetAutoPrint((uint8_t)on);
            snprintf(reply, sizeof(reply), "AUTO print = %d\r\n", on ? 1 : 0);
        }
        else
        {
            snprintf(reply, sizeof(reply), "ERR: Usage: AUTO <0|1>\r\n");
        }
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "CAN", 3) == 0)
    {
        cmd_CAN_Diagnostic();
        return;
    }

    if (strcmp(line, "BK1 ON") == 0)
    {
        PWM_BK1_On();
        CDC_Transmit_FS((uint8_t*)"[PWM] BK1 ON\r\n", 14);
        return;
    }

    if (strcmp(line, "BK1 OFF") == 0)
    {
        PWM_BK1_Off();
        CDC_Transmit_FS((uint8_t*)"[PWM] BK1 OFF\r\n", 15);
        return;
    }

    if (strcmp(line, "BK2 ON") == 0)
    {
        PWM_BK2_On();
        CDC_Transmit_FS((uint8_t*)"[PWM] BK2 ON\r\n", 14);
        return;
    }

    if (strcmp(line, "BK2 OFF") == 0)
    {
        PWM_BK2_Off();
        CDC_Transmit_FS((uint8_t*)"[PWM] BK2 OFF\r\n", 15);
        return;
    }

    if (strcmp(line, "SOL ON") == 0)
    {
        PWM_SOL_On();
        CDC_Transmit_FS((uint8_t*)"[PWM] SOL ON\r\n", 14);
        return;
    }

    if (strcmp(line, "SOL OFF") == 0)
    {
        PWM_SOL_Off();
        CDC_Transmit_FS((uint8_t*)"[PWM] SOL OFF\r\n", 15);
        return;
    }

    if (strcmp(line, "PWM STATUS") == 0)
    {
        PWM_PrintStatus();
        return;
    }
    
    if (strncmp(line, "MOVE", 4) == 0)
    {
        float q0_rad, q1_rad, j2_mm, T;
        if (sscanf(line + 4, "%f %f %f %f", &q0_rad, &q1_rad, &j2_mm, &T) == 4)
        {
            /* J2: mm → 연속각(rad) 변환 */
            float j2_rad = axis_J2_MmToRad(j2_mm);

            /* 시작점: J0/J1은 raw 피드백, J2는 연속각 */
            float start[3] = {
                RS_J0.feedback.Angle,
                RS_J1.feedback.Angle,
                axis_J2_GetContinuousRad()
            };
            float goal[3] = { q0_rad, q1_rad, j2_rad };

            traj_StartAll(start, goal, T);

            snprintf(reply, sizeof(reply),
                "MOVE: J0=%.2frad J1=%.2frad J2=%.1fmm(%.3frad) T=%.1f\r\n",
                q0_rad, q1_rad, j2_mm, j2_rad, T);
        }
        else
        {
            snprintf(reply, sizeof(reply),
                "ERR: Usage: MOVE <j0_rad> <j1_rad> <j2_mm> <T>\r\n");
        }
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "E", 1) == 0 && line[1] == '\0')
    {
        RS_MIT_Enable(&RS_J0);
        RS_MIT_Enable(&RS_J1);
        RS_MIT_Enable(&RS_J2);
        snprintf(reply, sizeof(reply), "All motors Enable OK\r\n");
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;     // ★ 이 return이 핵심
    }

    if (strncmp(line, "D", 1) == 0 && line[1] == '\0')
    {
        traj_StopAll();
        RS_MIT_Disable(&RS_J0);
        RS_MIT_Disable(&RS_J1);
        RS_MIT_Disable(&RS_J2);
        snprintf(reply, sizeof(reply), "All motors Disable OK\r\n");
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "PD", 2) == 0)
    {
        int axis; float kp, kd;
        if (sscanf(line + 2, "%d %f %f", &axis, &kp, &kd) == 3)
        {
            pd_SetGains(axis, kp, kd);
            snprintf(reply, sizeof(reply), "PD J%d: kp=%.2f kd=%.2f\r\n", axis, kp, kd);
        }
        else snprintf(reply, sizeof(reply), "ERR: PD <axis> <kp> <kd>\r\n");
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "GOTO", 4) == 0)
    {
        float x, y, z, T;
        if (sscanf(line + 4, "%f %f %f %f", &x, &y, &z, &T) == 4)
        {
            /* 현재 관절 (시드 + 궤적 시작점) */
            float cur_j0 = RS_J0.feedback.Angle;
            float cur_j1 = RS_J1.feedback.Angle;
            float cur_j2_mm = axis_J2_GetContinuousMm();

            /* IK 풀기 */
            KinTarget_t tgt;
            kin_SolveGoto(x, y, z, cur_j0, cur_j1, cur_j2_mm, &tgt);

            if (tgt.ok)
            {
                /* 시작점: J0/J1 raw, J2 연속각 */
                float start[3] = {
                    cur_j0, cur_j1, axis_J2_GetContinuousRad()
                };
                float goal[3] = { tgt.j0_rad, tgt.j1_rad, tgt.j2_rad };

                traj_StartAll(start, goal, T);

                snprintf(reply, sizeof(reply),
                    "GOTO ok: J0=%.2f J1=%.2f J2=%.1fmm err=%.2fmm T=%.1f\r\n",
                    tgt.j0_rad, tgt.j1_rad, axis_J2_RadToMm(tgt.j2_rad),
                    tgt.error_mm, T);
            }
            else
            {
                snprintf(reply, sizeof(reply),
                    "GOTO unreachable: err=%.2fmm (closest)\r\n", tgt.error_mm);
            }
        }
        else
        {
            snprintf(reply, sizeof(reply),
                "ERR: Usage: GOTO <x_mm> <y_mm> <z_mm> <T>\r\n");
        }
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    if (strncmp(line, "IKTEST", 6) == 0)
    {
        float x, y, z;
        if (sscanf(line + 6, "%f %f %f", &x, &y, &z) == 3)
        {
            float cur_j2_mm = axis_J2_GetContinuousMm();
            KinTarget_t tgt;
            kin_SolveGoto(x, y, z, RS_J0.feedback.Angle,
                        RS_J1.feedback.Angle, cur_j2_mm, &tgt);

            snprintf(reply, sizeof(reply),
                "IK: ok=%d J0=%.3f J1=%.3f J2=%.1fmm err=%.2fmm\r\n",
                tgt.ok,
				tgt.j0_rad * 57.2958f,
				tgt.j1_rad * 57.2958f,
                axis_J2_RadToMm(tgt.j2_rad),
				tgt.error_mm);
        }
        else snprintf(reply, sizeof(reply), "ERR: IKTEST <x> <y> <z>\r\n");
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    /* ---- 관절 선택 ---- */
    RS_Motor_t* motor = NULL;
    if      (strncmp(line, "J0", 2) == 0) motor = &RS_J0;
    else if (strncmp(line, "J1", 2) == 0) motor = &RS_J1;
    else if (strncmp(line, "J2", 2) == 0) motor = &RS_J2;
    else
    {
        CDC_Transmit_FS((uint8_t*)"ERR: Unknown joint\r\n", 20);
        return;
    }

    int idx = RS_GetIndex(motor);
    char* p = line + 3;   /* "J? " 다음 문자 */

    if (*p == 'E')        /* Enable */
    {
        RS_MIT_Enable(motor);
        snprintf(reply, sizeof(reply), "%s Enable OK\r\n", line);
    }
    else if (*p == 'D')   /* Disable */
    {
        if (idx >= 0) g_target[idx].active = 0;
        RS_MIT_Disable(motor);
        snprintf(reply, sizeof(reply), "%s Disable OK\r\n", line);
    }
    else if (*p == 'Z')   /* Set Zero */
    {
        RS_MIT_SetZeroPos(motor);
        uint32_t cnt = g_rx_callback_count;
        uint32_t t0 = HAL_GetTick();
        while (g_rx_callback_count == cnt && (HAL_GetTick()-t0) < 20) {}
        if (motor == &RS_J2) {
            axis_J2_ResetUnwrap(RS_J2.feedback.Angle);
            pd_SetGoal(2, axis_J2_GetContinuousRad());
        } else {
            pd_SetGoal(idx, 0.0f);
        }
        snprintf(reply, sizeof(reply), "%s SetZero OK\r\n", line);
    }
    else if (*p == 'C')   /* MIT Control 목표값 설정 */
    {
        float ang=0, spd=0, kp=0, kd=0, trq=0;
        int n = sscanf(p + 2, "%f %f %f %f %f", &ang, &spd, &kp, &kd, &trq);
        if (n == 5 && idx >= 0)
        {
            g_target[idx].angle  = ang;
            g_target[idx].speed  = spd;
            g_target[idx].kp     = kp;
            g_target[idx].kd     = kd;
            g_target[idx].torque = trq;
            g_target[idx].active = 1;

            snprintf(reply, sizeof(reply),
                "%s MIT set: A=%.2f S=%.2f Kp=%.1f Kd=%.2f T=%.2f\r\n",
                line, ang, spd, kp, kd, trq);
        }
        else
        {
            snprintf(reply, sizeof(reply),
                "ERR: Usage: J? C <ang> <spd> <kp> <kd> <trq>\r\n");
        }
    }
    
    else
    {
        snprintf(reply, sizeof(reply), "ERR: Unknown command\r\n");
    }

    CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
}

/* 패킷이 어떻게 쪼개져 와도 \r 또는 \n 만나면 한 줄로 처리 */
static void cmd_DrainRing(void)
{
    while (rx_tail != rx_head)
    {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) % RX_RING_SIZE);

        if (c == '\n' || c == '\r')
        {
            if (s_line_len > 0)
            {
                s_line[s_line_len] = '\0';
                cmd_ProcessLine(s_line);
                s_line_len = 0;
            }
            /* 빈 줄(\r\n 연속 등)은 무시 */
        }
        else if (s_line_len < (CMD_LINE_SIZE - 1))
        {
            s_line[s_line_len++] = c;
        }
        else
        {
            /* 줄이 너무 길면 리셋 (오버플로 방지) */
            s_line_len = 0;
        }
    }
}

/* main while */
void cmd_Update(void)
{
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();

    /* 수신 링버퍼 처리 (줄 조립 → 파싱) */
    cmd_DrainRing();

    /* 피드백 주기 출력 (AUTO ON 일 때만) */
    if (s_auto_print && (now - last_print >= CMD_PRINT_PERIOD_MS))
    {
        last_print = now;

        char buf[160];
        int len = snprintf(buf, sizeof(buf),
            "J0: A=%.2f V=%.2f T=%.2f\r\n"
            "J1: A=%.2f V=%.2f T=%.2f\r\n"
            "J2: A=%.2f V=%.2f T=%.2f\r\n"
            "J2: A=%.2fmm  V=%.2fmm/s T=%.2f\r\n",
            RS_J0.feedback.Angle, RS_J0.feedback.Speed, RS_J0.feedback.Torque,
            RS_J1.feedback.Angle, RS_J1.feedback.Speed, RS_J1.feedback.Torque,
            RS_J2.feedback.Angle, RS_J2.feedback.Speed, RS_J2.feedback.Torque,
            axis_J2_GetContinuousMm(),
            axis_J2_RadpsToMmps(RS_J2.feedback.Speed),
            RS_J2.feedback.Torque);

        CDC_Transmit_FS((uint8_t*)buf, len);
    }
}
