//###########################################################################
// FILE:    cmd.c
// TITLE:   UART(USB CDC) 명령 처리 — 링버퍼 기반
//          STM32F407G-DISC1 / CubeIDE
//
// 설계 원칙: 받기와 처리를 분리
//   - USB 콜백(cmd_RxPush): 링버퍼에 복사만, 즉시 반환 → 누락 방지
//   - 메인 루프(cmd_Update): 링버퍼에서 줄 조립 → 파싱 → 송신/출력
//   - 응답 출력(CDC_Transmit_FS)은 메인 루프에서만 → USB 충돌 방지
//###########################################################################

#include "cmd.h"
#include "usbd_cdc_if.h"
#include "trajectory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================
 * 목표값 저장소
 * ========================================================= */
RS_Target_t g_target[3] = {0};   /* [0]=J0 [1]=J1 [2]=J2 */

/* 주기 출력 ON/OFF (기본 OFF) */
static uint8_t s_auto_print = 0;

/* CAN 진단용 외부 카운터 (RobStride_MIT.c) */
extern volatile uint32_t g_rx_callback_count;

/* =========================================================
 * 수신 링버퍼
 *  - head: 콜백(인터럽트)이 쓰는 위치
 *  - tail: 메인 루프가 읽는 위치
 *  - head==tail 이면 비어있음
 * ========================================================= */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* 줄 조립 버퍼 (메인 루프 전용) */
static char     s_line[CMD_LINE_SIZE];
static uint16_t s_line_len = 0;

/* =========================================================
 * 모터 포인터 ↔ 인덱스
 * ========================================================= */
static int RS_GetIndex(RS_Motor_t* motor)
{
    if (motor == &RS_J0) return 0;
    if (motor == &RS_J1) return 1;
    if (motor == &RS_J2) return 2;
    return -1;
}

/* =========================================================
 * 초기화
 * ========================================================= */
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

/* =========================================================
 * USB CDC 수신 콜백 — 링버퍼에 복사만 (빠르게 반환)
 *  - 인터럽트 컨텍스트에서 실행되므로 파싱/출력 금지
 *  - 버퍼 가득 차면 통제된 버림 (오버플로 방지)
 * ========================================================= */
void cmd_RxPush(uint8_t* buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint16_t next = (uint16_t)((rx_head + 1) % RX_RING_SIZE);
        if (next != rx_tail)               /* 가득 안 찼으면 */
        {
            rx_ring[rx_head] = buf[i];
            rx_head = next;
        }
        /* 가득 차면 해당 바이트 버림 */
    }
}

/* =========================================================
 * CAN 진단
 * ========================================================= */
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

/* =========================================================
 * 완성된 한 줄 파싱 + 실행
 *
 * 지원 명령:
 *   "J0 E"                    Enable (주기 송신 시작)
 *   "J0 D"                    Disable (주기 송신 중지)
 *   "J0 Z"                    Set Zero
 *   "J0 C ang spd kp kd trq"  MIT Control 목표값 설정
 *   "STATUS"                  전체 피드백 1회 출력
 *   "AUTO 1" / "AUTO 0"       주기 출력 ON/OFF
 *   "CAN"                     CAN 진단
 * ========================================================= */
static void cmd_ProcessLine(char* line)
{
    char reply[256] = {0};

    /* ---- STATUS ---- */
    if (strncmp(line, "STATUS", 6) == 0)
    {
        snprintf(reply, sizeof(reply),
            "J0: A=%.2f V=%.2f T=%.2f\r\n"
            "J1: A=%.2f V=%.2f T=%.2f\r\n"
            "J2: A=%.2f V=%.2f T=%.2f\r\n",
            RS_J0.feedback.Angle, RS_J0.feedback.Speed, RS_J0.feedback.Torque,
            RS_J1.feedback.Angle, RS_J1.feedback.Speed, RS_J1.feedback.Torque,
            RS_J2.feedback.Angle, RS_J2.feedback.Speed, RS_J2.feedback.Torque);
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    /* ---- AUTO ---- */
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

    /* ---- CAN 진단 ---- */
    if (strncmp(line, "CAN", 3) == 0)
    {
        cmd_CAN_Diagnostic();
        return;
    }

    /* ---- MOVE: 3축 동시 궤적 ----
    "MOVE qf0 qf1 qf2 T"  예: MOVE 1.57 0.5 -0.3 2.0 */
    if (strncmp(line, "MOVE", 4) == 0)
    {
        float qf0, qf1, qf2, T;
        if (sscanf(line + 4, "%f %f %f %f", &qf0, &qf1, &qf2, &T) == 4)
        {
            /* 현재 피드백 각도를 시작점으로 */
            float q0[3] = { RS_J0.feedback.Angle,
                            RS_J1.feedback.Angle,
                            RS_J2.feedback.Angle };
            float qf[3] = { qf0, qf1, qf2 };
            traj_StartAll(q0, qf, T);

            snprintf(reply, sizeof(reply),
                "MOVE: qf=(%.2f %.2f %.2f) T=%.1f\r\n", qf0, qf1, qf2, T);
        }
        else
        {
            snprintf(reply, sizeof(reply),
                "ERR: Usage: MOVE <qf0> <qf1> <qf2> <T>\r\n");
        }
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;
    }

    /* ---- E: 전체 모터 Enable ---- */
    if (strncmp(line, "E", 1) == 0 && line[1] == '\0')   // ★ "E" 한 글자만
    {
        RS_MIT_Enable(&RS_J0);
        RS_MIT_Enable(&RS_J1);
        RS_MIT_Enable(&RS_J2);
        snprintf(reply, sizeof(reply), "All motors Enable OK\r\n");
        CDC_Transmit_FS((uint8_t*)reply, strlen(reply));
        return;     // ★ 이 return이 핵심
    }

    /* ---- D: 전체 모터 Disable ---- */
    if (strncmp(line, "D", 1) == 0 && line[1] == '\0')   // ★ "D" 한 글자만
    {
        traj_StopAll();              // 궤적도 정지
        RS_MIT_Disable(&RS_J0);
        RS_MIT_Disable(&RS_J1);
        RS_MIT_Disable(&RS_J2);
        snprintf(reply, sizeof(reply), "All motors Disable OK\r\n");
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

/* =========================================================
 * 링버퍼에서 줄 조립
 *  - 패킷이 어떻게 쪼개져 와도 \r 또는 \n 만나면 한 줄로 처리
 * ========================================================= */
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

/* =========================================================
 * 주기 처리 — main while 루프에서 매번 호출
 * ========================================================= */
void cmd_Update(void)
{
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();

    /* ① 수신 링버퍼 처리 (줄 조립 → 파싱) */
    cmd_DrainRing();

    /* ③ 피드백 주기 출력 (AUTO ON 일 때만) */
    if (s_auto_print && (now - last_print >= CMD_PRINT_PERIOD_MS))
    {
        last_print = now;

        char buf[160];
        int len = snprintf(buf, sizeof(buf),
            "J0:A=%.2f V=%.2f T=%.1f | "
            "J1:A=%.2f V=%.2f T=%.1f | "
            "J2:A=%.2f V=%.2f T=%.1f\r\n",
            RS_J0.feedback.Angle, RS_J0.feedback.Speed, RS_J0.feedback.Torque,
            RS_J1.feedback.Angle, RS_J1.feedback.Speed, RS_J1.feedback.Torque,
            RS_J2.feedback.Angle, RS_J2.feedback.Speed, RS_J2.feedback.Torque);

        CDC_Transmit_FS((uint8_t*)buf, len);
    }
}
