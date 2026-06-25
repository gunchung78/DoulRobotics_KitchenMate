/**
 * servo_control.c
 * RS232 텍스트 명령 → TIM4 PWM 서보 제어
 *
 * 동작 흐름:
 *   UART RX 인터럽트(1 byte씩) → RS232_ProcessByte()
 *   → '\n' 수신 시 파싱 → Servo_SetAngle() / Servo_SetMode()
 *   → TIM4 CCR1 업데이트 → 서보 이동
 *   → RS232 응답 전송
 */

#include "servo_control.h"

/* ── 내부 상태 ────────────────────────────────────────── */
static ServoState_t s_servo = {
    .mode  = SERVO_MODE_90,
    .angle = 45,
    .ccr   = 75   /* 45° @ 90°모드 초기값 */
};

/* RX 링버퍼 (인터럽트 → 파싱) */
static uint8_t  s_rx_buf[RX_BUF_SIZE];
uint8_t  s_rx_byte;          /* HAL 1바이트 수신용 */
static uint16_t s_rx_idx = 0;

/* TX 임시버퍼 */
static char s_tx_buf[128];

/* ── 내부 함수 선언 ────────────────────────────────────── */
static uint32_t Servo_AngleToCCR(uint8_t angle);
static void     Servo_ParseLine(char *line);
static void     Servo_SendStatus(void);
static void     Servo_SendHelp(void);

/* ─────────────────────────────────────────────────────── */
/*  공개 API                                               */
/* ─────────────────────────────────────────────────────── */

/**
 * @brief 서보 + RS232 초기화. main.c USER CODE BEGIN 2에 호출.
 */
void Servo_Init(void)
{
    /* 초기 CCR 계산 후 PWM 시작 */
    s_servo.ccr = Servo_AngleToCCR(s_servo.angle);
    __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, s_servo.ccr);
    HAL_TIM_PWM_Start(&SERVO_TIMER, SERVO_CHANNEL);

    /* RS232 RX 인터럽트 활성화 (1바이트씩) */
    HAL_UART_Receive_IT(&UART_RS232, &s_rx_byte, 1);

    RS232_SendString("\r\n=== STM32 Servo RS232 Controller ===\r\n");
    Servo_SendHelp();
    Servo_SendStatus();
}

/**
 * @brief 서보 각도 설정. CCR 계산 후 TIM4 레지스터 즉시 반영.
 * @param angle  목표 각도 (0 ~ mode_max)
 */
void Servo_SetAngle(uint8_t angle)
{
    uint8_t max_angle = (uint8_t)s_servo.mode;

    if (angle > max_angle) {
        snprintf(s_tx_buf, sizeof(s_tx_buf),
                 "[ERR] Angle out of range. Max=%u for %u-deg mode\r\n",
                 max_angle, max_angle);
        RS232_SendString(s_tx_buf);
        return;
    }

    s_servo.angle = angle;
    s_servo.ccr   = Servo_AngleToCCR(angle);

    __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, s_servo.ccr);

    uint32_t us = (s_servo.ccr) * 20UL;   /* CCR × 20μs/tick */
    snprintf(s_tx_buf, sizeof(s_tx_buf),
             "[OK] Servo -> %u deg | CCR=%lu | pulse=%lu us\r\n",
             s_servo.angle, s_servo.ccr, us);
    RS232_SendString(s_tx_buf);
}

/**
 * @brief 서보 범위 모드 전환 (90° ↔ 180°).
 *        전환 후 현재 각도를 클램프하여 재적용.
 */
void Servo_SetMode(ServoMode_t mode)
{
    s_servo.mode = mode;

    /* 현재 각도가 새 범위를 초과하면 클램프 */
    uint8_t max_angle = (uint8_t)mode;
    if (s_servo.angle > max_angle) {
        s_servo.angle = max_angle;
    }
    s_servo.ccr = Servo_AngleToCCR(s_servo.angle);
    __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, s_servo.ccr);

    snprintf(s_tx_buf, sizeof(s_tx_buf),
             "[OK] Mode -> %u-deg | Angle clamped to %u deg\r\n",
             (uint8_t)mode, s_servo.angle);
    RS232_SendString(s_tx_buf);
}

/**
 * @brief 현재 서보 상태 반환.
 */
ServoState_t Servo_GetState(void)
{
    return s_servo;
}

/**
 * @brief RS232 문자열 송신 (블로킹).
 */
void RS232_SendString(const char *str)
{
    HAL_UART_Transmit(&UART_RS232,
                      (uint8_t *)str,
                      (uint16_t)strlen(str),
                      100);
}

/**
 * @brief UART RX 콜백에서 1바이트씩 호출.
 *        '\n' 수신 시 라인 파싱 트리거.
 *
 * 사용법 (stm32f4xx_it.c 또는 main.c):
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART2) {
 *           RS232_ProcessByte(s_rx_byte);          // 이 함수 호출
 *           HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1); // 재등록
 *       }
 *   }
 */
void RS232_ProcessByte(uint8_t byte)
{
    /* 에코 (선택사항: 터미널에서 입력 확인용) */
    HAL_UART_Transmit(&UART_RS232, &byte, 1, 10);

    if (byte == '\n' || byte == '\r') {   // ← \r 추가
        if (s_rx_idx > 0) {
            s_rx_buf[s_rx_idx] = '\0';
            Servo_ParseLine((char *)s_rx_buf);
            s_rx_idx = 0;
        }
        return;
    }

    /* 버퍼 오버플로 방지 */
    if (s_rx_idx < RX_BUF_SIZE - 1) {
        s_rx_buf[s_rx_idx++] = (char)byte;
    }
}

/* ─────────────────────────────────────────────────────── */
/*  내부 함수                                              */
/* ─────────────────────────────────────────────────────── */

/**
 * @brief 각도 → CCR 변환 (모드 자동 선택).
 *
 * 90° 모드:  CCR = 50  + angle × 50/90   (1000~2000μs)
 * 180° 모드: CCR = 25  + angle × 100/180 (500~2500μs)
 */
static uint32_t Servo_AngleToCCR(uint8_t angle)
{
    if (s_servo.mode == SERVO_MODE_90) {
        /* 50 + (angle/90) * 50  →  정수: 50 + angle*50/90 */
        return SERVO_90_CCR_MIN +
               ((uint32_t)angle * (SERVO_90_CCR_MAX - SERVO_90_CCR_MIN))
               / (uint32_t)SERVO_MODE_90;
    } else {
        /* 25 + (angle/180) * 100 */
        return SERVO_180_CCR_MIN +
               ((uint32_t)angle * (SERVO_180_CCR_MAX - SERVO_180_CCR_MIN))
               / (uint32_t)SERVO_MODE_180;
    }
}

/**
 * @brief 수신된 한 줄을 파싱하여 명령 실행.
 *
 * 지원 명령:
 *   SERVO <0~max>        각도 설정
 *   SERVO MODE <90|180>  범위 모드 변경
 *   STATUS               현재 상태 출력
 *   HELP                 도움말
 */
static void Servo_ParseLine(char *line)
{
    /* 앞뒤 공백 제거 (간단 구현) */
    while (*line == ' ') line++;

    /* ─ SERVO ─ */
    if (strncmp(line, "SERVO", 5) == 0) {
        char *arg = line + 5;
        while (*arg == ' ') arg++;

        /* SERVO MODE <90|180> */
        if (strncmp(arg, "MODE", 4) == 0) {
            char *val = arg + 4;
            while (*val == ' ') val++;
            int m = atoi(val);
            if (m == 90) {
                Servo_SetMode(SERVO_MODE_90);
            } else if (m == 180) {
                Servo_SetMode(SERVO_MODE_180);
            } else {
                RS232_SendString("[ERR] MODE must be 90 or 180\r\n");
            }
            return;
        }

        /* SERVO <angle> */
        if (*arg >= '0' && *arg <= '9') {
            int angle = atoi(arg);
            if (angle < 0 || angle > 255) {
                RS232_SendString("[ERR] Invalid angle\r\n");
            } else {
                Servo_SetAngle((uint8_t)angle);
            }
            return;
        }

        RS232_SendString("[ERR] Usage: SERVO <angle> | SERVO MODE <90|180>\r\n");
        return;
    }

    /* ─ STATUS ─ */
    if (strncmp(line, "STATUS", 6) == 0) {
        Servo_SendStatus();
        return;
    }

    /* ─ HELP ─ */
    if (strncmp(line, "HELP", 4) == 0) {
        Servo_SendHelp();
        return;
    }

    /* 알 수 없는 명령 */
    snprintf(s_tx_buf, sizeof(s_tx_buf),
             "[ERR] Unknown command: '%s'. Type HELP\r\n", line);
    RS232_SendString(s_tx_buf);
}

static void Servo_SendStatus(void)
{
    uint32_t us = s_servo.ccr * 20UL;
    snprintf(s_tx_buf, sizeof(s_tx_buf),
             "[STATUS] Mode=%u-deg | Angle=%u | CCR=%lu | Pulse=%lu us\r\n",
             (uint8_t)s_servo.mode, s_servo.angle, s_servo.ccr, us);
    RS232_SendString(s_tx_buf);
}

static void Servo_SendHelp(void)
{
    RS232_SendString(
        "Commands:\r\n"
        "  SERVO <angle>        Set angle (0~mode_max)\r\n"
        "  SERVO MODE <90|180>  Switch range mode\r\n"
        "  STATUS               Print current state\r\n"
        "  HELP                 This message\r\n"
    );
}
