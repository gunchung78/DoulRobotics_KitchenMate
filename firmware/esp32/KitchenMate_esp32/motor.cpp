#include "motor.h"
#include "driver/twai.h"

static uint16_t floatToBits(float x, float minVal, float maxVal, uint8_t bits) {
  x = constrain(x, minVal, maxVal);
  return (uint16_t)((x - minVal) / (maxVal - minVal) * (float)((1UL << bits) - 1));
}

static float bitsToFloat(uint16_t raw, float minVal, float maxVal, uint8_t bits) {
  return (float)raw / (float)((1UL << bits) - 1) * (maxVal - minVal) + minVal;
}

bool initTWAI() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("[ERR] TWAI install failed");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("[ERR] TWAI start failed");
    return false;
  }

  Serial.println("[OK] TWAI 1Mbps ready");
  return true;
}

bool sendFrame(uint32_t id, uint8_t* data, uint8_t len) {
  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));

  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  memcpy(msg.data, data, len);

  return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

void sendMIT(uint8_t id, float pos, float vel,
             float kp, float kd, float torque) {
  uint16_t p = floatToBits(pos,    P_MIN,  P_MAX,  16);
  uint16_t v = floatToBits(vel,    V_MIN,  V_MAX,  12);
  uint16_t K = floatToBits(kp,    KP_MIN, KP_MAX,  12);
  uint16_t D = floatToBits(kd,    KD_MIN, KD_MAX,  12);
  uint16_t t = floatToBits(torque, T_MIN,  T_MAX,  12);

  uint8_t data[8] = {
    (uint8_t)(p >> 8),
    (uint8_t)(p & 0xFF),
    (uint8_t)(v >> 4),
    (uint8_t)(((v & 0x0F) << 4) | (K >> 8)),
    (uint8_t)(K & 0xFF),
    (uint8_t)(D >> 4),
    (uint8_t)(((D & 0x0F) << 4) | (t >> 8)),
    (uint8_t)(t & 0xFF)
  };

  sendFrame((uint32_t)id, data, 8);
  lastCmd[id] = { pos, vel, kp, kd, torque };
}

static void sendSpecialCommand(uint8_t id, uint8_t cmd) {
  uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd };
  sendFrame((uint32_t)id, data, 8);
}

void enableAll() {
  for (int i = 0; i < 3; i++) {
    sendSpecialCommand(MOTOR_ID1, 0xFC); delay(10);
    sendSpecialCommand(MOTOR_ID2, 0xFC); delay(10);
    sendSpecialCommand(MOTOR_ID3, 0xFC); delay(10);
    delay(20);
  }
  Serial.println("[CMD] Enable ALL (x3)");
}

void disableAll() {
  sendSpecialCommand(MOTOR_ID1, 0xFD);
  sendSpecialCommand(MOTOR_ID2, 0xFD);
  sendSpecialCommand(MOTOR_ID3, 0xFD);
  Serial.println("[CMD] Disable ALL");
}


void setZeroPosition(uint8_t id) {
  if (id < 1 || id > MOTOR_COUNT) {
    Serial.println("[ZERO] invalid motor id. Use 1~3");
    return;
  }

  // MIT Set Zero command: FF FF FF FF FF FF FF FE
  uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
  sendFrame((uint32_t)id, data, 8);

  Serial.printf("[ZERO] M%d set current position as zero\n", id);
}

void repeatLastCommands() {
  for (uint8_t i = 1; i <= MOTOR_COUNT; i++) {
    const MotorCommand& c = lastCmd[i];
    sendMIT(i, c.p, c.v, c.kp, c.kd, c.t);
  }
}

void recvFeedback() {
  twai_message_t msg;

  while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) {
    if (rawDebug) {
      Serial.printf("[RX-%s] ID=0x%03lX len=%d Data:",
                    msg.extd ? "29bit" : "11bit",
                    msg.identifier,
                    msg.data_length_code);
      for (int i = 0; i < msg.data_length_code; i++) {
        Serial.printf(" %02X", msg.data[i]);
      }
      Serial.println();
    }

    if (msg.extd || msg.data_length_code < 8) continue;

    uint8_t mid = msg.data[0];
    if (mid < 1 || mid > MOTOR_COUNT) continue;

    uint16_t rawPos  = ((uint16_t)msg.data[1] << 8) | msg.data[2];
    uint16_t rawVel  = ((uint16_t)msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t rawTrq  = (((uint16_t)msg.data[4] & 0x0F) << 8) | msg.data[5];
    uint16_t rawTemp = ((uint16_t)msg.data[6] << 8) | msg.data[7];

    fb[mid].pos  = bitsToFloat(rawPos, P_MIN, P_MAX, 16);
    fb[mid].vel  = bitsToFloat(rawVel, V_MIN, V_MAX, 12);
    fb[mid].trq  = bitsToFloat(rawTrq, T_MIN, T_MAX, 12);
    fb[mid].temp = (float)rawTemp / 10.0f;

    // Normal MIT feedback uses data[6:7] as temperature in this project.
    // Actual fault read should be implemented as a separate command if needed.
    fb[mid].err = 0x00;
    fb[mid].ts  = millis();
  }
}
