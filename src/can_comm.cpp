// can_comm.cpp

#include "can_comm.hpp"
#include "bt.hpp"
#include "driver/gpio.h"
#include "vesc.hpp"

#include "soc/rtc_wdt.h"


uint activeCounter = 0;

// loggign vars
extern float pwm;
extern esc vescFL, vescFR, vescRL, vescRR;
extern float currentFL, currentFR, currentRL, currentRR;

extern TaskHandle_t Handler0;
extern bool SD_ACTIVE;
volatile uint8_t msgCount = 0;

MCP_CAN CAN0(CAN0_CS);

long unsigned int rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

TaskHandle_t HandlerCAN;

intr_handle_t handleCAN; // Declare the handle variable globally or in an
                         // appropriate scope

char msgBuffer[RX_MSG_BUFFER_LEN][12];

void can_configure_gpio_interrupt() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on rising or falling edge
  io_conf.pin_bit_mask = (1ULL << CAN0_INT_PIN); // Bitmask for the pin
  io_conf.mode = GPIO_MODE_INPUT;                // Set as input mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;      // Disable pull-up
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;   // Enable pull-down
  gpio_config(&io_conf);
}

void can_setup_gpio_interrupt() {
  gpio_isr_handler_add(CAN0_INT_PIN, vTaskResume, HandlerCAN);
}

void core_0_setup(void *params) {
  can_setup_gpio_interrupt();     // Set up interrupt handler for GPIO pin
  can_configure_gpio_interrupt(); // Configure GPIO pin for interrupt
  xTaskCreatePinnedToCore(&put_message_in_buffer, 
    "CAN read task", 
    1024, 
    NULL, 
    2,
    &HandlerCAN, 
    0
  );
  vTaskDelete(NULL);
}

void IRAM_ATTR put_message_in_buffer(void *arg) {
  // activeCounter++;
  while (true) {
    if (CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_NOMSG) {
      // return;
      // }
      // while (CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_NOMSG)
      // {
      volatile const uint8_t msgId = (msgCount) % RX_MSG_BUFFER_LEN;
      msgBuffer[msgId][0] = (byte)(rxId >> 8);
      msgBuffer[msgId][1] = (byte)rxId;
      msgBuffer[msgId][2] = (byte)len;
      for (int i = 0; i < 8; ++i) {
        msgBuffer[msgId][i + 3] = rxBuf[i];
      }
      msgBuffer[msgId][11] = 0x00;
      msgCount++;
    } else {
      vTaskSuspend(NULL);
    }
  }
  
  // activeCounter--;
  // vTaskDelete(NULL);
}

void this_is_needed(void *params) {
  vTaskDelete(NULL);
}

void this_is_needed2(void *params) {
  vTaskDelete(NULL);
}

// Implementation for sending extended ID CAN-frames
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len,
                      uint8_t rtr) {
  CAN0.sendMsgBuf((unsigned long)id, (byte)1, (byte)rtr, (byte)len,
                  (byte *)data);
#if VERBOSE
  // Serial.print("CAN message sent");
  Serial.print(" txID: ");
  Serial.print(id, HEX);
  Serial.print(" Data: ");
  for (int i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif
}

void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t *buffer, float number, float scale,
                           int32_t *index) {
  buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t *buffer, float number, float scale,
                           int32_t *index) {
  buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer,
                   send_index, 0);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                   buffer, send_index, 0);
}

void comm_can_set_current_off_delay(uint8_t controller_id, float current,
                                    float off_delay) {
  int32_t send_index = 0;
  uint8_t buffer[6];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  buffer_append_float16(buffer, off_delay, 1e3, &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                   buffer, send_index, 0);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  can_transmit_eid(controller_id |
                       ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
                   buffer, send_index, 0);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer,
                   send_index, 0);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer,
                   send_index, 0);
}

void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_float32(buffer, current_rel, 1e5, &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                   buffer, send_index, 0);
}

/**
 * Same as above, but also sets the off delay. Note that this command uses 6
 * bytes now. The off delay is useful to set to keep the current controller
 * running for a while even after setting currents below the minimum current.
 */
void comm_can_set_current_rel_off_delay(uint8_t controller_id,
                                        float current_rel, float off_delay) {
  int32_t send_index = 0;
  uint8_t buffer[6];
  buffer_append_float32(buffer, current_rel, 1e5, &send_index);
  buffer_append_float16(buffer, off_delay, 1e3, &send_index);
  can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                   buffer, send_index, 0);
}

void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_float32(buffer, current_rel, 1e5, &send_index);
  can_transmit_eid(controller_id |
                       ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8),
                   buffer, send_index, 0);
}

void comm_can_set_handbrake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_float32(buffer, current, 1e3, &send_index);
  can_transmit_eid(controller_id |
                       ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8),
                   buffer, send_index, 0);
}

void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_float32(buffer, current_rel, 1e5, &send_index);
  can_transmit_eid(controller_id |
                       ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8),
                   buffer, send_index, 0);
}