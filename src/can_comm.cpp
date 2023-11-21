#include "can_comm.hpp"

extern TaskHandle_t Handler0;
uint8_t msgCount = 0;

MCP_CAN CAN0(CAN0_CS);

long unsigned int rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

char msgBuffer[RX_MSG_BUFFER_LEN][12];

void core_0_setup(void *params)
{
    pinMode(CAN0_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAN0_INT), put_message_in_buffer, FALLING);
    // Serial.println("Interrupt attached");
    vTaskDelete(Handler0);
}

void put_message_in_buffer()
{
  for (; CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_NOMSG;)
  {
    const char message[12] = {
        (byte) (rxId >> 8),
        (byte) rxId,
        (byte) len,
        rxBuf[0],
        rxBuf[1],
        rxBuf[2],
        rxBuf[3],
        rxBuf[4],
        rxBuf[5],
        rxBuf[6],
        rxBuf[7],
        0x00
    };
    std::copy(message, message + 12, msgBuffer[msgCount%RX_MSG_BUFFER_LEN]);
    msgCount++;
  }
}

// Implementation for sending extended ID CAN-frames
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len, uint8_t rtr)
{
    CAN0.sendMsgBuf((unsigned long)id, (byte)1, (byte)rtr, (byte)len, (byte * ) data);
}

void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_DUTY << 8),
                     buffer, send_index, 0);
}

void comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                     buffer, send_index, 0);
}

void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay)
{
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    buffer_append_float16(buffer, off_delay, 1e3, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                     buffer, send_index, 0);
}

void comm_can_set_current_brake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
                     buffer, send_index, 0);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_RPM << 8),
                     buffer, send_index, 0);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_POS << 8),
                     buffer, send_index, 0);
}

void comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                     buffer, send_index, 0);
}

/**
 * Same as above, but also sets the off delay. Note that this command uses 6 bytes now. The off delay is useful to set to keep the current controller running for a while even after setting currents below the minimum current.
 */
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay)
{
    int32_t send_index = 0;
    uint8_t buffer[6];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    buffer_append_float16(buffer, off_delay, 1e3, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                     buffer, send_index, 0);
}

void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8),
                     buffer, send_index, 0);
}

void comm_can_set_handbrake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1e3, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8),
                     buffer, send_index, 0);
}

void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    can_transmit_eid(controller_id |
                         ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8),
                     buffer, send_index, 0);
}