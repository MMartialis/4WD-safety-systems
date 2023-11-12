// vesc.cpp

#include <stdint.h>
#include <SPI.h>
#include "mcp_can.h"
#include "vesc.hpp"

MCP_CAN CAN0(5);
long unsigned int rxId;
extern uint8_t len;
uint8_t rxBuf[8];
char msgString[128]; // Array to store serial string
extern TaskHandle_t fasz;


// Implementation for sending extended ID CAN-frames
void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len, uint8_t rtr)
{
    portDISABLE_INTERRUPTS();
    CAN0.sendMsgBuf((unsigned long) id, (byte)1, (byte)rtr, (byte)len, (byte*)data);
    portENABLE_INTERRUPTS();
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

void comm_can_status_1(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_1 << 8), buffer, 0, 1);
}

void comm_can_status_2(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_2 << 8), buffer, 0, 1);
}

void comm_can_status_3(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_3 << 8), buffer, 0, 1);
}

void comm_can_status_4(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_4 << 8), buffer, 0, 1);
}

void comm_can_status_5(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_5 << 8), buffer, 0, 1);
}

void comm_can_status_6(uint8_t controller_id)
{
    uint8_t buffer[0];
    can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_STATUS_6 << 8), buffer, 0, 1);
}

void print_raw_can_data(void* penis)
{
    Serial.println("listening");
    int8_t received = CAN0.readMsgBuf(&rxId, &len, rxBuf);

    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %d  Data:", rxId, len);
    Serial.print(msgString);
    for (byte i = 0; i < len; i++)
    {
        sprintf(msgString, " 0x%c", rxBuf[i]);
    }
    Serial.println(msgString);
    std::fill_n(msgString, 128, 0);
    vTaskDelete(fasz);
}