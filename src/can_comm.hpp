#include "mcp_can.h"

#include "defs.hpp"

extern MCP_CAN CAN0;

void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);

typedef enum
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_SET_POS = 4,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;


void core_0_setup(void* params);

void put_message_in_buffer(void);

void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);

void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);

void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);

void comm_can_set_duty(uint8_t controller_id, float duty);

void comm_can_set_current(uint8_t controller_id, float current);

void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);

void comm_can_set_current_brake(uint8_t controller_id, float current);

void comm_can_set_rpm(uint8_t controller_id, float rpm);

void comm_can_set_pos(uint8_t controller_id, float pos);

void comm_can_set_current_rel(uint8_t controller_id, float current_rel);

void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);

void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);

void comm_can_set_handbrake(uint8_t controller_id, float current);

void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);