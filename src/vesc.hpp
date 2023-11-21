//vesc.hpp
#include "mcp_can.h"


#define MOTOR1 0x35
#define MOTOR2 0x4D
#define MOTOR3 0x28
#define MOTOR4 0x0D

extern MCP_CAN CAN0;

// void print_raw_can_data(void* params);

void update_esc_status_control(void);

void update_esc_status_log(void);