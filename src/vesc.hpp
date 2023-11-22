//vesc.hpp
#include "mcp_can.h"

#include "defs.hpp"

extern MCP_CAN CAN0;

// void print_raw_can_data(void* params);

void update_esc_status_control(void);

void update_esc_status_log(void);