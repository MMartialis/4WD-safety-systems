//vesc.hpp

#include "mcp_can.h"

#include "defs.hpp"

extern MCP_CAN CAN0;

struct esc
{
  uint32_t erpm;
  float current;
  float fet_temp;
  float motor_temp;
  float current_in;
  float pid_pos;
};
// void print_raw_can_data(void* params);

void update_esc_status_control(void);

void update_esc_status_log(void);