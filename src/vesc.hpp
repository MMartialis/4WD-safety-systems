//vesc.hpp

#include "mcp_can.h"

#include "defs.hpp"

extern MCP_CAN CAN0;

struct esc
{
  int32_t erpm;
  int32_t last_erpm;
  int64_t erpm_time = 0;
  int64_t last_erpm_time = 0;
  float current;
  float fet_temp;
  float motor_temp;
  float current_in;
  float pid_pos;
};

// void print_raw_can_data(void* params);

void update_esc_status_control();

void update_esc_status_log();