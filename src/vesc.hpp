//vesc.hpp

#include "../include/mcp_can.h"

#define CAN0_INT 21
#define RX_MSG_BUFFER_LEN 8

// extern MCP_CAN CAN0;
extern long unsigned int rxId;
extern unsigned char len;
extern unsigned char rxBuf[8];
extern char msgBuffer[RX_MSG_BUFFER_LEN][11];// Array to store serial string

// void print_raw_can_data(void* params);

void core_0_setup(void* params);

void put_message_in_buffer(void);