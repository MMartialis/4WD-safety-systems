//vesc.hpp

#define CAN0_INT 21
#define RX_MSG_BUFFER_LEN 8
#define MOTOR1 0x35
#define MOTOR2 0x4D
#define MOTOR3 0x28
#define MOTOR4 0x0D

extern MCP_CAN CAN0;
extern long unsigned int rxId;
extern unsigned char len;
extern unsigned char rxBuf[8];
extern char msgString[RX_MSG_BUFFER_LEN][12];// Array to store serial string

// void print_raw_can_data(void* params);

void core_0_setup(void* params);

void put_message_in_buffer(void);

void update_esc_status_control(void);

void update_esc_status_log(void);