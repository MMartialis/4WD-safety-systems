// vesc.cpp

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
#include <stdint.h>
#else
#include <cstdint>
#endif
#include <Arduino.h>
#include <SPI.h>
#include <algorithm>

#include "mcp_can.h"
#include "vesc.hpp"

extern MCP_CAN CAN0;
extern TaskHandle_t Handler0;
extern uint8_t msgCount;

long unsigned int rxId;
uint8_t len = 0;
uint8_t rxBuf[8];
char msgBuffer[RX_MSG_BUFFER_LEN][11]; // [command id, rxId, len, data1, data2, data3, data4, data5, data6, data7, data8]

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
        const char message[11] = {
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
            rxBuf[7]};
        std::copy(message, message + 11, msgBuffer[++msgCount % RX_MSG_BUFFER_LEN]);
    }
}
