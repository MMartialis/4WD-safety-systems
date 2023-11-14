// main.cpp
// can IDs
// első 53, 77
// hátsó jobb 40, bal 13
#include "stdint.h"
#include <mcp_can.h>
#include <SPI.h>
#include "vesc.hpp"
#include "can_comm.hpp"

extern char msgBuffer[RX_MSG_BUFFER_LEN][128];

TaskHandle_t Handler0;

uint8_t msgCount = 0;



void setup()
{
  Serial.begin(250000);
  while (!Serial)
    ; // Wait for serial port to connect

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  xTaskCreatePinnedToCore(
    &core_0_setup, /* Function to implement the task */
    "setup",       /* Name of the task */
    10000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    3,             /* Priority of the task */
    &Handler0,     /* Task handle. */
    0              /* Core where the task should run */
  );
}

void loop()
{
  for (int8_t i = 0; i < RX_MSG_BUFFER_LEN; i++)
  {
    if (strlen(msgBuffer[i]) > 0) {
      // do something with msgString[i]
    }
    std::fill(msgBuffer[i], msgBuffer[i] + 128, 0);
  }
}
