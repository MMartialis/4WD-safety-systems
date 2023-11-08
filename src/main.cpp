// main.cpp

#include <mcp_can.h>
#include <SPI.h>

#include "vesc.hpp"

#define CAN0_INT 21
MCP_CAN CAN0(5); // Set CS to pin 10

void setup()
{
  Serial.begin(115200);
  while(!Serial); // Wait for serial port to connect

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}


void loop()
{
  delay(5000);
  Serial.println("Sending CAN Message...");
  for (int i = 0; i < 20; i++)
  {
    comm_can_set_duty(77, 0.2); 
    comm_can_set_duty(53, 0.2); 
    //comm_can_set_current(77);
    delay(250);
  }
  Serial.println("sent");
  delay(5000);
  comm_can_set_duty(77, 0);
  comm_can_set_duty(53, 0);
  Serial.println("stopped");
  delay(100000000);
}
