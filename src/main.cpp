// main.cpp

#include <mcp_can.h>
#include <SPI.h>
#include "vesc.hpp"

#define CAN0_INT 19

TaskHandle_t fasz;
uint8_t len = 8;

// start print job on core 0
void start_print_job()
{
  portDISABLE_INTERRUPTS();
  Serial.print("a");
  xTaskCreatePinnedToCore(
    &print_raw_can_data,  /* Function to implement the task */
    "print_raw_can_data", /* Name of the task */
    10000,                 /* Stack size in words */
    NULL,                 /* Task input parameter */
    0,                    /* Priority of the task */
    &fasz,                /* Task handle. */
    0                     /* Core where the task should run */
  );                     
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to connect

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  attachInterrupt(digitalPinToInterrupt(CAN0_INT), start_print_job, FALLING);
  portDISABLE_INTERRUPTS();
}

void loop()
{
  delay(5000);
  Serial.println("Sending CAN Message...");
  for (int i = 0; i < 20; i++)
  {
    comm_can_set_duty(77, 0.2);
    comm_can_set_duty(53, 0.2);
    Serial.println("sent");
    delay(250);
    comm_can_status_1(77);
    portENABLE_INTERRUPTS();
  }
  comm_can_set_duty(77, 0.0);
  comm_can_set_duty(53, 0.0);
  comm_can_status_1(77);
  Serial.println("stopped");
}
