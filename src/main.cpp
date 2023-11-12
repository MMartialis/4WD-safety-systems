// main.cpp
// can IDs
// első 53, 77
// hátsó jobb 40, bal 13

#include <mcp_can.h>
#include <SPI.h>
#include "vesc.hpp"

#define CAN0_INT 21

TaskHandle_t fasz;
uint8_t len = 0;
uint32_t intCd = 1;

// start print job on core 0
void start_print_job()
{ 
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
  Serial.begin(250000);
  while (!Serial); // Wait for serial port to connect

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
  
  pinMode(CAN0_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN0_INT), start_print_job, FALLING);
  Serial.println("Interrupt attached");
  start_print_job();
}

void loop()
{
  // if (!digitalRead(CAN0_INT))
  // {
  //   Serial.println("Interrupt triggered");
  //   start_print_job();

  // }
  delay(1000);
  // Serial.println("Sending CAN Message...");
  // for (int i = 0; i < 20; i++)
  // {
  //   // comm_can_set_duty(77, 0.1);
  //   // Serial.println("sent setduty 10%");
  //   delay(250);
  //   // comm_can_status_1(77);
  //   // comm_can_status_2(77);
  //   // comm_can_status_3(77);
  //   // comm_can_status_4(77);
  //   // comm_can_status_5(77);
  //   // comm_can_status_6(40);
  //   // comm_can_status_6(13);

  //   Serial.println("sent status request 6");
  // }
  // comm_can_status_1(77);
  // comm_can_set_duty(77, 0.0);
  // Serial.println("sent setduty 0%");
}
