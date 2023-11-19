// main.cpp

#include <Arduino.h>
#include <BluetoothSerial.h>

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
#include <stdint.h>
#else
#include <cstdint>
#endif

#include "vesc.hpp"
#include "pwm.hpp"
#include "can_comm.hpp"
#include "./mcp_can.h"

#define FL_ID 77
#define FL_MAX_CURRENT 60
#define FL_MAX_BRAKE_CURRENT 60
#define FR_ID 53
#define FR_MAX_CURRENT 60
#define FR_MAX_BRAKE_CURRENT 60
#define BL_ID 13
#define BL_MAX_CURRENT 90
#define BL_MAX_BRAKE_CURRENT 90
#define BR_ID 40
#define BR_MAX_CURRENT 90
#define BR_MAX_BRAKE_CURRENT 90

#define PWM_PIN GPIO_NUM_2

extern char msgBuffer[RX_MSG_BUFFER_LEN][11];
extern double lastPwmRead;
extern MCP_CAN CAN0;

TaskHandle_t Handler0;

uint8_t msgCount = 0;

BluetoothSerial SerialBt;

float currentFL = 0;
float currentFR = 0;
float currentBL = 0;
float currentBR = 0;

void setup()
{
  // Create a Status variable

  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to connect

  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  SerialBt.begin("ESP32test"); // Bluetooth device name
  CAN0.setMode(MCP_NORMAL);    // Change to normal mode to allow messages to be transmitted

  xTaskCreatePinnedToCore(
      &core_0_setup, /* Function to implement the task */
      "setup",       /* Name of the task */
      1000,          /* Stack size in words */
      NULL,          /* Task input parameter */
      3,             /* Priority of the task */
      &Handler0,     /* Task handle. */
      0              /* Core where the task should run */
  );
}

void loop()
{
  if (lastPwmRead >= 0)
  {
    currentFL = lastPwmRead * FR_MAX_CURRENT;
    currentFR = lastPwmRead * FR_MAX_CURRENT;
    currentBL = lastPwmRead * FR_MAX_CURRENT;
    currentBR = lastPwmRead * FR_MAX_CURRENT;
  } else {
    currentFL = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentFR = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentBL = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentBR = lastPwmRead * FR_MAX_BRAKE_CURRENT;
  }
  /*
   * if csúsz, apply minimum deterration to current
   *
   *
   *
   *
   *
   *
   *
   *
   *
   *
   *
   *
   */
  comm_can_set_current(FL_ID, currentFL);
  comm_can_set_current(FR_ID, currentFR);
  comm_can_set_current(BL_ID, currentBL);
  comm_can_set_current(BR_ID, currentBR);
  delay(10);
}