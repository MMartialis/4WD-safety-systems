// main.cpp

#include <Arduino.h>
// #include <BluetoothSerial.h>
#include <SPI.h>
#include <SD.h>
#include <cstring>
// #include <ESP32TimerInterrupt.h>

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
#include <stdint.h>
#else
#include <cstdint>
#endif

// #include <string>

#include "defs.hpp"

#include "vesc.hpp"
#include "pwm.hpp"
#include "can_comm.hpp"
#include "bt.hpp"
#include "sd.hpp"
#include "./mcp_can.h"

extern char msgBuffer[RX_MSG_BUFFER_LEN][12];
extern double lastPwmRead;
extern MCP_CAN CAN0;

TaskHandle_t Handler0;

float currentFL = 0;
float currentFR = 0;
float currentRL = 0;
float currentRR = 0;

void setup()
{
  // pinMode(SD_CS_PIN, OUTPUT);
  // pinMode(CAN0_CS, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial port to connect

  pinMode(PWM_PIN, INPUT); // pwm setup
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);
  if (VERBOSE)
    Serial.println("PWM interrupt attached");

  // Initialize SD card
  // digitalWrite(SD_CS_PIN, LOW);

  // if (VERBOSE) Serial.println("SD_CS_PIN LOW");
  if (VERBOSE)
    Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("Card failed, or not present");
    // while (1)
    //   ; // don't do anything more
  }
  if (VERBOSE)
    Serial.println("card initialized");
  // resetting undefined value floats to 0.00 for SD logging
  SD.open(findDataLogFileName(), FILE_WRITE).close();
  if (VERBOSE)
    Serial.println("log file created");
  // digitalWrite(SD_CS_PIN, HIGH);
  if (VERBOSE)
    Serial.println("CS pin high");
  FillLogWithZeros();

  // Init MCP2515
  // digitalWrite(CAN0_CS, LOW);
  delay(10);
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
    if (VERBOSE)
      Serial.println("MCP2515 Initialized Successfully!");
    CAN0.setMode(MCP_NORMAL);
  }
  else
  {
    if (VERBOSE)
      Serial.println("Error Initializing MCP2515...");
  }
  // digitalWrite(CAN0_CS, HIGH);
  if (VERBOSE)
    Serial.println("MCP2515 Normal Mode Activated!");

  xTaskCreatePinnedToCore(
      &core_0_setup, /* Function to implement the task */
      "setup",       /* Name of the task */
      700,           /* Stack size in words */
      NULL,          /* Task input parameter */
      1,             /* Priority of the task */
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
    currentRL = lastPwmRead * FR_MAX_CURRENT;
    currentRR = lastPwmRead * FR_MAX_CURRENT;
  }
  else
  {
    currentFL = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentFR = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentRL = lastPwmRead * FR_MAX_BRAKE_CURRENT;
    currentRR = lastPwmRead * FR_MAX_BRAKE_CURRENT;
  }
  /*
   * if csúsz, apply minimum deterration to current
   */
  comm_can_set_current(FL_ID, currentFL);
  comm_can_set_current(FR_ID, currentFR);
  comm_can_set_current(RL_ID, currentRL);
  comm_can_set_current(RR_ID, currentRR);
  delay(2);
  if (VERBOSE)
    Serial.println("current set");
  LogAppendValues();
  if (VERBOSE)
    Serial.println("values logged");
  saveDataLog();
  if (VERBOSE)
    Serial.println("log saved");
  delay(200);
}