// main.cpp

// can IDs
// első         53,     77
// hátsó  jobb  40, bal 13

#include <Arduino.h>

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
# include <stdint.h>
#else
# include <cstdint>
#endif

// #include <string>

#include <../include/mcp_can.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include <SD.h>

#include "vesc.hpp"
#include "pwm.hpp"
#include "can_comm.hpp"
#include "sd.hpp"

#define PWM_PIN GPIO_NUM_2
#define SD_CS_PIN 17

extern char msgBuffer[RX_MSG_BUFFER_LEN][11];
extern double lastPwmRead;

TaskHandle_t Handler0;

uint8_t msgCount = 0;

bool print_realtime_data = 1;
long last_print_data;

BluetoothSerial SerialBt;

void setup()
{

  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect

  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);


  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");


  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted



  // Initialize SD card
  pinMode(SD_CS_PIN, OUTPUT);
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS_PIN)){
      Serial.println("Card failed, or not present");
      while (1);      
  }
  Serial.println("card initialized");
  // resetting undefined value floats to 0.00 for SD logging
  FillLogWithZeros();

  SD.open(findDataLogFileName(), FILE_WRITE).close();

  xTaskCreatePinnedToCore(
    &core_0_setup, /* Function to implement the task */
    "setup",       /* Name of the task */
    700,         /* Stack size in words */
    NULL,          /* Task input parameter */
    1,             /* Priority of the task */
    &Handler0,     /* Task handle. */
    0              /* Core where the task should run */
  );
}

void loop()
{
  // for (int8_t i = 0; i < RX_MSG_BUFFER_LEN; i++)
  // {
  //   if (strlen(msgString[i]) > 0)
  //   {
  //     Serial.print(msgCount);
  //     Serial.println(msgString[i]);
  //     SerialBt.write(msgCount);
  //     SerialBt.write((uint8_t *)msgString[i], strlen(msgString[i]));
  //     Serial.println("Free memory: " + String(esp_get_free_heap_size()) + " bytes");
  //     }
  //   std::fill(msgString[i], msgString[i] + 128, 0);
  // }
  LogAppendValues();
  Serial.println("appended");
  saveDataLog();
  Serial.println("saved");
  delay(20);
}