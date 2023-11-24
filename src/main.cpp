// main.cpp

//---------------------------------------------------------------------------------------------
// big includes, that are part of the framework

#include <Arduino.h>
#include <BluetoothSerial.h>
// #include <SD.h>
#include <SPI.h>
#include <cstring>
// #include <ESP32TimerInterrupt.h>

#ifdef AVR // arduino and esp32 have different names for the same thing
#include <stdint.h>
#else
#include <cstdint>
#endif

// #include <string>

//---------------------------------------------------------------------------------------------
// small includes, that we wrote
#include "defs.hpp"

#include "./mcp_can.h"
#include "can_comm.hpp"
#include "vesc.hpp"
#include "pwm.hpp"
#include "bt.hpp"
// #include "sd.hpp"

//---------------------------------------------------------------------------------------------
// global variables

extern char msgBuffer[RX_MSG_BUFFER_LEN][12];
// extern double lastPwmRead;
extern MCP_CAN CAN0;
extern BluetoothSerial SerialBt;

extern esc vescFL;
extern esc vescFR;
extern esc vescRL;
extern esc vescRR;

TaskHandle_t Handler0;

float lastPWM = 0;

float currentFL = 0;
float currentFR = 0;
float currentRL = 0;
float currentRR = 0;

//---------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  //*******************************************************************************************

  ///*  Debug code, to make sure, canbus and sd are compatible  *///
  // TODO: remove this
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(CAN0_CS, OUTPUT);
  digitalWrite(CAN0_CS, HIGH);

  //*******************************************************************************************
  // PWM setup
  // pinMode(PWM_PIN, INPUT); // pwm setup
  // attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);
  gpio_config_t io_conf = {
    .pin_bit_mask = PWM_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_ANYEDGE,
  };
  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
  gpio_isr_handler_add(PWM_PIN, pwm_interrupt, (void*)PWM_PIN);

  if (VERBOSE)
    Serial.println("PWM interrupt attached");

  //*******************************************************************************************
  // Bluetooth setup
  bt_setup();

  //*******************************************************************************************
  // SD setup
  // see if the card is present and can be initialized:


  // Initialize SD card
  // if (VERBOSE)
  //   Serial.print("Initializing SD card...");
  // if (!SD.begin(SD_CS_PIN)) {
  //   Serial.println("Card failed, or not present");
  //   while (1)
  //     ; // don't do anything more
  // }
  // // if (VERBOSE)
  // //   Serial.println("card initialized");
  // // resetting undefined value floats to 0.00 for SD logging
  // SD.open(findDataLogFileName(), FILE_WRITE);
  // if (VERBOSE)
  //   Serial.println("log file created");
  // FillLogWithZeros();

  // delay(100); 

  //*******************************************************************************************
  // Init MCP2515
  void mcp2515_reset(void); // Soft Reset MCP2515
  if (VERBOSE)
    Serial.print("MCP2515 Initializing...");
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    if (VERBOSE)
      Serial.println("MCP2515 Initialized Successfully!");
    CAN0.setMode(MCP_NORMAL);
    if (VERBOSE)
      Serial.println("MCP2515 Normal Mode Activated!");
  } else {
      Serial.println("Error Initializing MCP2515, entering infinite loop");
    while (1)
      ; // don't do anything more
  }

  xTaskCreatePinnedToCore(&core_0_setup, /* Function to implement the task */
                          "core_0_setup",       /* Name of the task */
                          3000,           /* Stack size in words */
                          NULL,          /* Task input parameter */
                          1,             /* Priority of the task */
                          &Handler0,     /* Task handle. */
                          0              /* Core where the task should run */
  );

  delay(100);
  void mcp2515_reset(void); // Soft Reset MCP2515
  void *params;
  put_message_in_buffer(params);
  delay(100);
  if (VERBOSE)
    Serial.println("setup done");
}

//---------------------------------------------------------------------------------------------
void loop() {
  //*******************************************************************************************
  // Bluetooth check
  if (SerialBt.available()) {
    bt_cmd(SerialBt.readStringUntil('\n'));
  }

  lastPWM = get_pwm();
  update_esc_status_control();
  if (lastPWM >= 0 || true) {
    currentFL = lastPWM * FL_MAX_CURRENT;
    currentFR = lastPWM * FR_MAX_CURRENT;
    currentRL = lastPWM * RL_MAX_CURRENT;
    currentRR = lastPWM * RR_MAX_CURRENT;
  }
  /*
   * if csúsz, apply minimum deterration to current
   */
  if (VERBOSE)
    Serial.println(lastPWM);
  comm_can_set_current(FL_ID, currentFL);
  comm_can_set_current(FR_ID, currentFR);
  comm_can_set_current(RL_ID, currentRL);
  comm_can_set_current(RR_ID, currentRR);
  if (VERBOSE)
    Serial.println("current set");
  
  bt_log("PWM: ", lastPWM, " FL erpm: ", vescFL.erpm, " FL current: ", vescFL.current, "\n");
  // LogAppendValues();
  // if (VERBOSE)
  //   Serial.println("values logged");
  // saveDataLog();
  // if (VERBOSE)
  //   Serial.println("log saved");
  delay(200);
}