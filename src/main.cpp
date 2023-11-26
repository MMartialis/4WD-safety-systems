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

#include "soc/rtc_wdt.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

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
extern MCP_CAN CAN0;
extern BluetoothSerial SerialBt;

extern esc vescFL, vescFR, vescRL, vescRR;

TaskHandle_t Handler0;

float currentFL = 0;
float currentFR = 0;
float currentRL = 0;
float currentRR = 0;

float pwm = 0;

//---------------------------------------------------------------------------------------------

void setup() {
// TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // Unlock timer config.
// TIMERG1.wdt_feed = 1; // Reset feed count.
// TIMERG1.wdt_config0.en = 0; // Disable timer.
// TIMERG1.wdt_wprotect = 0; // Lock timer config.

// TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
// TIMERG0.wdt_feed = 1;
// TIMERG0.wdt_config0.en = 0;
// TIMERG0.wdt_wprotect = 0;
  Serial.begin(115200);
  // rtc_wdt_protect_off();
  // rtc_wdt_disable();
  gpio_install_isr_service(0); // Install the driver's GPIO ISR handler service

  //*******************************************************************************************
  // PWM setup
  // pinMode(PWM_PIN, INPUT); // pwm setup
  // attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);

  pwm_setup_gpio_interrupt(); // Set up interrupt handler for GPIO pin
  pwm_configure_gpio_interrupt(); // Configure GPIO pin for interrupt

  if (VERBOSE)
    Serial.println("PWM interrupt attached");

  //*******************************************************************************************
  // Bluetooth setup
  bt_setup();

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


}

//---------------------------------------------------------------------------------------------
void loop() {
  //*******************************************************************************************
  // Bluetooth check
  if (SerialBt.available()) {
    bt_cmd(SerialBt.readStringUntil('\n'));
  }

  pwm = get_pwm();

  if (pwm >= 0) {
    currentFL = pwm * FR_MAX_CURRENT;
    currentFR = pwm * FR_MAX_CURRENT;
    currentRL = pwm * FR_MAX_CURRENT;
    currentRR = pwm * FR_MAX_CURRENT;
  } else {
    currentFL = pwm * FR_MAX_BRAKE_CURRENT;
    currentFR = pwm * FR_MAX_BRAKE_CURRENT;
    currentRL = pwm * FR_MAX_BRAKE_CURRENT;
    currentRR = pwm * FR_MAX_BRAKE_CURRENT;
  }
  /*
   * if csúsz, apply minimum deterration to current
   */
  // disable can interrupt


  // gpio_isr_handler_remove(CAN0_INT_PIN);
  // gpio_set_intr_type(CAN0_INT_PIN, GPIO_INTR_DISABLE);
  gpio_intr_disable(CAN0_INT_PIN);
  comm_can_set_current(FL_ID, currentFL);
  comm_can_set_current(FR_ID, currentFR);
  comm_can_set_current(RL_ID, currentRL);
  comm_can_set_current(RR_ID, currentRR);
  put_message_in_buffer(NULL);
  if (VERBOSE)
    Serial.println("current set");
  
  update_esc_status_control();
  bt_log("PWM: ", pwm, " FL erpm: ", vescFL.erpm, "\n");
  // LogAppendValues();
  // if (VERBOSE)
  //   Serial.println("values logged");
  // saveDataLog();
  // if (VERBOSE)
  //   Serial.println("log saved");
}