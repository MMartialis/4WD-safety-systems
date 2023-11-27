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
TaskHandle_t HandlerBt;

float currentFL = 0;
float currentFR = 0;
float currentRL = 0;
float currentRR = 0;

float pwm = 0;

//---------------------------------------------------------------------------------------------

void bt_log_csv(void *params){
  while(1){
    char log[100] = "";
    sprintf(log, "%.2f, %.2d, %.2f, %.2f, %.2d, %.2f, %.2f, %.2d, %.2f, %.2f, %.2d, %.2f, %.2f\n", pwm,
    vescFL.erpm, vescFL.current, currentFL,
    vescFR.erpm, vescFR.current, currentFR,
    vescRL.erpm, vescRL.current, currentRL,
    vescRR.erpm, vescRR.current, currentRR);
    // Serial.println(log);
    bt_log((String)log);
    // bt_log(pwm,
    //  vescFL.erpm, vescFL.current, currentFL,
    //  vescFR.erpm, vescFR.current, currentFR,
    //  vescRL.erpm, vescRL.current, currentRL,
    //  vescRR.erpm, vescRR.current, currentRR,
    // "\n");
    // bt_log("PWM: ", pwm,
    // "FLe: ", vescFL.erpm, ", FLc: ", vescFL.current,", FLs: ", currentFL,
    // "FRe: ", vescFR.erpm, ", FRc: ", vescFR.current,", FRs: ", currentFR,
    // "RLe: ", vescRL.erpm, ", RLc: ", vescRL.current,", RLs: ", currentRL,
    // "RRe: ", vescRR.erpm, ", RRc: ", vescRR.current,", RRs: ", currentRR,
    // "\n");
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  gpio_install_isr_service(0); // Install the driver's GPIO ISR handler service

  //*******************************************************************************************
  // PWM setup
  // pinMode(PWM_PIN, INPUT); // pwm setup
  // attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);

  pwm_setup_gpio_interrupt(); // Set up interrupt handler for GPIO pin
  pwm_configure_gpio_interrupt(); // Configure GPIO pin for interrupt

  #ifdef VERBOSE
    Serial.println("PWM interrupt attached");
  #endif

  //*******************************************************************************************
  // Bluetooth setup
  bt_setup();

  xTaskCreatePinnedToCore(&bt_log_csv, /* Function to implement the task */
                          "bt_log_csv",       /* Name of the task */
                          3000,           /* Stack size in words */
                          NULL,          /* Task input parameter */
                          1,             /* Priority of the task */
                          &HandlerBt,     /* Task handle. */
                          0              /* Core where the task should run */
  );

  //*******************************************************************************************
  // Init MCP2515
    void mcp2515_reset(void); // Soft Reset MCP2515
  #ifdef VERBOSE
    Serial.print("MCP2515 Initializing...");
  #endif
    // CAN0.init_Mask(0, 1, 0xff00 & 0); // Init Mask
    // CAN0.init_Mask(1, 1, 0xff00 & 0); // Init Mask
    // CAN0.init_Filt(0, 1, (STATUS_1_COMMAND_ID << 8)); // Init Filter 0
    // CAN0.init_Filt(2, 1, (STATUS_1_COMMAND_ID << 8)); // Init Filter 1
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
  #ifdef VERBOSE
      Serial.println("MCP2515 Initialized Successfully!");
  #endif
    CAN0.setMode(MCP_NORMAL);
  #ifdef VERBOSE
      Serial.println("MCP2515 Normal Mode Activated!");
  #endif
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
    currentFL = pwm * FL_MAX_CURRENT;
    currentFR = pwm * FR_MAX_CURRENT;
    currentRL = pwm * RL_MAX_CURRENT;
    currentRR = pwm * RR_MAX_CURRENT;
  } else {
    currentFL = pwm * FL_MAX_BRAKE_CURRENT;
    currentFR = pwm * FR_MAX_BRAKE_CURRENT;
    currentRL = pwm * RL_MAX_BRAKE_CURRENT;
    currentRR = pwm * RR_MAX_BRAKE_CURRENT;
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
  update_esc_status_control();
  gpio_intr_enable(CAN0_INT_PIN);
  // put_message_in_buffer(NULL);
  // #ifdef VERBOSE
  //   Serial.println("current set");
  // #endif

  
  
  
   // LogAppendValues();
  // #ifdef VERBOSE
  //   Serial.println("values logged");
  // #endif
  // saveDataLog();
  // #ifdef VERBOSE
  //   Serial.println("log saved");
  // #endif
  delayMicroseconds(MAIN_LOOP_DELAY_TICKS);
}