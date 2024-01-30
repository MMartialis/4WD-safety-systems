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
extern TaskHandle_t HandlerCAN;
extern volatile boolean newMsg;

extern float traction_conrtol_gain;

extern esc vescFL, vescFR, vescRL, vescRR;

extern bool en_pwm;

TaskHandle_t Handler0;
TaskHandle_t HandlerBt;

float currentFL = 0;
float currentFR = 0;
float currentRL = 0;
float currentRR = 0;

int64_t erpm_sum = 0;
float pwm = 0;

float if_sliding();
int8_t sliding = 0;
boolean traction_control_enabled = true;
boolean traction_control_active = false;
int32_t traction_control_front_timer = 0;
int32_t traction_control_rear_timer = 0;

//---------------------------------------------------------------------------------------------

void bt_log_csv(void *params)
{
  while (1)
  {

    if (newMsg)
    {
      newMsg = false;

      char log[100] = "";
      sprintf(log, "%.2f,\t%.2d,%.2f,\t%.2d,%.2f,\t%.2d,%.2f,\t%.2d,%.2f,\t%d\n", pwm,
              vescFL.erpm, vescFL.current,
              vescFR.erpm, vescFR.current,
              vescRL.erpm, vescRL.current,
              vescRR.erpm, vescRR.current,
              sliding);
      bt_log((String)log);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  gpio_install_isr_service(0); // Install the driver's GPIO ISR handler service

  //*******************************************************************************************
  // Bluetooth setup
  bt_setup();

  // wait for bluetooth to connect
  // while (!SerialBt.available()) {
  //   delay(100);
  // }

  //*******************************************************************************************
  // PWM setup

  pwm_setup_gpio_interrupt();     // Set up interrupt handler for GPIO pin
  pwm_configure_gpio_interrupt(); // Configure GPIO pin for interrupt

#if VERBOSE
  Serial.println("PWM interrupt attached");
#endif

  //*******************************************************************************************
  // Init MCP2515
  void mcp2515_reset(void); // Soft Reset MCP2515
#if VERBOSE
  Serial.print("MCP2515 Initializing...");
#endif
  // CAN0.init_Mask(0, 1, 0xff00 & 0); // Init Mask
  // CAN0.init_Mask(1, 1, 0xff00 & 0); // Init Mask
  // CAN0.init_Filt(0, 1, (STATUS_1_COMMAND_ID << 8)); // Init Filter 0
  // CAN0.init_Filt(2, 1, (STATUS_1_COMMAND_ID << 8)); // Init Filter 1
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
#if VERBOSE
    Serial.println("MCP2515 Initialized Successfully!");
#endif
    CAN0.setMode(MCP_NORMAL);
#if VERBOSE
    Serial.println("MCP2515 Normal Mode Activated!");
#endif
  }
  else
  {
    Serial.println("Error Initializing MCP2515, entering infinite loop");
    while (1)
      ; // don't do anything more
  }

  //*******************************************************************************************
  // CAN setup
  // this is needed dont delete
  xTaskCreatePinnedToCore(&this_is_needed, /* Function to implement the task */
                          "whyyyy",        /* Name of the task */
                          1024,            /* Stack size in words */
                          NULL,            /* Task input parameter */
                          3,               /* Priority of the task */
                          &HandlerCAN,     /* Task handle. */
                          0                /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(&core_0_setup,  /* Function to implement the task */
                          "core_0_setup", /* Name of the task */
                          2048,           /* Stack size in words */
                          NULL,           /* Task input parameter */
                          2,              /* Priority of the task */
                          &Handler0,      /* Task handle. */
                          0               /* Core where the task should run */
  );

  // can_setup_gpio_interrupt();     // Set up interrupt handler for GPIO pin
  // can_configure_gpio_interrupt(); // Configure GPIO pin for interrupt
#if VERBOSE
  Serial.println("CAN0 interrupt attached");
#endif

  //*******************************************************************************************
  // start Bluetooth logging
  xTaskCreatePinnedToCore(&bt_log_csv,  /* Function to implement the task */
                          "bt_log_csv", /* Name of the task */
                          3000,         /* Stack size in words */
                          NULL,         /* Task input parameter */
                          0,            /* Priority of the task */
                          &HandlerBt,   /* Task handle. */
                          0             /* Core where the task should run */
  );

#if VERBOSE
  Serial.println("Bluetooth logging started");
#endif
  delay(2000);
}

//---------------------------------------------------------------------------------------------
void loop()
{
  if (SerialBt.available())
  {
    bt_cmd(SerialBt.readStringUntil('\n'));
  }
  if (en_pwm)
  {
    pwm = get_pwm();

    erpm_sum = vescFL.erpm + vescFR.erpm - vescRL.erpm - vescRR.erpm;
    if ((pwm >= 0 ) == (erpm_sum >= 0))
    {
      currentFL = pwm * FL_MAX_CURRENT;
      currentFR = pwm * FR_MAX_CURRENT;
      currentRL = pwm * RL_MAX_CURRENT;
      currentRR = pwm * RR_MAX_CURRENT;
    }
    else
    {
      currentFL = pwm * FL_MAX_BRAKE_CURRENT;
      currentFR = pwm * FR_MAX_BRAKE_CURRENT;
      currentRL = pwm * RL_MAX_BRAKE_CURRENT;
      currentRR = pwm * RR_MAX_BRAKE_CURRENT;
    }
  }
  if (traction_control_enabled)
  {
    sliding = if_sliding();
  }
  gpio_intr_disable(CAN0_INT_PIN);
  update_esc_status_control();
  // if sliding
  if (sliding)
  {
//--------------------------------v1---------------------------------------------------
#if CHOOSE_TCS_DONT_VERSION == 1
    if (sliding == 1)
    { // sliding front
      comm_can_set_rpm(FL_ID, vescRL.erpm * RATIO_R2F_L);
      comm_can_set_rpm(FR_ID, vescRR.erpm * RATIO_R2F_R);
      comm_can_set_current(RL_ID, currentRL);
      comm_can_set_current(RR_ID, currentRR);
    }
    else if (sliding == -1)
    {
      comm_can_set_rpm(RL_ID, vescFL.erpm * RATIO_F2R_L);
      comm_can_set_rpm(RR_ID, vescFR.erpm * RATIO_F2R_R);
      comm_can_set_current(FL_ID, currentFL);
      comm_can_set_current(FR_ID, currentFR);
    }

//--------------------------------v2---------------------------------------------------
#else if CHOOSE_TCS_DONT_VERSION == 2
    double angular_accelaration_fl = ((vescFL.erpm - vescFL.last_erpm) / (vescFL.last_erpm_time - vescFL.erpm_time));
    double angular_accelaration_fr = ((vescFR.erpm - vescFR.last_erpm) / (vescFR.last_erpm_time - vescFR.erpm_time));
    double angular_accelaration_rl = ((vescRL.erpm - vescRL.last_erpm) / (vescRL.last_erpm_time - vescRL.erpm_time));
    double angular_accelaration_rr = ((vescRR.erpm - vescRR.last_erpm) / (vescRR.last_erpm_time - vescRR.erpm_time));
    if (sliding == 1)
    { // sliding front
      currentFL = vescFL.current + ((erpm_sum < 0) - (erpm_sum > 0)) * (angular_accelaration_rl * REFFERENCE_BETA_MULTIPLIER_FL - angular_accelaration_fl * BETA_MULTIPLIER_FL) * traction_conrtol_gain;
      currentFR = vescFR.current + ((erpm_sum < 0) - (erpm_sum > 0)) * (angular_accelaration_rr * REFFERENCE_BETA_MULTIPLIER_FR - angular_accelaration_fr * BETA_MULTIPLIER_FR) * traction_conrtol_gain;
      
    }
    else if (sliding == -1)
    {
      currentRL = vescRL.current + ((erpm_sum < 0) - (erpm_sum > 0)) * (angular_accelaration_fl * REFFERENCE_BETA_MULTIPLIER_RL - angular_accelaration_rl * BETA_MULTIPLIER_RL) * traction_conrtol_gain;
      currentRR = vescRR.current + ((erpm_sum < 0) - (erpm_sum > 0)) * (angular_accelaration_fr * REFFERENCE_BETA_MULTIPLIER_RR - angular_accelaration_rr * BETA_MULTIPLIER_RR) * traction_conrtol_gain;
    }

    comm_can_set_current(FL_ID, currentFL);
    comm_can_set_current(FR_ID, currentFR);
    comm_can_set_current(RL_ID, currentRL);
    comm_can_set_current(RR_ID, currentRR);

#endif
  }
  else
  {
    comm_can_set_current(FL_ID, currentFL);
    comm_can_set_current(FR_ID, currentFR);
    comm_can_set_current(RL_ID, currentRL);
    comm_can_set_current(RR_ID, currentRR);
  }

  gpio_intr_enable(CAN0_INT_PIN);
  vTaskResume(HandlerCAN);

  // delayMicroseconds(MAIN_LOOP_DELAY_US);
  vTaskDelay(MAIN_LOOP_DELAY_TICKS);
}

float if_sliding()
{ // 0: no sliding/both/any axle straight, 1: front sliding, -1: rear sliding

#if CHOOSE_TCS_IS_SLIDING_VERSION == 1
  //--------------------------------v1---------------------------------------------------
  double v_fl = vescFL.erpm * ERPM_TO_MPS_FRONT;
  double v_fr = vescFR.erpm * ERPM_TO_MPS_FRONT;
  double v_rl = vescRL.erpm * ERPM_TO_MPS_REAR;
  double v_rr = vescRR.erpm * ERPM_TO_MPS_REAR;

  // try catch zero devider (any axle straight)
  try
  {
    float radious_front = HALF_AXLE_WIDTH * (v_fl + v_fr) / (v_fl - v_fr);
    float radious_rear = HALF_AXLE_WIDTH * (v_rl + v_rr) / (v_rl - v_rr);

    int16_t sum_abs_v = (abs(v_fl) + abs(v_fr) + abs(v_rl) + abs(v_rr)) / 4;
    int16_t radious_sum = abs(radious_front) + abs(radious_rear);

    float sliding_ratio_front = (sum_abs_v >= MAGIC_TRESHOLD) ? (radious_rear / radious_front) / radious_sum : 0;
    float sliding_ratio_rear = (sum_abs_v >= MAGIC_TRESHOLD) ? (radious_front / radious_rear) / radious_sum : 0;

    return (sliding_ratio_front > IS_SLIDING_THRESHOLD_FRONT) ? (sliding_ratio_rear > IS_SLIDING_THRESHOLD_REAR ? 0 : -1) : (sliding_ratio_rear > IS_SLIDING_THRESHOLD_REAR ? 1 : 0);
  }
  catch (const std::exception &e)
  {
    return 0;
  }

#else if CHOOSE_TCS_IS_SLIDING_VERSION == 2
  //--------------------------------v2---------------------------------------------------
  if (pwm == 0)
  {
    return 0;
  }

  float front_rear_diff = (vescFL.erpm + vescFR.erpm) * ERPM_TO_MPS_FRONT - (vescRL.erpm - vescRR.erpm) * ERPM_TO_MPS_REAR;

  Serial.println(front_rear_diff);
  front_rear_diff = (abs(front_rear_diff) > MAX_AXLE_DIFFERENCE) ? front_rear_diff : 0;
  front_rear_diff = (front_rear_diff > 0) - (front_rear_diff < 0);

  return ((pwm >= 0) == (front_rear_diff > 0)) ? (front_rear_diff) : (-front_rear_diff);
#endif
}