// main.cpp

#include <Arduino.h>

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
# include <stdint.h>
#else
# include <cstdint>
#endif

// #include <string>

#include <SPI.h>
#include <BluetoothSerial.h>

#include "vesc.hpp"
#include "status.hpp"
#include "pwm.hpp"
#include "can_comm.hpp"
#include "./mcp_can.h"

// can IDs
// első         53,     77
// hátsó  jobb  40, bal 13

#define ESC_FR 53
#define ESC_FL 77
#define ESC_BR 40
#define ESC_BL 13
#define BT_CAN 174

#define PWM_PIN GPIO_NUM_2

extern char msgBuffer[RX_MSG_BUFFER_LEN][12];
extern double lastPwmRead;
extern MCP_CAN CAN0;  // can controller

TaskHandle_t Handler0;  // core 0 task handler

uint8_t msgCount = 0;

// bool print_realtime_data = 1; // function is not used
long last_print_data;

VehicleStatus::Status Status; // initial status is "booting"
BluetoothSerial SerialBt;

MotorControl::Motor MotorFr(ESC_FR);
MotorControl::Motor MotorFl(ESC_FL);
MotorControl::Motor MotorBr(ESC_BR);
MotorControl::Motor MotorBl(ESC_BL);

// dictionary for motor id and motor object
std::map<uint8_t, MotorControl::Motor> motors = {
  {ESC_FR, MotorFr},
  {ESC_FL, MotorFl},
  {ESC_BR, MotorBr},
  {ESC_BL, MotorBl}
};

MotorControl::Motor get_motor_by_id(uint8_t id) {
  return motors.at(id);
}


void setup()
{
  // Create a Status variable
  for(int i = 0; i < RX_MSG_BUFFER_LEN; i++) {
    msgBuffer[i][11] = 0x01;
  }
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect

  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);


  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  SerialBt.begin("ESP32test"); // Bluetooth device name
  CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  xTaskCreatePinnedToCore(
    &core_0_setup, /* Function to implement the task */
    "setup",       /* Name of the task */
    1000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    3,             /* Priority of the task */
    &Handler0,     /* Task handle. */
    0              /* Core where the task should run */
  );



}

void loop()
{
  for (int i = msgCount%RX_MSG_BUFFER_LEN; msgBuffer[i][11] == 0x01; i--) {
    // if id in motors
    if (motors.find((byte) msgBuffer[i][1]) != motors.end()) {

      Serial.println(get_motor_by_id((byte) msgBuffer[i][1]).update_can_data(&msgBuffer[i]));
    }
  }
  delay(50);
}

