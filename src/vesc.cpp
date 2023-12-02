// vesc.cpp

#ifdef AVR // arduino and esp32 have different names for the same thing
#include <stdint.h>
#else
#include <cstdint>
#endif

#include <Arduino.h>
#include <SPI.h>
#include <algorithm>

#include "can_comm.hpp"
#include "mcp_can.h"
#include "vesc.hpp"

extern uint8_t msgCount;
extern char msgBuffer[RX_MSG_BUFFER_LEN][12];
volatile boolean newMsg = false;

// can IDs
// első   jobb  53, bal 77
// hátsó  jobb  40, bal 13

// 0x35, 0x4D, 0x28, 0x0D
//   ezek nem jok >>  if [0] == 09 command id, [1] rx id, [2-5] erpm, [6-7]
//   current*10, [8-9] erpm, [10] isread
esc vescFL, vescFR, vescRL, vescRR;

//---------------------------------------------------------------------------------------------
// this function is ran on demand, it empties the msgBuffer, and fills the translated data into the esc structs

void update_esc_status_control() { // updates the esc status variables for
                                   // control funcs.
  byte esc_stat = 0; // byte to store whether the escs are updated or not
  uint8_t msgId = (uint8_t)(msgCount - 1) % RX_MSG_BUFFER_LEN;
  // uint8_t count = 0; // number of messages processed, stops after
  // RX_MSG_BUFFER_LEN

  for (uint8_t i = 0;
       i < RX_MSG_BUFFER_LEN && esc_stat != 0x0f /*all esc updated*/;
       i++) // main loop, until everything is updated
  {
#if VERBOSE
    // print can message
    Serial.printf("msgId: %d, CAN RX: com: %.2X id: %d len: %d data: %.2X %.2X "
                  "%.2X %.2X %.2X %.2X %.2X %.2X:",
                  msgId, msgBuffer[msgId][0], msgBuffer[msgId][1],
                  msgBuffer[msgId][2], msgBuffer[msgId][3], msgBuffer[msgId][4],
                  msgBuffer[msgId][5], msgBuffer[msgId][6], msgBuffer[msgId][7],
                  msgBuffer[msgId][8], msgBuffer[msgId][9],
                  msgBuffer[msgId][10]);
#endif
    // verifying command id
    if (msgBuffer[msgId][0] != 0x09) {
#if VERBOSE
      Serial.printf("invalid command id %d \n", msgBuffer[msgId][0]);
#endif
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1) {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    esc *myMotor;

    // verify esc canbus id
    switch (msgBuffer[msgId][1]) {
    case FL_ID:
      esc_stat |= 0b00000001;
      myMotor = &vescFL;
      break;
    case FR_ID:
      esc_stat |= 0b00000010;
      myMotor = &vescFR;
      break;
    case RL_ID:
      esc_stat |= 0b00000100;
      myMotor = &vescRL;
      break;
    case RR_ID:
      esc_stat |= 0b00001000;
      myMotor = &vescRR;
      break;
    default: // if the esc id is not valid, skip the message
#if VERBOSE
      Serial.println("invalid esc id");
#endif
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1) {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    // if the message is a read message, skip it
    if (msgBuffer[msgId][11]) {
      #if VERBOSE
        Serial.println("already read message");
      #endif
      break;
    }
    (*myMotor).last_erpm_time = (*myMotor).erpm_time;
    (*myMotor).erpm_time = esp_timer_get_time();

    // if not read message, update the esc status variables
    msgBuffer[msgId][11] = 1; // set the message to read
    (*myMotor).last_erpm = (*myMotor).erpm;
    (*myMotor).erpm = (msgBuffer[msgId][3] << 24) |
                      (msgBuffer[msgId][4] << 16) | (msgBuffer[msgId][5] << 8) |
                      (msgBuffer[msgId][6]);

    // (*myMotor).current =
    //     float(((msgBuffer[msgId][7] << 8) | (msgBuffer[msgId][8])) / 10.0);

    (*myMotor).current =
        static_cast<float>(((int16_t)(msgBuffer[0][7] << 8) | msgBuffer[0][8])) / 10.0;


    // go to the previous message
    msgId--;
    newMsg=true;
    // if msgId is out of bounds, set it to the last element
    if (msgId > RX_MSG_BUFFER_LEN - 1) {
      msgId = RX_MSG_BUFFER_LEN - 1;
    }
    #if VERBOSE
      Serial.println("SUCCESS");
    #endif
  }
}

//---------------------------------------------------------------------------------------------
// similar to update_esc_status_control, but it updates the less important data

//   ezek nem jok >>  if [0] == 16 command id, [1] rx id, [2-3] fet temp*10,
//   [4-5] motor temp*10, [6-7] current in*10, [8-9] pid pos*50],
void update_esc_status_log() {
  byte esc_stat = 0; // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount % RX_MSG_BUFFER_LEN;
  uint8_t count = 0;
  while (esc_stat != 0x0f && count <= RX_MSG_BUFFER_LEN) {
    if (msgBuffer[msgId][0] != 0x09) {
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1) {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      count++;
      continue;
    }

    if (msgBuffer[msgId][12] == 1) { // if the message is a read message
      switch (msgBuffer[msgId][1]) {
      case 0x35:
        esc_stat |= 0b00000001;
        break;
      case 0x4D:
        esc_stat |= 0b00000010;
        break;
      case 0x28:
        esc_stat |= 0b00000100;
        break;
      case 0x0D:
        esc_stat |= 0b00001000;
        break;
      default:
        break;
      }
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1) {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      count++;
      continue;
    }

    if (msgBuffer[msgId][0] == 0x16) {
      esc *myMotor;
      switch (msgBuffer[msgId][1]) {
      case 0x35:
        myMotor = &vescFL;
      case 0x4D:
        myMotor = &vescFR;
        break;
      case 0x28:
        myMotor = &vescRL;
        break;
      case 0x0D:
        myMotor = &vescRR;
        break;
      default:
        break;
      }
      msgBuffer[msgId][12] = 1; // set the message to read
      (*myMotor).fet_temp =
          float(((msgBuffer[msgId][2] << 8) | (msgBuffer[msgId][3])) / 10.0);
      (*myMotor).motor_temp =
          float(((msgBuffer[msgId][4] << 8) | (msgBuffer[msgId][5])) / 10.0);
      (*myMotor).current_in =
          float(((msgBuffer[msgId][6] << 8) | (msgBuffer[msgId][7])) / 10.0);
      (*myMotor).pid_pos =
          float(((msgBuffer[msgId][8] << 8) | (msgBuffer[msgId][9])) / 50.0);
      break;
    }
    count++;
  }
}