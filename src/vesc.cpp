// vesc.cpp

#ifdef AVR // arduino and esp32 have different names for the same thing
#include <stdint.h>
#else
#include <cstdint>
#endif

#include <Arduino.h>
#include <SPI.h>
#include <algorithm>

#include "vesc.hpp"
#include "mcp_can.h"
#include "can_comm.hpp"

extern uint8_t msgCount;
extern char msgBuffer[RX_MSG_BUFFER_LEN][12];

// can IDs
// első   jobb  53, bal 77
// hátsó  jobb  40, bal 13

// 0x35, 0x4D, 0x28, 0x0D
//   ezek nem jok >>  if [0] == 09 command id, [1] rx id, [2-5] erpm, [6-7] current*10, [8-9] erpm, [10] isread

esc vescFL, vescFR, vescRL, vescRR;

void update_esc_status_control()
{
  byte esc_stat = 0; // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount % RX_MSG_BUFFER_LEN - 1;
  uint8_t count = 0;
  while (esc_stat != 0x0f && count < RX_MSG_BUFFER_LEN)
  {
    if (VERBOSE)
    {
      // print processed message
      Serial.print("CAN RX: ");
      for (int i = 0; i < 12; i++)
      {
        Serial.print(msgBuffer[msgId][i], HEX);
        Serial.print(" ");
      }
      // print esc_stat bit by bit
      for (int i = 3; i >= 0; i--)
      {
        Serial.print((esc_stat >> i) & 0x01, BIN);
      }
      Serial.print(" ");
    }
    count++;
    if (msgBuffer[msgId][0] != 0x09)
    {
      if (VERBOSE)
      {
        Serial.println("not staus 1");
      }
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }
    switch (msgBuffer[msgId][1])
    {
    case FL_ID:
      esc_stat |= 0b00000001;
      break;
    case FR_ID:
      esc_stat |= 0b00000010;
      break;
    case RL_ID:
      esc_stat |= 0b00000100;
      break;
    case RR_ID:
      esc_stat |= 0b00001000;
      break;
    default:
      break;
    }
    if (msgBuffer[msgId][11] == 1)
    { // if the message is a read message
      if (VERBOSE)
      {
        Serial.println("already read message");
      }
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    msgBuffer[msgId][11] = 1; // set the message to read
    esc *myMotor;
    switch (msgBuffer[msgId][1])
    {
    case FL_ID:
      myMotor = &vescFL;
      break;
    case FR_ID:
      myMotor = &vescFR;
      break;
    case RL_ID:
      myMotor = &vescRL;
      break;
    case RR_ID:
      myMotor = &vescRR;
      break;
    default:
      break;
    }
    (*myMotor).erpm = (msgBuffer[msgId][3] << 24) |
                      (msgBuffer[msgId][4] << 16) |
                      (msgBuffer[msgId][5] << 8) |
                      (msgBuffer[msgId][6]);

    (*myMotor).current = float(((msgBuffer[msgId][7] << 8) | (msgBuffer[msgId][8])) / 10.0);
    msgId--;
    if (msgId > RX_MSG_BUFFER_LEN - 1)
    {
      msgId = RX_MSG_BUFFER_LEN - 1;
    }
    if (VERBOSE)
    {
      Serial.print("current: ");
      Serial.print((*myMotor).current);
      Serial.print(" erpm: ");
      Serial.println((*myMotor).erpm);
    }
  }
}

//   ezek nem jok >>  if [0] == 16 command id, [1] rx id, [2-3] fet temp*10, [4-5] motor temp*10, [6-7] current in*10, [8-9] pid pos*50],
// void update_esc_status_log()
// {
//   byte esc_stat = 0; // byte to store whether the escs are updated or not
//   uint8_t msgId = msgCount % RX_MSG_BUFFER_LEN;
//   uint8_t count = 0;
//   while (esc_stat != 0x0f && count <= RX_MSG_BUFFER_LEN)
//   {
//     if (msgBuffer[msgId][0] != 0x09)
//     {
//       msgId--;
//       if (msgId > RX_MSG_BUFFER_LEN - 1)
//       {
//         msgId = RX_MSG_BUFFER_LEN - 1;
//       }
//       count++;
//       continue;
//     }

//     if (msgBuffer[msgId][11] == 1)
//     { // if the message is a read message
//       switch (msgBuffer[msgId][1])
//       {
//       case 0x35:
//         esc_stat |= 0b00000001;
//         break;
//       case 0x4D:
//         esc_stat |= 0b00000010;
//         break;
//       case 0x28:
//         esc_stat |= 0b00000100;
//         break;
//       case 0x0D:
//         esc_stat |= 0b00001000;
//         break;
//       default:
//         break;
//       }
//       msgId--;
//       if (msgId > RX_MSG_BUFFER_LEN - 1)
//       {
//         msgId = RX_MSG_BUFFER_LEN - 1;
//       }
//       count++;
//       continue;
//     }

//     if (msgBuffer[msgId][0] == 0x16)
//     {
//       esc *myMotor;
//       switch (msgBuffer[msgId][1])
//       {
//       case 0x35:
//         myMotor = &vescFL;
//       case 0x4D:
//         myMotor = &vescFR;
//         break;
//       case 0x28:
//         myMotor = &vescRL;
//         break;
//       case 0x0D:
//         myMotor = &vescRR;
//         break;
//       default:
//         break;
//       }
//       msgBuffer[msgId][12] = 1; // set the message to read
//       (*myMotor).fet_temp = float(((msgBuffer[msgId][2] << 8) | (msgBuffer[msgId][3])) / 10.0);
//       (*myMotor).motor_temp = float(((msgBuffer[msgId][4] << 8) | (msgBuffer[msgId][5])) / 10.0);
//       (*myMotor).current_in = float(((msgBuffer[msgId][6] << 8) | (msgBuffer[msgId][7])) / 10.0);
//       (*myMotor).pid_pos = float(((msgBuffer[msgId][8] << 8) | (msgBuffer[msgId][9])) / 50.0);
//       break;
//     }
//     count++;
//   }
// }