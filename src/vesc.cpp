// vesc.cpp

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
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
// első         53,     77
// hátsó  jobb  40, bal 13
// 0x35, 0x4D, 0x28, 0x0D
//   ezek nem jok >>  if [0] == 09 command id, [1] rx id, [2-5] erpm, [6-7] current*10, [8-9] erpm, [10] isread
struct esc
{
  uint32_t erpm;
  float current;
  float fet_temp;
  float motor_temp;
  float current_in;
  float pid_pos;
} Motor1, Motor2, Motor3, Motor4;

void update_esc_status_control()
{                    // updates the esc status variables for control funcs.
  byte esc_stat = 0; // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount % RX_MSG_BUFFER_LEN;
  while (esc_stat != 0x0f)
  {
    if (msgBuffer[msgId][0] != 0x09)
    {
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }
    if (msgBuffer[msgId][12] == 1)
    { // if the message is a read message
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
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    esc *myMotor;
    switch (msgBuffer[msgId][1])
    {
    case FL_ID:
      myMotor = &Motor1;
      esc_stat = esc_stat | 0b00000001;
      break;
    case FR_ID:
      myMotor = &Motor2;
      esc_stat = esc_stat | 0b00000010;
      break;
    case RL_ID:
      myMotor = &Motor3;
      esc_stat = esc_stat | 0b00000100;
      break;
    case RR_ID:
      myMotor = &Motor4;
      esc_stat = esc_stat | 0b00001000;
      break;
    default:
      break;
    }
    msgBuffer[msgId][12] = 1; // set the message to read
    (*myMotor).erpm = (msgBuffer[msgId][2] << 24) |
                      (msgBuffer[msgId][3] << 16) |
                      (msgBuffer[msgId][4] << 8) |
                      (msgBuffer[msgId][5]);

    (*myMotor).current = float(((msgBuffer[msgId][8] << 8) | (msgBuffer[msgId][9])) / 10.0);
    msgId--;
    if (msgId > RX_MSG_BUFFER_LEN - 1)
    {
      msgId = RX_MSG_BUFFER_LEN - 1;
    }
  }
}

//   ezek nem jok >>  if [0] == 16 command id, [1] rx id, [2-3] fet temp*10, [4-5] motor temp*10, [6-7] current in*10, [8-9] pid pos*50],
void update_esc_status_log()
{
  byte esc_stat = 0; // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount % RX_MSG_BUFFER_LEN;
  while (esc_stat != 0x0f)
  {
    if (msgBuffer[msgId][0] != 0x09)
    {
      msgId--;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    if (msgBuffer[msgId][12] == 1)
    { // if the message is a read message
      switch (msgBuffer[msgId][1])
      {
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
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    if (msgBuffer[msgId][0] == 0x16)
    {
      esc *myMotor;
      switch (msgBuffer[msgId][1])
      {
      case 0x35:
        myMotor = &Motor1;
      case 0x4D:
        myMotor = &Motor2;
        break;
      case 0x28:
        myMotor = &Motor3;
        break;
      case 0x0D:
        myMotor = &Motor4;
        break;
      default:
        break;
      }
      msgBuffer[msgId][12] = 1; // set the message to read
      (*myMotor).fet_temp = float(((msgBuffer[msgId][2] << 8) | (msgBuffer[msgId][3])) / 10.0);
      (*myMotor).motor_temp = float(((msgBuffer[msgId][4] << 8) | (msgBuffer[msgId][5])) / 10.0);
      (*myMotor).current_in = float(((msgBuffer[msgId][6] << 8) | (msgBuffer[msgId][7])) / 10.0);
      (*myMotor).pid_pos = float(((msgBuffer[msgId][8] << 8) | (msgBuffer[msgId][9])) / 50.0);
      break;
    }
  }
}