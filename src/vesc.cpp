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
struct  esc{
  uint32_t erpm;
  float current;
  float fet_temp;
  float motor_temp;
  float current_in;
  float pid_pos;
} Motor1, Motor2, Motor3, Motor4;

void update_esc_status_contol(){ //updates the esc status variables for control funcs.
  byte esc_stat = 0;  // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount%RX_MSG_BUFFER_LEN; 
  for (;esc_stat != 0x0f;)
  {
    if (msgBuffer[msgId][12] == 1){     // if the message is a read message
      switch (msgBuffer[msgId][1])
      {
      case MOTOR1:
        esc_stat |= 0b00000001;
        break;
      case MOTOR2:
        esc_stat |= 0b00000010;
        break;
      case MOTOR3:
        esc_stat |= 0b00000100;
        break;
      case MOTOR4:  
        esc_stat |= 0b00001000;
        break;
      default:
        break;
      }
      msgId --;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }
  
  
  
    if (msgBuffer[msgId][0] == 0x09)
    {
      switch (msgBuffer[msgId][1])
      {
      case MOTOR1:
        Motor1.erpm = (msgBuffer[msgId][2] << 24) 
          | (msgBuffer[msgId][3] << 16) 
          | (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]);

        Motor1.current = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /10.0);

        esc_stat = esc_stat | 0b00000001;
        break;    
      case MOTOR2:
        Motor2.erpm = (msgBuffer[msgId][2] << 24) 
          | (msgBuffer[msgId][3] << 16) 
          | (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]);

        Motor2.current = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /10.0);

        esc_stat = esc_stat | 0b00000010;
        break;
      case MOTOR3:
        Motor3.erpm = (msgBuffer[msgId][2] << 24) 
          | (msgBuffer[msgId][3] << 16) 
          | (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]);

        Motor3.current = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /10.0);

        esc_stat = esc_stat | 0b00000100;
        break;
      case MOTOR4: 
        Motor4.erpm = (msgBuffer[msgId][2] << 24) 
          | (msgBuffer[msgId][3] << 16) 
          | (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]);

        Motor4.current = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /10.0);

        esc_stat = esc_stat | 0b00001000;
        break;
      default:
        break;
      }
     
    }
    msgId --;
    if (msgId > RX_MSG_BUFFER_LEN - 1)
    {
      msgId = RX_MSG_BUFFER_LEN - 1;
    }
  }
}

//   ezek nem jok >>  if [0] == 16 command id, [1] rx id, [2-3] fet temp*10, [4-5] motor temp*10, [6-7] current in*10, [8-9] pid pos*50],
void update_esc_status_log(){
  byte esc_stat = 0;  // byte to store whether the escs are updated or not
  uint8_t msgId = msgCount%RX_MSG_BUFFER_LEN; 
  for (;esc_stat != 0x0f;)
  {
    if (msgBuffer[msgId][12] == 1){ // if the message is a read message
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
      msgId --;
      if (msgId > RX_MSG_BUFFER_LEN - 1)
      {
        msgId = RX_MSG_BUFFER_LEN - 1;
      }
      continue;
    }

    if (msgBuffer[msgId][0] == 0x16)
    {
      switch (msgBuffer[msgId][1])
      {
      case 0x35:
        Motor1.fet_temp = float((
          (msgBuffer[msgId][2] << 8) 
          | (msgBuffer[msgId][3]))
          /10.0);
        
        Motor1.motor_temp = float((
          (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]))
          /10.0);

        Motor1.current_in = float((
          (msgBuffer[msgId][6] << 8) 
          | (msgBuffer[msgId][7]))
          /10.0);
        
        Motor1.pid_pos = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /50.0);
        break;
      case 0x4D:
        Motor2.fet_temp = float((
          (msgBuffer[msgId][2] << 8) 
          | (msgBuffer[msgId][3]))
          /10.0);
        
        Motor2.motor_temp = float((
          (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]))
          /10.0);

        Motor2.current_in = float((
          (msgBuffer[msgId][6] << 8) 
          | (msgBuffer[msgId][7]))
          /10.0);
        
        Motor2.pid_pos = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /50.0);
        break;
      case 0x28:
        Motor3.fet_temp = float((
          (msgBuffer[msgId][2] << 8) 
          | (msgBuffer[msgId][3]))
          /10.0);
        
        Motor3.motor_temp = float((
          (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]))
          /10.0);

        Motor3.current_in = float((
          (msgBuffer[msgId][6] << 8) 
          | (msgBuffer[msgId][7]))
          /10.0);
        
        Motor3.pid_pos = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /50.0);
        break;
      case 0x0D:
        Motor4.fet_temp = float((
          (msgBuffer[msgId][2] << 8) 
          | (msgBuffer[msgId][3]))
          /10.0);
        
        Motor4.motor_temp = float((
          (msgBuffer[msgId][4] << 8) 
          | (msgBuffer[msgId][5]))
          /10.0);

        Motor4.current_in = float((
          (msgBuffer[msgId][6] << 8) 
          | (msgBuffer[msgId][7]))
          /10.0);
        
        Motor4.pid_pos = float((
          (msgBuffer[msgId][8] << 8) 
          | (msgBuffer[msgId][9]))
          /50.0);
        break;
      default:
        break;
      }
    }
  }
}