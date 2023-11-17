// vesc.cpp

#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
#include <stdint.h>
#else
#include <cstdint>
#endif
#include <Arduino.h>
#include <SPI.h>
#include <algorithm>

#include "mcp_can.h"
#include "vesc.hpp"

extern MCP_CAN CAN0;
extern TaskHandle_t Handler0;
extern uint8_t msgCount;

long unsigned int rxId;
uint8_t len = 0;
uint8_t rxBuf[8];
char msgBuffer[RX_MSG_BUFFER_LEN][12]; // [command id, rxId, len, data1, data2, data3, data4, data5, data6, data7, data8, isRead]

void core_0_setup(void *params)
{
    pinMode(CAN0_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAN0_INT), put_message_in_buffer, FALLING);
    Serial.println("Interrupt attached");
    vTaskDelete(Handler0);
}

void put_message_in_buffer()
{
    for (; CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_NOMSG;)
    {
        const char message[12] = {
            (byte) (rxId >> 8),
            (byte)rxId,
            (byte)len,
            rxBuf[0],
            rxBuf[1],
            rxBuf[2],
            rxBuf[3],
            rxBuf[4],
            rxBuf[5],
            rxBuf[6],
            rxBuf[7],
            0x01}; //0x01 not read, 0x00 read
        std::copy(message, message + 12, msgBuffer[msgCount % RX_MSG_BUFFER_LEN]);
        msgCount++;
    }
}

namespace MotorControl {

    // Constructor
    Motor::Motor(uint8_t id) {
        this->id = id;
    }

    byte Motor::get_id(void) {
        return this->id;
    }

    float Motor::get_dutyCycle(void) {
        return this->dutyCycle;
    }

    float Motor::get_Current(void) {
        return this->Current;
    }

    long Motor::get_erpm(void) {
        return this->erpm;
    }

    float Motor::process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2) {
        char receivedByte[4], *p;
        sprintf(receivedByte, "%02X%02X", byte1, byte2);
        float output = (short) strtol(receivedByte, NULL, 16);

        switch (datatype) {
            case 'D': output *= 0.001; break; //dutyCycleNow
            case 'C': output *= 0.1; break; //avgMotorCurrent
            case 'F': output *= 0.1; break; //tempFET
            case 'T': output *= 0.1; break; //tempMotor
            case 'I': output *= 0.1; break; //avgInputCurrent
            case 'V': output *= 0.1; break; //inpVoltage
        }
        return output;
    }

    char* Motor::update_can_data(char (*messageBuffer)[12]){
        Serial.println("update_can_data");
        char* result = new char[255];
        const byte rxId = (byte) (*messageBuffer)[1];
        if (rxId != this->id) {
            sprintf(result, "ID mismatch: %d != %d\n", rxId, this->id);
            return result;
        }
        if ((*messageBuffer)[11] == 0x00) {
            sprintf(result, "Read: %d\n", rxId);
            return result;
        }
        messageBuffer[0][11] = 0x00;
        byte cmdId = (byte) (*messageBuffer)[0];
        byte len = (byte) (*messageBuffer)[2];
        char data[8];
        for(int i = 0; i < 8; i++) {
            data[i] = (*messageBuffer)[i+3];
        }
        if (cmdId == 9) { // CAN_PACKET_STATUS: ERPM, Current, Duty Cycle
            this->dutyCycle = process_data_frame_vesc('D', rxBuf[6], rxBuf[7]);
            this->Current = process_data_frame_vesc('C', rxBuf[4], rxBuf[5]);
            unsigned char erpmvals[4] = {rxBuf[3], rxBuf[2], rxBuf[1], rxBuf[0]};
            this->erpm = *(long *)erpmvals;
            sprintf(result,"Id: %2d, Duty cycle: %3f, Current: %2d, Erpm: %3d\n", this->id, this->dutyCycle, this->Current, this->erpm);
        }
        else if (cmdId == 14) { // CAN_PACKET_STATUS_4: Temp Fet, Temp Motor, Current In, PID position
            this->tempFET = process_data_frame_vesc('F', rxBuf[0], rxBuf[1]);
            this->tempMotor = process_data_frame_vesc('T', rxBuf[2], rxBuf[3]);
            this->currentIn = process_data_frame_vesc('I', rxBuf[4], rxBuf[5]);
            sprintf(result,"Id: %2d, Temp Fet: %3f, Temp Motor: %3f, Current In: %3f\n", this->id, this->tempFET, this->tempMotor, this->currentIn);
        }
        else{
            sprintf(result, "Invalid command id: %3d\n", cmdId);
        }

        return result;
    }

}  // namespace MotorControl
