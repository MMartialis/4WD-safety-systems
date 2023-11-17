// vesc.hpp

#ifndef VESC_H
#define VESC_H

#define CAN0_INT 21
#define RX_MSG_BUFFER_LEN 8

extern long unsigned int rxId;
extern unsigned char len;
extern unsigned char rxBuf[8];
extern char msgBuffer[RX_MSG_BUFFER_LEN][12]; // Array to store serial string

namespace MotorControl {

    class Motor {
        private:
            uint8_t id;
            float dutyCycle;
            float Current;
            long erpm;
            float tempFET;
            float tempMotor;
            float currentIn;
            float PIDposition;

            float process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2);

        public:
            // Constructor
            Motor(uint8_t id);
            byte get_id(void);
            float get_dutyCycle(void);
            float get_Current(void);
            long get_erpm(void);
            char* update_can_data(char (*messageBuffer)[12]);
    };
}

void core_0_setup(void* params);
void put_message_in_buffer(void);

#endif // VESC_H