// defs.hpp

#ifndef DEFS_HPP
#define DEFS_HPP

#define VERBOSE 1

#define MAIN_LOOP_DELAY 20000 // microseconds
#define MAIN_LOOP_DELAY_TICKS (MAIN_LOOP_DELAY * configTICK_RATE_HZ) / 1000000

#define FL_ID 53 // 0x35
#define FL_MAX_CURRENT 60
#define FL_MAX_BRAKE_CURRENT 75
#define FR_ID 77 // 0x4D
#define FR_MAX_CURRENT 60
#define FR_MAX_BRAKE_CURRENT 75
#define RL_ID 13 // 0x0D
#define RL_MAX_CURRENT 0
#define RL_MAX_BRAKE_CURRENT 0
#define RR_ID 40 // 0x28
#define RR_MAX_CURRENT 0
#define RR_MAX_BRAKE_CURRENT 0

#define CAN0_INT 21
#define CAN0_INT_PIN GPIO_NUM_21
#define CAN0_CS 5
#define RX_MSG_BUFFER_LEN 8
#define STATUS_1_COMMAND_ID 9

#define PWM_PIN 2
#define PWM_PIN_PIN GPIO_NUM_2
#define PWM_MIN_INTERVAL_MICROS 1108
#define PWM_MAX_INTERVAL_MICROS 1880
#define PWM_MEDIAN_INTERVAL_MICROS 1500
#define PWM_DEADZONE 20 // the deadzone for the pwm signal in microseconds

#define SD_LOG_ENTRY_SIZE 32
#define SD_CS_PIN 17

#endif