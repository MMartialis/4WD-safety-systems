// defs.hpp

#ifndef DEFS_HPP
#define DEFS_HPP

#define VERBOSE 1

#define PWM_DEADZONE 15 // the deadzone for the pwm signal in microseconds
#define FL_ID 77
#define FL_MAX_CURRENT 60
#define FL_MAX_BRAKE_CURRENT 60
#define FR_ID 53
#define FR_MAX_CURRENT 60
#define FR_MAX_BRAKE_CURRENT 60
#define RL_ID 13
#define RL_MAX_CURRENT 90
#define RL_MAX_BRAKE_CURRENT 90
#define RR_ID 40
#define RR_MAX_CURRENT 90
#define RR_MAX_BRAKE_CURRENT 90

#define CAN0_INT 21
#define CAN0_CS 5
#define RX_MSG_BUFFER_LEN 8

#define PWM_PIN 2
#define PWM_MIN_INTERVAL_MICROS 1108
#define PWM_MAX_INTERVAL_MICROS 1880
#define PWM_MEDIAN_INTERVAL_MICROS 1500

#define SD_LOG_ENTRY_SIZE 32
#define SD_CS_PIN 17

#endif