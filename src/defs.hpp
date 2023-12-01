// defs.hpp

#ifndef DEFS_HPP
#define DEFS_HPP

#define VERBOSE 0
#define VERBOSE_PWM 0

#define BT_LOG 1
#define BT_LOG_TIMESTAMP 1

#define MAIN_LOOP_DELAY_US 3000 // microseconds
#define MAIN_LOOP_DELAY_TICKS (MAIN_LOOP_DELAY_US * configTICK_RATE_HZ) / 1000000

#define FL_ID 53 // 0x35
#define FL_MAX_CURRENT 60
#define FL_MAX_BRAKE_CURRENT 75
#define FR_ID 77 // 0x4D
#define FR_MAX_CURRENT 60
#define FR_MAX_BRAKE_CURRENT 75
#define RL_ID 13 // 0x0D
#define RL_MAX_CURRENT 90
#define RL_MAX_BRAKE_CURRENT 90
#define RR_ID 40 // 0x28
#define RR_MAX_CURRENT 90
#define RR_MAX_BRAKE_CURRENT 90

#define CAN0_INT 21
#define CAN0_INT_PIN GPIO_NUM_21
#define CAN0_CS 5
#define RX_MSG_BUFFER_LEN 8
#define STATUS_1_COMMAND_ID 9

#define PWM_PIN 2
#define PWM_PIN_PIN GPIO_NUM_2
#define PWM_MIN_INTERVAL_MICROS 1108
#define PWM_MAX_INTERVAL_MICROS 1878
#define PWM_MEDIAN_INTERVAL_MICROS 1498
#define PWM_DEADZONE 25 // the deadzone for the pwm signal in microseconds
#define PWM_WAS_NOT_ZERO_THRESHOLD 2

#define SD_LOG_ENTRY_SIZE 32
#define SD_CS_PIN 17

#define WHEEL_RADIUS 0.095 // meters
#define WHEEL_DIAMETER WHEEL_RADIUS * 2 // meters
#define WHEEL_CIRCUMFERENCE WHEEL_DIAMETER * PI // meters
#define POLE_PAIR_COUNT 7
#define GEAR_RATIO_FRONT 3.6
#define GEAR_RATIO_REAR 5.2
#define ERPM_TO_MPS_FRONT WHEEL_CIRCUMFERENCE / (POLE_PAIR_COUNT * GEAR_RATIO_FRONT * 60)
#define ERPM_TO_MPS_REAR WHEEL_CIRCUMFERENCE / (POLE_PAIR_COUNT * GEAR_RATIO_REAR * 60)
#define AXLE_WIDTH 0.338 // meters
#define HALF_AXLE_WIDTH AXLE_WIDTH / 2 // meters

#define MAGIC_TRESHOLD 0.5 // m/s
#define IS_SLIDING_THRESHOLD_FRONT 0.2 // unit: I have no idea, please ask about this in the exam :D
#define IS_SLIDING_THRESHOLD_REAR 0.2 // unit: I have no idea
#define REAR_TRACTION_CONTROL_TIMEOUT 0 // milliseconds
#define FRONT_TRACTION_CONTROL_TIMEOUT 0 // milliseconds

#endif

