// pwm.hpp

#include <Arduino.h>

#define PWM_PIN 2

extern double lastPwmRead;

void pwm_interrupt();