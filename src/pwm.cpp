// pwm.cpp

#include "pwm.hpp"

extern bool en_pwm; // PWM read enabled or not

unsigned long last_time = micros();
int16_t lastPwmRead = 0; // the global variable that stores the last pwm read

void pwm_interrupt() {
  unsigned long time = micros();
  if ((time - last_time) < 3000) {
    int number = time - last_time - PWM_MEDIAN_INTERVAL_MICROS;
    if (abs(number) > PWM_DEADZONE) {
      lastPwmRead = number;
    } else {
      lastPwmRead = 0;
    }
  } else {
    lastPwmRead = 0;
  }
  last_time = time;
}

void pwm_interrupt(void *args) { pwm_interrupt(); }

float get_pwm() {
  if (!en_pwm) {
    return 0;
  }
  if (lastPwmRead > 0) {
    return float(lastPwmRead) * pwm_multiplier_pos;
  } else {
    return float(lastPwmRead) * pwm_multiplier_neg;
  }
}