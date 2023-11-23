// pwm.cpp

#include "pwm.hpp"

const float pwm_multiplier_pos =
    pow((PWM_MAX_INTERVAL_MICROS - PWM_MEDIAN_INTERVAL_MICROS), -1);
const float pwm_multiplier_neg =
    pow((PWM_MEDIAN_INTERVAL_MICROS - PWM_MIN_INTERVAL_MICROS), -1);

unsigned long last_time = micros();
double lastPwmRead = 0; // the global variable that stores the last pwm read


void pwm_interrupt() {
  unsigned long time = micros();
  if ((time - last_time) <
      3000) // if the time between interrupts is less than 3000 microseconds, it
            // is probably a false interrupt
  {
    int number = time - last_time - PWM_MEDIAN_INTERVAL_MICROS;
    if (abs(number) < PWM_DEADZONE) {
      lastPwmRead = 0;
    }
    else{
      if (number > 0) {
        lastPwmRead = double(number) * pwm_multiplier_pos;
      } else {
        lastPwmRead = double(number) * pwm_multiplier_neg;
      }
    }
  }
  last_time = time;
}