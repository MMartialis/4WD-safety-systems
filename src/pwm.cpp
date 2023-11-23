// pwm.cpp

#include "pwm.hpp"

const float pwm_multiplier_pos = pow((PWM_MAX_INTERVAL_MICROS - PWM_MEDIAN_INTERVAL_MICROS), -1);
const float pwm_multiplier_neg = pow((PWM_MEDIAN_INTERVAL_MICROS - PWM_MIN_INTERVAL_MICROS), -1);


unsigned long last_time = micros();
double lastPwmRead = 0;
void pwm_interrupt()
{
  unsigned long time = micros();
  if ((time - last_time) < 3000)
  {
    int number = time - last_time - PWM_MEDIAN_INTERVAL_MICROS;
    if (abs(number) < 0.002)
    {
      number = 0;
    }
    else if (number > 0)
    {
      lastPwmRead = double(number) * pwm_multiplier_pos;
    }
    else
    {
      lastPwmRead = double(number) * pwm_multiplier_neg;
    }
  }
  last_time = time;
}