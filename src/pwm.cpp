// pwm.cpp

#include <Arduino.h>
#include "pwm.hpp"

#define pwm_micro_min 1108
#define pwm_micro_max 1880
#define pwm_micro_median 1500

float pwm_multiplier_pos = pow((pwm_micro_max - pwm_micro_median), -1);
float pwm_multiplier_neg = pow((pwm_micro_median - pwm_micro_min), -1);


unsigned long last_time = micros();
double lastPwmRead = 0;
void pwm_interrupt()
{
  unsigned long time = micros();
  lastPwmRead++;
  if ((time - last_time) < 3000)
  {
    int number = time - last_time - pwm_micro_median;
    if (abs(number) < 0.002)
    {
      number = 0;
    }
    else if (number > 0)
    {
      lastPwmRead = float(number) * pwm_multiplier_pos;
    }
    else
    {
      lastPwmRead = float(number) * pwm_multiplier_neg;
    }
  }
  last_time = time;
}