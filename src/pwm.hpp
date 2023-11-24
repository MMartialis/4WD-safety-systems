// pwm.hpp

#include <Arduino.h>

#include "defs.hpp"

const float pwm_multiplier_pos =
    pow((PWM_MAX_INTERVAL_MICROS - PWM_MEDIAN_INTERVAL_MICROS), -1);
const float pwm_multiplier_neg =
    pow((PWM_MEDIAN_INTERVAL_MICROS - PWM_MIN_INTERVAL_MICROS), -1);

void pwm_interrupt(void *args);

float get_pwm();