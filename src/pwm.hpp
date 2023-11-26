// pwm.hpp

#include <Arduino.h>

#include "defs.hpp"

void pwm_configure_gpio_interrupt();

void pwm_setup_gpio_interrupt();

void IRAM_ATTR pwm_interrupt(void* arg);