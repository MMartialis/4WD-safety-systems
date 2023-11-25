// pwm.cpp

#include "pwm.hpp"
#include "soc/rtc_wdt.h"

extern bool en_pwm; // PWM read enabled or not

const float pwm_multiplier_pos =
    pow((PWM_MAX_INTERVAL_MICROS - PWM_MEDIAN_INTERVAL_MICROS), -1);
const float pwm_multiplier_neg =
    pow((PWM_MEDIAN_INTERVAL_MICROS - PWM_MIN_INTERVAL_MICROS), -1);

unsigned long last_time = micros();
double lastPwmRead = 0; // the global variable that stores the last pwm read

void pwm_configure_gpio_interrupt() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on rising or falling edge
    io_conf.pin_bit_mask = (1ULL << PWM_PIN_PIN); // Bitmask for the pin
    io_conf.mode = GPIO_MODE_INPUT; // Set as input mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Disable pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Enable pull-down
    // increase timeout to avoid watchdog trigger

    gpio_config(&io_conf);
}

void pwm_setup_gpio_interrupt() {
    gpio_isr_handler_add(PWM_PIN_PIN, pwm_interrupt, NULL); // Attach the handler to the GPIO pin
}



void IRAM_ATTR pwm_interrupt(void* arg) {
  if (!en_pwm) return;
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