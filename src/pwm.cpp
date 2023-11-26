// pwm.cpp

#include "pwm.hpp"
#include "soc/rtc_wdt.h"

extern bool en_pwm; // PWM read enabled or not

volatile uint8_t pwm_was_not_zero = 0;

volatile int16_t pwm_value = 0;

intr_handle_t handlePWM; // Declare the handle variable globally or in an
                         // appropriate scope

const float pwm_multiplier_pos = pow(
    (PWM_MAX_INTERVAL_MICROS - PWM_MEDIAN_INTERVAL_MICROS + PWM_DEADZONE), -1);
const float pwm_multiplier_neg = pow(
    (PWM_MEDIAN_INTERVAL_MICROS - PWM_DEADZONE - PWM_MIN_INTERVAL_MICROS), -1);

volatile unsigned long long last_time = esp_timer_get_time();

void pwm_configure_gpio_interrupt() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on rising or falling edge
  io_conf.pin_bit_mask = (1ULL << PWM_PIN_PIN); // Bitmask for the pin
  io_conf.mode = GPIO_MODE_INPUT;               // Set as input mode
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;      // Disable pull-up
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Enable pull-down
  // increase timeout to avoid watchdog trigger

  gpio_config(&io_conf);
}

void pwm_setup_gpio_interrupt() {
  gpio_isr_handler_add(PWM_PIN_PIN, pwm_interrupt,
                       NULL); // Attach the handler to the GPIO pin

  // esp_intr_alloc(GPIO_INTR_ANYEDGE, ESP_INTR_FLAG_LEVEL2, &pwm_interrupt,
  // NULL, &handlePWM);
}

void IRAM_ATTR pwm_interrupt(void *arg) {
  volatile unsigned long long time = esp_timer_get_time();
  if (time - last_time > 3000) {
    last_time = time;
    return;
  }
  pwm_value = time - last_time - PWM_MEDIAN_INTERVAL_MICROS;
  last_time = time;

  //---------------------------------------------------------------------------------------------
  // pwm old version
  //---------------------------------------------------------------------------------------------
  // if (RTCWDT_TIMEOUT_REG == 0) {
  //   RTCWDT_TIMEOUT_REG = 0x50F0D0; // 5 seconds
  // }
  // if (!en_pwm) return;
  // volatile unsigned long long time = esp_timer_get_time();
  // if ((time - last_time) <
  //     3000) // if the time between interrupts is less than 3000 microseconds,
  //     it
  //           // is probably a false interrupt
  // {
  //   volatile int number = time - last_time - PWM_MEDIAN_INTERVAL_MICROS;
  //   if (abs(number) < PWM_DEADZONE) {
  //     lastPwmRead = 0;
  //     pwm_was_not_zero = false;
  //   }
  //   else if (pwm_was_not_zero){
  //     if (number > 0) {
  //       lastPwmRead = double(number - PWM_DEADZONE) * pwm_multiplier_pos;
  //     } else {
  //       lastPwmRead = double(number + PWM_DEADZONE) * pwm_multiplier_neg;
  //     }
  //   } else {
  //     pwm_was_not_zero = true;
  //   }
  // }
  // last_time = time;
}
float get_pwm() {
  if (VERBOSE)
    Serial.println("PWM read: " + String(pwm_value) + " microseconds");

  if (abs(pwm_value) < PWM_DEADZONE) {
    pwm_was_not_zero = 0;
    return 0;
  } else {
    if (pwm_was_not_zero>2) {
      if (pwm_value > 0) {
        return float(pwm_value - PWM_DEADZONE) * pwm_multiplier_pos;
      } else {
        return float(pwm_value + PWM_DEADZONE) * pwm_multiplier_neg;
      }
    } else {
      pwm_was_not_zero++;
      return 0;
    }
  }
}
