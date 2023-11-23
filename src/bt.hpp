// bt.hpp

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "defs.hpp"

extern BluetoothSerial SerialBt;

bool en_pwm = true; // PWM read enabled or not

void bt_setup();

char* timestamp();


template<typename... Args>
void bt_log(Args... args) {
    String msg = ((String(args) + ...));
    SerialBt.printf("%s: %s", timestamp(), msg.c_str());
}

void bt_cmd(String cmd);

void esp_reset();

void pwm_status();

void pwm_enable(bool enable); // set emulated pwm value, Warning: THIS WILL DISABLE THE REMOTE!
