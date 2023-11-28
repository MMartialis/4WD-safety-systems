// bt.hpp

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "defs.hpp"

extern BluetoothSerial SerialBt;

void bt_setup();

char* timestamp();


template<typename... Args>
void bt_log(Args... args) {
    String msg = ((String(args) + ...));
#if BT_LOG && BT_LOG_TIMESTAMP
    SerialBt.printf("%s: %s", timestamp(), msg.c_str());
#elif BT_LOG
    SerialBt.printf("%s", msg.c_str());
#endif
}

void bt_cmd(String cmd);

void esp_reset();

void pwm_status();

void pwm_enable(bool enable); // set emulated pwm value, Warning: THIS WILL DISABLE THE REMOTE!
