// bt.cpp

#include "bt.hpp"

BluetoothSerial SerialBt;

bool en_pwm = true; // PWM read enabled or not
extern double pwm;
extern boolean traction_control_enabled;
extern boolean traction_control_active;
extern int8_t sliding;
// extern TaskHandle_t Handler1;

// a list of words meaning enable/on
const String enable_words[] = {"enable", "on", "true", "1"};
// a list of words meaning disable/off
const String disable_words[] = {"disable", "off", "false", "0"};

//---------------------------------------------------------------------------------------------
// evaluate bluetooth command
void bt_cmd(String cmd) {
  // parse command by spaces
  // first word is the command
  String command = cmd.substring(0, cmd.indexOf(' '));
  // switch by first word

  if (command == "r" || command == "reset") { // reset
    esp_reset();
  } else if (command == "t" || command == "tc"|| command == "traction-control") { // traction control
    // if command[1] in enable_words:
    String second_word = cmd.substring(cmd.indexOf(' ') + 1);
    if (std::find(std::begin(enable_words), std::end(enable_words), second_word) !=
        std::end(enable_words)) { // enable
      traction_control_enabled = true;
      bt_log("Traction control enabled\n");
    } else if (std::find(std::begin(disable_words), std::end(disable_words), second_word) !=
               std::end(disable_words)) { // disable
      traction_control_enabled = false;
      traction_control_active = false;
      sliding = 0;
      bt_log("Traction control disabled\n");
    }
    // if no second word, just print status
    else {
      if (traction_control_enabled) {
        bt_log("Traction control enabled\n");
      } else {
        bt_log("Traction control disabled\n");
      }
    }
  } 
  
  
  else if (command == "p" || command == "pwm") { // pwm
    // if command[1] in enable_words:
    String second_word = cmd.substring(cmd.indexOf(' ') + 1);
    if (std::find(std::begin(enable_words), std::end(enable_words), second_word) !=
        std::end(enable_words)) { // enable
      pwm_enable(true);
    } else if (std::find(std::begin(disable_words), std::end(disable_words), second_word) !=
               std::end(disable_words)) { // disable
      pwm_enable(false);
    }
    // if no second word, just print status
    else {
      pwm_status();
    }
  } else {
    bt_log("Invalid command\n");
    // list of valid commands with syntax
    bt_log("Valid commands:\n");
    bt_log("r, reset\n");
    bt_log("p, pwm [enable|disable]\n");
    bt_log("t, tc, traction-control [enable|disable]\n");
  }
}

//---------------------------------------------------------------------------------------------

void bt_setup() {
  SerialBt.begin("esp_32_awd");
  bt_log("Bluetooth setup completed\n");
  #if VERBOSE
    Serial.println("Bluetooth setup completed");
  #endif
  // vTaskDelete(Handler1);
}

// timestamp string format
char *timestamp() {
  static char timestamp[13];
  time_t now = time(nullptr);
  strftime(timestamp, 9, "%H:%M:%S", localtime(&now));
  unsigned long milliseconds = millis();
  sprintf(timestamp + 8, ".%03lu", milliseconds % 1000);
  return timestamp;
}

void esp_reset() {
  bt_log("Resetting ESP32...");
  Serial.println("Resetting ESP32...");
  esp_restart();
}

void pwm_status() {
  if (en_pwm) {
    bt_log("Remote is enabled\n");
  } else {
    bt_log("Remote is disabled\n");
  }
}

void pwm_enable(bool enable) { // set emulated pwm value, Warning: THIS WILL
                               // DISABLE THE REMOTE!
  if (enable) {
    en_pwm = true;
    gpio_intr_enable(PWM_PIN_PIN);
    bt_log("Remote enabled\n");
  } else {
    en_pwm = false;
    gpio_intr_disable(PWM_PIN_PIN);
    pwm = 0;
    bt_log("Remote disabled\n");
  }
}