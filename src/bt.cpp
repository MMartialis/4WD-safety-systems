// bt.cpp

#include "bt.hpp"

BluetoothSerial SerialBt;

bool en_pwm = true; // PWM read enabled or not
extern double lastPwmRead;
// extern TaskHandle_t Handler1;

// a list of words meaning enable/on
const String enable_words[] = {"enable", "on", "true", "1"};
// a list of words meaning disable/off
const String disable_words[] = {"disable", "off", "false", "0"};

/**
 * @brief Executes a command received over Bluetooth.
 *
 * This function takes a command as input and executes the corresponding action.
 * a viable command is a string starting with a single character, followed by
 * the command itself. Example: "p enable" will enable the pwm reading.
 *
 * @param cmd The command to be executed.
 */
void bt_cmd(String cmd)
{
  char cmdId = cmd[0];
  String command = "";
  if (cmd[1] == ' ')
  {
    String command = cmd.substring(2);
  }

  // switch by first word
  switch (cmdId)
  {
  case 'r': // reset
    esp_reset();
    break;
  case 'p': // pwm
    // if command[1] in enable_words:
    if (enable_words->indexOf(command) != -1)
    { // enable
      pwm_enable(true);
    }
    else if (disable_words->indexOf(command) != -1)
    { // disable
      pwm_enable(false);
    }
    // if no second word, just print status
    else
    {
      pwm_status();
    }
    break;
  default:
    break;
  }
}

void bt_setup()
{
  SerialBt.begin("esp_32_awd");
  bt_log("Bluetooth setup completed");
  if (VERBOSE)
    Serial.println("Bluetooth setup completed");
  // vTaskDelete(Handler1);
}

// timestamp string format
char *timestamp()
{
  static char timestamp[13];
  time_t now = time(nullptr);
  strftime(timestamp, 9, "%H:%M:%S", localtime(&now));
  unsigned long milliseconds = millis();
  sprintf(timestamp + 8, ".%03lu", milliseconds % 1000);
  return timestamp;
}

void esp_reset()
{
  bt_log("Resetting ESP32...");
  Serial.println("Resetting ESP32...");
  esp_restart();
}

void pwm_status()
{
  if (en_pwm)
  {
    bt_log("Remote is enabled");
  }
  else
  {
    bt_log("Remote is disabled");
  }
}

void pwm_enable(bool enable)
{ // set emulated pwm value, Warning: THIS WILL
  // DISABLE THE REMOTE!
  if (enable)
  {
    en_pwm = true;
    bt_log("Remote enabled");
  }
  else
  {
    en_pwm = false;
    lastPwmRead = 0;
    bt_log("Remote disabled");
  }
}