// bt.hpp

#include <BluetoothSerial.h>
#include "defs.hpp"


void bt_setup();

// void machineLearning_status(); // print Machine Learning status on serial and bluetooth

// void machineLearning_set(bool on_off); // Set Machine Learning on or off (true or false)

// void machineLearning_sdbackup(int8_t backup_id); // create backup on sd card

// void machineLearning_sdrestore(int8_t backup_id); // restore from backup

// void machineLearning_btbackup(int8_t backup_id); // send raw backup data over bluetooth

// void machineLearning_btrestore(int8_t backup_i ); // restore from raw backup data

// void esp_reset();

// void ram_usage_print();

// void pwm_status();

// void pwm_status(bool on_off); // Remote input (true or false)

// void pwm_set(uint8_t emulated_pwm); // set emulated pwm value, Warning: THIS WILL DISABLE THE REMOTE!

// void current_set(uint8_t current); // current in A

// void current_set(uint8_t current1, uint8_t current2, uint8_t current3, uint8_t current4); // left front, right front, left rear, right rear

// void current_set(uint8_t current_front, uint8_t current_rear); // front, rear

// void duty_set(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4); // left front, right front, left rear, right rear

// void duty_set(uint8_t duty);

// void esc_status(); // basic status message

// void esc_status(bool extended); // extended status message (true or false)

// void esc_status_polling_rate(uint8_t esc_polling_rate); // polling rate is f/s