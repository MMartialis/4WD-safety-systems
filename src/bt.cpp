// #include "bt.hpp"
// #include "main.cpp"
// bool pwm_state;
// int8_t em_pwm;
// bool ml_status;
// char bt_cache;
// BluetoothSerial SerialBt;

// extern int8_t current_lf;
// extern int8_t current_rf;
// extern int8_t current_lr;
// extern int8_t current_rr;
// extern int8_t duty_lf;
// extern int8_t duty_rf;
// extern int8_t duty_lr;
// extern int8_t duty_rr;
// extern TaskHandle_t Handler1;

// BluetoothSerial SerialBt;

// void bt_setup(void* params){
//     SerialBt.begin("esp_32_awd");
//     SerialBt.println("Bluetooth setup completed");
//     Serial.println("Bluetooth setup completed");
//     vTaskDelete(Handler1);
// }

// void machinelearning_status() {
//     if (ml_status) {
//         SerialBt.println("Machine Learning is enabled");
//     } else {
//         SerialBt.println("Machine Learning is disabled");
//     }
// }

// void machineLearning_set(bool on_off){ // Set Machine Learning on or off (true or false)
//     ml_status = on_off;
//     machinelearning_status();
// } 

// void machineLearning_sdbackup(int8_t backup_id){  // create backup on sd card
//     // TODO
// }

// void machineLearning_sdrestore(int8_t backup_id){ // restore from backup
//     // TODO
// } 

// void machineLearning_btbackup(int8_t backup_id){ // send raw backup data over bluetooth
//     // TODO
// }

// void machineLearning_btrestore(int8_t backup_id){ // restore from raw backup data
//     // TODO
// }

// void esp_reset(){
//     SerialBt.println("Resetting ESP32...");
//     Serial.println("Resetting ESP32...");
//     delay(100);
//     esp_restart();
// }

// void ram_usage_print(){
//     SerialBt.printf("RAM usage: %d", esp_get_free_heap_size());
// }

// void pwm_status(){
//     if (pwm_state) {
//         SerialBt.println("Remote is enabled");
//     } else {
//         SerialBt.println("Remote is disabled");
//     }
// }

// void pwm_set(uint8_t em_pwm_in){ // set emulated pwm value, Warning: THIS WILL DISABLE THE REMOTE!
//     if(pwm_state == 0){
//         em_pwm = em_pwm_in;
//         SerialBt.printf("PWM set to: %d", em_pwm_in);
//     } 
//     else {
//         pwm_state = 0;
//         em_pwm = em_pwm_in;
//         SerialBt.printf("Remote disabled and const PWM set to: %d", em_pwm_in);
//     }
//     }

// void current_set(uint8_t current){  // sets current for all motors
//     current_lf = current;
//     current_rf = current;
//     current_lr = current;
//     current_rr = current;
//     SerialBt.printf("Current set to: %d", current);
// }


// void duty_set(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4){ // left front, right front, left rear, right rear
//     duty_lf = duty1;
//     duty_rf = duty2;
//     duty_lr = duty3;
//     duty_rr = duty4;
//     SerialBt.printf("Duty set to: %d, %d, %d, %d", duty1, duty2, duty3, duty4);
// }
// void duty_set(uint8_t duty){
//     duty_lf = duty;
//     duty_rf = duty;
//     duty_lr = duty;
//     duty_rr = duty;
//     SerialBt.printf("Duty set to: %d", duty);

// }


// // void esc_status(){
// //     SerialBt.printf();
 
// // } // basic status message

// void esc_status(bool extended); // extended status message (true or false)

// void esc_status_polling_rate(uint8_t esc_polling_rate); // polling rate is f/s