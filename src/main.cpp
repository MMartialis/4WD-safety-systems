// main.cpp

#include <Arduino.h>
// #include "status.h"

#include <mcp_can.h>
#include "vesc_can_bus_arduino.h"

#define pwm_micro_min 1108
#define pwm_micro_max 1880
#define pwm_micro_median 1500

float pwm_multiplier_pos = pow((pwm_micro_max - pwm_micro_median), -1);
float pwm_multiplier_neg = pow((pwm_micro_median - pwm_micro_min), -1);

#define PWM_PIN GPIO_NUM_2

//CANBUS

CAN can;             // get torque sensor data, throttle for now

bool print_realtime_data;
long last_print_data;





// using namespace VehicleStatus;

unsigned long last_time = micros();
float lastvalue = 0;
void pwm_interrupt()
{
  unsigned long time = micros();
  lastvalue==0 ? lastvalue = 1 : lastvalue = 1;
  // if ((time - last_time) < 3000)
  // {
  //   // int number = time - last_time - pwm_micro_median;
  //   // if (abs(number) < 0.002)
  //   // {
  //   //   number = 0;
  //   // }
  //   // else if (number > 0)
  //   // {
  //   //   lastvalue = float(number) * pwm_multiplier_pos;
  //   // }
  //   // else
  //   // {
  //   //   lastvalue = float(number) * pwm_multiplier_neg;
  //   // }
  // }
  last_time = time;
}

void setup()
{
  // Create a Status variable
  // Status status;

  // // Set a new value for the status
  // status.set_status(STATUS_IDLE);

  // setup the serial port
  Serial.begin(115200);
  // wait for the serial port to open
  while (!Serial)
  {
    // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(CAN0_INT, INPUT);
  can.initialize();

  pinMode(PWM_PIN, INPUT);
  // attach the interrupt to the pin
  Serial.println("Attaching interrupt");
  //attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwm_interrupt, CHANGE);
  Serial.println("Setup complete");
}

void loop()
{
  //Serial.println(lastvalue, 3);
  //delay(200);
    can.spin();

    //can.vesc_set_current(2); //2 amps of current

    if (print_realtime_data == true)
    {
        if (millis() - last_print_data > 200)
        {
            Serial.print(can.erpm); 
            Serial.print(can.inpVoltage);
            Serial.print(can.dutyCycleNow);
            Serial.print(can.avgInputCurrent);
            Serial.print(can.avgMotorCurrent);
            Serial.print(can.tempFET);
            Serial.print(can.tempMotor);

            last_print_data = millis();
        }
    }
}