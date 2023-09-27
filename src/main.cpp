// main.cpp

#include <Arduino.h>
#include "status.h"

using namespace VehicleStatus;

void setup() {
    // Create a Status variable
    Status status;

    // Set a new value for the status
    status.set_status(STATUS_IDLE);
}

void loop() {
    // Your main loop code here
}