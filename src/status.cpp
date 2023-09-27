// status.cpp

#include "status.h"

namespace VehicleStatus {

    uint8_t Status::status = STATUS_BOOTING;

    uint8_t Status::get_status() {
        return status;
    }

    void Status::set_status(uint8_t new_status) {
        if (is_valid_status(new_status)) {
            status = new_status;
        } else {
            #if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
                if (Serial && Serial.availableForWrite()) {
                    Serial.println("Error: Invalid status value!");
                }
            #endif
        }
    }

    bool Status::is_valid_status(uint8_t test_status) {
        // Add any additional validation logic here if needed
        return test_status >= STATUS_BOOTING && test_status <= STATUS_TESTING;
    }

}  // namespace VehicleStatus