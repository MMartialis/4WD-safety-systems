// status.h

#pragma once
#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
# include <stdint.h>
#else
# include <cstdint>
#endif

namespace VehicleStatus {

    // Status enumeration
    enum StatusEnum {
        STATUS_BOOTING = 0,
        STATUS_IDLE,
        STATUS_FORWARD,
        STATUS_BACKWARD,
        STATUS_TURNING,
        STATUS_BRAKING,
        STATUS_CORRECTION,
        STATUS_ERROR,
        STATUS_TESTING
    };

    class Status {
    public:
        static uint8_t get_status();
        static void set_status(uint8_t new_status);

    private:
        static uint8_t status;
        static bool is_valid_status(uint8_t test_status);
    };

}  // namespace VehicleStatus