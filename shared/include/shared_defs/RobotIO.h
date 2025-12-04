#pragma once

#define SYNC_BYTE 0xAA

namespace shared_defs {
    inline char sync_byte = SYNC_BYTE;

    struct __attribute__((packed)) MotorCommand {
        int32_t left_motor;
        int32_t right_motor;
    };

    union MotorCommandUnion {
        MotorCommand motor_command;
        uint8_t raw_data[sizeof(MotorCommand)];
    };

    static_assert(sizeof(MotorCommand) == 8);
}
