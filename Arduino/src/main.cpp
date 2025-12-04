#include <Arduino.h>
#include "motor.h"

#include "../../shared/include/shared_defs/RobotIO.h"

constexpr byte leftMDiagPin = 3;     // Enable pin for left motor
constexpr byte leftMInputA = 4;
constexpr byte leftMInputB = 5;
constexpr byte leftMPwm = 6;

constexpr byte rightMDiagPin = 7;    // Enable pin for right motor
constexpr byte rightMInputA = 8;
constexpr byte rightMInputB = 9;
constexpr byte rightMPwm = 10;

constexpr byte onBoardLED = 13;




// Motor object
auto *p_motorController = new motor(leftMDiagPin, leftMInputA, leftMInputB, leftMPwm, rightMDiagPin, rightMInputA,
                                   rightMInputB, rightMPwm);

void setup() {

    pinMode(leftMDiagPin, OUTPUT);
    pinMode(leftMInputA, OUTPUT);
    pinMode(leftMInputB, OUTPUT);
    pinMode(leftMPwm, OUTPUT);
    pinMode(rightMDiagPin, OUTPUT);
    pinMode(rightMInputA, OUTPUT);
    pinMode(rightMInputB, OUTPUT);
    pinMode(rightMPwm, OUTPUT);

    pinMode(onBoardLED, OUTPUT);

    p_motorController->enableMotors();

    Serial.begin(115200);
    p_motorController->enableMotors();

}
bool data_ready = false;
void loop() {
    shared_defs::MotorCommandUnion MotorCommand{};
    byte c;
    {
        delay(100);
        if (Serial.available() > 0 and !data_ready) {
            Serial.readBytes(&c, 1);
            if (c == SYNC_BYTE) {
                Serial.readBytes(MotorCommand.raw_data, sizeof(MotorCommand.raw_data));
                data_ready = true;
            }
        }

        if (data_ready) {
            p_motorController->setMotor(MotorCommand.motor_command.left_motor, MotorCommand.motor_command.right_motor);
            data_ready = false;
        }
    }
}