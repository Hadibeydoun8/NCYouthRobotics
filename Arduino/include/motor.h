//
// Created by hadib on 12/4/2024.
//
#pragma once


#include <Arduino.h>
// Motor Directions

struct MotorSet {
    byte M1inputA;
    byte M1inputB;
    byte M2inputA;
    byte M2inputB;
};

// Set magic numbers for motor control
constexpr MotorSet FORWARD =     {HIGH, LOW, HIGH, LOW};
constexpr MotorSet BACKWARD =    {LOW, HIGH, LOW, HIGH};

constexpr MotorSet TANK_LEFT_TURN =   {LOW, HIGH, HIGH, LOW};
constexpr MotorSet TANK_RIGHT_TURN =  {HIGH, LOW, LOW, HIGH};

constexpr MotorSet HIGH_TEST =    {HIGH, HIGH, HIGH, HIGH};

class motor {

    const byte _leftMDiagPin;
    const byte _leftMInputA;
    const byte _leftMInputB;
    const byte _leftMPwm;
    const byte _rightMDiagPin;
    const byte _rightMInputA;
    const byte _rightMInputB;
    const byte _rightMPwm;


public:
    motor(const byte leftMDiagPin, const byte leftMInputA, const byte leftMInputB, const byte leftMPwm, const byte rightMDiagPin, const byte rightMInputA, const byte rightMInputB, const byte rightMPwm):
    _leftMDiagPin(leftMDiagPin), _leftMInputA(leftMInputA), _leftMInputB(leftMInputB), _leftMPwm(leftMPwm), _rightMDiagPin(rightMDiagPin), _rightMInputA(rightMInputA), _rightMInputB(rightMInputB), _rightMPwm(rightMPwm) {
        pinMode(leftMDiagPin, OUTPUT);
        pinMode(leftMInputA, OUTPUT);
        pinMode(leftMInputB, OUTPUT);
        pinMode(leftMPwm, OUTPUT);
        pinMode(rightMDiagPin, OUTPUT);
        pinMode(rightMInputA, OUTPUT);
        pinMode(rightMInputB, OUTPUT);
        pinMode(rightMPwm, OUTPUT);
    }

    void disableMotors() const {
        setMotor(FORWARD, 0, 0);
        digitalWrite(_leftMDiagPin, LOW);
        digitalWrite(_rightMDiagPin, LOW);
    }

    void enableMotors() const {
        setMotor(FORWARD, 0, 0);
        digitalWrite(_leftMDiagPin, HIGH);
        digitalWrite(_rightMDiagPin, HIGH);
    }


    void setMotor (int left_pwm, int right_pwm) const {
        MotorSet direction = FORWARD;
        if (left_pwm < 0) {
            direction = BACKWARD;
            left_pwm = -left_pwm;
        }
        if (right_pwm < 0) {
            direction = BACKWARD;
            right_pwm = -right_pwm;
        }
        setMotor(direction, left_pwm, right_pwm);
    }


    void setMotor(const MotorSet &motorSet, const int left_pwm = 155, const int right_pwm = 155) const {


        Serial.println(_rightMInputA);
        Serial.println(_rightMInputB);
        Serial.println(_leftMInputA);
        Serial.println(_leftMInputB);



        digitalWrite(_leftMInputA, motorSet.M1inputA);
        digitalWrite(_leftMInputB, motorSet.M1inputB);

        digitalWrite(_rightMInputA, motorSet.M2inputA);
        digitalWrite(_rightMInputB, motorSet.M2inputB);

        analogWrite(_leftMPwm, left_pwm);
        analogWrite(_rightMPwm, right_pwm);
    }
};
