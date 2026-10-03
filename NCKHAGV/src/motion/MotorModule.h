#pragma once
#include <Arduino.h>
#include "Types.h"

class MotorModule {
public:
    void begin();
    void motorForward();
    void motorBackward();
    void motorLeft();
    void motorRight();
    void motorStop();
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void setMotorSpeed(int speed);
    bool configured() const;
    int leftDuty() const { return appliedLeft_; }
    int rightDuty() const { return appliedRight_; }
private:
    void drive(int left, int right);
    void writeSide(bool left, int duty);
    int leftSpeed_ = Config::DEFAULT_MOTOR_SPEED;
    int rightSpeed_ = Config::DEFAULT_MOTOR_SPEED;
    int appliedLeft_ = 0, appliedRight_ = 0;
    bool initialized_ = false;
};
