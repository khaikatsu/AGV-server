#include "LineFollowModule.h"
#include "core/ControlLogic.h"

void LineFollowModule::begin() {
    pinMode(Config::LINE_LEFT_PIN, INPUT);
    pinMode(Config::LINE_CENTER_PIN, INPUT);
    pinMode(Config::LINE_RIGHT_PIN, INPUT);
}
LineReadings LineFollowModule::readSensors() const {
    LineReadings values;
    values.left = digitalRead(Config::LINE_LEFT_PIN);
    values.center = digitalRead(Config::LINE_CENTER_PIN);
    values.right = digitalRead(Config::LINE_RIGHT_PIN);
    values.configured = Config::LINE_SENSOR_ACTIVE_STATE == 0 || Config::LINE_SENSOR_ACTIVE_STATE == 1;
    values.pattern = AGVMath::normalizeLine(values.left, values.center, values.right, Config::LINE_SENSOR_ACTIVE_STATE);
    return values;
}
void LineFollowModule::followLine() {
    switch (AGVMath::lineAction(readSensors().pattern)) {
        case MotorAction::Forward: motor_.setMotorSpeed(Config::BASE_SPEED); motor_.motorForward(); break;
        case MotorAction::CorrectLeft: correctLeft(); break;
        case MotorAction::CorrectRight: correctRight(); break;
        default: stop(); break;
    }
}
void LineFollowModule::correctLeft() {
    motor_.setLeftSpeed(Config::CORRECTION_SPEED + Config::LEFT_SPEED_TRIM);
    motor_.setRightSpeed(0);
    motor_.motorForward();
}
void LineFollowModule::correctRight() {
    motor_.setLeftSpeed(0);
    motor_.setRightSpeed(Config::CORRECTION_SPEED + Config::RIGHT_SPEED_TRIM);
    motor_.motorForward();
}
void LineFollowModule::stop() { motor_.motorStop(); }
