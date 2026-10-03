#include "MovementModule.h"
#include "core/ControlLogic.h"

void MovementModule::moveForward() { motor_.setMotorSpeed(Config::BASE_SPEED); motor_.motorForward(); buzzer_.off(); action_ = MotorAction::Forward; }
void MovementModule::moveBackward() {
    motor_.setMotorSpeed(Config::BASE_SPEED);
    motor_.motorBackward();
    action_ = MotorAction::Backward;
    buzzer_.setReverse(AGVMath::reverseBuzzerRequired(action_, motor_.leftDuty(), motor_.rightDuty()));
}
void MovementModule::turnLeft() { motor_.setMotorSpeed(Config::TURN_SPEED); motor_.motorLeft(); buzzer_.off(); action_ = MotorAction::PivotLeft; }
void MovementModule::turnRight() { motor_.setMotorSpeed(Config::TURN_SPEED); motor_.motorRight(); buzzer_.off(); action_ = MotorAction::PivotRight; }
void MovementModule::stop() { motor_.motorStop(); buzzer_.off(); action_ = MotorAction::Stop; }
void MovementModule::followLine() { apply(AGVMath::lineAction(line_.readSensors().pattern)); }
void MovementModule::apply(MotorAction action) {
    switch (action) {
        case MotorAction::Stop: stop(); break;
        case MotorAction::Forward: moveForward(); break;
        case MotorAction::Backward: moveBackward(); break;
        case MotorAction::PivotLeft: turnLeft(); break;
        case MotorAction::PivotRight: turnRight(); break;
        case MotorAction::CorrectLeft: line_.correctLeft(); buzzer_.off(); action_ = action; break;
        case MotorAction::CorrectRight: line_.correctRight(); buzzer_.off(); action_ = action; break;
    }
}
