#pragma once
#include "LineFollowModule.h"
#include "actuators/BuzzerModule.h"

class MovementModule {
public:
    MovementModule(MotorModule& motor, LineFollowModule& line, BuzzerModule& buzzer)
        : motor_(motor), line_(line), buzzer_(buzzer) {}
    void begin() { stop(); }
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();
    void followLine();
    void apply(MotorAction action);
    MotorAction action() const { return action_; }
private:
    MotorModule& motor_;
    LineFollowModule& line_;
    BuzzerModule& buzzer_;
    MotorAction action_ = MotorAction::Stop;
};
