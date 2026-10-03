#pragma once
#include "MotorModule.h"

class LineFollowModule {
public:
    explicit LineFollowModule(MotorModule& motor) : motor_(motor) {}
    void begin();
    LineReadings readSensors() const;
    void followLine();
    void correctLeft();
    void correctRight();
    void stop();
private:
    MotorModule& motor_;
};
