#pragma once
#include <Arduino.h>
#include "Types.h"

class SensorModule {
public:
    void begin();
    void update();
    float readDistanceCm() const;
    bool isObstacleDetected() const;
    bool valid() const;
private:
    static void IRAM_ATTR echoInterrupt(void* argument);
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile bool waiting_ = false, risingSeen_ = false, ready_ = false;
    volatile uint32_t risingUs_ = 0, durationUs_ = 0;
    uint32_t triggerUs_ = 0, lastTriggerMs_ = 0, lastResultMs_ = 0;
    float distanceCm_ = UNKNOWN_VALUE;
};
