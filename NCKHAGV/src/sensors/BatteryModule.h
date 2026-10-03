#pragma once
#include <Arduino.h>
#include "Types.h"

class BatteryModule {
public:
    void begin();
    void update();
    float readVoltage() const { return voltage_; }
    float getBatteryPercentage() const { return percentage_; }
    uint32_t adcMillivolts() const { return adcMv_; }
    bool calibrated() const;
private:
    uint32_t adcMv_ = 0, lastSampleMs_ = 0;
    float voltage_ = UNKNOWN_VALUE, percentage_ = UNKNOWN_VALUE;
};
