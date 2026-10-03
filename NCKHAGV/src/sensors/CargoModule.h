#pragma once
#include <Arduino.h>
#include "Config.h"

class CargoModule {
public:
    void begin();
    void update(uint32_t now);
    bool stable() const { return stable_; }
    bool present() const { return stable_ && value_; }
    bool absent() const { return stable_ && !value_; }
private:
    bool raw_ = false, value_ = false, stable_ = false;
    uint32_t changedAtMs_ = 0;
};
