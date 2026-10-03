#pragma once
#include <Arduino.h>
#include "Config.h"

class BuzzerModule {
public:
    void begin() {
        digitalWrite(Config::BUZZER_PIN, !Config::BUZZER_ACTIVE_STATE);
        pinMode(Config::BUZZER_PIN, OUTPUT);
        off();
    }
    void setReverse(bool active) {
        digitalWrite(Config::BUZZER_PIN, active ? Config::BUZZER_ACTIVE_STATE : !Config::BUZZER_ACTIVE_STATE);
    }
    void off() { setReverse(false); }
};
