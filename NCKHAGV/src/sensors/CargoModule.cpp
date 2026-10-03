#include "CargoModule.h"

void CargoModule::begin() {
    pinMode(Config::CARGO_IR_PIN, INPUT);
    raw_ = digitalRead(Config::CARGO_IR_PIN) == Config::CARGO_IR_ACTIVE_STATE;
    changedAtMs_ = millis();
    stable_ = false;
}
void CargoModule::update(uint32_t now) {
    const bool sample = digitalRead(Config::CARGO_IR_PIN) == Config::CARGO_IR_ACTIVE_STATE;
    if (sample != raw_) { raw_ = sample; changedAtMs_ = now; stable_ = false; }
    if (static_cast<uint32_t>(now - changedAtMs_) >= Config::CARGO_IR_STABLE_MS) {
        value_ = raw_;
        stable_ = true;
    }
}
