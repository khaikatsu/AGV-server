#include "BatteryModule.h"
#include "core/ControlLogic.h"
#include <cmath>

void BatteryModule::begin() {
    pinMode(Config::BATTERY_ADC_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(Config::BATTERY_ADC_PIN, ADC_11db);
    lastSampleMs_ = millis() - Config::BATTERY_SAMPLE_INTERVAL_MS;
}
void BatteryModule::update() {
    if (static_cast<uint32_t>(millis() - lastSampleMs_) < Config::BATTERY_SAMPLE_INTERVAL_MS) return;
    lastSampleMs_ = millis();
    adcMv_ = analogReadMilliVolts(Config::BATTERY_ADC_PIN);
    voltage_ = AGVMath::batteryVoltage(adcMv_, Config::BATTERY_VOLTAGE_SCALE, Config::BATTERY_VOLTAGE_OFFSET);
    percentage_ = AGVMath::batteryPercentage(voltage_, Config::BATTERY_EMPTY_VOLTAGE, Config::BATTERY_FULL_VOLTAGE);
}
bool BatteryModule::calibrated() const { return std::isfinite(voltage_); }
