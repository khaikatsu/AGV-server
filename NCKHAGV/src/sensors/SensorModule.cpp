#include "SensorModule.h"
#include <cmath>

void SensorModule::begin() {
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    pinMode(Config::ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(Config::ULTRASONIC_ECHO_PIN, INPUT);
    attachInterruptArg(Config::ULTRASONIC_ECHO_PIN, echoInterrupt, this, CHANGE);
    lastTriggerMs_ = millis() - Config::ULTRASONIC_INTERVAL_MS;
}
void IRAM_ATTR SensorModule::echoInterrupt(void* argument) {
    auto* self = static_cast<SensorModule*>(argument);
    const uint32_t now = micros();
    const bool high = digitalRead(Config::ULTRASONIC_ECHO_PIN) == HIGH;
    portENTER_CRITICAL_ISR(&self->mux_);
    if (self->waiting_) {
        if (high) { self->risingUs_ = now; self->risingSeen_ = true; }
        else if (self->risingSeen_) {
            self->durationUs_ = now - self->risingUs_;
            self->ready_ = true;
            self->waiting_ = false;
        }
    }
    portEXIT_CRITICAL_ISR(&self->mux_);
}
void SensorModule::update() {
    const uint32_t now = millis();
    uint32_t duration = 0;
    bool ready, waiting;
    portENTER_CRITICAL(&mux_);
    ready = ready_;
    if (ready) { duration = durationUs_; ready_ = false; }
    if (waiting_ && static_cast<uint32_t>(micros() - triggerUs_) >= Config::ULTRASONIC_ECHO_TIMEOUT_US) {
        waiting_ = false;
        distanceCm_ = UNKNOWN_VALUE;
    }
    waiting = waiting_;
    portEXIT_CRITICAL(&mux_);
    if (ready) {
        distanceCm_ = duration > 0 && duration <= Config::ULTRASONIC_ECHO_TIMEOUT_US ? duration * Config::ULTRASONIC_SOUND_CM_PER_US / 2.0f : UNKNOWN_VALUE;
        lastResultMs_ = now;
    }
    if (!waiting && static_cast<uint32_t>(now - lastTriggerMs_) >= Config::ULTRASONIC_INTERVAL_MS) {
        portENTER_CRITICAL(&mux_);
        waiting_ = true;
        risingSeen_ = false;
        triggerUs_ = micros();
        portEXIT_CRITICAL(&mux_);
        digitalWrite(Config::ULTRASONIC_TRIG_PIN, HIGH);
        delayMicroseconds(Config::ULTRASONIC_TRIGGER_US);
        digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
        lastTriggerMs_ = now;
    }
}
bool SensorModule::valid() const {
    return std::isfinite(distanceCm_) && static_cast<uint32_t>(millis() - lastResultMs_) <= Config::ULTRASONIC_STALE_MS;
}
float SensorModule::readDistanceCm() const { return valid() ? distanceCm_ : UNKNOWN_VALUE; }
bool SensorModule::isObstacleDetected() const { return valid() && readDistanceCm() <= Config::OBSTACLE_DISTANCE_CM; }
