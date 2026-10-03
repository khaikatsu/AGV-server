#include "EncoderModule.h"
#include "core/ControlLogic.h"

void EncoderModule::begin() {
    pinMode(Config::ENCODER_LEFT_PIN, INPUT);
    pinMode(Config::ENCODER_RIGHT_PIN, INPUT);
    reset();
    attachInterruptArg(Config::ENCODER_LEFT_PIN, leftInterrupt, this, RISING);
    attachInterruptArg(Config::ENCODER_RIGHT_PIN, rightInterrupt, this, RISING);
}
void IRAM_ATTR EncoderModule::leftInterrupt(void* argument) {
    auto* self = static_cast<EncoderModule*>(argument);
    portENTER_CRITICAL_ISR(&self->mux_);
    ++self->left_;
    portEXIT_CRITICAL_ISR(&self->mux_);
}
void IRAM_ATTR EncoderModule::rightInterrupt(void* argument) {
    auto* self = static_cast<EncoderModule*>(argument);
    portENTER_CRITICAL_ISR(&self->mux_);
    ++self->right_;
    portEXIT_CRITICAL_ISR(&self->mux_);
}
void EncoderModule::reset() { (void)snapshotAndReset(); }
void EncoderModule::resetLeft() { portENTER_CRITICAL(&mux_); left_ = 0; portEXIT_CRITICAL(&mux_); }
void EncoderModule::resetRight() { portENTER_CRITICAL(&mux_); right_ = 0; portEXIT_CRITICAL(&mux_); }
void EncoderModule::update() { /* ISR counters are the live source; snapshots are atomic. */ }
EncoderData EncoderModule::convert(uint64_t left, uint64_t right) {
    EncoderData data;
    data.leftPulses = left;
    data.rightPulses = right;
    data.leftDistanceCm = AGVMath::distanceCm(left, Config::ENCODER_PPR, Config::WHEEL_DIAMETER_CM);
    data.rightDistanceCm = AGVMath::distanceCm(right, Config::ENCODER_PPR, Config::WHEEL_DIAMETER_CM);
    data.averageDistanceCm = (data.leftDistanceCm + data.rightDistanceCm) / 2.0f;
    return data;
}
EncoderData EncoderModule::snapshot() const {
    portENTER_CRITICAL(&mux_);
    const uint64_t left = left_, right = right_;
    portEXIT_CRITICAL(&mux_);
    return convert(left, right);
}
EncoderData EncoderModule::snapshotAndReset() {
    portENTER_CRITICAL(&mux_);
    const uint64_t left = left_, right = right_;
    left_ = right_ = 0;
    portEXIT_CRITICAL(&mux_);
    return convert(left, right);
}
uint64_t EncoderModule::getLeftPulseCount() const { return snapshot().leftPulses; }
uint64_t EncoderModule::getRightPulseCount() const { return snapshot().rightPulses; }
uint64_t EncoderModule::getPulseCount() const { auto data = snapshot(); return data.leftPulses / 2 + data.rightPulses / 2 + (data.leftPulses % 2 + data.rightPulses % 2) / 2; }
float EncoderModule::getDistanceCm() const { return snapshot().averageDistanceCm; }
float EncoderModule::getLeftDistanceCm() const { return snapshot().leftDistanceCm; }
float EncoderModule::getRightDistanceCm() const { return snapshot().rightDistanceCm; }
