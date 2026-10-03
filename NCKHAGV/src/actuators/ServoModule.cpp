#include "ServoModule.h"

bool ServoModule::rolesValid() const {
    const int roles[] = {Config::SERVO_SCAN, Config::SERVO_LIFT, Config::SERVO_OTHER};
    for (size_t i = 0; i < 3; ++i) {
        if (roles[i] < -1 || roles[i] > 2) return false;
        for (size_t j = i + 1; j < 3; ++j)
            if (roles[i] >= 0 && roles[i] == roles[j]) return false;
    }
    return true;
}
void ServoModule::begin() {
    if (!rolesValid() || Config::SERVO_MIN_PULSE_US <= 0 || Config::SERVO_MAX_PULSE_US <= Config::SERVO_MIN_PULSE_US ||
        Config::SERVO_MAX_PULSE_US >= 1000000 / static_cast<int>(Config::SERVO_PWM_HZ)) return;
    for (size_t i = 0; i < Config::SERVO_PINS.size(); ++i) {
        if (!Config::SERVO_ENABLED[i]) continue;
        ledcSetup(Config::SERVO_PWM_CHANNELS[i], Config::SERVO_PWM_HZ, Config::SERVO_PWM_BITS);
        ledcWrite(Config::SERVO_PWM_CHANNELS[i], 0);
        ledcAttachPin(Config::SERVO_PINS[i], Config::SERVO_PWM_CHANNELS[i]);
        attached_[i] = true;
    }
}
bool ServoModule::liftConfigured() const {
    const int pair[] = {Config::SERVO_LIFT, Config::SERVO_OTHER};
    if (pair[0] < 0 || pair[0] >= 3 || pair[1] < 0 || pair[1] >= 3 ||
        pair[0] == pair[1] || !attached_[pair[0]] || !attached_[pair[1]] ||
        !Config::SERVO_MOVE_TIME_MS) return false;
    for (size_t i = 0; i < 2; ++i)
        if (Config::SERVO_LIFT_UP_ANGLES[i] < 0 || Config::SERVO_LIFT_UP_ANGLES[i] > 180 ||
            Config::SERVO_LIFT_DOWN_ANGLES[i] < 0 || Config::SERVO_LIFT_DOWN_ANGLES[i] > 180) return false;
    return true;
}
bool ServoModule::setAngle(size_t index, int angle) {
    if (index >= 3 || !attached_[index] || angle < 0 || angle > 180 || !Config::SERVO_MOVE_TIME_MS) return false;
    const uint32_t pulseUs = Config::SERVO_MIN_PULSE_US + (Config::SERVO_MAX_PULSE_US - Config::SERVO_MIN_PULSE_US) * angle / 180;
    const uint32_t duty = static_cast<uint64_t>(pulseUs) * Config::SERVO_PWM_HZ * ((1UL << Config::SERVO_PWM_BITS) - 1) / 1000000;
    ledcWrite(Config::SERVO_PWM_CHANNELS[index], duty);
    angles_[index] = angle;
    movedAtMs_ = millis();
    busy_ = true;
    return true;
}
bool ServoModule::liftUp() {
    return liftConfigured() && setAngle(Config::SERVO_LIFT, Config::SERVO_LIFT_UP_ANGLES[0]) &&
        setAngle(Config::SERVO_OTHER, Config::SERVO_LIFT_UP_ANGLES[1]);
}
bool ServoModule::liftDown() {
    return liftConfigured() && setAngle(Config::SERVO_LIFT, Config::SERVO_LIFT_DOWN_ANGLES[0]) &&
        setAngle(Config::SERVO_OTHER, Config::SERVO_LIFT_DOWN_ANGLES[1]);
}
void ServoModule::update() { if (busy_ && static_cast<uint32_t>(millis() - movedAtMs_) >= Config::SERVO_MOVE_TIME_MS) busy_ = false; }
int ServoModule::getAngle(size_t index) const { return index < 3 ? angles_[index] : -1; }
