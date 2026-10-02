#pragma once

#include "Config.h"

namespace TestConfig {

// Doi thanh Config::MotorDriver::BTS7960 neu xe dang dung BTS7960.
constexpr Config::MotorDriver DRIVER = Config::MotorDriver::L298N;

// Doi false -> true cho tung ben neu banh ben do quay nguoc.
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = true;

constexpr int BASE_SPEED = 210;       // 0..255; toc do chung cua hai ben khi di thang.
constexpr int CORRECT_SPEED = 210;    // Toc do cum banh doi dien sensor de quay tim line.
// Cum banh trai dang manh hon: tru 20 PWM o moi lenh chay cua ben trai.
constexpr int LEFT_PWM_TRIM = -20;
constexpr int RIGHT_PWM_TRIM = 0;

// false: doc dung muc dien GPIO; true: dao LOW <-> HIGH.
constexpr bool INVERT_LINE_INPUTS = false;
// Voi bo cam bien hien tai, muc HIGH duoc xem la dang nam tren line den.
constexpr int LINE_ACTIVE_LEVEL = 1;
constexpr uint32_t LINE_CALIBRATION_STABLE_MS = 120;

// SERVO_INDEX = 0 -> Servo 1 GPIO15; 1 -> GPIO17; 2 -> GPIO21.
constexpr size_t SERVO_INDEX = 0;
constexpr uint16_t SERVO_MIN_US = 1000;
constexpr uint16_t SERVO_MAX_US = 2000;
constexpr int SERVO_STEP_DEG = 15;
constexpr uint32_t SERVO_CENTER_WAIT_MS = 800;
constexpr uint32_t SERVO_SETTLE_MS = 250;

constexpr uint32_t ECHO_TIMEOUT_US = 30000;
constexpr uint32_t STATUS_PERIOD_MS = 250;
constexpr int BOOT_BUTTON_PIN = 0;
constexpr uint32_t BOOT_DEBOUNCE_MS = 30;

}  // namespace TestConfig

