#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace Config {
// Official wiring. Do not change these pins without a hardware revision.
constexpr int LEFT_ENA = 14;
constexpr int LEFT_IN1 = 27;
constexpr int LEFT_IN2 = 26;
constexpr int RIGHT_ENB = 25;
constexpr int RIGHT_IN3 = 33;
constexpr int RIGHT_IN4 = 32;
constexpr int LINE_LEFT_PIN = 39;
constexpr int LINE_CENTER_PIN = 34;
constexpr int LINE_RIGHT_PIN = 35;
constexpr int ULTRASONIC_TRIG_PIN = 12;
constexpr int ULTRASONIC_ECHO_PIN = 13;
constexpr int BATTERY_ADC_PIN = 36;
constexpr int RFID_SCK_PIN = 18;
constexpr int RFID_MISO_PIN = 19;
constexpr int RFID_MOSI_PIN = 23;
constexpr int RFID_SS_PIN = 5;
constexpr int RFID_RST_PIN = 22;
constexpr int ENCODER_LEFT_PIN = 4;
constexpr int ENCODER_RIGHT_PIN = 16;
constexpr int SERVO_1_PIN = 15;
constexpr int SERVO_2_PIN = 17;
constexpr int SERVO_3_PIN = 21;
// New wiring revision: both are strapping pins; see docs/03-hardware.md.
constexpr int BUZZER_PIN = 0;
constexpr int CARGO_IR_PIN = 2;
constexpr int BUZZER_ACTIVE_STATE = 0;
constexpr int CARGO_IR_ACTIVE_STATE = 1;
constexpr uint32_t CARGO_IR_STABLE_MS = 150;
constexpr uint32_t REVERSE_STOP_MS = 150;
// Front-mounted line sensors have not been validated for backward steering.
constexpr bool REVERSE_STEERING_CONFIRMED = false;

enum class MotorDriver { Unconfigured, L298N, BTS7960 };
// Verified in AGV-PROFILE-001 for forward motion; loaded-vehicle trim still needs measurement.
constexpr MotorDriver MOTOR_DRIVER = MotorDriver::L298N;
constexpr bool MOTOR_DIRECTION_CONFIRMED = true;
constexpr bool LEFT_FORWARD_INVERTED = true;
constexpr bool RIGHT_FORWARD_INVERTED = true;
constexpr int DEFAULT_MOTOR_SPEED = 210;
constexpr int LEFT_SPEED_TRIM = -20;
constexpr int RIGHT_SPEED_TRIM = 0;
constexpr int BASE_SPEED = DEFAULT_MOTOR_SPEED;
constexpr int TURN_SPEED = DEFAULT_MOTOR_SPEED;
constexpr int CORRECTION_SPEED = DEFAULT_MOTOR_SPEED;
constexpr uint32_t MOTOR_PWM_HZ = 5000;
constexpr int MOTOR_PWM_BITS = 8;
constexpr std::array<int, 4> MOTOR_PWM_CHANNELS = {0, 1, 2, 3};

// TODO: USER CONFIGURATION: 0 = active LOW, 1 = active HIGH, -1 = unknown.
constexpr int LINE_SENSOR_ACTIVE_STATE = 1;
// TODO: USER CONFIGURATION: measured safety and steering thresholds.
constexpr float OBSTACLE_DISTANCE_CM = 0.0f;
constexpr uint32_t TURN_MIN_MS = 0;
constexpr uint32_t TURN_TIMEOUT_MS = 0;
constexpr uint32_t STRAIGHT_DEPARTURE_MS = 0;
constexpr uint32_t ULTRASONIC_INTERVAL_MS = 60;
constexpr uint32_t ULTRASONIC_ECHO_TIMEOUT_US = 25000;
constexpr uint32_t ULTRASONIC_STALE_MS = 200;
constexpr uint32_t ULTRASONIC_TRIGGER_US = 10;
constexpr float ULTRASONIC_SOUND_CM_PER_US = 0.0343f;

// TODO: USER CONFIGURATION: measured pulses at the encoder and loaded wheel.
constexpr uint32_t ENCODER_PPR = 0;
constexpr float WHEEL_DIAMETER_CM = 0.0f;
constexpr float WHEEL_CIRCUMFERENCE_CM = 3.14159265358979323846f * WHEEL_DIAMETER_CM;

struct NodeMapping { const char* node; const char* uid; };
// TODO: USER CONFIGURATION: uppercase UID hex, no spaces/colons, never invent UIDs.
constexpr std::array<NodeMapping, 26> RFID_NODES = {{
    {"B1", ""}, {"B2", ""},
    {"K1", ""}, {"K2", ""}, {"K3", ""}, {"K4", ""}, {"K5", ""}, {"K6", ""},
    {"C1", ""}, {"C2", ""}, {"C3", ""}, {"C4", ""},
    {"M1", ""}, {"M2", ""}, {"M3", ""}, {"M4", ""}, {"M5", ""}, {"M6", ""},
    {"G1", ""}, {"G2", ""}, {"G3", ""}, {"G4", ""}, {"G5", ""}, {"G6", ""},
    {"GS1", ""}, {"GS2", ""}
}};
constexpr uint32_t RFID_POLL_INTERVAL_MS = 30;

constexpr std::array<int, 3> SERVO_PINS = {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN};
// Channels 4/5/6 use LEDC timers 2/3; motor channels 0..3 use timers 0/1.
constexpr std::array<int, 3> SERVO_PWM_CHANNELS = {4, 5, 6};
constexpr uint32_t SERVO_PWM_HZ = 50;
constexpr int SERVO_PWM_BITS = 16;
// TODO: USER CONFIGURATION: enable only verified servos and assign roles (0..2).
constexpr std::array<bool, 3> SERVO_ENABLED = {false, false, false};
constexpr int SERVO_SCAN = 0;
constexpr int SERVO_LIFT = 1;
constexpr int SERVO_OTHER = 2;
// SERVO_LIFT and SERVO_OTHER are the independently calibrated lift pair.
constexpr std::array<int, 2> SERVO_LIFT_UP_ANGLES = {-1, -1};
constexpr std::array<int, 2> SERVO_LIFT_DOWN_ANGLES = {-1, -1};
// TODO: USER CONFIGURATION: verified pulse limits, mechanical angles and settling.
constexpr int SERVO_MIN_PULSE_US = 0;
constexpr int SERVO_MAX_PULSE_US = 0;
constexpr uint32_t SERVO_MOVE_TIME_MS = 0;

// TODO: USER CONFIGURATION: B25/divider calibration and chemistry-dependent range.
// 0 means unconfigured, not a claim about the physical battery voltage.
constexpr float BATTERY_VOLTAGE_SCALE = 0.0f;
constexpr float BATTERY_VOLTAGE_OFFSET = 0.0f;
constexpr float BATTERY_EMPTY_VOLTAGE = 0.0f;
constexpr float BATTERY_FULL_VOLTAGE = 0.0f;
constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 500;

// TODO: USER CONFIGURATION: network identity, credentials and final topics.
constexpr const char* AGV_ID = "";
constexpr const char* WIFI_SSID = "";
constexpr const char* WIFI_PASSWORD = "";
constexpr const char* MQTT_SERVER = "";
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char* MQTT_USERNAME = "";
constexpr const char* MQTT_PASSWORD = "";
constexpr const char* MQTT_TOPIC_COMMAND = "";
constexpr const char* MQTT_TOPIC_STATUS = "";
constexpr const char* MQTT_TOPIC_POSITION = "";
constexpr const char* MQTT_TOPIC_DISTANCE = "";
constexpr const char* MQTT_TOPIC_EVENT = "";
constexpr const char* MQTT_TOPIC_ERROR = "";
// TODO: USER CONFIGURATION: maximum loss intervals; 0 stops on first loss.
constexpr uint32_t WIFI_TIMEOUT_MS = 0;
constexpr uint32_t MQTT_TIMEOUT_MS = 0;

// Software limits/tuning, not measured hardware calibration.
constexpr uint32_t WIFI_RETRY_MS = 5000;
constexpr uint32_t MQTT_RETRY_MS = 3000;
constexpr uint16_t MQTT_SOCKET_TIMEOUT_SECONDS = 1;
constexpr uint16_t MQTT_KEEPALIVE_SECONDS = 5;
constexpr uint32_t NETWORK_WORKER_STALE_MS = 2000;
constexpr uint32_t COMMAND_MAX_AGE_MS = 2000;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
constexpr uint32_t NETWORK_TASK_STACK_BYTES = 12288;
constexpr size_t MAX_PATH_NODES = 32;
constexpr size_t MAX_COMMAND_BYTES = 1024;
constexpr size_t MAX_PUBLISH_BYTES = 1536;
constexpr size_t COMMAND_QUEUE_LENGTH = 8;
constexpr size_t EVENT_QUEUE_LENGTH = 16;
constexpr size_t RECENT_COMMAND_IDS = 16;
constexpr bool DEBUG_ENABLED = true;
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr std::array<int, 24> ALL_PINS = {
    LEFT_ENA, LEFT_IN1, LEFT_IN2, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4,
    LINE_LEFT_PIN, LINE_CENTER_PIN, LINE_RIGHT_PIN,
    ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, BATTERY_ADC_PIN,
    RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN, RFID_RST_PIN,
    ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN, SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN,
    BUZZER_PIN, CARGO_IR_PIN
};
constexpr bool pinsUnique() {
    for (size_t i = 0; i < ALL_PINS.size(); ++i)
        for (size_t j = i + 1; j < ALL_PINS.size(); ++j)
            if (ALL_PINS[i] == ALL_PINS[j]) return false;
    return true;
}
static_assert(pinsUnique(), "GPIO collision");
static_assert(LINE_LEFT_PIN >= 34 && LINE_CENTER_PIN >= 34 && LINE_RIGHT_PIN >= 34, "Line inputs must remain input-only pins");
static_assert(BATTERY_ADC_PIN == 36, "Battery must remain on ADC1 GPIO36");
static_assert(BUZZER_ACTIVE_STATE == 0 && CARGO_IR_ACTIVE_STATE == 1, "Unexpected buzzer/IR polarity");
static_assert(CARGO_IR_STABLE_MS > 0 && REVERSE_STOP_MS > 0, "Safety debounce/hold must be nonzero");
static_assert(BASE_SPEED >= 0 && BASE_SPEED <= 255 && TURN_SPEED >= 0 && TURN_SPEED <= 255 && CORRECTION_SPEED >= 0 && CORRECTION_SPEED <= 255, "Invalid PWM duty");
}
