#pragma once

// Read ONLY the official GPIO map; never write production configuration.
#include "../../../NCKHAGV/src/Config.h"

namespace Bench {
constexpr bool LEFT_FORWARD_INVERTED = false;
constexpr bool RIGHT_FORWARD_INVERTED = false;
// Bench tuning defaults, NOT measured calibration or acceptance thresholds.
constexpr int BASE_PWM = 100;
constexpr int CORRECTION_PWM = 45;
constexpr int TURN_PWM = 95;
constexpr uint32_t TURN_MIN_MS = 200;
constexpr uint32_t TURN_TIMEOUT_MS = 5000;
constexpr uint32_t LINE_STABLE_MS = 30;
constexpr uint32_t MANUAL_PULSE_MS = 400;
constexpr uint32_t RFID_POLL_MS = 60;
// Optional real UID: uppercase hexadecimal, no separators. Empty: learn via 'u'.
constexpr const char* CARD_1_UID = "";
// User described clockwise pivot until LEFT sensor finds the new line.
constexpr bool CARD_1_PIVOT_RIGHT = true;
}
