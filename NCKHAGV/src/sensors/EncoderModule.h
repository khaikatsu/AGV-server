#pragma once
#include <Arduino.h>
#include "Types.h"

class EncoderModule {
public:
    void begin();
    void reset();
    void resetLeft();
    void resetRight();
    void update();
    uint64_t getPulseCount() const;
    uint64_t getLeftPulseCount() const;
    uint64_t getRightPulseCount() const;
    float getDistance() const { return getDistanceCm(); }
    float getDistanceCm() const;
    float getLeftDistanceCm() const;
    float getRightDistanceCm() const;
    EncoderData snapshot() const;
    EncoderData snapshotAndReset();
private:
    static void IRAM_ATTR leftInterrupt(void* argument);
    static void IRAM_ATTR rightInterrupt(void* argument);
    static EncoderData convert(uint64_t left, uint64_t right);
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile uint64_t left_ = 0, right_ = 0;
};
