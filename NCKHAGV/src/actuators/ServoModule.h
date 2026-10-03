#pragma once
#include <Arduino.h>
#include "Types.h"

class ServoModule {
public:
    void begin();
    void update();
    bool liftUp();
    bool liftDown();
    bool setAngle(size_t servo, int angle);
    int getAngle(size_t servo) const;
    bool liftConfigured() const;
    bool busy() const { return busy_; }
private:
    bool rolesValid() const;
    bool attached_[3]{};
    int angles_[3] = {-1, -1, -1};
    bool busy_ = false;
    uint32_t movedAtMs_ = 0;
};
