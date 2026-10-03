#pragma once
#include <Arduino.h>

class WiFiModule {
public:
    void begin();
    void connect();
    void loop();
    bool isConnected() const;
    void reconnect();
private:
    uint32_t lastAttemptMs_ = 0;
};
