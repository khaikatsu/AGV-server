#include "WiFiModule.h"
#include "Config.h"
#include <WiFi.h>

void WiFiModule::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    connect();
}
void WiFiModule::connect() {
    lastAttemptMs_ = millis();
    if (!Config::WIFI_SSID[0]) return;
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    if (Config::DEBUG_ENABLED) Serial.println("[WiFi] Connection requested");
}
void WiFiModule::reconnect() { connect(); }
void WiFiModule::loop() {
    if (!isConnected() && static_cast<uint32_t>(millis() - lastAttemptMs_) >= Config::WIFI_RETRY_MS) reconnect();
}
bool WiFiModule::isConnected() const { return WiFi.status() == WL_CONNECTED; }
