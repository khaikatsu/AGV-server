#pragma once
#include <Arduino.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <atomic>
#include "WiFiModule.h"
#include "Types.h"

class MQTTModule {
public:
    explicit MQTTModule(WiFiModule& wifi) : wifi_(wifi), client_(transport_) {}
    bool begin();
    void connect();
    void loop();
    bool subscribeTopics();
    bool isConnected() const;
    uint32_t session() const { return session_.load(); }
    bool popCommand(AGVCommand& command);
    bool consumeEmergencyStop() { return emergencyStop_.exchange(false); }
    bool consumeStop() { return stop_.exchange(false); }
    bool consumeQueueFault() { return queueFault_.exchange(false); }
    void publishStatus(const TelemetrySnapshot& snapshot);
    void publishPosition(const TelemetrySnapshot& snapshot) { publishStatus(snapshot); }
    void publishDistance(const TelemetrySnapshot& snapshot) { publishStatus(snapshot); }
    bool publishEvent(const char* name, const char* node, const EncoderData& distance, uint32_t uptimeMs,
                      const char* taskId = "", const char* fromNode = "", bool reverse = false);
    bool publishCargo(const char* name, const char* node, const char* taskId, bool present, uint32_t uptimeMs);
    bool publishError(ErrorCode reason, uint32_t uptimeMs);
    bool publishCommandResult(const char* id, bool accepted, ErrorCode reason, uint32_t uptimeMs);
private:
    struct Outbound { bool errorTopic = false; char payload[Config::MAX_PUBLISH_BYTES]{}; };
    static void workerEntry(void* argument);
    void onMessage(char* topic, uint8_t* payload, unsigned int length);
    bool enqueue(const Outbound& message);
    void sendTelemetry(const TelemetrySnapshot& snapshot);
    bool configured() const;
    WiFiModule& wifi_;
    WiFiClient transport_;
    PubSubClient client_;
    QueueHandle_t commands_ = nullptr, events_ = nullptr, telemetry_ = nullptr;
    TaskHandle_t worker_ = nullptr;
    std::atomic<bool> ready_{false}, emergencyStop_{false}, stop_{false}, queueFault_{false};
    std::atomic<uint32_t> servicedAtMs_{0}, session_{0};
    uint32_t lastAttemptMs_ = 0;
    char recentIds_[Config::RECENT_COMMAND_IDS][COMMAND_ID_SIZE]{};
    size_t nextId_ = 0;
    Outbound pending_{};
};
