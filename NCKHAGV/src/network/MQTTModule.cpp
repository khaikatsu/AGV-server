#include "MQTTModule.h"
#include "core/CommandCodec.h"
#include "core/ControlLogic.h"
#include <ArduinoJson.h>
#include <esp_system.h>
#include <cmath>
#include <cstring>

namespace {
void number(JsonObject object, const char* key, float value) {
    if (std::isfinite(value)) object[key] = value;
    else object[key] = nullptr;
}
void base(JsonObject object, uint32_t session, uint32_t uptime) {
    object["version"] = 1;
    object["agvId"] = Config::AGV_ID;
    object["session"] = session;
    object["uptimeMs"] = uptime;
}
bool serialize(const JsonDocument& doc, char* output, size_t capacity) {
    if (doc.overflowed() || measureJson(doc) >= capacity) return false;
    return serializeJson(doc, output, capacity) > 0;
}
}
bool MQTTModule::configured() const {
    const char* required[] = {Config::AGV_ID, Config::MQTT_SERVER, Config::MQTT_TOPIC_COMMAND,
        Config::MQTT_TOPIC_STATUS, Config::MQTT_TOPIC_POSITION, Config::MQTT_TOPIC_DISTANCE,
        Config::MQTT_TOPIC_EVENT, Config::MQTT_TOPIC_ERROR};
    for (const char* value : required) if (!value[0]) return false;
    // Publish topics must be concrete, distinct and fit the MQTT packet buffer.
    for (size_t i = 2; i < 8; ++i) {
        if (std::strlen(required[i]) > 128 || std::strchr(required[i], '#') || std::strchr(required[i], '+')) return false;
        for (size_t j = i + 1; j < 8; ++j) if (std::strcmp(required[i], required[j]) == 0) return false;
    }
    return std::strlen(Config::AGV_ID) < COMMAND_ID_SIZE;
}
bool MQTTModule::begin() {
    commands_ = xQueueCreate(Config::COMMAND_QUEUE_LENGTH, sizeof(AGVCommand));
    events_ = xQueueCreate(Config::EVENT_QUEUE_LENGTH, sizeof(Outbound));
    telemetry_ = xQueueCreate(1, sizeof(TelemetrySnapshot));
    if (!commands_ || !events_ || !telemetry_) { queueFault_ = true; return false; }
    client_.setServer(Config::MQTT_SERVER, Config::MQTT_PORT);
    client_.setSocketTimeout(Config::MQTT_SOCKET_TIMEOUT_SECONDS);
    client_.setKeepAlive(Config::MQTT_KEEPALIVE_SECONDS);
    if (!client_.setBufferSize(Config::MAX_PUBLISH_BYTES + 256)) { queueFault_ = true; return false; }
    transport_.setTimeout(Config::MQTT_SOCKET_TIMEOUT_SECONDS);
    client_.setCallback([this](char* topic, uint8_t* payload, unsigned int length) { onMessage(topic, payload, length); });
    servicedAtMs_ = millis();
    const BaseType_t result = xTaskCreatePinnedToCore(workerEntry, "agv-network", Config::NETWORK_TASK_STACK_BYTES, this, 1, &worker_, 0);
    if (result != pdPASS) queueFault_ = true;
    return result == pdPASS;
}
void MQTTModule::workerEntry(void* argument) {
    auto* self = static_cast<MQTTModule*>(argument);
    self->wifi_.begin();
    for (;;) {
        self->loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
bool MQTTModule::isConnected() const {
    return ready_.load() && wifi_.isConnected() && static_cast<uint32_t>(millis() - servicedAtMs_.load()) < Config::NETWORK_WORKER_STALE_MS;
}
bool MQTTModule::subscribeTopics() { return client_.subscribe(Config::MQTT_TOPIC_COMMAND, 0); }
void MQTTModule::connect() {
    lastAttemptMs_ = millis();
    if (!wifi_.isConnected() || !configured()) return;
    char clientId[COMMAND_ID_SIZE + 16];
    uint32_t nonce = esp_random();
    if (!nonce || nonce == session_.load()) { nonce = session_.load() + 1; if (!nonce) nonce = 1; }
    snprintf(clientId, sizeof(clientId), "%s-%08lx", Config::AGV_ID, static_cast<unsigned long>(nonce));
    StaticJsonDocument<256> will;
    base(will.to<JsonObject>(), nonce, 0);
    will["event"] = "AGV_CONNECTION_LOST";
    char willPayload[256];
    if (!serialize(will, willPayload, sizeof(willPayload))) return;
    const bool connected = client_.connect(clientId, Config::MQTT_USERNAME, Config::MQTT_PASSWORD,
        Config::MQTT_TOPIC_ERROR, 0, false, willPayload);
    if (!connected || !subscribeTopics()) { client_.disconnect(); ready_ = false; return; }
    std::memset(recentIds_, 0, sizeof(recentIds_));
    nextId_ = 0;
    session_ = nonce;
    ready_ = true;
    if (Config::DEBUG_ENABLED) Serial.printf("[MQTT] Connected session=%lu\n", static_cast<unsigned long>(nonce));
}
void MQTTModule::loop() {
    wifi_.loop();
    if (!wifi_.isConnected()) { ready_ = false; transport_.stop(); }
    if (wifi_.isConnected() && !client_.connected() && static_cast<uint32_t>(millis() - lastAttemptMs_) >= Config::MQTT_RETRY_MS) connect();
    if (client_.connected()) {
        if (!client_.loop()) ready_ = false;
        if (ready_.load()) {
            // A failed QoS0 publish retains the front event for the next connection.
            if (xQueuePeek(events_, &pending_, 0) == pdTRUE &&
                client_.publish(pending_.errorTopic ? Config::MQTT_TOPIC_ERROR : Config::MQTT_TOPIC_EVENT, pending_.payload, false)) {
                xQueueReceive(events_, &pending_, 0);
            }
            TelemetrySnapshot snapshot;
            if (xQueueReceive(telemetry_, &snapshot, 0) == pdTRUE) sendTelemetry(snapshot);
        }
    } else ready_ = false;
    servicedAtMs_ = millis();
}
bool MQTTModule::popCommand(AGVCommand& command) { return commands_ && xQueueReceive(commands_, &command, 0) == pdTRUE; }
void MQTTModule::onMessage(char* topic, uint8_t* payload, unsigned int length) {
    if (std::strcmp(topic, Config::MQTT_TOPIC_COMMAND) != 0) return;
    AGVCommand command;
    ErrorCode reason;
    if (!CommandCodec::decode(payload, length, command, reason)) { publishError(reason, millis()); return; }
    const bool safetyCommand = command.type == CommandType::Stop || command.type == CommandType::EmergencyStop;
    // Stop priority is independent of deduplication, age, session and queue capacity.
    if (command.type == CommandType::EmergencyStop) emergencyStop_ = true;
    if (command.type == CommandType::Stop) stop_ = true;
    if (!safetyCommand && command.session != session_.load()) { publishCommandResult(command.id, false, ErrorCode::StaleCommand, millis()); return; }
    for (const auto& id : recentIds_)
        if (std::strcmp(id, command.id) == 0) { publishCommandResult(command.id, false, ErrorCode::InvalidCommand, millis()); return; }
    command.receivedAtMs = millis();
    if (xQueueSend(commands_, &command, 0) != pdTRUE) { queueFault_ = true; return; }
    AGVMath::copyText(recentIds_[nextId_], COMMAND_ID_SIZE, command.id);
    nextId_ = (nextId_ + 1) % Config::RECENT_COMMAND_IDS;
}
bool MQTTModule::enqueue(const Outbound& message) {
    if (!events_ || xQueueSend(events_, &message, 0) != pdTRUE) { queueFault_ = true; return false; }
    return true;
}
void MQTTModule::publishStatus(const TelemetrySnapshot& snapshot) {
    if (!telemetry_ || xQueueOverwrite(telemetry_, &snapshot) != pdTRUE) queueFault_ = true;
}
bool MQTTModule::publishEvent(const char* name, const char* node, const EncoderData& distance, uint32_t uptime,
                              const char* taskId, const char* fromNode, bool reverse) {
    StaticJsonDocument<512> doc;
    auto object = doc.to<JsonObject>();
    base(object, session(), uptime);
    object["event"] = name;
    object["node"] = node;
    object["taskId"] = taskId;
    object["fromNode"] = fromNode;
    object["reverse"] = reverse;
    object["leftPulses"] = distance.leftPulses;
    object["rightPulses"] = distance.rightPulses;
    number(object, "distanceCm", distance.averageDistanceCm);
    number(object, "distanceMm", reverse ? -distance.averageDistanceCm * 10.0f : distance.averageDistanceCm * 10.0f);
    number(object, "leftDistanceCm", distance.leftDistanceCm);
    number(object, "rightDistanceCm", distance.rightDistanceCm);
    Outbound message;
    if (!serialize(doc, message.payload, sizeof(message.payload))) { queueFault_ = true; return false; }
    return enqueue(message);
}
bool MQTTModule::publishCargo(const char* name, const char* node, const char* taskId, bool present, uint32_t uptime) {
    StaticJsonDocument<384> doc;
    auto object = doc.to<JsonObject>();
    base(object, session(), uptime);
    object["event"] = name;
    object["node"] = node;
    object["taskId"] = taskId;
    object["cargoPresent"] = present;
    Outbound message;
    if (!serialize(doc, message.payload, sizeof(message.payload))) { queueFault_ = true; return false; }
    return enqueue(message);
}
bool MQTTModule::publishError(ErrorCode reason, uint32_t uptime) {
    StaticJsonDocument<256> doc;
    auto object = doc.to<JsonObject>();
    base(object, session(), uptime);
    object["error"] = errorName(reason);
    Outbound message;
    message.errorTopic = true;
    if (!serialize(doc, message.payload, sizeof(message.payload))) { queueFault_ = true; return false; }
    return enqueue(message);
}
bool MQTTModule::publishCommandResult(const char* id, bool accepted, ErrorCode reason, uint32_t uptime) {
    StaticJsonDocument<384> doc;
    auto object = doc.to<JsonObject>();
    base(object, session(), uptime);
    object["event"] = "AGV_COMMAND_RESULT";
    object["id"] = id;
    object["accepted"] = accepted;
    object["error"] = errorName(reason);
    Outbound message;
    if (!serialize(doc, message.payload, sizeof(message.payload))) { queueFault_ = true; return false; }
    return enqueue(message);
}
void MQTTModule::sendTelemetry(const TelemetrySnapshot& s) {
    StaticJsonDocument<1536> doc;
    auto object = doc.to<JsonObject>();
    base(object, session(), s.uptimeMs);
    object["state"] = stateName(s.state);
    object["error"] = errorName(s.error);
    object["node"] = s.position.node;
    object["nextNode"] = s.nextNode;
    number(object, "distanceCm", s.position.distanceCm);
    number(object, "distanceMm", s.motor == MotorAction::Backward ? -s.position.distanceCm * 10.0f : s.position.distanceCm * 10.0f);
    object["taskId"] = s.taskId;
    object["taskActive"] = s.taskActive;
    object["pathIndex"] = s.pathIndex;
    object["pathLength"] = s.pathLength;
    object["motor"] = motorActionName(s.motor);
    object["leftDuty"] = s.leftDuty;
    object["rightDuty"] = s.rightDuty;
    object["cargoStable"] = s.cargoStable;
    object["cargoPresent"] = s.cargoStable ? s.cargoPresent : false;
    object["buzzerOn"] = s.buzzerOn;
    object["wifi"] = s.wifi;
    object["mqtt"] = s.mqtt;
    number(object, "ultrasonicCm", s.ultrasonicCm);
    object["batteryAdcMv"] = s.batteryAdcMv;
    object["batteryCalibrated"] = s.batteryCalibrated;
    number(object, "batteryVoltage", s.batteryVoltage);
    number(object, "batteryPercentage", s.batteryPercentage);
    char payload[Config::MAX_PUBLISH_BYTES];
    if (serialize(doc, payload, sizeof(payload))) client_.publish(Config::MQTT_TOPIC_STATUS, payload, false);
    else queueFault_ = true;
    doc.clear();
    object = doc.to<JsonObject>();
    base(object, session(), s.uptimeMs);
    object["node"] = s.position.node;
    object["nextNode"] = s.nextNode;
    number(object, "distanceCm", s.position.distanceCm);
    number(object, "distanceMm", s.motor == MotorAction::Backward ? -s.position.distanceCm * 10.0f : s.position.distanceCm * 10.0f);
    object["pathIndex"] = s.pathIndex;
    object["motor"] = motorActionName(s.motor);
    if (serialize(doc, payload, sizeof(payload))) client_.publish(Config::MQTT_TOPIC_POSITION, payload, false);
    doc.clear();
    object = doc.to<JsonObject>();
    base(object, session(), s.uptimeMs);
    object["node"] = s.position.node;
    object["leftPulses"] = s.encoder.leftPulses;
    object["rightPulses"] = s.encoder.rightPulses;
    number(object, "leftDistanceCm", s.encoder.leftDistanceCm);
    number(object, "rightDistanceCm", s.encoder.rightDistanceCm);
    number(object, "distanceCm", s.encoder.averageDistanceCm);
    number(object, "distanceMm", s.motor == MotorAction::Backward ? -s.encoder.averageDistanceCm * 10.0f : s.encoder.averageDistanceCm * 10.0f);
    if (serialize(doc, payload, sizeof(payload))) client_.publish(Config::MQTT_TOPIC_DISTANCE, payload, false);
}
