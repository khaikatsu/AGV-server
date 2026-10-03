#include "CommandCodec.h"
#include "ControlLogic.h"
#include <ArduinoJson.h>
#include <cstring>

namespace {
struct PayloadReader {
    const uint8_t* data;
    size_t length, offset = 0;
    int read() { return offset < length ? data[offset++] : -1; }
    size_t readBytes(char* buffer, size_t count) {
        const size_t available = length - offset;
        if (count > available) count = available;
        std::memcpy(buffer, data + offset, count);
        offset += count;
        return count;
    }
};
bool identifier(const char* value) {
    if (!value || !value[0] || std::strlen(value) >= COMMAND_ID_SIZE) return false;
    for (const char* p = value; *p; ++p)
        if (!( (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z') ||
               (*p >= '0' && *p <= '9') || *p == '-' || *p == '_' || *p == '.')) return false;
    return true;
}
bool text(JsonVariantConst value, char* destination, size_t capacity) {
    if (!value.is<const char*>()) return false;
    const JsonString string = value.as<JsonString>();
    return string.size() == std::strlen(string.c_str()) && AGVMath::copyText(destination, capacity, string.c_str());
}
bool departure(const char* value, Departure& output) {
    if (!value || std::strcmp(value, "FOLLOW") == 0) output = Departure::FollowLine;
    else if (std::strcmp(value, "LEFT") == 0) output = Departure::Left;
    else if (std::strcmp(value, "RIGHT") == 0) output = Departure::Right;
    else if (std::strcmp(value, "STRAIGHT") == 0) output = Departure::Straight;
    else if (std::strcmp(value, "BACKWARD") == 0) output = Departure::Backward;
    else return false;
    return true;
}
}
bool CommandCodec::decode(const uint8_t* payload, size_t length, AGVCommand& command, ErrorCode& reason) {
    command = AGVCommand{};
    reason = ErrorCode::InvalidCommand;
    if (!payload || !length || length > Config::MAX_COMMAND_BYTES) return false;
    for (size_t i = 0; i < length; ++i) if (payload[i] == 0) return false;
    StaticJsonDocument<4096> doc;
    PayloadReader reader{payload, length};
    if (deserializeJson(doc, reader) || !doc.is<JsonObject>() ||
        !doc["version"].is<int>() || doc["version"].as<int>() != 1 || !doc["command"].is<const char*>()) return false;
    for (size_t i = reader.offset; i < length; ++i)
        if (payload[i] != ' ' && payload[i] != '\t' && payload[i] != '\n' && payload[i] != '\r') return false;
    if (!text(doc["id"], command.id, sizeof(command.id)) || !identifier(command.id)) return false;
    if (doc["command"].as<JsonString>().size() != std::strlen(doc["command"].as<const char*>())) return false;
    const char* name = doc["command"];
    if (std::strcmp(name, "TASK") == 0) command.type = CommandType::Task;
    else if (std::strcmp(name, "START") == 0) command.type = CommandType::Start;
    else if (std::strcmp(name, "STOP") == 0) command.type = CommandType::Stop;
    else if (std::strcmp(name, "EMERGENCY_STOP") == 0) command.type = CommandType::EmergencyStop;
    else if (std::strcmp(name, "RESTART") == 0) command.type = CommandType::Restart;
    else if (std::strcmp(name, "CANCEL_TASK") == 0) command.type = CommandType::CancelTask;
    else if (std::strcmp(name, "LIFT_UP") == 0) command.type = CommandType::LiftUp;
    else if (std::strcmp(name, "LIFT_DOWN") == 0) command.type = CommandType::LiftDown;
    else if (std::strcmp(name, "NODE_ORDER") == 0) command.type = CommandType::NodeOrder;
    else return false;
    const bool stop = command.type == CommandType::Stop || command.type == CommandType::EmergencyStop;
    if (!doc["session"].is<uint32_t>() || (!stop && doc["session"].as<uint32_t>() == 0)) return false;
    command.session = doc["session"].as<uint32_t>();
    if (command.type == CommandType::Task) {
        reason = ErrorCode::InvalidTask;
        JsonObjectConst task = doc["task"].as<JsonObjectConst>();
        if (task.isNull() || !text(task["id"], command.task.id, sizeof(command.task.id)) || !identifier(command.task.id) ||
            !text(task["startNode"], command.task.startNode, sizeof(command.task.startNode)) ||
            !text(task["targetNode"], command.task.targetNode, sizeof(command.task.targetNode))) return false;
        JsonArrayConst path = task["path"].as<JsonArrayConst>();
        if (path.isNull() || path.size() == 0 || path.size() > Config::MAX_PATH_NODES) return false;
        command.task.pathLength = static_cast<uint8_t>(path.size());
        size_t i = 0;
        for (JsonVariantConst value : path) {
            PathStep& step = command.task.path[i++];
            if (value.is<const char*>()) {
                if (!text(value, step.node, sizeof(step.node))) return false;
            } else if (value.is<JsonObjectConst>()) {
                if (!text(value["node"], step.node, sizeof(step.node))) return false;
                if (!value["departure"].isNull() && !value["departure"].is<const char*>()) return false;
                if (value["departure"].is<const char*>() && value["departure"].as<JsonString>().size() != std::strlen(value["departure"].as<const char*>())) return false;
                if (!departure(value["departure"].as<const char*>(), step.departure)) return false;
            } else return false;
        }
        if (!AGVMath::validTask(command.task)) return false;
    } else if (command.type == CommandType::NodeOrder || command.type == CommandType::LiftUp || command.type == CommandType::LiftDown) {
        if (!text(doc["taskId"], command.taskId, sizeof(command.taskId)) || !identifier(command.taskId) ||
            !text(doc["node"], command.node, sizeof(command.node)) || !AGVMath::validNode(command.node) ||
            !doc["pathIndex"].is<uint8_t>()) return false;
        command.pathIndex = doc["pathIndex"].as<uint8_t>();
        if (command.type == CommandType::NodeOrder) {
            if (!text(doc["nextNode"], command.nextNode, sizeof(command.nextNode)) ||
                !AGVMath::validNode(command.nextNode) || !doc["departure"].is<const char*>() ||
                !departure(doc["departure"].as<const char*>(), command.departure) ||
                command.departure == Departure::FollowLine) return false;
        }
    }
    reason = ErrorCode::None;
    return true;
}
