#pragma once

#include "Config.h"
#include <cstdint>
#include <limits>

constexpr size_t NODE_TEXT_SIZE = 8;
constexpr size_t UID_TEXT_SIZE = 21;
constexpr size_t COMMAND_ID_SIZE = 48;
constexpr float UNKNOWN_VALUE = std::numeric_limits<float>::quiet_NaN();

enum AGVState { AGV_IDLE, AGV_MOVING, AGV_LOADING, AGV_UNLOADING, AGV_WAITING,
                AGV_COMPLETED, AGV_SAFE_STOP, AGV_ERROR };
enum class Departure { FollowLine, Left, Right, Straight, Backward };
enum class MotorAction { Stop, Forward, Backward, CorrectLeft, CorrectRight, PivotLeft, PivotRight };
enum class ServoAction { None, LiftUp, LiftDown };
enum class CommandType { Task, Start, Stop, EmergencyStop, Restart, CancelTask, LiftUp, LiftDown, NodeOrder };
enum class ErrorCode { None, Configuration, WifiTimeout, MqttTimeout, EmergencyStop,
    Obstacle, SensorInvalid, LineLost, AmbiguousLine, UnknownTag, UnexpectedNode,
    TurnTimeout, TurnConfiguration, TurnInterrupted, ServoConfiguration, InvalidTask,
    TaskBusy, NoTask, NotConnected, NotAtRouteStart, RestartDenied, QueueOverflow,
    InvalidCommand, StaleCommand, CargoMissing, CargoLost,
    NodeOrderMismatch, ServoInterrupted };

struct PathStep { char node[NODE_TEXT_SIZE]{}; Departure departure = Departure::FollowLine; };
struct AGVTask {
    char id[COMMAND_ID_SIZE]{};
    char startNode[NODE_TEXT_SIZE]{};
    char targetNode[NODE_TEXT_SIZE]{};
    PathStep path[Config::MAX_PATH_NODES]{};
    uint8_t pathLength = 0;
    bool active = false;
};
struct AGVPosition { char node[NODE_TEXT_SIZE]{}; float distanceCm = UNKNOWN_VALUE; };
struct EncoderData {
    uint64_t leftPulses = 0, rightPulses = 0;
    float leftDistanceCm = UNKNOWN_VALUE, rightDistanceCm = UNKNOWN_VALUE;
    float averageDistanceCm = UNKNOWN_VALUE;
};
struct RFIDEvent { char uid[UID_TEXT_SIZE]{}; char node[NODE_TEXT_SIZE]{}; bool newNode = false; };
struct LineReadings {
    int left = 0, center = 0, right = 0;
    uint8_t pattern = 0;
    bool configured = false;
};
struct AGVCommand {
    CommandType type = CommandType::Stop;
    char id[COMMAND_ID_SIZE]{};
    uint32_t session = 0;
    uint32_t receivedAtMs = 0;
    AGVTask task{};
    char taskId[COMMAND_ID_SIZE]{};
    char node[NODE_TEXT_SIZE]{};
    char nextNode[NODE_TEXT_SIZE]{};
    uint8_t pathIndex = 0;
    Departure departure = Departure::FollowLine;
};
struct ControlInput {
    uint32_t now = 0;
    bool wifi = false, mqtt = false;
    bool motionConfigured = false, liftConfigured = false;
    bool distanceValid = false, obstacle = false, servoBusy = false;
    bool lineConfigured = false;
    uint8_t linePattern = 0;
    bool cargoStable = false, cargoPresent = false;
};
struct ControlOutput { MotorAction motor = MotorAction::Stop; ServoAction servo = ServoAction::None; };
struct SafetySettings {
    uint32_t wifiTimeoutMs = 0, mqttTimeoutMs = 0;
    uint32_t turnMinMs = 0, turnTimeoutMs = 0, straightDepartureMs = 0;
    uint32_t servoMoveMs = 0;
    bool requireNodeOrders = false, requireLiftCommands = false;
    bool reverseSteeringConfirmed = false;
};
struct TelemetrySnapshot {
    AGVState state = AGV_IDLE;
    ErrorCode error = ErrorCode::None;
    AGVPosition position{};
    char nextNode[NODE_TEXT_SIZE]{};
    EncoderData encoder{};
    char taskId[COMMAND_ID_SIZE]{};
    bool taskActive = false, wifi = false, mqtt = false;
    bool batteryCalibrated = false;
    uint8_t pathIndex = 0, pathLength = 0;
    int leftDuty = 0, rightDuty = 0;
    MotorAction motor = MotorAction::Stop;
    float ultrasonicCm = UNKNOWN_VALUE;
    uint32_t batteryAdcMv = 0, uptimeMs = 0;
    float batteryVoltage = UNKNOWN_VALUE, batteryPercentage = UNKNOWN_VALUE;
    bool cargoStable = false, cargoPresent = false;
    bool buzzerOn = false;
};

const char* stateName(AGVState state);
const char* errorName(ErrorCode error);
const char* motorActionName(MotorAction action);
