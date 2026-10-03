#include "ControlLogic.h"
#include <cmath>
#include <cstring>

const char* stateName(AGVState s) {
    switch (s) {
        case AGV_IDLE: return "AGV_IDLE";
        case AGV_MOVING: return "AGV_MOVING";
        case AGV_LOADING: return "AGV_LOADING";
        case AGV_UNLOADING: return "AGV_UNLOADING";
        case AGV_WAITING: return "AGV_WAITING";
        case AGV_COMPLETED: return "AGV_COMPLETED";
        case AGV_SAFE_STOP: return "AGV_SAFE_STOP";
        case AGV_ERROR: return "AGV_ERROR";
    }
    return "AGV_ERROR";
}
const char* errorName(ErrorCode e) {
    switch (e) {
        case ErrorCode::None: return "NONE";
        case ErrorCode::Configuration: return "CONFIGURATION_INCOMPLETE";
        case ErrorCode::WifiTimeout: return "WIFI_TIMEOUT";
        case ErrorCode::MqttTimeout: return "MQTT_TIMEOUT";
        case ErrorCode::EmergencyStop: return "EMERGENCY_STOP";
        case ErrorCode::Obstacle: return "OBSTACLE";
        case ErrorCode::SensorInvalid: return "ULTRASONIC_INVALID";
        case ErrorCode::LineLost: return "LINE_LOST";
        case ErrorCode::AmbiguousLine: return "AMBIGUOUS_LINE";
        case ErrorCode::UnknownTag: return "RFID_UNMAPPED_UID";
        case ErrorCode::UnexpectedNode: return "UNEXPECTED_NODE";
        case ErrorCode::TurnTimeout: return "TURN_TIMEOUT";
        case ErrorCode::TurnConfiguration: return "TURN_NOT_CONFIGURED";
        case ErrorCode::TurnInterrupted: return "TURN_INTERRUPTED_CANCEL_TASK";
        case ErrorCode::ServoConfiguration: return "SERVO_NOT_CONFIGURED";
        case ErrorCode::InvalidTask: return "INVALID_TASK";
        case ErrorCode::TaskBusy: return "TASK_BUSY";
        case ErrorCode::NoTask: return "NO_TASK";
        case ErrorCode::NotConnected: return "NETWORK_UNAVAILABLE";
        case ErrorCode::NotAtRouteStart: return "NOT_AT_ROUTE_NODE";
        case ErrorCode::RestartDenied: return "RESTART_REQUIRED_OR_DENIED";
        case ErrorCode::QueueOverflow: return "QUEUE_OVERFLOW";
        case ErrorCode::InvalidCommand: return "INVALID_COMMAND";
        case ErrorCode::StaleCommand: return "STALE_COMMAND";
        case ErrorCode::CargoMissing: return "CARGO_MISSING";
        case ErrorCode::CargoLost: return "CARGO_LOST";
        case ErrorCode::NodeOrderMismatch: return "NODE_ORDER_MISMATCH";
        case ErrorCode::ServoInterrupted: return "SERVO_INTERRUPTED_CANCEL_TASK";
    }
    return "UNKNOWN_ERROR";
}
const char* motorActionName(MotorAction action) {
    switch (action) {
        case MotorAction::Stop: return "STOP";
        case MotorAction::Forward: return "FORWARD";
        case MotorAction::Backward: return "BACKWARD";
        case MotorAction::CorrectLeft: return "CORRECT_LEFT";
        case MotorAction::CorrectRight: return "CORRECT_RIGHT";
        case MotorAction::PivotLeft: return "LEFT";
        case MotorAction::PivotRight: return "RIGHT";
    }
    return "STOP";
}

namespace AGVMath {
bool copyText(char* dst, size_t capacity, const char* src) {
    if (!src || capacity == 0 || std::strlen(src) >= capacity) return false;
    std::memcpy(dst, src, std::strlen(src) + 1);
    return true;
}
bool reverseBuzzerRequired(MotorAction action, int leftDuty, int rightDuty) {
    return action == MotorAction::Backward && leftDuty < 0 && rightDuty < 0;
}
float distanceCm(uint64_t pulses, uint32_t ppr, float diameter) {
    if (!ppr || !std::isfinite(diameter) || diameter <= 0) return UNKNOWN_VALUE;
    return static_cast<float>(static_cast<double>(pulses) / ppr * 3.14159265358979323846 * diameter);
}
float batteryVoltage(uint32_t adcMv, float scale, float offset) {
    if (!std::isfinite(scale) || scale <= 0 || !std::isfinite(offset)) return UNKNOWN_VALUE;
    const float value = adcMv / 1000.0f * scale + offset;
    return value >= 0 && std::isfinite(value) ? value : UNKNOWN_VALUE;
}
float batteryPercentage(float voltage, float empty, float full) {
    if (!std::isfinite(voltage) || !std::isfinite(empty) || !std::isfinite(full) || empty <= 0 || full <= empty) return UNKNOWN_VALUE;
    const float value = (voltage - empty) * 100.0f / (full - empty);
    return value < 0 ? 0 : (value > 100 ? 100 : value);
}
uint8_t normalizeLine(int l, int c, int r, int active) {
    if (active != 0 && active != 1) return 0;
    return static_cast<uint8_t>((l == active ? 4 : 0) | (c == active ? 2 : 0) | (r == active ? 1 : 0));
}
MotorAction lineAction(uint8_t p) {
    switch (p) {
        case 2: case 5: case 7: return MotorAction::Forward;
        case 4: case 6: return MotorAction::CorrectLeft;
        case 1: case 3: return MotorAction::CorrectRight;
        default: return MotorAction::Stop;
    }
}
bool validNode(const char* node) {
    if (!node) return false;
    for (const auto& entry : Config::RFID_NODES)
        if (std::strcmp(node, entry.node) == 0) return true;
    return false;
}
bool validTask(const AGVTask& task) {
    if (!task.id[0] || !validNode(task.startNode) || !validNode(task.targetNode) ||
        task.pathLength == 0 || task.pathLength > Config::MAX_PATH_NODES) return false;
    bool pickupFound = false;
    for (size_t i = 0; i < task.pathLength; ++i) {
        if (!validNode(task.path[i].node)) return false;
        if (i && std::strcmp(task.path[i - 1].node, task.path[i].node) == 0) return false;
        if (std::strcmp(task.path[i].node, task.startNode) == 0) pickupFound = true;
        if (task.path[i].departure != Departure::FollowLine && task.path[i].departure != Departure::Left &&
            task.path[i].departure != Departure::Right && task.path[i].departure != Departure::Straight &&
            task.path[i].departure != Departure::Backward) return false;
    }
    return pickupFound && std::strcmp(task.path[task.pathLength - 1].node, task.targetNode) == 0;
}
}

ControlLogic::ControlLogic(SafetySettings settings) : settings_(settings) {}
void ControlLogic::begin(uint32_t now) {
    wifiLastGoodMs_ = mqttLastGoodMs_ = phaseStartedMs_ = now;
    state_ = AGV_IDLE;
}
void ControlLogic::safeStop(ErrorCode reason) {
    if (turning_) turnInterrupted_ = true;
    if ((state_ == AGV_LOADING || state_ == AGV_UNLOADING) && servoIssued_) servoInterrupted_ = true;
    turning_ = false;
    waitingForOrder_ = hasOrder_ = reverseHold_ = false;
    state_ = AGV_SAFE_STOP;
    error_ = reason;
}
void ControlLogic::fail(ErrorCode reason) {
    safeStop(reason);
    state_ = AGV_ERROR;
}
void ControlLogic::pause() {
    if (state_ == AGV_SAFE_STOP || state_ == AGV_ERROR) return;
    if ((state_ == AGV_LOADING || state_ == AGV_UNLOADING) && servoIssued_) {
        safeStop(ErrorCode::ServoInterrupted);
        return;
    }
    if (turning_) turnInterrupted_ = true;
    turning_ = false;
    waitingForOrder_ = hasOrder_ = reverseHold_ = false;
    state_ = task_.active ? AGV_WAITING : AGV_IDLE;
}
void ControlLogic::cancelTask() {
    task_ = AGVTask{};
    pickupDone_ = servoIssued_ = turning_ = turnInterrupted_ = liftAuthorized_ = waitingForOrder_ = hasOrder_ = reverseHold_ = servoInterrupted_ = false;
    pathIndex_ = 0;
    if (state_ != AGV_SAFE_STOP && state_ != AGV_ERROR) state_ = AGV_IDLE;
}
bool ControlLogic::setTask(const AGVTask& task, ErrorCode& reason) {
    if (state_ == AGV_SAFE_STOP || state_ == AGV_ERROR) { reason = ErrorCode::RestartDenied; return false; }
    if (state_ == AGV_MOVING || state_ == AGV_LOADING || state_ == AGV_UNLOADING || task_.active) { reason = ErrorCode::TaskBusy; return false; }
    if (!AGVMath::validTask(task)) { reason = ErrorCode::InvalidTask; return false; }
    task_ = task;
    task_.active = true;
    pickupDone_ = servoIssued_ = turning_ = turnInterrupted_ = liftAuthorized_ = waitingForOrder_ = hasOrder_ = reverseHold_ = servoInterrupted_ = false;
    pathIndex_ = 0;
    state_ = AGV_WAITING;
    reason = ErrorCode::None;
    return true;
}
bool ControlLogic::motionClear(const ControlInput& in, ErrorCode& reason) const {
    if (!in.wifi || !in.mqtt) reason = ErrorCode::NotConnected;
    else if (!in.motionConfigured || !in.lineConfigured) reason = ErrorCode::Configuration;
    else if (!in.distanceValid) reason = ErrorCode::SensorInvalid;
    else if (in.obstacle) reason = ErrorCode::Obstacle;
    else reason = ErrorCode::None;
    return reason == ErrorCode::None;
}
void ControlLogic::enterServoState(AGVState state, uint32_t now) {
    state_ = state;
    servoIssued_ = false;
    liftAuthorized_ = !settings_.requireLiftCommands;
    waitingForOrder_ = false;
    phaseStartedMs_ = now;
    turning_ = false;
}
void ControlLogic::beginSegment(uint32_t now) {
    phaseStartedMs_ = now;
    if (!pickupDone_ && std::strcmp(currentNode_, task_.startNode) == 0) {
        enterServoState(AGV_LOADING, now);
    } else if (pathIndex_ + 1 >= task_.pathLength) {
        if (pickupDone_) enterServoState(AGV_UNLOADING, now);
        else fail(ErrorCode::InvalidTask);
    } else {
        if (reverseHold_ && static_cast<uint32_t>(now - reverseStoppedAtMs_) < Config::REVERSE_STOP_MS) {
            state_ = AGV_WAITING;
            return;
        }
        reverseHold_ = false;
        Departure departure = task_.path[pathIndex_].departure;
        if (settings_.requireNodeOrders) {
            if (!hasOrder_ || pendingOrder_.pathIndex != pathIndex_) {
                state_ = AGV_WAITING;
                waitingForOrder_ = true;
                return;
            }
            departure = pendingOrder_.departure;
            hasOrder_ = false;
        }
        waitingForOrder_ = false;
        activeDeparture_ = departure;
        if (departure == Departure::Backward && !settings_.reverseSteeringConfirmed) {
            safeStop(ErrorCode::Configuration);
            return;
        }
        if ((departure == Departure::Left || departure == Departure::Right) &&
            (!settings_.turnMinMs || settings_.turnTimeoutMs <= settings_.turnMinMs)) {
            safeStop(ErrorCode::TurnConfiguration);
            return;
        }
        if (departure == Departure::Straight && !settings_.straightDepartureMs) {
            safeStop(ErrorCode::TurnConfiguration);
            return;
        }
        state_ = AGV_MOVING;
        turning_ = departure == Departure::Left || departure == Departure::Right;
    }
}
bool ControlLogic::setNodeOrder(const AGVCommand& order, uint32_t now, ErrorCode& reason) {
    reason = ErrorCode::NodeOrderMismatch;
    if (!settings_.requireNodeOrders || !task_.active || state_ == AGV_SAFE_STOP || state_ == AGV_ERROR) return false;
    if (std::strcmp(order.taskId, task_.id) != 0 || order.pathIndex >= task_.pathLength - 1 ||
        order.pathIndex < pathIndex_ || order.pathIndex > pathIndex_ + 1 ||
        std::strcmp(order.node, task_.path[order.pathIndex].node) != 0 ||
        std::strcmp(order.nextNode, task_.path[order.pathIndex + 1].node) != 0) return false;
    if (state_ == AGV_MOVING && order.pathIndex == pathIndex_) return false;
    if (order.pathIndex == pathIndex_ + 1 && state_ != AGV_MOVING) return false;
    if (order.departure == Departure::Backward) {
        if (order.pathIndex == 0 || order.node[0] != 'K' || order.nextNode[0] != 'G' ||
            std::strcmp(order.nextNode, task_.path[order.pathIndex - 1].node) != 0) return false;
    }
    if (hasOrder_) { reason = ErrorCode::TaskBusy; return false; }
    pendingOrder_ = order;
    hasOrder_ = true;
    reason = ErrorCode::None;
    if (waitingForOrder_ && order.pathIndex == pathIndex_) beginSegment(now);
    return true;
}
bool ControlLogic::authorizeLift(const AGVCommand& command, const ControlInput& in, ErrorCode& reason) {
    reason = ErrorCode::NodeOrderMismatch;
    if (!settings_.requireLiftCommands || !task_.active ||
        (state_ != AGV_LOADING && state_ != AGV_UNLOADING) || liftAuthorized_ || servoIssued_ ||
        std::strcmp(command.taskId, task_.id) != 0 ||
        std::strcmp(command.node, currentNode_) != 0 ||
        command.pathIndex != pathIndex_ ||
        (state_ == AGV_LOADING && command.type != CommandType::LiftUp) ||
        (state_ == AGV_UNLOADING && command.type != CommandType::LiftDown)) return false;
    if (!in.cargoStable || !in.cargoPresent) { reason = ErrorCode::CargoMissing; return false; }
    if (in.servoBusy || !in.liftConfigured) { reason = ErrorCode::ServoConfiguration; return false; }
    if (!motionClear(in, reason)) return false;
    liftAuthorized_ = true;
    reason = ErrorCode::None;
    return true;
}
bool ControlLogic::start(const ControlInput& in, ErrorCode& reason) {
    if (state_ == AGV_SAFE_STOP || state_ == AGV_ERROR) { reason = ErrorCode::RestartDenied; return false; }
    if (state_ == AGV_MOVING || state_ == AGV_LOADING || state_ == AGV_UNLOADING) { reason = ErrorCode::TaskBusy; return false; }
    if (!task_.active) { reason = ErrorCode::NoTask; return false; }
    if (turnInterrupted_) { reason = ErrorCode::TurnInterrupted; return false; }
    if (servoInterrupted_) { reason = ErrorCode::ServoInterrupted; return false; }
    if (in.servoBusy) { reason = ErrorCode::TaskBusy; return false; }
    if (!motionClear(in, reason)) return false;
    if (!in.liftConfigured || !settings_.servoMoveMs) { reason = ErrorCode::ServoConfiguration; return false; }
    if (std::strcmp(currentNode_, task_.path[pathIndex_].node) != 0) { reason = ErrorCode::NotAtRouteStart; return false; }
    beginSegment(in.now);
    reason = error_;
    return state_ != AGV_SAFE_STOP && state_ != AGV_ERROR;
}
bool ControlLogic::restart(const ControlInput& in, ErrorCode& reason) {
    if (state_ != AGV_SAFE_STOP && state_ != AGV_ERROR) { reason = ErrorCode::RestartDenied; return false; }
    if (turnInterrupted_) { reason = ErrorCode::TurnInterrupted; return false; }
    if (servoInterrupted_) { reason = ErrorCode::ServoInterrupted; return false; }
    if (!motionClear(in, reason)) return false;
    if (AGVMath::lineAction(in.linePattern) == MotorAction::Stop) {
        reason = in.linePattern == 0 ? ErrorCode::LineLost : ErrorCode::AmbiguousLine;
        return false;
    }
    state_ = task_.active ? AGV_WAITING : AGV_IDLE;
    error_ = reason = ErrorCode::None;
    return true;
}
void ControlLogic::reachedNode(const char* node, uint32_t now) {
    if (!AGVMath::validNode(node)) {
        if (state_ == AGV_MOVING) {
            currentNode_[0] = '\0';
            safeStop(ErrorCode::UnknownTag);
        }
        return;
    }
    const bool changed = std::strcmp(currentNode_, node) != 0;
    AGVMath::copyText(currentNode_, sizeof(currentNode_), node);
    if (!changed) return;
    if (state_ == AGV_MOVING) {
        if (turning_ || pathIndex_ + 1 >= task_.pathLength || std::strcmp(node, task_.path[pathIndex_ + 1].node) != 0) {
            safeStop(ErrorCode::UnexpectedNode);
            return;
        }
        ++pathIndex_;
        if (activeDeparture_ == Departure::Backward) {
            reverseHold_ = true;
            reverseStoppedAtMs_ = now;
        }
        beginSegment(now);
    } else if (state_ == AGV_LOADING || state_ == AGV_UNLOADING) {
        safeStop(ErrorCode::UnexpectedNode);
    }
}
void ControlLogic::checkSafety(const ControlInput& in) {
    if (in.wifi) wifiLastGoodMs_ = in.now;
    if (in.mqtt) mqttLastGoodMs_ = in.now;
    if (state_ != AGV_SAFE_STOP && state_ != AGV_ERROR) {
        if (!in.wifi && static_cast<uint32_t>(in.now - wifiLastGoodMs_) >= settings_.wifiTimeoutMs) safeStop(ErrorCode::WifiTimeout);
        else if (!in.mqtt && static_cast<uint32_t>(in.now - mqttLastGoodMs_) >= settings_.mqttTimeoutMs) safeStop(ErrorCode::MqttTimeout);
    }
    if (state_ == AGV_SAFE_STOP || state_ == AGV_ERROR) return;
    if (!in.wifi || !in.mqtt) { pause(); return; }
    if (state_ == AGV_MOVING || state_ == AGV_LOADING || state_ == AGV_UNLOADING) {
        ErrorCode reason;
        if (!motionClear(in, reason)) safeStop(reason);
    }
    if (state_ == AGV_MOVING && pickupDone_ && in.cargoStable && !in.cargoPresent)
        safeStop(ErrorCode::CargoLost);
}
ControlOutput ControlLogic::update(const ControlInput& in) {
    ControlOutput out;
    checkSafety(in);
    if (state_ == AGV_SAFE_STOP || state_ == AGV_ERROR || !in.wifi || !in.mqtt) return out;
    if (state_ == AGV_COMPLETED) { state_ = AGV_IDLE; return out; }
    if (state_ == AGV_WAITING && reverseHold_) beginSegment(in.now);
    if (state_ != AGV_MOVING && state_ != AGV_LOADING && state_ != AGV_UNLOADING) return out;
    ErrorCode reason;
    if (!motionClear(in, reason)) { safeStop(reason); return out; }
    if (state_ == AGV_LOADING || state_ == AGV_UNLOADING) {
        if (!in.liftConfigured || !settings_.servoMoveMs) { fail(ErrorCode::ServoConfiguration); return out; }
        if (!liftAuthorized_) return out;
        if (!servoIssued_) {
            out.servo = state_ == AGV_LOADING ? ServoAction::LiftUp : ServoAction::LiftDown;
            servoIssued_ = true;
            phaseStartedMs_ = in.now;
        } else if (!in.servoBusy && static_cast<uint32_t>(in.now - phaseStartedMs_) >= settings_.servoMoveMs) {
            if (settings_.requireLiftCommands && !in.cargoStable) return out;
            if (state_ == AGV_LOADING) {
                if (settings_.requireLiftCommands && !in.cargoPresent) { safeStop(ErrorCode::CargoMissing); return out; }
                pickupDone_ = true; beginSegment(in.now);
            } else if (!settings_.requireLiftCommands || !in.cargoPresent) { task_.active = false; state_ = AGV_COMPLETED; }
        }
        return out;
    }
    if (in.linePattern == 0) { safeStop(ErrorCode::LineLost); return out; }
    if (turning_) {
        const uint32_t elapsed = in.now - phaseStartedMs_;
        if (elapsed >= settings_.turnTimeoutMs) { safeStop(ErrorCode::TurnTimeout); return out; }
        if (elapsed >= settings_.turnMinMs && in.linePattern == 2) {
            turning_ = false;
            out.motor = MotorAction::Forward;
        } else out.motor = activeDeparture_ == Departure::Left ? MotorAction::PivotLeft : MotorAction::PivotRight;
        return out;
    }
    if (activeDeparture_ == Departure::Backward) { out.motor = MotorAction::Backward; return out; }
    if (activeDeparture_ == Departure::Straight &&
        static_cast<uint32_t>(in.now - phaseStartedMs_) < settings_.straightDepartureMs) {
        out.motor = MotorAction::Forward;
        return out;
    }
    out.motor = AGVMath::lineAction(in.linePattern);
    if (out.motor == MotorAction::Stop) safeStop(in.linePattern == 0 ? ErrorCode::LineLost : ErrorCode::AmbiguousLine);
    return out;
}
