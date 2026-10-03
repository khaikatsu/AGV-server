#pragma once

#include "Types.h"

namespace AGVMath {
float distanceCm(uint64_t pulses, uint32_t ppr, float diameterCm);
float batteryVoltage(uint32_t adcMv, float scale, float offset);
float batteryPercentage(float voltage, float emptyVoltage, float fullVoltage);
uint8_t normalizeLine(int left, int center, int right, int activeState);
MotorAction lineAction(uint8_t pattern);
bool validNode(const char* node);
bool validTask(const AGVTask& task);
bool copyText(char* destination, size_t capacity, const char* source);
bool reverseBuzzerRequired(MotorAction action, int leftDuty, int rightDuty);
}

class ControlLogic {
public:
    explicit ControlLogic(SafetySettings settings);
    void begin(uint32_t now);
    void checkSafety(const ControlInput& input);
    ControlOutput update(const ControlInput& input);
    bool setTask(const AGVTask& task, ErrorCode& reason);
    bool setNodeOrder(const AGVCommand& order, uint32_t now, ErrorCode& reason);
    bool authorizeLift(const AGVCommand& command, const ControlInput& input, ErrorCode& reason);
    bool start(const ControlInput& input, ErrorCode& reason);
    bool restart(const ControlInput& input, ErrorCode& reason);
    void pause();
    void cancelTask();
    void reachedNode(const char* node, uint32_t now);
    void safeStop(ErrorCode reason);
    void fail(ErrorCode reason);
    AGVState state() const { return state_; }
    ErrorCode error() const { return error_; }
    const AGVTask& task() const { return task_; }
    const char* currentNode() const { return currentNode_; }
    uint8_t pathIndex() const { return pathIndex_; }
    bool pickupDone() const { return pickupDone_; }
    bool waitingForOrder() const { return waitingForOrder_; }
    bool segmentReverse() const { return activeDeparture_ == Departure::Backward && state_ == AGV_MOVING; }
private:
    void beginSegment(uint32_t now);
    void enterServoState(AGVState state, uint32_t now);
    bool motionClear(const ControlInput& input, ErrorCode& reason) const;
    SafetySettings settings_;
    AGVState state_ = AGV_IDLE;
    ErrorCode error_ = ErrorCode::None;
    AGVTask task_{};
    char currentNode_[NODE_TEXT_SIZE]{};
    uint8_t pathIndex_ = 0;
    bool pickupDone_ = false, servoIssued_ = false;
    bool liftAuthorized_ = false, waitingForOrder_ = false, reverseHold_ = false;
    bool servoInterrupted_ = false;
    AGVCommand pendingOrder_{};
    bool hasOrder_ = false;
    Departure activeDeparture_ = Departure::FollowLine;
    uint32_t reverseStoppedAtMs_ = 0;
    bool turning_ = false, turnInterrupted_ = false;
    uint32_t phaseStartedMs_ = 0, wifiLastGoodMs_ = 0, mqttLastGoodMs_ = 0;
};
