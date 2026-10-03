#pragma once
#include "sensors/BatteryModule.h"
#include "sensors/EncoderModule.h"
#include "sensors/RFIDModule.h"
#include "sensors/SensorModule.h"
#include "actuators/ServoModule.h"
#include "motion/MovementModule.h"
#include "network/MQTTModule.h"
#include "ControlLogic.h"
#include "sensors/CargoModule.h"
#include "actuators/BuzzerModule.h"

class AGVController {
public:
    AGVController();
    void begin();
    void update();
    AGVState state() const { return control_.state(); }
private:
    ControlInput input() const;
    bool consumePriority();
    void handleCommand(const AGVCommand& command, bool inhibited);
    void publishTelemetry();
    void observeState();
    MotorModule motor_;
    SensorModule sensor_;
    EncoderModule encoder_;
    RFIDModule rfid_;
    ServoModule servo_;
    LineFollowModule line_;
    BuzzerModule buzzer_;
    MovementModule movement_;
    BatteryModule battery_;
    CargoModule cargo_;
    WiFiModule wifi_;
    MQTTModule mqtt_;
    ControlLogic control_;
    uint32_t lastTelemetryMs_ = 0;
    AGVState lastState_ = AGV_IDLE;
    ErrorCode lastError_ = ErrorCode::None;
    bool pickupIrReported_ = false;
};
