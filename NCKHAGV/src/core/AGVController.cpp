#include "AGVController.h"
#include <cstring>

AGVController::AGVController() : line_(motor_), movement_(motor_, line_, buzzer_), mqtt_(wifi_),
    control_({Config::WIFI_TIMEOUT_MS, Config::MQTT_TIMEOUT_MS, Config::TURN_MIN_MS,
              Config::TURN_TIMEOUT_MS, Config::STRAIGHT_DEPARTURE_MS, Config::SERVO_MOVE_TIME_MS,
              true, true, Config::REVERSE_STEERING_CONFIRMED}) {}
void AGVController::begin() {
    buzzer_.begin();
    motor_.begin();
    movement_.begin();
    line_.begin();
    sensor_.begin();
    encoder_.begin();
    rfid_.begin();
    servo_.begin();
    battery_.begin();
    cargo_.begin();
    control_.begin(millis());
    if (!mqtt_.begin()) control_.fail(ErrorCode::QueueOverflow);
    if (Config::DEBUG_ENABLED) Serial.println("[AGV] Initialized; motion requires calibrated Config.h and explicit commands");
}
ControlInput AGVController::input() const {
    ControlInput result;
    const auto readings = line_.readSensors();
    result.now = millis();
    result.wifi = wifi_.isConnected();
    result.mqtt = mqtt_.isConnected();
    result.motionConfigured = motor_.configured() && Config::OBSTACLE_DISTANCE_CM > 0 &&
        Config::ENCODER_PPR > 0 && Config::WHEEL_DIAMETER_CM > 0;
    result.liftConfigured = servo_.liftConfigured();
    result.distanceValid = sensor_.valid();
    result.obstacle = sensor_.isObstacleDetected();
    result.servoBusy = servo_.busy();
    result.lineConfigured = readings.configured;
    result.linePattern = readings.pattern;
    result.cargoStable = cargo_.stable();
    result.cargoPresent = cargo_.present();
    return result;
}
bool AGVController::consumePriority() {
    bool inhibited = false;
    if (mqtt_.consumeStop()) { control_.pause(); inhibited = true; }
    if (mqtt_.consumeEmergencyStop()) { control_.safeStop(ErrorCode::EmergencyStop); inhibited = true; }
    if (mqtt_.consumeQueueFault()) { control_.safeStop(ErrorCode::QueueOverflow); inhibited = true; }
    if (inhibited) movement_.stop();
    return inhibited;
}
void AGVController::handleCommand(const AGVCommand& command, bool inhibited) {
    ErrorCode reason = ErrorCode::None;
    bool accepted = false;
    const ControlInput in = input();
    if (command.type == CommandType::EmergencyStop) {
        control_.safeStop(ErrorCode::EmergencyStop);
        movement_.stop();
        accepted = true;
    } else if (command.type == CommandType::Stop) {
        control_.pause();
        movement_.stop();
        accepted = true;
    } else if (inhibited || command.session != mqtt_.session() ||
               static_cast<uint32_t>(in.now - command.receivedAtMs) > Config::COMMAND_MAX_AGE_MS) {
        reason = ErrorCode::StaleCommand;
    } else {
        switch (command.type) {
            case CommandType::Task: {
                bool mapped = true;
                for (size_t i = 0; i < command.task.pathLength; ++i)
                    if (!rfid_.hasMapping(command.task.path[i].node)) mapped = false;
                if (!mapped) reason = ErrorCode::Configuration;
                else accepted = control_.setTask(command.task, reason);
                break;
            }
            case CommandType::Start: accepted = control_.start(in, reason); break;
            case CommandType::NodeOrder: accepted = control_.setNodeOrder(command, in.now, reason); break;
            case CommandType::Restart: accepted = control_.restart(in, reason); break;
            case CommandType::CancelTask:
                movement_.stop();
                control_.cancelTask();
                accepted = true;
                break;
            case CommandType::LiftUp: case CommandType::LiftDown:
                if (movement_.action() != MotorAction::Stop) reason = ErrorCode::TaskBusy;
                else accepted = control_.authorizeLift(command, in, reason);
                break;
            default: reason = ErrorCode::InvalidCommand; break;
        }
    }
    if (Config::DEBUG_ENABLED) Serial.printf("[CMD] id=%s accepted=%d error=%s\n", command.id, accepted, errorName(reason));
    mqtt_.publishCommandResult(command.id, accepted, reason, millis());
}
void AGVController::update() {
    cargo_.update(millis());
    sensor_.update();
    encoder_.update();
    servo_.update();
    bool inhibited = consumePriority();
    control_.checkSafety(input());
    if (control_.state() != AGV_MOVING) movement_.stop();
    if (rfid_.readCard()) {
        const RFIDEvent& event = rfid_.event();
        if (!event.node[0]) {
            control_.reachedNode("", millis());
            if (control_.state() != AGV_MOVING) movement_.stop();
        }
        else if (event.newNode) {
            char fromNode[NODE_TEXT_SIZE]{};
            AGVMath::copyText(fromNode, sizeof(fromNode), control_.currentNode());
            const bool reverse = control_.segmentReverse();
            control_.reachedNode(event.node, millis());
            if (control_.state() != AGV_MOVING) movement_.stop();
            // Capture and reset BOTH counters atomically; the event retains the completed segment.
            const EncoderData segment = encoder_.snapshotAndReset();
            mqtt_.publishEvent("AGV_REACHED_NODE", event.node, segment, millis(), control_.task().id, fromNode, reverse);
            publishTelemetry();
        }
    }
    sensor_.update();
    control_.checkSafety(input());
    if (control_.state() != AGV_MOVING) movement_.stop();
    if (control_.state() == AGV_LOADING && cargo_.stable() && !pickupIrReported_) {
        mqtt_.publishCargo("AGV_PICKUP_IR", control_.currentNode(), control_.task().id, cargo_.present(), millis());
        pickupIrReported_ = true;
    }
    if (control_.state() != AGV_LOADING) pickupIrReported_ = false;
    inhibited = consumePriority() || inhibited;
    AGVCommand command;
    for (size_t i = 0; i < Config::COMMAND_QUEUE_LENGTH && mqtt_.popCommand(command); ++i) {
        inhibited = consumePriority() || inhibited;
        handleCommand(command, inhibited);
        if (command.type == CommandType::Stop || command.type == CommandType::EmergencyStop) inhibited = true;
    }
    inhibited = consumePriority() || inhibited;
    if (inhibited) control_.pause();
    sensor_.update();
    const ControlOutput output = control_.update(input());
    movement_.apply(output.motor);
    if (output.servo != ServoAction::None) {
        movement_.stop();
        const bool moved = output.servo == ServoAction::LiftUp ? servo_.liftUp() : servo_.liftDown();
        if (!moved) control_.fail(ErrorCode::ServoConfiguration);
    }
    if (control_.state() != AGV_MOVING) movement_.stop();
    battery_.update();
    observeState();
    if (static_cast<uint32_t>(millis() - lastTelemetryMs_) >= Config::TELEMETRY_INTERVAL_MS) publishTelemetry();
}
void AGVController::observeState() {
    if (lastState_ == control_.state() && lastError_ == control_.error()) return;
    if (Config::DEBUG_ENABLED) Serial.printf("[STATE] %s -> %s error=%s node=%s\n",
        stateName(lastState_), stateName(control_.state()), errorName(control_.error()), control_.currentNode());
    if (control_.error() != ErrorCode::None && control_.error() != lastError_) mqtt_.publishError(control_.error(), millis());
    if (control_.state() == AGV_COMPLETED) mqtt_.publishEvent("AGV_TASK_COMPLETED", control_.currentNode(), encoder_.snapshot(), millis(), control_.task().id);
    lastState_ = control_.state();
    lastError_ = control_.error();
    publishTelemetry();
}
void AGVController::publishTelemetry() {
    TelemetrySnapshot snapshot;
    snapshot.state = control_.state();
    snapshot.error = control_.error();
    snapshot.encoder = encoder_.snapshot();
    AGVMath::copyText(snapshot.position.node, sizeof(snapshot.position.node), control_.currentNode());
    if (control_.task().active && control_.pathIndex() + 1 < control_.task().pathLength)
        AGVMath::copyText(snapshot.nextNode, sizeof(snapshot.nextNode),
                          control_.task().path[control_.pathIndex() + 1].node);
    snapshot.position.distanceCm = snapshot.encoder.averageDistanceCm;
    AGVMath::copyText(snapshot.taskId, sizeof(snapshot.taskId), control_.task().id);
    snapshot.taskActive = control_.task().active;
    snapshot.pathIndex = control_.pathIndex();
    snapshot.pathLength = control_.task().pathLength;
    snapshot.motor = movement_.action();
    snapshot.leftDuty = motor_.leftDuty();
    snapshot.rightDuty = motor_.rightDuty();
    snapshot.wifi = wifi_.isConnected();
    snapshot.mqtt = mqtt_.isConnected();
    snapshot.ultrasonicCm = sensor_.readDistanceCm();
    snapshot.batteryAdcMv = battery_.adcMillivolts();
    snapshot.batteryCalibrated = battery_.calibrated();
    snapshot.batteryVoltage = battery_.readVoltage();
    snapshot.batteryPercentage = battery_.getBatteryPercentage();
    snapshot.cargoStable = cargo_.stable();
    snapshot.cargoPresent = cargo_.present();
    snapshot.buzzerOn = AGVMath::reverseBuzzerRequired(movement_.action(), motor_.leftDuty(), motor_.rightDuty());
    snapshot.uptimeMs = millis();
    mqtt_.publishStatus(snapshot);
    lastTelemetryMs_ = snapshot.uptimeMs;
}
