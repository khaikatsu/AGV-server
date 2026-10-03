#include <unity.h>
#include "core/ControlLogic.h"
#include "core/CommandCodec.h"
#include <cmath>
#include <cstring>
#include <string>

void setUp() {}
void tearDown() {}
namespace {
SafetySettings settings() { return {100, 100, 20, 100, 30, 50}; }
ControlInput healthy(uint32_t now = 0) {
    ControlInput in;
    in.now = now;
    in.wifi = in.mqtt = in.motionConfigured = in.liftConfigured = in.distanceValid = in.lineConfigured = true;
    in.linePattern = 2;
    in.cargoStable = in.cargoPresent = true;
    return in;
}
AGVTask task() {
    AGVTask value;
    AGVMath::copyText(value.id, sizeof(value.id), "task-1");
    AGVMath::copyText(value.startNode, sizeof(value.startNode), "K1");
    AGVMath::copyText(value.targetNode, sizeof(value.targetNode), "K2");
    value.pathLength = 3;
    const char* nodes[] = {"B1", "K1", "K2"};
    for (size_t i = 0; i < 3; ++i) AGVMath::copyText(value.path[i].node, sizeof(value.path[i].node), nodes[i]);
    return value;
}
void moving(ControlLogic& core, uint32_t now = 0, AGVTask route = task()) {
    core.begin(now);
    core.update(healthy(now));
    core.reachedNode("B1", now);
    ErrorCode reason;
    TEST_ASSERT_TRUE(core.setTask(route, reason));
    TEST_ASSERT_TRUE(core.start(healthy(now), reason));
}
bool decode(const std::string& json, AGVCommand& result, ErrorCode& reason) {
    return CommandCodec::decode(reinterpret_cast<const uint8_t*>(json.data()), json.size(), result, reason);
}
const char* taskJson = R"({"version":1,"id":"cmd-1","session":123,"command":"TASK","task":{"id":"task-1","startNode":"K1","targetNode":"K2","path":["B1","K1","K2"]}})";
}

void test_official_pin_map_and_uniqueness() {
    TEST_ASSERT_TRUE(Config::pinsUnique());
    const int expected[] = {14,27,26,25,33,32,39,34,35,12,13,36,18,19,23,5,22,4,16,15,17,21,0,2};
    for (size_t i = 0; i < Config::ALL_PINS.size(); ++i) TEST_ASSERT_EQUAL_INT(expected[i], Config::ALL_PINS[i]);
}
void test_pwm_timer_partition() {
    for (int motor : Config::MOTOR_PWM_CHANNELS)
        for (int servo : Config::SERVO_PWM_CHANNELS) TEST_ASSERT_NOT_EQUAL(motor / 2, servo / 2);
}
void test_unknown_distance_is_not_fake_zero() {
    TEST_ASSERT_TRUE(std::isnan(AGVMath::distanceCm(20, 0, 0)));
    TEST_ASSERT_TRUE(std::isnan(AGVMath::distanceCm(20, 20, -1)));
}
void test_encoder_distance_formula() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 31.4159265f, AGVMath::distanceCm(40, 20, 5));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, AGVMath::distanceCm(0, 20, 5));
}
void test_battery_unknown_and_formula() {
    TEST_ASSERT_TRUE(std::isnan(AGVMath::batteryVoltage(1000, 0, 0)));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.2f, AGVMath::batteryVoltage(1000, 5, 0.2f));
    TEST_ASSERT_TRUE(std::isnan(AGVMath::batteryPercentage(5, 0, 0)));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, AGVMath::batteryPercentage(3, 4, 6));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50, AGVMath::batteryPercentage(5, 4, 6));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100, AGVMath::batteryPercentage(7, 4, 6));
}
void test_line_polarity_and_actions() {
    TEST_ASSERT_EQUAL_UINT8(2, AGVMath::normalizeLine(0,1,0,1));
    TEST_ASSERT_EQUAL_UINT8(2, AGVMath::normalizeLine(1,0,1,0));
    TEST_ASSERT_EQUAL_UINT8(0, AGVMath::normalizeLine(0,1,0,-1));
    const MotorAction actions[] = {MotorAction::Stop,MotorAction::CorrectRight,MotorAction::Forward,
        MotorAction::CorrectRight,MotorAction::CorrectLeft,MotorAction::Forward,MotorAction::CorrectLeft,MotorAction::Forward};
    for (uint8_t i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_INT(static_cast<int>(actions[i]), static_cast<int>(AGVMath::lineAction(i)));
}
void test_task_rejects_bad_node_or_destination() {
    auto route = task();
    TEST_ASSERT_TRUE(AGVMath::validTask(route));
    AGVMath::copyText(route.targetNode, sizeof(route.targetNode), "GS2");
    TEST_ASSERT_FALSE(AGVMath::validTask(route));
    route = task();
    AGVMath::copyText(route.path[1].node, sizeof(route.path[1].node), "X2");
    TEST_ASSERT_FALSE(AGVMath::validTask(route));
}
void test_task_rejects_duplicate_missing_pickup_empty() {
    auto route = task();
    AGVMath::copyText(route.path[1].node, sizeof(route.path[1].node), "B1");
    TEST_ASSERT_FALSE(AGVMath::validTask(route));
    route = task(); route.pathLength = 0;
    TEST_ASSERT_FALSE(AGVMath::validTask(route));
    route = task(); AGVMath::copyText(route.startNode, sizeof(route.startNode), "GS2");
    TEST_ASSERT_FALSE(AGVMath::validTask(route));
}
void test_start_requires_task_and_confirmed_node() {
    ControlLogic core(settings()); core.begin(0);
    ErrorCode reason;
    TEST_ASSERT_FALSE(core.start(healthy(), reason)); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::NoTask), static_cast<int>(reason));
    TEST_ASSERT_TRUE(core.setTask(task(), reason));
    TEST_ASSERT_FALSE(core.start(healthy(), reason)); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::NotAtRouteStart), static_cast<int>(reason));
}
void test_start_rejects_configuration_and_busy_servo() {
    ControlLogic core(settings()); core.begin(0); core.reachedNode("B1",0);
    ErrorCode reason; core.setTask(task(), reason);
    auto in = healthy(); in.motionConfigured = false;
    TEST_ASSERT_FALSE(core.start(in,reason));
    in = healthy(); in.liftConfigured = false; TEST_ASSERT_FALSE(core.start(in,reason));
    in = healthy(); in.servoBusy = true; TEST_ASSERT_FALSE(core.start(in,reason));
}
void test_full_pickup_and_delivery_cycle() {
    ControlLogic core(settings()); moving(core);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Forward), static_cast<int>(core.update(healthy()).motor));
    core.reachedNode("K1",10); TEST_ASSERT_EQUAL_INT(AGV_LOADING,core.state());
    auto out = core.update(healthy(10));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(out.motor));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftUp),static_cast<int>(out.servo));
    auto in = healthy(60); in.servoBusy = true; core.update(in); TEST_ASSERT_EQUAL_INT(AGV_LOADING,core.state());
    core.update(healthy(61)); TEST_ASSERT_EQUAL_INT(AGV_MOVING,core.state()); TEST_ASSERT_TRUE(core.pickupDone());
    core.reachedNode("K2",100); TEST_ASSERT_EQUAL_INT(AGV_UNLOADING,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftDown),static_cast<int>(core.update(healthy(100)).servo));
    core.update(healthy(150)); TEST_ASSERT_EQUAL_INT(AGV_COMPLETED,core.state()); TEST_ASSERT_FALSE(core.task().active);
    core.update(healthy(151)); TEST_ASSERT_EQUAL_INT(AGV_IDLE,core.state());
}
void test_same_node_does_not_advance_path() {
    ControlLogic core(settings()); moving(core); core.reachedNode("B1",10);
    TEST_ASSERT_EQUAL_UINT8(0,core.pathIndex()); TEST_ASSERT_EQUAL_INT(AGV_MOVING,core.state());
}
void test_unexpected_node_latches_stop() {
    ControlLogic core(settings()); moving(core); core.reachedNode("K2",10);
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::UnexpectedNode),static_cast<int>(core.error()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(healthy(11)).motor));
}
void test_unknown_tag_latches_stop() {
    ControlLogic core(settings()); moving(core); core.reachedNode("",10); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
    TEST_ASSERT_EQUAL_STRING("", core.currentNode());
    ErrorCode reason; TEST_ASSERT_TRUE(core.restart(healthy(11),reason));
    TEST_ASSERT_FALSE(core.start(healthy(12),reason));
}
void test_node_change_during_loading_stops() {
    ControlLogic core(settings()); moving(core); core.reachedNode("K1",10); core.reachedNode("K2",11);
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_wifi_grace_pauses_then_timeout_latches() {
    ControlLogic core(settings()); moving(core);
    auto in = healthy(10); in.wifi = false;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(in).motor)); TEST_ASSERT_EQUAL_INT(AGV_WAITING,core.state());
    in.now = 100; core.update(in); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::WifiTimeout),static_cast<int>(core.error()));
}
void test_mqtt_timeout_latches() {
    ControlLogic core(settings()); moving(core);
    auto in = healthy(100); in.mqtt = false; core.update(in);
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::MqttTimeout),static_cast<int>(core.error()));
}
void test_reconnection_does_not_auto_move_during_grace() {
    ControlLogic core(settings()); moving(core); auto in = healthy(10); in.mqtt = false; core.update(in);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(healthy(20)).motor)); TEST_ASSERT_EQUAL_INT(AGV_WAITING,core.state());
    ErrorCode reason; TEST_ASSERT_TRUE(core.start(healthy(21),reason));
}
void test_zero_timeout_safe_stop_in_idle() {
    ControlLogic core({0,0,0,0,0,0}); core.begin(0); core.update(ControlInput{}); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_timeout_across_millis_wrap() {
    ControlLogic core(settings()); moving(core,UINT32_MAX - 50); auto in = healthy(49); in.wifi = false; core.update(in);
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_emergency_stop_requires_restart_then_start() {
    ControlLogic core(settings()); moving(core); core.safeStop(ErrorCode::EmergencyStop);
    ErrorCode reason; TEST_ASSERT_FALSE(core.start(healthy(1),reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(healthy(1)).motor));
    TEST_ASSERT_TRUE(core.restart(healthy(2),reason)); TEST_ASSERT_EQUAL_INT(AGV_WAITING,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(healthy(3)).motor));
    TEST_ASSERT_TRUE(core.start(healthy(4),reason));
}
void test_restart_rejects_obstacle_lost_line_and_network() {
    ControlLogic core(settings()); moving(core); core.safeStop(ErrorCode::EmergencyStop); ErrorCode reason;
    auto in = healthy(); in.obstacle = true; TEST_ASSERT_FALSE(core.restart(in,reason));
    in = healthy(); in.linePattern = 0; TEST_ASSERT_FALSE(core.restart(in,reason));
    in = healthy(); in.mqtt = false; TEST_ASSERT_FALSE(core.restart(in,reason));
}
void test_obstacle_stops_without_servo() {
    ControlLogic core(settings()); moving(core); auto in = healthy(1); in.obstacle = true;
    auto out = core.update(in); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(out.motor));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::None),static_cast<int>(out.servo));
}
void test_invalid_ultrasonic_stops() {
    ControlLogic core(settings()); moving(core); auto in = healthy(1); in.distanceValid = false; core.update(in);
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::SensorInvalid),static_cast<int>(core.error()));
}
void test_only_000_stops_for_line_loss() {
    for (uint8_t pattern = 1; pattern < 8; ++pattern) {
        ControlLogic core(settings()); moving(core); auto in = healthy(1); in.linePattern = pattern;
        TEST_ASSERT_NOT_EQUAL(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(in).motor));
        TEST_ASSERT_EQUAL_INT(AGV_MOVING, core.state());
    }
    ControlLogic core(settings()); moving(core); auto in = healthy(1); in.linePattern = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(in).motor));
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::LineLost), static_cast<int>(core.error()));
}
void test_restart_accepts_101_and_111_but_not_000() {
    for (uint8_t pattern : {5, 7}) {
        ControlLogic core(settings()); moving(core); core.safeStop(ErrorCode::EmergencyStop);
        auto in = healthy(1); in.linePattern = pattern; ErrorCode reason;
        TEST_ASSERT_TRUE(core.restart(in, reason));
        TEST_ASSERT_EQUAL_INT(AGV_WAITING, core.state());
    }
}
void test_turn_requires_calibration() {
    auto route = task(); route.path[0].departure = Departure::Left;
    ControlLogic core({100,100,0,0,0,50}); core.begin(0); core.reachedNode("B1",0); ErrorCode reason; core.setTask(route,reason);
    TEST_ASSERT_FALSE(core.start(healthy(),reason)); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_turn_completes_only_after_minimum_and_center() {
    auto route = task(); route.path[0].departure = Departure::Left; ControlLogic core(settings()); moving(core,0,route);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::PivotLeft),static_cast<int>(core.update(healthy(19)).motor));
    auto in = healthy(20); in.linePattern = 7; TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::PivotLeft),static_cast<int>(core.update(in).motor));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Forward),static_cast<int>(core.update(healthy(21)).motor));
}
void test_turn_timeout_stops() {
    auto route = task(); route.path[0].departure = Departure::Right; ControlLogic core(settings()); moving(core,0,route);
    auto in = healthy(100); in.linePattern = 7; core.update(in); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::TurnTimeout),static_cast<int>(core.error()));
}
void test_turn_stops_when_all_line_sensors_clear() {
    auto route = task(); route.path[0].departure = Departure::Left; ControlLogic core(settings()); moving(core,0,route);
    auto in = healthy(1); in.linePattern = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(in).motor));
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::LineLost), static_cast<int>(core.error()));
}
void test_interrupted_turn_requires_cancel() {
    auto route = task(); route.path[0].departure = Departure::Right; ControlLogic core(settings()); moving(core,0,route); core.safeStop(ErrorCode::EmergencyStop);
    ErrorCode reason; TEST_ASSERT_FALSE(core.restart(healthy(1),reason)); TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::TurnInterrupted),static_cast<int>(reason));
    core.cancelTask(); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_TRUE(core.restart(healthy(2),reason));
}
void test_straight_crossing_never_ignores_lost_line() {
    auto route = task(); route.path[0].departure = Departure::Straight; ControlLogic core(settings()); moving(core,0,route);
    auto in = healthy(1); in.linePattern = 7; TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Forward),static_cast<int>(core.update(in).motor));
    in.now = 2; in.linePattern = 0; core.update(in); TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_stop_and_cancel_do_not_clear_latch() {
    ControlLogic core(settings()); moving(core); core.safeStop(ErrorCode::EmergencyStop); core.pause(); core.cancelTask();
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state()); TEST_ASSERT_FALSE(core.task().active);
}
void test_active_task_cannot_be_overwritten() {
    ControlLogic core(settings()); moving(core); ErrorCode reason; TEST_ASSERT_FALSE(core.setTask(task(),reason));
}
void test_parser_accepts_task_and_departures() {
    AGVCommand command; ErrorCode reason; TEST_ASSERT_TRUE(decode(taskJson,command,reason)); TEST_ASSERT_EQUAL_UINT8(3,command.task.pathLength);
    TEST_ASSERT_EQUAL_STRING("K1",command.task.startNode);
    TEST_ASSERT_TRUE(decode(R"({"version":1,"id":"c","session":123,"command":"TASK","task":{"id":"t","startNode":"K1","targetNode":"K2","path":[{"node":"B1","departure":"LEFT"},"K1","K2"]}})",command,reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Departure::Left),static_cast<int>(command.task.path[0].departure));
}
void test_parser_accepts_all_simple_commands() {
    for (const char* name : {"START","STOP","EMERGENCY_STOP","RESTART","CANCEL_TASK"}) {
        AGVCommand command; ErrorCode reason;
        TEST_ASSERT_TRUE(decode(std::string("{\"version\":1,\"id\":\"c\",\"session\":123,\"command\":\"") + name + "\"}",command,reason));
    }
}
void test_parser_rejects_bad_json_version_types_unknown() {
    for (const char* json : {"{", "[]", R"({"version":2,"id":"c","session":123,"command":"START"})",
         R"({"version":1,"id":"c","session":"123","command":"START"})", R"({"version":1,"id":"c","session":123,"command":"RUN"})",
         R"({"version":1,"id":123,"session":123,"command":"START"})", R"({"version":1,"id":"bad id","session":123,"command":"START"})"}) {
        AGVCommand command; ErrorCode reason; TEST_ASSERT_FALSE(decode(json,command,reason));
    }
}
void test_parser_rejects_zero_session_for_motion_but_allows_stop() {
    AGVCommand command; ErrorCode reason;
    TEST_ASSERT_FALSE(decode(R"({"version":1,"id":"c","session":0,"command":"START"})",command,reason));
    TEST_ASSERT_TRUE(decode(R"({"version":1,"id":"c","session":0,"command":"EMERGENCY_STOP"})",command,reason));
}
void test_parser_rejects_bad_task_and_oversized_payload() {
    AGVCommand command; ErrorCode reason;
    TEST_ASSERT_FALSE(decode(std::string(Config::MAX_COMMAND_BYTES+1,' '),command,reason));
    TEST_ASSERT_FALSE(decode(R"({"version":1,"id":"c","session":1,"command":"TASK","task":{"id":"t","startNode":"B1","targetNode":"K1","path":["B1","GS2"]}})",command,reason));
    TEST_ASSERT_FALSE(decode(R"({"version":1,"id":"c","session":1,"command":"TASK","task":{"id":"t","startNode":"B1","targetNode":"K1","path":[{"node":"B1","departure":"UTURN"},"K1"]}})",command,reason));
}
void test_parser_rejects_long_identifier_and_path() {
    AGVCommand command; ErrorCode reason;
    TEST_ASSERT_FALSE(decode(std::string("{\"version\":1,\"id\":\"")+std::string(COMMAND_ID_SIZE,'a')+"\",\"session\":1,\"command\":\"START\"}",command,reason));
    std::string json = R"({"version":1,"id":"c","session":1,"command":"TASK","task":{"id":"t","startNode":"B1","targetNode":"B1","path":[)";
    for (size_t i = 0; i <= Config::MAX_PATH_NODES; ++i) json += std::string(i ? "," : "") + (i % 2 ? "\"K1\"" : "\"B1\"");
    TEST_ASSERT_FALSE(decode(json+"]}}",command,reason));
}
void test_parser_rejects_trailing_data_and_nul() {
    AGVCommand command; ErrorCode reason;
    TEST_ASSERT_FALSE(decode(std::string(taskJson)+"{}",command,reason));
    TEST_ASSERT_FALSE(decode(std::string(taskJson)+std::string(1,'\0'),command,reason));
    TEST_ASSERT_TRUE(decode(std::string(taskJson)+" \r\n",command,reason));
    TEST_ASSERT_FALSE(decode(R"({"version":1,"id":"c","session":1,"command":"START\u0000STOP"})",command,reason));
}
void test_sensor_failure_during_loading_suppresses_servo() {
    ControlLogic core(settings()); moving(core); core.reachedNode("K1",10);
    auto in = healthy(10); in.distanceValid = false;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::None),static_cast<int>(core.update(in).servo));
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP,core.state());
}
void test_error_state_stays_stopped_until_explicit_restart() {
    ControlLogic core(settings()); moving(core); core.fail(ErrorCode::ServoConfiguration); ErrorCode reason;
    TEST_ASSERT_FALSE(core.setTask(task(),reason)); TEST_ASSERT_FALSE(core.start(healthy(),reason));
    TEST_ASSERT_EQUAL_INT(AGV_ERROR,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop),static_cast<int>(core.update(healthy()).motor));
}
void test_single_node_pickup_and_drop_cycle() {
    auto route = task(); route.pathLength = 1;
    AGVMath::copyText(route.startNode,sizeof(route.startNode),"B1");
    AGVMath::copyText(route.targetNode,sizeof(route.targetNode),"B1");
    ControlLogic core(settings()); moving(core,0,route);
    TEST_ASSERT_EQUAL_INT(AGV_LOADING,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftUp),static_cast<int>(core.update(healthy(0)).servo));
    core.update(healthy(50)); TEST_ASSERT_EQUAL_INT(AGV_UNLOADING,core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftDown),static_cast<int>(core.update(healthy(51)).servo));
    core.update(healthy(101)); TEST_ASSERT_EQUAL_INT(AGV_COMPLETED,core.state());
}
void test_reverse_buzzer_tracks_applied_motor() {
    TEST_ASSERT_TRUE(AGVMath::reverseBuzzerRequired(MotorAction::Backward, -190, -210));
    TEST_ASSERT_FALSE(AGVMath::reverseBuzzerRequired(MotorAction::Backward, 0, 0));
    TEST_ASSERT_FALSE(AGVMath::reverseBuzzerRequired(MotorAction::Stop, -190, -210));
    TEST_ASSERT_FALSE(AGVMath::reverseBuzzerRequired(MotorAction::Forward, 190, 210));
}
void test_26_nodes_and_reverse_order_validation() {
    TEST_ASSERT_EQUAL_UINT(26, Config::RFID_NODES.size());
    for (const auto& node : Config::RFID_NODES) TEST_ASSERT_TRUE(AGVMath::validNode(node.node));
    AGVTask route;
    AGVMath::copyText(route.id, sizeof(route.id), "trip-1");
    AGVMath::copyText(route.startNode, sizeof(route.startNode), "K1");
    AGVMath::copyText(route.targetNode, sizeof(route.targetNode), "K2");
    route.pathLength = 4;
    const char* nodes[] = {"G1", "K1", "G1", "K2"};
    for (size_t i = 0; i < 4; ++i)
        AGVMath::copyText(route.path[i].node, sizeof(route.path[i].node), nodes[i]);
    SafetySettings policy = settings();
    policy.requireNodeOrders = policy.requireLiftCommands = policy.reverseSteeringConfirmed = true;
    ControlLogic core(policy); core.begin(0); core.reachedNode("G1", 0);
    ErrorCode reason;
    TEST_ASSERT_TRUE(core.setTask(route, reason));
    TEST_ASSERT_TRUE(core.start(healthy(), reason));
    TEST_ASSERT_EQUAL_INT(AGV_WAITING, core.state());
    AGVCommand order;
    order.type = CommandType::NodeOrder;
    AGVMath::copyText(order.taskId, sizeof(order.taskId), "trip-1");
    AGVMath::copyText(order.node, sizeof(order.node), "G1");
    AGVMath::copyText(order.nextNode, sizeof(order.nextNode), "K1");
    order.pathIndex = 0; order.departure = Departure::Straight;
    TEST_ASSERT_TRUE(core.setNodeOrder(order, 1, reason));
    TEST_ASSERT_EQUAL_INT(AGV_MOVING, core.state());
    core.reachedNode("K1", 10);
    TEST_ASSERT_EQUAL_INT(AGV_LOADING, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::None), static_cast<int>(core.update(healthy(11)).servo));
    order.pathIndex = 1; order.departure = Departure::Backward;
    AGVMath::copyText(order.node, sizeof(order.node), "K1");
    AGVMath::copyText(order.nextNode, sizeof(order.nextNode), "G1");
    TEST_ASSERT_TRUE(core.setNodeOrder(order, 12, reason));
    AGVCommand lift;
    lift.type = CommandType::LiftUp; lift.pathIndex = 1;
    AGVMath::copyText(lift.taskId, sizeof(lift.taskId), "trip-1");
    AGVMath::copyText(lift.node, sizeof(lift.node), "K1");
    TEST_ASSERT_TRUE(core.authorizeLift(lift, healthy(13), reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftUp), static_cast<int>(core.update(healthy(13)).servo));
    core.update(healthy(63));
    TEST_ASSERT_EQUAL_INT(AGV_MOVING, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Backward), static_cast<int>(core.update(healthy(64)).motor));
    core.reachedNode("G1", 70);
    TEST_ASSERT_EQUAL_INT(AGV_WAITING, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(healthy(71)).motor));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(healthy(220)).motor));
    TEST_ASSERT_TRUE(core.waitingForOrder());
    order.pathIndex = 2; order.departure = Departure::Straight;
    AGVMath::copyText(order.node, sizeof(order.node), "G1");
    AGVMath::copyText(order.nextNode, sizeof(order.nextNode), "K2");
    TEST_ASSERT_TRUE(core.setNodeOrder(order, 221, reason));
    TEST_ASSERT_EQUAL_INT(AGV_MOVING, core.state());
    auto lost = healthy(222); lost.cargoPresent = false;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MotorAction::Stop), static_cast<int>(core.update(lost).motor));
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::CargoLost), static_cast<int>(core.error()));
}
void test_parser_requires_bound_node_and_lift_commands() {
    AGVCommand command; ErrorCode reason;
    TEST_ASSERT_TRUE(decode(R"({"version":1,"id":"n1","session":123,"command":"NODE_ORDER","taskId":"t1","node":"K1","nextNode":"G1","pathIndex":2,"departure":"BACKWARD"})", command, reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Departure::Backward), static_cast<int>(command.departure));
    TEST_ASSERT_TRUE(decode(R"({"version":1,"id":"l1","session":123,"command":"LIFT_UP","taskId":"t1","node":"K1","pathIndex":2})", command, reason));
    TEST_ASSERT_FALSE(decode(R"({"version":1,"id":"l2","session":123,"command":"LIFT_UP"})", command, reason));
}
void test_stop_during_lift_requires_cancel_before_restart() {
    ControlLogic core(settings()); moving(core); core.reachedNode("K1", 10);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::LiftUp), static_cast<int>(core.update(healthy(10)).servo));
    core.pause();
    TEST_ASSERT_EQUAL_INT(AGV_SAFE_STOP, core.state());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::ServoInterrupted), static_cast<int>(core.error()));
    ErrorCode reason;
    TEST_ASSERT_FALSE(core.restart(healthy(11), reason));
    core.cancelTask();
    TEST_ASSERT_TRUE(core.restart(healthy(12), reason));
    TEST_ASSERT_EQUAL_INT(AGV_IDLE, core.state());
}
void test_stop_waiting_does_not_auto_resume_and_empty_shelf_denies_lift() {
    SafetySettings policy = settings();
    policy.requireNodeOrders = policy.requireLiftCommands = true;
    ControlLogic core(policy); core.begin(0); core.reachedNode("B1", 0);
    ErrorCode reason;
    TEST_ASSERT_TRUE(core.setTask(task(), reason));
    TEST_ASSERT_TRUE(core.start(healthy(), reason));
    TEST_ASSERT_TRUE(core.waitingForOrder());
    core.pause();
    AGVCommand order;
    order.type = CommandType::NodeOrder; order.pathIndex = 0; order.departure = Departure::Straight;
    AGVMath::copyText(order.taskId, sizeof(order.taskId), "task-1");
    AGVMath::copyText(order.node, sizeof(order.node), "B1");
    AGVMath::copyText(order.nextNode, sizeof(order.nextNode), "K1");
    TEST_ASSERT_TRUE(core.setNodeOrder(order, 1, reason));
    TEST_ASSERT_EQUAL_INT(AGV_WAITING, core.state());
    TEST_ASSERT_TRUE(core.start(healthy(2), reason));
    core.reachedNode("K1", 3);
    AGVCommand lift;
    lift.type = CommandType::LiftUp; lift.pathIndex = 1;
    AGVMath::copyText(lift.taskId, sizeof(lift.taskId), "task-1");
    AGVMath::copyText(lift.node, sizeof(lift.node), "K1");
    auto empty = healthy(4); empty.cargoPresent = false;
    TEST_ASSERT_FALSE(core.authorizeLift(lift, empty, reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ErrorCode::CargoMissing), static_cast<int>(reason));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ServoAction::None), static_cast<int>(core.update(empty).servo));
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_official_pin_map_and_uniqueness);
    RUN_TEST(test_pwm_timer_partition);
    RUN_TEST(test_unknown_distance_is_not_fake_zero);
    RUN_TEST(test_encoder_distance_formula);
    RUN_TEST(test_battery_unknown_and_formula);
    RUN_TEST(test_line_polarity_and_actions);
    RUN_TEST(test_task_rejects_bad_node_or_destination);
    RUN_TEST(test_task_rejects_duplicate_missing_pickup_empty);
    RUN_TEST(test_start_requires_task_and_confirmed_node);
    RUN_TEST(test_start_rejects_configuration_and_busy_servo);
    RUN_TEST(test_full_pickup_and_delivery_cycle);
    RUN_TEST(test_same_node_does_not_advance_path);
    RUN_TEST(test_unexpected_node_latches_stop);
    RUN_TEST(test_unknown_tag_latches_stop);
    RUN_TEST(test_node_change_during_loading_stops);
    RUN_TEST(test_wifi_grace_pauses_then_timeout_latches);
    RUN_TEST(test_mqtt_timeout_latches);
    RUN_TEST(test_reconnection_does_not_auto_move_during_grace);
    RUN_TEST(test_zero_timeout_safe_stop_in_idle);
    RUN_TEST(test_timeout_across_millis_wrap);
    RUN_TEST(test_emergency_stop_requires_restart_then_start);
    RUN_TEST(test_restart_rejects_obstacle_lost_line_and_network);
    RUN_TEST(test_obstacle_stops_without_servo);
    RUN_TEST(test_invalid_ultrasonic_stops);
    RUN_TEST(test_only_000_stops_for_line_loss);
    RUN_TEST(test_restart_accepts_101_and_111_but_not_000);
    RUN_TEST(test_turn_requires_calibration);
    RUN_TEST(test_turn_completes_only_after_minimum_and_center);
    RUN_TEST(test_turn_timeout_stops);
    RUN_TEST(test_turn_stops_when_all_line_sensors_clear);
    RUN_TEST(test_interrupted_turn_requires_cancel);
    RUN_TEST(test_straight_crossing_never_ignores_lost_line);
    RUN_TEST(test_stop_and_cancel_do_not_clear_latch);
    RUN_TEST(test_active_task_cannot_be_overwritten);
    RUN_TEST(test_parser_accepts_task_and_departures);
    RUN_TEST(test_parser_accepts_all_simple_commands);
    RUN_TEST(test_parser_rejects_bad_json_version_types_unknown);
    RUN_TEST(test_parser_rejects_zero_session_for_motion_but_allows_stop);
    RUN_TEST(test_parser_rejects_bad_task_and_oversized_payload);
    RUN_TEST(test_parser_rejects_long_identifier_and_path);
    RUN_TEST(test_parser_rejects_trailing_data_and_nul);
    RUN_TEST(test_sensor_failure_during_loading_suppresses_servo);
    RUN_TEST(test_error_state_stays_stopped_until_explicit_restart);
    RUN_TEST(test_single_node_pickup_and_drop_cycle);
    RUN_TEST(test_reverse_buzzer_tracks_applied_motor);
    RUN_TEST(test_26_nodes_and_reverse_order_validation);
    RUN_TEST(test_parser_requires_bound_node_and_lift_commands);
    RUN_TEST(test_stop_during_lift_requires_cancel_before_restart);
    RUN_TEST(test_stop_waiting_does_not_auto_resume_and_empty_shelf_denies_lift);
    return UNITY_END();
}
