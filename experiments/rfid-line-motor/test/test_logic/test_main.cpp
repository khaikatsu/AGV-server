#include <unity.h>
#include "TestLogic.h"
#include <limits>

using namespace Bench;
const Tuning tuning{100, 45, 95, 200, 5000, 30, true};
void setUp() {}
void tearDown() {}
void check(Output output, int left, int right) {
    TEST_ASSERT_EQUAL_INT(left, output.left);
    TEST_ASSERT_EQUAL_INT(right, output.right);
}
void test_boot_stop() {
    Controller c(tuning);
    check(c.update(0, {false, true, false}), 0, 0);
    TEST_ASSERT_FALSE(c.card1(0, {true, true, true}));
}
void test_follow_patterns() {
    for (unsigned p = 0; p < 8; ++p) {
        Controller c(tuning); c.start();
        const auto out = c.update(0, {bool(p & 4), bool(p & 2), bool(p & 1)});
        switch (p) {
            case 0: case 5: check(out, 0, 0); TEST_ASSERT_EQUAL(int(State::Fault), int(c.state())); break;
            case 2: case 7: check(out, 100, 100); break;
            case 4: case 6: check(out, 45, 100); break;
            case 1: case 3: check(out, 100, 45); break;
        }
    }
}
void test_pivot_requires_new_left_edge() {
    Controller c(tuning); c.start();
    TEST_ASSERT_TRUE(c.card1(0, {true, true, true}));
    check(c.update(400, {true, true, true}), 95, -95);
    check(c.update(500, {false, false, false}), 95, -95);
    check(c.update(530, {false, false, false}), 95, -95);
    check(c.update(600, {true, false, false}), 95, -95);
    check(c.update(629, {true, false, false}), 95, -95);
    check(c.update(630, {true, false, false}), 45, 100);
    TEST_ASSERT_EQUAL(int(State::Follow), int(c.state()));
    check(c.update(640, {false, true, false}), 100, 100);
    TEST_ASSERT_FALSE(c.card1(650, {true, true, true}));
}
void test_minimum_turn_and_noise() {
    Controller c(tuning); c.start(); c.card1(0, {});
    c.update(0, {}); c.update(30, {});
    c.update(40, {true, false, false});
    check(c.update(70, {true, false, false}), 95, -95);
    c.update(180, {}); // a glitch must reset the detection timer
    c.update(190, {true, false, false});
    check(c.update(200, {true, false, false}), 95, -95);
    check(c.update(220, {true, false, false}), 45, 100);
}
void test_absence_must_be_stable() {
    Controller c(tuning); c.start(); c.card1(0, {});
    c.update(0, {}); c.update(20, {true, false, false});
    check(c.update(500, {true, false, false}), 95, -95);
}
void test_timeout_latches_stop() {
    Controller c(tuning); c.start(); c.card1(0, {true, true, true});
    check(c.update(5000, {true, true, true}), 0, 0);
    TEST_ASSERT_EQUAL(int(Fault::TurnTimeout), int(c.fault()));
    check(c.update(5001, {false, true, false}), 0, 0);
    TEST_ASSERT_FALSE(c.card1(5002, {true, true, true}));
}
void test_stop_during_turn_and_rearm() {
    Controller c(tuning); c.start(); c.card1(0, {}); c.stop();
    check(c.update(10, {true, true, true}), 0, 0);
    c.start(); TEST_ASSERT_TRUE(c.card1(20, {}));
}
void test_line_fault_does_not_auto_resume() {
    Controller c(tuning); c.start();
    check(c.update(0, {}), 0, 0);
    TEST_ASSERT_EQUAL(int(Fault::LineLost), int(c.fault()));
    check(c.update(10, {false, true, false}), 0, 0);
}
void test_millis_wrap() {
    Controller c(tuning); c.start();
    const uint32_t start = std::numeric_limits<uint32_t>::max() - 100;
    c.card1(start, {});
    c.update(start, {}); c.update(start + 30u, {});
    c.update(start + 200u, {true, false, false});
    check(c.update(start + 230u, {true, false, false}), 45, 100);
}
void test_left_pivot_option() {
    auto t = tuning; t.pivotRight = false;
    Controller c(t); c.start(); c.card1(0, {});
    check(c.update(1, {}), -95, 95);
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_boot_stop);
    RUN_TEST(test_follow_patterns);
    RUN_TEST(test_pivot_requires_new_left_edge);
    RUN_TEST(test_minimum_turn_and_noise);
    RUN_TEST(test_absence_must_be_stable);
    RUN_TEST(test_timeout_latches_stop);
    RUN_TEST(test_stop_during_turn_and_rearm);
    RUN_TEST(test_line_fault_does_not_auto_resume);
    RUN_TEST(test_millis_wrap);
    RUN_TEST(test_left_pivot_option);
    return UNITY_END();
}
