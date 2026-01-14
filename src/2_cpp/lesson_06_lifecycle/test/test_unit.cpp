#include <gtest/gtest.h>
#include "lesson_06_lifecycle/utils.hpp"
#include "lesson_06_lifecycle/logic.hpp"

using namespace lesson_06_lifecycle_cpp;

// --- Validator Utils Tests ---

TEST(TestParams, TimerPeriodAcceptsPositive) {
  auto r = validate_timer_period_s(0.5);
  EXPECT_TRUE(r.ok);
  EXPECT_DOUBLE_EQ(*r.value, 0.5);
}

TEST(TestParams, TimerPeriodRejectsZero) {
  auto r = validate_timer_period_s(0.0);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find(">"), std::string::npos);
}

TEST(TestParams, TimerPeriodRejectsNegative) {
  auto r = validate_timer_period_s(-1.0);
  EXPECT_FALSE(r.ok);
}

TEST(TestParams, ResetMaxAcceptsPositive) {
  auto r = validate_reset_max_value(3);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(*r.value, 3);
}

TEST(TestParams, ResetMaxRejectsNegative) {
  auto r = validate_reset_max_value(-1);
  EXPECT_FALSE(r.ok);
}

// --- Logic Tests ---

TEST(TestLogic, StreamInitialSetsExpected) {
  TelemetryStreamValidator v(1);
  auto d = v.on_count(10);
  
  EXPECT_EQ(d.event, StreamEvent::INITIAL);
  EXPECT_EQ(d.expected_after, 11);
}

TEST(TestLogic, StreamOutOfOrderDetected) {
  TelemetryStreamValidator v(1);
  v.on_count(10); // expect 11
  
  auto d = v.on_count(9);
  EXPECT_EQ(d.event, StreamEvent::OUT_OF_ORDER);
  EXPECT_EQ(d.expected_after, 11); // Should not advance
}

TEST(TestLogic, StreamResetDetected) {
  TelemetryStreamValidator v(1); // Tolerance 1
  v.on_count(10); // expect 11
  
  // 1 is <= reset_max_value(1), so this is a reset
  auto d = v.on_count(1);
  EXPECT_EQ(d.event, StreamEvent::RESET);
  EXPECT_EQ(d.expected_after, 2);
}

TEST(TestLogic, StreamOkAdvances) {
  TelemetryStreamValidator v(1);
  v.on_count(1); // expect 2
  
  auto d = v.on_count(2);
  EXPECT_EQ(d.event, StreamEvent::OK);
  EXPECT_EQ(d.expected_after, 3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}