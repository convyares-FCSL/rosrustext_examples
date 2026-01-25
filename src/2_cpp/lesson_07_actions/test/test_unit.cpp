#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "lesson_07_actions/logic.hpp"

using namespace lesson_07_actions_cpp;

// -----------------------------------------------------------------------------
// TelemetryPublisherCore
// -----------------------------------------------------------------------------

TEST(TestTelemetryPublisherCore, NextValueIsMonotonic)
{
  TelemetryPublisherCore core;

  EXPECT_EQ(core.next_value(), 0u);
  EXPECT_EQ(core.next_value(), 1u);
  EXPECT_EQ(core.next_value(), 2u);
}

TEST(TestTelemetryPublisherCore, ResetRestartsSequence)
{
  TelemetryPublisherCore core;

  (void)core.next_value();  // 0
  (void)core.next_value();  // 1
  core.reset();

  EXPECT_EQ(core.next_value(), 0u);
  EXPECT_EQ(core.next_value(), 1u);
}

// -----------------------------------------------------------------------------
// FibonacciGenerator
// -----------------------------------------------------------------------------

TEST(TestFibonacciGenerator, StepBuildsExpectedSequence)
{
  FibonacciGenerator gen;
  std::vector<std::int32_t> seq;

  EXPECT_EQ(gen.step(seq), 0);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0}));

  EXPECT_EQ(gen.step(seq), 1);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0, 1}));

  EXPECT_EQ(gen.step(seq), 1);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0, 1, 1}));

  EXPECT_EQ(gen.step(seq), 2);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0, 1, 1, 2}));

  EXPECT_EQ(gen.step(seq), 3);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0, 1, 1, 2, 3}));
}

// -----------------------------------------------------------------------------
// FibonacciRoutine
// -----------------------------------------------------------------------------

TEST(TestFibonacciRoutine, RunCallsProgressAndReturnsExpectedSequence)
{
  // Unit tests must not sleep. Use 0ms delay.
  FibonacciRoutine routine(FibonacciGenerator{}, std::chrono::milliseconds(0));

  int calls = 0;
  std::vector<std::vector<std::int32_t>> snapshots;

  const auto seq = routine.run(5, [&](const std::vector<std::int32_t> & s) {
    ++calls;
    snapshots.push_back(s);
  });

  EXPECT_EQ(calls, 5);
  EXPECT_EQ(seq, (std::vector<std::int32_t>{0, 1, 1, 2, 3}));

  ASSERT_EQ(snapshots.size(), 5u);
  EXPECT_EQ(snapshots[0], (std::vector<std::int32_t>{0}));
  EXPECT_EQ(snapshots[1], (std::vector<std::int32_t>{0, 1}));
  EXPECT_EQ(snapshots[2], (std::vector<std::int32_t>{0, 1, 1}));
  EXPECT_EQ(snapshots[3], (std::vector<std::int32_t>{0, 1, 1, 2}));
  EXPECT_EQ(snapshots[4], (std::vector<std::int32_t>{0, 1, 1, 2, 3}));
}

TEST(TestFibonacciRoutine, OrderZeroOrNegativeReturnsEmptyAndNoProgress)
{
  FibonacciRoutine routine(FibonacciGenerator{}, std::chrono::milliseconds(0));

  int calls = 0;
  const auto seq0 = routine.run(0, [&](const std::vector<std::int32_t> &) { ++calls; });
  EXPECT_TRUE(seq0.empty());
  EXPECT_EQ(calls, 0);

  calls = 0;
  const auto seqn = routine.run(-3, [&](const std::vector<std::int32_t> &) { ++calls; });
  EXPECT_TRUE(seqn.empty());
  EXPECT_EQ(calls, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
