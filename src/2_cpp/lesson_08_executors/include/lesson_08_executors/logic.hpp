#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

namespace lesson_08_executors_cpp
{

// -----------------------------------------------------------------------------
// Telemetry publishing core
// -----------------------------------------------------------------------------
class TelemetryPublisherCore
{
public:
  std::uint64_t next_value()
  {
    return count_++;
  }

  void reset()
  {
    count_ = 0;
  }

private:
  std::uint64_t count_{0};
};

// -----------------------------------------------------------------------------
// Fibonacci generation core
// -----------------------------------------------------------------------------
class FibonacciGenerator
{
public:
  std::int32_t step(std::vector<std::int32_t> & sequence) const
  {
    if (sequence.empty()) {
      sequence.push_back(0);
      return 0;
    }

    if (sequence.size() == 1) {
      sequence.push_back(1);
      return 1;
    }

    const auto n = sequence.size();
    const std::int32_t next = sequence[n - 1] + sequence[n - 2];
    sequence.push_back(next);
    return next;
  }
};

// -----------------------------------------------------------------------------
// Fibonacci routine
// Simulates long-running work by blocking within the routine.
// -----------------------------------------------------------------------------
class FibonacciRoutine
{
public:
  using ProgressFn = std::function<void(const std::vector<std::int32_t> &)>;

  explicit FibonacciRoutine(
    FibonacciGenerator gen = FibonacciGenerator(),
    std::chrono::milliseconds step_delay = std::chrono::seconds(1))
  : gen_(std::move(gen)), step_delay_(step_delay)
  {}

  std::vector<std::int32_t> run(std::int32_t order, const ProgressFn & on_progress) const
  {
    std::vector<std::int32_t> sequence;
    if (order <= 0) {
      return sequence;
    }

    sequence.reserve(static_cast<std::size_t>(order));

    for (std::int32_t i = 0; i < order; ++i) {
      std::this_thread::sleep_for(step_delay_);
      gen_.step(sequence);
      on_progress(sequence);
    }

    return sequence;
  }

private:
  FibonacciGenerator gen_;
  std::chrono::milliseconds step_delay_;
};


}  // namespace lesson_08_executors_cpp