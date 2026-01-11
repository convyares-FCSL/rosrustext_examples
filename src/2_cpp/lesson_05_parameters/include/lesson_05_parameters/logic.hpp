#pragma once

#include <cstdint>
#include <string>
#include <sstream>

namespace lesson_05_parameters_cpp {

enum class StreamEvent {
  INITIAL,
  RESET,
  OUT_OF_ORDER,
  OK
};

struct StreamDecision {
  StreamEvent event;
  std::uint64_t count;
  std::uint64_t expected_after;
  std::string message;
};

/**
 * Validates a monotonically increasing counter stream with reset tolerance.
 * Matches logic.py TelemetryStreamValidator.
 */
class TelemetryStreamValidator {
public:
  explicit TelemetryStreamValidator(std::uint64_t reset_max_value = 1)
  : reset_max_value_(reset_max_value) {}

  void set_reset_max_value(std::uint64_t value) {
    reset_max_value_ = value;
  }

  [[nodiscard]] std::uint64_t get_reset_max_value() const {
    return reset_max_value_;
  }

  [[nodiscard]] std::uint64_t expected() const {
    return expected_;
  }

  StreamDecision on_count(std::uint64_t count) {
    // 1. Initial State
    if (!initialized_) {
      initialized_ = true;
      expected_ = count + 1;
      return {StreamEvent::INITIAL, count, expected_, fmt("Received (initial): ", count)};
    }

    // 2. Reset Detection
    // If count drops significantly (<= reset_max) and is less than expected, treat as reset.
    if (count <= reset_max_value_ && count < expected_) {
      expected_ = count + 1;
      return {StreamEvent::RESET, count, expected_, fmt("Detected counter reset. Re-syncing at: ", count)};
    }

    // 3. Out-of-Order/Stale Detection
    if (count < expected_) {
      std::stringstream ss;
      ss << "Out-of-order/invalid: " << count << " < " << expected_;
      return {StreamEvent::OUT_OF_ORDER, count, expected_, ss.str()};
    }

    // 4. Normal Progression
    expected_ = count + 1;
    return {StreamEvent::OK, count, expected_, fmt("Received: ", count)};
  }

private:
  std::string fmt(const char* prefix, std::uint64_t value) const {
    return std::string(prefix) + std::to_string(value);
  }

  bool initialized_{false};
  std::uint64_t expected_{0};
  std::uint64_t reset_max_value_{1};
};

} // namespace lesson_05_parameters_cpp