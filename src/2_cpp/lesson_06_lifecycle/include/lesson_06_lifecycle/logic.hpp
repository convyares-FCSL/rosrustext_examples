#pragma once

#include <cstdint>
#include <string>
#include <sstream>

namespace lesson_06_lifecycle_cpp {

// --- Publisher Logic ---

class TelemetryPublisherCore {
public:
  std::uint64_t next_value() {
    return count_++;
  }

  void reset() {
    count_ = 0;
  }

private:
  std::uint64_t count_{0};
};

// --- Subscriber Logic ---

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

  StreamDecision on_count(std::uint64_t count) {
    if (!initialized_) {
      initialized_ = true;
      expected_ = count + 1;
      return {StreamEvent::INITIAL, count, expected_, fmt("Received (initial): ", count)};
    }

    if (count <= reset_max_value_ && count < expected_) {
      expected_ = count + 1;
      return {StreamEvent::RESET, count, expected_, fmt("Detected reset: ", count)};
    }

    if (count < expected_) {
      std::stringstream ss;
      ss << "Out-of-order: " << count << " < " << expected_;
      return {StreamEvent::OUT_OF_ORDER, count, expected_, ss.str()};
    }

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

} // namespace lesson_06_lifecycle_cpp