#pragma once

#include <string>
#include <optional>
#include <variant>

namespace lesson_05_parameters_cpp {

template <typename T>
struct ValidationResult {
  bool ok;
  std::optional<T> value;
  std::string reason;

  static ValidationResult<T> Success(T val) {
    return {true, val, ""};
  }

  static ValidationResult<T> Failure(const std::string& reason) {
    return {false, std::nullopt, reason};
  }
};

/**
 * Validates timer period (must be > 0.0).
 */
inline ValidationResult<double> validate_timer_period_s(double value) {
  if (value <= 0.0) {
    return ValidationResult<double>::Failure("timer_period_s must be > 0.0");
  }
  return ValidationResult<double>::Success(value);
}

/**
 * Validates reset tolerance (must be >= 0).
 */
inline ValidationResult<int64_t> validate_reset_max_value(int64_t value) {
  if (value < 0) {
    return ValidationResult<int64_t>::Failure("reset_max_value must be >= 0");
  }
  return ValidationResult<int64_t>::Success(value);
}

} // namespace lesson_05_parameters_cpp