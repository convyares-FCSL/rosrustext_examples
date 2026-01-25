#pragma once

#include <optional>
#include <string>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace lesson_07_actions_cpp
{

template <typename T>
struct ValidationResult
{
  bool ok;
  std::optional<T> value;
  std::string reason;

  static ValidationResult<T> Success(T val)
  {
    return {true, val, ""};
  }

  static ValidationResult<T> Failure(const std::string & reason)
  {
    return {false, std::nullopt, reason};
  }
};

inline ValidationResult<double> validate_timer_period_s(double value)
{
  if (value <= 0.0) {
    return ValidationResult<double>::Failure("timer_period_s must be > 0.0");
  }
  return ValidationResult<double>::Success(value);
}

inline rcl_interfaces::msg::SetParametersResult ok_result()
{
  rcl_interfaces::msg::SetParametersResult r;
  r.successful = true;
  return r;
}

inline rcl_interfaces::msg::SetParametersResult fail_result(const std::string & reason)
{
  rcl_interfaces::msg::SetParametersResult r;
  r.successful = false;
  r.reason = reason;
  return r;
}

}  // namespace lesson_07_actions_cpp
