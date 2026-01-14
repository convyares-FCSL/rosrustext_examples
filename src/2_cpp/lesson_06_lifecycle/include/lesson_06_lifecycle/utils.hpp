#pragma once

#include <string>
#include <optional>
#include <variant>

// << NEW: Add ROS Interface Includes >>
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter_value.hpp"

namespace lesson_06_lifecycle_cpp {

// ... (Keep ValidationResult and validation functions exactly as they were) ...

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

inline ValidationResult<double> validate_timer_period_s(double value) {
  if (value <= 0.0) {
    return ValidationResult<double>::Failure("timer_period_s must be > 0.0");
  }
  return ValidationResult<double>::Success(value);
}

inline ValidationResult<int64_t> validate_reset_max_value(int64_t value) {
  if (value < 0) {
    return ValidationResult<int64_t>::Failure("reset_max_value must be >= 0");
  }
  return ValidationResult<int64_t>::Success(value);
}

// << NEW: Production Helper Function >>
/**
 * Retrieves or declares a parameter using the underlying Node Interface.
 * This abstracts away the difference between rclcpp::Node and rclcpp_lifecycle::LifecycleNode.
 */
inline std::string get_or_declare_parameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface,
  const std::string& param_name,
  const std::string& default_value)
{
  if (!params_interface->has_parameter(param_name)) {
    params_interface->declare_parameter(param_name, rclcpp::ParameterValue(default_value));
  }
  return params_interface->get_parameter(param_name).as_string();
}

} // namespace lesson_06_lifecycle_cpp