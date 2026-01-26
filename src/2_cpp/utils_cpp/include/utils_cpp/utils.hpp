#pragma once

#include <string>
#include <map>
#include <vector>
#include <cmath> // for std::abs
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace utils_cpp
{

// -----------------------------------------------------------------------------
// Core Interface-First Implementations
// -----------------------------------------------------------------------------

// Policy: Tolerant. If type mismatch occurs (e.g. user overrides string param with int), 
// we catch the exception, return the default, and rely on the warning to alert the user.
// This avoids crashing the node during configuration.

inline std::string get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  const std::string & default_value,
  const std::string & warn_label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }
  
  std::string value;
  try {
      value = params.get_parameter(name).as_string();
  } catch(const rclcpp::exceptions::InvalidParameterTypeException &) {
      RCLCPP_WARN(logger, "[config] Parameter '%s' has invalid type (expected string). Using default.", name.c_str());
      value = default_value; 
  }

  auto overrides = params.get_parameter_overrides();
  bool is_overridden = (overrides.find(name) != overrides.end());

  if (value == default_value && !is_overridden) {
    RCLCPP_WARN(logger, 
      "[config] Using default %s: %s='%s' (no startup override detected)", 
      warn_label.c_str(), name.c_str(), value.c_str());
  }
  return value;
}

inline int get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  int default_value,
  const std::string & warn_label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  int value;
  try {
      rclcpp::Parameter p = params.get_parameter(name);
      value = static_cast<int>(p.as_int());
  } catch(const rclcpp::exceptions::InvalidParameterTypeException &) {
      RCLCPP_WARN(logger, "[config] Parameter '%s' has invalid type (expected int). Using default.", name.c_str());
      value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  bool is_overridden = (overrides.find(name) != overrides.end());

  if (value == default_value && !is_overridden) {
    RCLCPP_WARN(logger, 
      "[config] Using default %s: %s='%d' (no startup override detected)", 
      warn_label.c_str(), name.c_str(), value);
  }
  return value;
}

inline double get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  double default_value,
  const std::string & warn_label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  double value;
  try {
      value = params.get_parameter(name).as_double();
  } catch(const rclcpp::exceptions::InvalidParameterTypeException &) {
      RCLCPP_WARN(logger, "[config] Parameter '%s' has invalid type (expected double). Using default.", name.c_str());
      value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  bool is_overridden = (overrides.find(name) != overrides.end());

  const double epsilon = 1e-6;
  if (std::abs(value - default_value) < epsilon && !is_overridden) {
    RCLCPP_WARN(logger, 
      "[config] Using default %s: %s='%.2f' (no startup override detected)", 
      warn_label.c_str(), name.c_str(), value);
  }
  return value;
}

inline bool get_or_declare_param(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger,
  const std::string & name,
  bool default_value,
  const std::string & warn_label)
{
  if (!params.has_parameter(name)) {
    params.declare_parameter(name, rclcpp::ParameterValue(default_value));
  }

  bool value;
  try {
      value = params.get_parameter(name).as_bool();
  } catch(const rclcpp::exceptions::InvalidParameterTypeException &) {
      RCLCPP_WARN(logger, "[config] Parameter '%s' has invalid type (expected bool). Using default.", name.c_str());
      value = default_value;
  }

  auto overrides = params.get_parameter_overrides();
  bool is_overridden = (overrides.find(name) != overrides.end());

  if (value == default_value && !is_overridden) {
    RCLCPP_WARN(logger, 
      "[config] Using default %s: %s='%s' (no startup override detected)", 
      warn_label.c_str(), name.c_str(), value ? "true" : "false");
  }
  return value;
}

} // namespace utils_cpp