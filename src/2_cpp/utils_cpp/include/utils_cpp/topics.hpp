#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace topics {

// Canonical Defaults (matching Python utils_py/topics.py)
inline constexpr char DEFAULT_CHATTER[] = "/tutorial/chatter";    // Topics 01+
inline constexpr char DEFAULT_TELEMETRY[] = "/tutorial/telemetry"; // Topics 05+
inline constexpr char DEFAULT_FIBONACCI[] = "/tutorial/fibonacci";  // Topics 07+

// Core Interface
inline std::string chatter(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return utils_cpp::get_or_declare_param(
      params, logger, "topics.chatter", std::string(DEFAULT_CHATTER), "topic");
}

inline std::string telemetry(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return utils_cpp::get_or_declare_param(
      params, logger, "topics.telemetry", std::string(DEFAULT_TELEMETRY), "topic");
}

inline std::string fibonacci(
  rclcpp::node_interfaces::NodeParametersInterface & params,
  const rclcpp::Logger & logger)
{
  return utils_cpp::get_or_declare_param(
      params, logger, "topics.fibonacci", std::string(DEFAULT_FIBONACCI), "action");
}

// -----------------------------------------------------------------------------
// Adapters (Thin Wrappers)
// -----------------------------------------------------------------------------

// rclcpp::Node
inline std::string chatter(rclcpp::Node & node) {
  return chatter(*node.get_node_parameters_interface(), node.get_logger());
}
inline std::string telemetry(rclcpp::Node & node) {
  return telemetry(*node.get_node_parameters_interface(), node.get_logger());
}
inline std::string fibonacci(rclcpp::Node & node) {
  return fibonacci(*node.get_node_parameters_interface(), node.get_logger());
}

// rclcpp_lifecycle::LifecycleNode
inline std::string chatter(rclcpp_lifecycle::LifecycleNode & node) {
    return chatter(*node.get_node_parameters_interface(), node.get_logger());
}
inline std::string telemetry(rclcpp_lifecycle::LifecycleNode & node) {
    return telemetry(*node.get_node_parameters_interface(), node.get_logger());
}
inline std::string fibonacci(rclcpp_lifecycle::LifecycleNode & node) {
    return fibonacci(*node.get_node_parameters_interface(), node.get_logger());
}

}  // namespace topics