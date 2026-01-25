#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/utils.hpp"

namespace topics {

// Canonical Defaults (matching Python utils_py/topics.py)
inline constexpr char DEFAULT_CHATTER[] = "/tutorial/chatter";    // Topics 01+
inline constexpr char DEFAULT_TELEMETRY[] = "/tutorial/telemetry"; // Topics 05+
inline constexpr char DEFAULT_FIBONACCI[] = "/tutorial/fibonacci";  // Topics 07+

/**
 * @brief Get the chatter topic name (Topics 01+).
 * Param: topics.chatter
 */
inline std::string chatter(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "topics.chatter", DEFAULT_CHATTER, "topic");
}

/**
 * @brief Get the telemetry topic name (Topics 05+).
 * Param: topics.telemetry
 */
inline std::string telemetry(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "topics.telemetry", DEFAULT_TELEMETRY, "topic");
}

/**
 * @brief Get the Fibonacci action name (Topics 07+).
 * Param: topics.fibonacci
 */
inline std::string fibonacci(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "topics.fibonacci", DEFAULT_FIBONACCI, "action");
}

}  // namespace topics