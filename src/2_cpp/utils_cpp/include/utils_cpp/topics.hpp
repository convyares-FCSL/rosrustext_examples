#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/utils.hpp"

namespace topics {

// Canonical Defaults (matching Python utils_py/topics.py)
inline constexpr char DEFAULT_CHATTER[] = "/tutorial/chatter";    // Lessons 00-04
inline constexpr char DEFAULT_TELEMETRY[] = "/tutorial/telemetry"; // Lesson 05

/**
 * @brief Get the chatter topic name (Lessons 00-04).
 * Param: topics.chatter
 */
inline std::string chatter(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "topics.chatter", DEFAULT_CHATTER, "topic");
}

/**
 * @brief Get the telemetry topic name (Lesson 05).
 * Param: topics.telemetry
 */
inline std::string telemetry(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "topics.telemetry", DEFAULT_TELEMETRY, "topic");
}

}  // namespace topics