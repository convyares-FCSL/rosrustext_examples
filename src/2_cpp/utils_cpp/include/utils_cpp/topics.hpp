#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace topics {

inline constexpr char CHATTER[] = "chatter";
inline constexpr char NUMBERS[] = "numbers";
inline constexpr char COUNTS[] = "counts";
inline constexpr char ROBOT_NEWS_TOPIC[] = "robot_news";
inline constexpr char NUMBER_TOPIC[] = "number";
inline constexpr char NUMBER_COUNT_TOPIC[] = "number_count";

inline std::string from_params(
    rclcpp::Node &node,
    const std::string &topic_name,
    const std::string &default_value) {
  return node.declare_parameter<std::string>(
      std::string("topics.") + topic_name, default_value);
}

inline std::string chatter(rclcpp::Node &node) {
  return from_params(node, "chatter", CHATTER);
}

inline std::string robot_news(rclcpp::Node &node) {
  return from_params(node, "robot_news", ROBOT_NEWS_TOPIC);
}

inline std::string number(rclcpp::Node &node) {
  return from_params(node, "number", NUMBER_TOPIC);
}

inline std::string number_count(rclcpp::Node &node) {
  return from_params(node, "number_count", NUMBER_COUNT_TOPIC);
}

}  // namespace topics
