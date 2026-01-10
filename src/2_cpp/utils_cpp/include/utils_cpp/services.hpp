#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace services {

inline constexpr char COMPUTE_STATS[] = "compute_stats";

// Helper to load service name from parameters (or default)
inline std::string from_params(
    rclcpp::Node &node,
    const std::string &service_name,
    const std::string &default_value) {
  return node.declare_parameter<std::string>(
      std::string("services.") + service_name, default_value);
}

inline std::string compute_stats(rclcpp::Node &node) {
  return from_params(node, COMPUTE_STATS, COMPUTE_STATS);
}

}  // namespace services