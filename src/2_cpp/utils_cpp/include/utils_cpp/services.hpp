#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/utils.hpp"

namespace services {

inline constexpr char DEFAULT_COMPUTE_STATS[] = "/tutorial/compute_stats";

/**
 * @brief Get the compute_stats service name.
 * Param: services.compute_stats
 */
inline std::string compute_stats(rclcpp::Node &node) {
  return utils_cpp::get_or_declare_parameter<std::string>(
      node, "services.compute_stats", DEFAULT_COMPUTE_STATS, "service");
}

}  // namespace services