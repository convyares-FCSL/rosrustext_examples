#pragma once
//------------------------------------------------------------------------------
/**
 * @file    qos.hpp
 * @brief   Shared QoS presets and helpers.
 */
//------------------------------------------------------------------------------
#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace qos {

inline constexpr int DEFAULT_DEPTH = 10;

inline rclcpp::QoS telemetry(std::size_t depth = DEFAULT_DEPTH) {
  return rclcpp::QoS{depth}
      .best_effort()
      .durability_volatile()
      .deadline(std::chrono::nanoseconds{0})
      .keep_last(depth);
}

inline rclcpp::QoS commands(std::size_t depth = 1) {
  return rclcpp::QoS{depth}
      .reliable()
      .durability_volatile()
      .keep_last(depth);
}

inline rclcpp::QoS state_latched(std::size_t depth = 1) {
  return rclcpp::QoS{depth}
      .reliable()
      .transient_local()
      .keep_last(depth);
}

}  // namespace qos
