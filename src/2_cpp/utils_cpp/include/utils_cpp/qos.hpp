#pragma once
//------------------------------------------------------------------------------
/**
 * @file    qos.hpp
 * @brief   Shared QoS presets and helpers.
 */
//------------------------------------------------------------------------------
#include <algorithm>
#include <chrono>
#include <cctype>
#include <string>

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

struct ProfileDefaults {
  const char *reliability;
  const char *durability;
  std::size_t depth;
};

inline std::string normalize_profile(const std::string &profile) {
  std::string key = profile;
  if (key.empty()) {
    return "telemetry";
  }
  std::transform(key.begin(), key.end(), key.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (key == "statelatched") {
    return "state_latched";
  }
  if (key == "reliabledata") {
    return "reliable_data";
  }
  if (key == "staticdatalatched") {
    return "static_data_latched";
  }
  return key;
}

inline ProfileDefaults defaults_for_profile(const std::string &profile) {
  if (profile == "commands") {
    return {"reliable", "volatile", 1};
  }
  if (profile == "state_latched") {
    return {"reliable", "transient_local", 1};
  }
  if (profile == "events") {
    return {"reliable", "volatile", 50};
  }
  if (profile == "reliable_data") {
    return {"reliable", "volatile", DEFAULT_DEPTH};
  }
  if (profile == "static_data_latched") {
    return {"reliable", "transient_local", 1};
  }
  return {"best_effort", "volatile", DEFAULT_DEPTH};
}

inline rclcpp::QoS from_parameters(rclcpp::Node &node) {
  const auto profile_name =
      node.declare_parameter<std::string>("qos.profile", "telemetry");
  const auto profile = normalize_profile(profile_name);
  const auto defaults = defaults_for_profile(profile);
  const auto base = std::string("qos.profiles.") + profile + ".";
  const auto rel = node.declare_parameter<std::string>(
      base + "reliability", defaults.reliability);
  const auto durability = node.declare_parameter<std::string>(
      base + "durability", defaults.durability);
  const auto depth = node.declare_parameter<int>(
      base + "depth", static_cast<int>(defaults.depth));

  auto qos = rclcpp::QoS{static_cast<std::size_t>(depth)};
  if (rel == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  qos.keep_last(static_cast<std::size_t>(depth));
  return qos;
}

}  // namespace qos
