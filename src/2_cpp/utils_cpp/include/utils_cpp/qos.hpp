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

// Helper to get or declare a parameter with a default value
template <typename T>
T get_or_declare_param(rclcpp::Node &node, const std::string &name, const T &default_value) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

// Internal implementation of loading a profile
inline rclcpp::QoS load_profile(rclcpp::Node &node, const std::string &profile_name) {
  const auto key = normalize_profile(profile_name);
  const auto defaults = defaults_for_profile(key);
  const auto base = std::string("qos.profiles.") + key + ".";

  const auto rel = get_or_declare_param<std::string>(node, base + "reliability", defaults.reliability);
  const auto durability = get_or_declare_param<std::string>(node, base + "durability", defaults.durability);
  const auto depth = get_or_declare_param<int>(node, base + "depth", static_cast<int>(defaults.depth));

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

// 2. Explicit Profile Getters (Match Python API)
inline rclcpp::QoS telemetry(rclcpp::Node &node) {
  return load_profile(node, "telemetry");
}

inline rclcpp::QoS commands(rclcpp::Node &node) {
  return load_profile(node, "commands");
}

inline rclcpp::QoS state_latched(rclcpp::Node &node) {
  return load_profile(node, "state_latched");
}

inline rclcpp::QoS events(rclcpp::Node &node) {
  return load_profile(node, "events");
}

inline rclcpp::QoS reliable_data(rclcpp::Node &node) {
  return load_profile(node, "reliable_data");
}

inline rclcpp::QoS static_data_latched(rclcpp::Node &node) {
  return load_profile(node, "static_data_latched");
}

inline rclcpp::QoS from_parameters(rclcpp::Node &node) {
  std::string profile_name = "telemetry";
  if (node.has_parameter("qos.profile")) {
    profile_name = node.get_parameter("qos.profile").as_string();
  } else {
    profile_name = node.declare_parameter<std::string>("qos.profile", "telemetry");
  }
  return load_profile(node, profile_name);
}

}  // namespace qos
