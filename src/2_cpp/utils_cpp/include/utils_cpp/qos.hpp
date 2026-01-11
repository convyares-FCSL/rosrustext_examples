#pragma once

#include <string>
#include <algorithm>
#include <cctype>
#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/utils.hpp"

namespace qos {

inline constexpr char DEFAULT_PROFILE_NAME[] = "telemetry";

struct ProfileDefaults {
    std::string reliability;
    std::string durability;
    int depth;
};

// Internal dictionary matching Python's _PROFILES
inline ProfileDefaults get_defaults_for_key(const std::string& key) {
    if (key == "commands")            return {"reliable",    "volatile",        1};
    if (key == "state_latched")       return {"reliable",    "transient_local", 1};
    if (key == "events")              return {"reliable",    "volatile",        50};
    if (key == "reliable_data")       return {"reliable",    "volatile",        10};
    if (key == "static_data_latched") return {"reliable",    "transient_local", 1};
    // Default / "telemetry"
    return {"best_effort", "volatile", 10}; 
}

// Internal loader
inline rclcpp::QoS _load_profile(rclcpp::Node &node, std::string profile_name) {
    // Normalize key
    std::string key = profile_name;
    // (Simple trim/lower logic could be added here if strict matching isn't desired, 
    // but Python version relies on exact keys or fallback in the dictionary lookup)
    if (key.empty()) key = DEFAULT_PROFILE_NAME;

    // Get defaults (local C++ defaults, used if params are missing)
    ProfileDefaults defs = get_defaults_for_key(key);
    
    // Construct param base: qos.profiles.<name>
    std::string base = "qos.profiles." + key;

    // Load individual policies via params
    std::string rel = utils_cpp::get_or_declare_parameter<std::string>(
        node, base + ".reliability", defs.reliability, "qos");
    
    std::string dur = utils_cpp::get_or_declare_parameter<std::string>(
        node, base + ".durability", defs.durability, "qos");
    
    int depth = utils_cpp::get_or_declare_parameter<int>(
        node, base + ".depth", defs.depth, "qos");

    // Construct QoS
    rclcpp::QoS qos(depth);

    if (rel == "best_effort") qos.best_effort();
    else qos.reliable();

    if (dur == "transient_local") qos.transient_local();
    else qos.durability_volatile();

    return qos;
}

// --- Public API ---

inline rclcpp::QoS from_parameters(rclcpp::Node &node) {
    std::string profile = utils_cpp::get_or_declare_parameter<std::string>(
        node, "qos.default_profile", DEFAULT_PROFILE_NAME, "qos.default_profile");
    return _load_profile(node, profile);
}

inline rclcpp::QoS telemetry(rclcpp::Node &node) {
    return _load_profile(node, "telemetry");
}

inline rclcpp::QoS commands(rclcpp::Node &node) {
    return _load_profile(node, "commands");
}

inline rclcpp::QoS state_latched(rclcpp::Node &node) {
    return _load_profile(node, "state_latched");
}

inline rclcpp::QoS events(rclcpp::Node &node) {
    return _load_profile(node, "events");
}

inline rclcpp::QoS reliable_data(rclcpp::Node &node) {
    return _load_profile(node, "reliable_data");
}

inline rclcpp::QoS static_data_latched(rclcpp::Node &node) {
    return _load_profile(node, "static_data_latched");
}

} // namespace qos