#pragma once

#include <string>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace utils_cpp
{

/**
 * @brief Declares a parameter (so it's visible), reads it, and warns if the 
 * default value is used AND it was not overridden externally (e.g. via YAML or CLI).
 * * @tparam T Parameter type
 * @param node The node instance
 * @param name Parameter name
 * @param default_value Default value to use if not set
 * @param warn_label Label for the warning log (e.g. "topic", "qos")
 * @return The effective parameter value
 */
template <typename T>
T get_or_declare_parameter(rclcpp::Node& node, const std::string& name, const T& default_value, const std::string& warn_label)
{
    // 1. Declare if it doesn't exist
    if (!node.has_parameter(name)) {
        node.declare_parameter<T>(name, default_value);
    }

    // 2. Get the effective value
    T value;
    node.get_parameter(name, value);

    // 3. Check for external overrides
    // rclcpp provides access to overrides passed at startup
    bool is_overridden = false;
    auto overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    
    // Check if 'name' exists in the overrides map
    // Note: This checks exact name matches. Nested params might appear differently depending on ROS 2 version,
    // but standard YAML loading populates this map.
    if (overrides.find(name) != overrides.end()) {
        is_overridden = true;
    }

    // 4. Warn logic: If value is default AND not overridden
    // We use strict equality check.
    if (value == default_value && !is_overridden) {
        // Convert value to string for logging
        std::string val_str;
        if constexpr (std::is_same_v<T, std::string>) {
            val_str = value;
        } else {
            val_str = std::to_string(value);
        }

        RCLCPP_WARN(node.get_logger(), 
            "[config] Using default %s: %s='%s' (no external override found)", 
            warn_label.c_str(), name.c_str(), val_str.c_str());
    }

    return value;
}

} // namespace utils_cpp