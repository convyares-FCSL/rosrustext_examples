#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace services {

inline constexpr char ADD_TWO_INTS[] = "add_two_ints";
inline constexpr char ADD_TWO_INTS_SERVICE[] = "add_two_ints";

inline std::string from_params(
    rclcpp::Node &node,
    const std::string &service_name,
    const std::string &default_value) {
  return node.declare_parameter<std::string>(
      std::string("services.") + service_name, default_value);
}

}  // namespace services
