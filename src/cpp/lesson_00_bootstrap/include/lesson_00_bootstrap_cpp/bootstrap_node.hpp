#pragma once

#include "rclcpp/rclcpp.hpp"

class BootstrapNode final : public rclcpp::Node {
public:
  explicit BootstrapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  BootstrapNode(const BootstrapNode &) = delete;
  BootstrapNode &operator=(const BootstrapNode &) = delete;
};
