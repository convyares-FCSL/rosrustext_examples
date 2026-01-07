#pragma once

#include "rclcpp/rclcpp.hpp"

// Struct representing the bootstrap node.
class BootstrapNode final : public rclcpp::Node {
public:
  // Constructor: create the node and log startup.
  explicit BootstrapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Delete copy constructor and assignment operator.
  BootstrapNode(const BootstrapNode &) = delete;
  BootstrapNode & operator=(const BootstrapNode &) = delete;

private:
  // No members required for this bootstrap lesson.
};
