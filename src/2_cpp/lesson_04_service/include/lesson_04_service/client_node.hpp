#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/srv/compute_stats.hpp"

// Lesson 04: Simple Service Client
// Responsibilities:
// 1. Discovery (Wait for service)
// 2. Action (Send Request)
// 3. Reaction (Handle Response Async)
class Lesson04ClientNode final : public rclcpp::Node {
public:
  explicit Lesson04ClientNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  Lesson04ClientNode(const Lesson04ClientNode &) = delete;
  Lesson04ClientNode &operator=(const Lesson04ClientNode &) = delete;

  // Trigger the request sequence
  void send_sample_request();

private:
  // Callback for the asynchronous future
  void handle_response(rclcpp::Client<lesson_interfaces::srv::ComputeStats>::SharedFuture future);

  rclcpp::Client<lesson_interfaces::srv::ComputeStats>::SharedPtr client_;
};