#pragma once

#include <cstdint>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

// Lesson 01: simple node with timer + logging.
class Lesson01Node final : public rclcpp::Node {
public:
  explicit Lesson01Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  Lesson01Node(const Lesson01Node &) = delete;
  Lesson01Node & operator=(const Lesson01Node &) = delete;

private:
  // Create or recreate the timer based on the current parameter value
  void start_timer_from_param();

  // Timer callback
  void on_tick();

  // Internal state
  double timer_period_s_{1.0};
  std::uint64_t tick_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};
