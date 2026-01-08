#pragma once

#include <cstdint>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"

// Lesson 02: simple node with publisher.
class Lesson02Node final : public rclcpp::Node {
public:
  explicit Lesson02Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Delete copy constructor and assignment operator
  Lesson02Node(const Lesson02Node &) = delete;
  Lesson02Node & operator=(const Lesson02Node &) = delete;

private:
  // Create or recreate the timer based on the current parameter value
  void setup_publisher();
  void setup_timer();

  // Timer callback
  void on_tick();

  // Internal state
  double timer_period_s_{1.0};
  std::uint64_t tick_{0};

  // Publisher
  rclcpp::Publisher<lesson_interfaces::msg::MsgCount>::SharedPtr publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;  
};
