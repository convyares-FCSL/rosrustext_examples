#pragma once

#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"

// Pure message-handling logic (no ROS resources owned here).
class MessageListener final {
public:
  explicit MessageListener(rclcpp::Logger logger, std::uint64_t reset_max_value = 1);

  void on_message(const lesson_interfaces::msg::MsgCount & msg);

private:
  void init_from(std::uint64_t msg_count, bool initial = false, bool reset = false);

  rclcpp::Logger logger_;

  bool initialized_{false};
  std::uint64_t expected_count_{0};
  std::uint64_t reset_max_value_{1};
};

// Lesson 03: simple node with subscriber.
class Lesson03Node final : public rclcpp::Node {
public:
  explicit Lesson03Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  Lesson03Node(const Lesson03Node &) = delete;
  Lesson03Node & operator=(const Lesson03Node &) = delete;

private:
  void setup_subscriber();

  // Node owns ROS resources; listener owns stream validation logic.
  std::unique_ptr<MessageListener> listener_;

  rclcpp::Subscription<lesson_interfaces::msg::MsgCount>::SharedPtr subscriber_;
};
