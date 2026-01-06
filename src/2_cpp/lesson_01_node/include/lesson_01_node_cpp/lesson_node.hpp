#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

class LessonNode final : public rclcpp::Node {
public:
  explicit LessonNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  LessonNode(const LessonNode &) = delete;
  LessonNode &operator=(const LessonNode &) = delete;

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds period_;
  int64_t count_ = 0;
};
