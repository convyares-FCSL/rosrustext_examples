#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace lesson_cpp_template
{

class LessonNode final : public rclcpp::Node
{
public:
  // << FILL IN HERE >>: Update the default node name if needed
  static constexpr char DEFAULT_NODE_NAME[] = "lesson_node";

  explicit LessonNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Rule of 5: Delete copy/move to prevent resource duplication
  LessonNode(const LessonNode &) = delete;
  LessonNode & operator=(const LessonNode &) = delete;
  LessonNode(LessonNode &&) = delete;
  LessonNode & operator=(LessonNode &&) = delete;
  ~LessonNode() override = default;

private:
  // Helper to declutter constructor
  void init_parameters();

  // << FILL IN HERE >>: Declare Callbacks
  // void on_timer();

  // << FILL IN HERE >>: Private Members (State & Resources)
  // rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace lesson_cpp_template