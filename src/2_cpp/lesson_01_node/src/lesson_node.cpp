#include "lesson_01_node_cpp/lesson_node.hpp"

LessonNode::LessonNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("lesson_01_node", options) {
  const double period_s = declare_parameter<double>("timer_period_s", 0.5);
  period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(period_s));

  timer_ = create_wall_timer(period_, [this]() { on_timer(); });
  RCLCPP_INFO(
      get_logger(),
      "Lesson 01 node started (period: %ld ms).",
      static_cast<long>(period_.count()));
}

void LessonNode::on_timer() {
  RCLCPP_INFO(get_logger(), "Tick %ld", static_cast<long>(count_++));
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LessonNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
