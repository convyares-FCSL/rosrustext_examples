#include "lesson_cpp_template/lesson_node.hpp"

LessonNode::LessonNode(
    std::chrono::milliseconds period,
    const rclcpp::NodeOptions &options)
: rclcpp::Node("lesson_node", options), period_(period) {
  timer_ = create_wall_timer(period_, [this]() { on_timer(); });
  RCLCPP_INFO(
      get_logger(),
      "Lesson node started (period: %ld ms).",
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