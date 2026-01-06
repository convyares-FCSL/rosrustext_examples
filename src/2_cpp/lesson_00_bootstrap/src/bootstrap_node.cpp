#include "lesson_00_bootstrap_cpp/bootstrap_node.hpp"

BootstrapNode::BootstrapNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("lesson_00_bootstrap", options) {
  RCLCPP_INFO(get_logger(), "Lesson 00 bootstrap node started.");
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BootstrapNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
