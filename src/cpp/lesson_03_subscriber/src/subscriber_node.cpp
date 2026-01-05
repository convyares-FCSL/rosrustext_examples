#include "lesson_03_subscriber_cpp/subscriber_node.hpp"

SubscriberNode::SubscriberNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("lesson_03_subscriber", options) {
  subscription_ = create_subscription<Msg>(
      topics::CHATTER,
      qos::telemetry(),
      [this](const Msg &msg) { on_message(msg); });

  RCLCPP_INFO(
      get_logger(),
      "Lesson 03 subscriber started (topic: %s).",
      topics::CHATTER);
}

void SubscriberNode::on_message(const Msg &msg) {
  RCLCPP_INFO(get_logger(), "Heard: %s", msg.data.c_str());
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
