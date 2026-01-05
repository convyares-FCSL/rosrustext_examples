#include "lesson_02_publisher_cpp/publisher_node.hpp"

PublisherNode::PublisherNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("lesson_02_publisher", options) {
  prefix_ = declare_parameter<std::string>("message_prefix", "Hello");
  const double period_s = declare_parameter<double>("publish_period_s", 0.5);
  period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(period_s));

  publisher_ = create_publisher<Msg>(topics::CHATTER, qos::telemetry());
  timer_ = create_wall_timer(period_, [this]() { publish_message(); });

  RCLCPP_INFO(
      get_logger(),
      "Lesson 02 publisher started (topic: %s, period: %ld ms).",
      topics::CHATTER,
      static_cast<long>(period_.count()));
}

void PublisherNode::publish_message() {
  Msg msg;
  msg.data = prefix_ + " " + std::to_string(count_++);
  publisher_->publish(msg);
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
