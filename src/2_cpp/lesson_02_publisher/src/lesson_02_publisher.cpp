#include <cstdlib>
#include <memory>

#include "lesson_02_publisher/lesson_02_publisher.hpp"
#include "utils_cpp/qos.hpp"
#include "utils_cpp/topics.hpp"

using lesson_interfaces::msg::MsgCount;

// Constructor
Lesson02Node::Lesson02Node(const rclcpp::NodeOptions & options) : rclcpp::Node("lesson_02_node", options)
{
  // Parameter: timer_period_s (float seconds)
  this->declare_parameter<double>("timer_period_s", 1.0);

  // Create publisher
  setup_publisher();

  // Create timer
  setup_timer();

  // Initial log message
  RCLCPP_INFO(this->get_logger(), "Lesson 02 node started (publisher). Ctrl+C to exit.");
}

// Helper to create publisher
void Lesson02Node::setup_publisher()
{
  // Load topic name and QoS profile
  auto topic_name = topics::chatter(*this);
  auto qos_profile = qos::telemetry(*this);

  // Create publisher
  publisher_ = this->create_publisher<MsgCount>(topic_name, qos_profile);
}

// Helper to create timer
void Lesson02Node::setup_timer(){
  timer_period_s_ = this->get_parameter("timer_period_s").as_double();

  // Validate timer period
  if (timer_period_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "timer_period_s=%.6f is invalid; using 1.0s", timer_period_s_);
    timer_period_s_ = 1.0;
  }

  // Reset timer and create a new one
  timer_.reset();
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_period_s_));
  timer_ = this->create_wall_timer(period, std::bind(&Lesson02Node::on_tick, this));  
}

// Timer callback
void Lesson02Node::on_tick()
{
  // Increment tick count
  ++tick_;

  // Create message and publish
  MsgCount msg;
  msg.count = tick_;
  publisher_->publish(msg);
}

// Main entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Lesson02Node>();
    rclcpp::spin(node);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("lesson_02_node_cpp"), "Exception in main: %s", e.what());

    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
}
