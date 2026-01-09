#include <cstdlib>
#include <memory>

#include "lesson_01_node_cpp/lesson.hpp"

// Constructor
Lesson01Node::Lesson01Node(const rclcpp::NodeOptions & options) : rclcpp::Node("lesson_01_node", options)
{
  // Parameter: timer_period_s (float seconds)
  this->declare_parameter<double>("timer_period_s", 1.0);

  // Create timer using the current parameter value
  start_timer_from_param();

  // Initial log message
  RCLCPP_INFO(this->get_logger(), "Lesson 01 node started (timer + logging). Ctrl+C to exit.");
}

// Create or recreate the timer based on the current parameter value
void Lesson01Node::start_timer_from_param()
{
  // Get the parameter value
  timer_period_s_ = this->get_parameter("timer_period_s").as_double();

  // Basic sanity clamp: prevent 0 or negative periods
  if (timer_period_s_ <= 0.0) {
    // Log warning
    RCLCPP_WARN(this->get_logger(), "timer_period_s=%.6f is invalid; using 1.0s", timer_period_s_);

    // Reset to default
    timer_period_s_ = 1.0;
  }

  // Log the timer period
  RCLCPP_INFO(this->get_logger(), "Timer running with period=%.3fs", timer_period_s_);

  // Convert to duration
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_period_s_));

  // Reset existing timer if present
  timer_.reset();

  // Create the timer
  timer_ = this->create_wall_timer(period, [this]() { this->on_tick();});
}

// Timer callback
void Lesson01Node::on_tick()
{
  // Increment tick count
  ++tick_;
  
  // Log tick message
  RCLCPP_INFO(this->get_logger(), "tick %llu", static_cast<unsigned long long>(tick_));
}

// Main entry point
int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  try {
    /// Create and spin the node
    auto node = std::make_shared<Lesson01Node>();
    rclcpp::spin(node);

  } catch (const std::exception & e) {
    // Log any exceptions
    RCLCPP_ERROR(rclcpp::get_logger("lesson_01_node_cpp"), "Exception in main: %s", e.what());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
}
