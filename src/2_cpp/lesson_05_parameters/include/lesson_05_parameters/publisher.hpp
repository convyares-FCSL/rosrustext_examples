#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"

namespace lesson_05_parameters_cpp
{

// Pure logic core for the publisher (separates state from ROS).
class TelemetryPublisherCore {
public:
  lesson_interfaces::msg::MsgCount next_message();
private:
  std::uint64_t count_{0};
};

class Lesson05Publisher final : public rclcpp::Node {
public:
  explicit Lesson05Publisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Delete copy/move to prevent accidental slicing or duplication
  Lesson05Publisher(const Lesson05Publisher &) = delete;
  Lesson05Publisher & operator=(const Lesson05Publisher &) = delete;

private:
  void setup_parameters();
  void setup_publisher();
  void apply_initial_configuration();
  
  // Callback for runtime parameter updates
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);
  
  // Safe timer rebuilding
  void create_or_update_timer(double period_s);

  // Logic Core
  TelemetryPublisherCore core_;
  
  // State
  double timer_period_s_{1.0};

  // ROS Resources
  rclcpp::Publisher<lesson_interfaces::msg::MsgCount>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace lesson_05_parameters_cpp