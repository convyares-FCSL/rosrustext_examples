#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"
#include "lesson_05_parameters/logic.hpp"

namespace lesson_05_parameters_cpp
{

// Adapter: Converts ROS msgs -> Validator -> Log Output
// Defined in header to match Lesson 03 style (MessageListener)
class TelemetryListener {
public:
  TelemetryListener(rclcpp::Logger logger, int64_t reset_max);

  void set_reset_max_value(int64_t val);
  void on_message(const lesson_interfaces::msg::MsgCount & msg);

private:
  rclcpp::Logger logger_;
  TelemetryStreamValidator validator_;
};

class Lesson05Subscriber final : public rclcpp::Node {
public:
  explicit Lesson05Subscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  Lesson05Subscriber(const Lesson05Subscriber &) = delete;
  Lesson05Subscriber & operator=(const Lesson05Subscriber &) = delete;

private:
  void setup_parameters();
  void setup_subscriber();

  // Callback for runtime parameter updates
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<TelemetryListener> listener_;
  rclcpp::Subscription<lesson_interfaces::msg::MsgCount>::SharedPtr subscriber_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace lesson_05_parameters_cpp