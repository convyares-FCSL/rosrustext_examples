#include <chrono>

#include "lesson_05_parameters/publisher.hpp"
#include "lesson_05_parameters/utils.hpp"
#include "utils_cpp/topics.hpp"
#include "utils_cpp/qos.hpp"

using lesson_interfaces::msg::MsgCount;

namespace lesson_05_parameters_cpp
{

// --- TelemetryPublisherCore Implementation ---

MsgCount TelemetryPublisherCore::next_message() {
  MsgCount msg;
  msg.count = count_++;
  return msg;
}

// --- Lesson05Publisher Implementation ---

Lesson05Publisher::Lesson05Publisher(const rclcpp::NodeOptions & options)
: Node("lesson_05_publisher", options)
{
  setup_parameters();
  setup_publisher();
  apply_initial_configuration();

  // Register callback for hot updates
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) { 
      return this->on_parameters(params); 
    });

  RCLCPP_INFO(this->get_logger(), 
    "Lesson 05 publisher started. Publishing to '%s' every %.2fs", 
    publisher_->get_topic_name(), timer_period_s_);
}

void Lesson05Publisher::setup_parameters() {
  // Declare with default, but validate usage later
  this->declare_parameter<double>("timer_period_s", 1.0);
}

void Lesson05Publisher::setup_publisher() {
  // Use shared utils for configuration
  auto topic = topics::telemetry(*this);
  auto qos_profile = qos::telemetry(*this);
  
  publisher_ = this->create_publisher<MsgCount>(topic, qos_profile);
}

void Lesson05Publisher::apply_initial_configuration() {
  double raw = this->get_parameter("timer_period_s").as_double();
  auto result = validate_timer_period_s(raw);

  if (!result.ok) {
    RCLCPP_WARN(this->get_logger(), "Invalid initial timer_period_s=%.2f. Defaulting to 1.0s", raw);
    create_or_update_timer(1.0);
  } else {
    create_or_update_timer(*result.value);
  }
}

rcl_interfaces::msg::SetParametersResult Lesson05Publisher::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters) 
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "timer_period_s") {
      auto valid = validate_timer_period_s(param.as_double());
      
      if (!valid.ok) {
        result.successful = false;
        result.reason = valid.reason;
        return result;
      }

      create_or_update_timer(*valid.value);
      RCLCPP_INFO(this->get_logger(), "Updated timer_period_s -> %.2fs", *valid.value);
    }
  }
  return result;
}

void Lesson05Publisher::create_or_update_timer(double period_s) {
  timer_period_s_ = period_s;
  
  if (timer_) {
    timer_->cancel();
  }
  
  auto duration = std::chrono::duration<double>(timer_period_s_);
  timer_ = this->create_wall_timer(duration, [this]() {
    auto msg = core_.next_message();
    publisher_->publish(msg);
  });
}

} // namespace lesson_05_parameters_cpp

// --- Main Entry Point ---

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lesson_05_parameters_cpp::Lesson05Publisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}