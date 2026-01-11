#include "lesson_05_parameters/subscriber.hpp"
#include "lesson_05_parameters/utils.hpp"
#include "utils_cpp/topics.hpp"
#include "utils_cpp/qos.hpp"

using lesson_interfaces::msg::MsgCount;

namespace lesson_05_parameters_cpp
{

// --- TelemetryListener Implementation ---

TelemetryListener::TelemetryListener(rclcpp::Logger logger, int64_t reset_max)
: logger_(logger), validator_(static_cast<std::uint64_t>(reset_max)) 
{}

void TelemetryListener::set_reset_max_value(int64_t val) {
  validator_.set_reset_max_value(static_cast<std::uint64_t>(val));
}

void TelemetryListener::on_message(const MsgCount & msg) {
  auto decision = validator_.on_count(static_cast<std::uint64_t>(msg.count));

  switch (decision.event) {
    case StreamEvent::RESET:
    case StreamEvent::OUT_OF_ORDER:
      RCLCPP_WARN(logger_, "%s", decision.message.c_str());
      break;
    default:
      RCLCPP_INFO(logger_, "%s", decision.message.c_str());
      break;
  }
}

// --- Lesson05Subscriber Implementation ---

Lesson05Subscriber::Lesson05Subscriber(const rclcpp::NodeOptions & options)
: Node("lesson_05_subscriber", options)
{
  setup_parameters();
  
  // Get initial effective value
  int64_t raw = this->get_parameter("reset_max_value").as_int();
  auto valid = validate_reset_max_value(raw);
  int64_t effective = valid.ok ? *valid.value : 1;
  
  listener_ = std::make_unique<TelemetryListener>(this->get_logger(), effective);

  setup_subscriber();

  // Register callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) { 
      return this->on_parameters(params); 
    });

  RCLCPP_INFO(this->get_logger(), 
    "Lesson 05 subscriber started. Subscribing to '%s'", 
    subscriber_->get_topic_name());
}

void Lesson05Subscriber::setup_parameters() {
  this->declare_parameter<int64_t>("reset_max_value", 1);
}

void Lesson05Subscriber::setup_subscriber() {
  auto topic = topics::telemetry(*this);
  auto qos_profile = qos::telemetry(*this);
  
  subscriber_ = this->create_subscription<MsgCount>(
    topic, qos_profile,
    [this](const MsgCount::SharedPtr msg) { listener_->on_message(*msg); }
  );
}

rcl_interfaces::msg::SetParametersResult Lesson05Subscriber::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters) 
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "reset_max_value") {
      auto valid = validate_reset_max_value(param.as_int());
      
      if (!valid.ok) {
        result.successful = false;
        result.reason = valid.reason;
        return result;
      }

      listener_->set_reset_max_value(*valid.value);
      RCLCPP_INFO(this->get_logger(), "Updated reset_max_value -> %ld", *valid.value);
    }
  }
  return result;
}

} // namespace lesson_05_parameters_cpp

// --- Main Entry Point ---

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lesson_05_parameters_cpp::Lesson05Subscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}