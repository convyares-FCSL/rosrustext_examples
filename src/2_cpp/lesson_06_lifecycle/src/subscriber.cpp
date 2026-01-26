#include "lesson_06_lifecycle/subscriber.hpp"
#include "lesson_06_lifecycle/utils.hpp"

using lesson_interfaces::msg::MsgCount;

namespace lesson_06_lifecycle_cpp
{

// ==============================================================================
// TelemetryListener (Component) Implementation
// ==============================================================================

void TelemetryListener::on_configure(rclcpp_lifecycle::LifecycleNode* parent) {
  logger_ = parent->get_logger();
  
  // Production Pattern: Use the shared utility
  std::string topic = get_or_declare_parameter(
    parent->get_node_parameters_interface(), 
    "topic_names.telemetry", 
    "telemetry"
  );
  
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable(); 

  subscriber_ = parent->create_subscription<MsgCount>(
    topic, qos_profile,
    [this](const MsgCount::SharedPtr msg) { this->on_message(*msg); }
  );

  RCLCPP_INFO(logger_, "Component configured: Subscriber on '%s'", topic.c_str());
}

void TelemetryListener::on_activate() {
  enabled_ = true;
  RCLCPP_INFO(logger_, "Subscriber Activated (Gate Open)");
}

void TelemetryListener::on_deactivate() {
  enabled_ = false;
  RCLCPP_INFO(logger_, "Subscriber Deactivated (Gate Closed)");
}

void TelemetryListener::on_cleanup() {
  subscriber_.reset();
  enabled_ = false;
}

void TelemetryListener::update_config(int64_t reset_max) {
  validator_.set_reset_max_value(static_cast<std::uint64_t>(reset_max));
}

void TelemetryListener::on_message(const MsgCount & msg) {
  if (!enabled_) {
    return;
  }

  auto decision = validator_.on_count(static_cast<std::uint64_t>(msg.count));

  switch (decision.event) {
    case StreamEvent::RESET:
      RCLCPP_WARN(logger_, "%s", decision.message.c_str());
      break;
    case StreamEvent::OUT_OF_ORDER:
      RCLCPP_ERROR(logger_, "%s", decision.message.c_str());
      break;
    default:
      RCLCPP_INFO(logger_, "%s", decision.message.c_str());
      break;
  }
}

// ==============================================================================
// LifecycleSubscriberNode Implementation
// ==============================================================================

LifecycleSubscriberNode::LifecycleSubscriberNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lesson_06_lifecycle_subscriber", options)
{
  listener_ = std::make_shared<TelemetryListener>();
  RCLCPP_INFO(get_logger(), "Node initialized (Unconfigured). Waiting for manager...");
}

CallbackReturn LifecycleSubscriberNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring from %s...", state.label().c_str());

  try {
    if (!this->has_parameter("subscriber.reset_max_value")) {
      this->declare_parameter<int64_t>("subscriber.reset_max_value", 10);
    }
    
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { 
        return this->on_parameters(params); 
      });

    listener_->on_configure(this);

    int64_t raw = this->get_parameter("subscriber.reset_max_value").as_int();
    auto valid = validate_reset_max_value(raw);
    
    listener_->update_config(valid.ok ? *valid.value : 10);

    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn LifecycleSubscriberNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating from %s...", state.label().c_str());
  listener_->on_activate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriberNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating from %s...", state.label().c_str());
  listener_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriberNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up from %s...", state.label().c_str());
  
  listener_->on_cleanup();
  
  param_callback_handle_.reset();
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleSubscriberNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down from %s...", state.label().c_str());
  listener_->on_cleanup();
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult LifecycleSubscriberNode::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters) 
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "subscriber.reset_max_value") {
      auto valid = validate_reset_max_value(param.as_int());
      
      if (!valid.ok) {
        result.successful = false;
        result.reason = valid.reason;
        return result;
      }

      listener_->update_config(*valid.value);
      RCLCPP_INFO(this->get_logger(), "Updated reset_max_value -> %ld", *valid.value);
    }
  }
  return result;
}

} // namespace lesson_06_lifecycle_cpp

#include "rclcpp_components/register_node_macro.hpp"

// Topic 09 (Composition): expose this node as an rclcpp component so it can be
// discovered/loaded/unloaded via canonical `ros2 component ...` tooling.
RCLCPP_COMPONENTS_REGISTER_NODE(lesson_06_lifecycle_cpp::LifecycleSubscriberNode)

#ifndef COMPOSITION_BUILD
// Topic 09 (Composition): when building the component shared library we compile
// without `main()` to avoid duplicate symbols. The standalone executable build
// path remains unchanged.
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lesson_06_lifecycle_cpp::LifecycleSubscriberNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif