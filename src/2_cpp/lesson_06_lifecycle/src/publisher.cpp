#include <chrono>

#include "lesson_06_lifecycle/publisher.hpp"
#include "lesson_06_lifecycle/utils.hpp"

using lesson_interfaces::msg::MsgCount;

namespace lesson_06_lifecycle_cpp
{

// ==============================================================================
// TelemetryPublisher (Component) Implementation
// ==============================================================================

void TelemetryPublisher::on_configure(rclcpp_lifecycle::LifecycleNode* parent) {
  std::string topic = get_or_declare_parameter(
    parent->get_node_parameters_interface(), 
    "topic_names.telemetry", 
    "telemetry"
  );

  rclcpp::QoS qos_profile(10);
  qos_profile.reliable(); 

  publisher_ = parent->create_publisher<MsgCount>(topic, qos_profile);
  
  RCLCPP_INFO(parent->get_logger(), "Component configured: Publisher on '%s'", topic.c_str());
}

void TelemetryPublisher::on_activate() {
  if (publisher_) {
    publisher_->on_activate();
  }
}

void TelemetryPublisher::on_deactivate() {
  if (publisher_) {
    publisher_->on_deactivate();
  }
}

void TelemetryPublisher::on_cleanup() {
  publisher_.reset();
}

void TelemetryPublisher::publish() {
  if (publisher_ && publisher_->is_activated()) {
      auto val = core_.next_value();
      
      MsgCount msg;
      msg.count = val;
      
      publisher_->publish(msg);
  }
}

// ==============================================================================
// LifecyclePublisherNode Implementation
// ==============================================================================

LifecyclePublisherNode::LifecyclePublisherNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lesson_06_lifecycle_publisher", options)
{
  component_ = std::make_shared<TelemetryPublisher>();
  RCLCPP_INFO(get_logger(), "Node initialized (Unconfigured). Waiting for manager...");
}

CallbackReturn LifecyclePublisherNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring from %s...", state.label().c_str());

  try {
    // 1. Setup Parameters
    // Check existence first to persist parameters across cleanup/re-configure cycles
    if (!this->has_parameter("timer_period_s")) {
      this->declare_parameter<double>("timer_period_s", 1.0);
    }

    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { 
        return this->on_parameters(params); 
      });

    // 2. Configure Component
    component_->on_configure(this);

    // 3. Create Timer (Start Paused)
    double raw = this->get_parameter("timer_period_s").as_double();
    auto valid = validate_timer_period_s(raw);
    
    create_or_update_timer(valid.ok ? *valid.value : 1.0);
    if (timer_) {
        timer_->cancel();
    }

    return CallbackReturn::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn LifecyclePublisherNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating from %s...", state.label().c_str());
  component_->on_activate();
  if (timer_) {
    timer_->reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisherNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating from %s...", state.label().c_str());
  component_->on_deactivate();
  if (timer_) {
    timer_->cancel();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisherNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up from %s...", state.label().c_str());
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  component_->on_cleanup();
  param_callback_handle_.reset();
  
  // Parameter persistence handled by NOT undeclaring here
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn LifecyclePublisherNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down from %s...", state.label().c_str());
  timer_.reset();
  component_->on_cleanup();
  return CallbackReturn::SUCCESS;
}

void LifecyclePublisherNode::create_or_update_timer(double period_s) {
  timer_period_s_ = period_s;
  
  bool should_be_running = (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  
  auto duration = std::chrono::duration<double>(timer_period_s_);
  timer_ = this->create_wall_timer(duration, [this]() {
    component_->publish();
  });

  if (!should_be_running) {
    timer_->cancel();
  } else {
    RCLCPP_INFO(get_logger(), "Timer updated to %.2fs", period_s);
  }
}

rcl_interfaces::msg::SetParametersResult LifecyclePublisherNode::on_parameters(
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
    }
  }
  return result;
}

} // namespace lesson_06_lifecycle_cpp


#include "rclcpp_components/register_node_macro.hpp"

// Topic 09 (Composition): expose this node as an rclcpp component so it can be
// discovered/loaded/unloaded via canonical `ros2 component ...` tooling.
RCLCPP_COMPONENTS_REGISTER_NODE(lesson_06_lifecycle_cpp::LifecyclePublisherNode)

#ifndef COMPOSITION_BUILD
// Topic 09 (Composition): when building the component shared library we compile
// without `main()` to avoid duplicate symbols. The standalone executable build
// path remains unchanged.
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lesson_06_lifecycle_cpp::LifecyclePublisherNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif