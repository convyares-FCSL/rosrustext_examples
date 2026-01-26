#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "lesson_08_executors/action_server_node.hpp"
#include "lesson_08_executors/utils.hpp"

#include "utils_cpp/qos.hpp"
#include "utils_cpp/topics.hpp"

using lesson_interfaces::msg::MsgCount;

namespace lesson_08_executors_cpp
{

// -----------------------------------------------------------------------------
// TelemetryPublisher
// -----------------------------------------------------------------------------

void TelemetryPublisher::on_configure(rclcpp_lifecycle::LifecycleNode * parent)
{
  const std::string topic = topics::telemetry(*parent);
  const auto qos_profile = qos::telemetry(*parent);

  publisher_ = parent->create_publisher<MsgCount>(topic, qos_profile);
  RCLCPP_INFO(parent->get_logger(), "Component configured: Publisher on '%s'", topic.c_str());
}

void TelemetryPublisher::on_activate()
{
  if (!publisher_) { return; }

  publisher_->on_activate();
}

void TelemetryPublisher::on_deactivate()
{
  if (!publisher_) { return; }

  publisher_->on_deactivate();
}

void TelemetryPublisher::on_cleanup()
{
  publisher_.reset();
}

void TelemetryPublisher::publish()
{
  if (!publisher_ || !publisher_->is_activated()) {
    return;
  }

  MsgCount msg;
  msg.count = core_.next_value();
  publisher_->publish(msg);
}

// -----------------------------------------------------------------------------
// ActionServerNode
// -----------------------------------------------------------------------------

ActionServerNode::ActionServerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lesson_08_action_server", options),
  telemetry_(std::make_shared<TelemetryPublisher>()),
  action_(std::make_shared<ActionServerComponent>())
{
  RCLCPP_INFO(get_logger(), "Node initialized (Unconfigured). Waiting for manager...");
}

CallbackReturn ActionServerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring from %s...", state.label().c_str());

  try {
    // Parameters persist across cleanup/re-configure.
    if (!has_parameter("timer_period_s")) {
      declare_parameter<double>("timer_period_s", 1.0);
    }

    param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) { return on_parameters(params); });

    // Create callback groups
    cb_group_telemetry_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_action_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    telemetry_->on_configure(this);
    action_->on_configure(this, cb_group_action_);

    const auto valid = validate_timer_period_s(get_parameter("timer_period_s").as_double());
    if (!valid.ok) {
      throw std::runtime_error(valid.reason);
    }

    // Inactive -> start paused.
    create_or_update_timer(*valid.value, /*start_running=*/false);

    return CallbackReturn::SUCCESS;
  } 
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn ActionServerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating from %s...", state.label().c_str());

  telemetry_->on_activate();
  action_->on_activate();

  if (timer_) {
    timer_->reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ActionServerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating from %s...", state.label().c_str());

  telemetry_->on_deactivate();
  action_->on_deactivate();

  if (timer_) {
    timer_->cancel();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ActionServerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up from %s...", state.label().c_str());

  timer_.reset();
  action_->on_cleanup();
  telemetry_->on_cleanup();
  param_callback_handle_.reset();

  cb_group_telemetry_.reset();
  cb_group_action_.reset();

  // Parameters intentionally persist (no undeclare).

  return CallbackReturn::SUCCESS;
}

CallbackReturn ActionServerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down from %s...", state.label().c_str());

  timer_.reset();
  action_->on_cleanup();
  telemetry_->on_cleanup();
  
  cb_group_telemetry_.reset();
  cb_group_action_.reset();

  return CallbackReturn::SUCCESS;
}

void ActionServerNode::create_or_update_timer(double period_s, bool start_running)
{
  timer_period_s_ = period_s;

  timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period_s_),
    [this]() { telemetry_->publish(); },
    cb_group_telemetry_);

  if (!start_running) {
    timer_->cancel();
    return;
  }

  RCLCPP_INFO(get_logger(), "Timer updated to %.2fs", period_s);
}

rcl_interfaces::msg::SetParametersResult ActionServerNode::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() != "timer_period_s") {
      continue;
    }

    const auto valid = validate_timer_period_s(param.as_double());
    if (!valid.ok) {
      return fail_result(valid.reason);
    }

    const bool active =
      (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    create_or_update_timer(*valid.value, active);
  }

  return ok_result();
}

}  // namespace lesson_08_executors_cpp


#include "rclcpp_components/register_node_macro.hpp"


// Topic 09 (Composition): expose this node as an rclcpp component so it can be
// discovered/loaded/unloaded via canonical `ros2 component ...` tooling.
RCLCPP_COMPONENTS_REGISTER_NODE(lesson_08_executors_cpp::ActionServerNode)

#ifndef COMPOSITION_BUILD
// Topic 09 (Composition): when building the component shared library we compile
// without `main()` to avoid duplicate symbols. The standalone executable build
// path remains unchanged.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<lesson_08_executors_cpp::ActionServerNode>();

    // Lesson 08: MultiThreadedExecutor to recover availability
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node->get_node_base_interface());
    executor.spin();
  }
  catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
}
#endif
