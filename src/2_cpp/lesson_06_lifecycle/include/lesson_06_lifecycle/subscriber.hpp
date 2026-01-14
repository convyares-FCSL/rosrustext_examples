#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"

#include "lesson_06_lifecycle/logic.hpp"

namespace lesson_06_lifecycle_cpp
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ==============================================================================
// 1. The Component
// Wraps the Logic + The ROS Subscription + Manual Gating.
// Matches Python 'TelemetrySubscriber'.
// ==============================================================================
class TelemetryListener {
public:
  TelemetryListener() = default;

  // Lifecycle Hooks
  void on_configure(rclcpp_lifecycle::LifecycleNode* parent);
  void on_activate();
  void on_deactivate();
  void on_cleanup();

  // Configuration
  void update_config(int64_t reset_max);

private:
  // Internal callback
  void on_message(const lesson_interfaces::msg::MsgCount & msg);

  // State
  bool enabled_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("telemetry_listener")};
  
  // Logic & Resources
  TelemetryStreamValidator validator_;
  rclcpp::Subscription<lesson_interfaces::msg::MsgCount>::SharedPtr subscriber_;
};

// ==============================================================================
// 2. The Node
// Manages Lifecycle State & Parameters. Owns the Component.
// ==============================================================================
class LifecycleSubscriberNode final : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LifecycleSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // --- Lifecycle Callbacks ---
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Dynamic Parameter Callback
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);

  // Component
  std::shared_ptr<TelemetryListener> listener_;
  
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace lesson_06_lifecycle_cpp