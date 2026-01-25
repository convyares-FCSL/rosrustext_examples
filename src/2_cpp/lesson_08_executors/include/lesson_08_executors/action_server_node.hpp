#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lesson_interfaces/msg/msg_count.hpp"

#include "lesson_08_executors/logic.hpp"
#include "lesson_08_executors/action_server.hpp"

namespace lesson_08_executors_cpp
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// -----------------------------------------------------------------------------
// TelemetryPublisher (Component)
// -----------------------------------------------------------------------------
class TelemetryPublisher
{
public:
  TelemetryPublisher() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode * parent);
  void on_activate();
  void on_deactivate();
  void on_cleanup();

  void publish();

private:
  TelemetryPublisherCore core_;
  rclcpp_lifecycle::LifecyclePublisher<lesson_interfaces::msg::MsgCount>::SharedPtr publisher_;
};

// -----------------------------------------------------------------------------
// ActionServerNode (Lifecycle Orchestrator)
// -----------------------------------------------------------------------------
class ActionServerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void create_or_update_timer(double period_s, bool start_running);
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::shared_ptr<TelemetryPublisher> telemetry_;
  std::shared_ptr<ActionServerComponent> action_;

  rclcpp::TimerBase::SharedPtr timer_;
  double timer_period_s_{1.0};

  rclcpp::CallbackGroup::SharedPtr cb_group_telemetry_;
  rclcpp::CallbackGroup::SharedPtr cb_group_action_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace lesson_08_executors_cpp
