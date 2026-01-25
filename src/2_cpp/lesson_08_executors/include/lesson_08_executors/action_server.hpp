#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lesson_interfaces/action/fibonacci.hpp"
#include "lesson_08_executors/logic.hpp"

namespace lesson_08_executors_cpp
{

class ActionServerComponent
{
public:
  using Fibonacci = lesson_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ActionServerComponent() = default;

  void on_configure(rclcpp_lifecycle::LifecycleNode * parent, rclcpp::CallbackGroup::SharedPtr group);
  void on_activate();
  void on_deactivate();
  void on_cleanup();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  std::atomic<bool> enabled_{false};

  FibonacciRoutine routine_;
};

}  // namespace lesson_08_executors_cpp
