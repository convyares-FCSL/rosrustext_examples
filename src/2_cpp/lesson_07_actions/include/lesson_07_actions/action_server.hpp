#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lesson_interfaces/action/fibonacci.hpp"
#include "lesson_07_actions/logic.hpp"

namespace lesson_07_actions_cpp
{

class ActionServerComponent
{
public:
  using Fibonacci = lesson_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ActionServerComponent() = default;

  void on_configure(rclcpp::Node * parent);
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

  rclcpp::Node * node_{nullptr};
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  std::atomic<bool> enabled_{false};

  FibonacciRoutine routine_;
};

}  // namespace lesson_07_actions_cpp
