#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lesson_interfaces/action/fibonacci.hpp"

namespace lesson_07_actions_cpp
{

class ActionClientNode : public rclcpp::Node
{
public:
  using Fibonacci = lesson_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit ActionClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void send_goal(int32_t order);
  void demo_cancel();

private:
  void feedback_callback(GoalHandleFibonacci::SharedPtr,
                         const std::shared_ptr<const Fibonacci::Feedback> feedback);

  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
};

}  // namespace lesson_07_actions_cpp
