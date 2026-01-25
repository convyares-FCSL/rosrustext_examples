#include "lesson_08_executors/action_client_node.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "utils_cpp/topics.hpp"

namespace lesson_08_executors_cpp
{

ActionClientNode::ActionClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lesson_08_action_client", options)
{
  const std::string action_name = topics::fibonacci(*this);
  RCLCPP_INFO(get_logger(), "Connecting to action server: %s", action_name.c_str());
  client_ = rclcpp_action::create_client<Fibonacci>(shared_from_this(), action_name);
}

void ActionClientNode::feedback_callback(
  GoalHandleFibonacci::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  std::string s = "[";
  for (std::size_t i = 0; i < feedback->partial_sequence.size(); ++i) {
    s += std::to_string(feedback->partial_sequence[i]);
    if (i + 1 < feedback->partial_sequence.size()) { s += ", "; }
  }
  s += "]";
  RCLCPP_INFO(get_logger(), "Received feedback: %s", s.c_str());
}

void ActionClientNode::send_goal(int32_t order)
{
  if (!client_) { return; }

  RCLCPP_INFO(get_logger(), "Waiting for action server 'fibonacci'...");
  if (!client_->wait_for_action_server(std::chrono::seconds(60))) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return;
  }

  Fibonacci::Goal goal_msg;
  goal_msg.order = order;

  RCLCPP_INFO(get_logger(), "Sending goal: order=%d", order);

  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.feedback_callback =
    [this](GoalHandleFibonacci::SharedPtr gh, const std::shared_ptr<const Fibonacci::Feedback> fb) {
      feedback_callback(gh, fb);
    };

  auto send_future = client_->async_send_goal(goal_msg, options);
  if (rclcpp::spin_until_future_complete(shared_from_this(), send_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed while waiting for goal response");
    return;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle) {
    RCLCPP_INFO(get_logger(), "Goal rejected");
    return;
  }

  RCLCPP_INFO(get_logger(), "Goal accepted!");

  auto result_future = client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed while waiting for result");
    return;
  }

  const auto wrapped = result_future.get();

  std::string s = "[";
  for (std::size_t i = 0; i < wrapped.result->sequence.size(); ++i) {
    s += std::to_string(wrapped.result->sequence[i]);
    if (i + 1 < wrapped.result->sequence.size()) { s += ", "; }
  }
  s += "]";

  RCLCPP_INFO(get_logger(), "Result Received: %s (Status: %d)", s.c_str(), static_cast<int>(wrapped.code));
}

void ActionClientNode::demo_cancel()
{
  if (!client_) { return; }

  RCLCPP_INFO(get_logger(), "--- Starting Cancellation Demo ---");

  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Server not available for cancel demo");
    return;
  }

  Fibonacci::Goal goal_msg;
  goal_msg.order = 10;

  RCLCPP_INFO(get_logger(), "Sending goal (to be canceled)...");

  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.feedback_callback =
    [this](GoalHandleFibonacci::SharedPtr gh, const std::shared_ptr<const Fibonacci::Feedback> fb) {
      feedback_callback(gh, fb);
    };

  auto send_future = client_->async_send_goal(goal_msg, options);
  if (rclcpp::spin_until_future_complete(shared_from_this(), send_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed while waiting for goal response");
    return;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal rejected, cannot demonstrate cancel");
    return;
  }

  RCLCPP_INFO(get_logger(), "Goal accepted. Waiting 3 seconds allowing some feedback...");

  const auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(3)) {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(get_logger(), "Canceling goal...");

  auto cancel_future = client_->async_cancel_goal(goal_handle);
  if (rclcpp::spin_until_future_complete(shared_from_this(), cancel_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed while waiting for cancel response");
    return;
  }

  const auto cancel_resp = cancel_future.get();
  if (!cancel_resp->goals_canceling.empty()) {
    RCLCPP_INFO(get_logger(), "Cancel request accepted");
  } else {
    RCLCPP_WARN(get_logger(), "Cancel request did not return any cancelling goals");
  }

  auto result_future = client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed while waiting for final result");
    return;
  }

  const auto wrapped = result_future.get();

  std::string s = "[";
  for (std::size_t i = 0; i < wrapped.result->sequence.size(); ++i) {
    s += std::to_string(wrapped.result->sequence[i]);
    if (i + 1 < wrapped.result->sequence.size()) { s += ", "; }
  }
  s += "]";

  RCLCPP_INFO(get_logger(), "Final Result after cancel: Sequence=%s, Status=%d", s.c_str(), static_cast<int>(wrapped.code));
}

}  // namespace lesson_08_executors_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<lesson_08_executors_cpp::ActionClientNode>();

    std::printf("\n=== CASE 1: SUCCESS ===\n");
    node->send_goal(5);

    std::printf("\n=== CASE 2: CANCELLATION ===\n");
    node->demo_cancel();
  }
  catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
}
