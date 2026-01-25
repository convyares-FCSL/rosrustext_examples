#include "lesson_08_executors/action_server.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "utils_cpp/topics.hpp"

namespace lesson_08_executors_cpp
{

// -----------------------------------------------------------------------------

void ActionServerComponent::on_configure(rclcpp_lifecycle::LifecycleNode * parent, rclcpp::CallbackGroup::SharedPtr group)
{
  node_ = parent;

  const std::string action_name = topics::fibonacci(*parent);

  server_ = rclcpp_action::create_server<Fibonacci>(
    parent,
    action_name,
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Fibonacci::Goal> goal) {
      return handle_goal(uuid, std::move(goal));
    },
    [this](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
      return handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
      handle_accepted(goal_handle);
    },
    rcl_action_server_get_default_options(),
    group);

  RCLCPP_INFO(parent->get_logger(), "Action server configured on '%s'", action_name.c_str());
}

void ActionServerComponent::on_activate()
{
  enabled_.store(true, std::memory_order_relaxed);
  if (!node_) { return; }
  RCLCPP_INFO(node_->get_logger(), "Action server activated");
}

void ActionServerComponent::on_deactivate()
{
  enabled_.store(false, std::memory_order_relaxed);
  if (!node_) { return; }
  RCLCPP_INFO(node_->get_logger(), "Action server deactivated");
}

void ActionServerComponent::on_cleanup()
{
  enabled_.store(false, std::memory_order_relaxed);
  server_.reset();
  node_ = nullptr;
}

rclcpp_action::GoalResponse ActionServerComponent::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Fibonacci::Goal> /*goal*/)
{
  if (!enabled_.load(std::memory_order_relaxed)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerComponent::handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerComponent::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  if (!node_) {
    // Should not happen if accepted
    return;
  }
  
  // Blocking execution is intentional for this lesson to demonstrate starvation.
  // (In a real application, consider offloading to a separate thread.)
  execute(goal_handle);
}

void ActionServerComponent::execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  if (!node_) { return; }

  const int32_t order = goal_handle->get_goal()->order;
  std::vector<std::int32_t> last_sequence;

  auto on_progress = [this, &goal_handle, &last_sequence](const std::vector<std::int32_t> & seq) {
    last_sequence = seq;

    if (!enabled_.load(std::memory_order_relaxed)) {
      throw std::runtime_error("deactivated");
    }
    if (goal_handle->is_canceling()) {
      throw std::runtime_error("canceled");
    }

    Fibonacci::Feedback feedback;
    feedback.partial_sequence = seq;
    goal_handle->publish_feedback(std::make_shared<Fibonacci::Feedback>(feedback));
  };

  try {
    auto sequence = routine_.run(order, on_progress);

    Fibonacci::Result result;
    result.sequence = std::move(sequence);
    goal_handle->succeed(std::make_shared<Fibonacci::Result>(result));
  }
  catch (const std::runtime_error & e) {
    Fibonacci::Result result;
    result.sequence = std::move(last_sequence);

    const std::string why = e.what();
    if (why == "canceled") {
      goal_handle->canceled(std::make_shared<Fibonacci::Result>(result));
      return;
    }

    goal_handle->abort(std::make_shared<Fibonacci::Result>(result));
  }
}

}  // namespace lesson_08_executors_cpp
