#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "utils_cpp/qos.hpp"
#include "utils_cpp/topics.hpp"

class SubscriberNode final : public rclcpp::Node {
public:
  using Msg = std_msgs::msg::String;

  explicit SubscriberNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  SubscriberNode(const SubscriberNode &) = delete;
  SubscriberNode &operator=(const SubscriberNode &) = delete;

private:
  void on_message(const Msg &msg);

  rclcpp::Subscription<Msg>::SharedPtr subscription_;
};
