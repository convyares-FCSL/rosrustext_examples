#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "utils_cpp/qos.hpp"
#include "utils_cpp/topics.hpp"

class PublisherNode final : public rclcpp::Node {
public:
  using Msg = std_msgs::msg::String;
  using ms = std::chrono::milliseconds;

  explicit PublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  PublisherNode(const PublisherNode &) = delete;
  PublisherNode &operator=(const PublisherNode &) = delete;

private:
  void publish_message();

  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds period_;
  std::string prefix_;
  int64_t count_ = 0;
};
