#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lesson_interfaces/srv/compute_stats.hpp"


namespace lesson_interfaces {
namespace srv {
  using ComputeStats = lesson_interfaces::srv::ComputeStats;
}
}

class ServiceListener final {
public:
  explicit ServiceListener(rclcpp::Logger logger);

  void handle_request(
      const lesson_interfaces::srv::ComputeStats::Request::SharedPtr request,
      lesson_interfaces::srv::ComputeStats::Response::SharedPtr response);

private:
  rclcpp::Logger logger_;
};

class Lesson04ServiceNode final : public rclcpp::Node {
public:
  explicit Lesson04ServiceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  Lesson04ServiceNode(const Lesson04ServiceNode &) = delete;
  Lesson04ServiceNode &operator=(const Lesson04ServiceNode &) = delete;

private:
  void setup_service_server();

  std::unique_ptr<ServiceListener> listener_;

  rclcpp::Service<lesson_interfaces::srv::ComputeStats>::SharedPtr service_;
};