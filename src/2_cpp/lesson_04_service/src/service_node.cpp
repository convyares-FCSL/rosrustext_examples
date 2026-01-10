#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

// FIX: Ensure include path matches package name
#include "lesson_04_service/service_node.hpp"
#include "lesson_04_service/stats_logic.hpp"
#include "utils_cpp/services.hpp"

using lesson_interfaces::srv::ComputeStats;

// FIX: Correct C++ 'using' syntax
using Request = ComputeStats::Request;
using Response = ComputeStats::Response;

// =============================================================================
// ServiceListener Implementation
// =============================================================================

ServiceListener::ServiceListener(rclcpp::Logger logger) : logger_(std::move(logger)) {}

void ServiceListener::handle_request(const Request::SharedPtr request, Response::SharedPtr response) {
  
  RCLCPP_INFO(logger_, "Incoming request with %zu samples.", request->data.size());

  auto result = stats_logic::Logic::compute(request->data);

  response->sum = result.sum;
  response->average = result.average;
  response->status = result.status;

  if (result.status != "Success") {
    RCLCPP_WARN(logger_, "Logic Warning: %s", result.status.c_str());
  } else {
    RCLCPP_INFO(logger_, "Computation Complete: Sum=%.2f, Avg=%.2f", 
                result.sum, result.average);
  }
}

// =============================================================================
// Lesson04ServiceNode Implementation
// =============================================================================

Lesson04ServiceNode::Lesson04ServiceNode(const rclcpp::NodeOptions &options) : rclcpp::Node("lesson_04_service_server", options) {
  listener_ = std::make_unique<ServiceListener>(this->get_logger());
  setup_service_server();
  RCLCPP_INFO(this->get_logger(), "Lesson 04 service node started (server). Ctrl+C to exit.");
}

void Lesson04ServiceNode::setup_service_server() {
  // This will now work because we updated utils_cpp
  auto service_name = services::compute_stats(*this);

  service_ = this->create_service<ComputeStats>(
      service_name,
      [this](const Request::SharedPtr req, Response::SharedPtr res) {
        listener_->handle_request(req, res);
      });
      
  RCLCPP_INFO(this->get_logger(), "Service '%s' is ready.", service_name.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Lesson04ServiceNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("lesson_04_service_server"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}