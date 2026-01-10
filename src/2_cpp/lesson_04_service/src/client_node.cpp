#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>

#include "lesson_04_service/client_node.hpp"
#include "utils_cpp/services.hpp"

using lesson_interfaces::srv::ComputeStats;
using namespace std::chrono_literals;

Lesson04ClientNode::Lesson04ClientNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("lesson_04_service_client", options) {
  
  // 1. Configuration
  auto service_name = services::compute_stats(*this);

  // 2. Create Client
  client_ = this->create_client<ComputeStats>(service_name);

  // 3. Discovery (Blocking wait in constructor for tutorial simplicity)
  //    In production, this might happen in a timer to avoid blocking startup.
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service '%s' not available, waiting...", service_name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Service '%s' available.", service_name.c_str());
}

void Lesson04ClientNode::send_sample_request() {
  auto request = std::make_shared<ComputeStats::Request>();
  
  // Sample data (matching Python lesson)
  request->data = {10.5, 20.2, 30.7};

  RCLCPP_INFO(this->get_logger(), "Sending request with %zu samples...", request->data.size());

  // 4. Async Call
  //    We define the callback *before* sending to keep ownership clear.
  auto future_callback = [this](rclcpp::Client<ComputeStats>::SharedFuture future) {
    this->handle_response(future);
  };

  client_->async_send_request(request, future_callback);
}

void Lesson04ClientNode::handle_response(rclcpp::Client<ComputeStats>::SharedFuture future) {
  try {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), 
      "Result -> Sum: %.2f, Avg: %.2f, Status: '%s'",
      response->sum, response->average, response->status.c_str());
      
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }

  // Tutorial logic: Shutdown after one successful call
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Lesson04ClientNode>();
    
    // Trigger the action
    node->send_sample_request();

    // Spin to allow the async callback to fire
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    // Check if exception is just normal shutdown
    if (rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("lesson_04_service_client"), "Exception: %s", e.what());
        return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}