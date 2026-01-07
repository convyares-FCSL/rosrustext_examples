#include <cstdlib>   // EXIT_FAILURE
#include <memory>

#include "lesson_00_bootstrap_cpp/bootstrap_node.hpp"

// Constructor: create the node and log startup.
BootstrapNode::BootstrapNode(const rclcpp::NodeOptions & options): rclcpp::Node("lesson_00_bootstrap", options)
{
  // Log an informational message indicating the node has started.
  RCLCPP_INFO(get_logger(), "Lesson 00 bootstrap node started...");
}

// Main function: entry point of the program.
int main(int argc, char ** argv)
{
  try {
    // Initialize the ROS 2 context from command-line arguments.
    rclcpp::init(argc, argv);

    // Create the node via a constructor, mirroring the rclpy/rclrs pattern.
    auto bootstrap = std::make_shared<BootstrapNode>();

    // Spin the node once without blocking.
    rclcpp::spin_some(bootstrap);

    // Shutdown the ROS 2 context cleanly.
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    // Log the error once at the boundary, then exit with failure.
    RCLCPP_ERROR(
      rclcpp::get_logger("lesson_00_bootstrap_cpp"),
      "Executor stopped with error: %s",
      e.what());

    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}
