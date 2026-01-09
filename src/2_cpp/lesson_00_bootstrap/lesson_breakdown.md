# Lesson 00 Breakdown: The C++ Node Structure

## Architecture: Header vs. Source

In C++, scalable software architecture relies on separating **Declaration** (what exists) from **Definition** (how it works). This allows other parts of the system to include your node (e.g., in a composable container) without recompiling the implementation.



1.  **Header (`.hpp`)**: Defines the interface (class name, constructor arguments, public methods).
2.  **Source (`.cpp`)**: Contains the logic.

## Code Walkthrough

### 1. The Header (`bootstrap_node.hpp`)
This file defines the blueprint of our node.

```cpp
// 1. Pragma Once
// Prevents the compiler from reading this file twice (include guard).
#pragma once

#include "rclcpp/rclcpp.hpp"

// 2. Class Inheritance
// We inherit from rclcpp::Node to gain all standard ROS 2 capabilities.
// 'final' indicates no other class should inherit from this one.
class BootstrapNode final : public rclcpp::Node {
public:
  // 3. Explicit Constructor
  // 'explicit' prevents accidental type conversions.
  // We accept NodeOptions to allow configuration (like params) at creation.
  explicit BootstrapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 4. Rule of Five (Safety)
  // We explicitly delete copy constructors. 
  // Nodes manage unique resources (handles) and should generally not be copied.
  BootstrapNode(const BootstrapNode &) = delete;
  BootstrapNode & operator=(const BootstrapNode &) = delete;
};

```

### 2. The Implementation (`bootstrap_node.cpp`)

Here we flesh out the constructor.

```cpp
// Constructor Initialization List
// We pass the node name "lesson_00_bootstrap" to the base class rclcpp::Node.
BootstrapNode::BootstrapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lesson_00_bootstrap", options)
{
  // RCLCPP_INFO is a macro that handles efficient logging.
  RCLCPP_INFO(get_logger(), "Lesson 00 bootstrap node started...");
}

```

### 3. The Execution Entry Point (`main`)

The `main` function handles the lifecycle. Note the use of `try/catch` to ensure robust error reporting.

```cpp
int main(int argc, char ** argv)
{
  try {
    // 1. Initialize Middleware
    rclcpp::init(argc, argv);

    // 2. Create Node (Shared Pointer)
    // std::make_shared is the standard way to create nodes in ROS 2 C++.
    auto bootstrap = std::make_shared<BootstrapNode>();

    // 3. Spin Once
    // 'spin_some' processes all currently available work (events) and then returns immediately.
    // This effectively runs the node "once" for this bootstrap lesson.
    rclcpp::spin_some(bootstrap);

    // 4. Clean Shutdown
    rclcpp::shutdown();

  } catch (const std::exception & e) {
    // 5. Global Error Handling
    // If the node throws an exception during construction, we catch it here.
    // We use a temporary logger because the node itself might be invalid.
    RCLCPP_ERROR(
      rclcpp::get_logger("lesson_00_bootstrap_cpp"),
      "Executor stopped with error: %s",
      e.what());

    // Clean up even if we crash
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return EXIT_FAILURE;
  }
}

```

**Why `std::make_shared`?**
In ROS 2 C++, the Executor expects a `std::shared_ptr<rclcpp::Node>`. This allows the Executor to keep the node alive while it spins, even if the original variable goes out of scope.