# Lesson 01 Breakdown: The Event Loop & Parameters (C++)

## Architecture: The Event-Driven Node

In Lesson 00, we spun the node once and exited. In Lesson 01, the node enters an **Event Loop**.



1.  **Configuration**: The node declares and reads a **Parameter** (`timer_period_s`) at startup.
2.  **Scheduling**: It registers a **Timer** with the Executor using `create_wall_timer`.
3.  **Spinning**: The `main` function blocks on `rclcpp::spin()`. It sleeps until the timer fires, wakes up to run `on_tick()`, and repeats.

## Code Walkthrough

### 1. The Header (`lesson.hpp`)
We define the member variables and methods. Notice the usage of standard types.

```cpp
private:
  // Internal state
  double timer_period_s_{1.0};
  std::uint64_t tick_{0};
  
  // Timer Handle
  // We must hold this shared_ptr. If it goes out of scope, the timer is destroyed.
  rclcpp::TimerBase::SharedPtr timer_;

```

* **`rclcpp::TimerBase::SharedPtr`**: This is the standard handle type for timers in ROS 2 C++.

### 2. Parameter Declaration (`lesson.cpp`)

In the constructor, we declare the parameter before we can use it.

```cpp
// "timer_period_s" is the name, 1.0 is the default value.
this->declare_parameter<double>("timer_period_s", 1.0);

```

* **Templated Type**: `<double>` enforces type safety. If a user tries to pass a string at the command line, ROS 2 will reject it.

### 3. The Setup Helper

We use a helper method to keep the logic clean. This pattern is essential for C++ nodes that might need to "restart" timers dynamically.

```cpp
void Lesson01Node::start_timer_from_param()
{
  // 1. Get Value
  timer_period_s_ = this->get_parameter("timer_period_s").as_double();

  // 2. Validation
  if (timer_period_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid period...");
    timer_period_s_ = 1.0;
  }

  // 3. Time Conversion
  // rclcpp timers expect std::chrono::duration types.
  // We convert the double (seconds) into nanoseconds for precision.
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_period_s_));

  // 4. Create Timer
  // create_wall_timer(duration, callback)
  // We use std::bind to link the member function 'on_tick' to 'this' instance.
  timer_ = this->create_wall_timer(period, std::bind(&Lesson01Node::on_tick, this));
}

```

**Why `std::chrono`?**
ROS 2 is designed for real-time systems. Floating point math (`0.1 + 0.2`) usually results in precision errors. `std::chrono` handles time integers (nanoseconds) precisely, preventing drift in long-running control loops.

### 4. The Main Loop

Unlike the manual `spin_some` from Lesson 00, we now use the blocking `spin`.

```cpp
try {
  auto node = std::make_shared<Lesson01Node>();
  
  // BLOCKING CALL: Runs forever until Ctrl+C
  rclcpp::spin(node);

} catch (const std::exception & e) {
  // Global exception handler prevents core dumps on known errors
  RCLCPP_ERROR(..., e.what());
  return EXIT_FAILURE;
}

```

* **Exception Safety**: We wrap the spin in a `try/catch` block. If `on_tick` throws an exception (e.g., hardware failure), we catch it here and exit gracefully instead of crashing the entire process.