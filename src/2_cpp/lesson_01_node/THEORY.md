# Lesson 01 Breakdown: The Event Loop & Parameters (C++)

## Architecture: The Event-Driven Node

In Lesson 00, we spun the node once and exited. In Lesson 01, the node enters an **Event Loop**.

1. **Configuration**: The node declares and reads a **Parameter** (`timer_period_s`) at startup.
2. **Scheduling**: It registers a **Timer** with the Executor using `create_wall_timer`.
3. **Spinning**: The `main` function blocks on `rclcpp::spin()`. It sleeps until the timer fires, wakes up to run `on_tick()`, and repeats.

---

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

* **`rclcpp::TimerBase::SharedPtr`**: Standard ROS 2 C++ handle type for timers.
* **Lifetime rule**: If `timer_` is destroyed, the timer stops firing. This catches a lot of “why isn’t my callback running?” bugs.

---

### 2. Parameter Declaration (`lesson.cpp`)

In the constructor, we declare the parameter before we can use it:

```cpp
this->declare_parameter<double>("timer_period_s", 1.0);
```

* **Templated type**: `<double>` is the contract. If a user tries to set it to an incompatible type via CLI, ROS 2 rejects it.
* **Default value**: the node has deterministic behaviour even if no config is provided.

---

### 3. The Setup Helper

We use a helper method to keep the constructor clean. This pattern matters in real systems because it lets you recreate resources (timers, publishers, subscriptions) when config changes.

```cpp
void Lesson01Node::start_timer_from_param()
{
  timer_period_s_ = this->get_parameter("timer_period_s").as_double();

  if (timer_period_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "timer_period_s=%.6f is invalid; using 1.0s", timer_period_s_);
    timer_period_s_ = 1.0;
  }

  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_period_s_));

  timer_.reset();

  timer_ = this->create_wall_timer(period, [this]() { this->on_tick(); });
}
```

#### Why `std::chrono`?

ROS 2 timers expect `std::chrono::duration` types. Converting to nanoseconds is not “more real-time”; it’s about **precision and consistency**. You avoid accumulating floating point rounding errors in long-running loops.

#### Why a lambda instead of `std::bind`?

Older ROS 2 examples used:

```cpp
std::bind(&Lesson01Node::on_tick, this)
```

A lambda is preferred because:

* **Clearer**: the call is explicit (`this->on_tick()`), no placeholders, no mental parsing.
* **Better type checking**: the compiler verifies the lambda’s call operator matches what `create_wall_timer` expects. With `std::bind`, you often end up with a callable object with a messier, less transparent type.
* **Less error-prone captures**: `[this]` is explicit about what is being captured. `std::bind` can accidentally bind copies of values in surprising ways.
* **Modern C++ norm**: lambdas are the idiomatic callback mechanism in current C++ codebases.

In production code, “obvious is a feature.” Lambdas win.

---

### 4. The Timer Callback (`on_tick`)

```cpp
void Lesson01Node::on_tick()
{
  ++tick_;
  RCLCPP_INFO(this->get_logger(), "tick %llu", static_cast<unsigned long long>(tick_));
}
```

* **State mutation**: `tick_` is node-owned state; the callback is where event-driven state changes happen.
* **Logging format**: we cast to `unsigned long long` to match the expected `printf` formatting and avoid UB (undefined behaviour).

---

### 5. The Main Loop

```cpp
try {
  auto node = std::make_shared<Lesson01Node>();
  rclcpp::spin(node);

} catch (const std::exception & e) {
  RCLCPP_ERROR(rclcpp::get_logger("lesson_01_node_cpp"), "Exception in main: %s", e.what());
  rclcpp::shutdown();
  return EXIT_FAILURE;
}
```

* **Blocking behaviour**: `spin()` runs until shutdown (Ctrl+C or external shutdown).
* **Exception safety**: a top-level `try/catch` prevents uncontrolled termination if something throws unexpectedly.
* **Deterministic shutdown**: `rclcpp::shutdown()` ensures DDS resources are released cleanly.