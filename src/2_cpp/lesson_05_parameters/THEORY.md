# Lesson 05 Theory: Parameters & Central Configuration (C++ / rclcpp)

## Architectural Intent

In Lesson 02, we hardcoded behavior (like the publish timer). In Lesson 05, we treat configuration as a **first-class system contract**.

The goal is to prove that node behavior can be driven, inspected, and safely updated at runtime using ROS parameters, while keeping a single shared YAML schema across languages.

We introduce three production-grade concepts:

1. **Two-Stage Configuration**: Loading defaults from YAML at startup, then modifying them via ROS parameters at runtime.
2. **Hot Updates**: Using C++ callbacks (`OnSetParametersCallback`) to react to changes without restarting the node.
3. **Logic Separation**: Keeping validation logic in pure C++ headers (`logic.hpp`), completely isolated from ROS 2 headers.

## Code Walkthrough

### 1. The Header: Logic vs. Transport

We explicitly separate the "business logic" (validating the stream) from the "transport mechanism" (ROS 2).

**`logic.hpp` (Pure C++)**
Notice there are no `#include <rclcpp/rclcpp.hpp>` directives here. This class handles state and validation rules only.

```cpp
// Pure C++ logic, testable without ROS
class TelemetryStreamValidator {
public:
  StreamDecision on_count(std::uint64_t count); // Returns decision enum
private:
  std::uint64_t expected_{0};
  std::uint64_t reset_max_value_{1};
};

```

**`subscriber.hpp` (ROS 2)**
The ROS node owns the validator and acts as the adapter.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "lesson_05_parameters_cpp/logic.hpp"

class Lesson05Subscriber : public rclcpp::Node {
  // ...
  // The node owns the logic instance
  std::unique_ptr<TelemetryListener> listener_; 
};

```

### 2. Shared Configuration (`utils_cpp`)

Just like in Lesson 02, we avoid hardcoding. However, `utils_cpp` now does more than just return strings—it handles the parameter declaration pattern.

```cpp
// 1. Get Topic Name (Standardized)
// Internally declares 'topics.telemetry' and returns the string "/tutorial/telemetry"
auto topic = topics::telemetry(*this);

// 2. Get QoS Profile (Configurable)
// Internally reads 'qos.profiles.telemetry.reliability', etc.
auto qos_profile = qos::telemetry(*this);

```

**Why generic templates?**
In C++, `utils_cpp` uses templates (`get_or_declare_parameter<T>`) to ensure type safety. If you try to read a string parameter into an `int` variable, the compiler or runtime validator will catch it immediately.

### 3. The Parameter Callback (RAII)

Enabling runtime updates in C++ requires registering a callback. **Crucially**, C++ uses RAII (Resource Acquisition Is Initialization) for this callback.

```cpp
// In class definition
OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

// In constructor
param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) { 
        return this->on_parameters(params); 
    });

```

* **The Handle**: We must store `param_callback_handle_` as a member variable. If we don't, the `SharedPtr` goes out of scope at the end of the constructor, the reference count drops to zero, and the callback is **immediately unregistered**.

### 4. Runtime Logic: Validate  Rebuild

When a parameter changes, we don't just update a variable. We follow a strict safety pattern.

**Publisher Example (`timer_period_s`)**

```cpp
rcl_interfaces::msg::SetParametersResult on_parameters(...) {
    // 1. Validate (Pure Logic)
    auto valid = validate_timer_period_s(param.as_double());
    if (!valid.ok) {
        // Reject the change at the ROS level
        return {false, valid.reason}; 
    }

    // 2. Rebuild Resource (ROS Level)
    // We cannot just change a timer's period; we recreate it.
    create_or_update_timer(*valid.value);
    
    return {true, ""};
}

```

This ensures the node never enters an invalid state (e.g., a negative timer period).

### 5. Memory Management

We use `std::unique_ptr` for the listener logic to ensure exclusive ownership by the node.

```cpp
listener_ = std::make_unique<TelemetryListener>(this->get_logger(), effective_value);

```

* **Efficiency**: The logic object is small and stays in memory.
* **Safety**: When the node is destroyed (Ctrl+C), the `unique_ptr` automatically destroys the listener, ensuring no memory leaks.

## Testing Strategy

Because we separated `logic.hpp` from the ROS node, our unit tests (`test/test.cpp`) do not need to spin up a ROS graph.

```cpp
TEST(TestLogic, StreamResetDetected) {
  TelemetryStreamValidator v(1); 
  v.on_count(10); 
  
  // We test the business logic directly
  auto decision = v.on_count(1);
  EXPECT_EQ(decision.event, StreamEvent::RESET);
}

```

This makes tests millisecond-fast and deterministic, proving the logic works before we ever try to send a ROS message.