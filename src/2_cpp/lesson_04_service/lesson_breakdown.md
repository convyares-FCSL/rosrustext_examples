# Lesson 04 Breakdown: Services, Async Clients, and Testability (C++)

## Architectural Intent

Lesson 04 introduces the **Request-Response** paradigm, which differs fundamentally from the continuous data streams seen in Topics (Lesson 03).

The goal is to move beyond "making it work" and focus on **"making it verifiable"**.

We introduce two critical engineering standards:
1.  **Logic Separation**: Decoupling "Business Logic" (Math) from "Middleware" (ROS Node).
2.  **Asynchronous Client Design**: Preventing deadlocks in event-driven systems.

---

## Core Concept: Logic Separation (Header-Only)

A common anti-pattern in ROS 2 C++ development is writing complex algorithms directly inside the service callback.

```cpp
// ANTI-PATTERN
void handle_request(...) {
  // Complex math mixed with ROS logging and message types
  // Hard to test without running the full node!
}
```

Lesson 04 refactors this using **Composition** and a **Header-Only Library**:

```text
ServiceNode (ROS Wrapper)
 └── stats_logic.hpp (Pure C++)

```

### Why Header-Only?

We implemented `stats_logic.hpp` as a header-only library (`StatsResult` struct + `Logic` class).

1. **Simplicity**: It requires no separate compilation step or shared object (`.so`) management.
2. **Portability**: This logic relies only on the C++ Standard Library (`<numeric>`, `<vector>`). It has **zero ROS dependencies**.
3. **Testability**: It can be included directly into Google Test (`gtest`) executables.

---

## The Server Pattern: The Adapter

The `ServiceNode` does not perform calculations. It acts as an **Adapter** (or Wrapper).

Its responsibilities are strictly limited to:

1. **Deserialising**: Converting ROS `ComputeStats::Request` into `std::vector<double>`.
2. **Delegating**: Calling `stats_logic::Logic::compute()`.
3. **Serialising**: Converting the `StatsResult` back into ROS `ComputeStats::Response`.

This ensures that if we later decide to move this logic to a different framework (e.g., a web server or a CLI tool), the logic code remains untouched.

---

## The Client Pattern: Asynchronous Execution

Services are conceptually synchronous (Request → Wait → Response), but their implementation in ROS 2 **must be asynchronous** to avoid stalling the node.

### The Danger of Blocking Calls

If a node calls `client->async_send_request(req).get()` (blocking) inside a callback, it stops processing all other events (timers, subscriptions) until the server responds.

### The Solution: Callbacks

Lesson 04 demonstrates the robust pattern:

1. **Send**: `client_->async_send_request(request, callback)`
2. **Continue**: The node returns immediately and keeps spinning.
3. **React**: When the response arrives, the callback fires.

```cpp
auto future_callback = [this](SharedFuture future) {
  this->handle_response(future);
};
client_->async_send_request(request, future_callback);
```

This ensures the client application never hangs, even if the network is unreliable.

---

## Unit Testing (Google Test)

Because the logic is pure C++, we can verify it **at build time**.

In `CMakeLists.txt`:

```cmake
ament_add_gtest(test_stats_logic test/test_stats_logic.cpp)
target_link_libraries(test_stats_logic stats_logic)
```

This test:

1. Compiles instantly (no ROS graph overhead).
2. Verifies edge cases (empty lists, single elements).
3. Runs automatically in CI/CD pipelines via `colcon test`.

This provides a guarantee of algorithmic correctness *before* the code is ever deployed to a robot.

---

## Why This Lesson Matters

Lesson 04 proves that **testable code is better code**.

By strictly separating the "Business Logic" (Math) from the "Application Logic" (ROS Transport), we created a system where:

1. The math is verified via **Unit Tests** (`gtest`).
2. The communication is verified via **Integration Tests** (Manual CLI).
3. The configuration is centralized (`utils_cpp`).

This separation of concerns is the hallmark of professional C++ systems engineering.