# Lesson 06 Theory: Lifecycle Management (C++ / rclcpp_lifecycle)

## Architectural Intent

In previous lessons, nodes followed a "start-and-scream" pattern: they acquired resources and began publishing immediately upon construction.

In Lesson 06, we adopt the **Managed Node** pattern using the native `rclcpp_lifecycle` library. 

The goal is **Operational Determinism**: ensuring a node is fully configured and ready *before* it participates in the computational graph.

We introduce three production-grade concepts:

1.  **Native State Machine**: Inheriting from `rclcpp_lifecycle::LifecycleNode` instead of `rclcpp::Node`.
2.  **Interface Composition**: Writing shared logic that works across different node types (Standard vs. Lifecycle).
3.  **Black-Box Integration Testing**: Using Python harnesses to verify compiled C++ binaries.

---

## The Class Hierarchy: Node vs. LifecycleNode

A common misconception is that a `LifecycleNode` is just a `Node` with extra features. In `rclcpp`, they are **siblings**, not parent-child.

* **`rclcpp::Node`**: Wraps the standard interfaces for a generic ROS node.
* **`rclcpp_lifecycle::LifecycleNode`**: Wraps the *same* interfaces but exposes the Lifecycle State Machine API.

### The implication
You cannot pass a `LifecycleNode&` to a function expecting `rclcpp::Node&`. This strict type separation prevents developers from accidentally bypassing the state machine (e.g., by calling `create_publisher` instead of `create_publisher<LifecyclePublisher>`).

---

## Code Walkthrough

### 1. Interface-Based Logic (The "Advanced" Pattern)

Because `LifecycleNode` does not inherit from `Node`, standard helper libraries (like our `utils_cpp` from Lesson 05) will fail to compile if they expect `rclcpp::Node&`.

To solve this without code duplication, we use **Interface Composition**. Instead of asking for the whole Node, we ask for the specific *capability* we need.

**The Old Way (Lesson 05 - Brittle):**
```cpp
// Only works for standard Nodes
std::string get_param(rclcpp::Node& node, std::string name);
```

**The Production Way (Lesson 06 - Robust):**

```cpp
// Works for Node, LifecycleNode, and anything else that has parameters
std::string get_param(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface, 
  std::string name
);
```

This allows us to write configuration logic in `utils.hpp` that is universally compatible.

### 2. Gating Strategies

We must ensure no data flows when the node is `Inactive`.

**Publisher: Middleware Gating**
We use `rclcpp_lifecycle::LifecyclePublisher`.

* **Mechanism**: The DDS middleware handles the gating.
* **Behavior**: You can call `publish()` whenever you want, but if the state is not `Active`, the middleware silently drops the message.
* **Code**:
```cpp
// Logic runs, but data only leaves if Active
publisher_->publish(msg); 
```

**Subscriber: Manual Gating**
ROS 2 does not currently have a `LifecycleSubscription`. We must implement **Software Gating**.

* **Mechanism**: A boolean flag (`enabled_`) managed by state transitions.
* **Behavior**: The callback triggers, but we abort processing immediately if inactive.
* **Code**:
```cpp
void on_message(const MsgCount & msg) {
  if (!enabled_) return; // Explicit gate
  // ... process data ...
}
```

### 3. Parameter Persistence

In a Lifecycle Node, **Cleanup** (`Active` -> `Inactive` -> `Unconfigured`) is supposed to return the node to a fresh state. However, **Configuration Parameters** are often considered persistent infrastructure.

* **Destruction**: We destroy volatile resources (Timers, Publishers) in `on_cleanup`.
* **Persistence**: We *keep* the declared parameters. If we `undeclare` them, we lose the settings passed via the launch file, breaking the ability to re-configure the node later.

---

## Testing Strategy

We now have two distinct layers of verification:

### 1. Unit Tests (C++ / GTest)

* **Target**: `logic.hpp`
* **Scope**: Mathematical correctness of `StreamValidator`.
* **Execution**: Fast, no ROS graph required.

### 2. Integration Tests (Python / launch_testing)

* **Target**: The compiled C++ binary (`lesson_06_lifecycle_publisher`).
* **Scope**: Verifying the **Process** and **State Machine**.
* **Why Python?**: The ROS 2 Launch system is Python-based. It allows us to treat the C++ node as a "Black Box," launching it, waiting for it to spawn, and issuing Service Calls (`/change_state`) just like a real user would.

```python
# Python driving a C++ node
def test_lifecycle_sequence(self):
    # 1. Assert Start State
    self.assertEqual(self._get_state().label, 'unconfigured')
    
    # 2. Drive Transition
    self._change_state(Transition.TRANSITION_CONFIGURE)
    
    # 3. Assert New State
    self.assertEqual(self._get_state().label, 'inactive')
```

---

## Summary

Lesson 06 establishes that **Node Architecture** is distinct from **Business Logic**.

* The **Logic** (`logic.hpp`) calculates *what* to publish.
* The **Component** (`publisher.hpp`) handles *how* to publish (Gating).
* The **Node** handles *when* to publish (State Machine).

By separating these concerns, we create systems that are robust, testable, and ready for complex orchestration.