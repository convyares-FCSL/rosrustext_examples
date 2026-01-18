# Lesson 06: Lifecycle Management (Roslibrust)

## Design Summary

In this lesson, we implement **Lifecycle Management** for ROS 2 nodes using the **roslibrust** client library. Unlike `rclcpp` or `rclrs`, `roslibrust` functions as a bridge client and does not currently provide a built-in Lifecycle Node framework. Alternatively, we implement a **"Virtual Lifecycle Node"** pattern.

### 1. State Machine
We implement a simplified version of the standard ROS 2 Activity state machine with the following states:
*   **Unconfigured (1)**: The node is initialized but holds no functional resources.
*   **Inactive (2)**: Resources (publishers, subscribers) are created but "gated" (output is suppressed/ignored).
*   **Active (3)**: The node is fully functional; the publisher sends data, and the subscriber processes it.
*   **Finalized (4)**: Terminal state before shutdown.

### 2. Resource Management
Resources are managed dynamically based on state transitions:
*   **Publisher**: Created during `Configure`. The publishing loop runs continuously but only sends messages when in the `Active` state ("gating"). Dropped during `Cleanup`.
*   **Subscriber**: Created during `Configure`. The callback receives messages but logic is only executed if `Active`.
*   **Timer**: Implemented as a `tokio` task that sleeps and checks state.

### 3. Services
To interact with standard ROS 2 tools, we manually expose a subset of lifecycle and parameter services:
*   `~/change_state`: Accepts a `Transition` (e.g., Configure, Activate) to drive the state machine.
*   `~/get_state`: Returns the current lifecycle state.
*   `~/set_parameters`: Allows runtime updates via `ros2 param set`, maintaining parity with standard nodes.

### 4. Parameter Watcher
In addition to the `set_parameters` service, we implement a **Parameter Watcher** that subscribes to the global `/parameter_events` topic. This allows the node to react to parameter changes broadcast by other entities, demonstrating a reactive pattern common in distributed systems.
