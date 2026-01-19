# Lesson 06 (Python) – Lifecycle Management

## Goal

Upgrade the previous Publisher/Subscriber nodes to be **Managed Lifecycle Nodes**.

By the end of this lesson, the system:
- Starts in an `Unconfigured` state (Silent).
- Transitions through the standard **ROS 2 Lifecycle State Machine**:
  - `Unconfigured` → `Inactive` (Resources created, paused)
  - `Inactive` → `Active` (Data flows)
  - `Active` → `Inactive` (Data paused)
  - `*` → `Finalized` (Cleanup and exit)
- Responds to standard administrative service calls (`/get_state`, `/change_state`).

This lesson establishes **Operational Maturity**: nodes that wait for orchestration rather than "starting and screaming" immediately.

> Assumes completion of Lesson 05.

---

## What’s New in Lesson 06

Compared to earlier lessons:
- **No Automatic Start**: Nodes launch but do nothing until configured.
- **State-Based Resources**: Topics and timers are created/destroyed dynamically based on the state.
- **Manual Gating (Python)**: Explicit checks prevent data flow when the node is not `Active`.
- **Integration Testing**: We verify the *process* of starting up, not just the logic.

---

## The Lifecycle State Machine

The **Single Source of Truth** for node behavior is the standard ROS 2 state machine:

| State | Definition | Permitted Actions |
| :--- | :--- | :--- |
| **Unconfigured** | The node object exists, but has no configuration. | Load parameters. |
| **Inactive** | The node is configured (parameters loaded, topics advertised), but passive. | Inspect configuration. No publishing. |
| **Active** | The node is fully operational. | Publish data, run control loops. |
| **Finalized** | The node is destroyed. | None. |

### Transitions
Transitions are triggered by external service calls (e.g., from a Manager Node or CLI).
1. **Configure** (`Unconfigured` → `Inactive`): Allocates resources (topics, timers).
2. **Activate** (`Inactive` → `Active`): Enables data flow (opens the gate).
3. **Deactivate** (`Active` → `Inactive`): Disables data flow (closes the gate).
4. **Cleanup** (`Inactive` → `Unconfigured`): Destroys resources (resets to fresh state).

---

## Python Implementation Details (The Shim)

Unlike C++, `rclpy` does not yet have a native `LifecycleNode` class. To achieve compliance, we use a custom **Shim** (`utils_py.LifecycleNode`).

### 1. The Service Interface
The Shim implements the standard services so external tools treat the Python node exactly like a native C++ node:
* `~/get_state`
* `~/change_state`
* `~/get_available_transitions`

### 2. Manual Gating
In C++, the middleware automatically blocks publication when `Inactive`. In Python, we must implement **Software Gating**:
* **Publishers**: We check `if self._is_enabled:` before publishing.
* **Subscribers**: We check `if self._is_enabled:` inside the callback before processing data.

### 3. The Threading Requirement
Because the Lifecycle Shim relies on ROS Services to change state, the node must be able to process a "Configure" request even while it is doing other work.
* **Problem**: `rclpy.spin()` is single-threaded by default. If the node blocks, the management services hang.
* **Solution**: We use `rclpy.executors.MultiThreadedExecutor` and assign the lifecycle services to a `ReentrantCallbackGroup`.

### 4. Service Naming & Remapping
To ensure the node works with **ROS 2 Remapping** (e.g., in integration tests), services are declared using relative names (`~/get_state`).
* **Incorrect**: `f"{node_name}/get_state"` (Hardcodes the name, ignores remapping).
* **Correct**: `"~/get_state"` (Expands to `/remapped_name/get_state`).

---

## Architecture

```text
       +-----------------------+
       |   Lifecycle Manager   |
       |  (CLI or Test Script) |
       +-----------+-----------+
                   |
          Service Calls (Standard)
                   |
       +-----------v-----------+
       |   Managed Node (Py)   |
       |                       |
       |  +-----------------+  |
       |  | Lifecycle Shim  |  | <--- Handles State Machine
       |  +--------+--------+  |
       |           |           |
       |    +------v------+    |
       |    |  User Logic |    | <--- Implements on_configure(), on_activate()
       |    +-------------+    |
       +-----------------------+

```

---

## What This Lesson Proves

When Lesson 06 works correctly, you have demonstrated:

1. **Determinism**: You can launch 50 nodes and ensure they are all configured before a single message is sent.
2. **Standard Compliance**: Your Python node behaves exactly like a system-level C++ component (e.g., Nav2).
3. **Testability**: You can write integration tests that drive the node through its entire lifecycle programmatically.

---

## What Comes Next

We will implement the exact same architecture in **C++** (Lesson 06 C++).
Because C++ has native support (`rclcpp_lifecycle`), we will delete the Shim code and use the middleware's built-in gating, proving that the architecture persists even when the language changes.

```

```