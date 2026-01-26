# Lesson 06 Intent: Lifecycle Management

**Lesson 06 establishes the "Managed Node" pattern.**

This lesson refactors the Publisher and Subscriber to adhere to the standard **ROS 2 Lifecycle State Machine**. Instead of "starting and screaming" immediately, nodes must wait for explicit orchestration.

---

## The Vehicle

* A **Lifecycle Publisher** that automatically gates data flow based on its state.
* A **Lifecycle Subscriber** that creates/destroys subscriptions based on state transitions.
* A **Custom Lifecycle Manager** (A standard ROS node) that orchestrates the transition of the fleet via service calls.
* **Automated Integration Tests** (`launch_testing`) to verify state contracts.
* **Nav2 Integration** to demonstrate compliance with industry-standard toolchains.

---

## The Source of Truth

* **The ROS 2 Lifecycle State Machine** (Standard Definition).
* **Transition Map**:
* `Unconfigured`  `Inactive` (Configure)
* `Inactive`  `Active` (Activate)
* `Active`  `Inactive` (Deactivate)
* `Inactive`  `Finalized` (Shutdown)



This state machine is the **single source of truth** for node behaviour.

---

## What the System Must Do

When Lesson 06 is running correctly:

1. **Silence on Startup**: Nodes launch but do *not* advertise topics or start timers (`Unconfigured`).
2. **Explicit Configuration**: Resources are allocated only upon the `configure` transition.
3. **Gated Execution**:
* Data flow is gated by the middleware (using `LifecyclePublisher` / `LifecycleTimer`).


4. **Standard Compliance**: All four implementations (Python, C++, rclrs, roslibrust) must respond identically to standard service calls (`/get_state`, `/change_state`).
5. **Automated Verification**:
* A `launch_test` must pass that asserts:
* Node starts in `Unconfigured`.
* Transition to `Inactive` creates topics but sends no data.
* Transition to `Active` starts data flow.





---

## The New Concept: Managed Availability

Lesson 06 moves from *"I am running"* to *"I am ready"*.

* **Process vs. State**:
* **Lesson 09 (Launch)** handles the **Process** (spawning the binary).
* **Lesson 06 (Lifecycle)** handles the **State** (waking up the logic inside the binary).


* **Deterministic Startup**:
We can launch 50 nodes (Process), wait for them all to be available, and then configure them synchronously (State).

---

## Architecture (As Implemented)

### 1. State Machine Ownership

* **Python**: Inherit `rclpy.lifecycle.NodeLifecycle`.
* **C++**: Inherit `rclcpp_lifecycle::LifecycleNode`.
* **Rust (rclrs)**: Use `rosrustext_rclrs` to implement standard lifecycle traits.
* **Rust (roslibrust)**: Use `rosrustext_roslibrust` to provide the "Virtual Lifecycle Node" over the bridge.

### 2. Gated Resources

We avoid manual boolean flags in user code. We use library-provided lifecycle primitives:

* **Lifecycle Publishers**: Automatically mute messages when the node is not active.
* **Lifecycle Timers**: Automatically pause execution when the node is not active.

### 3. File Structure

The workspace adds a new orchestration layer:

```text
src/
├─ 1_python/lesson_06_lifecycle/   # Native rclpy Lifecycle Node
├─ 2_cpp/lesson_06_lifecycle/      # Native rclcpp Lifecycle Node
├─ 3_rust/
│   ├─ 1_rclrs/lesson_06_lifecycle/      # rosrustext_rclrs Lifecycle Node
│   └─ 2_rcllibrust/lesson_06_lifecycle/ # rosrustext_roslibrust Lifecycle Client
└─ 6_orchestration/                # NEW: Managers & Integration Tests
    └─ lesson_06_manager/
       ├─ src/manager_node.py      # Custom Lifecycle Manager (ROS Node)
       └─ test/test_lifecycle.py   # launch_testing suite

```

---

## Artifacts Produced by Lesson 06

1. **4x Lifecycle Nodes**: Fully compliant managed nodes in all target languages.
2. **Custom Manager Node**: A Python ROS node that issues service calls (`Configure`, `Activate`) to drive the fleet.
3. **Integration Test Suite**: Automated tests verifying the state machine contract using `launch_testing`.
4. **Nav2 Launch File**: A proof-of-concept launch file using `nav2_lifecycle_manager` to drive our custom nodes.

---

## What This Lesson Establishes

Lesson 06 establishes **Operational Maturity**.

After this lesson, our nodes are no longer standalone scripts; they are **managed components** ready for integration into complex systems (like Navigation or Manipulation stacks), verified by automated integration tests.