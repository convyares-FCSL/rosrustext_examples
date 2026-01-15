# Lesson 06 (Rust / rclrs) – Lifecycle Management

## Goal

Upgrade the parameterised nodes from Lesson 05 to fully managed **Lifecycle Nodes**.

By the end of this lesson, the system:

* Starts in an **Unconfigured** state (no resources allocated).
* Allocates resources (timers, publishers, logic) only during the **Configure** transition.
* Enables data flow only during the **Activate** transition.
* Reacts to parameter changes using the modern **ParameterWatcher** (v0.2.0) pattern.
* Cleans up deterministically during **Cleanup** or **Shutdown**.

This lesson establishes nodes as **managed components** rather than always-on scripts.

> Assumes completion of Lessons 00–05.

---

## What’s New in Lesson 06

Compared to earlier lessons:

* **State Machine**: Nodes have distinct states (`Unconfigured`, `Inactive`, `Active`, `Finalized`).
* **Deterministic Startup**: Launch order no longer dictates initialization order; an external manager controls when nodes configure.
* **Gated Execution**:
* Publishers/Timers are created "gated" (silent unless Active).
* Subscriptions are created "gated" (drop messages unless Active).


* **`rosrustext` Integration**: Uses `rosrustext_rosrs` v0.2.0 to provide the lifecycle state machine and parameter event stack missing from vanilla `rclrs`.

---

## Architecture Overview

Lesson 06 splits responsibilities to support the lifecycle state machine:

* **Pure Logic (unchanged)**
Implemented in `lib.rs`, tested independently.
* **Lifecycle Node (The Manager)**
The `Lesson06PublisherNode` struct implements `LifecycleCallbacksWithNode`:
* **`on_configure`**: Allocates the `PublisherComponent`, declares parameters, creates the `ManagedTimer`, and starts the `ParameterWatcher`.
* **`on_activate`**: Enables the data plane (transport).
* **`on_cleanup`**: Drops all handles (stops timers, frees DDS resources).


* **Passive Resources**
The `PublisherComponent` and `SubscriberComponent` are passive. They are allocated when needed and hold the business logic, but they do not control their own execution.

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_rclrs lesson_interfaces
colcon build --packages-select lesson_06_lifecycle_rclrs
source install/setup.bash

```

*Note: Ensure `rosrustext_rosrs` is present in your workspace or dependencies.*

---

## Verify: Unit Testing

Logic remains independent of the lifecycle wrapper, so tests still pass:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/1_rclrs/lesson_06_lifecycle
cargo test

```

---

## Run – Lifecycle Publisher

Run the publisher. Unlike previous lessons, it will start **silent**:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml

```

**Observation:**
The terminal will show:
`[INFO] ...: Node initialized (State: Unconfigured). Waiting for manager...`

It is **not** publishing yet. It hasn't even created the timer or declared parameters.

---

## Run – Lifecycle Subscriber

In a second terminal:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml

```

**Observation:**
Also starts silent. `Unconfigured`.

---

## Operate the Lifecycle (Manual Management)

We will use the CLI to drive the state machine manually.

### 1. Check Status

In a third terminal:

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
ros2 lifecycle get /lesson_06_lifecycle_subscriber

```

*Output: `unconfigured*`

### 2. Configure (Allocate Resources)

Trigger resource creation (parameters declared, timers created).

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure

```

*Output: `Transitioning: Unconfigured -> Inactive*`

**Status:** Nodes are now **Inactive**.

* Parameters exist and can be set.
* Topics exist.
* **BUT:** No data flows. The timers are gated.

### 3. Activate (Enable Data Flow)

Enable the transport layer.

```bash
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
ros2 lifecycle set /lesson_06_lifecycle_publisher activate

```

*Output: `Transitioning: Inactive -> Active*`

**Observation:**

* The publisher starts logging "Stream initialized".
* The subscriber starts receiving data.
* Data flow is now live.

### 4. Deactivate (Pause)

Pause execution without destroying resources.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate

```

**Observation:** Publishing stops immediately. The node is dormant but still configured.

### 5. Cleanup (Destroy Resources)

Return to fresh state.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher cleanup

```

**Observation:** All handles (timers, watchers, components) are dropped. The node is back to `Unconfigured`.

---

## Runtime Parameter Updates

Parameters can be updated while the node is **Configured** (Inactive or Active).

1. Ensure the publisher is **Active**.
2. Change the rate:

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s 0.2

```

**Observation:**

* The `ParameterWatcher` catches the event.
* The logic validates the value (> 0).
* The `ManagedTimer` is hot-swapped safely.
* The publish rate increases immediately.

**Try invalid values:**

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s -1.0

```

* The update is logged as ignored (warn), and the node continues running safely.

---

## Architecture Notes

* **Unconfigured**: Zero footprint. No timers, no subscribers, no parameters declared.
* **Configured (Inactive)**: Resources exist but are gated. Used for "warm standby".
* **Active**: Running.
* **Cleanup**: Crucial for avoiding resource leaks. We explicitly drop `Arc` handles to release the underlying DDS entities.
* **ParameterWatcher**:
* Created in `on_configure`.
* Dropped in `on_cleanup`.
* This ensures we don't process parameter events when the node is supposed to be unconfigured.



---

## What This Lesson Proves

When Lesson 06 works correctly, you have demonstrated:

1. **Managed Startup**: Nodes do not run until explicitly told to.
2. **Resource Hygiene**: Resources are allocated and freed deterministically.
3. **Gated Execution**: Logic runs only when the state machine permits.
4. **Modern Parity**: Implementing standard ROS 2 lifecycle behavior in Rust using `rosrustext`.

---
