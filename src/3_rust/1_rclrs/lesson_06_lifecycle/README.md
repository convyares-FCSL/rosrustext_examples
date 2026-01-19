# Lesson 06 (Rust / rclrs) – Lifecycle Management

## Goal

Convert the Publisher and Subscriber nodes from Lesson 05 into **Managed Lifecycle Nodes**.

The resulting system:

* Starts in an `Unconfigured` state with no runtime resources allocated.
* Transitions through the standard ROS 2 lifecycle states:
  `Unconfigured` → `Inactive` → `Active`.
* Processes data only while in the `Active` state.
* Supports deterministic shutdown and reset using standard ROS 2 lifecycle commands.

> Assumes completion of Lessons 00–05.

---

## Build (Terminal 1)

**Role:** This terminal is used for **building** and issuing **lifecycle commands**.

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_rclrs lesson_interfaces
colcon build --packages-select lesson_06_lifecycle_rclrs
source install/setup.bash
```

> Ensure `rosrustext_rosrs` is available in the workspace or as a dependency.

---

## Run – Lifecycle Publisher (Terminal 2)

**Role:** This terminal runs the **Publisher** node.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:**
The node starts and logs that it is waiting in the `Unconfigured` state.
No parameters are declared and no timers or publishers are created.

---

## Run – Lifecycle Subscriber (Terminal 3)

**Role:** This terminal runs the **Subscriber** node.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_rclrs lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:**
The subscriber also remains idle in the `Unconfigured` state.

---

## Orchestrating the System (Back to Terminal 1)

Lifecycle transitions are driven manually using the ROS 2 CLI.

### 1. Check Initial State

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
ros2 lifecycle get /lesson_06_lifecycle_subscriber
```

Expected output for both:

```
unconfigured
```

---

### 2. Configure Both Nodes (Allocate Resources)

This transition declares parameters, creates topics, and allocates internal resources, but does not enable data flow.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
```

**Observation:**

* Nodes transition to `Inactive`.
* Parameters exist and can be modified.
* Publishers, subscriptions, timers, and watchers are created.
* Data-plane execution remains gated.

---

### 3. Activate Subscriber First

Activating the subscriber first ensures it is ready before data production begins.

```bash
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

**Observation:**
The subscriber transitions to `Active` but receives no data yet.

---

### 4. Activate Publisher

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher activate
```

**Observation:**

* The publisher begins producing messages.
* The subscriber receives the first sample immediately (starting at count `0`).
* No messages are lost during startup.

---

### 5. Deactivate (Pause Execution)

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate
```

**Observation:**
Publishing stops immediately.
Resources remain allocated, and the node stays in the `Inactive` state.

---

### 6. Cleanup (Release Resources)

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher cleanup
```

**Observation:**

* Timers, publishers, parameter watchers, and internal components are dropped.
* The node returns to the `Unconfigured` state.
* No background activity remains.

---

## Runtime Parameter Updates

Parameters can be modified while the node is `Inactive` or `Active`.

### Example: Change Publish Rate

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s 0.2
```

**Behavior:**

* The `ParameterWatcher` receives the update.
* The value is validated.
* The existing timer is replaced with a new gated timer.
* The updated rate takes effect immediately if the node is `Active`.

### Invalid Values

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s -1.0
```

**Behavior:**
The update is rejected and logged.
The existing configuration remains unchanged.

---

## Automated Testing

### 1. Unit Tests (Logic Verification)
Verifies the business logic (validators, math) without loading the ROS middleware.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/1_rclrs/lesson_06_lifecycle
cargo test
# OR
colcon test --packages-select lesson_06_lifecycle_rclrs --event-handlers console_direct+
```

### 2. Integration Tests (System Verification)
Verifies the node's state machine by treating the compiled binary as a black box.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
launch_test src/3_rust/1_rclrs/lesson_06_lifecycle/test/test_integration.py
```

<details>
<summary><strong>Advanced: Run Stress Test (5x Loop)</strong></summary>

To verify reliability and ensure no race conditions exist during transitions, you can run the test suite in a loop:

```bash
for i in {1..5}; do
   echo "--- Run #$i ---"
   launch_test src/3_rust/1_rclrs/lesson_06_lifecycle/test/test_integration.py || break
done
```

</details>

---



## Architecture Notes

* **Lifecycle Implementation**
  Lifecycle behavior is implemented using `rosrustext_rosrs`, providing a state machine and parameter event handling absent from core `rclrs`.

* **Publisher Gating**
  Timers and publishers are created as *gated resources*. Execution occurs only while the node is `Active`.

* **Subscriber Gating**
  Subscriptions are created as gated subscriptions and drop incoming samples while inactive.

* **Resource Ownership**
  All runtime resources are allocated during `on_configure` and released during `on_cleanup` or `on_shutdown`.

* **ParameterWatcher Lifetime**
  Parameter watchers are created during `on_configure` and dropped during cleanup to prevent handling updates while unconfigured.

---

## Resulting System Properties

* Nodes do not allocate runtime resources until explicitly configured.
* Data-plane execution occurs only while nodes are in the `Active` state.
* Startup order is deterministic and controlled via lifecycle transitions.
* Resources are released deterministically during cleanup and shutdown.
* Runtime reconfiguration is handled safely without restarting nodes.
