# Lesson 06 (Python) – Lifecycle Management

## Goal

Upgrade the previous Publisher/Subscriber nodes to be **Managed Lifecycle Nodes**.

By the end of this lesson, the system:
* Starts in an `Unconfigured` state (no ROS resources created).
* Transitions through a standard state machine: `Unconfigured` → `Inactive` → `Active`.
* Only processes data when `Active`.
* Can be cleanly shut down or reset via standard ROS 2 lifecycle commands.

> Assumes completion of Lesson 05.

---

## Build (Terminal 1)

**Role:** This terminal is for **building** and sending **commands**.

```bash
cd ~/ros2_ws_tutorial
# Rebuild is required because we modified the utils_py library
colcon build --packages-select utils_py lesson_06_lifecycle_py --symlink-install
source install/setup.bash
```

---

## Run – Lifecycle Publisher (Terminal 2)

**Role:** This terminal runs the **Publisher** node.

Open a new terminal:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** The terminal sits silently. The node is running but unconfigured.

---

## Run – Lifecycle Subscriber (Terminal 3)

**Role:** This terminal runs the **Subscriber** node.

Open a new terminal:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** This terminal also sits silently.

---

## Orchestrating the System (Back to Terminal 1)

Return to **Terminal 1**. We will now drive the state machine manually.

### 1. Check Initial State

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
ros2 lifecycle get /lesson_06_lifecycle_subscriber
# Output for both should be: unconfigured
```

### 2. Configure Both (Create Resources)

This loads parameters and creates topics, but keeps them paused.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
```

*Check Terminals 2 & 3: You should see "Configuring..." logs.*

### 3. Activate Subscriber FIRST (Deterministic Startup)

We open the subscriber's gate first so it doesn't miss anything.

```bash
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

*Check Terminal 3: It says "Subscriber Activated" (or similar).*

### 4. Activate Publisher LAST

Now we start the data flow.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher activate
```

**Result:** Look at **Terminal 3 (Subscriber)**. You should see it catch **Count: 0** immediately. This proves zero data loss startup.

### 5. Deactivate (Pause)

You can stop the flow without killing the nodes.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate
```

---

## Automated Testing

This package includes both **Unit Tests** (logic checks) and **Integration Tests** (full state machine verification).

```bash
# Run all tests for this package
colcon test --packages-select lesson_06_lifecycle_py --return-code-on-test-failure --event-handlers console_direct+
```

<details>
<summary><strong>Advanced: Run Stress Test (5x Loop)</strong></summary>

To verify reliability and ensure no threading race conditions exist, you can run the integration test in a loop:

```bash
for i in {1..5}; do
   echo "--- Run #$i ---"
   colcon test --packages-select lesson_06_lifecycle_py \
     --pytest-args -k "test_lifecycle_integration" \
     --return-code-on-test-failure \
     --event-handlers console_cohesion+ || break
done
```

</details>

---

## Architecture Notes

* **Inheritance**: Nodes inherit from our custom `utils_py.LifecycleNode` shim (because `rclpy` lacks native support).
* **Manual Gating**: Since we are using a shim, we manually gate execution using `if self._is_enabled:` in both the Publisher and Subscriber.
* **Service Naming**: The Shim uses `~/get_state` (relative expansion) instead of `node_name/get_state`. This is critical for supporting standard ROS 2 node remapping during integration testing.

---

## What This Lesson Proves

1. **Gated Execution**: Nodes only run when explicitly told to.
2. **Determinism**: The ability to guarantee system readiness before motion/data begins.

```

**This concludes Lesson 06 (Python). We are now ready to begin C++.**