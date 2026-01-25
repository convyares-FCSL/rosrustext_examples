# Lesson 08 (C++) — Executors & Callback Groups

## Goal

Restore **operational availability** to the system from Lesson 07 by changing the execution model.

By the end of this lesson, the system:

* Runs the same Lifecycle-managed Action Server as Lesson 07.
* Uses **MultiThreadedExecutor** to allow concurrent callback execution.
* Uses **Callback Groups** to isolate telemetry and action processing.
* **Observably recovers** responsiveness (telemetry, lifecycle) during long-running action goals.

> Assumes completion of Lesson 07.

---

## What’s New in Lesson 08

Compared to Lesson 07, the **code logic remains identical** except for:

* `SingleThreadedExecutor` → `MultiThreadedExecutor` (in `action_server_node.cpp`)
* Explicit assignment of **Callback Groups**:
  * Telemetry Timer → `MutuallyExclusiveCallbackGroup`
  * Action Server → `ReentrantCallbackGroup`

No changes are made to the business logic or `utils_cpp` interfaces.

---

## Build (Terminal 1)

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_08_executors_cpp
source install/setup.bash
```

---

## Run – Subscriber (Terminal 2)

Monitor telemetry to observe gaps (or lack thereof).

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

---

## Run – Action Server (Terminal 3)

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_08_executors_cpp lesson_08_action_server --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

---

## Drive Lifecycle (Back to Terminal 1)

Configure and activate both nodes to start the flow.

```bash
ros2 lifecycle set /lesson_08_action_server configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure

ros2 lifecycle set /lesson_08_action_server activate
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

**Observation:** Telemetry should begin scrolling in Terminal 2.

---

## Run – Action Client (Terminal 4)

Trigger the stress test.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_08_executors_cpp lesson_08_action_client --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

The client performs:
1.  **Short Goal** (Order 5): Should succeed normally.
2.  **Long Goal + Cancel** (Order 10): Runs for up to 10 seconds, then requests cancellation.

---

## Observability Contract

While the long-running action (Case 2) is executing:

1.  **Telemetry:** Should continue publishing without significant pauses.
    *   *Lesson 07 observation:* Telemetry stopped completely.
    *   *Lesson 08 observation:* Telemetry continues (interleaved).

2.  **Lifecycle:** `ros2 lifecycle get /lesson_08_action_server` should return immediately.

3.  **Cancellation:** The cancel request is processed by the action server while the goal execution loop continues (on a separate thread).

---

## Tests

### Integration Tests

Verify the fix using the black-box integration test:

```bash
colcon test --packages-select lesson_08_executors_cpp --return-code-on-test-failure
```
