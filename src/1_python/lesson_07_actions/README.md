## Lesson 07 (Python) — Actions Under Lifecycle

## Goal

Extend the **Lifecycle Publisher** node to also host a **ROS 2 Action Server** and observe how **long-running action execution affects node responsiveness**.

By the end of this lesson, the system:

* Runs a Lifecycle-managed node that publishes telemetry
* Accepts Action goals and streams feedback
* Exhibits degraded responsiveness while an Action is executing
* Remains **intentionally single-threaded**

> Assumes completion of Lesson 06.

---

## What’s New in Lesson 07

Compared to Lesson 06:

* Adds a **Fibonacci Action Server**
* Adds a **standalone Action Client**
* Introduces **long-running callbacks**
* Demonstrates a failure mode that will be fixed in Lesson 08

---

## Build (Terminal 1)

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_interfaces lesson_07_actions
source install/setup.bash
```

---

## Run – Subscriber (Lesson 06) (Terminal 2)

We reuse the subscriber from Lesson 06 to observe telemetry behavior.

```bash
ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** Node starts silent (Unconfigured).

---

## Run – Action Server + Publisher (Terminal 3)

```bash
ros2 run lesson_07_actions lesson_07_action_server --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** Node starts silent (Unconfigured).

---

## Drive Lifecycle (Back to Terminal 1)

### 1. Configure Both Nodes

```bash
ros2 lifecycle set /lesson_07_action_server configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
```

### 2. Activate Both Nodes

```bash
ros2 lifecycle set /lesson_07_action_server activate
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

**Result:**
Telemetry begins flowing to the subscriber.

---

## Run – Action Client (Terminal 4)

```bash
ros2 run lesson_07_actions lesson_07_action_client \
  --ros-args -r fibonacci:=/lesson_07_action_server/fibonacci
```

The client demonstrates two cases:

### Case 1 — Success

* Sends a goal (order = 5)
* Receives feedback
* Receives a successful result

### Case 2 — Cancellation

* Sends a goal (order = 10)
* Waits 3 seconds
* Sends cancel request

**Observation:**
Cancellation is delayed or ineffective while the action executes.

---

## What You Should Observe

While the Action Server is executing:

* **Telemetry pauses** in the subscriber
* **Lifecycle commands** (`ros2 lifecycle get`) may delay
* **Cancel requests** are not immediately processed

This behavior is **expected** in Lesson 07.

---

## Tests

### Unit Tests

```bash
pytest src/lesson_07_actions/test/test_unit.py
```

### Integration Tests

```bash
colcon test --packages-select lesson_07_actions --return-code-on-test-failure
```

---

**Lesson 07 is complete when you can reliably reproduce the responsiveness degradation during action execution.**

---
