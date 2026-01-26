## Lesson 08 (Python) — Executors & Callback Groups

## Goal

Fix the Lesson 07 failure mode by restoring **responsiveness under long-running work**.

By the end of this lesson, the system:

* Runs a Lifecycle-managed node that publishes telemetry
* Accepts Action goals and streams feedback
* Remains responsive while an Action is executing
* Uses **MultiThreadedExecutor + explicit callback groups**

> Assumes completion of Lesson 07.

---

## What’s New in Lesson 08

Compared to Lesson 07:

* Switches from `SingleThreadedExecutor` → `MultiThreadedExecutor`
* Introduces explicit callback group isolation
* Keeps the Fibonacci action long-running (same interface / behaviour)
* Makes telemetry, lifecycle, and cancellation responsive under load

---

## Build (Terminal 1)

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_08_executors
source install/setup.bash
```

---

## Run – Subscriber (Lesson 06) (Terminal 2)

We reuse the subscriber from Lesson 06 to observe telemetry continuity.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** Node starts silent (Unconfigured).

---

## Run – Action Server + Publisher (Terminal 3)

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_08_executors lesson_08_action_server --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:** Node starts silent (Unconfigured).

---

## Drive Lifecycle (Back to Terminal 1)

### 1. Configure Both Nodes

```bash
ros2 lifecycle set /lesson_08_action_server configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
```

### 2. Activate Both Nodes

```bash
ros2 lifecycle set /lesson_08_action_server activate
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

**Result:** Telemetry begins flowing to the subscriber.

---

## Run – Action Client (Terminal 4)

```bash
ros2 run lesson_07_actions lesson_07_action_client --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
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

---

## What You Should Observe

While the Action Server is executing:

* **Telemetry continues** (no gaps in the subscriber)
* **Lifecycle commands remain responsive** (`ros2 lifecycle get` responds promptly)
* **Cancel requests take effect** (goal stops early and returns partial result)

---

## Tests

### Unit Tests

```bash
pytest src/1_python/lesson_08_executors/test/test_unit.py
```

### Integration Tests

```bash
colcon test --packages-select lesson_08_executors --return-code-on-test-failure
```