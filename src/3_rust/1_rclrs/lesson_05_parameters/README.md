# Lesson 05 (Rust / rclrs) – Parameters & Central Configuration

## Goal

Revisit the publisher–subscriber pair using **fully parameterised configuration**.

By the end of this lesson, the system:

* Loads **topic names, QoS profiles, and behaviour** from central YAML files (via ROS parameters).
* Applies configuration at startup using declared parameter defaults + YAML overrides.
* Applies supported parameter updates **at runtime** using `ParameterWatcher` callbacks.
* Exposes effective configuration using standard ROS 2 inspection tools.

This lesson treats configuration as a **system-level contract**, not node-local code.

> Assumes completion of Lessons 00–04.

---

## What’s New in Lesson 05

Compared to earlier lessons:

* No hardcoded strings for topics or QoS.
* Multiple parameter files are composed at startup.
* Behavioural parameters are validated before being applied.
* Runtime updates are applied without restarting the process.

---

## Architecture Overview (rclrs)

Lesson 05 splits responsibilities:

### Pure Logic (no ROS)
Implemented in `lib.rs`:

* `TelemetryPublisherCore`
* `TelemetryStreamValidator`

These are unit-testable without ROS 2.

### Middleware Adapters (ROS nodes)
Implemented in `src/bin/*.rs`:

* Publisher:
  * Owns publisher resource and core logic
  * Rebuilds the publish timer when `timer_period_s` changes
* Subscriber:
  * Owns subscription resource
  * Updates validator behaviour in-place when `reset_max_value` changes

---

## Configuration Model

Lesson 05 uses shared YAML files:

| File                 | Purpose                 |
| -------------------- | ----------------------- |
| `topics_config.yaml` | Topic names             |
| `qos_config.yaml`    | QoS profiles + defaults |

YAML is applied by ROS 2 before node startup. Each node declares the parameters it expects and reads the effective values (default or overridden).

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_rclrs lesson_interfaces
colcon build --packages-select lesson_05_parameters_rclrs
source install/setup.bash
````

If colcon warns about overriding packages from an underlay:

```bash
colcon build --packages-select utils_rclrs lesson_interfaces --allow-overriding utils_rclrs lesson_interfaces
```

---

## Verify: Unit Testing (Logic Only)

The business logic is middleware-free, so it can be tested without ROS:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/1_rclrs/lesson_05_parameters
cargo test
```

Expected: logic tests pass (stream behaviour + time conversion).

---

## Run – Publisher

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
ros2 run lesson_05_parameters_rclrs lesson_05_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

Expected behaviour:

* The node publishes `MsgCount` on the configured telemetry topic.
* The publish period defaults to `timer_period_s = 1.0` unless overridden.

---

## Run – Subscriber

In a second terminal:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
ros2 run lesson_05_parameters_rclrs lesson_05_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

Expected behaviour:

* The subscriber logs stream validation decisions.
* The reset tolerance defaults to `reset_max_value = 1` unless overridden.

---

## Inspect the Running System

List parameters:

```bash
ros2 param list /lesson_05_publisher
ros2 param list /lesson_05_subscriber
```

Read effective values:

```bash
ros2 param get /lesson_05_publisher timer_period_s
ros2 param get /lesson_05_subscriber reset_max_value
```

Inspect transport configuration:

```bash
ros2 topic info -v /tutorial/telemetry
```

---

## Runtime Parameter Updates

### Change publish rate (Publisher)

```bash
ros2 param set /lesson_05_publisher timer_period_s 0.2
```

Expected behaviour:

* The update is validated.
* The publish timer is rebuilt with the new period.
* The publish rate changes immediately.

Invalid values are ignored:

```bash
ros2 param set /lesson_05_publisher timer_period_s -1.0
```

### Adjust reset tolerance (Subscriber)

```bash
ros2 param set /lesson_05_subscriber reset_max_value 5
```

Expected behaviour:

* The update is validated.
* The validator threshold is updated in-place.
* Subsequent messages use the new threshold immediately.

Invalid values are ignored:

```bash
ros2 param set /lesson_05_subscriber reset_max_value -1
```

---

## Notes

* `utils_rclrs` provides centralised access to topic names and QoS profiles via parameters.
* Nodes declare their own behavioural parameters (`timer_period_s`, `reset_max_value`) and apply validated updates via `ParameterWatcher`.
