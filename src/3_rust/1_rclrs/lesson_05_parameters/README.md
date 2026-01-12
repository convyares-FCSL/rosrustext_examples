# Lesson 05 (Rust / rclrs) – Parameters & Central Configuration

## Goal

Revisit the publisher–subscriber pair using **fully parameterised configuration**.

By the end of this lesson, the system:

* Loads **topic names, QoS profiles, and behaviour** from central YAML files.
* Applies configuration at startup via ROS parameters.
* Safely updates supported parameters **at runtime** using callbacks.
* Exposes its effective configuration through standard ROS inspection tools.

This lesson establishes configuration as a **system-level concern**, not something embedded in node code.

> Assumes completion of Lessons 00–04.

---

## What’s New in Lesson 05

Compared to earlier lessons:

* **No hardcoded strings** for topics, QoS, or behaviour.
* **Multiple parameter files** are composed at startup.
* **Validation** happens before parameters take effect.
* **Runtime updates** are applied without restarting the node.

This is the point where “parameters exist” becomes “parameters are first-class”.

---

## Architecture Overview (rclrs)

Lesson 05 deliberately splits responsibilities:

* **Pure Logic (no ROS)**
  Implemented in `lib.rs`:

  * `TelemetryPublisherCore`
  * `TelemetryStreamValidator`

* **ROS Middleware (rclrs nodes)**

  * Publisher:

    * Owns publisher resource and logic
    * Rebuilds timer on parameter updates
  * Subscriber:

    * Owns subscription resource
    * Mutates validation behaviour in-place on parameter updates

This allows logic to be tested independently using `cargo test`, while ROS nodes focus on wiring and lifecycle.

---

## Configuration Model

Lesson 05 uses **three shared YAML files**:

| File                   | Purpose                 |
| ---------------------- | ----------------------- |
| `topics_config.yaml`   | Topic names             |
| `qos_config.yaml`      | QoS profiles + defaults |
| `services_config.yaml` | Service names           |

Each language consumes the same schema via its own thin utility layer (`utils_rclrs` here).

Composition happens explicitly at the command line.
Launch files and discovery logic are introduced later on purpose.

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_rclrs lesson_interfaces
colcon build --packages-select lesson_05_parameters_rclrs
source install/setup.bash
```

If colcon warns about overriding packages from an underlay:

```bash
colcon build --packages-select utils_rclrs lesson_interfaces --allow-overriding utils_rclrs lesson_interfaces
```

---

## Verify: Unit Testing (Logic Only)

Because the business logic lives in `src/lib.rs`, it can be tested without ROS:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/1_rclrs/lesson_05_parameters
cargo test
```

Expected: all stream and publisher logic tests pass.

---

## Run – Publisher

Run the publisher with all three config files composed:

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
ros2 run lesson_05_parameters_rclrs lesson_05_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/services_config.yaml
```

**Expected behaviour:**

* No default warnings if YAML is loaded correctly.
* The node publishes `MsgCount` at the configured rate (1.0 Hz).
* Topic and QoS match the YAML definitions (`/tutorial/telemetry`).

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

The subscriber validates stream ordering and reacts to resets using parameters.

---

## Inspect the Running System

### List parameters

```bash
ros2 param list /lesson_05_publisher
ros2 param list /lesson_05_subscriber
```

### Inspect effective values

```bash
ros2 param get /lesson_05_publisher timer_period_s
ros2 param get /lesson_05_subscriber reset_max_value
```

### Verify transport configuration

```bash
ros2 topic info -v /tutorial/telemetry
```

Confirm reliability, durability, and depth match `qos_config.yaml`, and both nodes appear.

---

## Runtime Parameter Updates (Hot Reload)

### Change publish rate (Publisher)

```bash
ros2 param set /lesson_05_publisher timer_period_s 0.2
```

* Validation runs before application.
* The timer resource is rebuilt safely.
* Publishing rate changes immediately.

### Adjust reset tolerance (Subscriber)

```bash
ros2 param set /lesson_05_subscriber reset_max_value 5
```

* Validation logic updates in place.
* Subscription resource is not rebuilt.
* No node restart required.

**Try invalid values:**

```bash
ros2 param set /lesson_05_publisher timer_period_s -1.0
```

The request is rejected with a clear reason.

---

## Common Pitfall: `\` Line Continuation

When writing multi-line commands, the backslash must be the **final character** on the line.

Bad (trailing space after `\`):

```bash
--params-file .../topics_config.yaml \ 
--params-file ...
```

Good:

```bash
--params-file .../topics_config.yaml \
--params-file ...
```

If the line continuation breaks, ROS won’t receive later `--params-file` arguments and you’ll see default warnings from `utils_rclrs`.

---

## Architecture Notes

* **Central YAML** is the startup source of truth.

* **ROS parameters** are the live configuration.

* **Utilities** (`utils_rclrs`) only:

  * read parameters
  * apply defaults
  * warn on missing config

* **Nodes**:

  * declare parameters
  * own ROS resources
  * react to updates via callbacks

No file watching, no custom loaders, and no launch files yet. Those are introduced later on purpose.

---

## What This Lesson Proves

When Lesson 05 works correctly, you have demonstrated:

1. Cross-language configuration parity using shared YAML.
2. Safe runtime mutation of node behaviour (two different strategies).
3. Clear separation between configuration, logic, and ROS resources.
4. A production-grade pattern suitable for real systems.

---

## What Comes Next

Lesson 06 introduces **Lifecycle Nodes**, where configuration and activation are no longer the same step.