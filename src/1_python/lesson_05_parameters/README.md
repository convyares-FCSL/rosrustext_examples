# Lesson 05 (Python) – Parameters & Central Configuration

## Goal

Revisit the publisher–subscriber pair using **fully parameterised configuration**.

By the end of this lesson, the system:

- Loads **topic names, QoS profiles, and behaviour** from central YAML files.
- Applies configuration at startup via ROS parameters.
- Safely updates supported parameters **at runtime** using callbacks.
- Exposes its effective configuration through standard ROS inspection tools.

This lesson establishes configuration as a **system-level concern**, not something embedded in node code.

> Assumes completion of Lessons 00–04.

---

## What’s New in Lesson 05

Compared to earlier lessons:

- **No hardcoded strings** for topics, QoS, or behaviour.
- **Multiple parameter files** are composed at startup.
- **Validation** happens before parameters take effect.
- **Runtime updates** are applied without restarting the node.

This is the point where “parameters exist” becomes “parameters are first-class”.

---

## Configuration Model

Lesson 05 uses **three shared YAML files**:

| File                   | Purpose                 |
|------------------------|-------------------------|
| `topics_config.yaml`   | Topic names             |
| `qos_config.yaml`      | QoS profiles + defaults |
| `services_config.yaml` | Service names           |

Each language consumes the same schema via its own thin utility layer.

Composition happens explicitly at the command line (launch wiring comes later).

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_py lesson_05_parameters_py --symlink-install
source install/setup.bash
````

If you are overriding an underlay-installed `utils_py`:

```bash
colcon build --packages-select utils_py --symlink-install --allow-overriding utils_py
source install/setup.bash
```

---

## Run Python unit tests (no ROS graph required)

```bash
colcon test --packages-select lesson_05_parameters_py
colcon test-result --verbose
```

---

## Run – Publisher

Run the publisher with all three config files composed:

```bash
ros2 run lesson_05_parameters_py lesson_05_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/services_config.yaml
```

**Expected behaviour:**

* No default warnings if YAML is loaded correctly.
* The node publishes `MsgCount` at the configured rate.
* Topic and QoS match the YAML definitions.

---

## Run – Subscriber

In a second terminal:

```bash
ros2 run lesson_05_parameters_py lesson_05_subscriber --ros-args \
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

Confirm reliability/durability/depth match `qos_config.yaml` and both nodes appear.

---

## Runtime Parameter Updates (Hot Reload)

### Change publish rate (publisher)

```bash
ros2 param set /lesson_05_publisher timer_period_s 0.2
```

* Validation runs before application.
* The timer is rebuilt safely.
* Publishing rate changes immediately.

### Adjust reset tolerance (subscriber)

```bash
ros2 param set /lesson_05_subscriber reset_max_value 5
```

* Listener behaviour updates in place.
* No node restart required.

Invalid values are rejected with clear reasons.

---

## Common Pitfall: `\` line continuation

When writing multi-line commands, the backslash must be the **final character** on the line.

Bad (has a trailing space after `\`):

```bash
--params-file .../topics_config.yaml \ 
--params-file ...
```

Good:

```bash
--params-file .../topics_config.yaml \
--params-file ...
```

If the line continuation breaks, ROS won’t receive later `--params-file` arguments and you’ll see default warnings.

---

## Architecture Notes

* Central YAML is the startup source of truth.
* ROS parameters are the live configuration.
* Utilities only:

  * read parameters
  * apply defaults
  * validate values
* Nodes:

  * declare parameters
  * own ROS resources
  * react to updates via callbacks

No file watching, no custom loaders, and no launch files yet. Those are introduced later on purpose.

---

## What This Lesson Proves

When Lesson 05 works correctly, you have demonstrated:

1. Cross-language configuration parity using shared YAML.
2. Safe runtime mutation of node behaviour.
3. Clear separation between configuration, logic, and ROS resources.
4. A production-grade pattern that scales beyond tutorials.

---

## What Comes Next

Lesson 06 introduces Lifecycle Nodes, where configuration and activation are no longer the same step.

Lesson 09 revisits this to wire the same YAML files automatically via launch/config discovery—without changing node code.