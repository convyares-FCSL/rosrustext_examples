# utils_rclrs (Rust Shared Utilities)

`utils_rclrs` is the shared Rust utility crate for this workspace.

It centralises:
- **Topic names** (`utils_rclrs::topics`)
- **QoS profiles** (`utils_rclrs::qos`)
- **Service names** (`utils_rclrs::services`)
- A small parameter helper (`utils_rclrs::utils`) used by all of the above

This keeps lesson nodes free of hardcoded strings and makes config consistent across languages.

---

## Design Rules

- Nodes should **not** hardcode topic/service strings.
- Nodes should obtain config through `utils_rclrs` helpers.
- Defaults exist to keep examples runnable, but:
  - If no external override is provided (e.g. no `--params-file`), we **warn**.
  - If a value is overridden via ROS parameters, we do **not** warn.

---

## How Configuration Works

All helpers follow the same pattern:

1. **Declare** the parameter (so it appears in `ros2 param list/get`).
2. **Read** the effective value.
3. **Warn** only if the value equals the default **and** it was not overridden externally.

The shared helper is:

- `utils_rclrs::utils::get_or_declare_parameter(node, name, default, warn_label)`

---

## Topics

Module: `utils_rclrs::topics`

Exposes canonical getters:

* `topics::chatter(node)` (Lessons 00–04)
* `topics::telemetry(node)` (Lesson 05+)

They resolve ROS parameters like:

* `topics.chatter`
* `topics.telemetry`

---

## Services

Module: `utils_rclrs::services`

Exposes canonical getters:

* `services::compute_stats(node)`

Resolves ROS parameters like:

* `services.compute_stats`

---

## QoS

Module: `utils_rclrs::qos`

Provides:

* `qos::from_parameters(node)` (uses `qos.default_profile`)
* explicit profile getters:
  * `qos::telemetry(node)`
  * `qos::commands(node)`
  * `qos::state_latched(node)`
  * etc...

These resolve ROS parameters like:

* `qos.default_profile`
* `qos.profiles.telemetry.reliability`
* `qos.profiles.telemetry.durability`
* `qos.profiles.telemetry.depth`

---

## Build

From workspace root:

```bash
colcon build --packages-select utils_rclrs