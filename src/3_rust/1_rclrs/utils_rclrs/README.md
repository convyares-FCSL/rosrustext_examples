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
- Defaults are provided to keep examples runnable, but can be overridden via standard ROS 2 mechanisms (CLI args or YAML files).

---

## How Configuration Works

All helpers follow the same pattern:

1. **Declare** the parameter via `rclrs` (making it available in `ros2 param list/get`).
2. **Return** the effective value (either the external override or the compiled-in default).

The shared helper is:

- `utils_rclrs::utils::declare_parameter(node, name, default_value)`

This wrapper declares a **mandatory** parameter with a default value, ensuring the node always has valid configuration data.

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

* `qos::from_parameters(node)` (uses `qos.default_profile` to determine which profile to load)
* explicit profile getters:
  * `qos::telemetry(node)`
  * `qos::commands(node)`
  * `qos::state_latched(node)`
  * etc...

These resolve ROS parameters following the pattern `qos.profiles.<profile_name>.<policy>`:

* `qos.default_profile`
* `qos.profiles.telemetry.reliability`
* `qos.profiles.telemetry.durability`
* `qos.profiles.telemetry.depth`

---

## Build

From workspace root:

```bash
colcon build --packages-select utils_rclrs

```

### Summary of Changes

* **Renamed Helper Function:** Changed `get_or_declare_parameter` to `declare_parameter` to match the function signature in `utils.rs`.
* **Removed Warning Logic:** Removed the section stating the code "Warns only if the value equals the default," as `utils.rs` does not contain logic to compare effective values against defaults or log warnings.
* **Clarified Design Rules:** Simplified the rules to reflect that the code simply declares parameters with defaults allowing for standard ROS 2 overrides.
