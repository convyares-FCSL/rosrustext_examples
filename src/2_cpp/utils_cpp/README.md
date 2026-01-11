# utils_cpp (C++ Shared Utilities)

`utils_cpp` is the shared C++ utility package for this workspace.

It centralises:
- **Topic names** (`utils_cpp/topics.hpp`)
- **QoS profiles** (`utils_cpp/qos.hpp`)
- **Service names** (`utils_cpp/services.hpp`)
- A small parameter helper (`utils_cpp/utils.hpp`) used by all of the above

This keeps lesson nodes free of hardcoded strings and makes config consistent across languages.

---

## Design Rules

- Nodes should **not** hardcode topic/service strings.
- Nodes should obtain config through `utils_cpp` helpers.
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

- `utils_cpp::get_or_declare_parameter<T>(node, name, default, warn_label)`

---

## Topics

Header: `#include "utils_cpp/topics.hpp"`

Exposes canonical getters:

* `topics::chatter(node)` (Lessons 00–04)
* `topics::telemetry(node)` (Lesson 05+)

They resolve ROS parameters like:

* `topics.chatter`
* `topics.telemetry`

---

## Services

Header: `#include "utils_cpp/services.hpp"`

Exposes canonical getters:

* `services::compute_stats(node)`

Resolves ROS parameters like:

* `services.compute_stats`

---

## QoS

Header: `#include "utils_cpp/qos.hpp"`

Provides:

* `qos::from_parameters(node)` (uses `qos.default_profile`)
* explicit profile getters:
  * `qos::telemetry(node)`
  * `qos::commands(node)`
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
colcon build --packages-select utils_cpp