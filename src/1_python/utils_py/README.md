# utils_py (Python Shared Utilities)

`utils_py` is the shared Python utility package for this workspace.

It centralises:
- **Topic names** (`utils_py/topics.py`)
- **QoS profiles** (`utils_py/qos.py`)
- **Service names** (`utils_py/services.py`)
- A small parameter helper (`utils_py/utils.py`) used by all of the above

This keeps lesson nodes free of hardcoded strings and makes config consistent across languages.

---

## Design Rules

- Nodes should **not** hardcode topic/service strings.
- Nodes should obtain config through `utils_py` helpers.
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

- `utils_py.utils._get_or_declare(node, name, default, warn_label=...)`

This is used by `topics.py`, `services.py`, and `qos.py`.

---

## YAML Source of Truth (ROS Native)

The workspace stores ROS parameter YAML in:

- `src/4_interfaces/lesson_interfaces/config/topics_config.yaml`
- `src/4_interfaces/lesson_interfaces/config/qos_config.yaml`
- `src/4_interfaces/lesson_interfaces/config/services_config.yaml`

Nodes consume them with `--ros-args --params-file ...`.

Example:

```bash
ros2 run lesson_05_parameters_py lesson_05_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/services_config.yaml
````

---

## Topics

Module: `utils_py/topics.py`

Exposes canonical getters:

* `topics.chatter(node)` (Lessons 00–04)
* `topics.telemetry(node)` (Lesson 05+)

They resolve ROS parameters like:

* `topics.chatter`
* `topics.telemetry`

---

## Services

Module: `utils_py/services.py`

Exposes canonical getters:

* `services.compute_stats(node)`

Resolves ROS parameters like:

* `services.compute_stats`

---

## QoS

Module: `utils_py/qos.py`

Provides:

* `qos.from_parameters(node)` (uses `qos.default_profile`)
* explicit profile getters:

  * `qos.telemetry(node)`
  * `qos.commands(node)`
  * `qos.state_latched(node)`
  * `qos.events(node)`
  * `qos.reliable_data(node)`
  * `qos.static_data_latched(node)`

These resolve ROS parameters like:

* `qos.default_profile`
* `qos.profiles.telemetry.reliability`
* `qos.profiles.telemetry.durability`
* `qos.profiles.telemetry.depth`
  (and equivalents for each profile)

---

## Build

From workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select utils_py --symlink-install
source install/setup.bash
```

If you are overriding an underlay-installed `utils_py`:

```bash
colcon build --packages-select utils_py --symlink-install --allow-overriding utils_py
```

---

## Intended Usage Pattern

In lesson nodes:

```python
from utils_py import topics, qos

topic = topics.telemetry(self)
qos_profile = qos.telemetry(self)
publisher = self.create_publisher(MsgCount, topic, qos_profile)
```

This is the standard pattern for all languages in this workspace: “strings and policy live in utils, node code stays lean.”

````

## One next step
Drop that file into `src/1_python/utils_py/README.md`, then run:

```bash
colcon build --packages-select utils_py --symlink-install --allow-overriding utils_py
````

That ensures the README ships with the package share directory if you later add it to `data_files` (optional in this repo).
