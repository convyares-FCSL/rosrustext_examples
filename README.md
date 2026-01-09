# ROS 2 Lessons Workspace

This repository contains a structured, multi-language ROS 2 lesson series.
It is designed to teach **core ROS 2 concepts consistently across languages**
while remaining close to professional, production-style usage.

The same lesson numbers exist for **Python**, **C++**, **rclrs**, and **rcllibrust**
so that behaviour, naming, and configuration patterns can be compared directly.

---

## Goals

- **Parity**: Provide a consistent lesson sequence across Python, C++, and Rust.
- **Simplicity vs. Reality**: Keep examples minimal but use production patterns (containers, lifecycle, configuration).
- **Consistency**: Use the same topic, service, and action names to enable cross-language comparison.
- **Explicit Trade-offs**: Make language trade-offs explicit (e.g., Latency vs. Dev Speed) via benchmarks.

---

## Workspace Layout

```text
src/
├─ 1_python/                 # Python lessons (rclpy)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ lesson_02_publisher
│   └─ lesson_03_subscriber
│   └─ utils_py/             # Python-specific utilities
│
├─ 2_cpp/                    # C++ lessons (rclcpp)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ lesson_02_publisher
│   └─ lesson_03_subscriber
│   └─ utils_cpp/            # C++-specific utilities
│
├─ 3_rust/
│   ├─ 1_rclrs/              # Native Rust Nodes (Primary Track)
│   │   ├─ lesson_00_bootstrap
│   │   ├─ lesson_01_node
│   │   ├─ lesson_02_publisher
│   │   └─ lesson_03_subscriber
│   │   └─ utils_rclrs/      # Rust (rclrs)-specific utilities
│   │
│   └─ 2_rcllibrust/         # Rust Client/Bridge (Secondary Track)
│       ├─ lesson_00_bootstrap
│       ├─ lesson_01_node
│       ├─ lesson_02_publisher
│       └─ lesson_03_subscriber
│       └─ utils_rcllibrust/    # Rust (rcllibrust)-specific utilities
│
├─ 4_interfaces/          
│   ├─ lesson_interfaces/    # Shared interfaces
│   │   ├─config/            # yaml config
│   │   ├─msg/               # message interface
│   │   ├─srv/               # service interface
│   │   ├─action/            # action interface
│   │   
│   ├─ rosidl_rust/          # ROSIDL for Rust
│
├─ 5_benchmark/              # Engineering Studies (Latency/Jitter)
│
templates/                   # Starter boilerplate for new packages
│
scripts/                     # Script for benchmarking
```

---

## Architectural Progression

Unlike tutorials that use simple scripts, this workspace teaches scalable patterns from day one:

1. **Lesson 00 (The Container)**:
Wraps the Node in a Class/Struct to manage lifecycle (RAII). Ensures clean shutdown and prevents "zombie nodes."

2. **Lesson 01 (The Event Loop)**:
Introduces shared state (Atomic counters in Rust, Class members in Python/C++) and timer scheduling.

3. **Lesson 02 (Composition)**:
Separates **Logic** (application behaviour) from **Lifecycle** (ROS resource ownership). Introduces dependency injection and **Custom Interfaces**.

4. **Lesson 03 (System Verification)**:
Validates language-agnostic communication, QoS compatibility, and stream robustness (late joiners, restarts, resets). Introduces logic injection for subscriber callbacks and transport-aware validation.

---

## Lesson Roadmap

### Lesson 00 – Workspace bootstrap (Implemented)

* **Goal**: One minimal package per language.
* **Focus**: Build tooling (`colcon` vs `cargo`) and environment verification.

### Lesson 01 – Simple node (Implemented)

* **Goal**: Continuous execution with a 1Hz timer.
* **Focus**: Parameter declaration, timer scheduling, and safe state mutation.

### Lesson 02 – Publisher (Implemented)

* **Goal**: Publish a custom `MsgCount` message on a shared topic.
* **Focus**:
* Defining `.msg` files in `lesson_interfaces`.
* Using shared utility libraries (`utils_py`, `utils_rclrs`, `utils_roslibrust`) for configuration.
* **Composition**: Moving logic out of the main node class.

### Lesson 03 – Subscriber (Implemented)

* **Goal**: Verify cross-language message flow using a shared interface.
* **Focus**:
  * QoS compatibility as a configuration concern
  * Late-joiner handling and publisher reset tolerance
  * Logic injection for subscriber callbacks

### Lesson 04 – Services (Planned)

* **Goal**: `example_interfaces/srv/AddTwoInts` server and client.
* **Focus**: Input validation, error handling, and asynchronous service calls.

### Lesson 05 – Parameters (Planned)

* **Goal**: Declare and validate parameters via `config/topics_config.yaml`.
* **Focus**: Runtime updates via callbacks and "Source of Truth" configuration management.

### Lesson 06 – Lifecycle publisher (Planned)

* **Goal**: Lifecycle-enabled node (Managed Node).
* **Focus**: Publisher created on `configure`, active only after `activate`.

### Lesson 07 – Actions (Planned)

* **Goal**: `CountUntil` action server and client.
* **Focus**: Feedback loops, result handling, and cancellation logic.

### Lesson 08 – Executors and callback groups (Planned)

* **Goal**: Multi-threaded execution.
* **Focus**: Callback group isolation and non-blocking patterns.

### Lesson 09 – Launch files and configuration discovery (Planned)

* **Goal**: Unified launch system.
* **Focus**: Developer-mode configuration (source tree) vs. Installed configuration (package share), demonstrating why hardcoding paths fails in production.

---

## Templates

Starter templates live in the `templates/` directory. They provide the minimal boilerplate required to start a new lesson with the correct build configuration.

* `templates/python`: `setup.py`, `package.xml`, and node skeleton.
* `templates/cpp`: `CMakeLists.txt` and `package.xml` with standard dependencies.
* `templates/rclrs`: `Cargo.toml` configured for `rosidl` generation dependencies.

To use one, copy it into `src/` and rename the folders.

---

## Configuration Pattern

We do **not** hardcode strings (e.g., `"chatter"`) inside Node code.

* **Definitions**: `src/4_interfaces/config/` (YAML files).
* **Access**: Language-specific helpers (`utils_py`, `utils_cpp`, `utils_rclrs`, `utils_roslibrust`).

This pattern allows topic names and QoS profiles to be changed system-wide by editing a single line of config, rather than hunting through source code.

---

## Language Notes

### Python (`rclpy`)

Optimized for orchestration and rapid iteration.

* **See**: `src/1_python/README.md` for build tips (`--symlink-install`).

### C++ (`rclcpp`)

The reference implementation offering the lowest latency and most manual control.

* **See**: `src/2_cpp/README.md` for header/source split details.

### Rust (`rclrs` & `roslibrust`)

Split into two tracks to demonstrate different architectural roles:

rclrs: Native DDS nodes using colcon. (The primary track).

roslibrust: Async/Tokio clients using rosbridge. (The external integration track).

* **See**: `src/3_rust/README.md` for critical build dependency instructions (`../../install/...`).

---

## Benchmarks

The `src/5_benchmark` directory contains optional studies on latency and jitter. These are **not** lessons but engineering references to help you choose the right language for a specific subsystem.

---

## Workspace Automation

For advanced users or CI/CD purposes, this workspace includes helper scripts to build the entire project in a deterministic order.

*   **See**: [scripts/README.md](scripts/README.md) for details on bulk building and cleaning the workspace.
