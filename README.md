# ROS 2 Lessons Workspace

This repository contains a structured, multi-language ROS 2 lesson series.
It is designed to teach **core ROS 2 concepts consistently across languages**
while remaining close to professional, production-style usage.

The same lesson numbers exist for **Python**, **C++**, **rclrs**, and **rcllibrust**
so that behaviour, naming, and configuration patterns can be compared directly.

---

## Goals

- Provide a consistent lesson sequence across Python, C++, rclrs, and rcllibrust.
- Keep examples minimal, readable, and stylistically aligned across languages.
- Use the same topic, service, and action names to enable cross-language comparison.
- Introduce professional configuration and bringup patterns without hiding fundamentals.
- Make language trade-offs explicit where they matter.

---

## Workspace Layout

```text
src/
├─ 1_python/
│  ├─ lesson_00_bootstrap
│  ├─ lesson_01_node
│  ├─ lesson_0x_xxxx
│  └─ utils_py
│
├─ 2_cpp/
│  ├─ lesson_00_bootstrap
│  ├─ lesson_01_node
│  ├─ lesson_0x_xxxx
│  └─ utils_cpp
│
├─ 3_rust/
│  ├─ 1_rclrs/
│  │  ├─ lesson_00_bootstrap
│  │  ├─ lesson_01_node
│  │  ├─ lesson_0x_xxxx
│  │
│  ├─ 2_rcllibrust/
│  │  ├─ lesson_00_bootstrap
│  │  ├─ lesson_01_node
│  │  ├─ lesson_0x_xxxx
│  │
│  └─ utils_rust
│
├─ 4_interfaces/
│  ├─ config/
│  │  ├─ topics_config.yaml
│  │  ├─ qos_config.yaml
│  │  └─ services_config.yaml
│  ├─ msg/
│  ├─ srv/
│  └─ action/
│
├─ 5_benchmark/
│  └─ README.md
│
templates/
````

Notes:

* Numeric prefixes (`1_`, `2_`, `3_`, `4_`, `5_`) are intentional and reflect increasing
  conceptual and architectural complexity.
* `rcllibrust` examples are **Cargo-only** and excluded from `colcon` via `COLCON_IGNORE`.
* Shared message, service, action, and configuration files live in `4_interfaces`.
* Benchmarks live in `5_benchmark` and are **not** part of the core lesson flow.

---

## Lesson Roadmap

### Lesson 00 – Workspace bootstrap

* One minimal package per language.
* Build and run verification only.
* Confirms toolchains and environment are working.

### Lesson 01 – Simple node

* Node creation.
* Timer + logging only.
* Clean startup and shutdown.

### Lesson 02 – Publisher

* Publish `std_msgs/String` on `chatter`.
* Parameter for publish rate.
* Uses shared configuration from:

  * `topics_config.yaml`
  * `qos_config.yaml`

### Lesson 03 – Subscriber

* Subscribe to `chatter`.
* QoS selection and logging.
* Uses the same shared configuration files as Lesson 02.

### Lesson 04 – Services

* `example_interfaces/srv/AddTwoInts` server and client.
* Input validation and error handling.

### Lesson 05 – Parameters

* Declare and validate parameters.
* Runtime updates via callbacks.

### Lesson 06 – Lifecycle publisher

* Lifecycle-enabled node.
* Publisher created on configure.
* Publish only when active; disabled on deactivate.

### Lesson 07 – Actions

* `CountUntil` action server and client.
* Feedback, result, and cancel handling.

### Lesson 08 – Executors and callback groups (optional)

* Multi-threaded executors.
* Callback group isolation.
* Non-blocking patterns.

### Lesson 09 – Launch files and configuration discovery

* Basic launch files.
* Developer-mode configuration (source tree).
* Installed configuration discovery using package share paths.
* Demonstrates why hardcoding `src/...` paths does not scale.

### Lesson 10 – Capstone (placeholder)

* Links to a **separate capstone project**.
* Demonstrates:

  * Multi-language systems (Python, C++, Rust together)
  * Bringup packages and launch trees
  * Lifecycle nodes and lifecycle managers
  * Shared interfaces and shared configuration
  * Professional error handling and naming conventions

---

## Configuration Pattern

Shared configuration lives in:

```text
src/4_interfaces/config/
```

* `topics_config.yaml` – topic names
* `qos_config.yaml` – QoS profiles and defaults
* `services_config.yaml` – service names

Language-specific utility libraries (`utils_py`, `utils_cpp`, `utils_rust`)
handle parameter declaration and retrieval so lesson code stays focused on behaviour.

This pattern prepares the workspace for later lessons involving launch files
and multi-language bringup.

---

## Benchmark Track (Optional)

`src/5_benchmark` contains **optional benchmarking material** comparing
communication latency, jitter, and throughput across languages.

This track is:

* Not required to complete the lessons
* Environment-dependent
* Intended to highlight **trade-offs**, not declare winners

---

## Templates

Starter templates live in `templates/` and are intended to be copied into `src/`:

* `templates/python`
* `templates/cpp`
* `templates/rclrs`
* `templates/rcllibrust`

Templates provide:

* Minimal boilerplate
* Consistent naming
* Correct build configuration per language

---

## Lesson readme

Each lesson directory contains a `Lesson.md` describing:

* The goal
* Build steps
* Run steps

## Lanaguage Notes

### Python Notes

* Python lessons use `rclpy` and follow standard ROS 2 node patterns.
* Logging uses ROS logging (`get_logger()`), publishing to `/rosout`.
* Python is well suited for:

  * orchestration
  * supervision
  * configuration management
  * rapid iteration
* Python is **not** ideal for:

  * high-frequency, low-jitter loops
  * hard real-time behaviour
* Garbage collection and interpreter scheduling introduce observable jitter.
* Clean shutdown requires explicit handling of timers, callbacks, and executors.

Python lessons prioritise **clarity and correctness** over performance.

---

### C++ Notes

* C++ lessons use `rclcpp` and the default ROS 2 execution model.
* Logging uses ROS logging and integrates fully with `/rosout`.
* C++ offers:

  * lowest latency floor
  * best control over executors, memory, and QoS
  * mature ROS ecosystem support
* C++ also brings:

  * higher complexity
  * longer compile times
  * more responsibility for correctness
* Lifecycle nodes and executor tuning are most explicit in C++.

C++ lessons demonstrate **canonical ROS 2 patterns** and are treated as the
reference implementation when behaviour must be unambiguous.

---

### Rust Notes (existing, unchanged but contextualised)

* rclrs provides ROS-native Rust bindings.
* rcllibrust is a client/bridge library and does **not** behave like a ROS node.
* Rust emphasises:

  * memory safety
  * explicit ownership
  * predictable long-running behaviour
* Rust logging differs:

  * `rclrs` → ROS logging
  * `rcllibrust` → standard Rust logging (`log` + `env_logger`)
* The Rust ecosystem for ROS is newer and intentionally conservative.

Rust lessons focus on **correctness, structure, and long-term reliability**.