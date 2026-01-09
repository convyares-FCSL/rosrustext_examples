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
├─ 1_python/              # Python lessons (rclpy)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   └─ lesson_02_publisher
│
├─ 2_cpp/                 # C++ lessons (rclcpp)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   └─ lesson_02_publisher
│
├─ 3_rust/
│   ├─ 1_rclrs/           # Native Rust Nodes (Primary Track)
│   └─ 2_rcllibrust/      # Rust Client/Bridge (Secondary Track)
│
├─ 4_interfaces/          # Shared Definitions
│   ├─ msg/               # e.g., MsgCount.msg
│   └─ config/            # Shared .yaml configuration
│
├─ 5_benchmark/           # Engineering Studies (Latency/Jitter)
│   └─ README.md
│
templates/                # Starter boilerplate for new packages

```

---

## Architectural Progression

Unlike tutorials that use simple scripts, this workspace teaches scalable patterns from day one:

1. **Lesson 00 (The Container)**:
Wraps the Node in a Class/Struct to manage lifecycle (RAII). Ensures clean shutdown and prevents "zombie nodes."
2. **Lesson 01 (The Event Loop)**:
Introduces shared state (Atomic counters in Rust, Class members in Python/C++) and timer scheduling.
3. **Lesson 02 (Composition)**:
Separates **Logic** (Business Components) from **Lifecycle** (The Node). Introduces dependency injection and **Custom Interfaces**.

---

## Lesson Roadmap

### Lesson 00 – Workspace bootstrap

* **Goal**: One minimal package per language.
* **Focus**: Build tooling (`colcon` vs `cargo`) and environment verification.

### Lesson 01 – Simple node

* **Goal**: Continuous execution with a 1Hz timer.
* **Focus**: Parameter declaration, timer scheduling, and safe state mutation.

### Lesson 02 – Publisher

* **Goal**: Publish a custom `MsgCount` message on a shared topic.
* **Focus**:
* Defining `.msg` files in `lesson_interfaces`.
* Using shared utility libraries (`utils_py`, `utils_rust`) for configuration.
* **Composition**: Moving logic out of the main node class.



### Lesson 03 – Subscriber (Planned)

* **Goal**: Subscribe to `chatter` and log data.
* **Focus**: QoS matching (Reliable vs Best Effort) and callback concurrency.

### Lesson 04 – Services (Planned)

* **Goal**: `example_interfaces/srv/AddTwoInts` server and client.
* **Focus**: Input validation and asynchronous service calls.

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
* **Access**: Language-specific helpers (`utils_py`, `utils_cpp`, `utils_rust`).

This pattern allows topic names and QoS profiles to be changed system-wide by editing a single line of config, rather than hunting through source code.

---

## Language Notes

### Python (`rclpy`)

Optimized for orchestration and rapid iteration.

* **See**: [src/1_python/README.md](https://www.google.com/search?q=src/1_python/README.md) for build tips (`--symlink-install`).

### C++ (`rclcpp`)

The reference implementation offering the lowest latency and most manual control.

* **See**: [src/2_cpp/README.md](https://www.google.com/search?q=src/2_cpp/README.md) for header/source split details.

### Rust (`rclrs`)

Native Rust bindings focusing on memory safety and correctness.

* **See**: [src/3_rust/README.md](https://www.google.com/search?q=src/3_rust/README.md) for critical build dependency instructions (`../../install/...`).

---

## Benchmarks

The `src/5_benchmark` directory contains optional studies on latency and jitter. These are **not** lessons but engineering references to help you choose the right language for a specific subsystem.
