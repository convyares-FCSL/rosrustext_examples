# ROS 2 Lessons Workspace

This repository contains a structured, multi-language ROS 2 lesson series.

It is designed to teach **core ROS 2 concepts consistently across languages**, while staying close to **professional, production-style usage** rather than toy examples.

The same lesson numbers exist for **Python**, **C++**, **rclrs**, and **roslibrust**, allowing behaviour, naming, configuration, and architectural trade-offs to be compared directly.

---

## Goals

* **Parity**: A consistent lesson sequence across Python, C++, and Rust.
* **Production Patterns**: Minimal examples that still use real-world structures (containers, composition, async execution).
* **Consistency**: Shared topic, service, and action names across languages.
* **Explicit Trade-offs**: Make language and transport trade-offs visible (e.g. DDS vs bridge, latency vs ergonomics).

---

## Workspace Layout

```text
src/
├─ 1_python/                 # Python lessons (rclpy)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ lesson_02_publisher
│   ├─ lesson_03_subscriber
│   ├─ lesson_04_service
│   └─ utils_py/
│
├─ 2_cpp/                    # C++ lessons (rclcpp)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ lesson_02_publisher
│   ├─ lesson_03_subscriber
│   ├─ lesson_04_service
│   └─ utils_cpp/
│
├─ 3_rust/
│   ├─ 1_rclrs/              # Native Rust nodes (DDS, primary track)
│   │   ├─ lesson_00_bootstrap
│   │   ├─ lesson_01_node
│   │   ├─ lesson_02_publisher
│   │   ├─ lesson_03_subscriber
│   │   ├─ lesson_04_service
│   │   └─ utils_rclrs/
│   │
│   └─ 2_rcllibrust/         # Rust via rosbridge (async / external clients)
│       ├─ lesson_00_bootstrap
│       ├─ lesson_01_node
│       ├─ lesson_02_publisher
│       ├─ lesson_03_subscriber
│       ├─ lesson_04_service
│       └─ utils_rcllibrust/
│
├─ 4_interfaces/
│   ├─ lesson_interfaces/    # Shared ROS interfaces
│   │   ├─ config/           # YAML configuration
│   │   ├─ msg/              # Messages
│   │   ├─ srv/              # Services
│   │   └─ action/           # Actions
│   └─ rosidl_rust/          # ROSIDL Rust bindings
│
├─ 5_benchmark/              # Latency / jitter studies
│
templates/                   # Starter boilerplate
scripts/                     # Workspace automation
```

---

## Lesson Structure

Each lesson is implemented across all languages using the same numbering and naming.

Every lesson is described using the same three lenses:

* **Goal** – what problem the lesson solves
* **Focus** – the ROS 2 concepts being introduced
* **Architecture** – the structural pattern being taught

This makes it possible to compare implementations directly while keeping the learning intent clear.

---

## Lessons

### Lesson 00 – Bootstrap

**Goal**
Create the smallest possible ROS-capable package per language.

**Focus**
Build systems (`colcon` vs `cargo`), environment setup, logging, clean shutdown.

**Architecture**
Node wrapped in a class/struct to enforce ownership, RAII, and predictable shutdown. No “naked scripts”.

---

### Lesson 01 – Event Loop

**Goal**
Introduce continuous execution.

**Focus**
Timers, parameters, safe state mutation.

**Architecture**
Explicit event loop with owned state (class members in Python/C++, struct fields in Rust).

---

### Lesson 02 – Publisher

**Goal**
Publish a shared custom message across languages.

**Focus**
Custom interfaces (`.msg`), configuration reuse, composition.

**Architecture**
Separation between:

* **Logic** (what data changes)
* **Lifecycle / Resources** (publisher ownership)

Publishers become injected dependencies rather than global objects.

---

### Lesson 03 – Subscriber & System Verification

**Goal**
Verify cross-language message flow and transport correctness.

**Focus**
QoS compatibility, late joiners, publisher restarts, reset tolerance.

**Architecture**
Subscriber implemented as a **self-contained component**:

* Logic injected into callbacks (native nodes)
* Async stream processing inside a background task (roslibrust)

This lesson treats the subscriber as a **verification tool**, not just a data consumer.

---

### Lesson 04 – Services & Unit Testing

**Goal**
Implement a request–response service with verifiable logic.

**Focus**
Services, async request handling, testability.

**Architecture**
Strict separation between:

* **Business Logic**: Pure, deterministic, no ROS dependencies
* **Middleware Adapter**: Service server/client that converts ROS types

---

### Lesson 05 – Parameters & Central Configuration

**Goal**
Make configuration explicit, shared, and safely changeable at runtime.

**Focus**
Use ROS parameters as a live configuration interface rather than startup-only defaults.

* validation and runtime updates via parameter callbacks
* updated, production-style publishers and subscribers

**Architecture**
Centralise system configuration while keeping node code explicit and deterministic.

* Central YAML defines topics, services, and QoS
* Language utilities adapt YAML → parameters → typed access
* Node behaviour updates safely when configuration changes


---

### Lesson 06 – Lifecycle Publisher (Planned)

**Goal**
Introduce managed nodes.

**Focus**
Lifecycle transitions, controlled activation.

**Architecture**
Resources created on `configure`, activated explicitly, destroyed cleanly.

---

### Lesson 07 – Actions (Planned)

**Goal**
Long-running tasks with feedback and cancellation.

**Focus**
Action servers and clients.

**Architecture**
Explicit state machines with feedback loops.

---

### Lesson 08 – Executors & Callback Groups (Planned)

**Goal**
Multi-threaded execution.

**Focus**
Callback group isolation, non-blocking patterns.

**Architecture**
Controlled concurrency instead of implicit threading.

---

### Lesson 09 – Launch & Configuration Discovery (Planned)

**Goal**
Production-grade startup.

**Focus**
Launch files, installed vs source-tree configuration.

**Architecture**
No hardcoded paths; runtime discovery only.

---

## Configuration Pattern

Strings such as topic and service names are **not hardcoded** in node code.

* **Definitions**: `src/4_interfaces/config/` (YAML)
* **Access**:

  * Python: `utils_py`
  * C++: `utils_cpp`
  * Rust (native): `utils_rclrs`
  * Rust (bridge): `utils_rcllibrust`

This allows system-wide changes by editing configuration rather than source.

---

## Language Notes

### Python (`rclpy`)

Optimised for orchestration and rapid iteration.

* **See**: `src/1_python/README.md` for build tips (`--symlink-install`).

### C++ (`rclcpp`)

Reference implementation with lowest latency and maximum control.

* **See**: `src/2_cpp/README.md` for header/source split details.

### Rust

Two complementary tracks:

* **rclrs**: Native DDS nodes built with `colcon`
* **roslibrust**: Async external clients using `rosbridge`

Together they demonstrate both embedded and integration-focused roles.

* **See**: `src/3_rust/README.md` for critical build dependency instructions (`../../install/...`).

---

## Benchmarks

`src/5_benchmark` contains optional latency and jitter studies.
These are **engineering references**, not lessons.

---

## Workspace Automation

Helper scripts are provided for deterministic builds and CI use.

See: `scripts/README.md`