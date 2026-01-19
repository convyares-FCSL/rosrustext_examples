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

<details open>
<summary>src/</summary>

```text
src/
├─ 1_python/                 # Python lessons (rclpy)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ 
│   └─ utils_py/
│
├─ 2_cpp/                    # C++ lessons (rclcpp)
│   ├─ lesson_00_bootstrap
│   ├─ lesson_01_node
│   ├─ 
│   └─ utils_cpp/
│
├─ 3_rust/
│   ├─ 1_rclrs/              # Native Rust nodes (DDS, primary track)
│   │   ├─ lesson_00_bootstrap
│   │   ├─ lesson_01_node
│   │   ├─ 
│   │   └─ utils_rclrs/
│   │
│   └─ 2_rcllibrust/         # Rust via rosbridge (async / external clients)
│       ├─ lesson_00_bootstrap
│       ├─ lesson_01_node
│       ├─ 
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

</details>

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

<details open>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 00 – Bootstrap :
    <br />
    <br />
  </summary>

**Goal**
Create the smallest possible ROS-capable package per language.

**Focus**
Build systems (`colcon` vs `cargo`), environment setup, logging, clean shutdown.

**Architecture**
Node wrapped in a class/struct to enforce ownership, RAII, and predictable shutdown. No “naked scripts”.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 01 – Event Loop :
    <br />
    <br />
  </summary>

**Goal**
Introduce continuous execution.

**Focus**
Timers, parameters, safe state mutation.

**Architecture**
Explicit event loop with owned state (class members in Python/C++, struct fields in Rust).

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 02 – Publisher :
    <br />
    <br />
  </summary>

**Goal**
Publish a shared custom message across languages.

**Focus**
Custom interfaces (`.msg`), configuration reuse, composition.

**Architecture**
Separation between:

* **Logic** (what data changes)
* **Lifecycle / Resources** (publisher ownership)

Publishers become injected dependencies rather than global objects.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 03 – Subscriber & System Verification :
    <br />
    <br />
  </summary>

**Goal**
Verify cross-language message flow and transport correctness.

**Focus**
QoS compatibility, late joiners, publisher restarts, reset tolerance.

**Architecture**
Subscriber implemented as a **self-contained component**:

* Logic injected into callbacks (native nodes)
* Async stream processing inside a background task (roslibrust)

This lesson treats the subscriber as a **verification tool**, not just a data consumer.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 04 – Services & Unit Testing :
    <br />
    <br />
  </summary>

**Goal**
Implement a request–response service with verifiable logic.

**Focus**
Services, async request handling, testability.

**Architecture**
Strict separation between:

* **Business Logic**: Pure, deterministic, no ROS dependencies
* **Middleware Adapter**: Service server/client that converts ROS types

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 05 – Parameters & Central Configuration :
    <br />
    <br />
  </summary>

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

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 06 – Lifecycle Management (Managed Nodes) :
    <br />
    <br />
  </summary>

**Goal**
Establish the "Managed Node" pattern where nodes wait for orchestration rather than starting immediately.

**Focus**
The ROS 2 Lifecycle State Machine (Unconfigured  Inactive  Active).

* **Deterministic Startup**: Orchestrating a fleet using a Manager Node.
* **Gated Execution**: Publishers and timers that are silent unless `Active`.
* **Verification**: Automated integration testing (`launch_testing`) and standard compliance check (Nav2).

**Architecture**

* **State Ownership**: Native inheritance (`rclpy`/`rclcpp`) or `rosrustext` adapters (Rust).
* **Gated Resources**: LifecyclePublishers and LifecycleTimers used to enforce state contracts.
* **Orchestration Layer**: A dedicated package containing a Custom Manager Node and Integration Test suite.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 07 – Actions (Long-Running Work) :
    <br />
    <br />
  </summary>

**Goal**
Introduce **long-running, cancellable tasks** and expose the difference between functional correctness and system responsiveness.

**Focus**
ROS 2 Actions:

* goals, feedback, results
* cancellation semantics
* client–server interaction under load

**Architecture**

* Extend the **Lesson 06 Lifecycle Publisher** with an **Action Server**.
* Add a **standalone Action Client** used to issue goals and observe behaviour.
* Reuse the **existing Subscriber** unchanged to verify system impact.

The action performs a deliberately **long-running Fibonacci calculation** on the node’s primary execution path.

This is intentional.

The lesson demonstrates that a node can:

* be lifecycle-correct
* respond to action goals
* **and still become operationally unresponsive**

This failure is resolved in Lesson 08.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 08 – Executors & Callback Groups :
    <br />
    <br />
  </summary>

**Goal**
Restore node responsiveness under long-running work without changing functional behaviour.

**Focus**
ROS 2 execution control:

* `SingleThreadedExecutor` vs `MultiThreadedExecutor`
* Callback groups and reentrancy
* Scheduling of actions, timers, and services

**Architecture**
Reuse the Lesson 07 managed node and action implementation.

Change only execution strategy:

* Multi-threaded executor
* Explicit callback group assignment

Concurrency is introduced deliberately to preserve availability, not to optimise performance.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 09 – Composition & Containers :
    <br />
    <br />
  </summary>

**Goal**
Validate that previously built nodes behave correctly when deployed together in a shared process.

**Focus**
ROS 2 deployment mechanisms:

* Composable nodes
* Component containers
* Shared executors and shared fate
* Runtime load / unload via ROS tooling

**Architecture**
Reuse existing nodes unchanged (publisher, subscriber, service, action).

Change only deployment topology:

* Multiple nodes loaded into a single component container
* Executor ownership moved to the container
* Callback group choices tested under shared execution

This lesson exposes hidden assumptions about execution, shutdown order, and isolation that only appear under composition.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 600;">
    Lesson 10 – Launch & Configuration Discovery :
    <br />
    <br />
  </summary>

**Goal**
Production-grade startup.

**Focus**
Launch files, installed vs source-tree configuration.

**Architecture**
No hardcoded paths; runtime discovery only.

</details>

---

## Testing

This workspace includes a robust testing suite for Python, C++, and Rust (`rclrs`) tracks. The tests verify node functionality, communication (pub/sub/service), parameter handling, and lifecycle orchestration.

### Track Commands

You can run tests for a specific track or all tracks at once:

| Track | Command |
|-------|---------|
| **All Tracks** | `./scripts/04_tests/run_all.sh` |
| Python | `./scripts/04_tests/run_python.sh` |
| C++ | `./scripts/04_tests/run_cpp.sh` |
| Rust (rclrs) | `./scripts/04_tests/run_rclrs.sh` |

### Logs & Interpretation

Test logs are stored in `log/tests/`, grouped by track (e.g., `log/tests/python/`). 
A **PASS** indicates the node output matches the expected pattern (regex) or the logic verification succeeded.

### Test Hygiene & "Ghost Nodes"

The suite implements a strict hygiene mechanism to ensure clean runs:
- **Process Groups**: Every node is launched in a dedicated process group (`setsid -w`).
- **Traps**: Scripts use bash traps to ensure all processes are terminated (SIGTERM followed by SIGKILL) on exit, failure, or interruption (Ctrl+C).
- **Graph & Residue Check**: Scripts verify the ROS graph is sterile between runs.
- **Daemon Flushing**: If a "ghost node" (a node visible in the graph whose process has actually terminated) is detected, the script automatically restarts the `ros2 daemon` to flush the cache.

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