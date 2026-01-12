# Lesson 05 Theory: Parameters & Central Configuration (Rust / roslibrust)

## Architectural Intent

In earlier lessons, node behaviour was hardcoded (publish rate, topic names, connection URLs).

In Lesson 05, configuration becomes a **first-class system contract**.

The goal is to demonstrate that, even when operating as an external client via `roslibrust`, we can:

1. **Load configuration from a Single Source of Truth** (Shared YAML files).
2. **Share configuration artifacts** between ROS 2 nodes (C++/Python) and external clients (Rust).
3. **Keep business logic isolated from middleware concerns**.
4. **Adapt client behaviour based on the environment it connects to**.

This lesson establishes the **Infrastructure-as-Code** pattern: the system's behavior is defined in configuration files, not in the compiled binary.

---

## Core Concepts Introduced

Lesson 05 introduces three production-grade patterns:

1. **Configuration-First Design**
Topic names, loop periods, and logic thresholds are no longer hardcoded. The client parses its instructions at startup before connecting to the network.
2. **Single Source of Truth**
We use shared YAML files in `lesson_interfaces/config`. These same files are used by ROS 2 launch systems and our Rust client, ensuring the entire system stays in sync.
3. **The Adapter Pattern**
The `utils_roslibrust` library acts as an adapter. It bridges the gap between raw YAML files on disk and strong Rust types (e.g., `topics::telemetry(cfg)`), shielding the application logic from file parsing details.

---

## Code Structure Overview

```text
3_rust/
 ├── utils_roslibrust             (Shared Configuration Adapter)
 └── lesson_05_parameter
     ├── src/bin/*.rs             (Middleware Adapters)
     └── ...

```

Each layer has a single responsibility:

* **Utils:** Knows *how* to parse YAML.
* **Binary:** Knows *which* YAML to load.
* **Logic:** Knows *what* to do with the data.

---

## 1. Pure Logic: Structs & Impls (No ROS Dependencies)

As in all lessons, **business logic is middleware-free**.

```rust
// Pure Rust logic, testable without ROS or Websockets
struct TelemetryStreamValidator {
    expected: i64,
    reset_max_value: i64,
}

impl TelemetryStreamValidator {
    pub fn on_count(&mut self, count: i64) -> StreamDecision {
        // validation logic (detecting resets, gaps, etc.)
    }
}

```

Key properties:

* No `roslibrust` imports.
* No `tokio` async runtimes.
* Deterministic and unit-testable.

This guarantees correctness *before* integration with the asynchronous websocket layer.

---

## 2. The Configuration Protocol: Client-Side Loading

Unlike `rclrs` (which interacts with the ROS 2 Parameter Server), `roslibrust` operates as an external client. It cannot easily "declare" parameters on the server.

Instead, we use a **Client-Side Loading** strategy.

### The Flow

1. **Input:** The user provides a path via CLI: `--params-file ./config/topics.yaml`.
2. **Parsing:** `utils::Config` reads the file and merges it into a generic Value tree.
3. **Resolution:** Helper modules (like `topics.rs`) extract specific keys, providing defaults if missing.

### Why this approach?

This decouples the client from the state of the bridge. The client knows its configuration *guaranteed* before it ever attempts to open a websocket connection. This is robust for edge devices or microservices that may need to operate even if the ROS bridge is temporarily offline.

---

## 3. Middleware Adapter: The Client Node

The `lesson_05_roslibrust` binary owns:

* **Configuration State** (Immutable snapshot of settings).
* **ClientHandle** (Websocket connection).
* **Logic Object** (State machine).

It acts as the **adapter** between the static configuration (YAML) and the dynamic ROS graph.

---

## 4. Runtime Updates: The "Restart" Strategy

### Limitation of File-Based Config

Since we are reading files at startup, the node does not automatically "know" when the file changes on disk.

### The Strategy

To handle runtime updates in this architecture, we adopt a **Stateless Client** model:

1. **Update on Disk:** Edit `topics_config.yaml` to change a topic name or rate.
2. **Restart Client:** The Rust binary is stopped and restarted.
3. **Re-load:** On startup, the client parses the *new* file content immediately.

This mimics the container/pod lifecycle management (e.g., Kubernetes), where configuration changes often trigger a pod restart to ensure a clean state.

---

## 5. Logic Adaptation (Publisher)

**Use case:** Changing the publishing frequency.

1. **Load:** `publisher.period_ms` is read from YAML.
2. **Build:** A `tokio::time::interval` is constructed with this duration.
3. **Run:** The loop runs at the configured rate.

If the key is missing in the file, the client **must** fall back to a safe default (e.g., 1000ms) to ensure robustness.

---

## 6. Logic Adaptation (Subscriber)

**Use case:** Changing logic thresholds (`reset_max_value`).

1. **Load:** `subscriber.reset_max_value` is read from YAML.
2. **Initialize:** The `StreamValidator` struct is initialized with this value.
3. **Process:** Incoming messages are validated against this specific threshold.

This ensures that the validation logic matches the expectations of the rest of the system, even though the code was compiled once.

---

## Memory & Async Model (Rust Perspective)

Rust enforces thread safety in this async environment:

* **`Arc`**: Shared ownership of the configuration and logic across the network task and the main thread.
* **`Atomic` / `Mutex**`: Safe mutation of state (e.g., sequence counters) across async boundaries.
* **`Config` (Immutable)**: The configuration object is read-only after startup, making it safe to share across threads without locks.

---

## Why This Lesson Matters

Lesson 05 demonstrates **System Integration**:

* **Decoupling**: The client binary is generic; its behavior is defined by the injected configuration file.
* **Consistency**: It proves that Rust clients can share the same "Source of Truth" (YAML) as C++ and Python nodes, preventing configuration drift.
* **Resilience**: It handles the "Happy Path" (config exists) and the "Fallback Path" (defaults) gracefully.

This architecture scales to **Fleet Management**, where a central configuration repository deploys `yaml` files to remote robots, ensuring all clients behave identically.