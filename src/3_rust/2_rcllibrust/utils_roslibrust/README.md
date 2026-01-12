# utils_roslibrust (Shared Utilities for roslibrust)

`utils_roslibrust` is the shared **configuration adapter** crate for Rust nodes built on **roslibrust**.

It provides a **single, consistent entry point** for reading system configuration that is shared across nodes and across lessons.

This crate exists to preserve the same **architectural intent** as `utils_cpp` and `utils_rclrs`, even though the underlying middleware differs.

---

## Purpose

`utils_roslibrust` centralises:

* **Topic names** (`utils_roslibrust::topics`)
* **QoS profiles** (`utils_roslibrust::qos`)
* **Service names** (`utils_roslibrust::services`)
* **Configuration loading** (`utils_roslibrust::utils`)

So that:

* Lesson code contains **no hardcoded strings**
* Configuration is **external, inspectable, and repeatable**
* Publisher and subscriber always agree on the same system contract

This crate lives **outside lesson packages** and is intended to be reused unchanged across lessons.

---

## Design Constraints (Important)

roslibrust is intentionally **more detached** from ROS middleware than `rclrs` or `rclcpp`.

As a result:

* There is **no parameter server**
* There are **no runtime parameter callbacks**
* There is **no parameter inspection via ROS tools**

This crate **does not attempt to fake those features**.

Instead, Lesson 05 uses **Startup Configuration Only** for roslibrust.

This is **explicit, deliberate, and documented**.

---

## How Configuration Works (roslibrust)

### Source of Truth

Configuration is supplied via **YAML files** that follow the ROS convention:

```yaml
/**:
  ros__parameters:
    topics:
      telemetry: "/tutorial/telemetry"
```

### Loading Model

1. Configuration is loaded **once at startup**
2. YAML is parsed using `serde_yaml`
3. Only the `/**/ros__parameters` subtree is considered
4. The resulting `Config` struct is passed by reference to:

   * `topics`
   * `qos`
   * `services`

There is **no global state** and **no reload mechanism**.

---

## utils_roslibrust::utils

The `utils` module owns configuration loading.

```rust
let config = utils::load_config("config.yaml")?;
```

Responsibilities:

* Parse YAML via `serde_yaml`
* Extract `ros__parameters`
* Deserialize into a strongly-typed `Config`
* Fail fast on malformed configuration

It does **not**:

* Create publishers or subscribers
* Own runtime resources
* Manage lifecycle
* Emulate ROS parameters

---

## Topics

Module: `utils_roslibrust::topics`

```rust
let topic = topics::telemetry(&config);
```

* Returns canonical topic names
* No string literals in node code
* Same YAML schema as C++ and rclrs

---

## Services

Module: `utils_roslibrust::services`

```rust
let srv = services::compute_stats(&config);
```

* Centralised service naming
* Matches the shared YAML schema

---

## QoS

Module: `utils_roslibrust::qos`

```rust
let qos = qos::telemetry(&config);
```

* QoS profiles are loaded from YAML
* Translated into roslibrust-native QoS representations
* Defaults exist only to keep examples runnable

---

## Relationship to Lesson 05

### What Matches Other Languages

* Central YAML is the source of truth
* No hardcoded endpoints
* Same topic/QoS/service schema
* Shared utility crate outside lessons
* Identical mental model

### What Is Different (Explicitly)

* No runtime mutation
* No inspection via `ros2 param`
* No parameter callbacks

This is **not a regression** — it reflects the current capabilities of roslibrust.

Lesson 05 remains about **Configuration as a System**, not about forcing parity where it does not exist.

---

## Build

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/utils_roslibrust
cargo build
```

---

## Why This Crate Exists

`utils_roslibrust` ensures that:

* Lessons remain architecture-driven, not framework-driven
* roslibrust users learn the *same engineering patterns*
* Parity can be restored later (e.g. via rosrustext proxies)
* Configuration does not leak into application logic

It is intentionally **boring**, **thin**, and **predictable**.

That is the point.