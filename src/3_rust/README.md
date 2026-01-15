# Rust Lessons

This directory contains ROS 2 lessons implemented in Rust, split into two distinct tracks based on their architectural role.

The goal is **full feature parity** with the C++ and Python lessons, while exposing the realities of Rust’s stricter build and dependency model.

---

## 1. `1_rclrs` — Native ROS 2 Nodes (DDS Peers)

These lessons use [`rclrs`](https://github.com/ros2-rust/ros2_rust), the native Rust client library for ROS 2.

* **Role**: First-class peer in the ROS 2 graph
* **Transport**: Native DDS (RMW)
* **Build system**: `colcon` via `ament_cargo`
* **Run**: `ros2 run <package> <executable>`

### Developer workflow

Rust nodes in this track build exactly like C++ nodes:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_00_bootstrap_rclrs
```

---

## Important: ROS Interfaces & Rust

Rust differs from C++ and Python in one crucial way:

> **ROS interface packages are workspace-local build artifacts, not registry dependencies.**

Rust tooling will not “discover” ROS messages, services, or core interfaces automatically.
You must explicitly depend on the **generated Rust crates** produced by `colcon`.

---

### Custom Interfaces (e.g. `lesson_interfaces`)

1. The interface package **must** depend on `rosidl_generator_rs` in its `package.xml`
2. Your Rust node must depend on the **generated crate**, not the source tree

```toml
[dependencies]
lesson_interfaces = {
  path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust"
}
```

---

### Core ROS Interfaces (`rcl_interfaces`, etc.)

Core ROS packages such as:

* `rcl_interfaces`
* `builtin_interfaces`
* `lifecycle_msgs`
* `service_msgs`
* `action_msgs`
* `unique_identifier_msgs`

are provided by `/opt/ros/<distro>` **for C++ and Python only**.

They **do not ship Rust bindings**.

If a Rust node depends on any of these, you must:

1. Overlay the package into the workspace
2. Enable Rust generation (`rosidl_generator_rs`)
3. Build it with `colcon`
4. Depend on the generated crate via a **path dependency**

Example:

```toml
rcl_interfaces = {
  path = "../../../../install/rcl_interfaces/share/rcl_interfaces/rust"
}
```

---

### Verification (required)

After building interfaces, these files **must exist**:

```bash
install/lesson_interfaces/share/lesson_interfaces/rust/Cargo.toml
install/rcl_interfaces/share/rcl_interfaces/rust/Cargo.toml
```

If they do not, Rust nodes depending on those interfaces will fail to build.

---

<details>
<summary><strong>Deep Dive: Why `rcl_interfaces = "*"` Looks Reasonable — Until It Breaks</strong></summary>

#### The symptom

Generated Rust bindings often emit:

```toml
rcl_interfaces = "*"
```

In Cargo, this means:

> “Resolve from crates.io”

That is **not** how ROS interfaces work.

#### Why this fails

* `rcl_interfaces` is a ROS package, not a published Rust crate
* crates.io cannot guarantee ROS-distro compatibility
* Offline builds and CI break
* Worse: mismatched types can compile but fail at runtime

#### The correct model

ROS interface crates are **generated locally** by `colcon`.

They must be consumed via **path dependencies**, never via the registry.

#### The fix

1. Overlay `rcl_interfaces` into `src/4_interfaces/`
2. Add `rosidl_generator_rs` as a build dependency
3. Rebuild interfaces as a unit
4. Depend on the generated Rust crate by path

```bash
ls install/rcl_interfaces/share/rcl_interfaces/rust/Cargo.toml
```

If that file exists, the setup is correct.

</details>

---

<details>
<summary><strong>Deep Dive: Why Rust Exposes This (and C++ / Python Don’t)</strong></summary>

* C++ uses CMake + ament
* Python uses generated modules installed into the ROS environment
* Neither relies on a global package registry

Rust does.

Cargo requires **explicit, resolvable dependencies**, which exposes mismatches between
ROS’s interface model and registry-based dependency systems.

Once aligned, Rust nodes behave exactly like their C++ and Python counterparts —
with stronger guarantees.

</details>

---

## 2. `2_rcllibrust` — Client / Bridge Nodes

These lessons use `roslibrust`, a pure-Rust ROS client over WebSockets.

* **Role**: External tool or client
* **Transport**: JSON over WebSockets (`rosbridge_suite`)
* **Build system**: `cargo` only
* **Ignored by colcon**: via `COLCON_IGNORE`

### Constraints

* Manual `serde` message definitions
* Async execution (`tokio`)
* Higher latency due to serialization

### Running

Terminal 1 — start the bridge:

```bash
sudo apt install ros-jazzy-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Terminal 2 — run the client:

```bash
cd src/3_rust/2_rcllibrust/lesson_00_bootstrap
cargo run
```

---

## Comparison

| Feature   | `rclrs`         | `roslibrust`     |
| --------- | --------------- | ---------------- |
| Role      | Full ROS 2 node | External client  |
| Transport | DDS             | WebSocket / JSON |
| Build     | `colcon`        | `cargo`          |
| Messages  | Generated       | Manual           |
| Latency   | Low             | Higher           |

---

## Lessons

1. **Lesson 00** — Bootstrap
   Tooling, logging, clean shutdown

2. **Lesson 01** — Event Loop
   Timers, executors, async flow

3. **Lesson 02** — Publishers
   Strong typing, QoS, composition

4. **Lesson 03** — Subscribers & Verification
   Stream validation, reset tolerance

5. **Lesson 04** — Services & Testability
   Service patterns, logic isolation, unit tests

6. **Lesson 05** — Parameters & Configuration
   Central YAML, runtime updates (rclrs only), production-grade patterns

---

### Final takeaway

Rust in ROS 2 is not experimental — it is **explicit**.

Once interface generation and dependency resolution are aligned with ROS’s actual model,
Rust nodes integrate cleanly, safely, and predictably into mixed-language systems.

This workspace encodes those rules so you only have to learn them once.
