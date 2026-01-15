
# Interfaces and Tools

This directory contains **shared ROS 2 interface packages** and the **tooling required to generate language bindings** from them.

It exists to make interface usage **consistent, deterministic, and cross-language**, including Rust.

---

## Directory Structure

```text
4_interfaces/
├── lesson_interfaces/        # Tutorial-specific msgs/srvs/actions
│   ├── config/
│   ├── msg/
│   ├── srv/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── rcl_interfaces/           # Core ROS 2 interfaces (overlay)
├── lifecycle_msgs/           # Core ROS 2 interfaces (overlay)
├── unique_identifier_msgs/   # Core ROS 2 interfaces (overlay)
│
├── rosidl_rust/              # rosidl_generator_rs (Rust interface generator)
└── README.md
```

Not every folder exists for *every* language — they exist because **Rust requires them**.

---

## Why These Packages Exist Here

### `lesson_interfaces`

Shared tutorial interfaces (messages, services, actions).

* Used by **all languages**
* Built once via `colcon`
* Acts as the canonical interface definition for the workspace

---

### `rosidl_rust`

Contains `rosidl_generator_rs`, the Rust code generator for ROS interfaces.

* Enabled explicitly via `<build_depend>rosidl_generator_rs</build_depend>`
* Generates Rust crates alongside C++ and Python artifacts
* Generator configuration is **intentionally modified** to emit **path-based Cargo dependencies**

This ensures Cargo resolves interfaces from the workspace, not crates.io.

---

### `rcl_interfaces`, `lifecycle_msgs` and `unique_identifier_msgs` (Overlays)

These are **local clones of core ROS 2 interface packages**.

They exist here because:

* System ROS installations (e.g. `/opt/ros/jazzy`) **do not ship Rust bindings**
* Rust nodes **cannot** depend on these interfaces unless Rust artifacts are generated
* Cargo cannot resolve these interfaces from crates.io

By overlaying and rebuilding them locally with `rosidl_generator_rs` enabled, we produce:

```text
install/<pkg>/share/<pkg>/rust/Cargo.toml
```

which Rust nodes can depend on safely via `path =`.

> These overlays are **not forks** and do **not change interface definitions** —
> they exist solely to generate Rust bindings that the system install does not provide.

The rationale and failure mode are documented in detail in:

**`src/3_rust/README.md` → “Rust Interface Generation & Local Overlay”**

---

> Core ROS interface packages (`rcl_interfaces`, `builtin_interfaces`, etc.) are **not committed to the repository**.
> They are cloned and built locally by `build_interfaces.sh` to generate Rust bindings required by `rclrs`.

---

## Usage in Rust Nodes

To use `lesson_interfaces` or any core ROS interface in a Rust (`rclrs`) node:

1. **Ensure the interface was built locally**

   Run:

   ```bash
   ./scripts/01_setup/build_interfaces.sh
   ```

2. **Depend on the generated artifact**, not the source package:

   ```toml
   [dependencies]
   lesson_interfaces = { path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust" }
   ```

   Core interfaces (e.g. `rcl_interfaces`) follow the same pattern.

If the `install/**/rust/Cargo.toml` file does not exist, the interface is **not usable from Rust**.

---

## Design Intent

This directory exists to enforce one rule:

> **ROS interfaces are build artifacts, not registry dependencies.**

C++ and Python resolve this implicitly through ament.
Rust requires it to be made explicit.

Everything here exists to make that explicit **once**, so lessons and nodes stay clean.

---

### What This Directory Is *Not*

* Not a fork of ROS core packages
* Not a vendor dump
* Not Rust-specific APIs

It is **infrastructure**, not tutorial code.

