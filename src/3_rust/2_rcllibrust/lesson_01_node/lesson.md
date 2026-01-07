# Lesson 01 (rcllibrust) – Simple Client (Timer + Logging)

## Goal

Create a minimal `roslibrust` application that:

* runs continuously (Tokio async)
* connects to ROS via `rosbridge` (`ws://localhost:9090`)
* uses a periodic timer (`tokio::time::interval`)
* logs a tick counter (Rust `log` + `env_logger`)
* shuts down cleanly on Ctrl+C

This mirrors the intent of `rclrs` Lesson 01 (timer + logging), but uses a rosbridge client rather than an `rcl` node.

---

## Prerequisites

Assumes Lesson 00 is complete and you already have `rosbridge_suite` available.

In a terminal where you will run rosbridge:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Where To Run Commands

This lesson is a **Cargo-only Rust crate** inside a larger ROS workspace.

All `cargo` commands must be run from the crate directory:

```bash
~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_01_node
```

This crate should remain **colcon-ignored** (contains `COLCON_IGNORE`).

---

## Build

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_01_node
cargo build
```

---

## Run

### Option A: build + run

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_01_node
RUST_LOG=info cargo run
```

### Option B: run the built binary directly

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_01_node
RUST_LOG=info ./target/debug/lesson_01_node_rcllibrust
```

Expected output (shape):

```text
[INFO] Lesson 01 client connected to ws://localhost:9090.
[INFO] Lesson 01 client started (timer + logging). Ctrl+C to exit. period=1.000s
[INFO] tick 1
[INFO] tick 2
...
```

Stop with **Ctrl+C**.

---

## Parameter: `timer_period_s`

ROS 2 parameters are **work in progress** for `roslibrustext` / rosbridge workflows.

For Lesson 01:

* the timer period is **fixed** in code (default `1.0s`)
* runtime configuration is deferred until parameter support is available

(Do **not** use `--ros-args` with this lesson; it is not an `rcl` node and will not parse ROS arguments automatically.)

---

## What this lesson teaches

* establishing a long-lived `roslibrust` rosbridge client connection
* periodic execution using Tokio timers (not a ROS executor)
* logging using Rust (`log` + `env_logger`)
* clean shutdown using `tokio::signal::ctrl_c`