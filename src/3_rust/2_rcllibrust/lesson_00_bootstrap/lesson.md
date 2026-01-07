# Lesson 00 (rcllibrust) – Bootstrap

## Goal
Connect to **rosbridge** and verify the `roslibrust` client runs.

---

## Where To Run Commands
This lesson is a **Cargo-only Rust crate** that lives inside a larger ROS workspace.

All `cargo` commands **must be run from the crate directory**:
```bash
~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_00_bootstrap
````

ROS setup (`source /opt/ros/jazzy/setup.bash`) and rosbridge launch can be run from any directory.

---

## Official Resources

* [RosLibRust / roslibrust (GitHub)](https://github.com/RosLibRust/roslibrust)
* [RosLibRust Documentation](https://roslibrust.github.io/roslibrust/)
* [rosbridge_suite (ROS 2 Jazzy docs)](https://docs.ros.org/en/ros2_packages/jazzy/api/rosbridge_suite/)
* [rosbridge_suite (GitHub)](https://github.com/RobotWebTools/rosbridge_suite)

---

## What is rosbridge?

`rosbridge` exposes ROS topics and services over a **WebSocket JSON API**, allowing non-ROS programs (such as Rust clients) to communicate with ROS 1 or ROS 2.

This lesson expects a rosbridge WebSocket server listening on:

```text
ws://localhost:9090
```

---

## Step-by-Step Setup

### 1. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Install rosbridge

```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
```

### 3. Run rosbridge WebSocket server

In a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Cargo-only crate (colcon ignore)

This lesson is intentionally **not** a ROS package.

A `COLCON_IGNORE` file is already present in the crate root so that `colcon build`
does not attempt to treat this directory as a ROS package.

---

## Build (compile only)

Use this to **verify the crate compiles**, without running it:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_00_bootstrap
cargo build
```

---

## Run (build + execute)

`cargo run` will **build if needed and then execute** the program:

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_00_bootstrap
RUST_LOG=info cargo run
```

Alternatively, after building once, you can run the binary directly to avoid a build step:

```bash
RUST_LOG=info ./target/debug/lesson_00_bootstrap_rcllibrust
```

### Expected Output

```text
[INFO] Lesson 00 bootstrap client connected to ws://localhost:9090.
```

This lesson connects once, logs a message, and exits.

---

## Notes

* This lesson uses standard Rust logging (`log` + `env_logger`).
  `roslibrust` is a client library and does **not** integrate with ROS logging
  or publish to `/rosout`.
* Logging verbosity is controlled via the `RUST_LOG` environment variable.
* This lesson intentionally performs a single connection and exits.
* No publishers or subscribers are created yet.
* Publishing, subscribing, and feature-gated backends (`ros1`, `ros2`, `zenoh`)
  are introduced in later lessons.
