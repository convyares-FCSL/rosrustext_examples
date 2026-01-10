# Lesson 02 (rcllibrust) – Publisher

## Goal
Connect to `rosbridge` and publish a custom message (`MsgCount`) to `/chatter` periodically.

This mirrors `rclrs` Lesson 02 but adapts it for an external client:
* **Manual Message Definition**: We define the Rust struct manually to match the JSON schema since we cannot link against the generated C++ library.
* **Shared Utilities**: We use the `utils_roslibrust` crate for shared constants (topics) and configuration (QoS), matching the structure of the native track.
* **Async Publishing**: We use `await` when publishing.

---

## Prerequisites
Assumes Lesson 01 is complete and `rosbridge_server` is installed.

**Terminal 1 (Bridge):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Build

This is a **Cargo-only Rust crate** inside the workspace.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_02_publisher/
cargo build
```

---

## Run

```bash
RUST_LOG=info cargo run
```

### Expected Output

```text
[INFO] Connected to ws://localhost:9090
[INFO] Lesson 02 node started...
```

*(Note: There is no periodic log output. The node publishes silently.)*

---

## Inspect at Runtime

Since the node publishes silently, use these commands in another terminal to verify it is working.

**Terminal 3 (Inspection):**

```bash
source /opt/ros/jazzy/setup.bash

# 1. Echo the topic to see the counting data
# You should see the custom integer message incrementing:
ros2 topic echo /chatter

# 2. Verify the message type is known
ros2 interface show lesson_interfaces/msg/MsgCount
```

---

## What this lesson teaches

* **Observability**: Trusting external tools (`ros2 topic echo`) over internal print debugging.
* **`RosMessageType` Trait**: How to manually map a Rust struct to a ROS 2 message type string (e.g., `lesson_interfaces/msg/MsgCount`).
* **Serde Serialization**: How `rcllibrust` uses `serde` to convert Rust structs to JSON for transmission.
* **Workspace Dependencies**: How to import and use shared logic from `utils_roslibrust` to maintain consistency across the project.