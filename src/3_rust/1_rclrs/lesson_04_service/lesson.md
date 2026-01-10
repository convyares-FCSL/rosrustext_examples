# Lesson 03 (roslibrust) – Subscriber

## Goal

Create a node that:

* connects to `rosbridge` and subscribes to the shared `chatter` topic
* manually defines the `MsgCount` struct (since we are in a Cargo-only crate)
* uses shared configuration for topic name and QoS (`utils_roslibrust`)
* validates message flow at runtime
* tolerates late joiners and publisher restarts / manual injections
* shuts down cleanly on Ctrl+C

This lesson focuses on **system verification and transport robustness** via the bridge.

> Assumes Lesson 02 is complete and `rosbridge_server` is running.

---

## Prerequisites

**Terminal 1 (Bridge):**

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

## Build

This is a **Cargo-only Rust crate** inside the workspace.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_03_subscriber/
cargo build
```

---

## Run

Terminal 2:

```bash
RUST_LOG=info cargo run
```

**Expected output (idle):**

```text
[INFO] Connected to ws://localhost:9090
[INFO] Lesson 03 node started (subscriber). Ctrl+C to exit.
```

The node remains quiet until it receives data. This is normal subscriber behaviour.

---

## Verify: Data Flow

### Method 1 – Manual CLI Test

Publish a single message to validate connectivity and QoS compatibility.

Terminal 3:

```bash
ros2 topic pub /chatter lesson_interfaces/msg/MsgCount "{count: 1}" --once
```

Terminal 2 output (example):

```text
[INFO] Received (initial): 1
```

---

### Method 2 – Cross-Language Integration

Keep the Rust subscriber running.

In a third terminal, start a **publisher from another language** (Python or C++ Lesson 02).

Example (C++):

```bash
ros2 run lesson_02_publisher_cpp node
```

Example (Python):

```bash
ros2 run lesson_02_publisher_py lesson_02_publisher
```

Terminal 2 output (example):

```text
[INFO] Received (initial): 1
[INFO] Received: 2
[INFO] Received: 3
...
```

If you restart the publisher or inject a low counter value, the subscriber may log:

```text
[WARN] Detected counter reset. Re-syncing at: 1
```

This is expected and confirms reset tolerance.

---

## Runtime Inspection

Inspect the active subscription and QoS settings:

```bash
ros2 topic info -v /chatter
```

Verify that:

* A node (likely unnamed or auto-named by the bridge) is listed as a subscriber.
* QoS settings match those defined by `utils_roslibrust`.
---

## Notes

* **Topic & QoS** are loaded via `utils_roslibrust` (not hardcoded).
* **Manual Message Definition**: Because this is a Cargo-only crate, we define `MsgCount` manually with `serde` rather than linking generated C++ headers.
* **Late joiners** initialise on the first observed message.
* **Publisher restarts / manual injections** are detected and handled via resync.

For architectural rationale and patterns, see `lesson_breakdown.md`.