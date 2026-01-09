# Lesson 03 (Rust / rclrs) – Subscriber

## Goal

Create a node that:

* subscribes to the shared `chatter` topic
* receives the custom `MsgCount` message
* uses shared configuration for topic name and QoS (`utils_rclrs`)
* validates message flow at runtime
* tolerates late joiners and publisher restarts / manual injections
* interoperates with publishers written in other languages
* shuts down cleanly on Ctrl+C

This lesson focuses on **system verification and transport robustness**, not Rust syntax.

> Assumes Lesson 00–02 (rclrs) are complete and at least one publisher (Rust, C++, or Python) is available as a data source.

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_03_subscriber_rclrs
source install/setup.bash
````

---

## Run

Terminal 1:

```bash
ros2 run lesson_03_subscriber_rclrs lesson_03_node
```

**Expected output (idle):**

```text
[INFO] [lesson_03_node]: Lesson 03 node started (subscriber). Ctrl+C to exit.
```

The node remains quiet until it receives data. This is normal subscriber behaviour.

---

## Verify: Data Flow

### Method 1 – Manual CLI Test

Publish a single message to validate connectivity and QoS compatibility.

Terminal 2:

```bash
ros2 topic pub /chatter lesson_interfaces/msg/MsgCount "{count: 1}" --once
```

Terminal 1 output (example):

```text
[INFO] [lesson_03_node]: Received (initial): 1
```

---

### Method 2 – Cross-Language Integration

Keep the Rust subscriber running.

In a second terminal, start a **publisher from another language** (Python or C++ Lesson 02).

Example (C++):

```bash
ros2 run lesson_02_publisher_cpp node
```

Example (Python):

```bash
ros2 run lesson_02_publisher_py lesson_02_publisher
```

Terminal 1 output (example):

```text
[INFO] [lesson_03_node]: Received (initial): 1
[INFO] [lesson_03_node]: Received: 2
[INFO] [lesson_03_node]: Received: 3
...
```

If you restart the publisher or inject a low counter value, the subscriber may log:

```text
[WARN] [lesson_03_node]: Detected counter reset. Re-syncing at: 1
```

This is expected and confirms reset tolerance.

---

## Runtime Inspection

Inspect the active subscription and QoS settings:

```bash
ros2 topic info -v /chatter
```

Verify that:

* `lesson_03_node` is listed as a subscriber
* QoS settings match those defined by `utils_rclrs` / shared configuration

---

## Notes

* **Topic & QoS** are loaded via `utils_rclrs` (not hardcoded)
* **Late joiners** initialise on the first observed message
* **Publisher restarts / manual injections** are detected and handled via resync
* **Out-of-order warnings** indicate genuine transport or configuration issues
* **No periodic logging** is intentional

For architectural rationale and patterns, see `lesson_breakdown.md`.