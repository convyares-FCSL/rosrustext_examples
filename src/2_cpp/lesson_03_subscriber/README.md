# Lesson 03 (C++) – Subscriber

## Goal

Create a node that:

* subscribes to the shared `chatter` topic
* receives the custom `MsgCount` message
* uses shared configuration for topic name and QoS (`utils_cpp`)
* validates message flow at runtime
* tolerates late joiners and publisher restarts
* interoperates with publishers written in other languages
* shuts down cleanly on Ctrl+C

This lesson focuses on **system verification and transport robustness**, not C++ syntax.

> Assumes Lesson 00–02 are complete and the publisher from Lesson 02 is available as a data source.

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_03_subscriber_cpp
source install/setup.bash
```

---

## Run

Terminal 1:

```bash
ros2 run lesson_03_subscriber_cpp node
```

**Expected output (idle):**

```text
[INFO] [lesson_03_node]: Lesson 03 node started (subscriber). Ctrl+C to exit.
```

The node remains silent until data is received.
This is correct subscriber behaviour.

---

## Verify: Data Flow

### Method 1 – Manual CLI Test

Publish a single message to validate connectivity and QoS compatibility.

Terminal 2:

```bash
ros2 topic pub /chatter lesson_interfaces/msg/MsgCount "{count: 1}" --once
```

Terminal 1 output:

```text
[INFO] [lesson_03_node]: Received (initial): 1
```

If a lower counter value is observed after normal operation, the node may log:

```text
[WARN] [lesson_03_node]: Detected counter reset. Re-syncing at: 1
```

This is expected behaviour.

---

### Method 2 – Cross-Language Integration

Keep the C++ subscriber running.

In a second terminal, start a **publisher from another language** (e.g. Rust or Python Lesson 02).

Terminal 2 (example – Rust):

```bash
ros2 run lesson_02_publisher_rclrs lesson_02_node
```

Terminal 1 output:

```text
[INFO] [lesson_03_node]: Received (initial): 0
[INFO] [lesson_03_node]: Received: 1
[INFO] [lesson_03_node]: Received: 2
...
```

This confirms:

* DDS serialization works across languages
* message definitions are shared correctly
* QoS profiles are compatible
* the system behaves as a single logical graph

---

## Runtime Inspection

Inspect the active subscription and QoS settings:

```bash
ros2 topic info -v /chatter
```

Verify that:

* `lesson_03_node` is listed as a subscriber
* QoS settings match those defined by `utils_cpp::qos::telemetry`

---

## Notes

* **Topic & QoS** are loaded via `utils_cpp` (not hardcoded)
* **Late joiners** initialise on the first observed message
* **Publisher restarts** are detected and handled gracefully
* **Out-of-order warnings** indicate genuine transport or configuration issues
* **No periodic logging** is intentional

For architectural rationale and design patterns, see `lesson_breakdown.md`.

---

## Summary

Lesson 03 (C++) validates that your ROS 2 system:

* communicates correctly across languages
* behaves predictably under shared configuration
* tolerates restarts and partial system resets
* is ready for services, actions, and lifecycle management

From this point onward, lessons build on **behavioural guarantees**, not API familiarity.