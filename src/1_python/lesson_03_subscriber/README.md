# Lesson 03 (Python) – Subscriber

## Goal

Create a node that:

* subscribes to the shared `chatter` topic
* receives a custom `MsgCount` message
* uses shared configuration for topic name and QoS
* validates message flow at runtime
* detects publisher resets and resynchronises safely
* interoperates with publishers written in other languages
* shuts down cleanly on Ctrl+C

This lesson focuses on **system verification**, not API mechanics.

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_03_subscriber_py --symlink-install
source install/setup.bash
```

---

## Run

Terminal 1:

```bash
ros2 run lesson_03_subscriber_py lesson_03_subscriber
```

**Expected output (idle):**

```text
[INFO] [lesson_03_node]: Lesson 03 node started. Subscribing to 'chatter'
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

If additional messages with a lower counter value are published later, you may see:

```text
[WARN] [lesson_03_node]: Detected counter reset. Re-syncing at: 1
```

This is expected and intentional.

---

### Method 2 – Cross-Language Integration

Keep the Python subscriber running.

In a second terminal, start the **Rust publisher from Lesson 02**.

Terminal 2:

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

If the publisher is restarted or a manual CLI message resets the counter, the subscriber will log a single reset warning and continue cleanly.

This confirms:

* message definitions are shared correctly
* DDS serialization works across languages
* QoS profiles are compatible
* late joiners and restarts are handled safely

---

## Runtime Inspection

Inspect the active subscription and QoS settings:

```bash
ros2 topic info -v /chatter
```

Verify:

* `lesson_03_node` is listed as a subscriber
* QoS settings match the shared configuration

---

## Notes

* **Topic & QoS** are loaded from `utils_py` (not hardcoded)
* **Late joiners** initialise on the first observed message
* **Publisher resets** are detected and handled automatically
* **Out-of-order warnings** indicate genuine transport or configuration issues
* **No periodic logging** is intentional

For architectural rationale and design patterns, see `lesson_breakdown.md`.

---

## Summary

Lesson 03 validates that your ROS 2 system:

* communicates correctly across languages
* behaves predictably under shared configuration
* tolerates restarts and manual injections
* is ready for services, actions, and lifecycle management

From here on, lessons assume a functioning, language-agnostic ROS graph and focus on coordination, correctness, and failure modes.
