# Lesson 02 (Rust) – Publisher

## Goal
Create a node that:
- publishes a custom message (increasing integer) on the `chatter` topic
- uses shared configuration for topic name and QoS (using `utils_rust`)
- supports a parameter to control the publish rate
- shuts down cleanly on Ctrl+C

> Assumes you already completed Lesson 00 & Lesson 01 (Rust).

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_02_publisher_rclrs
source install/setup.bash
```

---

## Run

```bash
ros2 run lesson_02_publisher_rclrs lesson_02_node
```

**Expected output:**
- Log message: "Lesson 02 node started..."
- (No periodic log output—messages are published silently)

---

## Inspect at runtime

Since the node publishes silently, use these commands in another terminal to verify it is working:

### 1. View published messages
See the data flowing in real-time. You should see the custom integer message incrementing:
```bash
ros2 topic echo /chatter
```

### 2. Inspect message structure
Check the field definition of the custom message (`MsgCount.msg`):
```bash
ros2 interface show lesson_interfaces/msg/MsgCount
```

### 3. Check publisher details
Verify the node is publishing with the correct QoS settings (Reliability, Durability, etc.):
```bash
ros2 topic info -v /chatter
```
Look for `Node name: lesson_02_node` and verify the QoS profile matched defaults (Reliable, Volatile).

---

## Parameter: timer_period_s

Change the publish rate without editing code:

```bash
ros2 run lesson_02_publisher_rclrs lesson_02_node --ros-args -p timer_period_s:=0.2
```

---

## Notes

* **Topic & QoS**: Handled by `utils_rust` helpers.
* **Message Type**: `MsgCount` (int) defined in `src/4_interfaces/msg/MsgCount.msg`.
* **Behavior**: Defaults to 1.0s period if `timer_period_s` is not set.

---

## Custom Messages in Rust

To use your own custom messages (like `MsgCount`):

1. **Define the Message**: Create `.msg` file in an interface package (e.g. `lesson_interfaces`).
2. **Enable Generator**: In `lesson_interfaces/package.xml`, you MUST add:
   ```xml
   <build_depend>rosidl_generator_rs</build_depend>
   ```
3. **Usage**: In your node's `Cargo.toml`, depend on the **generated** crate in the install folder, NOT the source folder:
   ```toml
   [dependencies]
   lesson_interfaces = { path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust" }
   ```
   *Note: The generator outputs the Rust crate to the install directory, it does not exist in the source tree.*

