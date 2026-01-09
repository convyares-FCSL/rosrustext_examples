# Lesson 02 (Python) – Publisher

## Goal
Create a node that:
- publishes a custom message (increasing integer) on the `chatter` topic
- uses shared configuration for topic name and QoS (loaded from `src/1_python/utils_py/config`)
- supports a parameter to control the publish rate
- shuts down cleanly on Ctrl+C

> Assumes you already completed Lesson 00 (ROS sourced, workspace builds, install/setup.bash sourcing) & Lesson 01 (Python: simple publisher node).

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_02_node_py --symlink-install
source install/setup.bash
```

---

## Run

```bash
ros2 run lesson_02_node_py lesson_02_node
```

**Expected output:**
- Log message: "Publisher node started..."
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
*(Note: Replace `lesson_interfaces` with the actual package name where the message is built)*

### 3. Check publisher details
Verify the node is publishing with the correct QoS settings (Reliability, Durability, etc.):
```bash
ros2 topic info -v /chatter
```
Look for `Node name: lesson_02_node` and verify the QoS profile matches `qos_config.yaml`.

---

## Parameter: timer_period_s

Change the publish rate without editing code:

```bash
ros2 run lesson_02_node_py lesson_02_node --ros-args -p timer_period_s:=0.2
```

---

## Notes

* **Topic & QoS**: Loaded from `src/1_python/utils_py/config` (not hardcoded).
* **Message Type**: `MsgCount` (int) defined in `src/lesson_interfaces/msg/MsgCount.msg`.
* **Behavior**: Defaults to 1.0s period if `timer_period_s` is not set.
