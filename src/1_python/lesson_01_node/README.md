# Lesson 01 (Python) – Simple Node (Timer + Logging)

## Goal
Create a node that:
- runs continuously
- logs periodically from a timer callback
- supports a parameter to control the timer period
- shuts down cleanly on Ctrl+C

> Assumes you already completed Lesson 00 (ROS sourced, workspace builds, install/setup.bash sourcing).

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_01_node_py --symlink-install
source install/setup.bash
````

---

## Run

```bash
ros2 run lesson_01_node_py lesson_01_node
```

**Expected output:** a log line every second (default).

---

## Parameter: timer_period_s

Change the timer period without editing code:

```bash
ros2 run lesson_01_node_py lesson_01_node --ros-args -p timer_period_s:=0.2
```

---

## Inspect at runtime

In another terminal (with the same ROS + workspace sourced):

Get node list:
```bash
ros2 node list  
```

Get parameter list:
```bash
ros2 param list /lesson_01_node
```

Get parameter value:
```bash
ros2 param get /lesson_01_node timer_period_s
```

---

## Notes

* This node uses `rclpy.spin(node)` and is intended to run until Ctrl+C.
* Ctrl+C should exit without a stack trace (clean destroy + shutdown).