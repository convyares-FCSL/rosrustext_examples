# Lesson 01 (rclrs) – Simple Node (Timer + Logging)

## Goal

Create a minimal `rclrs` node that:

- runs continuously
- uses a periodic timer
- logs a tick counter
- shuts down cleanly on Ctrl+C
- accepts a parameter to change the timer period

This mirrors Python/C++ Lesson 01.

---

## Prerequisites

You must source ROS 2 **before building** so `ROS_DISTRO` and the ament/rosidl environment are available:

```bash
source /opt/ros/jazzy/setup.bash
````

Then build and source your workspace as usual.

---

## Build

From the workspace root:

```bash
colcon build --packages-select lesson_01_node_rclrs
```

---

## Run

Source your workspace:

```bash
source install/setup.bash
```

Run the executable:

```bash
ros2 run lesson_01_node_rclrs lesson_01_node_rclrs
```

Expected output (shape):

```text
[INFO] [lesson_01_node]: Timer running with period=1.000s
[INFO] [lesson_01_node]: Lesson 01 node started (timer + logging). Ctrl+C to exit.
[INFO] [lesson_01_node]: tick 1
[INFO] [lesson_01_node]: tick 2
...
```

Stop with **Ctrl+C**.

---

## Parameter: `timer_period_s`

The node declares one parameter:

* `timer_period_s` (float seconds)

Override it at runtime:

```bash
ros2 run lesson_01_node_rclrs lesson_01_node_rclrs --ros-args -p timer_period_s:=0.2
```

Notes:

* Values `<= 0` are clamped back to `1.0`.
* Live parameter update callbacks are intentionally deferred to Lesson 05.

---

## What this lesson teaches

* `rclrs::Node` basics
* creating a repeating timer
* ROS logging
* declaring and reading a parameter
* clean shutdown under an app-owned executor