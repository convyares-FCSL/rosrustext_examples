# Lesson 01 (C++) – Simple Node (Timer + Logging)

## Goal

Create a minimal `rclcpp` node that:

- runs continuously
- uses a periodic timer
- logs a tick counter
- shuts down cleanly on Ctrl+C
- accepts a parameter to change the timer period

This mirrors the Python Lesson 01 behavior.

---

## Build

From the workspace root:

```bash
colcon build --packages-select lesson_01_node_cpp
````

---

## Run

Source your workspace:

```bash
source install/setup.bash
```

Run the node:

```bash
ros2 run lesson_01_node_cpp node
```

Expected output:

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

* `timer_period_s` (double, seconds)

Override it at runtime:

```bash
ros2 run lesson_01_node_cpp node --ros-args -p timer_period_s:=0.2
```

This runs the timer at 0.2 seconds (5 Hz).

Notes:

* Values `<= 0` are clamped back to `1.0`.
* The implementation is structured so we can later extend it in Lesson 05 to support **live parameter updates**.

---

## What this lesson teaches

* `rclcpp::Node` basics
* `create_wall_timer` usage
* ROS logging (`RCLCPP_INFO/WARN`)
* declaring and reading a parameter
* clean shutdown via `rclcpp::spin()` + Ctrl+C
