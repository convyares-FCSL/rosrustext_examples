# Lesson 01 (rclrs) - Simple Node

## Goal
Create a node with a periodic timer and logging.

## Prerequisites
- **Sourcing ROS 2**: You MUST source your ROS 2 installation before building. This sets the `ROS_DISTRO` environment variable which `rclrs` requires during its build process.
  ```bash
  source /opt/ros/<distro>/setup.bash
  ```

## Build
```bash
colcon build --packages-select lesson_01_node_rclrs
```

## Run
```bash
source install/setup.bash
ros2 run lesson_01_node_rclrs lesson_01_node_rclrs
```
