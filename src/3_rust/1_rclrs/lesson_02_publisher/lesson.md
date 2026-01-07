# Lesson 02 (rclrs) - Publisher

## Goal
Publish `std_msgs/String` messages on the `chatter` topic.

## Prerequisites
- **Sourcing ROS 2**: You MUST source your ROS 2 installation before building. This sets the `ROS_DISTRO` environment variable which `rclrs` requires during its build process.
  ```bash
  source /opt/ros/<distro>/setup.bash
  ```

## Build
```bash
colcon build --packages-select lesson_02_publisher_rclrs
```

## Run
```bash
source install/setup.bash
ros2 run lesson_02_publisher_rclrs lesson_02_publisher_rclrs
```

## Notes
- Inspect with:
  `ros2 topic echo /chatter`
