# Lesson 03 (rclrs) - Subscriber

## Goal
Subscribe to `std_msgs/String` on the `chatter` topic.

## Prerequisites
- **Sourcing ROS 2**: You MUST source your ROS 2 installation before building. This sets the `ROS_DISTRO` environment variable which `rclrs` requires during its build process.
  ```bash
  source /opt/ros/<distro>/setup.bash
  ```

## Build
```bash
colcon build --packages-select lesson_03_subscriber_rclrs
```

## Run
```bash
source install/setup.bash
ros2 run lesson_03_subscriber_rclrs lesson_03_subscriber_rclrs
```

## Notes
- Pair with Lesson 02 publisher from any language.
