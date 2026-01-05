# Lesson 02 (rclrs) - Publisher

## Goal
Publish `std_msgs/String` messages on the `chatter` topic.

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
