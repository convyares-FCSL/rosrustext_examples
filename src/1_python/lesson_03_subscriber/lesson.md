# Lesson 03 (Python) - Subscriber

## Goal
Subscribe to `std_msgs/String` on the `chatter` topic.

## Build
```bash
colcon build --packages-select lesson_03_subscriber_py
```

## Run
```bash
source install/setup.bash
ros2 run lesson_03_subscriber_py lesson_03_subscriber
```

## Notes
- Pair with Lesson 02 publisher:
  `ros2 run lesson_02_publisher_py lesson_02_publisher`
