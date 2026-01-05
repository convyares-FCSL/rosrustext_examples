# Lesson 02 (C++) - Publisher

## Goal
Publish `std_msgs/String` messages on the `chatter` topic.

## Build
```bash
colcon build --packages-select lesson_02_publisher_cpp
```

## Run
```bash
source install/setup.bash
ros2 run lesson_02_publisher_cpp lesson_02_publisher
```

## Notes
- Change prefix or rate:
  `--ros-args -p message_prefix:=Hi -p publish_period_s:=0.2`
- Inspect with:
  `ros2 topic echo /chatter`
