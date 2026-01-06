# Lesson 01 (C++) - Simple Node

## Goal
Create a node with a periodic timer and logging.

## Build
```bash
colcon build --packages-select lesson_01_node_cpp
```

## Run
```bash
source install/setup.bash
ros2 run lesson_01_node_cpp lesson_01_node
```

## Notes
- Adjust the timer period with:
  `--ros-args -p timer_period_s:=0.2`
