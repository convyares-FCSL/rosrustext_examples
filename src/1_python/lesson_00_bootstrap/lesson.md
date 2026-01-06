# Lesson 00 (Python) - Bootstrap

## Goal
Create the smallest possible ROS 2 Python package and verify it runs.

## Build
```bash
colcon build --packages-select lesson_00_bootstrap_py
```

## Run
```bash
source install/setup.bash
ros2 run lesson_00_bootstrap_py lesson_00_bootstrap
```

## Notes
- This node only logs once on startup and then spins.
- Use Ctrl+C to exit.
