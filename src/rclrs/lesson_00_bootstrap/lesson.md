# Lesson 00 (rclrs) - Bootstrap

## Goal
Create the smallest possible rclrs package and verify it runs.

## Build
```bash
colcon build --packages-select lesson_00_bootstrap_rclrs
```

## Run
```bash
source install/setup.bash
ros2 run lesson_00_bootstrap_rclrs lesson_00_bootstrap_rclrs
```

## Notes
- Ensure `ament_cargo` is installed and the ROS 2 environment is sourced.
- This node only logs once on startup and then spins.
