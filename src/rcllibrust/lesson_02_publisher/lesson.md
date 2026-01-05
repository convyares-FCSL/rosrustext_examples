# Lesson 02 (rcllibrust) - Publisher

## Goal
Publish messages on the `chatter` topic using rosbridge.

## Build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cargo build
```

## Run
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cargo run
```

## Notes
- Requires a running rosbridge server at `ws://localhost:9090`.
- Uses ROS 2 message definitions from `ROS_PACKAGE_PATH`.
