# Lesson 03 (rcllibrust) - Subscriber

## Goal
Subscribe to messages on the `chatter` topic using rosbridge.

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
- Pair with Lesson 02 publisher from any language.
