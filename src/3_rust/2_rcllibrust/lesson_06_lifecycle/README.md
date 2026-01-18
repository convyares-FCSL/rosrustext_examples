# Lesson 06: Lifecycle (Roslibrust)

This lesson demonstrates a managed lifecycle node (Publisher + Subscriber) using `roslibrust`.

## Build

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_06_lifecycle_roslibrust
source install/setup.bash
```

## Run

**Prerequisite**: Ensure `rosbridge_server` is running.
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 1. Run Publisher
```bash
cargo run --bin lesson_06_lifecycle_publisher
```
*   Node starts in **Unconfigured** state.

### 2. Run Subscriber
```bash
cargo run --bin lesson_06_lifecycle_subscriber
```
*   Node starts in **Unconfigured** state.

## Drive Lifecycle

Use `ros2 service call` to transition the nodes.

**Configure (1)**:
```bash
ros2 service call /lesson_06_lifecycle_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: 'configure'}}"
ros2 service call /lesson_06_lifecycle_subscriber/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: 'configure'}}"
```
*   Nodes transition to **Inactive**. Resources created.

**Activate (3)**:
```bash
ros2 service call /lesson_06_lifecycle_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: 'activate'}}"
ros2 service call /lesson_06_lifecycle_subscriber/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: 'activate'}}"
```
*   Nodes transition to **Active**. Data flows.

**Deactivate (4)**:
```bash
ros2 service call /lesson_06_lifecycle_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4, label: 'deactivate'}}"
```
*   Publisher stops sending data.

**Cleanup (2)**:
```bash
ros2 service call /lesson_06_lifecycle_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 2, label: 'cleanup'}}"
```
*   Resources destroyed. State back to **Unconfigured**.

## Update Parameters

You can update parameters using `ros2 param set`.

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s 0.2
```
*   Observe the publisher speed up (if Active).

```bash
ros2 param set /lesson_06_lifecycle_subscriber validator_reset_limit 100
```
*   Updates the subscriber validation logic.
