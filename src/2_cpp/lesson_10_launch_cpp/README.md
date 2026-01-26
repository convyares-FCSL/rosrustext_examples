# Lesson 10 (C++) — Launch, Topology & Deployment Verification

## Goal

Demonstrate **failure containment** by comparing two operational topologies:
1.  **Shared Fate**: All nodes in one process (Lesson 09 behavior).
2.  **Fault-Lined**: Resource-intensive nodes isolated in separate processes.

This lesson reuses the node artifacts from Lessons 06 and 08 without modification. The architectural choices are made entirely in the launch description.

---

## Deployment Topologies

### Profile A: Shared Fate (Composed)
*   **Launch**: `composed.launch.py`
*   **Topology**: Single C++ process (`component_container_mt`).
*   **Components**: Lifecycle Publisher, Lifecycle Subscriber, Action Server.
*   **Behavior**: High coupling. Resource starvation in the Action Server degrades the Lifecycle nodes. Process termination kills all nodes.

### Profile B: Fault-Lined (Isolated)
*   **Launch**: `isolated.launch.py`
*   **Topology**: Multiple processes.
    *   **Domain 1 (Control Plane)**: `component_container_mt` hosting Lifecycle Publisher + Subscriber.
    *   **Domain 2 (Worker Plane)**: `lesson_08_action_server` executable hosting the Action Server.
*   **Behavior**: Explicit isolation. Resource starvation in the Worker Plane does not block the Control Plane. Worker Plane termination does not kill the Control Plane.

---

## Build

```bash
colcon build --packages-select lesson_10_launch_cpp
source install/setup.bash
```

## Verification (A/B Test)

Run the automated verification script to observe the difference:

```bash
ros2 run lesson_10_launch_cpp verify_fault_tolerance.py
```

This script:
1.  Runs Profile A, applies load, measures degradation.
2.  Runs Profile B, applies load, proves stability/survival.
3.  Asserts that Profile B survives the destruction of the Worker Plane.

---

## Tooling

*   `launch_ros.actions.ComposableNodeContainer`
*   `launch_ros.actions.Node` (Executable)
*   `ros2 lifecycle`
*   `PGREP` detection of topology
