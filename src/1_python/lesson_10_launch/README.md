# Lesson 10 (Python) — Launch, Topology & Deployment Verification

## Goal

Close the verification pyramid by proving that the **final system topology** (Lesson 09) can be:

* brought up via canonical ROS launch tooling
* configured using installed resources (not source paths)
* driven through lifecycle transitions
* observed externally via standard ROS CLI tools
* shut down cleanly

Lesson 10 introduces **no new runtime semantics**.
It operationalises what already exists.

> Assumes completion of Lessons 06–09.

---

## What’s New in Lesson 10

Compared to Lesson 09:

* No node logic changes
* No behaviour changes
* No fixes to known failure modes

Lesson 10 adds only:

* a launch description declaring the final topology
* a verification script that proves deployability and hygiene

---

## Deployment Topologies

Lesson 10 introduces two deployment profiles to demonstrate **failure containment**:

### Profile A: Shared Fate (Composed)
*   **Launch**: `composed.launch.py`
*   **Topology**: Single Python process (Lesson 09).
*   **Characteristic**: High coupling. Resource starvation in one node (Action Server) degrades the entire process (Lifecycle Nodes).

### Profile B: Fault-Lined (Isolated)
*   **Launch**: `isolated.launch.py`
*   **Topology**: Multiple processes.
    *   **Domain 1**: Control Plane (Lifecycle Publisher + Subscriber).
    *   **Domain 2**: Worker Plane (Action Server).
*   **Characteristic**: Explicit isolation. Resource starvation in the Worker Plane does **not** degrade the Control Plane.

These profiles typically use the **same node artifacts** without modification. The only difference is the deployment configuration.

---

## Tooling Reality

Lesson 10 uses only canonical ROS 2 tooling:

* `launch` / `launch_ros`
* `ros2 lifecycle`
* `ros2 node`, `ros2 topic`, `ros2 action`
* `ament_index_python.get_package_share_directory`

No custom orchestration services or lesson-specific glue are introduced.

---

## Configuration Discovery

All runtime configuration is shared and installed via the `lesson_interfaces` package.

Lesson 10 locates configuration at runtime using:

```
ament_index_python.packages.get_package_share_directory('lesson_interfaces')
```

This ensures the system works identically:

* from source
* from an installed workspace

No repo-relative paths are used.

---

## Build

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_10_launch
source install/setup.bash
```

---

## Run — Launch Only

To bring up the system via launch:

```bash
ros2 launch lesson_10_launch composed.launch.py
```

This starts the Lesson 09 composed deployment with configuration injected from installed resources.

---

## Run — Verification

To verify deployability and hygiene end-to-end:

```bash
ros2 run lesson_10_launch verify_deployment
```

The verifier will:

1. Assert the cold-start contract (nodes present, actions absent)
2. Drive lifecycle transitions to Active
3. Assert the active-state contract (actions and telemetry visible)
4. Exercise the action pipeline
5. Request shutdown and assert graph convergence

Failures are surfaced, not retried or hidden.

---

## Verification (A/B Test)

To verify the containment hypothesis:

```bash
ros2 run lesson_10_launch verify_fault_tolerance
```

This script runs both profiles against a high-load stimulus:
1.  **Profile A** shows significant latency degradation (> 2x baseline).
2.  **Profile B** maintains nominal latency (~1.0x baseline).
3.  **Profile B** survives the termination of the Action Server process.

## What This Lesson Proves

By the end of Lesson 10, you have evidence that:

* the system is deployable using standard ROS tooling
* lifecycle orchestration works under launch
* known failure modes remain visible
* the system shuts down cleanly

This completes the operational verification layer.
