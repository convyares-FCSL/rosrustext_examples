# Lesson 06 (Rust / roslibrust) – Lifecycle Management

## Goal

Convert the Publisher and Subscriber nodes from Lesson 05 into **Managed Lifecycle Nodes** using `roslibrust` and the `rosrustext_lifecycle_proxy`.

This lesson uses ONLY standard ROS 2 lifecycle interfaces (`lifecycle_msgs`, optional `bond`).

* Starts in an `Unconfigured` state with no runtime resources allocated.
* Transitions through the standard ROS 2 lifecycle states:
  `Unconfigured` → `Inactive` → `Active`.
* Processes data only while in the `Active` state.
* Supports deterministic shutdown and reset.

**Architecture Note**:
This lesson relies on `rosrustext_lifecycle_proxy` (v0.2.3+) to project the lifecycle onto the ROS graph.
*   **Architecture**: The Rust node connects to rosbridge. The Proxy exposes canonical ROS 2 lifecycle interfaces (`~/change_state`) to the graph.
*   **Usage**: You interact with the **Proxy**, which manages the backend node.

> Assumes completion of Lessons 00–05.

---

## Prerequisites

Before running this lesson, you must ensure the following tools are installed:

1.  **rosbridge_server** (from `rosbridge_suite` package)
2.  **rosrustext_lifecycle_proxy** (External Rust tool)

<details>
<summary><strong>How to Install `rosrustext_lifecycle_proxy`</strong></summary>

This tool bridges the Rust node (via rosbridge) to the standard ROS 2 lifecycle graph. It is not part of the standard workspace and must be installed via Cargo.

Due to dependency updates, you MUST install it with `--locked` to ensure compatibility:

```bash
cargo install rosrustext_lifecycle_proxy --version 0.2.3 --locked
```

Verify installation:
```bash
rosrustext_lifecycle_proxy --help
```
</details>

---

## Build (Terminal 1)

**Role:** This terminal is used for **building** and issuing **lifecycle commands**.

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_06_lifecycle_roslibrust
source install/setup.bash
```

> **Important**: You MUST run `source install/setup.bash` after building to ensure `ros2 run` can find the package.

---

## Run – Infrastructure

We need infrastructure to bridge the Rust nodes to ROS 2. It is best to use separate terminals.

### Terminal 2: Rosbridge

**Role:** Bridges standard ROS 2 traffic to WebSocket (JSON) for the Rust nodes.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Terminal 3: Lifecycle Proxy (Publisher)

**Role:** Projects the **Publisher** node's lifecycle state onto the ROS 2 graph.

```bash
# Must specify the target node name!
rosrustext_lifecycle_proxy --rosbridge-url ws://localhost:9090 --target-node lesson_06_lifecycle_publisher
```

### Terminal 4: Lifecycle Proxy (Subscriber)

**Role:** Projects the **Subscriber** node's lifecycle state onto the ROS 2 graph.
*Required to control the subscriber manually.*

```bash
# Must specify the target node name!
rosrustext_lifecycle_proxy --rosbridge-url ws://localhost:9090 --target-node lesson_06_lifecycle_subscriber
```

---

## Run – Lifecycle Publisher (Terminal 5)

**Role:** Runs the actual Rust Publisher node.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

# Export RUST_LOG to see Info logs!
export RUST_LOG=info

ros2 run lesson_06_lifecycle_roslibrust lesson_06_lifecycle_publisher --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:**
The node starts and logs:
`[INFO] ... Node started. Current state: Unconfigured`

---

## Run – Lifecycle Subscriber (Terminal 6)

**Role:** Runs the actual Rust Subscriber node.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
export RUST_LOG=info

ros2 run lesson_06_lifecycle_roslibrust lesson_06_lifecycle_subscriber --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:**
The subscriber also logs `Current state: Unconfigured`.

---

## Orchestrating the System (Back to Terminal 1)

Lifecycle transitions are driven manually using the ROS 2 CLI.

### 1. Check Initial State

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
ros2 lifecycle get /lesson_06_lifecycle_subscriber
```

Expected output for both:

```
unconfigured
```

---

### 2. Configure Both Nodes (Allocate Resources)

This transition declares parameters, creates topics, and allocates internal resources, but does not enable data flow.

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
```

**Observation:**

* Nodes transition to `Inactive`.
* Parameters exist and can be modified.
* Publishers, subscriptions, timers, and watchers are created.
* Data-plane execution remains gated.

---

### 3. Activate Subscriber First

Activating the subscriber first ensures it is ready before data production begins.

```bash
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

**Observation:**
The subscriber transitions to `Active` but receives no data yet.

---

### 4. Activate Publisher

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher activate
```

**Observation:**

* The publisher begins producing messages.
* The subscriber receives the first sample immediately (starting at count `0`).
* No messages are lost during startup.

---

### 5. Deactivate (Pause Execution)

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate
```

**Observation:**
Publishing stops immediately.
Resources remain allocated, and the node stays in the `Inactive` state.

---

### 6. Cleanup (Release Resources)

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher cleanup
```

**Observation:**

* Timers, publishers, parameter watchers, and internal components are dropped.
* The node returns to the `Unconfigured` state.
* No background activity remains.

---

## Runtime Parameter Updates

Parameters can be modified while the node is `Inactive` or `Active`.

### Example: Change Publish Rate

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s 0.2
```

**Behavior:**

* The `ParameterWatcher` receives the update.
* The value is validated.
* The existing timer is replaced with a new gated timer.
* The updated rate takes effect immediately if the node is `Active`.

### Invalid Values

```bash
ros2 param set /lesson_06_lifecycle_publisher timer_period_s -1.0
```

**Behavior:**
The update is rejected and logged.
The existing configuration remains unchanged.

---

## Automated Testing

### 1. Unit Tests (Logic Verification)
Verifies the business logic (validators, math) without loading the ROS middleware.

```bash
cd ~/ros2_ws_tutorial/src/3_rust/2_rcllibrust/lesson_06_lifecycle
cargo test
# OR
colcon test --packages-select lesson_06_lifecycle_roslibrust --event-handlers console_direct+
```

### 2. Integration Tests (System Verification)
Verifies the node's state machine by treating the compiled binary as a black box.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash
launch_test src/3_rust/2_rcllibrust/lesson_06_lifecycle/test/test_integration.py
```

<details>
<summary><strong>Advanced: Run Stress Test (5x Loop)</strong></summary>

To verify reliability and ensure no race conditions exist during transitions, you can run the test suite in a loop:

```bash
for i in {1..5}; do
   echo "--- Run #$i ---"
   launch_test src/3_rust/2_rcllibrust/lesson_06_lifecycle/test/test_integration.py || break
done
```

</details>

---

## Architecture Notes

* **Async Engine Pattern**
  To ensure robustness, nodes are split into:
  1.  **Non-Blocking Callback Shim**: Implements `LifecycleCallbacks` and queues commands to the engine using `try_send`. This ensures the ROS 2 executor never blocks on the engine.
  2.  **Async Engine**: A dedicated Tokio task that owns all ROS resources (handles) and manages the data plane loop.
  
* **Resource Ownership**
  *   **Publisher**: Publisher handle is created on `Configure`. Data flow is gated by `ActivationGate` (tick loop continues but checks gate).
  *   **Subscriber**: Subscription handle is created on `Activate` and dropped on `Deactivate`. This ensures zero network traffic when inactive.

* **Deterministic Shutdown**
  *   Ctrl-C is intercepted by `tokio::signal`.
  *   A `Shutdown` command is sent to the engine.
  *   The engine drops all resources and exits the loop.
  *   `main` awaits the engine handle to ensure clean exit.
  *   *Note*: Ctrl-C stops the process. Ideally, one should use `ros2 lifecycle set ... shutdown` for orderly graph teardown, but this handler ensures the Rust process exits cleanly to the OS.

* **Lifecycle Proxy**
  State is projected to the ROS graph via `rosrustext_lifecycle_proxy`.

---

## Resulting System Properties

* Nodes do not allocate runtime resources until explicitly configured.
* Data-plane execution occurs only while nodes are in the `Active` state.
* Startup order is deterministic and controlled via lifecycle transitions.
* Resources are released deterministically during cleanup and shutdown.
* Runtime reconfiguration is handled safely without restarting nodes.

```bash
ros2 param set /lesson_06_lifecycle_subscriber validator_reset_limit 100
```
*   Updates the subscriber validation logic.

---

## Jazzy / Rolling Note

If you are running on ROS 2 Jazzy or later, the `-n 1` flag for `ros2 topic echo` might behave differently. Use the `--once` flag instead:

```bash
ros2 topic echo /lesson_06_lifecycle_publisher/transition_event --once
```

---

## Verification Evidence (v0.2.3 End-to-End)

The following transcript proves that `ros2 lifecycle` commands are fully functional on ROS 2 Jazzy using `rosrustext_lifecycle_proxy` v0.2.3.

### 1. Publisher Lifecycle Sequence
```bash
# Check Initial State
ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_publisher
unconfigured [1]

# Configure
ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle set /lesson_06_lifecycle_publisher configure
Transitioning successful

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_publisher
inactive [2]

# Activate
ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle set /lesson_06_lifecycle_publisher activate
Transitioning successful

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_publisher
active [3]
```

### 2. Subscriber Lifecycle Sequence
```bash
ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_subscriber
unconfigured [1]

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
Transitioning successful

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_subscriber
inactive [2]

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
Transitioning successful

ecm@FCS-AZ-LAP-CVA:~$ ros2 lifecycle get /lesson_06_lifecycle_subscriber
active [3]
```

### 3. Data Flow Verification
Verified via `ros2 topic echo` that messages are received only when nodes are `Active`.
```bash
ecm@FCS-AZ-LAP-CVA:~$ ros2 topic echo /tutorial/telemetry --once --timeout 10
count: 37
---
```

### 4. Transition Events
The proxy correctly emits `TransitionEvent` messages:
```bash
ecm@FCS-AZ-LAP-CVA:~$ ros2 topic echo /lesson_06_lifecycle_publisher/transition_event --once
timestamp: 1768827927174747636
transition:
  id: 4
  label: deactivate
start_state:
  id: 3
  label: Active
goal_state:
  id: 2
  label: Inactive
---
```
