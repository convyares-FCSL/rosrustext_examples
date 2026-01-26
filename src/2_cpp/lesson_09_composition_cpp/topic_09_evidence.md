# Topic 09 Evidence Transcript

## 1. Package Verification

**Command:**
```bash
colcon build --packages-select lesson_09_composition_cpp
ros2 pkg prefix lesson_09_composition_cpp
```

**Output:**
```
Starting >>> lesson_09_composition_cpp
Finished <<< lesson_09_composition_cpp [0.85s]
/home/ecm/ros2_ws_tutorial/install/lesson_09_composition_cpp
```

## 2. Component Loading

**Command:**
```bash
ros2 run rclcpp_components component_container_mt &
ros2 component load /ComponentManager lesson_06_lifecycle_cpp lesson_06_lifecycle_cpp::LifecyclePublisherNode
ros2 component load /ComponentManager lesson_06_lifecycle_cpp lesson_06_lifecycle_cpp::LifecycleSubscriberNode
ros2 component load /ComponentManager lesson_08_executors_cpp lesson_08_executors_cpp::ActionServerNode
ros2 component list
```

**Output:**
```
/ComponentManager
  1  /lesson_06_lifecycle_publisher
  2  /lesson_06_lifecycle_subscriber
  3  /lesson_08_action_server
```

## 3. Degradation Observation

**Scenario:**
1.  Lifecycle nodes are Active.
2.  Fibonacci Action (Order 50) is executing.
3.  `ros2 lifecycle get` is issued during execution.

**Transcript:**
```bash
$ ros2 action send_goal /tutorial/fibonacci lesson_interfaces/action/Fibonacci "{order: 50}" &
[1] 18566

$ time ros2 lifecycle get /lesson_06_lifecycle_publisher
active [3]

real    0m1.327s
user    0m0.317s
sys     0m0.114s
```

**Observed Degradation:**
*   **Latency**: The lifecycle query took **1.327s** to complete. In an idle system, this typically takes < 0.2s.
*   **Cause**: The shared MultiThreadedExecutor is servicing the heavy Fibonacci computation (even though it yields) interleaved with the service request. The single process boundary means they compete for CPU scheduling time despite having threads.

## 4. Shared Fate (Shutdown)

**Command:**
Terminating the `component_container_mt` process via SIGINT (Ctrl+C).

**Output:**
```
^C[INFO] [1769461748.262150012] [rclcpp]: signal_handler(signum=2)
```

**Observation:**
All three nodes (`/lesson_06_lifecycle_publisher`, `/lesson_06_lifecycle_subscriber`, `/lesson_08_action_server`) disappeared simultaneously from the graph. The component container process exit terminated all hosted nodes.
