# Python Lesson 10 Gap Analysis: Closure for Topic 10

## 1. Current State Summary (Facts Only)

- **Artifacts**
  - `composed.launch.py`: Launches `lesson_09_composed_deployment` (Lesson 09 topology).
  - `verification.py`: Verifies node presence, lifecycle drive, surface checks, and basic action execution.
- **Topology**
  - **Single failure domain**: Lifecycle Publisher, Lifecycle Subscriber, and Action Server run inside one OS process under a single `MultiThreadedExecutor`.
- **Observation**
  - **Containment failure**: During high-load action execution (`Fibonacci order=50`), unrelated lifecycle queries to the publisher exhibited materially increased latency compared to baseline. The load propagated across unrelated components because they share a failure domain.

## 2. Intent Alignment Check (Topic 10 Clauses)

| Clause | Status | Reasoning |
| :--- | :---: | :--- |
| Deterministic bring-up | PASS | `composed.launch.py` reliably starts the system using configuration from the installed share (`lesson_interfaces`). |
| Tooling parity preserved | PASS | Canonical `ros2` CLI tools interact correctly with the deployed graph. |
| Deployment verification exists | PASS | `verify_deployment` exists and asserts the contract, but only for the shared-fate topology. |
| Explicit failure containment (Profile B) | FAIL | No topology exists that isolates the long-running action server into a separate failure domain. Lesson 10 currently demonstrates *fragility* (Profile A), not *containment* (Profile B). |

## 3. Profile B Gap Statement

To satisfy Topic 10, Python Lesson 10 is missing a **fault-lined topology ("Profile B")**.

**Requirements**
1. **Second launch topology**: Introduce a second launch profile (e.g. `isolated.launch.py`) that deploys the *same node artifacts* but splits them into **at least two OS processes / failure domains**:
   - **Domain 1 (Control plane)**: Lifecycle publisher + subscriber
   - **Domain 2 (Worker plane)**: Action server
2. **Demonstrated containment**: Using the *same stimulus* as Profile A (high-order Fibonacci), Profile B must demonstrate that Domain 1 remains responsive while Domain 2 is under load.
3. **No logic changes**: This must be achieved purely via launch topology (composition/executable mapping). No modifications to node code, public interfaces, or lesson semantics.

## 4. Acceptance Evidence for Profile B (Observable, Canonical)

Profile B is complete only when the following is observable via canonical tools:

1. **Responsiveness under load**
   - **Stimulus**: `ros2 action send_goal /tutorial/fibonacci ... {order: 50}`
   - **Probe**: repeated `ros2 lifecycle get /lesson_06_lifecycle_publisher`
   - **Success criteria**: Probe latency remains within the baseline observed when the action is not running (i.e., no material degradation attributable to the action load).
2. **Topology confirmation**
   - `ros2 node list` shows all expected nodes.
   - Process evidence confirms distinct PIDs for the defined domains (e.g., via logs or `ps`).
3. **Surviving failure-domain termination**
   - Terminating the Action Server process does **not** terminate the Lifecycle nodes, and lifecycle tooling remains functional against the surviving domain.
