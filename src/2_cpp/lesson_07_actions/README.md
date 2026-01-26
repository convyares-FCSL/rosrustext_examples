# Lesson 07 (C++) – Actions: Long-Running Work Under Lifecycle

## Goal

Extend the **Lifecycle-managed Publisher** from Lesson 06 with an **Action Server** that performs long-running work, and observe how *functional correctness* does **not** guarantee *operational availability*.

By the end of this lesson, the system:

* Remains lifecycle-managed (`Unconfigured → Inactive → Active`).
* Publishes telemetry **only while Active**.
* Exposes a **Fibonacci Action Server** for long-running work.
* Accepts goals, streams feedback, supports cancellation.
* Exhibits **intentional starvation** under naïve execution:

  * telemetry pauses,
  * lifecycle services become sluggish,
  * cancellation latency increases.

> Assumes completion of Lesson 06.

---

## Build (Terminal 1)

**Role:** Build + orchestration commands.

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_07_actions
source install/setup.bash
```

---

## Run – Action Server (Terminal 2)

**Role:** Runs the **Lifecycle-managed Action Server**.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_07_actions lesson_07_actions_server --ros-args \
  --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml \
  --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
```

**Observation:**
The node starts silently in `Unconfigured` state:

```
Node initialized (Unconfigured). Waiting for manager...
```

No publishers, no action server resources exist yet.

---

## Orchestrating the System (Back to Terminal 1)

### 1. Check Initial State

```bash
ros2 lifecycle get /lesson_07_action_server
# unconfigured
```

---

### 2. Configure (Create Resources)

```bash
ros2 lifecycle set /lesson_07_action_server configure
```

**What happens:**

* Parameters are declared.
* Telemetry publisher is created (inactive).
* Action server is created (but gated).
* Timer exists but is paused.

---

### 3. Activate

```bash
ros2 lifecycle set /lesson_07_action_server activate
```

**What happens:**

* Telemetry starts publishing.
* Action server begins accepting goals.
* System is now operational.

---

## Run – Action Client (Terminal 3)

**Role:** Drive the system with realistic action requests.

```bash
cd ~/ros2_ws_tutorial
source install/setup.bash

ros2 run lesson_07_actions lesson_07_actions_client
```

The client performs two cases automatically:

---

### Case 1: Normal Success

* Sends a Fibonacci goal (`order=5`)
* Receives streamed feedback
* Receives final result

Expected output (approximate):

```
Goal accepted
Feedback: [0]
Feedback: [0, 1]
Feedback: [0, 1, 1]
...
Result: [0, 1, 1, 2, 3]
```

Telemetry continues publishing normally.

---

### Case 2: Cancellation + Starvation

* Sends a longer Fibonacci goal (`order=10`)
* While it runs:

  * telemetry pauses,
  * lifecycle services become slow or temporarily unresponsive
* Client requests cancellation
* Action eventually reports `CANCELED`

**Important:**
Nothing is *wrong* with the action server.
It is **correct**, **valid**, and **production-typical** — yet the node becomes operationally degraded.

This is the pressure.

---

## What You Should Observe

While the long action is executing:

* `ros2 topic echo /tutorial/telemetry` pauses.
* `ros2 lifecycle get /lesson_07_action_server` may stall.
* Cancellation is accepted, but not instantly.

After the action completes or is canceled:

* Telemetry resumes.
* Lifecycle services respond again.

No crashes. No exceptions.
Just degraded availability.

---

## Automated Testing

This package includes:

* **Unit Tests (C++)**

  * Fibonacci generation
  * Long-running routine correctness
* **Integration Tests (Python)**

  * Lifecycle transitions
  * Action success
  * Action cancellation
  * Explicit starvation probe

Run all tests:

```bash
colcon test --packages-select lesson_07_actions \
  --return-code-on-test-failure \
  --event-handlers console_direct+
```

---

## Architecture Notes

* **Lifecycle still owns “when work is allowed”**

  * Action server accepts goals only while Active.
* **Action server is production-grade**

  * correct goal / feedback / cancel / result semantics,
  * no artificial mistakes.
* **Business logic is isolated**

  * Fibonacci routine lives outside ROS code.
* **Starvation is an execution artifact**

  * caused by single-threaded executor + blocking work,
  * not by misuse of Actions or Lifecycle.

---

## What This Lesson Proves

1. **Correctness ≠ Availability**
2. **Actions introduce time as a first-class concern**
3. **Lifecycle alone does not prevent starvation**
4. **A “perfectly valid” node can still fail operationally**

This failure is intentional.

Lesson 08 will fix it **without changing the action or business logic**, by addressing **execution and scheduling** instead.