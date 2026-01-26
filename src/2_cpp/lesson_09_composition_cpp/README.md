# Lesson 09: Composition (C++)

This lesson demonstrates how nodes developed in previous lessons (06 and 08) behave when they are **composed into a single process** using ROS 2 component containers.

The nodes themselves are unchanged.  
Only the **deployment topology** changes.

---

## Overview

In earlier lessons, each node ran in its own executable.
That provided implicit isolation between scheduling, lifetime, and shutdown.

In this lesson, the same nodes are loaded as **components** into a single container.
They now:

- run in the same process
- share an executor
- share a shutdown boundary

The goal of this lesson is to observe how **operability degrades under co-location**, even while the system remains alive, correct, and observable.

---

## Components

The system is composed entirely from existing nodes:

1. **LifecyclePublisherNode** (`lesson_06_lifecycle_cpp`)
2. **LifecycleSubscriberNode** (`lesson_06_lifecycle_cpp`)
3. **ActionServerNode** (`lesson_08_executors_cpp`)

No node logic is modified for this lesson.

---

## Deployment Shape

A standard ROS 2 multi-threaded component container is used:

- `component_container_mt`
- single process
- shared executor

This container defines the deployment boundary.

---

## Canonical Tooling

After composition, the system remains fully observable and controllable using standard ROS 2 tools:

- `ros2 component load / list / unload`
- `ros2 node list`
- `ros2 lifecycle get / set`
- `ros2 action list / send_goal / cancel`
- `ros2 topic echo`

No lesson-specific tooling is introduced.

---

## Usage

### 1. Start the Component Container

Start a standard ROS 2 multi-threaded component container:

```bash
ros2 run rclcpp_components component_container_mt
````

The container owns the executor and the lifetime of all loaded nodes.

---

### 2. Load Components

In a separate terminal, load the nodes into the running container.

**Load the lifecycle publisher:**

```bash
ros2 component load /ComponentManager lesson_06_lifecycle_cpp lesson_06_lifecycle_cpp::LifecyclePublisherNode
```

**Load the lifecycle subscriber:**

```bash
ros2 component load /ComponentManager lesson_06_lifecycle_cpp lesson_06_lifecycle_cpp::LifecycleSubscriberNode
```

**Load the action server:**

```bash
ros2 component load /ComponentManager lesson_08_executors_cpp lesson_08_executors_cpp::ActionServerNode
```

---

### 3. Verify the Composed System

Confirm that the components are running in the container:

```bash
ros2 component list
```

Expected output:

```text
/ComponentManager
  1  /lesson_06_lifecycle_publisher
  2  /lesson_06_lifecycle_subscriber
  3  /lesson_08_action_server
```

All nodes should also be visible via:

```bash
ros2 node list
```

---

### 4. Observe Operability Under Composition

This section demonstrates **deployment-induced degradation while the system remains running**.

#### 4.1 Activate the Lifecycle Nodes

Configure and activate both lifecycle nodes:

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_publisher activate

ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
```

Verify their state:

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
ros2 lifecycle get /lesson_06_lifecycle_subscriber
```

---

#### 4.2 Start a Long-Running Action

Start a long-running Fibonacci action goal:

```bash
ros2 action send_goal /tutorial/fibonacci lesson_interfaces/action/Fibonacci "{order: 50}"
```

Leave the action running.

---

#### 4.3 Interact While the System Is Alive

While the action is executing, repeatedly perform the following in parallel:

**Lifecycle interaction**

```bash
ros2 lifecycle get /lesson_06_lifecycle_publisher
```

or attempt a transition:

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate
```

**Telemetry observation**

```bash
ros2 topic echo /telemetry
```

**Action cancellation**

```bash
ros2 action cancel /tutorial/fibonacci
```

---

#### 4.4 Expected Observations

While the container remains running and all nodes are visible:

* lifecycle queries or transitions may respond slowly or block
* telemetry may stall, jitter, or update irregularly
* action cancellation may be delayed or unresponsive
* one node’s work interferes with another’s responsiveness

No node crashes.
No logic changes occur.
The container remains alive.

**Operability degrades anyway.**

This behavior does not appear when the same nodes are run in isolation.
It emerges only when they share a process, executor, and lifetime.

---

### 5. Observe Shared Fate (Confirmation)

Terminate the container process (Ctrl+C in the first terminal).

**Observation:**

* all nodes exit together
* any in-flight action is interrupted
* lifecycle state disappears simultaneously

This confirms that all nodes share a single failure domain when composed.

---

## Notes

This lesson intentionally introduces no fixes or mitigations.

It demonstrates how **deployment topology alone** can change system behavior.
The exposed problem is left unresolved and becomes the input to Lesson 10.