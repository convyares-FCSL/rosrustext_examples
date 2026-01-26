# Lesson 09 (Python) — Composition & Containers

## Goal

Demonstrate that **deployment topology alone can change system behaviour**, even when node logic, interfaces, and semantics remain unchanged.

Lesson 09 does **not** teach a composition API.
It shows what happens when existing nodes are co-located in a single process under shared execution and shared fate.

> Assumes completion of Lessons 06–08.

---

## What’s New in Lesson 09

Compared to Lesson 08:

* **No node logic changes**
* No interface changes (topics, actions, lifecycle services)
* No new semantics
* Only the **deployment topology** changes

Existing nodes from Lessons 06–08 are now:

* instantiated in a single Python process
* executed under a container-owned executor
* subject to shared shutdown and scheduling fate

---

## Tooling Reality Check (Important)

Canonical ROS 2 composition tooling exists:

* `ros2 component`
* `ros2 component types`

However, in this environment:

* `ros2 component types` exposes **only C++ (`rclcpp`) components**
* Python (`rclpy`) nodes do **not** appear
* Python nodes cannot be dynamically loaded or unloaded via `ros2 component load`

This is **not** worked around in this lesson.
The limitation is documented explicitly.

---

## Build

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_09_composition
source install/setup.bash
```

---

## Run — Composed Deployment

```bash
ros2 run lesson_09_composition lesson_09_composed_deployment
```

This launches multiple existing nodes from Lessons 06–08 in **one Python process**.

---

## Drive Lifecycle

In another terminal:

```bash
ros2 lifecycle set /lesson_06_lifecycle_publisher configure
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure
ros2 lifecycle set /lesson_08_action_server configure

ros2 lifecycle set /lesson_06_lifecycle_publisher activate
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate
ros2 lifecycle set /lesson_08_action_server activate
```

Lifecycle tooling behaves the same as in earlier lessons.

---

## Observe Actions Under Shared Fate

Start an action goal:

```bash
ros2 action send_goal /tutorial/fibonacci lesson_interfaces/action/Fibonacci "{order: 50}"
```

While the action is executing:

* Telemetry continues
* Lifecycle services respond
* The action server remains correct

Now terminate the composed process (Ctrl-C).

---

## What You Should Observe

After termination:

* **All nodes disappear together**
* The action server vanishes mid-goal
* The client does **not** receive a clean result or abort

If the CLI action client process remains running, the action may briefly appear visible in discovery.
Once the client exits, the graph converges to empty.

This behaviour did **not** occur when nodes were deployed as separate processes.

---

## Observed Failure Mode

This failure is intentional.

* The action server is correct
* The lifecycle implementation is correct
* The executor is correct

The failure arises **only** because deployment topology changed.

Lesson 09 does not fix this.
It exposes it.

---

## Further Reading / Ecosystem Context

Python composition has been explored and requested for years:

* Historical experiment (non-canonical):

  * [https://github.com/crystaldust/rclpy_composition_example](https://github.com/crystaldust/rclpy_composition_example)
* Open feature request in core ROS 2:

  * [https://github.com/ros2/rclpy/issues/575](https://github.com/ros2/rclpy/issues/575)

These are included as context, not solutions.
Lesson 09 intentionally uses only canonical, observable ROS behaviour.
