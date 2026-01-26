# Lesson 07 Theory: Actions & Long-Running Work Under Lifecycle (C++ / rclcpp_action)

## Architectural Intent

Lesson 06 established **when** a node is allowed to exist and operate using the Lifecycle state machine.

Lesson 07 introduces a new dimension of pressure:

> **Time.**

Actions are the ROS 2 mechanism for work that:

* takes time,
* produces intermediate feedback,
* may need cancellation,
* and cannot be modeled as a simple request/response.

The intent of this lesson is **not** to teach how to write an action server.
It is to demonstrate that:

> **Lifecycle correctness does not imply operational availability once long-running work is introduced.**

---

## What Changes from Lesson 06

In Lesson 06, all callbacks were short and bounded:

* timer callbacks were fast,
* subscriber callbacks returned immediately,
* lifecycle services were always responsive.

In Lesson 07, we introduce:

* a **Lifecycle-managed Action Server**
* a **long-running action execution path**
* **feedback and cancellation semantics**

Nothing else changes.

This is deliberate.

---

## Actions as a New Pressure Axis

An Action Server introduces a callback that can legally:

* run for seconds or minutes,
* publish feedback repeatedly,
* block until completion.

From an API perspective, this is **correct** and **expected**.

From an execution perspective, this introduces a risk:

> **Long-running callbacks compete with every other callback in the node.**

This includes:

* timers,
* lifecycle services (`get_state`, `change_state`),
* parameter updates,
* other actions.

---

## The Action Server is Correct

The Action Server implemented in Lesson 07 is **production-grade**:

* proper goal acceptance / rejection,
* proper feedback publication,
* proper cancellation handling,
* deterministic result reporting,
* business logic isolated from ROS code.

There are **no shortcuts**, no artificial mistakes, no “bad example” code.

If placed into another node, it would behave correctly.

This matters.

The lesson is not “don’t do this.”
The lesson is “this alone is not enough.”

---

## Lifecycle Still Owns Permission, Not Scheduling

Lifecycle management answers the question:

> *“Is this node allowed to operate?”*

It does **not** answer:

> *“How should competing work be scheduled?”*

In Lesson 07:

* Lifecycle correctly gates:

  * telemetry publishing,
  * action acceptance.
* Lifecycle does **not** prevent:

  * starvation,
  * callback backlog,
  * degraded responsiveness.

This is not a flaw in Lifecycle.
It is simply **outside its scope**.

---

## Where the Failure Comes From

The system fails *operationally* because of one intentional choice:

* **Single-threaded executor**
* **Blocking action execution**

The long-running Fibonacci routine blocks the executor thread.

While blocked:

* telemetry timers cannot fire,
* lifecycle service callbacks wait,
* cancellation is delayed.

This is observable from the outside:

* telemetry pauses,
* `ros2 lifecycle get` becomes sluggish,
* cancel requests take time to process.

The system is still **correct**.
It is simply **unavailable**.

---

## Separation of Concerns (Reinforced)

Lesson 07 strengthens the architectural separation introduced in Lesson 06:

* **Logic**

  * Fibonacci generation and long-running routine
  * No ROS, no timing assumptions beyond what is explicit
* **Action Server Component**

  * Owns goal / feedback / cancel semantics
  * Delegates work to business logic
* **Lifecycle Node**

  * Owns resource creation and activation
  * Owns telemetry timing
  * Owns executor choice (and therefore scheduling behavior)

Each layer is correct in isolation.
The failure emerges only when they are composed.

---

## Testing Strategy (Expanded)

### Unit Tests (C++ / GTest)

* Target:

  * Fibonacci generator
  * Fibonacci routine
* Scope:

  * correctness of results
  * deterministic behavior
* No ROS graph required.

### Integration Tests (Python / launch_testing)

* Target:

  * compiled C++ action server node
* Scope:

  * lifecycle transitions,
  * action success,
  * action cancellation,
  * **observable starvation under load**.

The integration test does not inspect internals.
It treats the node as a **black box**, exactly as a real system would.

---

## Why This Lesson Exists

Lesson 07 demonstrates a critical systems truth:

> **A node can be correct, well-structured, lifecycle-managed, and still fail under real operational pressure.**

This is not a bug.
This is the natural consequence of:

* single-threaded execution,
* blocking work,
* increasing system complexity.

---

## Relationship to Lesson 08

Lesson 07 intentionally stops here.

No fixes.
No executor changes.
No callback group isolation.

Lesson 08 will address availability **without changing**:

* business logic,
* action semantics,
* lifecycle design.

The fix will come from **execution architecture**, not algorithms.

---

## Summary

Lesson 07 adds **time** as a first-class architectural concern.

* Lifecycle controls **permission**.
* Actions introduce **duration**.
* Executors determine **availability**.

The system is correct.
The system is stressed.
The system degrades.

That is the lesson.
