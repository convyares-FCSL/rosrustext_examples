# THEORY — Lesson 07 (Actions)

## Lesson 07 — Actions, Long-Running Work, and Node Responsiveness

Lesson 07 introduces **ROS 2 Actions** and deliberately demonstrates a common failure mode:

> **Long-running synchronous work blocks a node, even when using Actions.**

This lesson is intentionally **not production-correct**.
It is designed to expose behavior that many engineers only discover in the field.

Lesson 08 will fix the problems shown here **without changing the business logic**.

---

## What an Action Is (Conceptually)

A ROS 2 **Action** is a higher-level interaction pattern than a Topic or Service.

An Action supports:

* **Goals** — requests to perform work
* **Feedback** — incremental progress updates
* **Results** — final outcome
* **Cancellation** — cooperative stop requests

Actions are designed for **long-running operations** such as:

* path planning
* navigation
* calibration routines
* batch processing

---

## Action Server vs Action Client

### Action Server

An **Action Server**:

* Accepts goals
* Performs the requested work
* Publishes feedback
* Produces a result
* Optionally responds to cancellation

In this lesson:

* The Action Server computes a **Fibonacci sequence**
* It runs inside a **Lifecycle-managed node**
* Its execution callback is **blocking**

---

### Action Client

An **Action Client**:

* Sends goals
* Receives feedback
* Waits for results
* Requests cancellation

In this lesson:

* The client is a **standalone demo node**
* It sends two goals:

  1. A short goal that succeeds
  2. A longer goal that is canceled mid-execution

---

## How Actions Actually Run

Here is the critical point that Lesson 07 teaches:

> **Actions do not create threads. Executors do.**

An Action Server is implemented as:

* a goal service
* a cancel service
* a result future
* a user-defined execute callback

All of these are processed by the node’s **executor**.

---

## The Execution Model in Lesson 07

In this lesson, the node is deliberately configured with:

```
SingleThreadedExecutor
```

That means:

* Only **one callback** runs at a time
* No preemption
* No concurrency
* No fairness guarantees

---

## Architecture Used in This Lesson

```
+----------------------------------------------------+
| lesson_07_action_server (Lifecycle Node)           |
|                                                    |
|  +----------------+    +-------------------------+ |
|  | Telemetry      |    | Fibonacci Action Server | |
|  | Publisher      |    | (blocking execute)      | |
|  +--------+-------+    +------------+------------+ |
|           |                         |              |
|     Timer callback             Action callback     |
|           |                         |              |
|           +-----------[ SAME THREAD ]--------------+
|
|  Executor: SingleThreadedExecutor  ❌ (by design)
+----------------------------------------------------+
```

Everything — telemetry, lifecycle services, action execution — shares **one thread**.

---

## Walking Through the Fibonacci Action (Step by Step)

The Fibonacci Action Server implements the following behavior:

1. A goal arrives requesting `order = N`
2. The server enters the **execute callback**
3. For each step:

   * Sleep for 1 second (simulated heavy work)
   * Compute the next Fibonacci number
   * Publish feedback
4. When complete:

   * Return the full sequence as the result

This loop is **synchronous and blocking**.

That choice is deliberate.

---

## Why This Is a Bad Implementation (On Purpose)

This Fibonacci implementation violates best practices in several ways:

* Uses `time.sleep()` inside a callback
* Blocks the executor thread
* Performs work directly in the Action Server
* Prevents other callbacks from running

**We do this on purpose** to make the failure mode obvious.

---

## What Breaks as a Result

### 1. Telemetry Stalls

While the Fibonacci action is executing:

* Telemetry timers do not fire
* The subscriber sees gaps

Why:

* The timer callback cannot run
* The executor is blocked by the action

---

### 2. Lifecycle Services Become Unresponsive

While the action is running:

```bash
ros2 lifecycle get /lesson_07_action_server
```

may:

* hang
* respond late
* appear broken

Why:

* Lifecycle services are callbacks
* They cannot be serviced while the executor is blocked

---

### 3. Cancellation Is Delayed or Ineffective

Cancellation in ROS 2 is **cooperative**:

* The cancel request is queued
* The action callback must check for it
* That check only happens between loop iterations

Because the callback is blocking:

* Cancel requests are delayed
* The action may still complete successfully

This is expected behavior in this lesson.

---

## Why This Matters in Real Systems

This pattern appears frequently in production systems:

* calibration routines
* startup checks
* large computations
* blocking I/O

Symptoms often misdiagnosed as:

* DDS problems
* lifecycle bugs
* “ROS being slow”
* unreliable cancel behavior

The real cause is almost always **executor starvation**.

---

## The Key Teaching Points of Lesson 07

Lesson 07 intentionally teaches **multiple things at once**:

1. What an Action Server is
2. What an Action Client is
3. How Actions are scheduled
4. Why long callbacks are dangerous
5. How lifecycle responsiveness depends on executors
6. Why cancellation is cooperative, not preemptive

All of these are exposed with **one bad Fibonacci example**.

---

## Boundary to Lesson 08

Lesson 07:

* Demonstrates the failure
* Keeps logic simple
* Uses a single thread
* Makes the problem visible

Lesson 08:

* Changes executor semantics
* Introduces callback groups
* Fixes responsiveness
* Keeps the same Fibonacci logic

---

## Mental Model to Leave With

> **Actions describe *what* should happen.
> Executors determine *when* it can happen.**

Confusing the two leads to fragile systems.