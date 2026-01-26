
**Lesson 08 is complete when you can reliably execute long-running goals without losing telemetry, lifecycle responsiveness, or cancellation.**

---

# THEORY — Lesson 08 (Executors & Callback Groups)

## Lesson 08 — Scheduling Is Availability

Lesson 07 demonstrated that a node can be:

* lifecycle-correct
* action-capable
* logically correct

…and still become operationally unresponsive.

Lesson 08 fixes that by changing **how callbacks are scheduled**, not what the node does.

The Fibonacci action remains long-running.
The interfaces remain unchanged.
Only execution strategy changes.

---

## Executors: What They Do

An executor is the scheduler for a node’s callbacks:

* timers
* subscriptions
* services
* action goal/cancel/result handling
* action execute callbacks

Nothing in ROS 2 “runs by itself”.
Everything is executed because the executor selects it.

---

## SingleThreadedExecutor

A `SingleThreadedExecutor` guarantees:

* one callback at a time
* no preemption
* no concurrency

This is correct when callbacks are short and bounded.

Typical use cases:

* simple periodic publishers
* parameter-only nodes
* deterministic orchestration with lightweight work

Lesson 06 fits this model.

---

## Why SingleThreadedExecutor Failed in Lesson 07

In Lesson 07:

* the action execute callback was long-running
* it blocked the only executor thread

So the executor could not service:

* telemetry timers
* lifecycle services (`get_state`, `change_state`)
* action cancellation plumbing

The action math was correct.
The node was effectively dead.

---

## MultiThreadedExecutor

A `MultiThreadedExecutor` allows multiple callbacks to run concurrently across a pool of threads.

This is not primarily a “performance” feature.
It is an availability feature.

Typical use cases:

* actions and services that may block
* nodes that must remain responsive to management interfaces
* systems that require continuous telemetry under load

Lesson 08 crosses into this territory.

---

## Callback Groups: What They Are

Callback groups define concurrency rules within a node.

They answer:

* “Which callbacks are allowed to overlap?”
* “Which must remain mutually exclusive?”

They are isolation boundaries, not organisational labels.

---

## Callback Group Types

### MutuallyExclusiveCallbackGroup

Guarantees only one callback in the group runs at a time.

Use for:

* callbacks that share mutable state
* places where correctness depends on serialization

### ReentrantCallbackGroup

Allows callbacks in the group to overlap.

Use for:

* callbacks that are thread-safe
* cases where you must avoid starvation
* action servers where goal/cancel/result plumbing must progress under load

Reentrant does not make code safe.
It assumes your code already is safe (or isolated).

---

## What Lesson 08 Changes

Lesson 08 keeps the same node responsibilities:

* lifecycle-managed publisher
* long-running Fibonacci action server

But changes execution control:

* `MultiThreadedExecutor` instead of single-threaded
* explicit callback group assignment

A typical allocation used here:

* Telemetry timer callbacks → `MutuallyExclusiveCallbackGroup`
* Action server callbacks → `ReentrantCallbackGroup`
* Lifecycle services remain in the node default group (and must not be starved)

This allows:

* telemetry timers to continue running
* lifecycle services to respond promptly
* cancel requests to be processed while execution is ongoing

---

## Why Cancellation Now Works

Cancellation is still cooperative:

* the client requests cancel
* the server must observe it (via `goal_handle.is_cancel_requested`)
* the server must return accordingly

The key improvement is that with multi-threaded execution and isolation:

* the cancel request can be processed promptly by the executor
* the execute callback can observe it on its next check

You get timely cancellation without changing action semantics.

---

## The Mental Model

Lifecycle answers:

> “When is the node allowed to operate?”

Executors and callback groups answer:

> “Can the node remain responsive while it operates?”

Lesson 08 is the point where : 
availability is designed, not assumed.
