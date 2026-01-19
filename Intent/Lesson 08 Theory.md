# THEORY — Lesson 08 (Executors & Callback Groups)

## Lesson 08 — Executors, Callback Groups, and Staying Alive

Lesson 07 showed us a node having a cardiac arrest.

Everything was *correct*:

* lifecycle-compliant
* standard actions
* valid math
* clean structure

And yet:

* telemetry stopped
* lifecycle services timed out
* cancellation failed

Lesson 08 explains **why that happened**, and how ROS 2 gives us the tools to prevent it.

---

## First: What an Executor Is

An **executor** is the component that decides:

* **which callback runs**
* **when it runs**
* **whether it may run concurrently with others**

Topics, services, actions, timers —
none of them run “by themselves”.

They are all **scheduled by the executor**.

If you misunderstand the executor, you misunderstand ROS 2.

---

## SingleThreadedExecutor

### What It Is

A `SingleThreadedExecutor` guarantees:

* only **one callback runs at a time**
* callbacks run **to completion**
* no preemption
* no concurrency

This is the **simplest** executor.

---

### When SingleThreadedExecutor Is the Right Choice

Single-threaded execution is correct when:

* callbacks are short
* work is lightweight
* determinism matters more than availability
* you want the simplest mental model

Examples:

* simple sensors
* parameter-only nodes
* configuration loaders
* one-shot utilities

Lesson 06 used this correctly.

---

### When It Fails (Lesson 07)

Single-threaded execution **fails catastrophically** when:

* a callback blocks
* work takes time
* actions perform real computation
* I/O is involved

In Lesson 07:

* the Action execute callback blocked
* the executor had no alternative work path
* the node stopped responding

This is not a bug.
It is the contract of `SingleThreadedExecutor`.

---

## MultiThreadedExecutor

### What It Is

A `MultiThreadedExecutor` allows:

* multiple callbacks to run concurrently
* progress on independent execution paths
* responsiveness under load

This does **not** make your code faster.

It makes your node **survivable**.

---

### When MultiThreadedExecutor Is Required

You should consider a multi-threaded executor when:

* the node hosts Actions
* callbacks may block
* timers must stay periodic
* lifecycle services must always respond
* cancellation must be timely

Lesson 08 crosses this threshold.

---

## Important: MultiThreadedExecutor Is Not Enough

A common mistake is thinking:

> “I switched to MultiThreadedExecutor — I’m done.”

You are not.

Without structure:

* callbacks still compete
* long-running work can still dominate
* starvation bugs become nondeterministic

This is why **callback groups exist**.

---

## Callback Groups: What They Are

A **callback group** defines **which callbacks may execute concurrently**.

Every callback belongs to **exactly one group**.

The executor uses groups to decide:

* which callbacks can run together
* which must be serialized

Callback groups are **execution contracts**, not code organization.

---

## Callback Group Types

ROS 2 provides two primary types:

### MutuallyExclusiveCallbackGroup

Guarantees:

* only **one callback from this group** runs at a time
* callbacks in this group will **not overlap**

Use when:

* callbacks share state
* reentrancy would cause races
* correctness requires serialization

This is the **default behavior**.

---

### ReentrantCallbackGroup

Allows:

* multiple callbacks from the same group to run concurrently
* reentry into callbacks

Use when:

* callbacks are thread-safe
* blocking is expected
* you want concurrency

This is **powerful and dangerous** if misused.

---

## Reentrancy: What It Actually Means

Reentrancy means:

> A callback may be entered again
> before a previous invocation has completed.

This is not about recursion.
It is about **concurrent execution**.

If a callback:

* touches shared state
* mutates members
* assumes sequential execution

then reentrancy **will break it**.

Lesson 08 uses reentrancy **surgically**, not casually.

---

## How Lesson 08 Uses Callback Groups

In Lesson 08, callbacks are separated by **responsibility**:

### Lifecycle Services

* Must always respond
* Must never be blocked
* Assigned to their own group

### Telemetry Timers

* Must remain periodic
* Must not be starved by work
* Assigned to their own group

### Action Execution

* Allowed to be long-running
* Allowed to block
* Assigned to a separate group

This creates **fault isolation inside the node**.

---

## The Architecture (Lesson 08)

```
+----------------------------------------------------+
| lesson_07_action_server (Lifecycle Node)           |
|                                                    |
|  +----------------+    +-------------------------+ |
|  | Telemetry      |    | Fibonacci Action Server | |
|  | Publisher      |    | (long-running)          | |
|  +--------+-------+    +------------+------------+ |
|           |                         |              |
|   Timer Callback Group        Action Callback Group |
|           |                         |              |
|    Lifecycle Callback Group (always responsive)    |
|                                                    |
| Executor: MultiThreadedExecutor                    |
+----------------------------------------------------+
```

Same node.
Same logic.
Different execution guarantees.

---

## What Changed Compared to Lesson 07

Nothing changed in **what** the node does.

Everything changed in **how it is allowed to run**.

As a result:

* telemetry continues during actions
* lifecycle services remain responsive
* cancellation becomes effective

The node regains a pulse.

---

## Typical Usage Patterns (Mental Checklist)

Use **SingleThreadedExecutor** when:

* callbacks are short
* work is bounded
* simplicity matters

Use **MultiThreadedExecutor** when:

* hosting actions
* performing blocking work
* availability matters

Use **MutuallyExclusive groups** when:

* state is shared
* correctness > concurrency

Use **Reentrant groups** when:

* work blocks
* logic is thread-safe
* isolation is explicit

---

## What This Lesson Is Teaching (Explicitly)

Lesson 08 teaches that:

1. Executors are **availability mechanisms**
2. Callback groups are **isolation boundaries**
3. Reentrancy is a **contract**, not a feature toggle
4. Lifecycle only works if management paths are protected

This is not ROS trivia.
This is **production systems engineering**.

---

## Boundary to Lesson 09

Lesson 08 fixes **internal starvation**.

Lesson 09 will introduce:

* shared executors
* shared processes
* composable nodes

Lesson 08 ensures failures in Lesson 09 are about **deployment**, not sloppy execution.

---

## Mental Model to Keep

> **Executors decide who runs.
> Callback groups decide who must never be blocked.
> Lifecycle decides when work is allowed.**

If those three are aligned, your node survives.

If not, it will appear correct — right up until it stops responding.
