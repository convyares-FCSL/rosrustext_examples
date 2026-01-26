## Lesson 08 Intent: Executors & Callback Groups

### *Concurrency as Availability, Not Optimization*

---

## Lesson 08 – Restoring the Pulse

**Theme:** Concurrency is not about speed.
It is about **keeping the node alive**.

Lesson 07 proved a hard truth:
a node can be *functionally correct* and still be *operationally dead*.

Lesson 08 exists to fix that — **without changing the work**.

---

## Context (What the Student Already Knows)

By the end of Lesson 07, the student has seen:

* A lifecycle-managed node that is architecturally correct
* A valid Action Server and Action Client
* Correct Fibonacci math
* Correct feedback and result handling

And yet:

* Telemetry stalls
* Lifecycle services time out
* `/get_state` becomes unresponsive
* Cancellation is delayed or ineffective

Nothing is “wrong” with the logic.
The node is dying because **work is monopolizing the executor**.

---

## Lesson 08 Goal

Restore **responsiveness and survivability** to the Lesson 07 node by changing **how callbacks are scheduled**, not **what they do**.

The Fibonacci Action remains long-running.
The lifecycle model remains unchanged.
The business logic remains untouched.

Only **execution strategy** changes.

---

## What This Lesson Introduces

Lesson 08 introduces two core ROS 2 primitives as **operational tools**:

1. **Executors**
2. **Callback Groups**

Not as performance tricks.
As **availability guarantees**.

---

## Core Teaching Points

### 1. Executors Are the Scheduler of Reality

Lesson 07 used:

* `SingleThreadedExecutor`

Lesson 08 introduces:

* `MultiThreadedExecutor`

The student must understand:

> Executors decide *who gets to breathe* when work is happening.

This is not about “parallelism for speed”.
It is about **preventing starvation**.

---

### 2. Callback Groups Are Isolation Boundaries

Lesson 08 introduces **explicit callback group assignment**:

* Lifecycle services in one group
* Telemetry timers in another
* Action execution in a third

This teaches:

* Callback groups are **fault-containment units**
* Poor grouping recreates Lesson 07 failure
* Correct grouping restores availability

---

### 3. Long-Running Work Is Allowed — If Isolated

Lesson 08 does **not**:

* rewrite Fibonacci
* remove blocking sleep
* add async tricks
* offload to threads manually

Instead, it proves:

> Long-running work is acceptable **if it cannot starve the system**.

This is the crucial professional insight.

---

## Artifact Produced

### Modified Managed Node (Same Responsibilities)

The node still:

* Is lifecycle-managed
* Publishes telemetry
* Hosts an Action Server

But now:

* Uses a `MultiThreadedExecutor`
* Assigns callback groups deliberately
* Remains responsive under load

---

## What Must Be Demonstrated

Lesson 08 is complete only when **all of the following are true**:

1. **Action executes fully**

   * Fibonacci runs as before
   * Feedback and result unchanged

2. **Telemetry never stalls**

   * Subscriber sees continuous heartbeat

3. **Lifecycle services remain responsive**

   * `/get_state` always replies
   * `/deactivate` works during action execution

4. **Cancellation becomes effective**

   * Cancel request is serviced promptly
   * Action terminates early when canceled

Nothing else changes.

---

## What This Lesson Is *Not*

Lesson 08 is **not**:

* an optimization lesson
* an async programming tutorial
* a “make it faster” exercise
* a rewrite of business logic

It is a **survivability lesson**.

---

## Boundary Conditions (Architectural Rules)

Lesson 08 must obey:

* No change to public interfaces
* No change to action semantics
* No change to lifecycle contract
* No introduction of custom abstractions
* No tutorial glue

The *only* axis of change is **execution strategy**.

---

## Relationship to Lesson 09

Lesson 08 produces a node that:

* survives internal load
* behaves correctly under stress
* respects lifecycle boundaries

Lesson 09 then asks a harder question:

> What happens when **multiple nodes share an executor and a process**?

Lesson 08 ensures that any failure in Lesson 09 is about **deployment**, not sloppy concurrency.

---

## Mental Model the Student Must Leave With

> **Executors determine availability.
> Callback groups determine isolation.
> Business logic determines correctness.**

Mixing these responsibilities is how systems fail quietly.

---

If you want next, the *right* next step is one of these (in order of architectural safety):

1. Draft **Lesson 08 THEORY.md** using this intent
2. Define **Lesson 08 acceptance tests** *before* code
3. Review **callback group allocation strategy** at the design level

Do **not** start coding Lesson 08 yet.
