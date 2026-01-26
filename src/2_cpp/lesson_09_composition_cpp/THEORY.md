# THEORY — Lesson 09 (Composition)

## Lesson 09 — Composition, Shared Fate, and Operability Degradation

Lesson 09 introduces **ROS 2 Composition** and deliberately demonstrates a different class of failure than Lesson 07.

In Lesson 07, the system failed because **work monopolized an executor**.

In Lesson 09, the system degrades even though:

- executors are correctly configured
- callback groups are correctly isolated
- nodes are internally correct

The failure in this lesson is **not algorithmic**.

It is **deployment-induced**.

---

## What Composition Actually Does

ROS 2 composition allows multiple nodes to be loaded into a **single process**.

When nodes are composed:

- they share a process
- they share an executor
- they share a shutdown boundary

No node semantics change.
No APIs change.
No behavior inside a node is rewritten.

Only **where the nodes run** changes.

---

## The Assumption This Lesson Breaks

Many systems implicitly assume:

> “If each node is correct, the system will remain correct.”

Lesson 09 demonstrates that this assumption is false.

Correct nodes can still interfere with each other when:

- scheduling domains collapse
- lifetime domains collapse
- failure domains collapse

---

## Composition vs Isolation

In earlier lessons, each node ran in its own process.

That provided:

- independent executors
- independent scheduling
- independent shutdown
- implicit fault isolation

Composition removes those boundaries.

---

## The Execution Model Under Composition

When composed, all nodes are hosted inside a **single component container**.

That container:

- owns the executor
- schedules all callbacks
- controls lifetime and shutdown

Conceptually:

```

+------------------------------------------------------+
| Component Container (Single Process)                 |
|                                                      |
|  +-------------------+   +------------------------+ |
|  | Lifecycle Pub     |   | Lifecycle Sub          | |
|  | (Timers, Services)|   | (Subscriptions)       | |
|  +---------+---------+   +-----------+------------+ |
|            |                         |              |
|            |      +------------------v------------+ |
|            |      | Fibonacci Action Server        | |
|            |      | (long-running work)            | |
|            |      +------------------+-------------+ |
|            |                         |              |
|            +-----------[ SHARED EXECUTOR ]----------+
|
|  Executor: MultiThreadedExecutor (shared)
+------------------------------------------------------+

```

Each node is internally well-behaved.

The problem is **co-location**.

---

## What Changes Compared to Lesson 08

Lesson 08 fixed executor starvation by:

- introducing callback groups
- using a multi-threaded executor
- isolating long-running work

Those fixes **still apply here**.

Lesson 09 does **not** remove them.

Despite that:

- responsiveness degrades
- interactions interfere
- latency increases

Why?

Because **isolation inside a node is not isolation between nodes**.

---

## Operability Degradation (While Alive)

In Lesson 09, the container remains running.

Nodes remain visible via:

- `ros2 node list`
- `ros2 lifecycle get`
- `ros2 action list`

And yet:

- lifecycle transitions may lag
- telemetry may stall or jitter
- action cancellation may become slow
- one node’s work interferes with another’s responsiveness

Nothing crashes.
Nothing deadlocks.
Nothing is “broken”.

**Operability degrades anyway.**

---

## Why This Happens

The cause is not a bug.

It is the collapse of **deployment boundaries**.

Under composition:

- all callbacks compete for shared executor resources
- long-running work increases scheduling pressure
- service calls, timers, and actions are no longer isolated by process boundaries
- shutdown and lifetime become coupled

The system remains correct.
The system becomes fragile.

---

## Shared Fate (Secondary Effect)

Composition also introduces **process-level shared fate**.

When the container exits:

- all nodes exit
- all in-flight work is interrupted
- lifecycle state disappears together

This confirms the failure domain.

But this is **not the primary lesson**.

The primary lesson is that **things degrade before anything crashes**.

---

## Why This Matters

In real systems:

- calibration runs
- startup checks
- batch actions
- diagnostics

often coexist with telemetry and lifecycle control.

Under composition, these concerns interfere even when each node is “correct”.

This is why systems fail operationally without obvious bugs.

---

## What This Lesson Does Not Do

Lesson 09 does **not**:

- refactor nodes
- fix interference
- introduce orchestration
- introduce launch logic
- add safety guarantees

Any fix here would teach the wrong lesson.

---

## Boundary to Lesson 10

Lesson 09 asks:

> “What breaks when we deploy this for real?”

Lesson 10 answers:

> “How do we manage deployment topology so those failures are contained?”

Lesson 09 **creates the pressure**.
Lesson 10 **operationalises the response**.

---

## Mental Model to Leave With

> **Correct nodes are necessary.  
> Correct deployment is mandatory.  
> Composition tells the truth.**