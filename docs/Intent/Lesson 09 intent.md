## Lesson 09 Intent: Composition & Containers

### *Deployment as a Truth Serum*

---

## Lesson 09 – When Correct Nodes Fail Together

**Theme:**
Your nodes are correct.
Your deployment is not.

Lesson 08 proved that **internal concurrency** can be made survivable without changing logic.
Lesson 09 proves a harder truth:

> Correct nodes can still fail when **co-located**.

Nothing breaks because of bugs.
Things break because of **shared fate**.

---

## Context (What the Student Already Knows)

By the end of Lesson 08, the student has:

* Lifecycle-managed nodes that behave correctly
* Long-running actions that do not starve the node
* Explicit callback group isolation
* A working mental model of:

  * executors as schedulers
  * callback groups as isolation boundaries

Each node, **in isolation**, is operationally healthy.

---

## The New Question Lesson 09 Asks

> What happens when **multiple healthy nodes share a process, an executor, and a shutdown boundary**?

This is not a new algorithmic problem.
It is a **deployment topology problem**.

---

## Lesson 09 Goal

Expose **deployment-induced failure modes** by changing **only how nodes are hosted**, not how they behave.

Specifically:

* Nodes that worked perfectly as standalone processes
* Are now loaded into a **single composable container**
* With **shared executor ownership**
* And **shared lifecycle and shutdown fate**

No node logic changes.
No fixes are applied.

---

## What This Lesson Introduces

Lesson 09 introduces **composition as an operational primitive**, not a convenience feature.

### New primitives (deployment-level only):

1. **Component Containers**
2. **Runtime Node Loading / Unloading**
3. **Shared Executor Ownership**
4. **Process-Level Failure Domains**

These are not abstractions.
They are **constraints**.

---

## Artifact Produced

### A Composable System

A single **Component Container process** that dynamically hosts:

* Telemetry publisher (lifecycle-managed)
* Verifier / subscriber
* Service server
* Action server (long-running work)

All nodes are **previous lesson artifacts**, unmodified.

The container:

* Owns the executor
* Owns node lifetime
* Exposes standard ROS 2 composition services
* Is observable via standard CLI tools

---

## What Must Be Demonstrated

Lesson 09 is complete only when **all of the following are true**:

### 1. Tooling Parity Exists

* `ros2 component list` shows loaded nodes
* `ros2 component load` loads existing lesson nodes
* `ros2 component unload` removes them cleanly
* `ros2 lifecycle get/set` works on composed lifecycle nodes

No custom tools.
No proxy commands.
No lesson-only hacks.

---

### 2. Intentional Failure Is Observable

At least one of the following **must fail or degrade** under composition:

* Callback group isolation assumptions break
* One node’s work interferes with another’s responsiveness
* Lifecycle transitions are delayed or reordered
* Shutdown order causes observable misbehavior

This failure is **not fixed** in Lesson 09.

It is named.
Observed.
Explained.

---

### 3. Failure Is Deployment-Induced

The student must be able to reason:

> “This node worked before.
> It still works in isolation.
> It fails because of *where* it runs.”

If the failure can be blamed on:

* bad Fibonacci math
* missing locks
* sloppy callback groups inside a node

Then the lesson has failed.

---

### 4. Wins Still Exist

Despite failure modes:

* Nodes still perform their core work
* Behavior matches standalone execution **when not interfering**
* The system remains introspectable
* No undefined or “mystery” behavior is introduced

Lesson 09 is not chaos.
It is **controlled exposure**.

---

## Core Teaching Points

### 1. Composition Is Not an Optimization

Composition is not about:

* reducing processes
* saving memory
* improving performance

It is about **changing failure domains**.

---

### 2. Executors Define Shared Fate

Lesson 08 taught:

> Executors decide who gets to breathe.

Lesson 09 extends this:

> **Shared executors decide who suffocates together.**

Isolation inside a node is insufficient
when nodes share a scheduler.

---

### 3. Deployment Reveals Architectural Honesty

Composition does not change semantics.

It reveals:

* hidden coupling
* unexamined assumptions
* accidental dependencies

---

## What This Lesson Is *Not*

Lesson 09 is **not**:

* a refactor of nodes
* a concurrency fix
* a performance lesson
* a launch tutorial
* a workaround for missing tools

Any “fix” belongs to a **future lesson or capstone**, not here.

---

## Boundary Conditions (Architectural Rules)

Lesson 09 must obey:

* No changes to node logic
* No changes to public interfaces
* No new semantic behavior
* No tutorial glue inside nodes
* Only canonical ROS 2 interfaces

If composition parity is missing, that is a **library or infrastructure gap**, not a lesson excuse.

---

## Relationship to Lesson 10

Lesson 09 answers:

> “What breaks when we deploy this for real?”

Lesson 10 answers:

> “How do we bring this up, verify it, and shut it down safely every time?”

Lesson 09 **creates the problem space**.
Lesson 10 **operationalizes it**.

---

## Mental Model the Student Must Leave With

> **Correct nodes are necessary.
> Correct deployment is mandatory.
> Composition tells the truth.**



## Why This Lesson Does Not Fix the Failure

Lesson 09 deliberately ends with an unresolved problem.

The failure exposed here is **not a bug in the nodes** and **not a flaw in composition mechanics**.

It is a consequence of:

* shared executors
* shared process lifetimes
* shared shutdown domains

Composition provides **co-location**, not **isolation**.

No amount of callback group tuning, executor configuration, or container logic
can change the fact that all composed nodes share the same failure domain.

Attempting to “fix” this inside Lesson 09 would teach the wrong lesson:
that composition is a safety mechanism.

It is not.
