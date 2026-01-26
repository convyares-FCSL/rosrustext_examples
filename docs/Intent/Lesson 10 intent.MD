# Topic 10 Intent — Launch, Topology & Deployment Verification

## Architecture Is Where You Draw the Fault Lines

---

## Topic 10 — Choosing Where Failure Is Allowed

**Theme**

Correct nodes.  
Correct composition.  
Still unsafe — until topology is explicit.

Topic 09 exposed a hard truth:

> **Composition co-locates failure.**

Topic 10 exists to answer the next unavoidable systems question:

> **Where should failures be allowed to propagate — and where must they stop?**

This Topic introduces **no new node behaviour**.  
It introduces **intentional deployment structure**.

---

## Context (What the Engineer Already Knows)

By the end of Topic 09, the engineer has:

* Correct, lifecycle-managed nodes
* Long-running actions that are internally survivable
* Explicit callback-group isolation
* A composed system where:

  * multiple nodes share a process
  * executors are shared
  * shutdown and failure are shared

They have also *observed*:

* interference between nodes
* shutdown-order hazards
* action interruption under shared fate
* behaviour changing solely because of deployment

Nothing is broken.

The system is simply **honest**.

---

## The Question Topic 10 Answers

> **How do we deploy this system so that failure domains match engineering intent?**

This is no longer a ROS API question.  
It is an **operational architecture question**.

---

## Topic 10 Goal

Demonstrate **deployment-grade system design** by:

* Making **process boundaries explicit**
* Using **launch** to declare topology
* Proving behaviour through **deployment-level verification**

Topic 10 turns a truthful-but-fragile system  
into a **repeatably deployable one**.

---

## What This Topic Introduces

Topic 10 introduces **deployment orchestration as a first-class engineering concern**.

### New axes of control (deployment-level only)

1. **Launch files as architecture**
2. **Selective composition**
3. **Explicit process boundaries**
4. **Deployment verification scripts**

No new node logic.  
No new ROS semantics.  
No fixes hidden inside components.

---

## Artifact Produced

### An Explicitly Fault-Lined Deployment

The system is deployed via **launch**, with:

* One or more **component containers**
* One or more **standalone node processes**
* Explicit decisions about:

  * which nodes share a process
  * which nodes must be isolated
  * startup order
  * shutdown order
  * restart scope

Topology is no longer accidental.  
It is intentional, documented, and reproducible.

---

## What Changes from Topic 09

Nothing about the nodes changes.

What changes is **where they run**.

Topic 10 introduces **two intentional deployment profiles**, using the *same node artifacts*:

### Profile A — Fully Composed (Continuity from Topic 09)

* Nodes are co-located under a shared process boundary
* Shared executor, shared shutdown, shared fate

This profile preserves Topic 09’s truth and is used as a baseline.

---

### Profile B — Fault-Lined (Selective Composition + Isolation)

* Some nodes remain composed
* At least one node (the long-running action server) is isolated behind a process boundary

This profile exists solely to **draw a fault line**.

No code changes.  
Only topology changes.

---

## What Must Be Demonstrated

Topic 10 is complete only when **all** of the following are true.

---

### 1. Deterministic Bring-Up

* Launch brings the system up reliably
* Installed vs source-tree execution both work
* Configuration is discovered at runtime (no hard-coded paths)
* Parameters are injected via launch (e.g. YAML), not edited into code

Bring-up is executable, not remembered.

---

### 2. Explicit Failure Containment (Deadlock Containment)

At least one **intentional deployment-level failure** must be demonstrated:

* The action server is intentionally blocked or deadlocked
* In **Profile A**, this degrades or stalls unrelated behaviour due to shared fate
* In **Profile B**, unrelated nodes remain responsive

The engineer must be able to say:

> “This failure stopped here because of this process boundary.”

This is not about eliminating failure.  
It is about **containing it deliberately**.

---

### 3. Tooling Parity Is Preserved

Despite topology changes:

* `ros2 node list` remains meaningful
* `ros2 lifecycle get/set` continues to work
* `ros2 component list` works where composition is used
* No custom orchestration replaces standard ROS tools

Observability is preserved across deployment profiles.

---

### 4. Deployment Verification Exists

Topic 10 introduces **deployment verification scripts** that:

* launch the system
* wait for readiness
* drive lifecycle transitions
* detect failure (report, do not hide)
* enforce clean shutdown and graph convergence

This completes the verification pyramid:

> unit → integration → deployment

---

## Core Teaching Points

### 1. Launch Files Are Architecture

Launch is not a convenience wrapper.

It is where:

* topology is defined
* fault lines are drawn
* operational intent becomes executable

---

### 2. Composition Reveals Truth — Topology Restores Control

Topic 09 proved:

> Composition reveals coupling.

Topic 10 proves:

> **Topology determines survivability.**

---

### 3. Isolation Is a Deployment Choice

Isolation does **not** come from:

* callback groups
* executors
* async code

It comes from **process boundaries**.

---

## What This Topic Is *Not*

Topic 10 is **not**:

* a refactor of nodes
* a rewrite of Topic 09
* a performance-tuning exercise
* a replacement for ROS tooling
* an attempt to hide failure

It does not make the system perfect.

It makes it **operable**.

---

## Boundary Conditions

Topic 10 must obey:

* No changes to node logic
* No changes to public interfaces
* No new ROS semantics
* No tutorial glue
* Only deployment topology may change

If something cannot be solved with launch and orchestration,  
it does **not** belong here.

---

## Relationship to Benchmarks & Capstone

Topic 10 completes the instructional arc.

Everything after this point:

* benchmarks
* capstone systems
* performance analysis

is **application**, not instruction.

The engineer now owns a system that can:

* be reasoned about
* be deployed deliberately
* fail predictably

---

## Mental Model to Leave With

> **Correct code is necessary.**  
> **Correct deployment is decisive.**  
> **Architecture is where reliability lives.**
