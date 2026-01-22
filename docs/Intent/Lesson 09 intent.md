### Overview

Yes. Below is a **full, clean Lesson 10 intent**, written to match the **style, rigor, and architectural discipline of Lessons 08 and 09**, and explicitly positioned as the *resolution* to Lesson 09 — **without retroactively fixing it**.

This is an **intent document**, not an implementation guide. It locks scope, sequencing, and teaching responsibility.

---

## Lesson 10 Intent: Launch, Topology & Deployment Verification

### *Architecture Is Where You Draw the Fault Lines*

---

## Lesson 10 – Choosing Where Failure Is Allowed

**Theme:**
Correct nodes.
Correct composition.
Still unsafe — until topology is explicit.

Lesson 09 exposed a truth:

> Composition co-locates failure.

Lesson 10 exists to answer the next, unavoidable question:

> **Where should failures be allowed to propagate — and where must they stop?**

This lesson does not introduce new node behaviour.
It introduces **intentional deployment structure**.

---

## Context (What the Student Already Knows)

By the end of Lesson 09, the student has:

* Correct, lifecycle-managed nodes
* Long-running actions that are internally survivable
* Explicit callback group isolation
* A composable container that supports:

  * runtime load / unload
  * shared executors
  * standard ROS tooling introspection

They have also seen:

* interference between nodes
* shutdown-order hazards
* shared-fate failure modes

Nothing is “wrong”.

The system is simply **honest**.

---

## The New Question Lesson 10 Asks

> How do we deploy this system so that **failure domains match intent**?

This is no longer a ROS API question.
It is an **operational architecture question**.

---

## Lesson 10 Goal

Demonstrate **production-grade deployment** by:

* Making **process boundaries explicit**
* Using **launch** to define topology
* Proving correctness through **automated verification scripts**

Lesson 10 turns a truthful-but-fragile system
into a **repeatably deployable one**.

---

## What This Lesson Introduces

Lesson 10 introduces **deployment orchestration as a first-class concern**.

### New primitives (deployment-level only):

1. **Launch files as architecture**
2. **Selective composition**
3. **Explicit process boundaries**
4. **Deployment verification scripts**

No new node logic.
No new ROS semantics.
No “fixes” inside components.

---

## Artifact Produced

### A Deployment-Ready System

The system is now deployed using **launch**, with:

* One or more **component containers**
* One or more **standalone node processes**
* Explicit decisions about:

  * which nodes share a process
  * which nodes must be isolated
  * startup order
  * shutdown order

The topology is intentional, documented, and reproducible.

---

## What Changes from Lesson 09

Nothing about the nodes changes.

What changes is **where they run**.

Examples of intentional decisions (illustrative, not prescriptive):

* Telemetry publisher + verifier share a container
* Action server runs in its own process
* Lifecycle-managed nodes are activated only after all dependencies are present

These decisions are encoded **only** in launch and scripts.

---

## What Must Be Demonstrated

Lesson 10 is complete only when **all of the following are true**:

### 1. Deterministic Bring-Up

* Launch brings the full system up reliably
* No hardcoded paths (installed vs source-tree works)
* Parameters and configuration are discovered at runtime

---

### 2. Explicit Failure Containment

* Failure or load in one node does **not** stall unrelated nodes
* Long-running actions no longer degrade system-wide responsiveness
* Shutdown of one component does not cascade unexpectedly

The student must be able to point to **process boundaries** and say:

> “That’s why this failure stopped there.”

---

### 3. Tooling Parity Is Preserved

Despite the new topology:

* `ros2 node list` remains meaningful
* `ros2 lifecycle get/set` still works
* `ros2 component list` still works where composition is used
* No custom orchestration tools replace standard ROS ones

---

### 4. Deployment Verification Is Automated

Lesson 10 introduces **verification scripts** that:

* launch the system
* wait for readiness
* perform lifecycle transitions
* detect failure
* enforce clean shutdown

This completes the verification pyramid:

> unit → integration → deployment

---

## Core Teaching Points

### 1. Launch Files Are Architecture

Launch is not a convenience wrapper.

It is where:

* topology is defined
* failure domains are chosen
* operational intent becomes executable

---

### 2. Composition Is a Tool, Not a Goal

Lesson 09 proved:

> Composition reveals truth.

Lesson 10 proves:

> Composition must be used selectively.

---

### 3. Isolation Is an Architectural Choice

Isolation does not come from:

* callback groups
* executors
* async code

It comes from **process boundaries**.

---

## What This Lesson Is *Not*

Lesson 10 is **not**:

* a refactor of nodes
* a rewrite of Lesson 09
* a performance tuning exercise
* a replacement for ROS tools
* an excuse to hide failures

It does not make the system “perfect”.

It makes it **operable**.

---

## Boundary Conditions (Architectural Rules)

Lesson 10 must obey:

* No changes to node logic
* No changes to public interfaces
* No new ROS semantics
* No tutorial glue
* Only deployment topology may change

If something cannot be solved with launch and orchestration,
it does **not** belong in Lesson 10.

---

## Relationship to Capstone & Benchmark

Lesson 10 completes the teaching arc.

Everything after this point:

* benchmarks
* capstone systems
* performance analysis

is **application**, not instruction.

The student now owns a system that can:

* be reasoned about
* be deployed safely
* fail predictably

---

## Mental Model the Student Must Leave With

> **Correct code is necessary.
> Correct deployment is decisive.
> Architecture is where reliability lives.**

---

### One next step

Confirm this Lesson 10 intent as final.
Once locked, the next step is to **define Lesson 10 deployment verification scripts** that operationalize this intent without altering any node or container code.
