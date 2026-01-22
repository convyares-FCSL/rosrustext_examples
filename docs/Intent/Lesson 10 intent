# Topic 10 Intent: Launch, Topology & Deployment Verification

## Architecture Is Where You Draw the Fault Lines

---

## Topic 10 — Choosing Where Failure Is Allowed

**Theme**
Correct nodes.
Correct composition.
Still unsafe — until topology is explicit.

Topic 09 exposed a hard truth:

> Composition co-locates failure.

Topic 10 exists to answer the next, unavoidable systems question:

> **Where should failures be allowed to propagate — and where must they stop?**

This Topic does **not** introduce new node behaviour.
It introduces **intentional deployment structure**.

---

## Context (What the Engineer Already Knows)

By the end of Topic 09, the engineer has:

* Correct, lifecycle-managed nodes
* Long‑running actions that are internally survivable
* Explicit callback‑group isolation
* A composable container that supports:

  * runtime load / unload
  * shared executors
  * standard ROS tooling introspection

They have also *observed*:

* interference between nodes
* shutdown‑order hazards
* shared‑fate failure modes

Nothing is broken.

The system is simply **honest**.

---

## The Question Topic 10 Answers

> How do we deploy this system so that **failure domains match engineering intent**?

This is no longer a ROS API question.
It is an **operational architecture question**.

---

## Topic 10 Goal

Demonstrate **deployment‑grade system design** by:

* Making **process boundaries explicit**
* Using **launch** to define topology
* Proving behaviour through **automated deployment verification**

Topic 10 turns a truthful‑but‑fragile system
into a **repeatably deployable one**.

---

## What This Topic Introduces

Topic 10 introduces **deployment orchestration as a first‑class engineering concern**.

### New axes of control (deployment‑level only)

1. **Launch files as architecture**
2. **Selective composition**
3. **Explicit process boundaries**
4. **Deployment verification scripts**

No new node logic.
No new ROS semantics.
No fixes hidden inside components.

---

## Artifact Produced

### A Deployment‑Ready System Topology

The system is deployed via **launch**, with:

* One or more **component containers**
* One or more **standalone node processes**
* Explicit decisions about:

  * which nodes share a process
  * which nodes must be isolated
  * startup order
  * shutdown order

Topology is no longer accidental.
It is intentional, documented, and reproducible.

---

## What Changes from Topic 09

Nothing about the nodes changes.

What changes is **where they run**.

Examples of intentional (illustrative, not prescriptive) decisions:

* Telemetry publisher + verifier share a container
* Action server runs in its own process
* Lifecycle‑managed nodes activate only after dependencies are present

These decisions exist **only** in launch and verification scripts.

---

## What Must Be Demonstrated

Topic 10 is complete only when **all** of the following are true.

### 1. Deterministic Bring‑Up

* Launch brings the full system up reliably
* Installed vs source‑tree execution both work
* Configuration is discovered at runtime (no hard‑coded paths)

---

### 2. Explicit Failure Containment

* Load or failure in one node does **not** stall unrelated nodes
* Long‑running actions no longer degrade system‑wide responsiveness
* Shutdown of one component does not cascade unexpectedly

The engineer must be able to point to topology and say:

> “That is why this failure stopped there.”

---

### 3. Tooling Parity Is Preserved

Despite the new topology:

* `ros2 node list` remains meaningful
* `ros2 lifecycle get/set` continues to work
* `ros2 component list` works where composition is used
* No custom orchestration replaces standard ROS tools

---

### 4. Deployment Verification Exists

Topic 10 introduces **deployment verification scripts** that:

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

Topic 09 proved:

> Composition reveals truth.

Topic 10 proves:

> Composition must be used selectively.

---

### 3. Isolation Is an Architectural Choice

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
* a performance‑tuning exercise
* a replacement for ROS tools
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
