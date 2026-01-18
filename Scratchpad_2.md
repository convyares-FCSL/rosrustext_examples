## The Narrative Arc: “Build → Break → Fix → Deploy”

---

## Phase 1: The Chassis (Lesson 06)

### **Lesson 06 – The Managed Node**

**Theme:** Determinism before features.

We abandon “always-on” scripts and introduce the **Lifecycle State Machine**.

**Artifact**

* A node that starts silent (`Unconfigured`)
* Allocates resources only on `Configure`
* Emits data only when `Active`

**Win**

* Deterministic startup
* Tool parity (`ros2 lifecycle`)
* Nav2-compatible behavior

**Limit**

* Single-threaded
* Assumes work is light

This is a *correct* node — but not yet a *survivable* one.

---

## Phase 2: The Stress Test (Lesson 07)

### **Lesson 07 – The Cardiac Arrest**

**Theme:** Functional correctness ≠ operational correctness.

We add a **long-running synchronous Action** to the lifecycle node.

**Artifact**

* Action server embedded in the managed node
* Heartbeat / telemetry publisher remains on the main execution path

**Failure**

* Action blocks the executor
* Heartbeat stops
* Lifecycle services become unresponsive
* `/get_state` times out

**Lesson**

* The math is correct
* The node is dead

This is the moment students learn that **the main thread is sacred**.

---

## Phase 3: The Fix (Lesson 08)

### **Lesson 08 – Executors & Callback Groups**

**Theme:** Concurrency as availability, not optimization.

We fix the failure explicitly.

**Artifact**

* Action moved into a `ReentrantCallbackGroup`
* Node spun by a `MultiThreadedExecutor`

**Win**

* Action runs without blocking
* Heartbeat continues
* Lifecycle services remain responsive

**Insight**

* Executors are not about speed
* They are about **survival under load**

At this point, the student has a **production-grade ROS 2 microservice**.

---

## Phase 4: Deployment Reality (Lesson 09)

### **Lesson 09 – Composition & Containers**

**Theme:** Nodes are correct. Now deployment breaks them.

We change **nothing** about node logic.
We change **how nodes are deployed**.

**Artifact**

* A **Component Container** process
* Multiple previously-built nodes loaded at runtime:

  * publisher
  * verifier/subscriber
  * service server
  * action server

**Focus**

* Composable nodes as a *deployment pattern*
* Shared executor ownership
* Load / unload / list via standard ROS tools
* Shared-fate failure modes
* Callback group isolation under composition

**Failure (Intentional)**

* Poor callback group choices cause interference
* Shared executor reveals hidden assumptions
* Shutdown order and lifecycle transitions matter

**Win**

* Nodes behave identically to standalone processes
* `ros2 component load/list/unload` works
* Launch and CLI tooling observe full parity

**Core Insight**

> Composition doesn’t change *what* your node does.
> It exposes whether your architecture was honest.

---

## Unified Architecture (Post-Lesson 09)

By the end of Lesson 09, the student owns a **deployment-ready system**:

1. **Configuration** – Centralized, live (L05)
2. **State** – Lifecycle-managed (L06)
3. **Logic** – Pure, testable, ROS-free (L04)
4. **Work** – Actions with cancellation & feedback (L07)
5. **Concurrency** – Executors & callback groups (L08)
6. **Deployment** – Composable containers, runtime topology (L09)

This is no longer “ROS 2 code”.

This is **operable infrastructure**.

