## Topic 09 — Composition & Containers

### *Deployment as a Truth Serum*

---

## When Correct Nodes Fail Together

By the end of Topic 08, every node in the system is **internally correct**:

* lifecycle-managed and externally controllable
* resilient to long-running work
* responsive under load *in isolation*
* observable through standard ROS tooling

Topic 09 introduces the pressure that invalidates a common assumption:

> **Correct nodes do not guarantee a correct system once deployment topology changes.**

Nothing breaks because of bugs.
Things break because **failure domains change**.

---

## The Question This Topic Answers

> **What happens when multiple healthy nodes share a process, an executor, and a shutdown boundary?**

This is not a concurrency problem.
It is not an algorithmic problem.
It is a **deployment topology problem**.

---

## What Changes in Topic 09

Only **how nodes are hosted** changes.

* Nodes from Topics 06–08 are reused **without modification**
* Node logic, interfaces, and behaviour remain intact
* The only new factor is **co-location**

Specifically, nodes are now:

* loaded into a **single component container**
* scheduled by a **shared executor**
* subject to a **shared lifetime and shutdown domain**

No fixes are applied.
No mitigations are introduced.
The goal is exposure, not correction.

---

## Composition as an Operational Primitive

Composition is introduced here **not** as an optimisation and **not** as a convenience.

It is introduced because it:

* collapses process boundaries
* merges scheduling domains
* couples shutdown semantics
* reveals interference that isolation previously hid

Composition does not change semantics.
It reveals architectural honesty.

---

## What Must Be Demonstrated

Topic 09 is complete when **deployment-induced effects** are observable **while the system is still running**.

### 1. Canonical Tooling Still Applies

The composed system must remain visible and controllable via standard ROS 2 tools:

* `ros2 component load / list / unload`
* `ros2 node list`
* `ros2 lifecycle get / set`
* `ros2 action list / send_goal`

No proxy commands.
No lesson-specific shims.
No alternate semantics.

---

### 2. Operability Degrades Without Crashing

At least one of the following must be observable **without terminating the container**:

* lifecycle transitions are delayed or time out under load
* telemetry stalls or becomes jittery while work proceeds
* action cancellation latency becomes unacceptable
* one node’s work interferes with another’s responsiveness

The container remains alive.
The nodes remain “correct.”
**Operability degrades anyway.**

This is the core truth of the lesson.

---

### 3. Failure Is Clearly Deployment-Induced

The student must be able to state, truthfully:

> “Nothing in the nodes changed.
> This behaviour changed because of *where* the nodes run.”

If the failure can be attributed to:

* incorrect logic
* missing locks
* poor callback group design inside a node

then Topic 09 has failed.

---

### 4. Shared Fate Is Explicitly Visible

Process-level shared fate is still demonstrated — but as a **secondary confirmation**, not the headline.

When the container terminates:

* all nodes exit together
* in-flight actions are interrupted
* lifecycle state disappears simultaneously

This confirms the failure domain, but it is not the primary lesson.

---

## What This Topic Does *Not* Do

Topic 09 does **not**:

* refactor nodes
* “fix” interference
* introduce launch files
* introduce orchestration policy
* invent safety guarantees

Any attempt to “solve” the exposed problem here would teach the wrong lesson.

Composition provides **co-location**, not **isolation**.

---

## Relationship to Topic 10

Topic 09 answers:

> *What breaks when this system is deployed for real?*

Topic 10 answers:

> *How do we bring it up, verify it, and shut it down reliably despite that reality?*

Topic 09 creates the pressure.
Topic 10 operationalises the response.

---

## The Mental Model the Student Must Leave With

> **Correct nodes are necessary.**
> **Correct deployment is mandatory.**
> **Composition tells the truth.**

---

# 2. Supervisor Agent Alignment Prompt

Use this verbatim as a handoff / reset for the supervising agent.

