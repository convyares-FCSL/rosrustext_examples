# Lesson 06 Theory: Orchestration (Lifecycle Manager)

## Architectural Intent

In Lesson 06 (Lifecycle Management), individual nodes adopt the **Managed Node** pattern: they expose an explicit state machine and defer participation in the system until they are configured and activated.

This lesson introduces the complementary **system-level pattern**: **Orchestration**.

The goal of orchestration is **Operational Determinism at the system boundary**.

While lifecycle nodes ensure *local correctness* (“this node is ready to run”), orchestration ensures *global correctness* (“the system enters a valid operating state in a known order”).

Orchestration is not about adding intelligence to nodes.
It is about **coordinating state transitions across independently built components**.

---

## Orchestration vs Construction

A common anti-pattern in distributed systems is to rely on **construction-time behavior**:

* resources are acquired in constructors,
* communication begins immediately,
* failures are handled implicitly via retries or timeouts.

This “start-and-scream” model couples **process startup** with **operational readiness**.

Lifecycle management breaks that coupling at the node level.
Orchestration breaks it at the **system level**.

The separation is explicit:

* **Process lifetime**: managed by the OS and execution tooling (`ros2 run`, supervisors, containers).
* **Operational readiness**: managed explicitly through lifecycle state transitions.

Orchestration occurs *after* processes exist, using only their public control interfaces.

---

## The Manager as Infrastructure

The lifecycle manager introduced in this lesson is a **standard ROS 2 node**, not a lifecycle-managed node itself.

This is deliberate.

The manager:

* owns **no business logic**,
* allocates **no domain resources**,
* does **not** participate in data flow.

Its role is purely infrastructural:
to express **policy** about *when* other nodes are allowed to operate.

Control is exercised exclusively through **standard ROS 2 lifecycle services** (`GetState`, `ChangeState`).
Managed nodes are treated as **black boxes**.

This makes orchestration observable, auditable, language-agnostic, and tooling-compatible.

---

## Transition Semantics and Ordering

The manager drives a minimal, canonical subset of the lifecycle state machine, with a deliberate distinction between **configuration** and **activation**.

### Startup

Startup is treated as a **two-phase operation**.

#### Phase 1 — Configuration Barrier

All managed nodes are first driven through:

**Configure** (`Unconfigured → Inactive`)

Configuration is treated as a **system-wide barrier**, not a per-node action.

This establishes the invariant that:

* all required resources are allocated,
* all parameters are validated,
* all communication endpoints are created,

**before any node is permitted to produce externally visible effects**.

No node is activated until *every* node has successfully reached `Inactive`.

This avoids transient partial states where one component is live while others are still unprepared.

#### Phase 2 — Ordered Activation

Once all nodes are configured, the manager performs:

**Activate** (`Inactive → Active`)

Activation is **ordered**, not parallel.

For publish/subscribe systems, the ordering is intentional:

* **Subscribers are activated first**
* **Publishers are activated last**

This ensures that:

* consumers are ready before producers emit data,
* initial messages are not lost due to late activation,
* activation represents the moment the *system* goes live, not just an individual node.

This lesson uses a **simple, explicit ordering rule** rather than attempting to infer or solve a general dependency graph.

More complex activation strategies are possible, but are intentionally out of scope.

---

### Shutdown

Shutdown mirrors startup, but with weaker guarantees:

1. **Deactivate** (`Active → Inactive`)
2. **Shutdown** (`Inactive → Finalized`)

Shutdown is treated as **best-effort**.

Once termination begins, guarantees weaken:

* nodes may be unresponsive,
* services may time out,
* process-level shutdown may intervene.

The manager attempts orderly teardown while the system is still operational, but does not claim authority over process death.

---

## Control Plane vs Data Plane

Orchestration operates strictly in the **control plane**.

Lifecycle state transitions are driven via services and are independent of:

* topic names,
* namespaces,
* QoS policies,
* language-specific defaults.

As a result:

* lifecycle orchestration may succeed,
* all nodes may reach `Active`,
* yet **no data flows**.

This distinction is intentional.

Control-plane correctness ensures that the system is *allowed* to run.
Data-plane correctness determines whether components actually communicate as intended.

In same-language systems, defaults often align implicitly.
In mixed-language systems, defaults may diverge.

This lesson treats that divergence as a **first-class concern**, not a bug.

---

## Failure Policy

Orchestration must make failure **explicit**.

This manager supports two policies:

* **Fail-Fast (Default)**
  If a node fails to transition, orchestration halts immediately.
  Partial startup is treated as a system error.

* **Continue-On-Error**
  Remaining nodes are attempted even if one fails.
  This is useful for degraded or non-critical operation.

Recovery is not implicit.
Failures are logged, surfaced, and left for operators or external supervisors to resolve.

---

## Interoperability as a Design Constraint

The manager relies only on standard ROS 2 lifecycle services.

As a result, it can orchestrate:

* C++ lifecycle nodes (`rclcpp_lifecycle`)
* Python lifecycle nodes
* Rust lifecycle nodes (`rclrs` with lifecycle support)

Conversely, externally provided managers (for example, `nav2_lifecycle_manager`) can orchestrate the nodes produced in this lesson.

This bidirectional compatibility is not an optimization — it is a **correctness requirement**.

If a node cannot be driven correctly by standard tooling, it is not lifecycle-compliant.

---

## Summary

Lesson 06 establishes three distinct architectural layers:

* **Node Logic**
  Implements behavior and domain functionality.

* **Lifecycle Management**
  Enforces local readiness and safe activation.

* **Orchestration**
  Enforces system-wide order, determinism, and policy.

By separating these concerns, systems become:

* predictable to start,
* safe to reconfigure,
* observable to operators,
* resilient to partial failure.

Orchestration does not make systems “smarter”.
It makes them **intentional**.