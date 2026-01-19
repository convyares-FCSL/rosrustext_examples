# Lesson 06 Theory: Lifecycle Management (Rust / roslibrust + rosrustext)

## Architectural Intent

In Lessons 00–05, Rust nodes followed an *eager execution* model:

- Resources were allocated during startup.
- Publishers/subscriptions/timers became active immediately.
- Startup order implicitly defined system behavior.

In Lesson 06, this is replaced with **Managed Lifecycle Nodes**.

The objective is **operational determinism**:
a node must be fully configured, validated, and externally approved *before* it participates in the ROS graph.

Because `roslibrust` is a ROS-bridge/WebSocket client (not a native RMW node), it does **not** natively appear in the ROS 2 graph and it does **not** implement the ROS 2 lifecycle surface by default. This lesson uses the `rosrustext` stack to provide:

- A ROS 2-compatible lifecycle state machine and transition semantics.
- A canonical ROS-facing lifecycle API (`lifecycle_msgs/*`).
- A bridge strategy that keeps the lesson code clean and avoids custom interfaces.

This enables Rust nodes to participate in **production-grade orchestration** rather than acting as always-on scripts.

---

## Lifecycle Support in roslibrust: Important Context

Unlike `rclcpp_lifecycle::LifecycleNode` (C++) or managed nodes in `rclpy`, a `roslibrust` binary is not a native ROS 2 node. It talks to ROS 2 through:

- `rosbridge_server` (ROS 2 ↔ WebSocket JSON gateway)
- a `ClientHandle` WebSocket connection (in the Rust process)

### What that implies

1. **Discovery is not automatic**
   - A pure rosbridge client does not show up as a ROS graph participant.
2. **Lifecycle services must be *projected* into ROS**
   - The ROS 2 CLI expects canonical services/topics on a visible node.

---

## The rosrustext Solution: Split “Logic” from “ROS-Facing Surface”

This lesson uses two rosrustext pieces:

### 1) `rosrustext_roslibrust` (inside the lesson binary)

Provides:

- `LifecycleNode`: lifecycle state machine + transition callbacks
- `ActivationGate`: cheap “am I Active?” gating primitive
- `register_lifecycle_backend_rosbridge(...)`: a helper that advertises lifecycle services/topics over rosbridge

The important detail: the helper exposes lifecycle endpoints using **canonical ROS 2 types**:

- `lifecycle_msgs/srv/*`
- `lifecycle_msgs/msg/TransitionEvent`
- optional `bond/msg/Status` exists in the overall stack, but is not lesson-required

### 2) `rosrustext_lifecycle_proxy` (separate tool)

Provides:

- A ROS-visible node on the graph (Jazzy-safe as of v0.2.3)
- Standard ROS 2 lifecycle CLI compatibility (`ros2 lifecycle get/set`)
- A “projection” layer that makes the rosbridge-backed lifecycle backend feel like a normal ROS node

The lesson keeps the binaries simple and delegates graph-facing concerns to the proxy.

---

## Core Design Pattern in the Lesson

Lesson 06 uses a deliberately “hard” pattern that prevents common lifecycle mistakes:

```
Pure Logic  → what should happen
Lifecycle   → when it is allowed to happen (state machine)
Async Engine→ how it happens (resource ownership + async I/O)
````

### Why this pattern?

Because lifecycle callbacks are synchronous, while roslibrust I/O is async.

So the binaries enforce:

- **Sync callbacks that never block**
- **An async engine task that owns all resources**
- **A command channel bridging sync→async**

This yields deterministic resource ownership and eliminates “half-allocated” or “ghost” resources across transitions.

---

## 1. Pure Logic Remains Lifecycle-Agnostic

All business logic stays in `lib.rs`:

- `TelemetryPublisherCore`
- `TelemetryStreamValidator`
- `period_s_to_ms_strict`

These types:

- have no ROS dependencies
- have no lifecycle awareness
- are unit-testable without ROS

This is intentional: lifecycle should gate *execution*, not infect the core logic.

---

## 2. Lifecycle Node as the Orchestrator

The lesson implements:

- `LifecycleCallbacks` (rosrustext-provided) on a small shim struct
- `LifecycleNode` as a state machine wrapper around those callbacks

Lifecycle hooks exist for:

- `on_configure`
- `on_activate`
- `on_deactivate`
- `on_cleanup`
- `on_shutdown`

**Hard rule:**
> The callbacks must not directly create/destroy ROS resources.

Instead they enqueue a command to the async engine.

---

## 3. Non-Blocking Callbacks (Correctness & Safety)

The callbacks use:

- `tokio::sync::mpsc::Sender::try_send(...)`

This means:

- No blocking inside lifecycle callbacks
- No risk of deadlocking a lifecycle driver thread
- Backpressure is explicit: if the queue is full, the transition fails

This is “production standard” for synchronous callback surfaces that control async systems.

---

## 4. Async Engine Owns All ROS Resources

The async engine task owns:

- `ClientHandle` (rosbridge connection)
- `Publisher<MsgCount>` (publisher binary)
- `Subscriber<MsgCount>` (subscriber binary)
- timers / tick loop (publisher)
- validator state (subscriber)

The engine is the only place where:

- `client.advertise(...)` happens
- `client.subscribe(...)` happens
- messages are published
- inbound messages are processed

This creates a clean lifecycle boundary: callbacks trigger commands; engine performs work.

---

## 5. Gating: “Active means data-plane runs”

### Publisher

The publisher does not “start/stop” its tick loop. Instead it:

- keeps the timer running
- uses `ActivationGate::is_active()` to decide whether publishing is allowed

This ensures:

- deterministic behavior across transitions
- no ad-hoc `if active { ... }` scattered in user logic
- a single place where activation policy is enforced

### Subscriber

The subscriber demonstrates *resource-scoped gating*:

- on **Activate**: create subscription
- on **Deactivate**: drop subscription
- on **Cleanup**: drop subscription + reset validator

This yields the strongest teaching point:

- Inactive state has *no inbound traffic* and no processing.

---

## 6. The Backend Registration Helper (Why it exists)

Because a rosbridge client does not automatically provide ROS lifecycle services/topics, the lesson calls:

```rust
register_lifecycle_backend_rosbridge(&client, NODE_NAME, Arc::clone(&lifecycle_node)).await?;
````

This is minimal “wiring”, but it is **not lesson glue**:

* it is a stable helper in the library
* it exposes only canonical ROS 2 lifecycle types
* it keeps lesson code focused on architecture and behavior

### Why `Arc<Mutex<LifecycleNode>>`?

The backend services must read/transition lifecycle state from async service handlers. The simplest safe shared ownership boundary is:

* `Arc` for shared ownership across tasks
* `Mutex` for synchronized access to lifecycle state transitions

This is acceptable here because lifecycle operations are low-frequency control-plane actions, not hot data-plane paths.

---

## 7. Deterministic Shutdown

Both binaries trap Ctrl-C via:

* `tokio::signal::ctrl_c()`

and then send an explicit `Shutdown` command to the engine.

The engine:

* drops resources
* exits its loop
* allows `main` to join the task

This guarantees a clean OS-level shutdown even if the external orchestration is interrupted.

---

## Summary

Lesson 06 establishes deterministic lifecycle behavior for `roslibrust` nodes despite the constraints of a rosbridge/WebSocket transport:

* **The lesson binaries remain clean**: no custom interfaces, no hand-built lifecycle services.
* **Canonical ROS lifecycle types are used**: `lifecycle_msgs/*`.
* **Non-blocking lifecycle callbacks** feed an **async engine** that owns all resources.
* **Activation gating** and **resource-scoped subscription** enforce correct data-plane behavior.
* The **proxy** provides a ROS-visible control surface so standard `ros2 lifecycle` CLI works normally (Jazzy-safe with v0.2.3).

This model scales to real systems where startup order, safety, and orchestration matter.