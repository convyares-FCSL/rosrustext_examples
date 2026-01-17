# Lesson 06 Theory: Lifecycle Management (Rust / rclrs + rosrustext)

## Architectural Intent

In Lessons 00–05, Rust nodes followed an *eager execution* model:

* Resources were allocated during node construction.
* Timers, publishers, and subscriptions became active immediately.
* Startup order implicitly defined system behavior.

In Lesson 06, this model is replaced with **Managed Lifecycle Nodes**.

The objective is **operational determinism**:
ensuring a node is fully configured, validated, and externally approved *before* it participates in the ROS graph.

Because `rclrs` does not natively implement the ROS 2 lifecycle state machine, this lesson uses **`rosrustext_rosrs`** to provide:

* A lifecycle state machine compatible with ROS 2 tooling.
* Gated execution primitives.
* Parameter event handling equivalent to modern `rclcpp`.

This enables Rust nodes to participate in **production-grade orchestration** rather than acting as always-on scripts.

---

## Lifecycle Support in Rust: Important Context

Unlike C++, Rust does **not** have a native equivalent of `rclcpp_lifecycle::LifecycleNode`.

Instead:

* **`rclrs::Node`** remains the fundamental node abstraction.
* **`rosrustext_rosrs::LifecycleNode`** wraps an internal `rclrs::Node` and exposes:

  * Lifecycle state transitions
  * Transition callbacks
  * Managed (gated) resources

### Implication

Lifecycle behavior in Rust is **compositional**, not inheritance-based.

This has two important consequences:

1. You cannot accidentally bypass the lifecycle by calling low-level APIs directly.
2. Resource ownership and cleanup must be handled explicitly by user code.

This is more verbose than C++, but also more explicit and auditable.

---

## Core Design Pattern

Lesson 06 introduces a **three-layer separation**, enforced by structure rather than convention:

```
Pure Logic        → what should happen
Component Layer  → how it happens
Lifecycle Node   → when it is allowed to happen
```

Each layer has a single responsibility.

---

## Code Walkthrough

### 1. Pure Logic Remains Lifecycle-Agnostic

All business logic continues to live in `lib.rs`:

* `TelemetryPublisherCore`
* `TelemetryStreamValidator`
* `period_s_to_ms_strict`

These types:

* Have **no ROS dependencies**
* Have **no lifecycle awareness**
* Are tested using standard Rust unit tests

This ensures correctness can be validated independently of ROS infrastructure.

---

### 2. Lifecycle Node as the Orchestrator

The `Lesson06PublisherNode` and `Lesson06SubscriberNode` structs implement:

```rust
LifecycleCallbacksWithNode
```

This trait defines the lifecycle hooks:

* `on_configure`
* `on_activate`
* `on_deactivate`
* `on_cleanup`
* `on_shutdown`

Each callback corresponds to a **controlled phase of resource ownership**.

#### Key rule

> **No timers, publishers, subscriptions, or parameter watchers are created outside `on_configure`.**

This guarantees that an `Unconfigured` node has **no runtime footprint**.

---

### 3. Passive Components (Publisher / Subscriber)

The `PublisherComponent` and `SubscriberComponent` structs are **passive containers**.

They:

* Own publishers, timers, and logic
* Expose methods like `publish()` or `on_msg()`
* Do **not** decide when execution occurs

They are:

* Created during `on_configure`
* Stored in `Arc<Option<...>>`
* Dropped during `on_cleanup` or `on_shutdown`

This ensures deterministic release of DDS resources.

---

## Gating Strategies in Rust

Unlike C++, Rust does not get lifecycle gating “for free” from the middleware.

Gating is therefore **explicit and visible in code**.

### Publisher Gating (Timer-Based)

In Rust, publishers are gated by **controlling execution**, not transport.

* Timers are created using `create_timer_repeating_gated`
* The timer callback is only executed while the node is `Active`
* No timer exists in `Unconfigured`
* Timers exist but are silent in `Inactive`

```rust
node.create_timer_repeating_gated(duration, move || {
    component.publish();
});
```

**Effect:**
No `publish()` call is made unless the lifecycle permits it.

---

### Subscriber Gating

Subscriptions are also created as **managed (gated) subscriptions**.

* Messages received while `Inactive` are dropped
* The callback is not executed unless the node is `Active`

This avoids manual boolean guards inside callbacks and keeps lifecycle behavior centralized.

---

## Parameter Handling and Persistence

Lifecycle transitions introduce a subtle but critical design decision:
**what survives cleanup?**

### In this lesson:

* **Volatile resources** (timers, publishers, subscriptions, watchers)
  → created in `on_configure`, dropped in `on_cleanup`
* **Configuration parameters**
  → declared during `on_configure`, retained across deactivate/cleanup

### Rationale

Undeclaring parameters during cleanup would:

* Discard values provided via launch files or CLI
* Prevent reliable re-configuration after cleanup

Therefore, parameters are treated as **infrastructure**, not runtime state.

Parameter updates are handled via:

```rust
ParameterWatcher
```

The watcher itself is lifecycle-scoped and dropped during cleanup to avoid handling updates while unconfigured.

---

## Error Handling and Safety Considerations

Several deliberate design choices are visible in the code:

### Lock Poisoning

* Mutex poisoning is treated as **non-fatal**
* Callbacks exit early rather than panic
* The node remains alive but logs the failure

This aligns with ROS expectations for long-running processes.

---

### Error Normalization

All `rclrs` API errors are mapped into a consistent `RclrsError` form using helpers in `utils_rclrs`.

This avoids leaking backend-specific error types into lifecycle logic.

---

## Testing Strategy

Testing is split across two layers.

### 1. Unit Tests (Rust)

* Target: `lib.rs`
* Scope: Logic correctness
* Execution: `cargo test`
* No ROS graph required

---

### 2. Integration Tests (Python / launch_testing)

* **Target**: The compiled Rust binary (`lesson_06_lifecycle_publisher`).
* **Scope**: Verifying the **Process** and **State Machine**.
* **Why Python?**: The ROS 2 Launch system is Python-based. It allows us to treat the Rust node as a "Black Box," launching it, waiting for it to spawn, and issuing Service Calls (`/change_state`) just like a real user would.

```python
# Python driving a Rust node
def test_lifecycle_sequence(self):
    # 1. Assert Start State
    self.assertEqual(self._get_state().label, 'unconfigured')
    
    # 2. Drive Transition
    self._change_state(Transition.TRANSITION_CONFIGURE)
    
    # 3. Assert New State
    self.assertEqual(self._get_state().label, 'inactive')
```

---

## Summary

Lesson 06 establishes a **managed execution model** for Rust ROS 2 nodes.

* Business logic is isolated and testable.
* Resource allocation is explicit and deterministic.
* Execution is strictly controlled by lifecycle state.
* Runtime behavior is observable and externally orchestrated.

This architecture scales to systems where **startup order, safety, and coordination matter**, rather than assuming all nodes should begin operating immediately.
