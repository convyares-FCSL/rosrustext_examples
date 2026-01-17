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
  * Managed (gated) execution primitives

### Implication

Lifecycle behavior in Rust is **compositional**, not inheritance-based.

This has two important consequences:

1. You cannot accidentally bypass the lifecycle by calling low-level APIs directly.
2. Resource ownership and cleanup must be handled explicitly by user code.

This is more verbose than C++, but also more explicit, auditable, and predictable.

---

## Core Design Pattern

Lesson 06 enforces a **three-layer separation**, by structure rather than convention:

```

Pure Logic        → what should happen
Component Layer  → how it happens
Lifecycle Node   → when it is allowed to happen

````

Each layer has a single responsibility.

---

## 1. Pure Logic Remains Lifecycle-Agnostic

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

## 2. Lifecycle Node as the Orchestrator

`Lesson06PublisherNode` and `Lesson06SubscriberNode` implement:

```rust
LifecycleCallbacksWithNode
````

This trait defines lifecycle hooks:

* `on_configure`
* `on_activate`
* `on_deactivate`
* `on_cleanup`
* `on_shutdown`

Each callback corresponds to a **controlled phase of resource ownership**.

### Hard rule

> **No publishers, subscriptions, timers, or parameter watchers are created outside `on_configure`.**

An `Unconfigured` node has **zero runtime footprint**.

---

## 3. Builder-Based Resource Construction (Managed)

Lesson 06 adopts **builder-style APIs** for all lifecycle-managed resources.

This replaces earlier `create_*` helpers with explicit construction flows that:

* Preserve full `rclrs` option parity
* Store only owned configuration
* Enforce correct ordering (e.g. callbacks required before creation)
* Make lifecycle gating visible and auditable

### Managed Publisher

```rust
let publisher = node
    .publisher::<MsgCount>(&topic)
    .create()?;
```

Properties:

* Publisher exists only after `on_configure`
* Publish calls are gated by lifecycle state
* No DDS traffic occurs unless the node is `Active`

---

### Managed Timer (Gated Execution)

```rust
let timer = node
    .timer_repeating(Duration::from_millis(period_ms))
    .callback(move || {
        component.publish();
    })
    .create()?;
```

Key points:

* The timer **exists** in `Inactive`, but its callback is silent
* Callback execution begins only in `Active`
* The timer is dropped during `on_cleanup`

Gating occurs at **callback entry**, not inside user code.

---

### Managed Subscription (Lifecycle-Aware)

```rust
let sub = node
    .subscription::<MsgCount>(&topic)
    .callback(move |msg| {
        component.on_msg(msg);
    })
    .create()?;
```

Behavior:

* Subscriptions are lifecycle-scoped
* Callbacks are suppressed unless the node is `Active`
* No manual `if active { … }` guards are required

---

## 4. Passive Components (Publisher / Subscriber)

`PublisherComponent` and `SubscriberComponent` are **passive containers**.

They:

* Own publishers, timers, and validation logic
* Expose methods like `publish()` or `on_msg()`
* Do **not** decide when execution occurs

Lifecycle logic determines *when* they are allowed to run.

They are:

* Created during `on_configure`
* Stored in `Arc<Option<…>>`
* Dropped deterministically during `on_cleanup` or `on_shutdown`

This ensures clean DDS teardown and predictable resource lifetimes.

---

## 5. Parameter Handling and Lifecycle Boundaries

Lifecycle introduces a critical distinction:
**what survives cleanup?**

### In this lesson:

* **Volatile runtime resources**

  * publishers
  * subscriptions
  * timers
  * parameter watchers
    → created in `on_configure`, dropped in `on_cleanup`

* **Declared parameters**
  → retained across deactivate / cleanup

### Rationale

Parameters represent **system configuration**, not runtime state.

Undeclaring them would:

* Discard launch-time overrides
* Break reconfiguration workflows
* Diverge from ROS 2 expectations

Parameter updates are handled via:

```rust
ParameterWatcher
```

The watcher itself is lifecycle-scoped and destroyed during cleanup, preventing updates from being applied while unconfigured.

---

## 6. Error Handling and Safety

Several deliberate design choices appear in the code.

### Lock Poisoning

* Mutex poisoning is treated as **non-fatal**
* Callbacks exit early
* The process continues running

This matches ROS expectations for long-lived nodes.

---

### Error Normalization

All backend errors are normalized to `RclrsError` via helpers in `utils_rclrs`.

Lifecycle logic never depends on backend-specific error types.

---

## 7. Testing Strategy

Testing is split across layers.

### Unit Tests (Rust)

* Target: `lib.rs`
* Scope: Pure logic
* Execution: `cargo test`
* No ROS graph required

---

### Integration Tests (Python / launch_testing)

Lifecycle behavior is verified externally:

* Node is launched as a black-box binary
* Python drives lifecycle transitions via services
* State transitions are asserted explicitly

```python
def test_lifecycle_sequence(self):
    self.assertEqual(self._get_state().label, 'unconfigured')
    self._change_state(Transition.TRANSITION_CONFIGURE)
    self.assertEqual(self._get_state().label, 'inactive')
```

This mirrors how real operators interact with lifecycle-managed systems.

---

## Summary

Lesson 06 establishes **deterministic, externally orchestrated execution** for Rust ROS 2 nodes.

* Builders make resource construction explicit and auditable.
* Lifecycle controls *when* execution occurs.
* Logic remains pure and testable.
* Runtime behavior is predictable, inspectable, and safe.

This model scales to systems where **startup order, safety, and coordination matter**, rather than assuming nodes should begin operating immediately.