# Lesson 05 Theory: Parameters & Central Configuration (Rust / rclrs)

## Architectural Intent

Earlier lessons hardcoded runtime behaviour (topic names, QoS, publish rate).

Lesson 05 makes configuration a **system contract** that can be:

* supplied centrally (YAML → ROS parameters)
* inspected at runtime (ROS tooling)
* updated safely while the process runs

The design goal is not “parameters exist”, but “parameters control behaviour in a disciplined way”.

---

## Code Structure Overview

```text
3_rust/
 ├── utils_rclrs                (Shared configuration adapter)
 └── lesson_05_parameters
     ├── src/lib.rs             (Pure business logic)
     └── src/bin/*.rs           (ROS adapters: publisher + subscriber)
````

This separation is deliberate:

* `lib.rs` answers “what should we do?”
* `bin/*.rs` answers “how do we wire it into ROS 2 safely?”

---

## 1. YAML and ROS Parameters: What Actually Happens

A frequent misunderstanding is that the node “loads YAML”.

It does not.

The ROS 2 launch/runtime applies YAML files **before node construction**. When the node starts:

1. Parameters may already have override values (from YAML or CLI).
2. The node declares parameters (types + defaults).
3. The effective value is the merged result (override if provided, default otherwise).

In this lesson, the configuration contract is “declared + read”, not “file loaded”.

---

## 2. Shared Configuration Adapter: `utils_rclrs`

`utils_rclrs` centralises system-level configuration, so topic and QoS selection is consistent across nodes:

```rust
let topic = topics::telemetry(node);
let qos   = qos::telemetry(node);
```

This mirrors the intent of `utils_cpp`: configuration knowledge lives in one place, and nodes consume it consistently.

---

## 3. Runtime Updates: ParameterWatcher (No Polling)

Lesson 05 uses `rosrustext_rosrs::ParameterWatcher` to receive parameter change events and apply updates immediately.

The nodes subscribe to changes for specific parameters:

* publisher watches `timer_period_s`
* subscriber watches `reset_max_value`

The callback path performs:

1. name/type filtering
2. semantic validation
3. no-op detection (ignore unchanged values)
4. application of the new configuration

This matches the operational behaviour expected from other ROS 2 client libraries: parameter updates take effect without process restart.

---

## 4. Two Update Strategies (Why They Differ)

### Strategy A: Rebuild-on-Update (Publisher Timer)

Changing `timer_period_s` affects a scheduling resource (a timer). Rather than attempting to mutate an active timer, the publisher:

1. validates the new value
2. creates a new timer with the updated period
3. swaps it into stored state
4. drops the old timer by replacing the handle

This produces clean behaviour:

* no partial updates
* no timebase ambiguity
* no resource leaks if the swap fails

### Strategy B: In-Place Mutation (Subscriber Validator)

Changing `reset_max_value` affects only validation thresholds. The subscriber:

1. validates the new value
2. locks the validator
3. updates a single field (`reset_max_value`)
4. releases the lock

The subscription itself is unchanged, and the next message is evaluated using the new setting immediately.

---

## 5. Logic / Transport Separation

The stream validation and message production are pure Rust types in `lib.rs`.

Key outcomes:

* no ROS imports in logic
* deterministic, testable behaviour
* ROS-specific concerns live only in the node adapter layer

This is what enables meaningful unit tests (`cargo test`) independent of ROS graph state.

---

## Summary

Lesson 05 establishes a configuration architecture with three distinct roles:

* **YAML + ROS parameters** define system configuration inputs.
* **Nodes** declare, read, validate, and apply configuration changes.
* **Logic** remains pure and testable.

This is the base pattern that Lesson 06 later extends with lifecycle state management.
