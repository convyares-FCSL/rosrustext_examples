# Lesson 05 Theory: Parameters & Central Configuration (Rust / rclrs)

## Architectural Intent

In earlier lessons, node behaviour was hardcoded (publish rate, topic names, QoS).

In Lesson 05, configuration becomes a **first-class system contract**.

The goal is to demonstrate that, even with an evolving Rust client library (`rclrs`), we can:

1. **Load configuration centrally from YAML at startup**
2. **Inspect and modify behaviour at runtime using ROS parameters**
3. **Keep business logic isolated from middleware concerns**
4. **Apply updates safely without restarting the process**

This lesson mirrors the C++ (`rclcpp`) architecture as closely as the current Rust ecosystem allows.

---

## Core Concepts Introduced

Lesson 05 introduces three production-grade patterns:

1. **Configuration-First Design**
   Topic names, QoS, and behavioural parameters are no longer hardcoded.
   They are defined in shared YAML files and consumed uniformly across nodes.

2. **Hot Updates via a Control Plane**
   Runtime configuration changes are applied while the node is running.

3. **Logic / Transport Separation**
   Validation and sequencing logic lives in pure Rust, independent of ROS 2.

---

## Code Structure Overview

```text
3_rust/
 ├── utils_rclrs                (Shared Configuration Adapter)
 └── lesson_05_parameters
     ├── lib.rs                 (Pure Business Logic)
     └── bin/*.rs               (Middleware Adapters)
```

Each layer has a single responsibility.

---

## 1. Pure Logic: `lib.rs` (No ROS Dependencies)

As in the C++ lesson, **all business logic is middleware-free**.

```rust
// Pure Rust logic, testable without ROS
pub struct TelemetryStreamValidator {
    expected: i64,
    reset_max_value: i64,
}

impl TelemetryStreamValidator {
    pub fn on_count(&mut self, count: i64) -> StreamDecision {
        // validation logic
    }
}
```

Key properties:

* No `rclrs` imports
* No threading or timers
* Deterministic and unit-testable
* Mirrors `logic.hpp` in the C++ version

This guarantees correctness *before* integration with ROS.

---

## 2. Shared Configuration Adapter: `utils_rclrs`

utils_rclrs lives outside individual lessons and is shared across all Rust examples, enforcing a single workspace-wide configuration standard, exactly like utils_cpp in the C++ track.

Just like `utils_cpp`, the Rust `utils_rclrs` crate centralises system knowledge:

```rust
let topic = topics::telemetry(&node);
let qos   = qos::telemetry(&node);
```

### What `utils_rclrs` Actually Does (Important)

`utils_rclrs` **does not load YAML**.

Instead:

1. YAML files are loaded by ROS **before** the node starts
2. `declare_parameter` registers the parameter with the node
3. `MandatoryParameter<T>` provides typed access to the effective value
4. If no override exists, the declared default is used

This matches C++ behaviour exactly.

```rust
pub fn declare_parameter<T>(
    node: &Node,
    name: &str,
    default_value: T,
) -> Result<MandatoryParameter<T>, DeclarationError>
where
    T: ParameterVariant + Clone + 'static,
{
    node.declare_parameter(name)
        .default(default_value)
        .mandatory()
}
```

Across the entire Rust workspace, the adapter guarantees:

* One canonical parameter name
* One canonical type
* Identical resolution across publisher and subscriber

---

## 3. Middleware Adapter: The Node (Rust / rclrs)

The node owns:

* ROS resources (publishers, subscriptions, timers)
* The logic object
* The configuration handles

It acts as the **adapter** between ROS parameters and pure logic.

---

## 4. Runtime Updates: The “Hotfix” Control Plane

### Why Polling Exists in Rust

As of `rclrs v0.6.x`, there is **no equivalent** to:

```cpp
add_on_set_parameters_callback(...)
```

Rather than hiding this limitation, Lesson 05 makes it explicit and uses a **Control Plane / Data Plane split**.

---

### Control Plane vs Data Plane

```rust
// Data Plane (High frequency, deterministic)
fn on_tick(&self) {
    // publish data
}

// Control Plane (Low frequency, ~1 Hz)
fn check_parameters(&self) {
    let current = period_param.get();
    if current != last {
        // apply update
    }
}
```

Key properties:

* Configuration polling is **low priority**
* Publishing logic is **never blocked**
* Behaviour matches callback semantics in practice

This mirrors the intent of the C++ callback, even if the mechanism differs.

---

## 5. Update Strategy 1: Rebuild-on-Update (Publisher)

**Use case:** Changing infrastructure resources (timers).

In Rust, as in C++, we do **not** mutate a running timer.

Instead:

1. Validate the new value (pure logic)
2. Construct a new `rclrs::Timer`
3. Swap it into a `Mutex<Option<Timer>>`
4. Drop the old timer safely

```rust
fn on_param_change(new_period_ms: u64) {
    create_or_update_timer(new_period_ms);
}
```

This guarantees:

* No partial state
* No undefined timing behaviour
* No race conditions

This is the same strategy as the C++ `create_or_update_timer`.

---

## 6. Update Strategy 2: In-Place Mutation (Subscriber)

**Use case:** Logic thresholds (`reset_max_value`).

Here we mutate only the logic state:

```rust
let mut v = validator.lock().unwrap();
v.set_reset_max_value(new_value);
```

Why this is safe:

* The subscription remains unchanged
* Only a single integer is updated
* The next message uses the new value immediately

This mirrors the C++ approach of updating the validator without rebuilding the subscription.

---

## Memory & Ownership Model (Rust Perspective)

Rust enforces ownership at compile time:

* `Arc<Mutex<T>>` replaces `std::unique_ptr`
* Explicit locking replaces implicit thread safety
* Lifetimes are enforced by the type system

While more verbose, this eliminates entire classes of runtime errors.

---

## Testing Strategy

Because logic is isolated:

```bash
cargo test
```

Unit tests validate:

* Stream ordering
* Reset detection
* Period quantisation

No ROS graph is required, just like in the C++ lesson.

---

## Why This Lesson Matters

Lesson 05 demonstrates **professional-grade systems engineering in Rust**:

* Configuration is external and inspectable
* Logic is deterministic and testable
* Runtime updates are safe and controlled
* Middleware limitations are handled explicitly, not hidden

This architecture scales directly to:

* Larger systems
* Multiple nodes
* Mixed Rust / C++ deployments

It establishes a **shared mental model** across languages, not just shared YAML files.