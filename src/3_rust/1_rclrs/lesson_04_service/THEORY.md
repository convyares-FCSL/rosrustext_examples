# Lesson 04 Breakdown: Services, Async Clients, and Testability (Rust)

## Architectural Intent

Lesson 04 introduces the **Request–Response** communication model using ROS 2 Services.

Unlike Topics (Lesson 03), services are **discrete interactions**: a client issues a request and receives a single response. This lesson focuses on implementing services in a way that is:

* **Testable**
* **Non-blocking**
* **Architecturally clean**
* **Idiomatic Rust**

The emphasis is not on “calling a service”, but on building **professional service components** that scale.

---

## Core Principle: Logic Separation (Pure Rust Library)

Service callbacks must remain **thin adapters**. All business logic is implemented in a pure Rust library (`lib.rs`) with no ROS dependencies.

```text
service_server.rs   (ROS adapter)
 └── lib.rs         (Pure Rust logic)
```

### Why this matters

Separating logic from middleware provides:

1. **Deterministic unit testing** without ROS.
2. **Reusability** outside ROS (CLI tools, batch jobs, simulations).
3. **Clear ownership boundaries** between computation and transport.
4. **Predictable failure modes** (logic bugs vs. communication bugs).

---

## Business Logic Design

The core logic is implemented as a pure function:

```rust
pub fn compute(data: &[f64]) -> StatsResult
```

### Key properties

* Accepts a **slice** (`&[f64]`) — no allocation or ownership transfer.
* Returns a simple struct (`StatsResult`) — no ROS types.
* Encodes **semantic state** in `status` rather than throwing errors.
* Handles edge cases (empty input) deterministically.

### Unit Testing

Because the logic is middleware-free, it is verified using standard Rust tests:

```bash
cargo test
```

This confirms algorithm correctness **before any ROS node is launched**.

---

## The Server Pattern: Adapter, Not Logic

The service server acts as a **strict adapter**.

Its responsibilities are limited to:

1. Receiving a `ComputeStats_Request`
2. Delegating to `compute(&request.data)`
3. Translating `StatsResult` → `ComputeStats_Response`
4. Logging results

The server **does not** perform calculations or enforce business rules.

### Service Callback Form

In `rclrs` v0.6.x, the service callback uses the single-argument form:

```rust
move |request: ComputeStats_Request| -> ComputeStats_Response { ... }
```

This keeps the callback minimal and aligns with the executor-managed lifecycle.

---

## The Client Pattern: Asynchronous and Executor-Safe

Although services are conceptually synchronous, their implementation **must be asynchronous** to avoid blocking the executor.

### Design Constraints

* The executor must continue spinning to deliver responses.
* Blocking calls inside callbacks or the main thread can stall the node.
* The client must integrate cleanly with the event loop.

### Implemented Pattern

1. **Discovery**
   The client waits for the service using `service_is_ready()`.

2. **Dispatch**
   The request is sent using `call_then(...)`, which registers a response callback.

3. **Execution**
   `executor.spin(...)` drives the callback when the response arrives.

This guarantees:

* No deadlocks
* No busy waiting
* No thread management in user code

The executor remains the single coordination mechanism.

---

## Type Safety and Explicitness

Rust’s type system enforces clarity in asynchronous callbacks.

In the client, the response type is explicitly annotated:

```rust
move |response: ComputeStats_Response| { ... }
```

This removes ambiguity in generic contexts and makes callback intent explicit — an important practice in production Rust code.

---

## Error Handling and Executor Control

Executor shutdown and error handling use the `RclrsErrorFilter` trait:

```rust
.spin(...)
.ignore_non_errors()
.first_error()
```

This pattern ensures:

* Clean shutdown on fatal errors
* Signal handling remains centralized
* Middleware noise does not terminate the process prematurely

---

## Configuration via Shared Utilities

Service names are resolved via `utils_rclrs::services`, not hardcoded strings.

Benefits:

* Single source of truth
* Consistent naming across server, client, and CLI
* Safe refactoring without touching node logic

---

## Why This Lesson Matters

Lesson 04 establishes a **repeatable service architecture** in Rust:

* Business logic is verified through unit tests
* ROS nodes are thin, predictable adapters
* Clients are asynchronous and executor-safe
* Configuration is centralized
* The system scales without increasing complexity

This pattern forms the foundation for all higher-level service-based systems, including orchestration layers, diagnostics services, and supervisory control.
