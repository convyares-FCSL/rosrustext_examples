# Lesson 04 Breakdown: Services, Logic Separation, and Testability (roslibrust)

## Architectural Intent

Lesson 04 introduces **request–response semantics** into the roslibrust track.

Where Lesson 03 verified **continuous data flow** across the bridge, Lesson 04 verifies **transactional correctness**:

> A request issued outside the ROS 2 graph must reach a native ROS service,
> execute deterministic logic,
> and return a valid response without blocking or deadlocking the system.

The key objective is **testability**: proving that meaningful system logic can be validated *without* running ROS, while transport correctness is verified separately.

---

## What’s New in Lesson 04

Compared to Lesson 03, three new system concepts are introduced:

1. **Services (Request–Response)**  
   Unlike topics, services represent discrete transactions with a defined lifecycle:
   request → compute → response.

2. **Logic Separation**  
   Business logic is extracted into a pure Rust library (`lib.rs`) with zero ROS or bridge dependencies.

3. **Async Client–Server Coordination**  
   Service calls must remain non-blocking and tolerate startup ordering differences between client, server, and bridge.

---

## Component Boundary: Adapter Pattern

Lesson 04 applies the same architectural rule used in previous lessons:

> **Transport code adapts data; it does not own logic.**

The system is divided into three explicit layers:

```

Client (roslibrust)
└── Transport adapter (service call)
└── ROS graph
└── Transport adapter (service callback)
└── Pure logic (lib.rs)

````

### 1) Pure Logic (`lib.rs`)

The `compute()` function:

* accepts a Rust slice (`&[f64]`)
* returns a `StatsResult` struct
* contains no ROS types
* contains no async code
* is fully testable via `cargo test`

This ensures that algorithm correctness is independent of transport correctness.

---

### 2) Service Server (roslibrust)

The server is a **thin adapter**:

* Deserialises `ComputeStatsRequest`
* Delegates computation to `compute(&data)`
* Serialises `ComputeStatsResponse`
* Logs transport-level events only

The server owns a `ServiceHandle`. Holding this handle is what keeps the service advertised; dropping it removes the service from the graph. This explicit ownership mirrors how real middleware lifetimes behave.

---

### 3) Service Client (roslibrust)

The client is also an adapter:

* Constructs a request
* Calls the service asynchronously
* Handles retry until the service becomes available
* Processes the response without blocking the runtime

Because rosbridge does not expose DDS-level service discovery, readiness is handled pragmatically: failed calls trigger retry with backoff. This reflects real-world distributed startup behaviour.

---

## Async Model: Why Nothing Blocks

In contrast to synchronous service calls:

* All I/O runs on the Tokio runtime
* The service callback executes inside rosbridge’s async context
* The client awaits the response without blocking the executor

This ensures:

* No deadlocks
* No stalled event loops
* Safe coexistence with other async tasks (timers, subscriptions, future extensions)

---

## Configuration: Services as Shared Contracts

Service naming is centralised via `utils_roslibrust::services`:

```rust
let service_name = services::compute_stats();
````

This mirrors Lesson 03’s pattern:

```rust
let topic_name = topics::CHATTER;
```

Key differences from `utils_rclrs`:

* **roslibrust** uses compile-time constants
* **rclrs** resolves names via ROS parameters

The call site remains identical. Only the resolution mechanism differs, preserving conceptual symmetry across lessons and languages.

---

## Why Unit Tests Matter Here

By extracting logic into `lib.rs`, Lesson 04 proves that:

* The algorithm can be validated **without ROS**
* Edge cases (empty input, single element, float handling) are verified deterministically
* Transport bugs and logic bugs are no longer entangled

This enables:

* Faster feedback loops
* CI-friendly testing
* Confidence that runtime failures are integration issues, not math errors

---

## Cross-Language Verification

Once running, the service can be called by:

* ROS 2 CLI
* roslibrust client
* Native C++ and Python nodes

This validates:

* Service type compatibility
* JSON ↔ DDS translation correctness
* End-to-end graph interoperability

---

## What This Lesson Proves

When Lesson 04 succeeds, you have demonstrated:

1. **Logic Independence**
   Business logic is portable, testable, and framework-agnostic.

2. **Bridge Transparency**
   Rosbridge correctly transports service requests and responses.

3. **Async Safety**
   Request–response interactions do not block or destabilise the runtime.

4. **Professional Structure**
   The system follows the same separation of concerns expected in production robotics and distributed systems.

Lesson 04 marks the transition from *“nodes that talk”* to *“systems that can be reasoned about.”*

