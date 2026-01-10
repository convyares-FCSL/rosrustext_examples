# Lesson 04 Breakdown: Services, Async Patterns, and Testability

## Architectural Intent

Lesson 04 introduces the **Request-Response** paradigm, which differs fundamentally from the continuous data streams seen in Topics (Pub/Sub).

The goal is to move beyond "making it work" and focus on **"making it verifyable"**.

We introduce two critical engineering standards:
1.  **Logic Separation**: Decoupling the "Math" from the "Middleware".
2.  **Asynchronous Client Design**: Preventing deadlocks in event-driven systems.

---

## The Pattern: Logic Separation

A common anti-pattern in ROS 2 development is the "Fat Node," where complex algorithms are written directly inside the callback function.

```python
# ANTI-PATTERN
def callback(self, req, res):
    # Complex math mixed with ROS logging and message types
    if len(req.data) == 0: ... 
```

Lesson 04 refactors this using **Composition**:

```text
ServiceNode (ROS Wrapper)
 └── StatsLogic (Pure Python)
```

### Why this matters:

* **Testability**: `StatsLogic` is a standard Python class. It imports **no ROS libraries**. This means you can unit test it with standard tools (`pytest`) instantly, without needing a valid ROS environment, message definitions, or a build system.
* **Portability**: The same logic class could be dropped into a web server, a CLI tool, or a Jupyter notebook without modification.

---

## Unit Testing as a First-Class Citizen

Because we separated the logic, testing becomes trivial.

```python
def test_compute_empty_list():
    data = []
    total, avg, status = StatsLogic.compute(data)
    assert status == "Warning: No data provided. Returning 0."
```

This test runs in milliseconds. If we had buried this logic inside the `ServiceNode`, we would need to:

1. Launch the node.
2. Create a mock service client.
3. Send a request.
4. Wait for the asynchronous response.
5. Check the result.

By separating logic, we verify the **algorithm** (`StatsLogic`) independently of the **transport** (`ServiceNode`).

---

## Asynchronous Client Design

Services are conceptually synchronous (Request  Wait  Response), but their implementation in ROS 2 **must be asynchronous**.

### The Danger of Blocking Calls

If a node calls `client.call(req)` (blocking) inside a callback or the main thread, it stops processing all other events (timers, subscriptions) until the server responds. If the server is down or slow, the entire node freezes.

### The Solution: `call_async`

Lesson 04 demonstrates the robust pattern:

1. **Send**: `future = client.call_async(req)`
2. **Register**: `future.add_done_callback(callback)`
3. **Continue**: The node remains responsive while waiting.

```python
# Non-blocking execution
self._future = self._client.call_async(req)
self._future.add_done_callback(self.response_callback)
```

This ensures the client application never hangs, even if the network is unreliable.

---

## Service Configuration

Just like Topics in Lesson 03, Service names are not hardcoded.

```python
self.service_name = services.compute_stats(self)
```

* **Definition**: `utils_py`
* **Usage**: Both Server and Client

This guarantees that if the system architect decides to rename the service to `/telemetry/stats`, a single change in `utils_py` updates the entire system.

---

## Code Walkthrough

### 1. The Pure Logic (`StatsLogic`)

It accepts standard types (`list`) and returns standard types (`tuple`). It knows nothing about `ComputeStats.Request` or `ComputeStats.Response`.

### 2. The Server Adapter (`ServiceNode`)

The node acts as a **Translator**:

1. **Unpack**: Extracts data from the ROS Request.
2. **Compute**: Calls `StatsLogic`.
3. **Pack**: Fills the ROS Response.

### 3. The Client (`ClientNode`)

Demonstrates the lifecycle of a discrete task:

* Wait for service (Discovery).
* Send request (Action).
* Handle result (Reaction).
* Shutdown (Completion).

---

## Why This Lesson Matters

Lesson 04 proves that **testable code is better code**.

By isolating the "business logic" (the statistics math), we created a system where:

1. The math is verified via Unit Tests.
2. The communication is verified via Integration Tests (CLI interaction).
3. The configuration is centralized.

This separation of concerns is the hallmark of professional distributed systems engineering.