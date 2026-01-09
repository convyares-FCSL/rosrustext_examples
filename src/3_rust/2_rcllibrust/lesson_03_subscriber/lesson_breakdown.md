# Lesson 03 Breakdown: Subscribers, System Verification, and Async Streams (roslibrust)

## Architectural Intent

Lesson 03 is the point where “my node runs” stops being the goal.

The goal is:

> **Prove that the Bridge behaves correctly** as a transparent translation layer between native ROS 2 nodes and external Rust applications.

A subscriber is the best verification tool here because it validates the full pipeline:
Native Publisher $\to$ DDS $\to$ `rosbridge_server` $\to$ JSON $\to$ `roslibrust`.

---

## What’s New in Lesson 03

Compared to Lesson 02, the mechanics differ in three key ways:

1.  **Async Streams**: We consume messages via a `futures::Stream` loop rather than registering a passive callback.
2.  **State Management**: Validating stream continuity (ordering + resets) inside a concurrent background task.
3.  **Manual Schema Validation**: Ensuring our manual `serde` struct matches the actual ROS 2 message definition.

---

## Component Boundary: Subscriber as a “Unit of Integration”

Lesson 02 introduced the separation of Logic and Resource logic.

Lesson 03 applies the same rule:

* The **Node container** (`Lesson03Node`) exists to own the client connection and keep the component alive.
* The **Subscriber component** (`SubscriberChatter`) is a self-contained unit. It owns the validation state and the handle to the background processing task.

This distinction is critical in async Rust. If the Node struct grows too large, passing parts of it into async tasks becomes a nightmare of lifetime errors. By isolating the subscriber into its own struct, we make the ownership clear.

---

## Why the Component Owns a Task Handle

In modern `roslibrust` (and async Rust in general), subscriptions are often represented as **Streams**—infinite lists of values that arrive over time.

* **Creation**: `client.subscribe(...)` returns a Stream.
* **Processing**: To process a stream without blocking the main thread, we must **spawn** a Tokio task.
* **Ownership**: The `SubscriberChatter` struct holds `_task` (`tokio::task::JoinHandle`).

This `_task` field acts as the **lifetime anchor**. It represents the "running subscription." (Note: In strict Tokio terms, detaching a handle keeps the task running, but holding it allows for future cancellation or graceful shutdown logic).

---

## Rust-Specific Constraint: Shared State in Async Tasks

The most common hurdle in async Rust is sharing state between the main application and a background task.

1.  **The Environment**: The loop `while let Some(msg) = stream.next().await` runs inside a spawned Tokio task.
2.  **The Constraint**: The task needs to update state (`expected`, `initialized`), but that state might also need to be read by the main thread (e.g., for status checks or shutdown).

**The Solution: `Arc<Mutex<StreamState>>`**

* **`Arc`**: Allows multiple owners. We need one owner in the struct (`_state`) and a second owner (the clone `state_cb`) moved into the async task.
* **`Mutex`**: Provides safe mutable access. Even though the stream is processed sequentially, the state is owned by a structure that *could* be accessed concurrently.

---

## Stream Validation: What We’re Actually Checking

The logic inside `on_msg` is **identical** to the native `rclrs` track. This is intentional.

We are validating that the transport layer (WebSocket/JSON) has not introduced artifacts. We check for:

### 1) Late joiners
Does the bridge correctly forward the stream to a client that connects *after* the publisher started?
* *Policy:* First received message sets the baseline.

### 2) Publisher restarts
If the C++ publisher crashes and restarts (resetting its counter to 1), does the Rust client handle the sudden drop in value?
* *Policy:* A drop in value (within `reset_max_value` limits) triggers a re-sync.

### 3) Serialization Compatibility
Since we manually defined `MsgCount` using `serde`, does it actually match the binary layout of the C++ publisher?
* *Policy:* If deserialization fails, `serde` would error before we even reach the logic. If values look garbage, we log warnings.

---

## Why the Checks Live as Named Helpers

The production pattern is:

* **Thin Loop**: The async loop does only one thing—acquire the lock and delegate.
* **Pure Logic**: `on_msg` and its helpers (`handle_reset`, `handle_initial`) contain the business logic.

This makes the code readable and testable. You could technically unit test `on_msg` without spinning up a full ROS bridge, simply by passing in a dummy `StreamState`.

---

## Configuration: Topics and QoS

Even though we are external to the ROS graph, we must respect its configuration.

* `topics::CHATTER`: We use the workspace library to ensure we are listening to the exact string the publishers are using.
* **QoS**: While `rosbridge` abstracts some DDS QoS complexity, utilizing shared configuration ensures that if we later change the topic name in the workspace, this external client updates automatically upon re-compilation.

---

## What This Lesson Proves

When this node runs successfully against a C++ publisher, you have proven:

1.  **Cross-Language Serde**: Rust `serde` JSON $\leftrightarrow$ ROS Bridge $\leftrightarrow$ C++ DDS.
2.  **Transport Stability**: The WebSocket connection is stable enough to maintain message ordering.
3.  **Graph Interop**: Your external Rust app is now a first-class citizen in the ROS 2 graph.