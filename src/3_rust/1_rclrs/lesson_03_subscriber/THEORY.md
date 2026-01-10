# Lesson 03 Breakdown: Subscribers, QoS, and System Verification (Rust / rclrs)

## Architectural Intent

Lesson 03 is the point where “my node runs” stops being the goal.

The goal is:

> **Prove that the ROS graph behaves correctly** across languages and across real operator workflows (late joiners, restarts, manual injections).

A subscriber is the best verification tool because it sits on the boundary between “my code” and “the system”.

---

## What’s New in Lesson 03

Compared to Lesson 02, the mechanics aren’t new — the emphasis is.

Lesson 03 introduces three production concerns:

1. **QoS compatibility as configuration**
2. **Subscriber correctness under real graph conditions**
3. **Stream validation (ordering + reset tolerance)**

Everything else is established baseline.

---

## Component Boundary: Subscriber as a “Unit of Integration”

Lesson 02 introduced the idea that a publisher shouldn’t bloat the node struct.

Lesson 03 applies the same rule:

* The **Node container** exists to own lifecycle and keep handles alive.
* The **Subscriber component** is a self-contained unit: it owns the subscription handle and the stream-validation state and exposes one entrypoint (`on_msg`).

That structure matters because subscribers are where complexity accumulates first:
filters, transforms, health checks, latching behaviors, metrics, backpressure, etc.

If the subscriber starts life embedded directly in the node, it’s painful to grow later.

---

## Why the Subscriber Component Owns the Subscription Handle

In `rclrs`, if you drop the subscription handle, the subscription stops existing.

So the component holds:

* `_sub`: keep-alive handle for the DDS subscription
* `_state`: keep-alive handle for shared state
* `_logger`: stable logger handle used by both node and callback

The underscore naming is intentional: these fields are not “business data”, they are *lifetime anchors*.

---

## Rust-Specific Constraint: Callbacks Need Interior Mutability

The important Rust-specific reality:

* The callback runs from inside the executor.
* The closure you pass to `create_subscription` is typically called via `Fn`, not `FnMut`.
* That means you cannot safely mutate normal struct fields through `&self` inside the callback.

So Lesson 03 uses:

* `Arc<Mutex<StreamState>>`

This is not “because Rust is hard”.
It is the correct tradeoff for production:

* simple
* explicit
* safe under concurrency
* matches future executor evolution (multi-threaded / callback groups later)

You can swap `Mutex` for a lock-free approach later if you actually need it.
Default to correctness.

---

## Stream Validation: What We’re Actually Checking

The incoming message stream is assumed to be a monotonically increasing counter (`MsgCount.count`).

The subscriber validates that assumption at runtime to detect:

### 1) Late joiners

A subscriber may start after the publisher, so the first value is arbitrary.

Policy:

* first message initializes the baseline (`expected = first + 1`)
* logs `Received (initial)`

### 2) Publisher restarts / manual injection

A restarted publisher commonly resets the counter to `0` or `1`.
A human using CLI tools will also inject low values.

Policy:

* if `count` drops below expected and is “small enough” (≤ `reset_max_value`)
* treat it as a reset and resync (`expected = count + 1`)
* logs `Detected counter reset...`

### 3) Out-of-order / stale samples

If `count < expected` and does *not* look like a reset, it’s likely:

* QoS mismatch causing drops/replays
* multiple publishers on same topic
* ordering issues under load
* stale cache / unexpected topology

Policy:

* log warning
* do not update expected (don’t let bad data shift your baseline)

These rules are the same conceptual behavior you implemented in Python and C++ — Rust just forces you to be explicit about state access.

---

## Why the Checks Live as Named Helpers

The production pattern is:

* One authoritative entrypoint: `on_msg`
* Small named helpers for each decision gate

This makes the logic readable without tutorial comments:

* `extract_count`
* `handle_initial`
* `handle_reset`
* `handle_out_of_order`

The code reads like a state machine, not a blob of branching.

That matters because subscribers are frequently edited under pressure during commissioning.

---

## QoS Matching: Why the Subscriber Loads QoS the Same Way as the Publisher

In ROS 2, the most common silent failure is QoS mismatch.

Lesson 03 makes QoS a configuration concern by reusing the same shared config path:

* topic name: `topics::chatter(node)`
* QoS profile: `qos::from_parameters(node)`

This guarantees that:

* if the system works in one language, it works in the others
* compatibility is not “maintained by habit” but enforced by shared configuration

If comms fail, the debugging focus is clear:

* inspect `ros2 topic info -v /chatter`
* compare QoS profiles
* don’t hunt string literals in code

---

## What This Lesson Proves (When It Works)

When your Rust subscriber receives data from a C++ or Python publisher (or vice versa), you’ve validated:

* `MsgCount` is generated correctly across languages
* DDS serialization/deserialization is interoperable
* topic naming is consistent
* QoS compatibility is real, not assumed
* the system tolerates real operator behavior (late joiners, restarts, manual CLI testing)

That’s the baseline you need before you add services/actions/executors, because those features amplify any transport/config mistakes.

---

## Reinforced Concepts

Lesson 03 assumes and reinforces:

* RAII keep-alive handles (subscriptions must be stored)
* shared configuration (topic + QoS)
* non-blocking callbacks (callback delegates to logic)
* explicit state ownership (Rust’s concurrency rules are not optional)

From here on, the Rust track is not “special”: it’s just another first-class node in the graph.
