# Lesson 03 Breakdown: Subscribers, QoS, and Stream Resynchronisation

## Architectural Intent

Lesson 03 marks the transition from *learning ROS mechanics* to *validating system behaviour under real conditions*.

At this stage:

* Node construction is assumed knowledge
* The lesson objective is **verification and robustness**, not feature development
* Silence is a valid runtime state

The subscriber exists to answer a single question:

> “Does this node correctly receive, interpret, and recover from data produced elsewhere in the system?”

---

## Core Concept: Language-Agnostic Communication

The most important outcome of this lesson is not that a subscriber works, but that it works **regardless of the publisher’s language**.

When a Rust publisher from Lesson 02 successfully drives a Python subscriber in Lesson 03, the following are proven:

* DDS serialization is compatible
* Interface definitions are shared correctly
* Topic naming is consistent
* QoS policies are aligned

This is the practical foundation of multi-language ROS systems.

---

## QoS Matching as a First-Class Concern

ROS 2 nodes can fail silently when QoS policies are incompatible.

Common failure modes include:

* Reliable subscriber vs Best Effort publisher
* Volatile subscriber vs Transient Local publisher
* History depth mismatches under load

To avoid these failures, this lesson **does not configure QoS in code**.

```python
self.qos_profile = qos.telemetry(self)
```

The same configuration is used by:

* Python publishers
* C++ publishers
* Rust publishers
* Python subscribers

Compatibility is guaranteed by configuration, not convention.

---

## The Pattern: Logic Injection

ROS callbacks should not contain business logic or stateful transport assumptions.

Lesson 03 uses **logic injection**:

```text
Lesson03Node (ROS resources)
 └── ChatterListener (pure logic)
```

The subscription callback is a thin router:

```python
self.create_subscription(..., self._listener, ...)
```

This pattern:

* isolates ROS resources from logic
* keeps the listener ROS-agnostic
* allows the logic to be tested independently
* avoids refactoring when executors become multithreaded
* aligns Python with C++ and Rust architectural expectations

Python uses a callable object rather than a heavyweight component to keep the pattern idiomatic and lightweight.

---

## Message Stream Awareness (Ordering, Resets, Recovery)

The listener tracks an expected counter value to reason about the message stream.

It explicitly handles three cases:

### 1. Late Joiners

On first contact, the subscriber has no historical context.

```text
Received (initial): N
```

The first observed value defines the baseline.

---

### 2. Publisher Resets

If the counter suddenly drops to a small value (e.g. `0` or `1`) after normal operation, the listener treats this as a **publisher restart or manual injection**.

```text
Detected counter reset. Re-syncing at: 1
```

This avoids permanent error states when:

* a publisher is restarted
* a CLI message is injected mid-stream
* a system is partially rebooted

---

### 3. Genuine Out-of-Order Data

If a message arrives below the expected value and does **not** match the reset heuristic, it is flagged as invalid.

```text
Out-of-order/invalid: 3 < 10
```

This indicates a real transport or configuration issue.

---

This logic is **not business logic**.
It is transport and system-integration validation.

---

## Subscriber Behaviour: Silence Is Correct

Unlike publishers or timers, subscribers may legitimately produce no output for long periods.

This is intentional.

A silent subscriber indicates:

* no data is available
* no unnecessary work is being done
* the system is behaving correctly

Lesson 03 normalises this behaviour early, preventing “heartbeat spam” anti-patterns later.

---

## Code Walkthrough

### 1. Subscription Creation

```python
self._subscriber = self.create_subscription(
    MsgCount,
    self.topic_name,
    self._listener,
    self.qos_profile,
)
```

Key points:

* Message type is shared (`MsgCount`)
* Callback is a logic object
* QoS compatibility is enforced externally

---

### 2. Listener Responsibilities

```python
class ChatterListener:
```

The listener:

* owns stream validation state
* detects resets and re-syncs
* logs integration-relevant events
* has **no ROS dependencies**

This keeps it portable and predictable.

---

### 3. Clean Shutdown

The node follows the same shutdown pattern as Lesson 01 and 02:

* catch `KeyboardInterrupt`
* destroy the node
* shut down `rclpy`

No new lifecycle semantics are introduced here.

---

## Why This Lesson Matters

Lesson 03 establishes habits that prevent real-world ROS failures:

* treating QoS as configuration, not syntax
* validating transport assumptions early
* tolerating restarts and partial system resets
* separating logic from middleware
* proving multi-language interoperability before complexity increases

From this point forward, lessons build on **behavioural guarantees**, not API familiarity.
