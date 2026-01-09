# Lesson 03 Breakdown: Subscribers, QoS, and System Verification (C++)

## Architectural Intent

Lesson 03 transitions the system from **data production** to **data verification**.

In Lesson 02, the publisher emitted structured data into the ROS graph.
In Lesson 03, the subscriber exists to answer a harder question:

> *Does this data arrive correctly under real system conditions?*

This lesson validates behaviour, not mechanics.

---

## What’s New in Lesson 03

Lesson 03 introduces three system-level concerns:

1. **Language-Agnostic Verification**
   The subscriber must accept data produced by publishers written in *any* supported language.

2. **QoS as a Compatibility Contract**
   Communication success depends on matching QoS policies, not API correctness.

3. **Transport-Aware Validation**
   The node actively detects late joins, restarts, and invalid message sequences.

No new ROS APIs are introduced.

---

## Subscriber as a Verification Tool

Unlike publishers, subscribers may legitimately remain silent.

In Lesson 03, silence is not an error state — it is expected behaviour until data is available.

When data *does* arrive, the subscriber verifies:

* message ordering
* continuity of the stream
* tolerance to publisher restarts
* absence of stale or duplicated samples

The subscriber is therefore a **diagnostic component**, not a data sink.

---

## Logic Injection: Separating ROS from Validation

Lesson 03 enforces a strict separation between:

```text
Lesson03Node       → ROS resources (subscription, QoS, lifecycle)
MessageListener    → Message stream validation logic
```

The subscription callback is a **router only**:

```cpp
subscriber_ = this->create_subscription<MsgCount>(
  topic_name,
  qos_profile,
  [this](MsgCount::SharedPtr msg) {
    listener_->on_message(*msg);
  }
);
```

This prevents business or validation logic from leaking into ROS callbacks and keeps the node safe for future executor changes.

---

## Late Joiners and Initialization

The listener intentionally performs **lazy initialization**.

```cpp
if (!initialized_) {
  init_from(count, true);
  return;
}
```

This supports a common production scenario:

* Subscriber starts after the publisher
* First observed message defines the baseline
* No assumptions are made about initial counter values

The system does not require synchronized startup.

---

## Publisher Restart Detection

Lesson 03 tolerates publisher restarts and manual message injection.

```cpp
if (count <= reset_max_value_ && count < expected_count_) {
  init_from(count, false, true);
  return;
}
```

This allows the subscriber to:

* recover automatically from publisher restarts
* remain usable during development and testing
* avoid false-positive error states

Reset detection is explicit and logged.

---

## Out-of-Order and Stale Data Detection

Messages that arrive with unexpected sequence numbers are flagged:

```cpp
if (count < expected_count_) {
  RCLCPP_WARN(...);
  return;
}
```

These warnings indicate:

* QoS incompatibility
* transport issues
* dropped samples under load
* incorrect publisher behaviour

They are **system signals**, not application errors.

---

## Shared Configuration as a System Contract

Lesson 03 continues the Lesson 02 rule:

```cpp
auto topic_name = topics::chatter(*this);
auto qos_profile = qos::telemetry(*this);
```

Topic names and QoS policies are never duplicated in node code.

This guarantees:

* subscriber–publisher compatibility
* cross-language interoperability
* centralized system tuning

When communication fails, configuration is the first place to look.

---

## Why Lesson 03 Exists

Lesson 03 prevents a class of failures that appear late in real systems:

* “It works in Python but not in C++”
* “The subscriber starts but sees nothing”
* “Restarting a node breaks the pipeline”
* “QoS mismatches cause silent data loss”

By validating these conditions early, later lessons can focus on coordination, lifecycle, and concurrency — not transport debugging.

---

## Reinforced Concepts

Lesson 03 assumes familiarity with:

* event-driven execution
* lambda-based callbacks
* shared configuration patterns
* RAII-based cleanup

These concepts are **applied**, not reintroduced.

---

## What Comes Next

With a verified, language-agnostic ROS graph in place, subsequent lessons can safely introduce:

* services
* parameters as runtime configuration
* lifecycle-managed nodes
* actions and long-running workflows

Lesson 03 establishes the behavioural baseline that makes those patterns reliable.
