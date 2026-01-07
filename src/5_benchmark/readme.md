
# ROS 2 Communication Benchmarking (Optional Study)

This directory contains **optional benchmarking material** comparing
ROS 2 communication behaviour across languages.

This is **not a lesson**.
It is an engineering study intended to inform trade-offs.

---

## Purpose

The goal is to explore:

- latency (min / mean / max)
- jitter (“wiggle”)
- throughput
- CPU behaviour

across:

- Python (`rclpy`)
- C++ (`rclcpp`)
- Rust (`rclrs`)
- External clients (`roslibrust` via rosbridge)

The intent is **not** to declare winners, but to understand where each approach fits.

---

## Important Caveats

Benchmark results are:

- highly environment-dependent
- sensitive to CPU load, DDS vendor, QoS, and OS scheduling
- easy to misinterpret without context

Do **not** treat these numbers as absolute truth.
Treat them as **signals**, not guarantees.

---

## Benchmark Structure

```text
5_benchmark/
├─ publisher/
│  ├─ python
│  ├─ cpp
│  ├─ rclrs
│
├─ subscriber/
│  ├─ python
│  ├─ cpp
│  ├─ rclrs
│
├─ rosbridge_client/
│
├─ config/
│  ├─ qos_profiles.yaml
│  └─ benchmark_params.yaml
│
└─ results/
````

---

## Test Scenarios

Each benchmark should be run under multiple conditions:

### Message Size

* small (e.g. `std_msgs/String`)
* medium (structured message)
* large (array / payload)

### Frequency

* low rate (1–10 Hz)
* medium rate (50–100 Hz)
* high rate (500+ Hz where feasible)

### QoS

* best-effort
* reliable
* transient local (where applicable)

---

## Metrics Collected

* End-to-end latency
* Jitter (variance between samples)
* Message loss
* CPU utilisation (coarse)
* Allocation behaviour (where observable)

Timestamps should be taken **at the application level**, not DDS internals.

---

## Expected High-Level Trends (Not Guarantees)

### Python (`rclpy`)

**Strengths**

* Fast development
* Easy instrumentation

**Trade-offs**

* Higher jitter
* GC pauses
* Unsuitable for tight control loops

---

### C++ (`rclcpp`)

**Strengths**

* Lowest latency floor
* Best executor and QoS control
* Mature tooling

**Trade-offs**

* Complexity
* Harder to reason about correctness under pressure

---

### Rust (`rclrs`)

**Strengths**

* Strong correctness guarantees
* Predictable memory behaviour
* Good balance of performance and safety

**Trade-offs**

* Smaller ecosystem
* Fewer tuning examples
* Some overhead from safety abstractions

---

### roslibrust (rosbridge)

**Strengths**

* External integration
* No ROS build or runtime coupling
* Ideal for tooling and dashboards

**Trade-offs**

* Higher latency
* No lifecycle or ROS logging
* Not suitable for real-time paths

---

## How to Use This Material

Use this benchmark to answer questions like:

* “Is Python good enough for this role?”
* “Where does Rust sit between Python and C++?”
* “What happens when I cross a network boundary?”
* “Where does jitter start to matter?”

Not:

* “Which language is best?”
* “Can I replace C++ with X everywhere?”

---

## Relationship to Lessons

This benchmark assumes understanding of:

* publishers and subscribers
* QoS profiles
* executors
* parameters

It intentionally lives **outside** the lesson sequence.

---

## Non-Goals

* Benchmarking DDS vendors
* Proving one language superior
* Replacing real system profiling

This is an educational comparison, not a certification exercise.