# ROS 2 Communication Benchmarking (Optional Study)

This directory contains **optional benchmarking material** comparing ROS 2 communication behaviour across languages.

This is **not a lesson**. It is an engineering study intended to inform trade-offs.

---

## Purpose

The goal is to explore:
- **Latency** (min / mean / max)
- **Jitter** (“wiggle”)
- **Throughput**
- **CPU behaviour**

Across:
- Python (`rclpy`)
- C++ (`rclcpp`)
- Rust (`rclrs`)
- External clients (`roslibrust` via rosbridge)

The intent is **not** to declare winners, but to understand where each approach fits.

---

## Important Caveats

Benchmark results are:
- Highly environment-dependent.
- Sensitive to CPU load, DDS vendor, QoS, and OS scheduling.
- Easy to misinterpret without context.

Do **not** treat these numbers as absolute truth. Treat them as **signals**, not guarantees.

---

## Benchmark Structure

```text
5_benchmark/
├─ publisher/
│   ├─ python
│   ├─ cpp
│   └─ rclrs
│
├─ subscriber/
│   ├─ python
│   ├─ cpp
│   └─ rclrs
│
├─ rosbridge_client/
│
├─ config/
│   ├─ qos_profiles.yaml
│   └─ benchmark_params.yaml
│
└─ results/

```

---

## Test Scenarios

Each benchmark should be run under multiple conditions:

### 1. Message Size

* **Small**: `std_msgs/String` (minimal overhead)
* **Medium**: Structured message (serialization cost)
* **Large**: Array / Image payload (throughput limits)

### 2. Frequency

* **Low**: 1–10 Hz
* **Medium**: 50–100 Hz
* **High**: 500+ Hz (stress testing)

### 3. QoS

* **Best-Effort** (UDP-like)
* **Reliable** (TCP-like mechanism)

---

## Expected Trends (Signals, not Guarantees)

| Language | Primary Strength | Primary Trade-off |
| --- | --- | --- |
| **Python** (`rclpy`) | Fast iteration, easy instrumentation. | Higher jitter (GC pauses), unsuitable for tight control loops. |
| **C++** (`rclcpp`) | Lowest latency floor, mature tooling. | Complexity, harder to ensure memory safety under pressure. |
| **Rust** (`rclrs`) | Predictable memory, strong correctness guarantees. | Smaller ecosystem, overhead from safety abstractions. |
| **Bridge** (`roslibrust`) | Web/External integration, no build coupling. | Higher latency (JSON serialization), no native ROS lifecycle. |

---

## How to Use This Material

Use this benchmark to answer:

* “Is Python good enough for this 100Hz loop?”
* “What is the cost of crossing the network bridge via JSON?”
* “Where does jitter start to matter in my application?”

**Do NOT use this to ask:**

* “Can I replace C++ with X everywhere?”

---

## Relationship to Lessons

This benchmark assumes understanding of publishers, subscribers, QoS profiles, and executors. It intentionally lives **outside** the lesson sequence.