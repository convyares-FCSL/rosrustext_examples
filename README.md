# ros2-systems-operability

A pressure-driven, multi-language **systems reference** for building ROS 2 deployments that remain **observable, controllable, and verifiable** as complexity increases.

This repository is not a feature tour. It documents how real systems evolve:

> **pressure → operational failure mode → architectural response**

ROS primitives (interfaces, parameters, lifecycle, actions, executors, composition, launch) appear only when the system can no longer meet an operational goal without them.

---

## What this repo is (and is not)

<details open>
<summary><strong>What this is</strong></summary>

- A **language-neutral behavioural reference**, validated by **observable parity** under standard ROS tooling.
- A sequence of Topics that move from “it runs” to “it deploys” by confronting operational constraints: startup determinism, responsiveness under load, shared-fate deployment, repeatable bring-up and verification.
- A professional reference style: explicit ownership, minimal glue, structure and testing introduced only when they pay rent.

</details>

<details>
<summary><strong>What this is not</strong></summary>

- Not a beginner ROS tutorial.
- Not a language tutorial.
- Not a prescribed “one true architecture”.
- Not a claim of a complete, fully certified safety platform.

It shows problems you will hit and responses that restore control, with trade-offs made visible.

</details>

---

## The behavioural contract

Parity is judged by what the system does **from the outside**, not internal API similarity.

Examples of contract surfaces:

- canonical lifecycle behaviour and tooling (`ros2 lifecycle get/set`)
- action behaviour under load (goal / feedback / result / cancel responsiveness)
- composition behaviour via standard tools (`ros2 component load/list/unload`)
- graph visibility and introspection (`ros2 node/topic/service/action …`)
- repeatable bring-up, verification, and clean shutdown (no graph residue)

---

## Workspace layout

The workspace is organised by language implementations and shared contracts.
Expand only when you need orientation.

<details>
<summary><strong>Repository structure (click to expand)</strong></summary>

```text
src/
├─ 1_python/                 # rclpy topics + utilities
│   └─ utils_py/
├─ 2_cpp/                    # rclcpp topics + utilities
│   └─ utils_cpp/
├─ 3_rust/
│   ├─ 1_rclrs/              # native DDS nodes (Rust)
│   │   └─ utils_rclrs/
│   └─ 2_roslibrust/         # rosbridge/websocket clients (Rust)
│       └─ utils_roslibrust/
├─ 4_interfaces/
│   ├─ topic_interfaces/     # shared ROS interfaces + config contract
│   │   ├─ config/           # YAML naming/QoS contract
│   │   ├─ msg/
│   │   ├─ srv/
│   │   └─ action/
│   └─ rosidl_rust/          # Rust bindings support
├─ 5_benchmark/              # engineering benchmarks (evidence, not teaching)
├─ 6_capstone/               # integrated reference system
templates/                   # starter boilerplate
scripts/                     # build/test/verification automation
````

</details>

---

## Topics

Each Topic answers **one engineering question**.
Summaries here are deliberately short; the intent lives in each Topic’s
`INTENT.md` and `THEORY.md`.

<details open>
<summary style="font-size:1.25em;font-weight:650;">
Topic 00 — Bootstrap
<br><br>
</summary>

**Goal**
Establish a repeatable build/run loop per language implementation.

**Focus**
Tooling, environment setup, logging, clean shutdown.

**Architecture**
Explicit ownership and predictable node lifetime (no rituals, no residue).

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 01 — Event Loop
<br>
</summary>

**Goal**
Run continuously without state becoming opaque over time.

**Focus**
Timers as the event loop; owned state mutation.

**Architecture**
A clear “tick” of reality: deliberate state transitions explainable from logs.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 02 — Publisher
<br>
</summary>

**Goal**
Publish real data across languages without fragile guesses.

**Focus**
Custom interfaces (`.msg`), build + import parity, observable publication.

**Architecture**
Publication as a system contract: shared types and stable naming.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 03 — Subscriber & Verification
<br>
</summary>

**Goal**
Prove the system is communicating and make silent graph failures diagnosable.

**Focus**
QoS compatibility, late joiners, restarts, common failure signatures.

**Architecture**
Subscriber as a verification instrument, not just a consumer.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 04 — Services
<br>
</summary>

**Goal**
Add request/response without welding meaning to ROS callbacks.

**Focus**
Services, clean adapter boundaries, unit evidence when logic exists.

**Architecture**
Pure deterministic logic plus thin ROS adapters.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 05 — Parameters & Configuration
<br>
</summary>

**Goal**
Make configuration explicit, shared, and safely changeable at runtime.

**Focus**
Parameters as a live control surface; validation and runtime updates.

**Architecture**
Shared config contract (YAML) with typed accessors.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 06 — Lifecycle (Managed Nodes)
<br>
</summary>

**Goal**
Stop “noisy scripts” and make nodes externally orchestratable components.

**Focus**
Lifecycle state machine; gated side effects.

**Architecture**
Allocate on configure, produce effects only when active.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 07 — Actions
<br>
</summary>

**Goal**
Model long-running cancellable work and expose operational failure.

**Focus**
Actions; deliberate executor starvation.

**Architecture**
Correct work can still kill operability.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 08 — Executors & Callback Groups
<br>
</summary>

**Goal**
Restore responsiveness under load without changing the work.

**Focus**
Executor choice, callback group isolation.

**Architecture**
Scheduling changes; semantics remain identical.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 09 — Composition & Containers
<br>
</summary>

**Goal**
Expose deployment-induced failure modes under shared fate.

**Focus**
Component containers, runtime load/list/unload.

**Architecture**
Topology change only; co-location reveals coupling.

</details>

<details>
<summary style="font-size:1.25em;font-weight:650;">
Topic 10 — Launch, Topology & Deployment Verification
<br>
</summary>

**Goal**
Make bring-up, verification, and clean shutdown repeatable when topology matters.

**Focus**
Launch-driven bring-up, config discovery, deployment verification.

**Architecture**
Topology becomes code; verification becomes executable evidence.

</details>

---

## Language Notes

### Python (`rclpy`)

Python is used as the **orchestration and observability** track.

It prioritises clarity of behaviour, interaction with ROS control surfaces, and ease of provoking and observing failure modes.
When Python cannot express a canonical ROS capability, the limitation is documented rather than worked around.

*See:* `src/1_python/README.md` (development and iteration notes).

---

### C++ (`rclcpp`)

C++ is used as the **canonical feature surface**.

It exposes the full officially supported ROS API set (lifecycle, executors, composition, containers) and makes intended contracts explicit.
C++ is not “correct by definition”; it exists to show what the complete toolchain enables so parity can be judged honestly.

*See:* `src/2_cpp/README.md`.

---

### Rust

Rust is used as the **explicit architecture** track.

Ownership, construction order, and lifecycle boundaries must be expressed directly.
This makes architectural assumptions hard to hide and easy to audit.

Two Rust tracks are maintained:

* **rclrs** — Native DDS nodes built with `colcon`
* **roslibrust** — Async external clients via `rosbridge`

Rust does not redefine the contract; it enforces it.

*See:* `src/3_rust/README.md`.

---

### Reading the tracks

Tracks are not ranked and are not benchmarks.
Each exposes different truths under the same Topics.
Performance and role trade-offs are deferred intentionally.

---

## Benchmarks

Benchmarks are **post-arc evidence**, not teaching.

They answer one question *after* Topics 00–10 establish the behavioural contract:

**When observable behaviour is held constant, what do different runtime and topology choices change under pressure?**

`src/5_benchmark` contains repeatable measurements (latency, jitter, load response) used to make costs visible without turning the core Topics into a performance exercise.

---

## Capstone

The Capstone demonstrates mastery **without inventing new semantics**.

Located in `src/6_capstone`, it applies the established contract to a coherent system:

* explicit orchestration
* controlled failure domains
* configuration-driven behaviour
* deployment-level verification

---

## Verification philosophy

Verification appears only when it becomes the cheapest form of evidence:

* **Unit** — when logic is separable (Topic 04)
* **Integration** — when orchestration matters (Topic 06)
* **Deployment** — when topology dominates risk (Topic 10)

---

## Running the test / verification harness

|             Suite | Command                                |
| ----------------: | -------------------------------------- |
|               All | `./scripts/04_tests/run_all.sh`        |
|            Python | `./scripts/04_tests/run_python.sh`     |
|               C++ | `./scripts/04_tests/run_cpp.sh`        |
|      Rust (rclrs) | `./scripts/04_tests/run_rclrs.sh`      |
| Rust (roslibrust) | `./scripts/04_tests/run_roslibrust.sh` |

Logs land under `log/tests/`. The harness enforces graph hygiene.

---

## Configuration contract

System strings (topic/service/action names, QoS profiles) are treated as a shared contract.

* **Definitions:** `src/4_interfaces/topic_interfaces/config/`
* **Accessors:** `utils_py`, `utils_cpp`, `utils_rclrs`, `utils_roslibrust`

---

## References

Upstream projects referenced:

* ROS documentation: [https://docs.ros.org/](https://docs.ros.org/)
* `rclrs` (Rust DDS): [https://github.com/ros2-rust/ros2_rust](https://github.com/ros2-rust/ros2_rust)
* `rosbridge_suite`: [https://github.com/RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

---

## Templates and automation

* `templates/` — starter boilerplate
* `scripts/` — build, test, and verification runners
