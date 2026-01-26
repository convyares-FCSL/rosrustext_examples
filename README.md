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
- A professional reference style: explicit ownership, minimal glue, structure/testing introduced only when it pays rent.

</details>

<details>
<summary><strong>What this is not</strong></summary>

- Not a beginner ROS tutorial.
- Not a language tutorial.
- Not a prescribed “one true architecture”.
- Not a claim of a complete, fully-certified safety platform.

It shows problems you will hit and responses that restore control, with trade-offs made visible.

</details>

---

## The behavioural contract

Parity is judged by what the system does **from the outside**, not internal API similarity.

Examples:

- canonical lifecycle behaviour and tooling: `ros2 lifecycle get/set`
- action behaviour under load: goal / feedback / result / cancel responsiveness
- composition behaviour via standard tools: `ros2 component load/list/unload`
- graph visibility and introspection: `ros2 node/topic/service/action …`
- repeatable bring-up, verification, and clean shutdown (no graph residue)

---

## Workspace layout

The workspace is organised by language implementations and shared contracts. Expand only when you need orientation.

<details>
<summary><strong>src/ (click to expand)</strong></summary>

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
├─ 6_capstone/               # reference implementation of a larger coherent system
templates/                   # starter boilerplate
scripts/                     # build/test/verification automation
```
</details>

---

## Topics

Each Topic answers one engineering question. The summaries below are intentionally short; the intent lives in each Topic’s `INTENT.md` and `THEORY.md`.

<details open>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 00 — Bootstrap
    <br /><br />
  </summary>

**Goal**
Establish a repeatable build/run loop per language implementation.

**Focus**
Tooling, environment setup, logging, clean shutdown.

**Architecture**
Explicit ownership and predictable node lifecycle (no rituals, no residue).

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 01 — Event Loop
    <br /><br />
  </summary>

**Goal**
Run continuously without state becoming opaque over time.

**Focus**
Timers as the event loop; owned state mutation.

**Architecture**
A clear “tick” of reality: deliberate state transitions explainable from logs.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 02 — Publisher
    <br /><br />
  </summary>

**Goal**
Publish real data across languages without fragile guesses.

**Focus**
Custom interfaces (`.msg`), build + import parity, observable publication.

**Architecture**
Publication as a system contract: shared types and stable naming (not scattered literals).

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 03 — Subscriber & Verification
    <br /><br />
  </summary>

**Goal**
Prove the system is communicating and make silent graph failures diagnosable.

**Focus**
QoS compatibility, late joiners, restarts, common failure signatures.

**Architecture**
Subscriber as a verification instrument (detect/report/explain), not just a consumer.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 04 — Services
    <br /><br />
  </summary>

**Goal**
Add request/response without welding meaning to ROS callbacks.

**Focus**
Services, clean adapter boundaries, unit evidence when logic exists.

**Architecture**
Pure deterministic logic + thin ROS adapters (tests appear because they now pay rent).

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 05 — Parameters & Configuration
    <br /><br />
  </summary>

**Goal**
Make configuration explicit, shared, and safely changeable at runtime.

**Focus**
Parameters as a live control surface; validation and runtime updates.

**Architecture**
Shared config contract (YAML) with per-language typed accessors; no hidden coupling.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 06 — Lifecycle (Managed Nodes)
    <br /><br />
  </summary>

**Goal**
Stop “noisy scripts” and make nodes externally orchestratable components.

**Focus**
Lifecycle state machine; gated side effects; canonical tooling surfaces.

**Architecture**
Allocate on configure, produce effects only when active; observable state from the outside.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 07 — Actions
    <br /><br />
  </summary>

**Goal**
Model long-running cancellable work and expose the operational failure mode.

**Focus**
Actions (goal/feedback/result/cancel), deliberate executor starvation failure.

**Architecture**
Correct work can still kill operability; this Topic ends with controlled degradation.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 08 — Executors & Callback Groups
    <br /><br />
  </summary>

**Goal**
Restore responsiveness under load without changing the work.

**Focus**
Executor choice, callback group isolation, scheduling as availability.

**Architecture**
Change scheduling only; semantics remain identical; survivability becomes measurable.

</details>

<details>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 09 — Composition & Containers
    <br /><br />
  </summary>

**Goal**
Expose deployment-induced failure modes under shared process/executor fate.

**Focus**
Component containers, runtime load/list/unload, shared-fate behaviour.

**Architecture**
Change topology only; co-location reveals coupling and shutdown/order assumptions.

</details>

<details open>
  <summary style="font-size: 1.25em; font-weight: 650;">
    Topic 10 — Launch, Topology & Deployment Verification
    <br /><br />
  </summary>

**Goal**
Make bring-up, verification, and clean shutdown repeatable when topology matters.

**Focus**
Launch-driven bring-up, config discovery, deployment verification scripts.

**Architecture**
Topology becomes an engineering artifact; verification becomes executable evidence.

</details>

---

## Benchmarks

Benchmarks exist to answer one question *after* parity is established:

**Given identical observable behaviour, what do different language/runtime choices cost and buy us?**

`src/5_benchmark` contains latency/jitter and “under pressure” studies used to justify **right language for the right role** with evidence rather than preference.

---

## Capstone

The Capstone demonstrates mastery **without inventing new semantics**.

Located in `src/6_capstone`, it applies the contracts from Topics 00–10 to a realistic, coherent system. It serves as the final integration evidence:

* **Scope:** Multiple components with explicit orchestration.
* **Focus:** Controlled failure domains, configuration-driven behaviour, and clean shutdown.
* **Verification:** Exercises the deployment topology, not just the code.

---

## Verification philosophy

Verification grows only when it becomes the cheapest form of evidence:

* **Unit** evidence appears when logic is separable (Topic 04).
* **Integration** evidence appears when orchestration exists (Topic 06).
* **Deployment** evidence appears when topology and bring-up become dominant risk (Topic 10).

---

## Running the test/verification harness

|             Suite | Command                                |
| ----------------: | -------------------------------------- |
|               All | `./scripts/04_tests/run_all.sh`        |
|            Python | `./scripts/04_tests/run_python.sh`     |
|               C++ | `./scripts/04_tests/run_cpp.sh`        |
|      Rust (rclrs) | `./scripts/04_tests/run_rclrs.sh`      |
| Rust (roslibrust) | `./scripts/04_tests/run_roslibrust.sh` |

Logs land under `log/tests/`. The harness enforces graph hygiene (no residue, no ghost nodes).

---

## Configuration contract

System strings (topic/service/action names, QoS profiles) are treated as a shared contract.

* **Definitions:** `src/4_interfaces/topic_interfaces/config/`
* **Accessors:** `utils_py`, `utils_cpp`, `utils_rclrs`, `utils_roslibrust`

---

## Implementations (reference links)

This repo uses multiple implementations to validate the same observable behaviours. Upstream references:

* ROS docs (general, incl. rclpy/rclcpp): [https://docs.ros.org/](https://docs.ros.org/)
* `rclrs` (Rust DDS): [https://github.com/ros2-rust/ros2_rust](https://github.com/ros2-rust/ros2_rust)
* `rosbridge_suite`: [https://github.com/RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

---

## Templates and automation

* `templates/` — starter boilerplate for new packages
* `scripts/` — build/test/verification runners and utilities
