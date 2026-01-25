# TODO — ROS 2 Systems Workspace

This file tracks **release-critical work**, **structural alignment**, and **verification status** across the multi-language ROS 2 systems workspace.

It is not a task backlog.
Items exist here only if they affect **correctness, consistency, or release readiness**.

---

## 1. Structural Baseline (Stable)

These items define the workspace shape and are considered **closed unless explicitly reopened**.

* [x] **Directory Layout Normalised**

  * `1_python / 2_cpp / 3_rust / 4_interfaces`
  * `lesson_XX_<topic>` naming retained (structural debt tracked separately)

* [x] **Bootstrap Lesson (00)**

  * Workspace builds and runs across all language tracks.

* [x] **Roadmap & Progression Documented**

  * Root `README.md` lists all sections.
  * Architectural progression is explicit (Node → Lifecycle → Actions → Executors → Composition → Deployment).

---

## 2. Shared Contracts & Configuration (Authoritative)

These are **non-optional contract surfaces** used to prevent semantic drift across sections and languages.

### Interfaces (`src/4_interfaces`)

* [x] **Canonical configuration files**

  * `topics_config.yaml`
  * `qos_config.yaml`
  * `services_config.yaml`

* [ ] **Validation**

  * Confirm all config files match *actual runtime usage* across Python.
  * Confirm no unused or misleading entries remain.

---

### Utilities (Contract Surfaces, Not Helpers)

> Utilities are treated as **behavioural contracts**, not convenience libraries.

#### Python

* [x] **`utils_py` exists and is used consistently**
* [x] Installed and importable via standard packaging
* [ ] **Documentation**

  * Explicitly state `utils_py` is part of the system contract
  * Clarify guarantees and non-guarantees (e.g. warnings vs enforcement)

#### C++

* [x] **`utils_cpp` structure exists**
* [ ] **Export & linkage verification**

  * Confirm headers and CMake exports allow clean `find_package(utils_cpp)`
* [ ] Parity review against Python contracts

#### Rust

* [x] **`utils_rclrs` functional**
* [x] **`utils_roslibrust` functional**
* [ ] **Logging parity**

  * Align log format semantics (where feasible) with ROS 2 native logging

---

## 3. Section (Lesson) Status Matrix

> This matrix reflects **verified state**, not intent.

**Legend**

* [x] Complete & verified
* [ ] Not started / not yet verified

| Section | Topic               | Python | C++ | rclrs | rcllibrust | Scripts |
| ------: | ------------------- | :----: | :-: | :---: | :--------: | :-----: |
|      00 | Bootstrap           |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      01 | Node / Timer        |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      02 | Publisher           |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      03 | Subscriber          |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      04 | Services            |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      05 | Parameters          |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|      06 | Lifecycle           |   [X]  | [X] |  [X]  |     [X]    |   [X]   |
|     06b | Lifecycle Manager   |   [X]  | [-] |  [-]  |     [-]    |   [-]   |
|      07 | Actions             |   [X]  | [X] |  [ ]  |     [ ]    |   [ ]   |
|      08 | Executors           |   [X]  | [ ] |  [ ]  |     [ ]    |   [ ]   |
|      09 | Composition         |   [X]  | [ ] |  [ ]  |     [ ]    |   [ ]   |
|      10 | Launch / Deployment |   [X]  | [ ] |  [ ]  |     [ ]    |   [ ]   |


* [ ]  Update C++ lesson 06 to carry clean timer pattern - see lesson 07
---

## 4. Review & Release Governance (New)

These items tie the workspace to the **Consolidated Review & Release-Readiness Audit**.

* [ ] Close all **Open** and **Answered (Implicit)** items from the latest audit snapshot
* [ ] Explicitly document all **Deferred (Intentional)** items
* [ ] Re-run the audit prompt after fixes
* [ ] Record snapshot date and outcome (no new Open items introduced)

This section must be **empty** before release.

---

## 5. Documentation Alignment (Release-Blocking)

Only items that affect **reader understanding or correctness** are tracked here.

* [ ] Align top-level Python README with:

  * INTENT (one systems question per section)
  * PHILOSOPHY (pressure, failure, honesty)
* [ ] Explicitly surface:

  * intentional failures
  * assumption-breaking across sections
* [ ] Ensure documentation does **not** smooth over tooling gaps

---

## 6. Structural Debt (Tracked, Not Fixed)

These items are acknowledged but **not release-blocking**.

* [ ] Terminology mismatch (“lesson” vs actual function)
* [ ] Long-term documentation consolidation
* [ ] Optional diagram pass (only where behaviour is clarified)

---

---

## 7. Benchmarking & Performance Observation (Post-Contract)

Benchmarks are used to **observe system behaviour under load**, not to optimise or redefine semantics.

They must not:

* introduce new runtime primitives,
* change node behaviour,
* or invalidate earlier lessons/topics.

They operate **only on already-verified topologies** (post-Topic 10).

### Status

* [ ] Define benchmark scope:

  * latency
  * throughput
  * executor contention
  * action responsiveness under load
* [ ] Decide benchmark harness location:

  * separate repository **or**
  * isolated workspace subtree
* [ ] Ensure benchmarks:

  * use canonical ROS tooling
  * do not require lesson-local hooks
* [ ] Explicitly document:

  * what benchmarks do *not* claim
  * which results are environment-dependent

Benchmarks are **informational**, not pass/fail gates.

---

## 8. Capstone (Out of Scope for Core Release)

Tracked separately; not required for workspace release.

* [ ] Separate Capstone repository
* [ ] Decide scope and languages
* [ ] Link from Section 10 only

---

## 9. Explicit Non-Goals

The following are intentionally excluded:

* Teaching ROS 1
* Teaching DDS internals
* Teaching Python, C++, or Rust syntax
* Hiding system limits or tooling gaps

---

