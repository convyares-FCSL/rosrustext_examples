# Lesson 05 Theory: Parameters & Central Configuration (Python / rclpy)

## Architectural Intent

Lesson 05 is where configuration stops being “a couple of optional flags” and becomes a **first-class system contract**.

The goal is:

> **Prove that node behaviour can be driven, inspected, and safely updated at runtime using ROS parameters, while keeping a single shared YAML schema across languages.**

This lesson is explicitly ROS-native:
- Configuration is provided via `--params-file` (startup).
- Behaviour changes via `ros2 param set` (runtime).
- Launch-based discovery/wiring is deliberately deferred to Lesson 09.

---

## What’s New Compared to Lessons 00–04

Earlier lessons used parameters in a “nice to have” way (e.g., a publish period with a default).

Lesson 05 makes parameters the primary interface:

- **Startup composition**: multiple `--params-file` inputs combine into one effective parameter set.
- **Validation before application**: invalid values are rejected or clamped before changing runtime behaviour.
- **Hot updates**: parameters are actively handled via callbacks, not polled or ignored.
- **No accidental defaults**: defaults still exist, but are treated as intentional fallbacks (and warnings are only shown when appropriate).

---

## Configuration Model: Three YAML Files, One Schema

Lesson 05 consumes the same YAML schema used by the rest of the workspace:

- `topics_config.yaml` → topic names
- `qos_config.yaml` → QoS profiles and `qos.default_profile`
- `services_config.yaml` → service names

Each language has a thin adapter layer (`utils_py`, `utils_cpp`, `utils_rclrs`, `utils_rcllibrust`) that:
- reads parameters (already loaded by ROS)
- applies defaults when absent
- returns strongly-typed values (or as close as the language allows)

The node does **not** parse YAML directly. ROS does that at startup. The node just reads parameters.

---

## The “Two-Stage Config” Mental Model

Lesson 05 formalizes a pattern you will reuse everywhere:

1) **Startup source of truth**  
   YAML passed in via `--params-file`.

2) **Live source of truth**  
   Runtime parameter values held in the ROS parameter server and mutated via `ros2 param set`.

This split matters because:
- YAML is how you standardize fleets and deployments.
- Runtime parameters are how you debug, tune, and recover without redeploying.

---

## Node Structure: Transport vs Logic

In this lesson both publisher and subscriber follow the same boundary:

### 1) Node (ROS resources, lifecycle, callbacks)
Owns:
- publishers/subscriptions
- timers
- parameter declarations
- parameter callback registration
- safe “rebuild” of resources when needed

### 2) Logic (pure behaviour, testable)
Owns:
- validation rules (e.g., `timer_period_s > 0`)
- stream semantics (ordering, resets, tolerance)

This is why Lesson 05 keeps lesson-scoped logic in the lesson package (not in global `utils_py`):
- `utils_py` remains “workspace configuration adapters”.
- the lesson package remains “behaviour + validation for this lesson”.

That separation is what will map cleanly to Rust later (logic can live in `lib.rs` with tests, while node binaries stay thin).

---

## Shared Config Utilities: Defaults With Correct Warnings

A subtle failure mode in ROS parameter systems is “defaults masking misconfiguration”.

Lesson 05’s `_get_or_declare` helper does two important things:

1) Always declares the parameter (so it appears in `ros2 param list`).
2) Warns only when:
   - the effective value equals the default **and**
   - no override was provided via `--params-file` / `-p`

This prevents false warnings when YAML correctly sets the same value as the default.

Outcome:
- warnings signal a real condition: “you didn’t supply config, we fell back”.

---

## Publisher: Parameter-Driven Timer as a Reconfigurable Resource

The publisher’s runtime behaviour is governed by `timer_period_s`.

Key property:

- Changing `timer_period_s` requires rebuilding a ROS resource (the timer).
- The safe approach is:
  - validate
  - cancel old timer
  - create new timer
  - update internal state consistently

This pattern is the practical “production” version of parameter updates: not just accepting values, but applying them safely to runtime resources.

---

## Subscriber: Stream Semantics as Business Logic

The subscriber is not just “print messages”. It validates transport behaviour as a system property.

The logic layer handles:

- **Late joiners**: first received sample establishes baseline.
- **Publisher restart/reset**: a sudden drop to a low value can trigger resync.
- **Out-of-order/stale samples**: detected and logged without corrupting state.

The subscriber’s runtime parameter (`reset_max_value`) changes logic behaviour **without** needing to rebuild ROS resources (it updates the listener state in place).

This is a useful contrast:

- publisher parameter updates → resource rebuild (timer)
- subscriber parameter updates → logic update (validator state)

---

## Why We Compose YAML via CLI in Lesson 05

Lesson 05 uses explicit `--params-file ...` composition on purpose.

It proves:
- the schema works
- the nodes are correctly parameter-driven
- runtime updates behave safely

It does not solve:
- discovery of installed vs source-tree configs
- launch-driven wiring
- environment-based config selection

Those are Lesson 09 problems. Solving them early would hide the key mechanism Lesson 05 is teaching.

---

## Testing: What’s Actually Being Verified

Lesson 05 tests avoid requiring a ROS graph.

The unit tests should cover:

- `validate_timer_period_s`:
  - accepts float/int numeric inputs
  - rejects non-numeric
  - rejects <= 0
- `validate_reset_max_value`:
  - accepts integers
  - rejects non-integer inputs
  - rejects < 0

This reinforces the rule introduced in Lesson 04:

> test logic in isolation, keep ROS integration thin.

---

## What This Lesson Establishes

Lesson 05 demonstrates that:

1) Configuration can be standardised across languages using a shared YAML schema.
2) Node behaviour is correctly defined by the composition of multiple parameter files at startup.
3) Runtime parameter updates are applied safely and predictably through validation and controlled resource updates.
4) The codebase supports long-term maintainability by separating testable logic from ROS resource ownership.

This lesson marks the transition from illustrative examples to production-oriented ROS 2 systems.

