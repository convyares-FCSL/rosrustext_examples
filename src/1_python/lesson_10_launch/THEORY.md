# THEORY — Lesson 10 (Launch, Topology & Deployment Verification)

## Lesson 10 — Operational Closure

Lessons 06–09 established behaviour under increasing pressure:

* lifecycle correctness
* long-running actions
* scheduling and executor effects
* deployment-induced shared fate

Lesson 10 does not add a new concept.
It closes the system operationally.

---

## The Verification Pyramid

By Lesson 10, the system has passed through three layers:

1. **Unit correctness** — internal logic behaves as expected
2. **Integration correctness** — graph interactions and orchestration work
3. **Deployment correctness** — the system can be brought up and torn down cleanly

Lesson 10 addresses the third layer.

---

## Launch Is Not Logic

Launch files do not change behaviour.
They declare:

* which processes exist
* how configuration is injected
* how lifetimes are managed

Lesson 10 treats launch as a **declarative boundary**, not a programming surface.

---

## Installed vs Source Reality

Systems that only work from a source tree are not deployable systems.

Lesson 10 enforces:

* configuration discovered from installed package shares
* no reliance on `src/...` paths
* no environment-specific hacks

This is a deployment constraint, not a convenience.

---

## Verification as Evidence

The verification script does not test logic.

It asserts observable facts:

* nodes appear
* lifecycle states transition
* actions and telemetry surfaces exist when expected
* those surfaces disappear on shutdown

Nothing is inferred.
Everything is observed.

---

## Failure Is Preserved

Lesson 10 does not repair the failures revealed earlier.

* shared-fate shutdown still interrupts actions
* clients may hang if servers disappear

This is intentional.

Verification proves the system is deployable, not that it is pleasant.

---

## The Final Contract

After Lesson 10, the system satisfies a complete contract:

* behaviour is correct
* orchestration is observable
* deployment is reproducible
* shutdown is hygienic

No new primitives appear after this point.

Everything that follows is application, not instruction.
