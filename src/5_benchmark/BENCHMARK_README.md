# Benchmarking, Observability & Capability

*(Optional Engineering Study — Downstream of the Topics)*

This directory contains an **optional engineering study** examining how ROS 2 systems behave **under pressure** across language tracks.

It is not a lesson.
It is not a recommendation.
It is a decision-support artifact.

Benchmarking here is a **lens**, not the goal.

---

## Purpose

This section exists to make **language trade-offs legible without prescribing choices**, by examining what identical systems **cost, expose, and fail to provide** when stressed.

It evaluates behaviour across three inseparable axes:

* **Benchmarking** — what the system costs under load (latency, jitter, throughput, CPU)
* **Observability** — what can be proven from the outside when pressure is applied
* **Capability** — what control surfaces and deployment primitives actually exist

These are not independent concerns.
They describe one behavioural contract viewed from different angles.

---

## What This Section Is (and Is Not)

### What this section is

* A controlled way to compare **externally observable behaviour under pressure**
* A set of **repeatable harnesses** to expose cost, visibility, and control limits
* A place to document **capability boundaries and blind spots** that affect deployment
* A means to give engineers **tools to test their own assumptions**, not borrow conclusions

### What this section is not

* Not a language ranking or shoot-out
* Not a recommendation engine
* Not a substitute for the Topics
* Not a claim that numbers are portable across environments
* Not a place to hide missing features behind abstraction

---

## Relationship to the Topics

This study is **downstream of the Topics** and depends on them for meaning.

The Topics establish:

* lifecycle authority,
* executor and scheduling semantics,
* composition and shared-fate behaviour,
* shutdown correctness and orchestration reality.

This section does not invent new semantics or diagnose failures back to Topics.
It measures **what those decisions cost** under load and across language tracks.

---

## What Is Measured (and What Is Observed)

### Benchmark Signals

Benchmarks expose cost surfaces such as:

* latency (min / mean / max)
* jitter
* throughput
* CPU behaviour

These numbers are **signals**, not verdicts.

### Observability Under Pressure

Beyond metrics, this section examines:

* what ROS tooling can and cannot reveal under load,
* which control surfaces remain responsive,
* how starvation, contention, and shutdown failure present,
* what remains externally provable when the system is stressed.

### Capability Boundaries

Capability is treated as fact, not aspiration:

* lifecycle support and its authority
* composition and container behaviour
* orchestration and shutdown correctness
* where a language ceases to be a first-class participant

Missing capability is recorded, not compensated for.

---

## Environment Sensitivity

Exact numbers are expected to vary.

Hardware, kernel, DDS vendor, scheduling policy, and background load all matter.
Failure to reproduce identical measurements is normal.

What must reproduce are:

* failure shapes,
* visibility gaps,
* control authority limits,
* and the questions the system forces you to ask.

---

## Central Risk & Capability Index (Explicit Exception)

This section includes a **Central Risk & Capability Index** that aggregates findings across Topics and Benchmarks.

This is a **deliberate exception** to the repository’s usual refusal to summarise.

### Why it exists

Engineers operating under time pressure often need:

* a fast way to identify risk concentration,
* pointers to where deeper evidence lives,
* and visibility into trade-offs they may not have time to rediscover.

### What it is — and is not

* It is **explicitly lossy**
* It is **not authoritative**
* It never replaces raw observations
* It must always link back to concrete evidence

The index exists to answer:

> *“Where should I look more closely before committing?”*

Not:

> *“What should I choose?”*

---

## Risk and Trust Disclosure

Where behaviour depends on:

* community-maintained components,
* out-of-tree or non-canonical infrastructure,
* author-maintained extensions that improve capability,

that dependency is disclosed explicitly.

Improved capability achieved through such means is inseparable from the **ownership, maintenance, and trust risk** it introduces.
Author involvement does not reduce that risk; it makes it attributable.

---

## Structure

```text
5_benchmark/
├─ publisher/
├─ subscriber/
├─ rosbridge_client/
├─ config/
│   ├─ qos_profiles.yaml
│   └─ benchmark_params.yaml
├─ harness/
├─ results/
└─ risk_index/
```

---

## How to Use This Section

Use this material to ask:

* “What does this role cost under load?”
* “What control surfaces disappear when pressure rises?”
* “Where does observability break before correctness does?”
* “What risk am I implicitly accepting by choosing this stack?”

Do not use it to ask:

* “Which language is best?”

---

## Closing

This section exists to support **judgement without ideology**.

It does not promise safety.
It does not promise correctness.
It promises **visibility when it matters**.

---

## Status Check

With this rewrite:

* Benchmarking is no longer the goal, only a lens
* Observability and capability are first-class and explicit
* Prescriptive summaries are removed
* Author-introduced and ecosystem risk are disclosed
* The Central Risk Index is allowed **as an exception, not a precedent**
