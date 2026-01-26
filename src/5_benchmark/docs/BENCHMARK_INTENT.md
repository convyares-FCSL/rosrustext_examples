# INTENT — Benchmarking, Observability & Capability

*(Engineering Study, Downstream of the Topics)*

This section exists to make **language trade-offs in ROS 2 systems legible without prescribing choices**, by treating comparison as **engineering evidence under pressure**, not ideology.

It evaluates each language track across three inseparable axes that together define the **practical behavioural contract** of a system operating under real constraints:

1. **Benchmarking** — what the system *costs* under load (latency, jitter, throughput, CPU)
2. **Observability** — what can be *proven from the outside under pressure*, using ROS tooling and external instrumentation where necessary
3. **Capability** — what deployment primitives and control surfaces actually exist, and where parity ends

These axes are not independent features.
They are three lenses on the same question:

> *What does this system do from the outside, when it is stressed, and what does that imply for the engineer who owns it?*

This section is **not a Topic** in the main sequence.
It is an **engineering study downstream of the Topics**, and depends on them for meaning. It does not introduce new semantics or redefine correctness; it measures the **consequences** of decisions already made.

---

## What This Section Is (and Is Not)

<details open>
<summary><strong>What this section is</strong></summary>

* A controlled way to compare **externally observable system behaviour under pressure** across language tracks
* A set of **repeatable measurement and inspection harnesses**, not a performance article
* A place to document **capabilities, blind spots, and limits** that materially affect deployment
* A way to teach engineers **how to measure and observe**, so they can test their own assumptions
* A home for **explicit risk disclosure**, including ecosystem maturity, tooling gaps, and ownership boundaries

</details>

<details>
<summary><strong>What this section is not</strong></summary>

* Not a language ranking or shoot-out
* Not a recommendation engine
* Not a substitute for the Topics (it assumes their context and discipline)
* Not a claim that measurements are portable across machines, kernels, DDS vendors, or environments
* Not a place to hide missing features or smooth differences with abstraction

</details>

---

## Audience Assumptions

This section is written for **mid-career engineers** making real deployment decisions:

* experienced enough to ship systems,
* confident enough to be exposed to hidden risk,
* responsible for consequences they may not yet have lived through.

It may also be useful to senior engineers as a **reproducible artifact**, but it assumes no onboarding and offers no prescriptions.

---

## Repository Constraints Applied Here

<details>
<summary><strong>Evidence over authority</strong></summary>

Nothing here is “true because the repository says so.”

All claims must be supported by:

* repeatable scenarios,
* captured outputs,
* stable definitions of load and topology,
* and clear explanation of what was measured, what was observed, and what was not.

Authority comes from visibility under constraint, not from assertion.

</details>

<details>
<summary><strong>No parity laundering</strong></summary>

This section does not declare parity where none exists.

If a language track cannot participate in a deployment primitive — lifecycle control, composition, orchestration, shutdown correctness — that limitation is recorded as a **capability boundary**, not hidden behind shims or wrappers for symmetry.

</details>

<details>
<summary><strong>No prescriptions</strong></summary>

This section does not tell engineers what to choose.

It provides:

* measured costs,
* observable behaviours,
* capability limits,
* and explicit risk context

so engineers can decide within their own constraints.

It does **not** define canonical roles for languages.

</details>

<details>
<summary><strong>Risk attribution is required</strong></summary>

Where ecosystem maturity, dependency trust, community maintenance, or ownership boundaries affect adoption risk, this is stated plainly.

This includes:

* reliance on community-maintained packages,
* dependence on out-of-tree or non-canonical infrastructure,
* author-maintained extensions that improve capability while introducing ownership risk,
* behaviour that is stable in practice but not supported by canonical tooling.

Risk is treated as **operational cost**, not judgement.

</details>

---

<details> <summary><strong>Explicit exception: Central Risk & Capability Index</strong></summary>

This section permits a single Central Risk & Capability Index that aggregates findings from the Topics and Benchmarks.

This is a deliberate exception to the repository’s normal refusal to summarise.

The index exists to:

surface concentrations of risk and capability limits,

help time-constrained engineers identify where deeper evidence lives,

and make implicit trade-offs explicit.

The index is explicitly lossy and non-authoritative:

it must never replace raw observations,

it must always link back to concrete evidence,

and it must not be used to prescribe choices.

Aggregation is allowed only under these constraints.
The default rule remains: evidence precedes summary.

</details>

---
## The Three Axes

<details open>
<summary><strong>1) Benchmarking</strong></summary>

Benchmarks exist to expose **cost surfaces under controlled load**:

* latency (min/mean/max)
* jitter
* throughput
* CPU behaviour

Benchmarks are **signals**, not verdicts.
They are environment-sensitive and must be interpreted in context.

</details>

<details>
<summary><strong>2) Observability</strong></summary>

Observability asks:

> *What can be proven from the outside when the system is under pressure?*

This includes:

* what ROS graph and lifecycle tooling can reveal,
* what state and control surfaces are externally visible,
* what can be instrumented repeatably with profilers, tracers, logs, and counters,
* how failures present under load (starvation, hangs, discovery artefacts, shutdown residue).

ROS tooling is a shared lens, not the sole authority.

</details>

<details>
<summary><strong>3) Capability</strong></summary>

Capability asks:

> *What system primitives exist in this track, without pretending?*

This includes:

* lifecycle support and how authoritative it is,
* composition and container behaviour and whether it is tool-faithful,
* launch, topology expression, and deployability,
* orchestration surfaces and shutdown correctness,
* where the language ceases to be a first-class participant in a ROS-native deployment model.

Capability differences are treated as **engineering facts with consequences**.

</details>

---

## Relationship to the Topics

This study depends on the Topics for meaning and authority.

The Topics establish the behavioural contract and expose operational pressure.
This section measures **what those decisions cost** under load and across language tracks, and documents where the “same system” stops being expressible.

It is the only safe place to discuss differences **without turning the repository into a language war**.

---

## Reproducibility and Time

Exact numbers are not expected to reproduce.

Hardware, kernels, DDS vendors, schedulers, and environments vary, and failure to reproduce identical measurements is normal.

What must reproduce are:

* failure shapes,
* visibility gaps,
* control surfaces,
* and the questions the system forces an engineer to ask.

Specific measurements may age.
The pressures and decision shapes are expected to remain valid.

---

## Acceptance

This section is considered valid when:

* scenarios are defined clearly enough to rerun,
* measurement and observation methods are documented honestly,
* capability and observability deltas are stated plainly with evidence,
* and readers are equipped to run their own tests and reach their own judgement.

---

## Closing Statement

This section exists to support **judgement without ideology**.

> **Do not trust claims.
> Run the harness.
> Observe the system under pressure.
> Know the blind spots.**
