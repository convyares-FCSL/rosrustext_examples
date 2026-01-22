# INTENT
This repository exists to define and validate a **language-neutral behavioural contract for ROS 2 systems**, judged by what is **observable under standard ROS tooling**.

It demonstrates how real ROS 2 systems evolve under engineering pressure — from minimal working programs into **operable, deployable architectures** — while maintaining **identical externally visible behaviour across languages**.

It is a **systems engineering reference**: topics are driven by constraints, failures, and scaling pressure that force architectural responses rather than prescribing “best practice” up front.

---

## What This Repository Is (and Is Not)

<details open>
<summary><strong>What this repository is</strong></summary>

* A reference for **professional ROS 2 system construction** under real operational constraints
* A sequence of **engineering questions** whose answers are validated by observable behaviour
* A set of **behavioural parity checks** across languages (not API similarity)
* A record of **pressure → constraint → architectural response**, not feature coverage
* A way to reason about **operability, observability, orchestration, and deployment topology**

</details>

<details>
<summary><strong>What this repository is not</strong></summary>

* Not a ROS 2 beginner/onboarding tutorial
* Not a language advocacy project or a feature showcase
* Not a “best practices” checklist
* Not a claim of complete safety-critical readiness or a perfect architecture

It does not aim to show “the perfect system.”  
It shows how systems become more structured **because they must**.

</details>

---

## Audience Assumptions

This work assumes the reader is:
* an experienced software engineer new to ROS 2, **or**
* an existing ROS 2 user confronting real deployment complexity

It assumes familiarity with ROS fundamentals (nodes, topics, services) and focuses on what those basics don’t teach well:
* system behaviour under load
* orchestration and shutdown correctness
* deployment topology and shared fate
* long-term maintainability through explicit contracts

---

## Repository Constraints

These constraints are the “contract” for contributors as much as readers.

<details>
<summary><strong style="font-size: 1.2em;">Pressure rule</strong></summary>

Nothing appears “because ROS 2 has it”.

Every construct — lifecycle, actions, executors, composition, testing, launch — appears only when the current system can no longer meet its operational goal.

The governing pattern is:

> **Question → Constraint → Pressure → Architectural Response**

Early topics are driven by *existential pressure* (“make it exist, repeatably”).  
Later topics are driven by *operational pressure* (“it exists, but it’s no longer controllable or safe to deploy”).

</details>

<details>
<summary><strong style="font-size: 1.2em;">Behavioural parity rule</strong></summary>

Parity across languages is judged by **observable behaviour**, not internal structure.

Behavioural parity includes:

* stable node, topic, service, and action names
* equivalent lifecycle/orchestration behaviour where applicable
* compatibility with standard CLI observability (`ros2 node`, `ros2 lifecycle`, `ros2 component`)
* equivalent failure modes under equivalent conditions (when failure is the point)

If a language cannot express the same behaviour, the limitation is made visible and documented — not hidden by topic glue.

</details>

<details>
<summary><strong style="font-size: 1.2em;">Adapter boundary rule</strong></summary>

In this repository, domain/business logic is kept:

* ROS-free
* deterministic
* unit-testable

ROS nodes are treated as **adapters** that expose contracts to the graph.  
The boundary is introduced progressively as pressure warrants it (it is not imposed as ceremony from Topic 00).

</details>

<details>
<summary><strong style="font-size: 1.2em;">Proportional rigor rule</strong></summary>

Structure is not treated as virtue.

Comments, abstraction, testing, orchestration, and automation are introduced only when the system becomes unsafe, opaque, or unmanageable without them.

Testing is treated as evidence, not doctrine:

* unit tests appear when logic becomes separable
* integration tests appear when orchestration becomes system-critical
* deployment verification appears when topology and bring-up become dominant risks

</details>

<details>
<summary><strong style="font-size: 1.2em;">Multi-language implementations</strong></summary>

The same topic sequence is implemented in parallel across multiple tracks:

* Python (`rclpy`)
* C++ (`rclcpp`)
* Rust native (`rclrs`)
* Rust bridge (`roslibrust`)

Language choice is treated as an engineering decision tied to system role, not preference.  
Parity enables meaningful comparison later; it is not the end goal.

</details>

---

## The Narrative Arc

<details open>
<summary><strong>Phase 1 — Getting Something Working</strong></summary>

Establish existence and discipline:

* build/run loops you can repeat and trust
* explicit ownership and clean shutdown
* time as an explicit input (event loops)
* first system contracts (interfaces and names)

</details>

<details open>
<summary><strong>Phase 2 — Scaling and Losing Control</strong></summary>

Pressure arrives from growth:

* configuration becomes invisible coupling
* correctness becomes hard to prove without a live graph
* “working” stops meaning “operable”
* responsiveness collapses under realistic work

These are not mistakes; they are capability limits.

</details>

<details open>
<summary><strong>Phase 3 — Deployment Reality</strong></summary>

Topology becomes part of correctness:

* shared fate and interference become visible
* shutdown order and orchestration matter
* composition exposes coupling you didn’t know you had
* deployment requires repeatable bring-up and verification

</details>

---

# Topics
Each topic exists to answer **one systems question**.

Topics do not introduce features for their own sake.  
They validate behaviour, expose constraints, and force architectural responses that remain visible under ROS tooling.

Examples of the kinds of questions this series asks:

* How does a node remain alive without losing control of state?
* When does configuration become a system contract rather than a coding choice?
* How can correct work make a node operationally dead?
* Why does composition reveal coupling rather than fixing it?
* When do you stop trusting terminal rituals and start requiring bring-up verification?

---

<details>
<summary>   
<strong>Topic 00 — Bootstrap</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we get a minimal ROS-capable node running in each language track — with a build/run loop we can repeat and trust?</em>
</summary>

Topic 00 exists because the first barrier most engineers hit is brutally practical: **getting ROS 2 to work at all** in the chosen environment and language. Before topics, messages, services, or architecture, you need a functioning toolchain and a node that can start and stop reliably. This is especially true in non-default tracks (notably Rust), where “it should work” and “it does work” can be separated by a long trench of build, environment, and dependency friction.

The intent here is not to teach patterns. It is to establish the **minimum viable system boundary**:

* the workspace can build in this language
* a node can be launched predictably
* logging is present and readable (so runs are observable)
* shutdown is clean and repeatable (Ctrl-C/SIGTERM)
* the result is *boringly consistent* across reruns

This topic deliberately avoids piling on complexity. There is no custom interface yet, no configuration contract yet, no orchestration. Topic 00 is the moment the system first becomes real: you can run it, stop it, and trust that what you observe is not an artifact of a broken environment.

**Acceptance:** From a cold shell, each language track builds and runs a node that is visible in the graph, logs predictably, and exits cleanly on Ctrl-C/SIGTERM—repeatably, without special rituals.

</details>

---

<details>
<summary>   
<strong>Topic 01 — Event Loop</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>Once ROS 2 is running, how do we keep a node alive and evolving over time without turning state into an untraceable mess?</em>
</summary>

Topic 01 exists because getting a node to start is only the opening move. The moment it runs continuously, a new class of problems appears: **time**. Time creates implicit sequencing, accidental coupling, and “it seemed fine yesterday” behaviour unless you choose a clear rhythm for state change.

Engineers hit this immediately in real work: you bring up a node, then you want it to *keep doing something* — polling, supervising, publishing, reacting. If you don’t establish a disciplined event loop early, the codebase drifts toward scattered callbacks, hidden state transitions, and debugging-by-folklore. That chaos doesn’t always show up in Topic 01, but it becomes explosive later when configuration, services, and orchestration enter the picture.

This topic introduces continuous execution using the simplest honest mechanism: a **timer-driven event loop** with explicitly owned state.

The goal is not speed. The goal is **predictable state evolution**:

* there is a clear “tick” where decisions happen
* state lives in one place and is mutated deliberately
* logs make state evolution reconstructable from the outside
* shutdown discipline from Topic 00 still holds

Topic 01 still refuses premature structure. There is no need yet for unit tests or architectural abstractions; there isn’t enough domain logic to justify them. The pressure here is operational and foundational: **can the component stay alive without becoming opaque?**

**Acceptance:** The node runs indefinitely and evolves state on a clear cadence such that behaviour is reconstructable from logs (without guessing callback interleavings), and shutdown remains clean.

</details>

---

<details>
<summary>   
<strong>Topic 02 — Publisher</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we get real data out of a node — across languages — without turning naming, types, and compatibility into a pile of fragile guesses?</em>
</summary>

Topic 02 exists because once a node runs continuously, the next engineering instinct is obvious: *publish something useful*. And this is where many teams hit their first serious ROS 2 stumbling block: **interfaces and cross-language compatibility**.

Publishing isn’t just “call publish()”. The moment data leaves the process, you’re no longer writing a local program — you’re defining a **system contract**:

* what type is being published
* what the topic is called
* what QoS assumptions are being made
* whether other nodes can actually consume it (today, tomorrow, in another language)

This topic therefore introduces a publisher specifically to force a professional constraint into existence early: **types and names must be shared, not copied**. The pressure is not performance; it’s avoiding the trap where every node hardcodes its own reality and the system “works” only as long as nothing changes.

Topic 02 also deliberately acknowledges a real-world truth: **building custom messages is friction** — especially outside the happy path language/tooling. This topic is where the workspace earns the right to claim “multi-language parity” at all, because it proves that the interface contract can be built, imported, and used consistently across tracks.

The goal is not feature coverage. The goal is **making publication a system-level agreement**:

* a shared message/interface exists and is buildable in each track
* publishing is explicit and observable from ROS tools
* naming is stable and not scattered through code
* the node remains simple and readable — no premature architecture

Topic 02 still refuses to solve problems that haven’t arrived yet. It doesn’t build a configuration framework, it doesn’t introduce lifecycle, and it doesn’t add tests. It establishes the first external data contract that later topics will rely on and verify.

**Acceptance:** Each language track publishes the same shared interface on the same stable topic name, and the published data is visible and consistent via standard ROS graph introspection—without per-language hacks or scattered magic strings.

</details>

---

<details>
<summary>   
<strong>Topic 03 — Subscriber</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we prove the system is actually communicating — across languages, across restarts, and under normal ROS “graph reality” — rather than merely publishing into the void?</em>
</summary>

Topic 03 exists because publishing is not evidence. It’s a claim.

Once you have real data leaving a process, the next engineering instinct is equally obvious: *verify someone can consume it*. And this is where distributed systems begin doing what they do best: **failing silently**.

A subscriber is not introduced here as a symmetrical counterpart to the publisher. It’s introduced as a **verification instrument**. In a professional system, the first consumer is often the thing that tells you whether the system is alive, correctly configured, and behaving sensibly under operational churn.

This topic applies pressure that simple “echo the topic” workflows don’t cover:

* late joiners (subscribers starting after publishers)
* publisher restarts and reconnection behaviour
* QoS mismatches that look fine until you notice missing data
* stale or out-of-order observations that confuse downstream logic
* whether the graph stays intelligible as multiple languages participate

The goal is not to show a callback. The goal is to establish that cross-language publication from Topic 02 is **verifiable and diagnosable** using normal ROS observability, and that the subscriber can act as an early warning system rather than a passive sink.

Topic 03 still refuses premature architecture. It does not introduce services, actions, lifecycle, or orchestration. It also does not pretend verification is “just add tests.” At this stage, verification is about **operational visibility**: can you tell what’s happening from the outside, reliably, when the system behaves like a system?

**Acceptance:** The subscriber reliably receives the shared interface across language tracks and makes common ROS graph failure modes diagnosable (QoS mismatch, late joiners, restarts) through observable behaviour and clear reporting.

</details>

---

<details>
<summary>   
<strong>Topic 04 — Services</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we add request/response behaviour without welding business logic to ROS callbacks — so correctness can be proven without a live ROS graph?</em>
</summary>

Topic 04 exists because once a system can publish and subscribe, the next pressure arrives fast: you need **commands**, **queries**, and **decisions**. Some interactions aren’t streams — they’re questions with answers, or commands with acknowledgements. That’s services.

But services introduce a professional trap: the easiest way to write them is to put all the logic inside the ROS callback. That works until it doesn’t — and when it stops working, the failure isn’t “a bug,” it’s that your toolset can no longer support your operational goals:

* you can’t test logic without spinning a ROS graph
* you can’t reason about correctness without integration runs
* every change becomes expensive, slow, and fragile
* debugging turns into “reproduce it in ROS” instead of “prove it in isolation”

So Topic 04 draws the first hard architectural line in the series:

> **Business logic is not ROS. ROS is the adapter.**

The topic introduces services specifically to force that separation to become real:

* **pure deterministic logic** exists as a ROS-free unit
* the service server is a thin translation layer (ROS types ↔ domain types)
* the service client is equally explicit about what it requests and why
* correctness can be demonstrated without the network, without executors, without “ros2 run”

This is also the first point where testing appears — not as doctrine, but as consequence. Once logic exists independently, a unit test becomes the cheapest form of evidence. Before this, testing would have been theatre. Here, it becomes leverage.

Topic 04 still refuses premature complexity. It doesn’t introduce lifecycle, orchestration, or configuration frameworks beyond what already exists. It doesn’t create generic abstractions for services. It simply proves the system can handle request/response interactions while preserving the boundary between domain logic and middleware.

**Acceptance:** The service round-trip works on the ROS graph, and the core request/response logic is provably correct in isolation (unit-tested) with ROS confined to a thin adapter layer.

</details>

---

<details>
<summary>   
<strong>Topic 05 — Parameters</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we make configuration explicit, shared, and safely changeable at runtime — without turning the system into a bag of hardcoded strings and fragile assumptions?</em>
</summary>

Topic 05 exists because Topics 00–04 can still “work” while hiding a dangerous truth: the system is full of **implicit configuration**.

At small scale, it’s normal to hardcode a topic name, a rate, a QoS profile, or a threshold. At system scale, those become invisible coupling. The system remains “correct” right up until:

* two nodes disagree on a name or QoS and silently stop communicating
* an engineer needs to tune behaviour without rebuilding and redeploying
* the only way to understand what the system *thinks it is doing* is to read source code

That’s not a bug. It’s a toolset failure: the system is no longer **operable**.

So Topic 05 introduces parameters as an operational primitive: a live, inspectable configuration surface. But it does so with a hard constraint that preserves professional discipline:

> **Parameters are not a dumping ground. They are part of the behavioural contract.**

This topic makes configuration a system-level agreement:

* names (topics/services/actions) are not scattered through code
* configuration is discoverable from the outside (introspection)
* parameter updates are validated, not blindly accepted
* nodes respond deterministically to configuration changes
* the same configuration contract exists across language tracks

This is also where the series begins to resemble real deployments: configuration exists outside the binary, and the system can adapt without recompilation. It’s not about “ROS features.” It’s about making the system controllable by operators rather than only by developers.

Topic 05 still refuses premature machinery. It doesn’t introduce lifecycle yet, because the pressure it addresses is not startup orchestration — it’s *configuration drift and invisibility*. It doesn’t introduce heavy abstraction layers; it keeps configuration explicit and typed via utilities rather than magical frameworks.

**Acceptance:** Configuration is inspectable and safely changeable at runtime (validated updates, deterministic effects), and the shared configuration contract remains consistent across language tracks.

</details>

---

<details>
<summary>   
<strong>Topic 06 — Lifecycle</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we stop nodes behaving like noisy scripts — and instead make them controllable components that wait for orchestration before doing anything that matters?</em>
</summary>

Topic 06 exists because Topics 00–05 can still produce a system that “works” while being operationally unsafe: nodes start immediately, publish immediately, request services immediately, and the system’s behaviour depends on startup timing, terminal order, and luck.

At this point the pressure is no longer “can we do the thing?” It is:

* can we **bring the system up deterministically**
* can we prevent side effects before the system is ready
* can we prove readiness and state from the outside
* can we shut down without half the system still running “because it started first”

If the answer is “just start them in the right order,” the system has already failed its operational goal. That isn’t a code bug — it’s a missing control surface.

Topic 06 introduces lifecycle management because it is the smallest honest mechanism ROS provides for this class of problem: **external state control**.

The intent is to turn a node into something that behaves like an engineered component:

* it starts silent (no side effects)
* it allocates resources on configure (predictably)
* it produces externally visible effects only when active
* it can be queried and driven via canonical tooling (`ros2 lifecycle get/set`)
* it can be reasoned about as part of a larger orchestration plan

This topic is where ROS stops being a library and starts being a system discipline. It forces you to acknowledge that startup/shutdown is not incidental — it is part of correctness.

Topic 06 still refuses invented semantics. It uses canonical ROS 2 lifecycle interfaces and standard observability. If parity gaps exist in a language track, that is surfaced as an ecosystem limitation — not patched with topic-specific hacks.

**Acceptance:** Node side effects are gated by lifecycle state and are externally controllable/inspectable via canonical lifecycle tooling, with behaviour remaining predictable across tracks (and gaps surfaced rather than patched).

</details>

---

<details>
<summary>   
<strong>Topic 07 — Actions</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we model long-running, cancellable work — and what breaks when we do it “correctly” but naïvely?</em>
</summary>

Topic 07 exists because once a system has configuration and controllable state (Topics 05–06), the next pressure is unavoidable: real systems do **work that takes time**.

Some work is not a stream (topics) and not a quick question (services). It is a task with a lifecycle of its own:

* start it
* observe progress
* cancel it if conditions change
* receive a final result

That’s what actions are for.

But actions introduce a professional hazard: they make it *easy* to write something that is functionally correct and operationally disastrous.

A node can:

* accept a goal
* compute the right answer
* publish feedback
* return a valid result
* obey lifecycle state rules

…and still become **operationally dead** under load because the work monopolises the executor.

This topic is intentionally designed to surface that failure mode. The long-running work is not an accident; it is the pressure. The point is to show that “correct behaviour” is not the same as **available behaviour**.

Topic 07 therefore introduces:

* an **Action Server** hosted inside a lifecycle-managed node
* a **standalone Action Client** that drives goals and observes reality
* a long-running goal execution path that makes starvation visible

The action is powerful because it models real tasks. It is dangerous because it tempts engineers into blocking execution paths that starve everything else:

* telemetry stops
* lifecycle services time out
* cancellation becomes delayed or ineffective
* the system looks alive but cannot be controlled

That is not “you wrote bad Fibonacci.” It is the system telling you your current execution model can’t support your operational goal.

**Boundary condition:** Topic 07 is allowed to end with degraded responsiveness. It does not “fix” the failure with async tricks or rewritten logic, because that would hide the lesson. The fix is a new axis of control introduced next.

**Acceptance:** Action semantics are correct and the intentional failure mode is reproducible and explainable: long-running goal execution can be “correct” while degrading operability (telemetry/lifecycle responsiveness/cancel latency).

</details>

---

<details>
<summary>   
<strong>Topic 08 — Executors & Callback Groups</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we restore responsiveness under long-running work without changing what the node does — only how it schedules callbacks?</em>
</summary>

Topic 08 exists because Topic 07 proved a brutal truth: a node can be functionally correct and still operationally dead. The system didn’t fail because the action was wrong. It failed because the execution model couldn’t support the operational goal of staying alive while working.

The pressure here is clear:

* long-running work must be allowed
* telemetry must keep flowing
* lifecycle services must remain responsive
* cancellation must be serviced promptly
* the node must remain controllable from the outside

If the solution is “rewrite the work” or “make everything async,” the system has learned the wrong lesson. Real systems will always have long tasks. The question is whether the architecture can survive them.

Topic 08 introduces executors and callback groups as **operational tools**, not performance tricks.

The core idea is simple:

* an executor decides *who gets to run*
* a callback group decides *who can block whom*

By moving from a single-threaded execution model to one that can schedule concurrent callbacks, and by explicitly isolating categories of work, the node becomes survivable without changing its semantics.

This topic enforces a strict boundary:

* the action behaviour remains the same
* the lifecycle behaviour remains the same
* the business logic remains unchanged
* only scheduling strategy changes

This is where the system earns a professional property: **availability under load**.

Topic 08 also sets up the next pressure: once a node is internally survivable, deployment can still break it when multiple nodes share executors and process fate. That is not a concurrency mistake — it is a topology reality, and it comes next.

**Acceptance:** Under long-running action execution, telemetry remains continuous, lifecycle calls remain responsive, and cancellation is timely—without changing the work semantics, only the scheduling strategy.

</details>

---

<details>
<summary>
<strong>Topic 09 — Composition & Containers</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>What changes when correct nodes are deployed together in one process — and how do we observe and reason about those changes using standard ROS tooling?</em>
</summary>

Topic 09 exists because the system has already learned how to survive *internal* pressure. By Topic 08, a lifecycle-managed node can host long-running actions without starving itself. That solves a real problem — and it also removes a convenient excuse.

Now the system is ready for the pressure that breaks real deployments:

> **Correct nodes fail together when co-located.**

Not because the nodes are wrong.  
Because **deployment changes the failure domain**.

Topic 09 makes that change explicit. It does not change node logic. It changes only how nodes are hosted:

* previously-built nodes are loaded at runtime into a **component container**
* the container owns executor ownership, node lifetime, and shutdown boundaries
* nodes now share scheduling and process fate

This topic treats composition as a deployment primitive, not a convenience feature. The point is not fewer processes or better performance. The point is that composition forces hidden assumptions to become visible:

* callback isolation that was “good enough” in a single node becomes insufficient when scheduling is shared
* shutdown order becomes observable behaviour, not an implementation detail
* lifecycle transitions now occur inside a shared process boundary, exposing timing and ordering assumptions
* a system that looked healthy in multiple terminals reveals coupling when co-resident

Tooling parity is part of the contract, not an optional extra. The composed system must be controllable and inspectable through canonical mechanisms:

* `ros2 component load/list/unload` drives topology changes
* lifecycle tools remain valid against composed lifecycle nodes
* the ROS graph remains intelligible and externally verifiable
* no special-case commands, proxies, or topic glue are introduced to “make it pass”

Topic 09 is complete when composition stops being an abstract ROS feature and becomes an operational lens: the student can point to an observed degradation and state, plainly and correctly:

> “Nothing in the nodes changed. This behaviour changed because deployment changed.”

**Acceptance:** The composed system is controllable and inspectable through canonical component and lifecycle tooling, and co-location produces at least one observable, explainable deployment-induced change (interference/ordering/responsiveness) without modifying node logic.

</details>

---

<details>
<summary>
<strong>Topic 10 — Launch, Topology & Deployment Verification</strong>
<br><br>
<strong>This topic asks one question:</strong><br>
<em>How do we bring the system up, prove it is ready, and shut it down cleanly — every time — with topology and verification treated as first-class engineering artifacts?</em>
</summary>

Topic 10 exists because Topic 09 makes deployment truth unavoidable: once nodes share fate, “run these commands in three terminals” stops being a workable operational model. The system may be correct, but it is not yet **deployable** in the professional sense: repeatable bring-up, explicit topology, and clean shutdown are not optional.

Topic 10 turns the system from “something you can run” into “something you can deploy” by making three things explicit:

1. **Topology is intentional**  
   Where nodes live (process boundaries, containers, shared executors) is not an implementation detail. It is how you choose failure domains and operational coupling. Topic 10 makes that choice declarative and reproducible.

2. **Bring-up is deterministic**  
   Startup order, readiness, and orchestration cannot rely on human timing. The system must reach a known operational state via a repeatable procedure.

3. **Verification is deployment-grade evidence**  
   At this stage, correctness is no longer “the node printed the right log line.” Verification must exercise the system the way deployment will: start it, drive lifecycle transitions, confirm graph surfaces, detect failure, and enforce clean shutdown.

Topic 10 introduces launch and deployment verification not as ROS “features,” but as the natural architectural response to pressure:

* once topology matters, it must be declared, not remembered
* once orchestration matters, it must be automated, not ritual
* once failures are subtle, verification must be executable, not narrative

Critically, Topic 10 does not introduce new node semantics. Nodes are not modified to satisfy the harness. Any missing parity or tooling gaps are treated as ecosystem or infrastructure constraints, not solved with topic glue inside components.

Topic 10 is where the verification pyramid becomes complete:

* unit-level proof exists where logic is separable
* integration-level proof exists where orchestration exists
* deployment-level proof exists where topology and bring-up become the dominant risk

**Acceptance:** A fresh environment can launch the full system, drive it into the intended operational state, verify expected behaviour through standard tooling, and shut down without graph residue—making topology and verification repeatable, not ritual.

</details>

---

## Benchmark

Benchmarks exist to make *role-based* trade-offs legible once behavioural parity is established.

They are not a language popularity contest and not a feature comparison. They answer questions that only become meaningful after the contract is proven:

* what does this role cost in latency/jitter under this topology?
* what does the same system look like when the transport is DDS vs a bridge?
* where do different languages fit best when the operational goal is availability, not ideology?

Benchmarks are evidence. They help explain why “right language for the right job” is an engineering decision, not a preference.

## Capstone

The capstone exists to demonstrate mastery *without inventing new semantics*.

It applies the established contract to a larger, more realistic system shape:

* multiple components with explicit orchestration
* configuration-driven behaviour
* controlled failure domains and clean shutdown
* verification that exercises the deployment, not just the code

The capstone should feel like “Topic 00–10, but in one coherent system” — not “Topic 11”.

---

## Tone and longevity

This repository is authoritative without advocacy.

It does not argue that:

* one language is superior
* one ROS pattern is universally correct
* more abstraction is inherently better

It documents what becomes necessary as systems evolve, and it is intended to remain valid as ROS 2 and the repository grow.

---

## Closing statement

> **Correctness is easy.**
> **Operability is earned.**
> **Architecture emerges under pressure.**
```
