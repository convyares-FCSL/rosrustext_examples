## C++ Track — Consolidated Review Ledger (Snapshot)

This ledger covers **C++ lessons 00–10** plus **`utils_cpp`** and the **C++ track README**. It records **facts and closure conditions**, not refactors or redesigns.

### Legend

* ✅ **Answered (Explicit)** – resolved and documented clearly
* 🟡 **Answered (Implicit)** – resolved by later structure/behaviour, not explicitly documented
* ⚠️ **Open** – identified, not yet answered
* 🕓 **Deferred (Intentional)** – acknowledged, consciously postponed

---

## 1. Outstanding Concerns Requiring Attention

> Includes only: ⚠️ Open / 🟡 Answered (Implicit) / 🕓 Deferred (Intentional)
> Excludes ✅ Answered (Explicit)

### 1.1 Governance / Framing / Contract Truth

<details>
<summary><strong>⚠️ CXX-DOC-01 — C++ README: “C++ is correct by definition” conflicts with parity-as-constraint</strong></summary>

**Concern**
The C++ track README states that if behaviour differs between languages, the C++ behaviour is “correct” by definition. This conflicts with the repository’s core contract: **parity is judged by externally observable behaviour**, and deviations must be surfaced as costs/limits, not declared correct by fiat.

**Why it matters now**
This is release-framing. It changes how readers interpret mismatches: *constraint evidence* vs *authority override*.

**Current state**
⚠️ Open

**What explicit closure would look like**
C++ README explicitly aligns to INTENT/PHILOSOPHY language: parity is the contract; differences are surfaced and documented as ecosystem constraints or lane limitations—without declaring one lane “correct by definition”.

</details>

<details>
<summary><strong>⚠️ CXX-DOC-02 — Non-Markdown artefact markers in Lesson 00 docs</strong></summary>

**Concern**
Lesson 00 docs contain tooling-style artefact markers (e.g., citation placeholders) that do not function as intentional repo content.

**Why it matters now**
Docs are authoritative sources of truth. Stray artefacts reduce credibility and confuse what is “deliberate pressure” versus “accidental noise”.

**Current state**
⚠️ Open

**What explicit closure would look like**
Artefact markers removed, or explicitly documented as intentional and explained (format + interpretation).

</details>

<details>
<summary><strong>🟡 CXX-DOC-03 — Lesson 07 intentional degradation is not unmistakably marked as intentional</strong></summary>

**Concern**
Lesson 07’s executor starvation/degraded responsiveness is the point, but the lesson relies on reader inference rather than an explicit “this is intentional and is the acceptance condition” marker.

**Why it matters now**
Intentional failure must be unmistakable at release; otherwise it reads as a defect.

**Current state**
🟡 Answered (Implicit) by Lesson 08 contrast, but not explicitly documented.

**What explicit closure would look like**
Lesson 07 README/THEORY explicitly states the intentional failure mode and what “passing” means for that lesson.

</details>

---

### 1.2 Release Metadata / Provenance

<details>
<summary><strong>⚠️ CXX-REL-01 — Placeholder maintainer metadata in packages</strong></summary>

**Concern**
Multiple packages (including `utils_cpp`) use `Your Name <you@example.com>` in `package.xml`.

**Why it matters now**
This is a release-readiness signal: provenance and ownership are ambiguous at the package contract surface.

**Current state**
⚠️ Open

**What explicit closure would look like**
Replace placeholder maintainer metadata with real ownership, or explicitly declare (in governance docs) that maintainer identity is intentionally generic for this repo snapshot.

</details>

---

### 1.3 Configuration / Contract Surfaces

<details>
<summary><strong>🟡 CXX-CONFIG-01 — “Tolerant config” policy exists in utils_cpp, but is not elevated to track-level contract framing</strong></summary>

**Concern**
`utils_cpp` defines a policy: type mismatches fall back to defaults with warnings (no crash). This is a behavioural policy choice (“tolerant, warn-first”). It is documented in `utils_cpp` but not surfaced as a track-level contract statement.

**Why it matters now**
Config “healing vs rejection” is a philosophical axis in the repo. Leaving it buried in a utils README makes behaviour feel accidental instead of governed.

**Current state**
🟡 Answered (Implicit) by implementation + local README, but not elevated.

**What explicit closure would look like**
C++ track README (or top-level governance) explicitly declares the config policy class (warn/permit vs reject) and points to `utils_cpp` as the contract surface.

</details>

<details>
<summary><strong>🕓 CXX-CONFIG-02 — Startup override detection uses parameter override inspection; long-term stability of that signal is not discussed</strong></summary>

**Concern**
`utils_cpp` detects “startup override” via parameter overrides inspection. This is a subtle correctness signal and may vary with ROS behaviour and how params are supplied.

**Why it matters now**
Not release-blocking, but it’s a hidden dependency if readers treat warnings as authoritative indicators of configuration provenance.

**Current state**
🕓 Deferred (Intentional)

**What explicit closure would look like**
Tracked as a known dependency/assumption (what it detects, what it does not claim).

</details>

---

### 1.4 Deployment / Shutdown Truth

<details>
<summary><strong>🟡 CXX-DEPLOY-01 — Composition shutdown ordering acceptability remains implicit</strong></summary>

**Concern**
Composition (Lesson 09) exposes shutdown-order behaviour differences, but “acceptable ordering” is not stated as an explicit acceptance criterion; it is only enforced implicitly via Lesson 10 hygiene checks.

**Why it matters now**
Shutdown is part of correctness in this repo’s framing. Ambiguity at release creates future re-litigation.

**Current state**
🟡 Answered (Implicit) via Lesson 10 verification emphasis, but not explicitly stated.

**What explicit closure would look like**
Lesson 10 states shutdown acceptance criteria under composition (what must be true, what is allowed to vary).

</details>

<details>
<summary><strong>⚠️ CXX-VERIF-01 — Deployment verification does not explicitly preserve earlier intentional failures as “expected-to-fail” cases</strong></summary>

**Concern**
Lesson 10 verification asserts bring-up and hygiene, but does not explicitly encode which earlier intentional failure scenarios are *not expected to pass* (or are expected to fail under specific conditions).

**Why it matters now**
Without this, verification risks reading as “the whole sequence is now smoothed into a happy path,” which violates the repo’s honesty rule.

**Current state**
⚠️ Open

**What explicit closure would look like**
Verification docs include a short “non-goals / expected failures” section: which scenarios are intentionally broken in earlier lessons and why they are not “fixed” by deployment verification.

</details>

---

## 2. Fix & Verification Plan (Release-Oriented)

Ordered, minimal, TODO-suitable. No speculative refactors.

<details>
<summary><strong>Step 1 — Close CXX-DOC-01 (C++ README parity framing)</strong></summary>

**Close**

* CXX-DOC-01

**Action**
Edit the C++ track README to align with INTENT/PHILOSOPHY parity semantics (differences are surfaced as constraints/costs; no “correct by definition”).

**Verify closure**

* Documentation review: statement is unambiguous and consistent with repo governance
* No code changes required

</details>

<details>
<summary><strong>Step 2 — Close CXX-REL-01 (package.xml provenance)</strong></summary>

**Close**

* CXX-REL-01

**Action**
Replace placeholder maintainer metadata (or explicitly declare “generic maintainer is intentional” in governance docs).

**Verify closure**

* `package.xml` contains non-placeholder maintainer info (or governance doc explicitly declares intentional generic provenance)

</details>

<details>
<summary><strong>Step 3 — Close CXX-DOC-03 (Lesson 07 intentional failure marking)</strong></summary>

**Close**

* CXX-DOC-03

**Action**
Add explicit “intentional failure / acceptance condition” text to Lesson 07 README/THEORY.

**Verify closure**

* Doc contains explicit “this degradation is intentional” + describes what “passing” means for Lesson 07
* Behaviour unchanged

</details>

<details>
<summary><strong>Step 4 — Close CXX-DEPLOY-01 + CXX-VERIF-01 (shutdown + expected failures)</strong></summary>

**Close**

* CXX-DEPLOY-01
* CXX-VERIF-01

**Action**
In Lesson 10 docs (and/or verifier docs), explicitly state:

* shutdown acceptance criteria under composition
* expected failure cases that are intentionally not made “green” by the verifier

**Verify closure**

* Re-run existing Lesson 10 verification harness unchanged
* Confirm docs now describe what the harness does **and does not claim**

</details>

<details>
<summary><strong>Step 5 — Close CXX-DOC-02 (artefact marker cleanup)</strong></summary>

**Close**

* CXX-DOC-02

**Action**
Remove stray tooling artefact markers from Lesson 00 docs (or document them as intentional with interpretation rules).

**Verify closure**

* Markdown renders cleanly without unexplained tokens
* No behaviour changes

</details>

---

## 3. Full Issue Ledger (Discovery Order)
> **Purpose (unchanged)**
> This section exists to:
>
> * prevent circular debate,
> * preserve institutional memory,
> * allow future forensic review (“when did we notice X?”).
>
> **This is not an action list.**
> Duplication is allowed. Resolution is tracked, not erased.

**Total observations recorded: 43**

---

### Lesson 00 — Bootstrap

<details>
<summary><strong>L00-CXX-001 — Lesson 00 cannot demonstrate SIGINT/SIGTERM shutdown</strong></summary>

**Concern**
Node exits immediately; interactive shutdown behaviour cannot be observed.

**Initial assessment**
Acceptable only if explicitly scoped.

**Current state**
⚠️ Open
↳ Canonical: CXX-DOC-01 (framing / acceptance claims)

</details>

<details>
<summary><strong>L00-CXX-002 — Non-Markdown artefact markers in README</strong></summary>

**Concern**
Docs contain tooling-style citation artefacts that are not intentional repo syntax.

**Initial assessment**
Reduces documentation authority.

**Current state**
⚠️ Open
↳ Canonical: CXX-DOC-02

</details>

<details>
<summary><strong>L00-CXX-003 — Workspace naming implies “tutorial” semantics</strong></summary>

**Concern**
Path names and wording suggest tutorial framing, conflicting with systems-reference intent.

**Initial assessment**
Narrative debt, not behavioural risk.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: DOC-05 (terminology debt)

</details>

<details>
<summary><strong>L00-CXX-004 — Placeholder maintainer metadata in package.xml</strong></summary>

**Concern**
`Your Name &lt;you@example.com&gt;` used.

**Initial assessment**
Release provenance ambiguity.

**Current state**
⚠️ Open
↳ Canonical: CXX-REL-01

</details>

---

### Lesson 01 — Event Loop

<details>
<summary><strong>L01-CXX-005 — Shutdown behaviour actually first exercised in Lesson 01</strong></summary>

**Concern**
Clean shutdown claim from Lesson 00 is only observable once node runs continuously.

**Initial assessment**
Needs explicit cross-reference.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DOC-01

</details>

<details>
<summary><strong>L01-CXX-006 — Event-loop discipline relies on narrative, not invariants</strong></summary>

**Concern**
State evolution is explainable via logs, but invariants are unenforced.

**Initial assessment**
Acceptable early; must be exploited or constrained later.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-STATE-01 (narrative-first discipline)

</details>

<details>
<summary><strong>L01-CXX-007 — Logs are sole observability surface</strong></summary>

**Concern**
No graph-visible artefacts change over time yet.

**Initial assessment**
Acceptable until publication exists.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-OBS-01

</details>

---

### Lesson 02 — Publisher

<details>
<summary><strong>L02-CXX-009 — Publication contract asserted before verification</strong></summary>

**Concern**
Shared message + topic are defined, but no consumer yet proves contract.

**Initial assessment**
Acceptable sequencing.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-CONTRACT-01

</details>

<details>
<summary><strong>L02-CXX-010 — QoS assumptions unobservable</strong></summary>

**Concern**
QoS compatibility/mismatch not yet diagnosable.

**Initial assessment**
Intentional blind spot.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-QOS-01

</details>

<details>
<summary><strong>L02-CXX-011 — Logs vs wire truth hierarchy unstated</strong></summary>

**Concern**
Docs do not yet state whether logs or graph-visible data are authoritative.

**Initial assessment**
Must flip later.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-OBS-02

</details>

<details>
<summary><strong>L02-CXX-012 — Interface build friction not bounded</strong></summary>

**Concern**
Unclear what counts as expected ecosystem friction vs repo defect.

**Initial assessment**
Needs later clarification.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-ECOSYS-01

</details>

---

### Lesson 03 — Subscriber

<details>
<summary><strong>L03-CXX-013 — Lesson 02 contract closure not explicitly stated</strong></summary>

**Concern**
Subscriber verifies contract, but docs don’t say “this closes Lesson 02’s claim”.

**Initial assessment**
Traceability gap.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-CONTRACT-01

</details>

<details>
<summary><strong>L03-CXX-014 — Diagnosability depends on log literacy</strong></summary>

**Concern**
Failure signatures require human interpretation.

**Initial assessment**
Scaling limit, not a bug.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-OBS-03

</details>

<details>
<summary><strong>L03-CXX-015 — Subscriber correctness itself unverified</strong></summary>

**Concern**
Subscriber acts as canary, not trust anchor.

**Initial assessment**
Acceptable at this stage.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-VERIF-01

</details>

<details>
<summary><strong>L03-CXX-016 — External observability outranks logs but is unstated</strong></summary>

**Concern**
Graph truth becomes authoritative without explicit declaration.

**Initial assessment**
Needs explicit philosophy statement later.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-OBS-02

</details>

---

### Lesson 04 — Services

<details>
<summary><strong>L04-CXX-017 — Adapter boundary enforced by convention only</strong></summary>

**Concern**
Logic/ROS separation not mechanically enforced.

**Initial assessment**
Acceptable now; risk grows later.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-BOUNDARY-01

</details>

<details>
<summary><strong>L04-CXX-018 — Unit tests validate logic, not orchestration</strong></summary>

**Concern**
Risk of over-interpreting unit evidence.

**Initial assessment**
Needs explicit scope statement.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-VERIF-02

</details>

<details>
<summary><strong>L04-CXX-019 — Service availability under load unexamined</strong></summary>

**Concern**
Blocking/latency not stressed yet.

**Initial assessment**
Staged for actions.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-AVAIL-01

</details>

<details>
<summary><strong>L04-CXX-020 — Test harness scope is local</strong></summary>

**Concern**
No statement tying tests to parity or CI expectations.

**Initial assessment**
Acceptable but must be scoped.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-VERIF-03

</details>

---

### Lesson 05 — Parameters

<details>
<summary><strong>L05-CXX-021 — Parameter authority not exclusive</strong></summary>

**Concern**
Unclear whether parameters are the only supported control surface.

**Initial assessment**
Needs clarification.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-CONFIG-01

</details>

<details>
<summary><strong>L05-CXX-022 — Validation failures are human-facing only</strong></summary>

**Concern**
Invalid updates reported via logs/CLI only.

**Initial assessment**
Scaling limit.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-CONFIG-02

</details>

<details>
<summary><strong>L05-CXX-023 — Runtime mutability vs determinism unstated</strong></summary>

**Concern**
Replay determinism sacrificed unless parameter history captured.

**Initial assessment**
Real trade-off.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-DETERMINISM-01

</details>

<details>
<summary><strong>L05-CXX-024 — Adapter boundary stressed by parameter callbacks</strong></summary>

**Concern**
Logic creeps into ROS-facing layer.

**Initial assessment**
Pressure point to observe.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-BOUNDARY-01

</details>

---

### Lesson 06 — Lifecycle

<details>
<summary><strong>L06-CXX-025 — Lifecycle vs parameter update semantics unclear</strong></summary>

**Concern**
Which states honour parameter updates is not stated.

**Initial assessment**
Needs clarification.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-LIFECYCLE-01

</details>

<details>
<summary><strong>L06-CXX-026 — Secondary side effects outside ACTIVE state</strong></summary>

**Concern**
Logging/counters still evolve when inactive.

**Initial assessment**
Acceptable if intentional.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-LIFECYCLE-02

</details>

<details>
<summary><strong>L06-CXX-027 — Lifecycle correctness not yet automated</strong></summary>

**Concern**
CLI-observable but not encoded as verification.

**Initial assessment**
Deferred to deployment phase.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-VERIF-04

</details>

<details>
<summary><strong>L06-CXX-028 — Adapter boundary still conventional</strong></summary>

**Concern**
Lifecycle centralises state but doesn’t formalise boundary.

**Initial assessment**
Risk acknowledged.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-BOUNDARY-01

</details>

---

### Lesson 07 — Actions

<details>
<summary><strong>L07-CXX-029 — Intentional executor starvation not unmistakably marked</strong></summary>

**Concern**
Operational degradation could be mistaken for a bug.

**Initial assessment**
Must be explicit.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DOC-03

</details>

<details>
<summary><strong>L07-CXX-030 — Cancel latency bounds unstated</strong></summary>

**Concern**
Cancellation works but acceptable latency undefined.

**Initial assessment**
Comparison staged for Lesson 08.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-AVAIL-02

</details>

<details>
<summary><strong>L07-CXX-031 — Lifecycle authority collapses under load</strong></summary>

**Concern**
Lifecycle calls starved by long-running work.

**Initial assessment**
Excellent exposure of system truth.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-AVAIL-01

</details>

<details>
<summary><strong>L07-CXX-032 — Scheduling is invisible failure axis</strong></summary>

**Concern**
Scheduler determines availability but is not yet explicit.

**Initial assessment**
Sets up Lesson 08.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-SCHED-01

</details>

---

### Lesson 08 — Executors

<details>
<summary><strong>L08-CXX-033 — Scheduling becomes correctness-critical but under-framed</strong></summary>

**Concern**
Executor choice affects correctness but not declared as contract surface.

**Initial assessment**
Needs elevation.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-SCHED-01

</details>

<details>
<summary><strong>L08-CXX-034 — “Fix” vs “mask” relies on trust</strong></summary>

**Concern**
No explicit proof that semantics are unchanged.

**Initial assessment**
Must be shown later via deployment pressure.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-DEPLOY-01

</details>

<details>
<summary><strong>L08-CXX-035 — Callback group misassignment fragility</strong></summary>

**Concern**
Silent reintroduction of starvation possible.

**Initial assessment**
Real deployment risk.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-SCHED-02

</details>

---

### Lesson 09 — Composition

<details>
<summary><strong>L09-CXX-036 — Deployment topology becomes correctness-critical</strong></summary>

**Concern**
Topology changes behaviour without code changes.

**Initial assessment**
Must be explicit.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: ARCH-DEPLOY-01

</details>

<details>
<summary><strong>L09-CXX-037 — Executor ownership opaque inside containers</strong></summary>

**Concern**
Node code no longer reveals scheduling truth.

**Initial assessment**
Needs tooling/verification to surface.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-DEPLOY-02

</details>

<details>
<summary><strong>L09-CXX-038 — Shutdown ordering acceptability unclear</strong></summary>

**Concern**
Observed order changes; acceptability not stated.

**Initial assessment**
Shutdown is correctness behaviour.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DEPLOY-01

</details>

<details>
<summary><strong>L09-CXX-039 — Callback-group coupling under composition not audited</strong></summary>

**Concern**
Interference visible but not dissected.

**Initial assessment**
Acceptable pressure.

**Current state**
🕓 Deferred (Intentional)
↳ Canonical: ARCH-SCHED-02

</details>

---

### Lesson 10 — Launch & Verification

<details>
<summary><strong>L10-CXX-040 — Topology declared but not normatively constrained</strong></summary>

**Concern**
Launch files read as examples, not supported topology contract.

**Initial assessment**
Needs explicit scoping.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DEPLOY-01

</details>

<details>
<summary><strong>L10-CXX-041 — Verification does not encode expected failures</strong></summary>

**Concern**
Verifier risks implying all prior lessons are “green”.

**Initial assessment**
Must preserve intentional failure.

**Current state**
⚠️ Open
↳ Canonical: CXX-VERIF-01

</details>

<details>
<summary><strong>L10-CXX-042 — Shutdown acceptance criteria implicit</strong></summary>

**Concern**
Clean shutdown enforced, but criteria not restated.

**Initial assessment**
Traceability gap.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DEPLOY-01

</details>

<details>
<summary><strong>L10-CXX-043 — Launch is an architectural surface, but ownership unstated</strong></summary>

**Concern**
Launch files now encode architectural truth, but authority is not explicitly declared.

**Initial assessment**
Readers may treat launch as convenience.

**Current state**
🟡 Answered (Implicit)
↳ Canonical: CXX-DEPLOY-01 / CXX-VERIF-01

</details>

## 4. Reflections on the Review Process

Yes — a few **non-obvious, end-of-arc insights** only become visible after walking **both** Python and C++ tracks end-to-end. None of these are new issues; they’re **structural observations** that sharpen the project without altering intent.

---

<details><summary><strong>1. C++ and Python aren’t “two implementations” — they’re two different epistemologies</strong></summary>

Across the full arc, Python reads like “behaviour first, tooling truth, accept gaps honestly.”
C++ currently reads closer to “C++ is authoritative; others approximate.”

That difference is not about language capability; it’s about **how truth is declared**.

* Python track tends to treat **parity as the governing constraint**: if parity breaks, that’s data.
* C++ track has at least one framing line that implies **authority override** (“C++ is correct by definition”).

**Insight:**
The project’s rare strength is that it treats *observable behaviour* as truth, not “the reference lane says so.” Keeping the lanes epistemically aligned matters more than any specific implementation detail.

</details>

<details><summary><strong>2. The project quietly builds a “control surface stack” (logs → topics → services → params → lifecycle → scheduling → topology → verification)</strong></summary>

Walking the whole arc makes a layered structure obvious:

1. logs (existence, state trace)
2. topics/subscribers (wire truth, silent failure exposure)
3. services (logic separation; correctness in isolation)
4. params (live control surface)
5. lifecycle (orchestration authority)
6. executors/callback groups (availability authority)
7. composition (failure domain / shared fate)
8. launch + verifier (deployment truth + regression gate)

This reads like a **deliberate accretion of control surfaces**—each added only when the previous surface fails under pressure.

**Insight:**
You don’t need to “teach” this stack, but naming it once (even tersely) would give readers a durable mental model: the repo is about *earning* control surfaces, not learning APIs.

</details>

<details><summary><strong>3. utils_* is the real “spec”, and the lessons are experiments against it</strong></summary>

In both lanes, utils is not convenience. It is:

* naming contract (topics/services/actions)
* QoS profile policy
* config parsing policy (warn/heal vs reject)
* the place drift becomes impossible without breakage

By Lesson 06+, the lessons behave like **test fixtures** for the utils-defined contract rather than standalone examples.

**Insight:**
This is unusually strong: the repo’s “truth” is gradually externalised from “lesson code” into **contract surfaces**. The only missing piece is explicit declaration that “utils + interface config are part of the system contract.”

</details>

<details><summary><strong>4. Topic 07 is the moral center, and it’s fragile</strong></summary>

Only after the full arc do you see that Topic 07 does something most projects avoid:

* it demonstrates that “correct” can still be operationally dead
* it refuses to repair the failure early
* it forces the reader to accept that availability is architectural

That makes Topic 07 the *philosophical proof* of PHILOSOPHY’s claim that “failure is data.”

It’s also fragile because if it isn’t unmistakably labeled “intentional failure,” readers interpret it as a bug and the whole arc loses its spine.

**Insight:**
Topic 07 is not just “Actions.” It’s the inflection where the repo declares what kind of truth it cares about. Protecting that clarity is higher value than any polish elsewhere.

</details>

<details><summary><strong>5. Topic 10 isn’t “launch” — it’s an epistemic compiler</strong></summary>

After walking 00–10, Topic 10 reads like a compiler that takes:

* contracts (interfaces/utils),
* behaviours (lessons),
* topology choices (launch),
* and produces a single thing: **deployable evidence**.

It transforms narrative claims into executable truth (bring-up, verify, shutdown hygiene).

**Insight:**
This makes Topic 10 the real release gate. The repo doesn’t “teach deployment”; it **enforces that deployment truth must be machine-checkable**. That’s the uncommon thing you built.

</details>

<details><summary><strong>6. The biggest long-term risk is “authority drift” masquerading as “reference clarity”</strong></summary>

As the repo matures, there’s a temptation to designate a lane (often C++) as “the reference” and let others chase it. That is how most multi-language repos decay:

* parity becomes aspirational,
* drift gets rationalised,
* the contract stops being a contract.

Your PHILOSOPHY explicitly rejects this: parity is a constraint; gaps are findings.

**Insight:**
The thing to guard is not code similarity. It’s the rule:

> “No lane gets to declare itself correct by definition; correctness is what is observable under standard tooling.”

That’s the invariant that keeps this work honest over years.

</details>


## Final Takeaway

Nothing critical was missed by the linear traversal of the **combined Python + C++ arc**.

What the consolidation surfaced is not a pile of defects, but a sharper understanding of **what this project actually is** when viewed whole:

* not a tutorial,
* not a language showcase,
* not a “reference implementation” in the usual sense,

but a **controlled experiment in how distributed systems expose their truth under operational pressure**.

Across both tracks, the work consistently refuses to substitute authority, convenience, or pedagogy for evidence.
When something breaks, it is not patched to keep the narrative smooth; it is allowed to stand long enough to become explanatory.

The most valuable outcome of this review is that the project’s **real invariant** is now explicit:

> correctness is not declared,
> it is *observed* — and only after pressure is applied.

The next real task is not adding features or polishing language parity.
It is preserving this epistemic discipline as the codebase grows, resisting the temptation to smooth over the very failures that give the project its authority.

That discipline is rare.
And at this point, it is the project’s most important asset.
