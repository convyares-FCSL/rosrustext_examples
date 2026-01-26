# Cross-Track Parity Review Ledger (Python ↔ C++)

This ledger records **parity observations as facts**, not prescriptions.
It exists to prevent re-litigating decisions and to track work that can be done later without blocking current sequencing.

---

## Legend

* ✅ **Answered (Explicit)** – resolved and documented clearly in both tracks
* 🟡 **Answered (Implicit)** – resolved by behaviour or later lessons, but not explicitly documented
* ⚠️ **Open** – identified, not yet answered
* 🕓 **Deferred (Intentional)** – acknowledged and intentionally not fixed now

---

## 0. Scope & Rules

### 0.1 Parity definition (binding)

Parity is judged by what is externally observable through standard ROS tooling:

* `ros2 node`, `ros2 lifecycle`, `ros2 action`, `ros2 topic`
* graph topology visibility
* lifecycle behaviour
* action semantics (goal / feedback / result / cancel)
* observable starvation shapes under pressure

---

### 0.2 What this ledger is not

* Not a refactor plan
* Not a style guide
* Not a “make C++ match Python line-by-line” document

---

### 0.3 Epistemic rule (binding)

No language track is normative by definition.

If two tracks diverge, the source of truth is:

* externally observable behaviour under standard ROS tooling
* documented constraints of the language or ecosystem
* reproducible pressure-induced effects

Any claim of correctness based on **“reference lane” status alone** is invalid.

---

## 1. Release-Blocking Concerns (must not be ambiguous)

> Answers one question only:
> **What cannot be left ambiguous at point of release?**
> Items here need either: a fix, explicit docs, or explicit deferral.

---

### 1.1 Contract surfaces & names

<details>
<summary><strong>✅ PAR-01 — Canonical action name for Fibonacci</strong></summary>

**Observation**
Python uses `topics.fibonacci -> "/tutorial/fibonacci"` by default.
C++ must match; node-name-derived defaults break Topic 08+ reuse.

**Status**
✅ Answered (Explicit)

**Closure evidence**
`ros2 action list` shows `/tutorial/fibonacci` when server is Active.

</details>

<details>
<summary><strong>⚠️ PAR-02 — Topic / QoS configuration precedence is identical</strong></summary>

**Observation**
Python and C++ both support YAML overrides via `--params-file`.
Need to ensure precedence and failure policy are consistent where required.

**Status**
⚠️ Open

**Closure evidence**
Documented precedence in top README (or per-track README) + observed behaviour under override and invalid config.

</details>

---

### 1.2 Intentional starvation & pressure framing

<details>
<summary><strong>🟡 PAR-03 — “Production-grade but insufficient scheduling” framing</strong></summary>

**Observation**
Python Theory frames Lesson 07 as “intentionally not production-correct.”
C++ Theory frames it as “correct code that still degrades operationally.”

These are compatible if interpreted as:

> action & lifecycle usage is correct, availability is not ensured under naïve scheduling.

**Status**
🟡 Answered (Implicit)

**Closure evidence**
Observed starvation and recovery sequence is identical by Lesson 08.

</details>

---

### 1.3 Epistemic alignment across tracks

<details>
<summary><strong>⚠️ EPI-01 — “Reference lane” language implying authority</strong></summary>

**Observation**
At least one C++ document historically implied C++ as the authoritative or defining implementation, with other languages approximating it.

This conflicts with the repository’s governing rule that **observable behaviour defines truth**, not implementation lineage.

**Status**
⚠️ Open (documentation-level only)

**Closure evidence**
Explicit language-neutral statement in PHILOSOPHY or INTENT clarifying that:

> parity is a constraint, and gaps are findings — not deficiencies of “non-reference” lanes.

</details>

---

## 2. Non-Blocking Differences (allowed, but must be recorded)

### 2.1 Structure / file layout

<details>
<summary><strong>🟡 STR-01 — Different file breakdown across languages</strong></summary>

**Observation**
Python splits components across multiple small files.
C++ groups functionality via `.hpp/.cpp` and compile targets.

Difference is acceptable if contract surfaces remain stable and the split doesn’t introduce hidden semantics.

**Status**
🟡 Answered (Implicit)

**Closure evidence**
Behavioural parity holds under Topics 06–10.

</details>

<details>
<summary><strong>🕓 STR-02 — “Lesson” terminology mismatch</strong></summary>

**Observation**
“Lesson” is historical terminology; system behaves like an operability reference.

**Status**
🕓 Deferred (Intentional)

**Closure evidence**
Tracked as structural debt only.

</details>

---

### 2.2 Epistemic posture differences

<details>
<summary><strong>🟡 EPI-02 — Behaviour-first vs implementation-first truth framing</strong></summary>

**Observation**
Python track consistently treats correctness as:

> what standard tooling observes under pressure.

C++ track historically leaned toward:

> what canonical ROS implementations support.

These framings are compatible only when observable behaviour is explicitly the arbiter of truth.

**Status**
🟡 Answered (Implicit)

**Closure evidence**
By Topic 10, both tracks converge on identical behavioural outcomes despite tooling asymmetries being surfaced rather than hidden.

</details>

---

## 3. Per-Lesson Parity Ledger

### 3.1 Lesson 07 — Actions under Lifecycle

<details>
<summary><strong>✅ L07-01 — Lifecycle gating of action acceptance</strong></summary>

Goals rejected when Inactive; accepted when Active.

</details>

<details>
<summary><strong>✅ L07-02 — Feedback shape is stable and observable</strong></summary>

Feedback publishes partial sequences; result matches Fibonacci.

</details>

<details>
<summary><strong>✅ L07-03 — Cancellation is cooperative and delayed under load</strong></summary>

Cancel accepted; final status CANCELED; partial result returned.

</details>

<details>
<summary><strong>✅ L07-04 — Starvation probe is externally visible</strong></summary>

Lifecycle responsiveness degrades during long action under single-thread executor.

</details>

<details>
<summary><strong>⚠️ L07-05 — Business logic boundary consistency</strong></summary>

Ensure action servers remain orchestration layers only.

</details>

<details>
<summary><strong>🟡 L07-06 — Intentional failure is not a bug</strong></summary>

**Observation**
Lesson 07 demonstrates correct API usage that is operationally insufficient under pressure.

The failure is intentional and provides the pressure that justifies Lesson 08.

**Status**
🟡 Answered (Implicit)

</details>

---

### 3.2 Lesson 09 — Composition & Containers

<details>
<summary><strong>✅ L09-01 — Deployment-induced failure is observable</strong></summary>

Co-located nodes interfere without code changes.

</details>

<details>
<summary><strong>🟡 L09-02 — Tooling gaps are surfaced, not patched</strong></summary>

Python lacks canonical component loading; gap documented explicitly.

</details>

---

### 3.3 Lesson 10 — Launch, Topology & Deployment Verification

<details>
<summary><strong>✅ L10-01 — Profile A (shared fate) exists</strong></summary>

Both Python and C++ deploy full composition topology.

</details>

<details>
<summary><strong>✅ L10-02 — Profile B (fault-lined topology) exists</strong></summary>

Selective isolation via process boundaries demonstrated.

</details>

<details>
<summary><strong>✅ L10-03 — Failure containment is proven</strong></summary>

Worker process death does not terminate control plane.

</details>

---

### 3.4 Cross-cutting architectural observations

<details>
<summary><strong>🟡 ARC-01 — Control surfaces accrete under pressure</strong></summary>

logs → topics → services → params → lifecycle → scheduling → topology → verification

</details>

<details>
<summary><strong>🟡 ARC-02 — utils_* is the de facto system contract</strong></summary>

Lessons increasingly behave as experiments against utils-defined contracts.

</details>

<details>
<summary><strong>🟡 ARC-03 — Topic 10 is a release gate, not a feature</strong></summary>

Deployment truth becomes machine-checkable evidence.

</details>

---

## 4. Follow-Up Actions Queue (non-sequencing)

* Clarify config precedence documentation (PAR-02)
* Neutralise any “reference lane” wording (EPI-01)
* Optional: name the control-surface stack once for reader orientation

---

## 5. Notes / Decisions (append-only)

* 2026-01-25 — `/tutorial/fibonacci` confirmed as canonical action name.
* 2026-01-25 — Lesson 07 failure explicitly classified as intentional pressure.
* 2026-01-26 — Topic 10 confirmed as deployment verification gate, not a launch tutorial.

---

### Closing invariant (implicit, now explicit)

> **No lane declares truth.
> Pressure reveals it.
> Tooling observes it.
> Architecture responds to it.**
