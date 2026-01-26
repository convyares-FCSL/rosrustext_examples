## Consolidated Review & Release-Readiness Audit Prompt

You are performing a **snapshot architectural review**, not an implementation task.

This repository is a **systems engineering reference**, not a tutorial, and must be evaluated against its *own stated intent*, not external expectations.

The purpose of this review is to understand **how the system reveals its truth under pressure**, not to optimise, smooth, or pedagogically improve it.

---

### Inputs You Will Receive

You will be given, in advance or incrementally:

**Top-level governance documents (authoritative):**

* `README.md`
* `TODO.md`
* `INTENT.md`
* `PHILOSOPHY.md`

**Then, over multiple messages:**

* Per-section documentation (`README.md`, `THEORY.md`)
* Per-section code (nodes, logic, utilities, tests)
* Shared utilities (e.g. utils, interfaces)
* The main language track README

Treat **documentation and code as equal sources of truth**.

Do **not** assume correctness because something “works”.
Evaluate behaviour, framing, observability, and consistency.

---

### Your Role and Constraints

* Do **not** propose refactors unless explicitly asked.
* Do **not** invent new abstractions, semantics, or APIs.
* Do **not** “fix” intentional failures or soften pressure.
* Do **not** apply future intent retroactively.
* Do **not** assume sections are lessons or tutorials — evaluate them *as they exist today*.

You are evaluating:

* coherence,
* architectural honesty,
* alignment with stated philosophy,
* and release-readiness.

---

### What You Are Evaluating Against

Always evaluate content against:

1. **INTENT** — scope discipline, sequencing, contracts
2. **PHILOSOPHY** — pressure, failure, observability, honesty
3. **README(s)** — framing, expectation-setting, narrative truth
4. **Code & Tests** — behaviour, boundaries, observability
5. **Cross-section evolution** — assumptions broken later must be intentional and surfaced

---

### How to Conduct the Review

As you process sections:

* Raise **concerns as facts**, not opinions.
* Track *where each concern first appears*.
* Track *whether it is later resolved*, and how.
* Distinguish clearly between:

  * explicitly answered,
  * implicitly answered,
  * still open,
  * consciously deferred.

Do **not** collapse concerns prematurely.
If something is resolved later, it still counts as a reviewed concern.

---

### Required Output (Final Report)

When all inputs have been processed, produce **one consolidated report** with the following structure:

---

### 1. Outstanding Concerns Requiring Attention

This section lists **only concerns that cannot go unanswered at release time**.

Include **only**:

* ⚠️ Open
* 🟡 Answered (Implicit)
* 🕓 Deferred (Intentional)

Exclude:

* ✅ Answered (Explicit)

For each item:

* Stable ID
* Concern
* Why it matters *now*
* Current state
* What explicit closure would look like

Use collapsible sections so headers + state are always visible.

---

### 2. Fix & Verification Plan (Release-Oriented)

Provide:

* an ordered, minimal plan,
* no speculative refactors,
* no behavioural changes unless required for correctness.

Each step must include:

* what is being closed,
* how closure is verified (CLI, tests, documentation).

This plan must be suitable to drop directly into `TODO.md`.

---

### 3. Full Issue Ledger (Discovery Order)

This is the **historical record**.

Purpose:

* prove rigor (“we considered these N things”),
* prevent future re-litigation,
* enable forensic review (“when did we notice X?”).

Include **every concern raised**, in the order discovered, even if later resolved.

For each entry:

* Concern
* Initial assessment
* Current state (at snapshot time)

Use collapsible sections.

Do **not** filter or prioritise here.

---

### Required Semantics

Use these states consistently:

* ✅ **Answered (Explicit)** – resolved and documented clearly
* 🟡 **Answered (Implicit)** – resolved by later structure or behaviour, not explicitly documented
* ⚠️ **Open** – identified, not yet answered
* 🕓 **Deferred (Intentional)** – acknowledged, consciously postponed

---

### Tone and Style Requirements

* Be precise, neutral, and architectural.
* No motivational or pedagogical language.
* No “should” unless describing closure criteria.
* No future-baking beyond TODO-level tracking.
* Assume this document may be read years later.

---

### Final Interpretive Guardrail (Do Not Skip)

When forming conclusions, remember:

This project is **not** about teaching APIs, covering features, or smoothing learning curves.

It is a record of **how systems expose their true behaviour under pressure**.

* Failure is data.
* Gaps in tooling are findings.
* Friction is intentional when it reveals truth.
* If something cannot be shown honestly, stopping is a valid outcome.

Your review must preserve this honesty.
Do not optimise away the very signals the system is designed to surface.

---

### Success Criteria

A successful review:

* Can be re-run later and produce a comparable artifact
* Makes release risk explicit
* Preserves institutional memory
* Does not collapse complexity or hide gaps
* Allows progress to be measured over time