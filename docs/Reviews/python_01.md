# Python Track — Consolidated Review Ledger (Snapshot)

This section records **review findings as facts**, not prescriptions.

## Legend

* ✅ **Answered (Explicit)** – resolved and documented clearly
* 🟡 **Answered (Implicit)** – resolved by later lessons, but not explicitly documented
* ⚠️ **Open** – identified, not yet answered
* 🕓 **Deferred (Intentional)** – acknowledged, not to be fixed now

---

## 1. Outstanding Concerns Requiring Attention

> This section answers one question only:
> **“What cannot be left ambiguous at the point of release?”**
>
> Items listed here either:
>
> * still require action,
> * require explicit documentation to close,
> * or require an explicit decision to defer.
>
> Fully closed concerns (✅ Answered Explicit) are intentionally **excluded** and live only in the Full Issue Ledger.

### 1.1 Documentation & Framing

<details>
<summary><strong>⚠️ DOC-01 — README tone vs PHILOSOPHY (pedagogy vs pressure)</strong></summary>

**Concern**
Top-level Python README implies a smooth learning progression, while PHILOSOPHY and later sections intentionally surface friction, failure, and limits.

**Why it matters now**
This is a *first-contact* document. Mismatch here causes reader confusion before intent is established.

**Current state**
Open.

**What closure looks like**
README explicitly sets expectations about:

* investigation over instruction,
* intentional failure,
* ecosystem limits.

</details>

<details>
<summary><strong>⚠️ DOC-02 — “One systems question per section” rule not surfaced</strong></summary>

**Concern**
INTENT enforces strict scoping, but README does not make this rule visible.

**Why it matters now**
Without this rule, readers may misinterpret scope creep as inconsistency.

**Current state**
Open.

**What closure looks like**
README states that each section exists to answer exactly one systems question and references INTENT.

</details>

<details>
<summary><strong>⚠️ DOC-03 — Intentional failure and limits not previewed</strong></summary>

**Concern**
Users encounter intentional breakage or ecosystem limits without advance framing.

**Why it matters now**
Surprise failure erodes trust if not pre-signalled.

**Current state**
Open.

**What closure looks like**
README includes a clear statement that some sections are intentionally broken or incomplete by design.

</details>

<details>
<summary><strong>⚠️ DOC-04 — Cross-section assumption breaking not explicit</strong></summary>

**Concern**
Earlier assumptions (lifecycle, executors) are deliberately invalidated later, but this arc is not documented up front.

**Why it matters now**
Readers may interpret later behaviour as regression instead of investigation.

**Current state**
Open (implicitly addressed inside sections).

**What closure looks like**
README notes that earlier guarantees will be revisited and broken as pressure increases.

</details>

<details>
<summary><strong>🕓 DOC-05 — “Lesson” terminology mismatch</strong></summary>

**Concern**
“Lesson” no longer accurately describes the function of these sections.

**Why it matters now**
It does *not* block correctness or release, but it is acknowledged structural debt.

**Current state**
Deferred (Intentional).

**What closure looks like**
Tracked in TODO only; no renaming in this snapshot.

</details>

---

### 1.2 Python-Specific Technical Concerns

<details>
<summary><strong>⚠️ PY-01 — Lifecycle TransitionEvent.timestamp typing</strong></summary>

**Concern**
Timestamp assigned as `int`, while message expects `builtin_interfaces/Time`.

**Why it matters now**
Affects canonical lifecycle observability via CLI.

**Current state**
Open.

**What closure looks like**
Type verified and corrected if needed; validated via `ros2 lifecycle` tooling.

</details>

<details>
<summary><strong>⚠️ PY-03 — QoS invalid-value coercion policy undocumented</strong></summary>

**Concern**
Invalid QoS strings silently collapse to defaults.

**Why it matters now**
Silent healing is a behavioural policy choice and must be explicit.

**Current state**
Open.

**What closure looks like**
Policy chosen (warn vs silent) and documented.

</details>

---

### 1.3 Architectural / Cross-Cutting

<details>
<summary><strong>🟡 ARCH-01 — utils_py is a contract surface, not a helper</strong></summary>

**Concern**
utils_py is functionally mandatory but framed as optional.

**Why it matters now**
Hidden contracts undermine architectural clarity.

**Current state**
Answered implicitly by usage.

**What closure looks like**
Top-level documentation explicitly describes utils_py as part of the system contract.

</details>

<details>
<summary><strong>🟡 ARCH-02 — Config healing → rejection shift undocumented</strong></summary>

**Concern**
Early sections heal invalid config; later sections reject it, without explicit narrative.

**Why it matters now**
This is a deliberate philosophical shift that should be surfaced.

**Current state**
Answered implicitly by later enforcement.

**What closure looks like**
Documentation explains the shift and why it exists.

</details>

---

## 2. Fix & Verification Plan (Release-Oriented)

> This section exists to answer:
> **“What do we need to do, in what order, and how do we prove it’s done?”**

<details>
<summary><strong>2.1 Phase 1 — Python Correctness</strong></summary>

* Close **PY-01**: verify and correct lifecycle timestamp typing
  *Verification*: `ros2 lifecycle` CLI output

* Close **PY-03**: decide and document QoS invalid-value policy
  *Verification*: documentation + observed warning or behaviour

</details>

---

<details>
<summary><strong>2.2 Phase 2 — Documentation Alignment (No Behaviour Changes)</strong></summary>

* Update Python README to close:

  * DOC-01
  * DOC-02
  * DOC-03
  * DOC-04

* Add explicit notes covering:

  * ARCH-01 (utils_py as contract)
  * ARCH-02 (config philosophy shift)

</details>

---

<details>
<summary><strong>2.3 Phase 3 — Structural Debt Tracking</strong></summary>

* Add TODO entries for:

  * terminology refactor (“lesson”)
  * long-term documentation consolidation

</details>

---

<details>
<summary><strong>2.4 Phase 4 — Verification</strong></summary>

* Re-run:

  * lifecycle CLI checks
  * existing tests (Lessons 06–08)
  * Lesson 10 verifier

* Confirm **no behaviour changes** occurred as a result of doc-only updates.

</details>

---

## 3. Full Issue Ledger (Discovery Order)

> **Purpose of this section**
>
> * prove rigor (“we considered these 35 things”)
> * prevent future re-litigation
> * enable forensic review later (“when did we notice X?”)
>
> This is a **historical record**, not an action list.

**Total issues recorded: 35**

<details open>
<summary><strong>Full Issue Ledger — 35 items</strong></summary>

### Early Lessons / Foundations

<details><summary><strong>1. Hard-coded topic names in early nodes ✅</strong></summary>

* **Concern:** Topics embedded in node code undermine parity and refactorability.
* **Initial assessment:** Acceptable for Lesson 00–01, dangerous if it persists.
* **Current state:** **Answered (Explicit)** — resolved by `utils_py` centralisation.

</details>

<details><summary><strong>2. Business logic coupled to rclpy constructs ✅</strong></summary>

* **Concern:** Logic embedded in nodes violates adapter-boundary philosophy.
* **Initial assessment:** Must be separated before lifecycle/actions.
* **Current state:** **Answered (Explicit)** — logic modules introduced and tested.

</details>

<details><summary><strong>3. Unit tests exercising ROS, not logic ✅</strong></summary>

* **Concern:** Tests depending on ROS runtime reduce determinism.
* **Initial assessment:** Unit tests should target logic only.
* **Current state:** **Answered (Explicit)** — logic tests added.

</details>

<details><summary><strong>4. Parameter defaults silently masking config errors 🟡</strong></summary>

* **Concern:** Silent fallback hides misconfiguration.
* **Initial assessment:** Acceptable early, must change later.
* **Current state:** **Answered (Implicit)** — later lessons reject/guard more strictly.

</details>

<details><summary><strong>5. Configuration source of truth unclear ✅</strong></summary>

* **Concern:** YAML, code defaults, and CLI flags unclear in precedence.
* **Initial assessment:** Needs one authoritative source.
* **Current state:** **Answered (Explicit)** — `lesson_interfaces/config` + CLI injection.

</details>

### utils_py Emergence

<details><summary><strong>6. “utils” package acting as more than helpers 🟡</strong></summary>

* **Concern:** utils_py becoming architecturally significant.
* **Initial assessment:** Risk of hidden contract.
* **Current state:** **Answered (Implicit)** — accepted as contract surface.

</details>

<details><summary><strong>7. Topic/QoS/service naming fragmentation risk ✅</strong></summary>

* **Concern:** Multiple lessons defining overlapping names.
* **Initial assessment:** Centralise or drift will occur.
* **Current state:** **Answered (Explicit)** — centralised in utils_py.

</details>

<details><summary><strong>8. Lifecycle shim legitimacy ✅</strong></summary>

* **Concern:** Custom lifecycle wrapper might invent semantics.
* **Initial assessment:** Must be canonical-surface only.
* **Current state:** **Answered (Explicit)** — uses lifecycle_msgs only.

</details>

<details><summary><strong>9. Lifecycle state visibility via CLI ✅</strong></summary>

* **Concern:** Lifecycle must be externally observable.
* **Initial assessment:** Non-negotiable.
* **Current state:** **Answered (Explicit)** — verified via ros2 lifecycle tooling.

</details>

<details><summary><strong>10. Parameter override detection relying on rclpy internals ✅</strong></summary>

* **Concern:** `_parameter_overrides` is semi-private.
* **Initial assessment:** Risky if correctness depends on it.
* **Current state:** **Answered (Explicit)** — warning-only usage, documented.

</details>

### Services / Actions Transition

<details><summary><strong>11. Service semantics leaking into node lifecycle ✅</strong></summary>

* **Concern:** Services starting before lifecycle activation.
* **Initial assessment:** Violates lifecycle authority.
* **Current state:** **Answered (Explicit)** — gated correctly.

</details>

<details><summary><strong>12. Action server responsiveness under load ✅</strong></summary>

* **Concern:** Long-running callbacks block executor.
* **Initial assessment:** Must be demonstrated, not fixed yet.
* **Current state:** **Answered (Explicit)** — Lesson 07 intentional failure.

</details>

<details><summary><strong>13. Failure mode being “too easy to fix” ✅</strong></summary>

* **Concern:** Temptation to patch starvation early.
* **Initial assessment:** Must remain broken in isolation.
* **Current state:** **Answered (Explicit)** — separation enforced.

</details>

<details><summary><strong>14. Action cancellation semantics unclear 🟡</strong></summary>

* **Concern:** Cancel behaviour under load ambiguous.
* **Initial assessment:** Needs observable behaviour.
* **Current state:** **Answered (Implicit)** — visible via CLI and tests.

</details>

### Executors & Scheduling

<details><summary><strong>15. Executor choice implicit, not explicit ✅</strong></summary>

* **Concern:** SingleThreadedExecutor assumed by default.
* **Initial assessment:** Needs surfacing.
* **Current state:** **Answered (Explicit)** — Lesson 08.

</details>

<details><summary><strong>16. Callback group semantics underexplained ✅</strong></summary>

* **Concern:** Callback groups affect behaviour dramatically.
* **Initial assessment:** Must be observable.
* **Current state:** **Answered (Explicit)** — demonstrated.

</details>

<details><summary><strong>17. Starvation “fix” changing semantics ✅</strong></summary>

* **Concern:** Fixing starvation might change behaviour contract.
* **Initial assessment:** Fix must be via scheduling only.
* **Current state:** **Answered (Explicit)** — executor + groups only.

</details>

### Composition / Deployment

<details><summary><strong>18. Python component composition availability ✅</strong></summary>

* **Concern:** ros2 component appears C++-only.
* **Initial assessment:** Potential show-stopper.
* **Current state:** **Answered (Explicit)** — documented tooling gap.

</details>

<details><summary><strong>19. Risk of inventing Python component shims ✅</strong></summary>

* **Concern:** Fake parity worse than honest gap.
* **Initial assessment:** Must refuse workaround.
* **Current state:** **Answered (Explicit)** — no shims introduced.

</details>

<details><summary><strong>20. Composition meaning conflated with optimisation ✅</strong></summary>

* **Concern:** Composition framed incorrectly in ROS docs.
* **Initial assessment:** Must be treated as deployment truth.
* **Current state:** **Answered (Explicit)** — Lesson 09 framing.

</details>

<details><summary><strong>21. Shared-fate shutdown behaviour unclear ✅</strong></summary>

* **Concern:** Process kill vs lifecycle shutdown effects.
* **Initial assessment:** Needs demonstration.
* **Current state:** **Answered (Explicit)** — observed and recorded.

</details>

### Topic 10 / Deployment Verification

<details><summary><strong>22. Launch introducing new semantics ✅</strong></summary>

* **Concern:** Launch must not “fix” behaviour.
* **Initial assessment:** Strictly operational.
* **Current state:** **Answered (Explicit)** — launch only declares topology.

</details>

<details><summary><strong>23. Verification relying on node internals ✅</strong></summary>

* **Concern:** Verifier must use canonical tooling only.
* **Initial assessment:** CLI only.
* **Current state:** **Answered (Explicit)** — ros2 CLI exclusively.

</details>

<details><summary><strong>24. Cold-start vs active-state contract ambiguity ✅</strong></summary>

* **Concern:** When actions/topics should appear.
* **Initial assessment:** Needs explicit assertion.
* **Current state:** **Answered (Explicit)** — verifier enforces both.

</details>

<details><summary><strong>25. Graph convergence after shutdown ✅</strong></summary>

* **Concern:** Zombie nodes/actions possible.
* **Initial assessment:** Must be checked.
* **Current state:** **Answered (Explicit)** — convergence asserted.

</details>

### Cross-Cutting / Docs / Governance

<details><summary><strong>26. Tests missing for some topics ✅</strong></summary>

* **Concern:** Lesson 09 has no tests.
* **Initial assessment:** Might be a gap.
* **Current state:** **Answered (Explicit)** — topology topics exempt.

</details>

<details><summary><strong>27. Python README implying tutorial progression ⚠️</strong></summary>

* **Concern:** Conflicts with PHILOSOPHY.
* **Initial assessment:** Misleading tone.
* **Current state:** **Open**.

</details>

<details><summary><strong>28. README not surfacing intentional failure ⚠️</strong></summary>

* **Concern:** Users surprised by breakage.
* **Initial assessment:** Expectations mis-set.
* **Current state:** **Open**.

</details>

<details><summary><strong>29. “One systems question per section” rule invisible ⚠️</strong></summary>

* **Concern:** Structural discipline not advertised.
* **Initial assessment:** Weakens intent clarity.
* **Current state:** **Open**.

</details>

<details><summary><strong>30. Cross-lesson assumption breaking not previewed ⚠️</strong></summary>

* **Concern:** Earlier guarantees invalidated later.
* **Initial assessment:** Needs explicit signalling.
* **Current state:** **Open**.

</details>

<details><summary><strong>31. Lifecycle TransitionEvent timestamp typing ⚠️</strong></summary>

* **Concern:** Potential message type mismatch.
* **Initial assessment:** Could affect CLI observability.
* **Current state:** **Open**.

</details>

<details><summary><strong>32. QoS invalid-value handling undocumented ⚠️</strong></summary>

* **Concern:** Silent coercion may surprise users.
* **Initial assessment:** Policy must be explicit.
* **Current state:** **Open**.

</details>

<details><summary><strong>33. utils_py status not documented at top level ⚠️</strong></summary>

* **Concern:** Contract surface hidden as “utils”.
* **Initial assessment:** Needs elevation in docs.
* **Current state:** **Open**.

</details>

<details><summary><strong>34. “Lesson” terminology mismatch 🕓</strong></summary>

* **Concern:** Name no longer reflects function.
* **Initial assessment:** Structural debt, not a bug.
* **Current state:** **Deferred (Intentional)**.

</details>

<details><summary><strong>35. Need for explicit consolidation snapshot ✅</strong></summary>

* **Concern:** No single place showing what was considered vs left open.
* **Initial assessment:** Governance gap.
* **Current state:** **Answered (Explicit)** — this consolidation.

</details>

</details>

## 4. Reflections on the Review Process

Yes — there are a few **non-obvious, end-of-review insights** that only become visible *after* walking the whole arc. None of these are new “issues”; they’re **structural observations** you can use to strengthen the project without changing its intent.

----

</details>

<details><summary><strong>1. The Project Quietly Teaches *Meta-Skills*, Not ROS 2 APIs</strong></summary>
This is the biggest thing that only shows up in hindsight.

Across the arc, what’s actually being taught is:

* how to **reason about systems under pressure**,
* how to **trust observability over assumptions**,
* how to **recognise when tooling stops being honest**,
* how to **separate “works” from “behaves correctly”**.

The README and early Topics still frame the work as “learning ROS 2 features”, but the body of work is really about:

> *learning how to stop being fooled by ROS 2 systems.*

This mismatch explains several earlier tensions:

* “lesson” vs reality,
* smooth progression vs intentional failure,
* confusion when later Topics invalidate earlier guarantees.

**Insight:**
You don’t need to reframe the project now, but later, a single paragraph in the top README that says *“this is about systems thinking, not API coverage”* would collapse a lot of confusion.

</details>

<details><summary><strong>2. Documentation Quality Improves with Pressure — That’s a Pattern Worth Making Explicit</strong></summary>

* Early Topics have thinner README/THEORY docs.
* As pressure increases (Lifecycle → Actions → Executors → Composition → Deployment), the documentation becomes:

  * more precise,
  * more honest,
  * more evidence-driven.

That’s not accidental — it mirrors real systems work.

**Insight:**
This is actually a strength. It demonstrates that **documentation quality is itself an emergent property of system pressure**.

Rather than “fixing” early docs to match later tone, it may be more honest (later) to acknowledge that *documentation matures as system constraints appear*.

This supports your PHILOSOPHY rather than undermining it.

</details>

<details><summary><strong>3. utils_py Is Doing Something Rare — It Enforces Behavioural Parity Without Teaching It</strong></summary>

utils_py doesn’t just reduce duplication; it:

* freezes topic names, QoS, service contracts,
* makes cross-language parity possible later,
* silently enforces “don’t invent semantics”.

What’s interesting is that:

* you **never explain this explicitly early on**,
* yet by Topic 06+, violating it becomes impossible without obvious breakage.

**Insight:**
This is an example of **architectural pressure teaching discipline without instruction**.

When you eventually revisit docs, you might call this out as:

> “Some constraints are not taught; they are enforced.”

That’s unusual, and valuable.

</details>

<details><summary><strong>4. You Accidentally Built a Release Gate Before You Named It</strong></summary>

Topic 10 isn’t “about launch”.

Functionally, it is:

* a **release gate**,
* a **system contract verifier**,
* a **regression detector for deployment semantics**.

Only at the end does it become clear that Topics 06–09 are *preconditions* for Topic 10 to be meaningful.

**Insight:**
You’ve built the **verification pyramid backwards**:

* behaviour first,
* tooling last.

That’s the opposite of most ROS projects — and it’s why this holds together.

You don’t need to change anything now, but recognising this will help when you move to C++ or add benchmarks later.

</details>

<details><summary><strong>5. The Linear Read Didn’t Reveal This, but the Whole Arc Does:</strong></summary>

### You Never Cheat to Make a Lesson Pass

Across the entire Python track:

* you never add “lesson glue”,
* you never add fake services or helpers,
* you never patch around tooling gaps,
* you never soften failures for pedagogy.

That consistency only becomes obvious in retrospect.

**Insight:**
This is your strongest invariant. It’s worth protecting above all else when moving to C++.

If something in C++ *can’t* be shown honestly, the correct outcome (as you already proved in Python) is:

> “this stops here, and that’s the finding.”

</details>

<details><summary><strong>6. One Subtle Risk to Watch (Not a Current Issue)</strong></summary>

This is the only forward-looking caution I’ll give.

As the project matures, there’s a temptation to:

* consolidate docs,
* polish narratives,
* smooth edges.

The danger is accidentally:

* hiding how messy the evolution actually was,
* erasing the fact that understanding came *after* pressure.

**Insight:**
Preserve the messiness where it’s informative.
This review ledger itself is a good example of that.

</details>

## Final Takeaway

Nothing critical was missed by the linear traversal.

What you gained by doing the consolidation is not new defects, but **clarity about what this project really is**:

* not a tutorial,
* not a feature tour,
* not even a reference implementation,

but a **record of how systems reveal their truth under pressure**.

That’s rare. And you’ve preserved it without romanticising it.

The next main task won’t be porting code — it will be **maintaining this same level of honesty**.

</details>