# Cross-Track Parity Review Ledger (Python ↔ C++)

This ledger records **parity observations as facts**, not prescriptions.
It exists to prevent re-litigating decisions and to track work that can be done later without blocking current sequencing.

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

### 0.2 What this ledger is not

* Not a refactor plan
* Not a style guide
* Not a “make C++ match Python line-by-line” document

---

## 1. Release-Blocking Concerns (must not be ambiguous)

> Answers one question only:
> **What cannot be left ambiguous at point of release?**
> Items here need either: a fix, explicit docs, or explicit deferral.

### 1.1 Contract surfaces & names

<details>
<summary><strong>✅ PAR-01 — Canonical action name for Fibonacci</strong></summary>

**Observation**
Python uses `topics.fibonacci -> "/tutorial/fibonacci"` by default.
C++ must match; node-name-derived defaults break Topic 08+ reuse.

**Status**
✅ Answered (Explicit) — agreed to keep `DEFAULT_FIBONACCI="/tutorial/fibonacci"` in `utils_cpp/topics.hpp`.

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

> action & lifecycle usage is correct, availability is not ensured under single-threaded scheduling.

**Status**
🟡 Answered (Implicit)

**Closure evidence**
One sentence in both tracks making that equivalence explicit (optional).

</details>

---

## 2. Non-Blocking Differences (allowed, but must be recorded)

### 2.1 Structure / file layout

<details>
<summary><strong>🟡 STR-01 — Different file breakdown across languages</strong></summary>

**Observation**
Python splits components across multiple small files (publisher, node, action_server).
C++ uses .hpp/.cpp boundaries and tends to group by compile targets.

Difference is acceptable if contract surfaces remain stable and the split doesn’t introduce hidden semantics.

**Status**
🟡 Answered (Implicit)

**Closure evidence**
Document “structure differs, behaviour must not” in each track’s README.

</details>

<details>
<summary><strong>🕓 STR-02 — “Lesson” terminology mismatch</strong></summary>

**Observation**
“Lesson” is historical terminology; system behaves like an operability reference.

**Status**
🕓 Deferred (Intentional)

**Closure evidence**
Tracked only as structural debt (TODO), no rename required now.

</details>

---

## 3. Per-Lesson Parity Ledger

### 3.1 Lesson 07 — Actions under Lifecycle

<details>
<summary><strong>✅ L07-01 — Lifecycle gating of action acceptance</strong></summary>

**Observation**
Goals rejected when Inactive; accepted when Active.

**Status**
✅ Answered (Explicit)

**Closure evidence**
Attempt goal send while inactive; server rejects. After activate; accepts.

</details>

<details>
<summary><strong>✅ L07-02 — Feedback shape is stable and observable</strong></summary>

**Observation**
Feedback publishes partial sequences; result matches Fibonacci sequence for order=5.

**Status**
✅ Answered (Explicit)

**Closure evidence**
Integration test asserts result `[0,1,1,2,3]` and feedback count >= 1.

</details>

<details>
<summary><strong>✅ L07-03 — Cancellation is cooperative and delayed under load</strong></summary>

**Observation**
Cancel accepted; final status CANCELED; partial sequence returned; cancellation latency is visible.

**Status**
✅ Answered (Explicit)

**Closure evidence**
Integration test: status==5, sequence length in (1..order-1).

</details>

<details>
<summary><strong>✅ L07-04 — Starvation probe is externally visible</strong></summary>

**Observation**
While long action runs under single-threaded executor:

* telemetry pauses
* lifecycle service responsiveness degrades

**Status**
✅ Answered (Explicit)

**Closure evidence**
Integration test: lifecycle get_state future does not complete within short timeout during long action.

</details>

<details>
<summary><strong>⚠️ L07-05 — Business logic boundary: “long routine outside ROS code” consistency</strong></summary>

**Observation**
Python keeps generator logic outside ROS.
C++ moved Fibonacci routine into `logic.hpp` (good), but ensure action server remains only orchestration and does not re-embed algorithmic steps.

**Status**
⚠️ Open (tracking)

**Closure evidence**
Action server execute path calls into a routine API (e.g. `routine_.run(order, on_progress)`), not inline Fibonacci steps.

</details>

---

## 4. Follow-Up Actions Queue (non-sequencing, can be done later)

> This section answers:
> **What can be worked on later without breaking topic order?**
> Keep it small and specific.

<details>
<summary><strong>4.1 Documentation alignment</strong></summary>

* Align Lesson 07 Theory phrasing across tracks:

  * clarify “correct API usage but insufficient scheduling”
* Ensure Lesson 07 README (C++) mentions reusing Lesson 06 subscriber if desired (optional), or explains why not needed.

</details>

<details>
<summary><strong>4.2 Contract verification</strong></summary>

* Add explicit verification snippets (CLI) for:

  * `/tutorial/fibonacci` presence only after configure/activate (depending on contract)
  * expected feedback/result in `ros2 action send_goal`

</details>

---

## 5. Notes / Decisions (append-only)

* 2026-01-25 — Confirmed `topics.fibonacci` default must be `/tutorial/fibonacci` (not `/<node>/fibonacci`) to support Topics 08+ reuse.
* 2026-01-25 — Lesson 07 framing: do not “teach by wrongness”; demonstrate correct code that still fails operationally under naïve scheduling.
