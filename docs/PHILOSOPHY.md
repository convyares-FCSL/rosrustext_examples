# Philosophy — How this repo thinks (and why)

This document captures the decisions that shaped the project. Each entry is a **Question**, the agreed **Answer**, and the durable **Understanding** that should remain true even as implementations evolve.

---

<details open>
<summary><strong>1) Is this a pressure-driven progression?</strong></summary>




**Question**
Are ROS 2 primitives introduced only when the system cannot remain correct, observable, or operable without them?

**Answer**
Yes — and the same rule applies to structure, comments, abstraction, and testing: they appear only when the pressure warrants them.

**Understanding**
This repository is built around **necessity, not coverage**. The sequence is not a "curriculum" designed for students; it is a **progression of constraints** designed by system needs. A Topic is valid only if it introduces the minimum tool required by the current pressure — sometimes the pressure is **“make it exist”**, sometimes it’s **“restore operability.”**

</details>

---

<details>
<summary><strong>2) What is the primary contract being proven?</strong></summary>




**Question**
What is the spine of the repository: lifecycle/actions parity, production-grade systems across languages, or primitives composing into deployable systems?

**Answer**
**B is primary:** ROS 2 systems can be made **operable and verifiable across languages**, provided primitives are introduced in response to real pressure.
A and C exist as supporting artifacts and evidence.

**Understanding**
Lifecycle, actions, composition, and testing are not "the point." They are **vehicles** that appear when the system forces them. Cross-language parity turns this repository into **engineering evidence**: the same operational behaviours can be achieved across languages, and differences become visible costs rather than hidden excuses.

</details>

---

<details>
<summary><strong>3) What counts as “failure” here?</strong></summary>




**Question**
Do we define failure as operational breakdown rather than code bugs?

**Answer**
Yes. Failure is not “you did something wrong.” It is “your current toolset no longer supports your operational goal.”

**Understanding**
Failure means **loss of operability**: loss of responsiveness, loss of observability, inability to orchestrate, inability to reason about shutdown, inability to contain faults. A node can be correct and still be **unfit to deploy**. Topics exist to expose these failure modes and introduce the minimum architectural response that restores control.

</details>

---

<details>
<summary><strong>4) Is language parity a teaching aid or a constraint?</strong></summary>




**Question**
Is parity used mainly to compare languages, or as a constraint that shapes architecture?

**Answer**
Parity is a **constraint**, not a comparison exercise.

**Understanding**
If a pattern cannot be expressed across languages, it is suspect. If a language forces deviation, that deviation is documented as an **architectural cost**. The `roslibrust` lane is not second-class; it is a **stress test of the specification**: if the contract relies on C++ implementation details rather than the wire protocol, the repo treats that as a failure.

</details>

---

<details>
<summary><strong>5) Is testing a topic or a consequence?</strong></summary>




**Question**
Do tests appear only when complexity makes evidence necessary?

**Answer**
Yes. Testing is introduced only when the problem warrants it — not as a doctrine.

**Understanding**
Evidence scales with pressure. Unit tests appear when logic becomes separable. Integration tests appear when orchestration becomes system-critical. Deployment verification scripts appear when topology and bring-up become the dominant risk. This repo rejects ritual: tests and abstractions are tools that must **pay rent**.

</details>

---

<details>
<summary><strong>6) Who is this for?</strong></summary>




**Question**
Is the target audience engineers focused on operability rather than onboarding?

**Answer**
Yes. It is not for new programmers and not for first-time ROS users.

**Understanding**
The reader is expected to either already know ROS basics or be an experienced engineer who can absorb ROS concepts through systems framing. The value is not syntax acquisition; it is a distilled account of what professional systems demand as they scale.

</details>

---

<details>
<summary><strong>7) Should this be opinionated?</strong></summary>




**Question**
Is this repo trying to prove a point (“Rust is better”, “actions are best”, “always unit test”), or something else?

**Answer**
It should be **authoritative**, not opinionated.

**Understanding**
Opinionated says “do this.” Authoritative says “this is what happens.” The repository earns authority by constructing controlled scenarios where operational pressures force constraints into view. Language choice becomes a role decision rather than ideology. Primitives appear as responses rather than preferences.

</details>

---

<details>
<summary><strong>8) Why “Topics”, not “Lessons”?</strong></summary>




**Question**
Why is the repo organised into Topics rather than Lessons?

**Answer**
Because the sequence is not pedagogy-first; it is systems-pressure-first.

**Understanding**
“Lesson” implies instruction and correctness-by-following. “Topic” implies a real systems concern you engage with because deployment reality forced it. Docs should use verbs like **establish, expose, validate, constrain** — not *teach, learn, introduce*.

</details>

---

<details>
<summary><strong>9) Why avoid safety-critical/hydrogen framing in INTENT-level docs?</strong></summary>




**Question**
Why not frame the intent around a specific domain?

**Answer**
Because intent should be timeless and broadly applicable; authority comes from discipline and evidence, not domain signalling.

**Understanding**
Domain context can belong in README/benchmarks as credibility, but intent-level documents must not overclaim. The repo shows how fault-aware operability emerges under pressure; it does not claim a complete certified platform.

</details>

---

<details>
<summary><strong>10) Should we explicitly list “what ROS is missing”?</strong></summary>




**Question**
Do we call out ecosystem gaps directly in intent docs?

**Answer**
No. Gaps should emerge naturally as consequences of the Topics.

**Understanding**
Stating “ROS is missing X” turns the repo into advocacy. Instead, Topics act like controlled experiments: when parity is missing, system behaviour and tooling mismatch make it visible without editorialising.

</details>

---

<details>
<summary><strong>11) How should early Topics be framed?</strong></summary>




**Question**
Are early Topics (00–03) “break/fix” like later Topics?

**Answer**
No. Early Topics are driven by **existential pressure**: making the system exist and establishing invariants.

**Understanding**
Early Topics reflect how engineers actually start: get something running, publish data, verify the graph. They are not “foundational theory”; they are engineering instincts made explicit. Operational failure modes become the driver later when scale forces them.

</details>

---

<details>
<summary><strong>12) What’s the tone: instruction or distilled experience?</strong></summary>




**Question**
Is this repo telling engineers “how to code”, or something else?

**Answer**
It is a distilled account of what happens to real systems as they scale, and what restores control.

**Understanding**
Writing should avoid prescriptions and best-practice sermons. It should read like: **“This is what happened; this is what changed; this is what became observable.”** Authority comes from repeatability and tooling parity, not opinion.

</details>

---

<details>
<summary><strong>13) Should the docs include a refusal section?</strong></summary>




**Question**
Do we explicitly say what this repo refuses to be?

**Answer**
Yes.

**Understanding**
The repo actively refuses to be a syntax tutorial, a feature checklist, or a language war. It rejects any code that does not solve a stated operational pressure. This protects scope and keeps the project coherent as it gains visibility.

</details>

---

<details>
<summary><strong>14) Should the intent be timeless?</strong></summary>




**Question**
Should the project intent be tied to the current set of Topics?

**Answer**
No — it should be timeless.

**Understanding**
The intent describes an approach and an arc, not a frozen table-of-contents. It should remain correct if new Topics are added, languages change, or implementations evolve.

</details>

---

<details open>
<summary><strong>15) How do we signal “professional” without overclaiming?</strong></summary>




**Question**
How do we signal engineering rigour without claiming too much?

**Answer**
By describing operational properties (operable, observable, verifiable, fault-aware) rather than claiming completeness or certification-grade outcomes.

**Understanding**
The repo demonstrates how disciplined systems evolve under pressure. It does not claim a final, universal architecture. It aims to produce **deployability evidence**, not a grand promise.