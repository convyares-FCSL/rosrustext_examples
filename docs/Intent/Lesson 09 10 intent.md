### Overview

Your instinct is right: **leaving Lesson 09 “broken” without a sanctioned recovery path feels wrong** in *this* curriculum — because unlike Lesson 07, the **vehicle of the lesson (composition)** is *also* the thing that would normally be used to fix the failure.

But the key distinction is this:

* **Lesson 07 failure** is *intra-node* and is fixed by **introducing a new primitive** (executors + callback groups).
* **Lesson 09 failure** is *inter-node / deployment-level* and **cannot be fixed by “using composition better”** without introducing *another* primitive.

So you’re not missing a fix — you’re missing the **next axis of control**.

That axis is **process boundaries**.

---

### The clean architectural resolution (no extra lesson numbers)

Do **not** split Lesson 09 into A/B.
Do **not** add a Lesson 11.
Do **not** “fix” Lesson 09 inside Lesson 09.

Instead:

> **Lesson 09 is the last lesson that is allowed to end in a *deployment-induced failure*.**
> **Lesson 10 is where that failure becomes manageable — but not by composition.**

Why this works:

* Lesson 09’s vehicle is **composition**
* Lesson 10’s vehicle is **launch + deployment topology**
* The fix is **not better composition**, it is **selective composition + selective isolation**

That preserves conceptual hygiene.

---

### Reframe Lesson 09’s “brokenness” precisely

Lesson 09 is not “broken”.

Lesson 09 proves this invariant:

> **Composition is not an isolation mechanism.**

That is the *entire* lesson.

The student must leave knowing:

* Callback groups ≠ node isolation
* Executors ≠ fault domains
* Composition ≠ safety
* Containers ≠ sandboxes

This is not something you “fix” with another ROS primitive.

---

### Where the “solution” actually lives (and why it belongs in Lesson 10)

The *solution* is:

* **Topology choice**
* **Process boundaries**
* **Selective composition**

Which are all **deployment orchestration concerns**, not node concerns.

That maps *perfectly* to Lesson 10:

> **Lesson 10 – Launch & Configuration Discovery**
> *Production-grade startup is about choosing where failures are allowed to couple.*

Lesson 10 doesn’t change node code.
Lesson 10 doesn’t change container code.
Lesson 10 changes **what runs together**.

That is the correct symmetry with Lesson 08:

| Lesson | Failure Source            | Fix Vehicle                 |
| ------ | ------------------------- | --------------------------- |
| 07     | Work monopolizes executor | Executors + callback groups |
| 08     | —                         | (fix applied)               |
| 09     | Nodes share fate          | Deployment topology         |
| 10     | —                         | (fix applied)               |

---

### What Lesson 10 “fixes” (without rewriting history)

Lesson 10 demonstrates:

* One container for:

  * telemetry publisher
  * verifier
* Separate container (or process) for:

  * action server
* Lifecycle orchestration across **multiple containers**
* Launch-driven verification that:

  * failures are contained
  * shutdown order is controlled
  * restart scope is explicit

This doesn’t contradict Lesson 09.

It **answers it**.

---

### Why not split Lesson 09 into A/B?

Because that would:

* blur the boundary between **concept exposure** and **operational resolution**
* teach composition as both *problem* and *solution*
* weaken the mental model that *deployment choices are first-class design decisions*

Lesson 07/08 works because the fix introduces a **new primitive**.
Lesson 09 has no such primitive left.

Only **architecture** remains.

---

### Final architectural guidance (strong recommendation)

* **Lesson 09 ends with an acknowledged, explained deployment failure**
* **Lesson 10 resolves that failure using launch topology**
* No new lesson numbers
* No “composition v2”
* No silent fixes

This preserves:

* sequencing discipline
* parity across languages
* the thesis that *operability is architectural, not algorithmic*

---

### One next step

Confirm this framing:

> **Lesson 09 intentionally ends with a deployment-induced failure that has no in-lesson fix. Lesson 10 resolves it via selective composition and process boundaries using launch.**

Once confirmed, the next step is to **tighten Lesson 09 acceptance criteria** so the failure is *precise, reproducible, and explainable* — not vague or accidental.
