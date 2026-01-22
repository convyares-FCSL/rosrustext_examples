## Lesson 07 Intent: Actions (Long-Running Work Under Lifecycle)

**Lesson 07 introduces ROS 2 Actions as the standard mechanism for long-running work with feedback and cancellation, and shows why “functional correctness” is not the same as “operational availability.”**

In Lessons 00–06 we built deterministic, managed nodes:

* configuration is explicit (parameters + YAML),
* startup is orchestrated (lifecycle),
* telemetry is gated (Active state only).

Lesson 07 adds a second axis: **work that takes time**.

### Goal

Extend the existing **Lifecycle Publisher** node to also provide an **Action Server** that computes a Fibonacci sequence (intentionally long-running). Add a standalone **Action Client** as a demo tool to drive requests and observe feedback/cancellation.

### What this lesson must demonstrate

1. **Action Server and Client exist as first-class examples**

   * server exposes `lesson_interfaces/action/Fibonacci`
   * client sends a goal, receives feedback, handles cancellation, prints result

2. **Lifecycle still owns “when the system is allowed to operate”**

   * the publisher node remains lifecycle-managed (Unconfigured → Inactive → Active)
   * telemetry only publishes while Active

3. **Actions introduce “work can starve the node” (intentional)**

   * the first implementation may compute Fibonacci in a way that can block the main execution path
   * the consequence is visible (telemetry pauses / lifecycle services get sluggish)
   * this sets up Lesson 08: executors and callback groups to restore availability

4. **Professional structure**

   * node wiring is separate from business logic:

     * `node` (wiring + lifecycle)
     * `publisher` (telemetry publishing component)
     * `action_server` (fibonacci action server)
     * `action_client` (demo client tool)

### Produced artifacts

* **Managed Lifecycle Publisher + Action Server** (single node, two responsibilities)
* **Action Client demo tool** (clean reference implementation)
* **Verification steps** (CLI-based, no launch_testing required here)

### Relationship to later lessons

* Lesson 07 intentionally shows a failure mode: **long-running work can break responsiveness**
* Lesson 08 fixes it properly: **executor strategy + callback group isolation**
* Lesson 09+ focuses on deployment/topology, not algorithm changes

---
