### Short answer

Yes to both ideas — but **separate their roles**.

* **Capstone** should be a *systems-engineering problem*, not a robot.
* **Benchmarking** should be an *optional, clearly framed study*, not a core lesson.

That separation keeps the lesson track clean while still giving engineers the deep, comparative insight they actually care about.

---

## What the capstone *should* be (engineer-first, not robot-first)

Most ROS capstones are:

* motors
* cameras
* SLAM
* nav stacks

That’s fine for robotics courses, but your audience is closer to **systems, controls, infrastructure, and integration engineers**. You already lean that way with config discipline, lifecycle, and multi-language parity.

### Recommended Capstone Theme

**“Distributed Control & Telemetry System”**

Think *industrial system*, not robot.

### Concrete Capstone Concept

**Multi-language Distributed Plant Monitor & Command System**

Example narrative (no robotics baggage):

* Multiple data-producing nodes (simulated sensors)
* Multiple consumers
* Clear separation of:

  * telemetry
  * commands
  * state
  * events
* Central bringup + lifecycle coordination
* Cross-language interoperability

### Why this works

* Feels *real* to engineers
* Avoids robotics-specific APIs
* Naturally exercises:

  * QoS
  * lifecycle
  * parameters
  * launch
  * error handling
  * observability

---

## Suggested Capstone Architecture

### Nodes (example)

* **C++**
  High-rate telemetry publisher (deterministic, lifecycle-managed)
* **rclrs (Rust)**
  Processing / aggregation node (explicit executor, strong typing)
* **Python**
  Supervisor + configuration manager (dynamic, introspective)
* **rcllibrust (optional)**
  External client / dashboard / bridge (non-ROS-native)

### System behaviors

* Bringup via launch files
* Lifecycle manager transitions nodes
* Shared config via installed YAML
* Clean shutdown
* Namespacing + remaps
* Failure modes demonstrated (node dies, lifecycle state changes)

This feels like:

* industrial automation
* test infrastructure
* distributed control
* cloud-edge robotics

…without needing wheels or cameras.

