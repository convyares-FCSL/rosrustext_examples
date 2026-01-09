# Lesson 02 Breakdown: Components, State Boundaries, and Shared Interfaces (rclrs)

## Architectural Intent

Lesson 02 introduces the first real scaling constraint:

> A single “node struct” that owns every publisher, subscriber, timer, counter, and flag turns into an unmaintainable state blob.

The goal of this lesson is not “how to publish” — it’s establishing a **component boundary** so state stays local to the feature that owns it.

---

## The Component Rule

From Lesson 02 onward, the pattern is:

* **Node container** owns lifecycle and keep-alive handles.
* **Each ROS capability** (publisher / subscriber / service / action / timer-driven worker) lives in its own struct with:

  * the ROS handle(s) it needs
  * the state it owns
  * the methods that operate on that state

In Lesson 02, that “capability” is the chatter publisher.

---

## Why the Publisher Lives in `PublisherChatter` (Not the Node)

Your code puts the publisher into a dedicated struct for a concrete scaling reason:

### 1) State stays where it belongs

The counter is not “node state”. It is **publisher state**:

```rust
struct PublisherChatter {
    publisher: Publisher<MsgCount>,
    count: AtomicU64,
    _logger: Logger,
}
```

If later you add:

* a second publisher
* a subscriber
* a service
* an action server

each gets *its own* state struct, rather than growing a single `Lesson02Node` into a god-object.

### 2) The callback owns a stable dependency

The timer closure needs a stable target to call:

```rust
move || { chatter.on_tick().expect("Timer callback failed"); }
```

By putting publisher + state in one struct and wrapping it in `Arc`, the callback captures **one thing** that fully represents that capability.

### 3) The node stays wiring-only

`Lesson02Node::new()` reads like a composition root:

```rust
let publisher = Arc::new(Self::build_chatter_publisher(&node)?);
let timer = Self::build_timer(&node, Arc::clone(&publisher), period_param)?;
```

The node is responsible for:

* creating resources
* connecting them
* keeping them alive

It does not own the logic state directly.

---

## Shared Interfaces: `MsgCount` is the Contract

```rust
use lesson_interfaces::msg::MsgCount;
```

`MsgCount` is generated from a `.msg` definition and shared across languages. The practical outcome is:

* Rust publishes a message that Python/C++ subscribers can decode without translation glue.
* The system contract lives in `src/4_interfaces/msg`, not in any language’s code.

---

## Shared Configuration: Topics + QoS are Not Embedded in Nodes

```rust
use utils_rclrs::{qos, topics};
```

Lesson 02 deliberately avoids “string + QoS sprinkled in code”. Instead:

* `topics::chatter(node)` returns the canonical name
* `qos::from_parameters(node)` returns the canonical QoS profile

This makes compatibility a configuration concern, which becomes critical in Lesson 03.

---

## Setup vs Runtime Responsibilities

Lesson 02 keeps runtime logic narrow and local.

### Runtime: `on_tick()` owns only publish logic

```rust
fn on_tick(&self) -> Result<(), RclrsError> {
    let n = self.count.fetch_add(1, Ordering::Relaxed) + 1;
    let msg = MsgCount { count: n as i64 };
    self.publisher.publish(&msg)?;
    Ok(())
}
```

No config lookup. No ROS resource creation. No lifecycle work.

### Setup: `Lesson02Node` wires components and holds handles

```rust
struct Lesson02Node {
    pub node: Node,
    _publisher_component: Arc<PublisherChatter>,
    _timer: rclrs::Timer,
}
```

The underscore fields exist because dropping them would stop the system (RAII keep-alive).

---

## Why This Lesson Matters

Lesson 02 establishes the component boundary that later lessons depend on:

* Lesson 03 will add a subscriber component without inflating the node struct.
* Lesson 04+ will add services/actions using the same pattern.
* Growth happens by adding components, not by expanding a single state object.

This is the point where the track shifts from “examples” to “maintainable systems”.

---

## One next step only

If this breakdown matches your intent, I’ll write the **Lesson 03 rclrs breakdown** using the *same component rule* (SubscriberChatter + Node container + reset tolerance + QoS contract) once your Lesson 03 code is finalized.
