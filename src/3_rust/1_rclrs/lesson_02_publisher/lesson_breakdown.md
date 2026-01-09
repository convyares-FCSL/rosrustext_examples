# Lesson 02 Breakdown: Components & Shared Interfaces

## Architecture: Component-Based Design

As nodes grow, keeping all variables (publishers, subscribers, counters, flags) in one struct becomes unmanageable, in Lesson 01, we passed `count` manually into the timer.

**Lesson 02 introduces Composition.**



We separate the code into two distinct layers:
1.  **The Logic Component (`PublisherChatter`)**: Handles the "Business Logic" (what the node *does*). It owns the Publisher and the State (`count`).
2.  **The Node Container (`Lesson02Node`)**: Handles the "Lifecycle" (startup, shutdown, keeping handles alive). It owns the Component.

## New Concept: External Messages

In Lesson 01, we didn't publish anything. In this lesson, we publish a custom message.

```rust
use lesson_interfaces::msg::MsgCount;

```

* **The Source**: This struct (`MsgCount`) is NOT defined in our Rust code manually. It is defined in a ROS 2 interface file (`MsgCount.msg`) in the `lesson_interfaces` package.
* **The Generation**: When we build, `rosidl_generator_rs` reads that file and generates a Rust struct with the necessary traits (Serialize, Deserialize) to communicate over the DDS middleware.
* **The Import**: We import it like a standard crate, allowing us to share data structures seamlessly between C++, Python, and Rust nodes.

## New Concept: Shared Library (`utils_rust`)

Hardcoding topic names (e.g., `"chatter"`) and QoS settings inside the node code is a bad practice. If you change the topic name in the publisher, you must remember to change it in the subscriber.

We use a local crate `utils_rust` to centralize this configuration.

```rust
use utils_rust::{qos, topics};

```

1. **Topic Consistency**: `topics::chatter(node)` returns the canonical topic name. Both the Publisher node and (in future lessons) the Subscriber node call this same function.
2. **QoS Configuration**: `qos::from_parameters(node)` allows us to configure Quality of Service (Reliability, Durability) via command-line parameters without recompiling the code.

## Code Walkthrough

### 1. The Logic Component

This struct is the "Brain" of the operation. It is self-contained.

```rust
struct PublisherChatter {
    publisher: Publisher<MsgCount>, // Typed to our custom message
    count: AtomicU64,
    _logger: Logger, 
}

```

### 2. The Logic Implementation

Notice how clean the logic becomes. It operates on `self` and uses the generated message struct.

```rust
impl PublisherChatter {
    // Pure logic function.
    fn on_tick(&self) -> Result<(), RclrsError> {
        // Increment count.
        let n = self.count.fetch_add(1, Ordering::Relaxed) + 1;

        // Build message. 
        // We use the struct generated from the .msg file.
        let msg = MsgCount { count: n as i64 };

        // Publish message.
        self.publisher.publish(&msg)?;

        Ok(())
    }
}

```

### 3. The Resource Container

The container now holds the **Component** instead of raw data.

```rust
struct Lesson02Node {
    pub node: Node,
    // The underscore prefix indicates these are held for RAII (keep-alive) purposes.
    _publisher_component: Arc<PublisherChatter>,
    _timer: Arc<rclrs::Timer>, 
}

```

### 4. Dependency Injection (The Constructor)

This is where we wire the parts together.

```rust
pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
    // ... create node and params ...

    // Step A: Instantiate the Component
    // We build the "Brain" first.
    let publisher = Arc::new(Self::build_chatter_publisher(&node)?);
    
    // Step B: Inject into Timer
    // We clone the Arc (cheap pointer copy) and pass it to the timer builder.
    // The timer now has access to the component's logic (`on_tick`).
    let timer = Self::build_timer(&node, Arc::clone(&publisher), period_param)?;

    log_info!(node.logger(), "Lesson 02 node started...");

    Ok(Self {node, _publisher_component: publisher, _timer: timer})
}

```

### 5. The Builder Pattern (Configuring Resources)

We use a helper to keep the `new` function clean. Note the use of `utils_rust`.

```rust
fn build_chatter_publisher(node: &Node) -> Result<PublisherChatter, RclrsError> {
    // ... logger setup ...

    // Best Practice: Don't hardcode strings!
    // We get the topic name from our shared utility library.
    let topic_name = topics::chatter(node);

    // Best Practice: Don't hardcode QoS!
    // We load QoS from parameters (or defaults) via utility helpers.
    // This allows us to change reliability at runtime.
    let qos_profile = qos::from_parameters(node);

    // Options.
    let mut options = rclrs::PublisherOptions::new(topic_name.as_str());
    options.qos = qos_profile;

    // ... create and return ...
}

```

**Why do we do this?**

* **Single Source of Truth**: The `.msg` file is the definition of truth for data. The `utils_rust` library is the definition of truth for configuration.
* **Scalability**: If you change the topic name in `utils_rust`, the entire system updates automatically.
* **Interoperability**: Using standard ROS 2 interfaces ensures this Rust node can talk to a Python subscriber without any extra work.
