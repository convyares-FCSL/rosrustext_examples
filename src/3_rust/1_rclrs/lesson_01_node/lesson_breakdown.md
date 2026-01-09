# Lesson 01 Breakdown: The Event Loop & State

## Architecture: Thread-Safe State

This lesson extends the **Container Pattern** from Lesson 00 by adding two critical features: **State** (a counter) and **Logic** (a timer callback).

In ROS 2, the Executor "spins" to check for events. While our basic executor is single-threaded, ROS 2 is designed for concurrency. Rust forces us to design our state to be thread-safe from day one.



We achieve this using **Atomic Types** wrapped in **Arcs** (Atomically Reference Counted pointers). This allows the Timer Callback and the Node Container to share ownership of the counter without complex locking (Mutexes).

## Code Walkthrough

### 1. The Resource Container (RAII)
```rust
struct Lesson01Node {
    pub node: Node,

    // We must hold this handle. If this field is dropped, the timer stops.
    // Note: rclrs::Timer is internally an Arc, so we hold it directly.
    _timer: rclrs::Timer, 
}

```

* **The `_` Prefix**: We name the timer field `_timer` (starting with an underscore). This signals to the compiler and other developers: *"I am holding this variable solely to keep it alive."* We don't need to read the timer variable later; we just need to ensure it isn't dropped (garbage collected), which would stop the callback from firing.

### 2. The Constructor: State & Parameters

```rust
pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
    // ... create node ...

    // Step A: Declare Parameter
    // We declare "timer_period_s" with a default of 1.0. 
    // .mandatory() ensures we get a valid handle to read the value later.
    let period_param = node.declare_parameter("timer_period_s").default(1.0).mandatory()?;

    // Step B: Define Shared State
    // - AtomicU64: An integer safe to increment from any thread.
    // - Arc<...>: A "Smart Pointer" that lets multiple owners hold the same data.
    let count = Arc::new(AtomicU64::new(0));

    // Step C: Build the Timer
    // We pass the Arc (count) into the helper. This increments the reference count.
    let timer = Self::build_timer(&node, count, period_param)?;

    Ok(Self { node, _timer: timer })
}

```

### 3. The Timer Builder (Deep Dive)

This helper function constructs the logic. This separates *configuring* the resource from *using* it.

```rust
fn build_timer(
    node: &Node, 
    count: Arc<AtomicU64>, // Takes ownership of one Arc clone
    period_param: MandatoryParameter<f64>
) -> Result<rclrs::Timer, RclrsError> {
    
    // Step A: Read Parameter
    let mut period_s = period_param.get();
    if !period_s.is_finite() || period_s <= 0.0 {
        // ... validation logic ...
        period_s = 1.0;
    }

    // Step B: Capture Context
    // We need the node inside the callback for logging.
    // We clone it here so the callback owns its own reference to the node.
    let node_clone = node.clone();
    
    // Step C: Create the Callback Closure
    // 'move' moves the variables (node_clone and count) INTO the closure.
    node.create_timer_repeating(
        Duration::from_secs_f64(period_s), 
        move || { 
            // This is the actual code that runs every second.
            Self::on_tick(&node_clone, &count); 
        } 
    )
}

```

### 4. Pure Logic Function

```rust
fn on_tick(node: &Node, count: &AtomicU64) {
    // Increment: Atomic fetch_add is a safe operation.
    // Ordering::Relaxed is sufficient for simple counters.
    let n = count.fetch_add(1, Ordering::Relaxed) + 1;

    log_info!(node.logger(), "tick {}", n);
}

```

**Why do we do this?**

* **Separation of Concerns**: The `build_timer` function handles "Setup" (parsing params, validation), while `on_tick` handles "Business Logic" (counting).
* **Testability**: You can call `on_tick` in a unit test without needing to create a real ROS 2 timer or executor.
* **Safety**: Using `Arc` and `Atomic` ensures that even if we later switch to a Multi-Threaded Executor, this code will typically remain safe without modification.
