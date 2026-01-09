# Lesson 01 Breakdown: The Async Task Pattern

## Architecture: Tasks vs. Executors

In `rclcpp` or `rclrs`, a timer is a construct managed by a **Single-Threaded Executor**. The executor checks the clock in a loop and calls your callback function when time expires. The callback runs on the main thread (blocking other work if it's slow).

In `rcllibrust` (and async Rust), we invert this model. We **Spawn a Task**.
This task is an independent "green thread" managed by the **Tokio Runtime**. It runs concurrently with other tasks (like the rosbridge network client).



---

## Code Walkthrough

### 1. The Async Timer (`tokio::spawn`)

Instead of `node.create_timer(...)`, we create a loop in a background task:

```rust
tokio::spawn(async move {
    // A: Create an interval stream (efficient sleep)
    let mut interval = tokio::time::interval(Duration::from_secs_f64(period_s));
    
    loop {
        // B: Wait (Yield execution)
        // This line pauses THIS specific task, allowing the CPU to do other work
        // (like handling network packets) until the time has passed.
        interval.tick().await;

        // C: The Logic
        on_tick(&tick_cb);
    }
})

```

**Why `spawn`?**
If we ran this infinite loop directly in `main`, the program would block forever. It would never reach the line that listens for `Ctrl+C`. Spawning offloads the loop to the background.

### 2. Shared State (`Arc<AtomicU64>`)

In `rclrs`, we used a `struct NodeState` wrapped in a `Mutex`.
Here, because the timer runs in a separate async task, we need a thread-safe way to manage the counter.

* **`Arc` (Atomic Reference Counted)**: Allows the data to be owned by multiple places at once (e.g., if we wanted `main` to read the counter while the timer writes to it).
* **`AtomicU64`**: A specialized integer type that allows modification (`fetch_add`) without the heavy overhead of a `Mutex` lock.

### 3. RAII Life Support (`Lesson01Node`)

```rust
struct Lesson01Node {
    _client_handle: ClientHandle,
    _timer_handle: tokio::task::JoinHandle<()>,
}

```

This struct is the **"Life Support System"** for the application.

1. **`_client_handle`**: Represents the WebSocket connection. If this variable is dropped, `roslibrust` automatically closes the connection.
2. **`_timer_handle`**: Represents the background timer task. Holding this handle allows us to (optionally) abort the timer or check if it panicked.

### 4. Clean Shutdown (`tokio::signal`)

```rust
match tokio::signal::ctrl_c().await { ... }

```

Unlike `rclcpp::spin()`, which captures the thread until shutdown, async `main` functions usually just "park" at the end.

* This line puts the main thread to sleep.
* It wakes up **only** when the OS sends a SIGINT (Ctrl+C).
* Once it wakes, `main` returns, `Lesson01Node` is dropped, and resources are cleaned up.